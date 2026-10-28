"""NEW numerical interface examples for a planar car.

This module was not in the original interpolation program. It translates vehicle
pose into road coordinates and wheel positions into modeled environmental values.
It does NOT integrate dynamics, calculate tire forces, solve MPC, create native
Open-Car-Dynamics objects, or provide a CasADi-symbolic implementation.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Any
import numpy as np
from .data import FloatArray
from .geometry import wrap_angle
from .road import PeriodicTrack
from .surface import SurfaceModel

WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")


@dataclass(frozen=True)
class PlanarState:
    """Global pose and BODY-frame velocity; yaw rate is positive counterclockwise."""
    x_m: float
    y_m: float
    yaw_rad: float
    vx_mps: float
    vy_mps: float
    yaw_rate_radps: float

    def __post_init__(self) -> None:
        if not np.isfinite([self.x_m, self.y_m, self.yaw_rad,
                            self.vx_mps, self.vy_mps, self.yaw_rate_radps]).all():
            raise ValueError("All planar-state values must be finite.")

    @property
    def xy(self) -> FloatArray:
        return np.array([self.x_m, self.y_m])


@dataclass(frozen=True)
class WheelGeometry:
    """Contact-point offsets in body axes: x forward, y left.

    Track widths may be zero to represent a single-track planar reduction. This
    positioning model ignores heave, roll, pitch and suspension articulation.
    """
    lf_m: float = 1.724
    lr_m: float = 1.476
    front_track_m: float = 1.6
    rear_track_m: float = 1.6

    def __post_init__(self) -> None:
        values = [self.lf_m, self.lr_m, self.front_track_m, self.rear_track_m]
        if (not np.isfinite(values).all() or self.lf_m <= 0 or self.lr_m <= 0
                or self.front_track_m < 0 or self.rear_track_m < 0):
            raise ValueError("Axle distances must be positive and track widths nonnegative.")

    def body_positions(self) -> FloatArray:
        return np.array([
            [self.lf_m, self.front_track_m/2],
            [self.lf_m, -self.front_track_m/2],
            [-self.lr_m, self.rear_track_m/2],
            [-self.lr_m, -self.rear_track_m/2],
        ])


class RoadCarAdapter:
    """Observation and environmental mapping, NOT a feedback controller."""
    def __init__(self, road: PeriodicTrack, wheels: WheelGeometry | None = None,
                 surface: SurfaceModel | None = None):
        self.road = road
        self.wheels = WheelGeometry() if wheels is None else wheels
        self.surface = road.surface if surface is None else surface

    def observe(self, state: PlanarState, hint_s: float | None = None,
                window_m: float = 60.0) -> dict[str, Any]:
        """Local progress, lateral offset, heading error and CENTER-POINT margins.

        Positive e is to the left of the reference. Positive margins here do not
        certify that a finite car footprint is inside the physical road.
        """
        projection = self.road.project(state.xy, hint_s=hint_s, window_m=window_m)
        q = self.road.sample_geometry(projection["s_m"])
        e = projection["e_left_m"]
        epsi = float(wrap_angle(state.yaw_rad - float(q["heading_rad"])))
        return {
            "s_unwrapped_m": projection["s_m"],
            "e_left_m": e,
            "heading_error_rad": epsi,
            "projection_distance_m": projection["distance_m"],
            "point_margin_left_m": float(q["width_left_m"]) - e,
            "point_margin_right_m": float(q["width_right_m"]) + e,
            "road": q,
        }

    def frenet_rates(self, state: PlanarState, observation: dict[str, Any],
                     min_coordinate_jacobian: float = 1e-3) -> dict[str, float]:
        """Coordinate-rate formulas in a regular local road tube.

        The numerical arc-length inverse is approximate. The formulas use the
        geometric arc-length interpretation; they do not certify a globally
        unique projection or exact differentiation of the PCHIP inverse.
        No tire/body dynamics are evaluated here.
        """
        if min_coordinate_jacobian <= 0:
            raise ValueError("The Jacobian threshold must be positive.")
        e = observation["e_left_m"]
        ep = observation["heading_error_rad"]
        curvature = float(observation["road"]["curvature_1pm"])
        jacobian = 1.0 - curvature*e
        if not np.isfinite(jacobian) or jacobian <= min_coordinate_jacobian:
            raise ValueError("Road coordinates are singular or outside the permitted local tube.")
        tangential_speed = state.vx_mps*np.cos(ep) - state.vy_mps*np.sin(ep)
        lateral_speed = state.vx_mps*np.sin(ep) + state.vy_mps*np.cos(ep)
        sdot = tangential_speed/jacobian
        return {"s_dot_mps": float(sdot),
                "e_dot_mps": float(lateral_speed),
                "heading_error_dot_radps": float(state.yaw_rate_radps-curvature*sdot),
                "coordinate_jacobian": float(jacobian)}

    def wheel_positions(self, state: PlanarState) -> FloatArray:
        c, s = np.cos(state.yaw_rad), np.sin(state.yaw_rad)
        rotation = np.array([[c, -s], [s, c]])
        return self.wheels.body_positions() @ rotation.T + state.xy

    def wheel_environment(self, state: PlanarState) -> dict[str, Any]:
        """A plain-Python payload; it is NOT a native ExternalInfluences instance.

        lambda_mue and z_height_road_m match the concepts in the earlier integration
        guide. A separately tested binding adapter must copy them into a native
        object. Merely setting a height field does not establish terrain support.
        """
        xy = self.wheel_positions(state)
        values = self.surface.sample_xy(xy)
        mu = np.asarray(values["friction_multiplier"], dtype=float)
        height = np.asarray(values["height_m"], dtype=float)
        if mu.shape != (4,) or height.shape != (4,):
            raise ValueError("A wheel-surface query must return four scalar values per property.")
        if not np.isfinite(mu).all() or not np.isfinite(height).all() or np.any(mu <= 0):
            raise ValueError("Surface values must be finite with positive friction multipliers.")
        return {
            "wheel_names": list(WHEEL_NAMES),
            "wheel_positions_xy_m": xy,
            "lambda_mue": {name: float(mu[i]) for i, name in enumerate(WHEEL_NAMES)},
            "z_height_road_m": {name: float(height[i]) for i, name in enumerate(WHEEL_NAMES)},
        }
