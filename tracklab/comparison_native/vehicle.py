"""A common observation interface around all 33 native OCD continuous states."""
from __future__ import annotations

import math
from typing import Mapping, Sequence
import numpy as np

from .native import NativeVehicle, STATE_NAMES

OBSERVATION_INDICES = np.array([0, 1, 3, 6, 7, 9, 32])


def normalized_parameters(mu=1.0):
    """Explicit demonstrator normalization: native MF peak coefficients D=mu.

    Upstream uses D=1.7. Changing both to 1.0 permits the common benchmark to
    compare tires with the same nominal pure-slip friction coefficient. The
    steering angle limit is also normalized from 0.3 to 0.5 rad. Other native
    parameters retain upstream defaults unless overridden.
    """
    if not np.isfinite(mu) or mu <= 0:
        raise ValueError("mu must be finite and positive")
    parameters = {f"vehicle_dynamics_double_track.z.tire_model.{wheel}.{axis}.D": float(mu)
                  for wheel in ("front_left", "front_right", "rear_left", "rear_right")
                  for axis in ("longitudinal", "lateral")}
    parameters["steering_actuator.angle_max_rad"] = 0.5
    return parameters


class NativeDoubleTrack:
    """Actual 33-state OCD plant, with observation [X,Y,yaw,vx,vy,r,delta].

    Inputs are four wheel torques in Nm (FL, FR, RL, RR), plus front steering
    target in radians. All 28 dynamics states, four wheel rotations and one
    steering state persist between calls; there is no repeated planar reset.
    Default parameters are the original upstream defaults. Pass
    ``normalized_parameters()`` explicitly for the normalized comparison.
    """
    name = "native_ocd_mf_simple"
    state_dimension = 33

    def __init__(self, parameters: Mapping[str, float | Sequence[float]] | None = None,
                 integration_step_s=0.0005, library_path=None):
        if not np.isfinite(integration_step_s) or integration_step_s <= 0:
            raise ValueError("integration_step_s must be finite and positive")
        self.integration_step_s = float(integration_step_s)
        self.native = NativeVehicle(parameters, library_path)
        self._parameters = self.native.parameters()

    def reset(self, body, steering=None, settle_vertical=True):
        body = np.asarray(body, dtype=float)
        if body.shape == (33,) and np.isfinite(body).all():
            self.native.reset(body)
            return self.observation
        if body.shape not in ((6,), (7,)) or not np.isfinite(body).all():
            raise ValueError("body must be six/seven observations or a full 33-state snapshot")
        if steering is None:
            steering = float(body[6]) if len(body) == 7 else 0.0
        if not np.isfinite(steering):
            raise ValueError("steering must be finite")
        p = self._parameters
        prefix = "vehicle_dynamics_double_track."
        lf = p[prefix + "cog.distance_from_front_axle_m"]
        lr = p[prefix + "wheelbase_m"] - lf
        tf, tr = p[prefix + "track_width_m.front"], p[prefix + "track_width_m.rear"]
        radii = np.array([p[prefix+"tire.rolling_radius_m.front"]]*2 + [p[prefix+"tire.rolling_radius_m.rear"]]*2)
        offset = p["steering_actuator.static_offset_rad"]
        angles = np.array([steering + offset, steering + offset, 0., 0.])
        vxb = body[3] - body[5] * np.array([tf/2, -tf/2, tr/2, -tr/2])
        vyb = body[4] + body[5] * np.array([lf, lf, -lr, -lr])
        vtx = vxb * np.cos(angles) + vyb * np.sin(angles)
        vty = -vxb * np.sin(angles) + vyb * np.cos(angles)
        x = np.zeros(33)
        x[OBSERVATION_INDICES[:6]] = body[:6]
        x[28:32] = np.maximum(vtx, 0.) / radii
        x[24:28] = -np.arctan2(vty, np.maximum(1., vtx))
        x[32] = steering
        # Solve only the native vertical, roll and pitch force equilibrium.
        # Global pose, planar velocity, yaw rate and wheel speed stay fixed.
        # Initial longitudinal tire slip remains zero: no Fx trim is claimed.
        if settle_vertical:
            from scipy.optimize import root
            coordinates = np.array([2, 4, 5, 12, 13, 14, 15])
            accelerations = np.array([8, 10, 11, 16, 17, 18, 19])
            def residual(q):
                trial = x.copy()
                trial[coordinates] = q
                dx, _ = self.native.evaluate(trial, np.zeros(4), steering)
                return dx[accelerations]
            solved = root(residual, x[coordinates], tol=1e-10)
            if not np.isfinite(solved.x).all() or np.max(np.abs(residual(solved.x))) > 1e-7:
                raise RuntimeError("Native vertical equilibrium initialization did not converge")
            x[coordinates] = solved.x
        self.native.reset(x)
        return self.observation

    def step(self, torques, steering, dt, substeps=None, friction=(1., 1., 1., 1.)):
        if substeps is None:
            substeps = max(1, math.ceil(float(dt) / self.integration_step_s - 1e-10))
        return self.native.step(torques, steering, dt, substeps, friction)[OBSERVATION_INDICES]

    @property
    def state(self):
        return self.native.state

    @property
    def observation(self):
        return self.native.state[OBSERVATION_INDICES]

    @property
    def full_state(self):
        return self.native.state

    @property
    def diagnostics(self):
        d = self.native.diagnostics
        x = self.full_state
        d.update({"omega_radps": x[28:32], "roll_rad": float(x[4]),
                  "pitch_rad": float(x[5]), "heave_m": float(x[2]),
                  "wheel_speeds_radps": x[28:32],
                  "wheel_heave_m": x[12:16], "state_dimension": 33})
        return d

    def parameters(self):
        return self.native.parameters()

    def close(self):
        self.native.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
