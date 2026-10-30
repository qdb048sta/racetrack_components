"""Parameters for the pinned native Open-Car-Dynamics composition.

The four linear tires use explicit demonstration overrides: C_alpha=14 and
C_sx=100. The first gives restoring lateral forces with the single-track slip
convention; the second makes wheel-speed integration less stiff. These are not
measured tire parameters. Body dimensions are used only for replay/clearance.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path

import numpy as np

from ..driving.config import DrivingConfig as BaseDrivingConfig


OCD_COMMIT = "94f8fb187fb0ed22bba1d809bd74f66d1ff75af4"


@dataclass(frozen=True)
class VehicleConfig:
    mass_kg: float = 800.0
    yaw_inertia_kgm2: float = 1000.0
    wheelbase_m: float = 3.2
    lf_m: float = 1.724
    body_length_m: float = 4.4
    body_width_m: float = 1.9
    visual_track_width_m: float = 1.6
    cornering_coefficient_per_rad: float = 14.0
    longitudinal_slip_coefficient: float = 100.0
    wheel_radius_m: float = 0.3
    wheel_inertia_front_kgm2: float = 1.15
    wheel_inertia_rear_kgm2: float = 1.35
    rolling_resistance_coefficient: float = 0.025
    aero_area_m2: float = 1.0
    aero_drag_coefficient: float = 1.0
    aero_lift_coefficient: float = -1.5
    aero_center_offset_m: float = 0.0
    air_density_kgpm3: float = 1.225
    steering_limit_rad: float = 0.5
    steering_rate_limit_radps: float = 0.5
    steering_time_constant_s: float = 0.06
    steering_static_offset_rad: float = 0.0
    acceleration_limit_mps2: float = 2.5
    braking_limit_mps2: float = 4.0
    # Reference for force-utilization diagnostics only. The native linear tire
    # has no friction circle or saturation; road lambda_mue scales its forces.
    nominal_friction: float = 1.0

    @property
    def lr_m(self) -> float:
        return self.wheelbase_m - self.lf_m

    @property
    def gravity_mps2(self) -> float:
        # Fixed G_CONST in the pinned single-track equations, not a parameter.
        return 9.81

    @property
    def drag_coefficient_area_m2(self) -> float:
        return self.aero_drag_coefficient * self.aero_area_m2

    def validate(self) -> None:
        values = asdict(self)
        if not np.isfinite(list(values.values())).all():
            raise ValueError("Vehicle parameters must be finite.")
        signed = {"aero_lift_coefficient", "aero_center_offset_m", "steering_static_offset_rad"}
        nonnegative = {"rolling_resistance_coefficient", "aero_drag_coefficient"}
        for name, value in values.items():
            if name in signed:
                continue
            if name in nonnegative:
                if value < 0:
                    raise ValueError(f"{name} must be nonnegative.")
            elif value <= 0:
                raise ValueError(f"{name} must be positive.")
        if self.lf_m >= self.wheelbase_m:
            raise ValueError("lf must be less than wheelbase.")
        if abs(self.steering_static_offset_rad) >= self.steering_limit_rad:
            raise ValueError("Steering offset must be smaller than the steering limit.")

    def native_parameters(self) -> dict[str, float]:
        """Upstream parameter names for WHEEL_TORQUE/PT1/SINGLE_TRACK/LINEAR/DEFAULT."""
        self.validate()
        prefix = "vehicle_dynamics_single_track."
        params = {
            prefix + "mass_vehicle_kg": self.mass_kg,
            prefix + "moment_of_inertia_kgpm2.z": self.yaw_inertia_kgm2,
            prefix + "wheelbase_m": self.wheelbase_m,
            prefix + "cog.distance_from_front_axle_m": self.lf_m,
            prefix + "tire.rolling_radius_m.front": self.wheel_radius_m,
            prefix + "tire.rolling_radius_m.rear": self.wheel_radius_m,
            prefix + "tire.rolling_resistance_coefficient": self.rolling_resistance_coefficient,
            prefix + "aerodynamics.A_m2": self.aero_area_m2,
            prefix + "aerodynamics.c_d": self.aero_drag_coefficient,
            prefix + "aerodynamics.c_l": self.aero_lift_coefficient,
            prefix + "aerodynamics.d_aero_center_cog_m": self.aero_center_offset_m,
            prefix + "aerodynamics.air_density_kgpm3": self.air_density_kgpm3,
            "drivetrain.I_wheel_front_kgm2": self.wheel_inertia_front_kgm2,
            "drivetrain.I_wheel_rear_kgm2": self.wheel_inertia_rear_kgm2,
            "steering_actuator.angle_max_rad": self.steering_limit_rad,
            "steering_actuator.angle_rate_max_radps": self.steering_rate_limit_radps,
            "steering_actuator.T_PT1": self.steering_time_constant_s,
            "steering_actuator.static_offset_rad": self.steering_static_offset_rad,
        }
        for wheel in ("front_left", "front_right", "rear_left", "rear_right"):
            tire = prefix + f"z.tire_model.{wheel}."
            params[tire + "C_alpha"] = self.cornering_coefficient_per_rad
            params[tire + "C_sx"] = self.longitudinal_slip_coefficient
        return params


@dataclass(frozen=True)
class DrivingConfig(BaseDrivingConfig):
    integration_step_s: float = 0.0002
    logging_step_s: float = 0.01
    prediction_substeps: int = 1
    collocation_degree: int = 2

    def validate(self) -> None:
        super().validate()
        for name in ("horizon_steps", "prediction_substeps", "maximum_solver_iterations", "collocation_degree"):
            value = getattr(self, name)
            if not isinstance(value, (int, np.integer)) or isinstance(value, bool):
                raise ValueError(f"{name} must be an integer.")
        if self.collocation_degree != 2:
            raise ValueError("The predictor uses two-stage Radau IIA (collocation_degree=2).")
        if not self.integration_step_s <= self.logging_step_s <= self.control_period_s:
            raise ValueError("Require integration step <= logging step <= control period.")
        for larger, smaller in ((self.logging_step_s, self.integration_step_s),
                                (self.control_period_s, self.logging_step_s)):
            if not np.isclose(larger / smaller, round(larger / smaller), rtol=0, atol=1e-8):
                raise ValueError("Logging and control intervals must contain whole integration/logging steps.")


def load_config(path: str | Path | None = None) -> tuple[VehicleConfig, DrivingConfig]:
    if path is None:
        vehicle, driving = VehicleConfig(), DrivingConfig()
    else:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        if set(raw) - {"vehicle", "driving"}:
            raise ValueError("Config root accepts only vehicle and driving.")
        vehicle = VehicleConfig(**raw.get("vehicle", {}))
        driving = DrivingConfig(**raw.get("driving", {}))
    vehicle.validate()
    driving.validate()
    return vehicle, driving
