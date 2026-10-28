"""Explicit educational parameters; no measured Norisring grip is implied."""
from dataclasses import dataclass, asdict, fields
import json
from pathlib import Path

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
    nominal_friction: float = 1.0
    gravity_mps2: float = 9.81
    drag_coefficient_area_m2: float = 0.75
    air_density_kgpm3: float = 1.225
    steering_limit_rad: float = 0.50  # CHANGED from earlier graph-road demo's 0.30
    steering_rate_limit_radps: float = 0.9
    steering_time_constant_s: float = 0.12
    acceleration_limit_mps2: float = 2.5
    braking_limit_mps2: float = 4.0
    velocity_regularization_mps: float = 0.2

    @property
    def lr_m(self):
        return self.wheelbase_m - self.lf_m

    def validate(self):
        import numpy as np
        if not np.isfinite(list(asdict(self).values())).all():
            raise ValueError('Vehicle parameters must be finite.')
        if any(v <= 0 for v in asdict(self).values()):
            raise ValueError('All vehicle parameters must be strictly positive.')
        if self.lf_m >= self.wheelbase_m:
            raise ValueError('lf must be less than wheelbase.')
        if self.acceleration_limit_mps2 >= .9*self.nominal_friction*self.gravity_mps2:
            raise ValueError('Acceleration limit leaves insufficient nominal lateral grip.')
        if self.braking_limit_mps2 >= .9*self.nominal_friction*self.gravity_mps2:
            raise ValueError('Braking limit leaves insufficient nominal lateral grip.')

@dataclass(frozen=True)
class DrivingConfig:
    maximum_speed_mps: float = 18.0
    lateral_acceleration_budget_mps2: float = 2.0
    planning_acceleration_mps2: float = 1.5
    planning_braking_mps2: float = 2.0
    speed_grid_spacing_m: float = 0.5
    initial_speed_mps: float = 8.0
    initial_lateral_offset_m: float = 0.0
    control_period_s: float = 0.10
    integration_step_s: float = 0.01
    horizon_steps: int = 25
    prediction_substeps: int = 4
    maximum_time_s: float = 600.0
    laps: float = 1.0
    maximum_solver_iterations: int = 75
    corridor_center_margin_m: float = 2.45
    projection_window_m: float = 10.0

    def validate(self):
        import numpy as np
        if not np.isfinite(list(asdict(self).values())).all():
            raise ValueError('Driving parameters must be finite.')
        for k,v in asdict(self).items():
            if k != 'initial_lateral_offset_m' and v <= 0:
                raise ValueError(f'{k} must be positive.')
        if self.integration_step_s > self.control_period_s:
            raise ValueError('Integration step must not exceed control period.')
        if self.horizon_steps < 5 or self.prediction_substeps < 1:
            raise ValueError('Horizon >= 5 and prediction substeps >= 1 required.')
        if self.initial_speed_mps < 1.0:
            raise ValueError('This forward-driving dynamic model starts at >=1 m/s.')


def load_config(path=None):
    if path is None:
        vehicle,driving=VehicleConfig(),DrivingConfig()
    else:
        raw=json.loads(Path(path).read_text())
        if set(raw)-{'vehicle','driving'}:
            raise ValueError('Config root accepts only vehicle and driving.')
        vehicle=VehicleConfig(**raw.get('vehicle',{}))
        driving=DrivingConfig(**raw.get('driving',{}))
    vehicle.validate();driving.validate()
    return vehicle,driving
