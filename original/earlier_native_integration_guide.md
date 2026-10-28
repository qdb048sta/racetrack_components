# Connection to a native checkout

The two default demonstrations run without a checkout, ROS, a compiler, or an upstream Python extension. They are explicit modified ports. The following native configuration/patch instructions were derived from the pinned source; **the native build and simulations have not been executed here**.

## 1. Select and configure a native model

After building the upstream Python binding according to the pinned repository's README, the source-supported API pattern is:

```python
from ocd_vehicle_models_py import (
    VehicleFactory, DrivetrainType, SteeringActuatorType,
    VehicleDynamicsType, TireType, AerodynamicsType,
)
veh = VehicleFactory.create_vehicle(
    drivetrain=DrivetrainType.FX,
    steering=SteeringActuatorType.PT1,
    vdm=VehicleDynamicsType.SINGLE_TRACK,
    tire=TireType.LINEAR,
    aero=AerodynamicsType.DEFAULT,
)
pm = veh.get_param_manager()
params = pm.get_parameters_as_dict()
updates = {
    "vehicle_dynamics_single_track.mass_vehicle_kg": 800.0,
    "vehicle_dynamics_single_track.moment_of_inertia_kgpm2.z": 1000.0,
    "vehicle_dynamics_single_track.wheelbase_m": 3.2,
    "vehicle_dynamics_single_track.cog.distance_from_front_axle_m": 1.724,
    "steering_actuator.angle_max_rad": 0.3,
    "steering_actuator.angle_rate_max_radps": 0.5,
}
for wheel in ("front_left", "front_right", "rear_left", "rear_right"):
    prefix = f"vehicle_dynamics_single_track.z.tire_model.{wheel}."
    updates[prefix + "C_alpha"] = 14.0
    updates[prefix + "C_sx"] = 25.0
missing = set(updates) - set(params)
if missing:
    raise KeyError(f"Check selected model and commit; missing keys: {sorted(missing)}")
params.update(updates)
pm.set_parameters_from_dict(params)
```

These settings preserve native PT1 and native force-request drivetrain dynamics. They do **not** make this native vehicle identical to example1's ideal-slip, steering-rate model. The `FX` drivetrain is a force-request interface to drivetrain/braking behavior, not a replacement of the whole drivetrain by `Fx = command`.

The source-backed parameter spelling comes from `vehicle_dynamics_single_track_cpp/.../vehicle_dynamics_model_impl.hpp` L155–200 and `linear.cpp` L14–19. Print the parameter dictionary before editing; do not assume a parameter exists for every factory combination.

For native double track select `VehicleDynamicsType.DOUBLE_TRACK`; for extended tires select `TireType.MF_SIMPLE_EXTENDED`. The physical prefix becomes `vehicle_dynamics_double_track.` and per-wheel tire prefixes become `vehicle_dynamics_double_track.z.tire_model.<wheel>.`. Its parameters include `track_width_m.front`, `track_width_m.rear`, `cog.height_m`, wheel masses, spring and damper constants. Extended-tire parameters include `longitudinal.B/C/D/E`, `lateral.B/C/D/E`, `FNOMIN`, and `PDY2`. Setting different parameters does not remove the native 28-state vertical/slip dynamics.

To implement road friction, compute the wheel positions from the native pose and your road map before each native step, set the respective entries of `external_influences.lambda_mue`, then call `veh.set_external_influences(...)`. Construct this object with `veh.construct_external_influences()` as shown by the upstream example. Native stepping remains a time-domain integration; use a smaller plant step than a controller update and account for actuator and estimator dynamics.

## 2. Add the teaching examples into the repository

The package contains an actual patch adding the two standalone demos and the optional mesh checker under the upstream example directory. It does not replace or change the compiled native models:

```bash
git checkout --detach 94f8fb187fb0ed22bba1d809bd74f66d1ff75af4
git apply --check /path/to/add_python_demos.patch
git apply /path/to/add_python_demos.patch
python python3/ocd_vehicle_models_py/example/example_1_linear_tire_mpc.py
python python3/ocd_vehicle_models_py/example/example_2_combined_slip_mpc.py
```

Each demo is still individually copyable as a single file. It imports only external Python packages, not the sibling example or native project.

## 3. Optional native combined-slip regularization

`native_smooth_combined_slip.patch` modifies `lib/ocd_tire_models_cpp/src/mf_simple_extended.cpp`.

It changes the magnitude to `sqrt(kappa*kappa + tan(alpha)*tan(alpha) + 1e-14)` and removes the small-slip early return, so signed directional projection is always performed. This is the same zero-slip regularization used in example2. It does not change B,C,D,E automatically, and it does not reproduce the entire modified Python vehicle.

```bash
git apply --check /path/to/native_smooth_combined_slip.patch
git apply /path/to/native_smooth_combined_slip.patch
```

Only local patch-context application was checked against the inspected source text. It was not compiled, run through upstream unit tests, or validated in a native full-car simulation. Rebuild and test zero slip, pure lateral/longitudinal slip, force symmetry, saturation and continuity before use.

## 4. Optional native vertical road displacement

`native_vertical_road_input.patch` modifies the four tire-spring expressions in `vehicle_dynamics_equations_impl.hpp`. It replaces

```
kt * (wheel_z - initial_compression)
```

by

```
kt * (wheel_z - road_height - initial_compression)
```

for each tire. The convention is **positive road height upwards**: increasing road height while holding wheel position fixed increases compression and normal load. A zero road input leaves the original expressions unchanged.

This optional patch is not required for either demo: both use a flat surface. It has had a local source-context check only, not a native compilation or simulation. It provides vertical base displacement; it does not by itself model slope/bank normals, gravity components, velocity along a 3D surface, or impact/contact transitions. The clamp retaining a tiny positive normal load is also still present. Do not label it a complete terrain implementation.

## 5. Steps not completed by these deliverables

A native full-order validation would require the pinned C++ build, actuator-compatible controls, a low-level torque-to-slip controller for the demo's slip commands, state estimation or extraction, and plant/model mismatch tests. The full-model controller is not supplied or claimed to have been tested. Use the reduced Python plant first to study the mathematics and numerical methods, then treat this native coupling as a separate engineering experiment.
