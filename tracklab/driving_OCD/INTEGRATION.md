# TUM OCD integration walkthrough

[Overview and running](README.md) · [Model](MODEL.md) · [Integration walkthrough](INTEGRATION.md) · [Validation](VALIDATION.md) · [Project guide](../../README.md)

The directory is named `tracklab/driving_OCD`. The name `drive_OCD` appears in
the runner and output names. No upstream vehicle implementation was pasted into
this directory. The build downloads a separate upstream checkout, compiles its
selected components, and connects them to the existing road and replay APIs.
Only the symbolic prediction equations were translated into Python/CasADi.

## 1. Identify and fetch the exact original code

[`native/upstream.json`](native/upstream.json) is the build-input manifest. It
records the upstream repository, commit
`94f8fb187fb0ed22bba1d809bd74f66d1ff75af4`, three dependency revisions, model
composition, and 111 source/header hashes. These are immutable inputs rather
than a request for the latest upstream release. The dependency revisions are:

| Checkout | Pinned revision |
| --- | --- |
| `submodules/tam__common` | `021e9d07f47865eb48de07382d47ccbd85554dba` |
| `tam__common/submodules/parameter_management` | `66c1481b16d9a86497751f91b0d048ad8d129921` |
| `tam__common/submodules/time_series_logging` | `d91a26211bdba041e3e1d329755f2e5697b1db0a` |

[`build_native.py`](build_native.py) is new build orchestration.
`fetch_source()` clones/checks out that revision into the ignored
`dev/Open-Car-Dynamics` cache and initializes the three required submodules.
`verify_source()` checks their Git revisions and the manifest's LF-normalized
hashes; a different commit or changed compiled source is rejected. For a source
tree on a WSL Windows mount, the helper stages the required directories in a
temporary Linux directory to reduce compilation overhead. It then invokes CMake,
copies the resulting OS-specific library into `native/`, and writes local
`build_info.json` with source, adapter, and library hashes.

The temporary source staging copies upstream files unchanged. It does not
extract/rewrite vehicle equations. There is no ROS installation or upstream
Python package installation in this build. A Linux `.so` must be used by Linux
Python; Windows Python needs a separately compiled `.dll`.

## 2. Select the native components and compile them

[`native/CMakeLists.txt`](native/CMakeLists.txt) is a small local C++20 build.
Its source list points into `${OCD_SOURCE_DIR}`: `linear.cpp`, `default.cpp`,
the direct-wheel-torque drivetrain files, the PT1 steering file, and required
parameter-management sources. The body dynamics and vehicle composition are
templates whose implementations come from upstream headers. Header include
paths cover upstream packages and required common/type/logger dependencies;
adding include paths does not instantiate the unused double-track or other
tire models.

[`native/bridge.cpp`](native/bridge.cpp) selects the actual C++ types:

```cpp
using VD = tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel<
  tam::ocd::tire_models::Linear, tam::ocd::aerodynamics::DefaultAerodynamicsModel>;
using DT = tam::ocd::drivetrain::DrivetrainWheelTorqueModel;
using SA = tam::ocd::steering_actuator::PT1SteeringActuatorModel;
using Vehicle = tam::ocd::VehicleModel<DT, SA, VD>;
```

These types are defined in the pinned upstream
[single-track model](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_vehicle_dynamics_single_track_cpp/include/ocd_vehicle_dynamics_single_track_cpp/vehicle_dynamics_model.hpp),
[linear tire](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_tire_models_cpp/include/ocd_tire_models_cpp/linear.hpp),
and [vehicle composition](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_vehicle_model_cpp/include/ocd_vehicle_model_cpp/vehicle_model.hpp).
This explicitly selects one supported composition. The upstream
[Python factory](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/python3/ocd_vehicle_models_py/ocd_vehicle_models_py/get_vehicle.py#L16-L22)
defaults to FX, PT1, DOUBLE_TRACK, MF52, and DEFAULT at this revision.

The remainder of the bridge is adapter code: allocation/destruction, parameter
get/set, conversion of four-element wheel arrays, initial-state assignment,
state/diagnostic copying, and translation of C++ exceptions into a C error
message. `Handle` owns two upstream vehicles: the evolving plant and a separate
derivative evaluator. Evaluating an MPC/test state therefore does not overwrite
the plant's ongoing state.

## 3. Connect upstream stepping to Python

Upstream [`VehicleModel::step()`](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_vehicle_model_cpp/include/ocd_vehicle_model_cpp/vehicle_model_impl.hpp#L43-L66)
advances its composed state using this original line:

```cpp
x = tam::helpers::numerical::integration_step_DoPri45<state_vector_t>(bound_ode, 0, x, h);
```

The local `ocd_step()` delegates to that method:

```cpp
v.get_param_manager()->set_value("integration_step_size_s", dt / substeps);
inputs(v, torque, steering, friction);
for (int i = 0; i < substeps; ++i) v.step();
synchronize(v);
state(v, out);
```

[`native.py`](native.py) loads `tracklab_ocd.so`, `.dll`, or `.dylib` through
`ctypes.CDLL`, declares C argument/result types, checks the model/commit identity
string, validates NumPy buffers, and owns the C++ handle. `evaluate()` returns
eleven derivatives and per-wheel diagnostic arrays; `step()` returns eleven
states. `TRACKLAB_OCD_LIBRARY` can select another compatible built library.
The runtime identity check is separate from build-time source/hash verification;
it is not an independent proof of a third-party binary's contents.

`synchronize()` is an intentional local adaptation. Upstream's
[`ode()` evaluation order](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_vehicle_model_cpp/include/ocd_vehicle_model_cpp/vehicle_model_impl.hpp#L82-L125)
evaluates the body using cached wheel speeds and updates those speeds later,
after evaluating the drivetrain. Also, `step()` copies public outputs from its
last Runge-Kutta stage rather than reevaluating at the accepted state. The bridge
temporarily sets the interval to zero and calls the same upstream `step()` to
refresh output/caches after reset, derivative evaluation, and a returned step.
No physical state advances in that call. The original cached evaluation order
is preserved between physical native substeps. Consequently, native finite
steps and integration of a memoryless symbolic right-hand side are not claimed
to give identical trajectories, even when their synchronized derivatives agree.

## 4. Map configuration, states, road friction, and commands

[`config.py`](config.py) defines local demonstration parameters and maps them
to upstream names in `VehicleConfig.native_parameters()`. For example:

```python
prefix + "mass_vehicle_kg": self.mass_kg,
prefix + "cog.distance_from_front_axle_m": self.lf_m,
"steering_actuator.T_PT1": self.steering_time_constant_s,
```

The prefix is `vehicle_dynamics_single_track.`. Tire parameters use the exact
upstream `z.tire_model.front_left.` / `front_right.` / `rear_left.` /
`rear_right.` namespaces. The [parameter table](MODEL.md#tire-equations-and-parameter-choices)
identifies deliberate
changes from upstream declarations; changing coefficients is configuration,
not a modification of the force law. Body length/width and visual track width
are for display and sampled road-clearance checks, not lateral load transfer.
`DrivingConfig` inherits the existing driving settings and specializes native
integration/logging and implicit prediction settings.

[`vehicle.py`](vehicle.py) adapts the existing driving interface to those native
states. The order is body `[X,Y,psi,vx,vy,r]`, then four wheel speeds, then
measured actuator position. Effective front-wheel angle is actuator position
plus `steering_static_offset_rad`; rear steering is zero. `observe()` selects
indices `[0,1,2,3,4,5,10]` for existing plots/replay. `initial_state()` initializes
each wheel speed from its axle's forward tire-frame velocity divided by radius,
clamped nonnegative; it does not solve for a steady-state operating point.

`torques()` supplies a local control-allocation rule. With `a_req = command[1]`,
each front wheel receives `m*a_req*R*lr/(2*L)` and each rear wheel receives
`m*a_req*R*lf/(2*L)`. These torques sum to the requested force times radius.
They do not impose `dvx/dt = a_req`: the native drivetrain and tire slip produce
the actual tire forces. `_friction()` repeats front/rear road multipliers in
FL, FR, RL, RR order. `rhs()` and `step()` call the native library; a missing
library produces an error. `step()` divides a requested interval into native
substeps no larger than `integration_step_s`. It first resets the native model
from the supplied state and synchronizes it, so each Python step starts from
that explicit eleven-state vector rather than retaining a previous interval's
cached intermediate values.

## 5. Translate the component equations for the optimizer

`vehicle.py::symbolic_rhs()` is the part that was rewritten: a manual CasADi
transcription of the selected continuous component laws, used by the controller
so it can differentiate its prediction constraints. It is not called to advance
the actual vehicle. Small paired excerpts show which operations correspond.

Upstream [`Linear::evaluate()`](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_tire_models_cpp/src/linear.cpp#L6-L13):

```cpp
out.lateral_force_N = p_.C_alpha * input.slip_angle_rad * input.F_z_N;
out.longitudinal_force_N = p_.C_sx * input.longitudinal_slip * input.F_z_N;
```

Local predictor, including the road multiplier and upstream rest guard:

```python
fx.append(ca.if_else(at_rest, 0., scale*p.longitudinal_slip_coefficient*slip*loads[j]))
fy.append(ca.if_else(at_rest, 0., scale*p.cornering_coefficient_per_rad*alpha*loads[j]))
```

The rest guard, tire-frame rotation, low-speed slip denominators, slip clipping,
per-wheel load floor, aero terms, and wheel constraints are also transcribed.
In the original `driving/vehicle.py`, by comparison, `SaturatingAxleTire.lateral()`
uses `cap*tanh(C_alpha*Fz*alpha/cap)` and longitudinal force is allocated directly;
neither is used by the OCD plant or predictor.

Upstream [body/yaw derivatives](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_vehicle_dynamics_single_track_cpp/include/ocd_vehicle_dynamics_single_track_cpp/vehicle_dynamics_equations_impl.hpp#L353-L370):

```cpp
x_dot_vec_[x::psi_dot_radps] = imr_.resulting_torque_Nm.z / p_.I_z;
```

Local predictor's corresponding six body derivatives:

```python
rhs = [vx*cp-vy*sp, vx*sp+vy*cp, r,
       body_fx/p.mass_kg+r*vy, body_fy/p.mass_kg-r*vx, yaw_moment/p.yaw_inertia_kgm2]
```

The force sum and yaw moment use the front tire forces rotated into the body
frame and the rear forces at zero steering. The single-track model has no roll,
pitch, suspension, lateral load transfer, or tire relaxation states. Its
vertical loads include aero downforce and aero pitch moment quasi-statically.

Upstream [low-speed wheel inertia](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_drivetrain_wheel_torque_cpp/src/drivetrain_direct_torque_eqns.cpp#L6-L36):

```cpp
I_wheel_low_speed_kgm2 = std::max(0.0, 15 * p.I_wheel_rear_kgm2 * (1 - omega_mean_radps / 20));
```

Local predictor:

```python
extra_inertia = ca.fmax(0., 15*p.wheel_inertia_rear_kgm2*(1.-ca.sum1(z[6:10])/80.))
```

`sum(omega)/80` equals `mean(omega)/20`. The following local loop also translates
the upstream input-minus-load torque equation, front/rear wheel inertias,
rolling-resistance load, and nonnegative derivative guard below `1e-5 rad/s`.
The control-allocation rule described above supplies the input torque.

Upstream [PT1 steering](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_steering_actuator_pt1_cpp/src/steering_actuator_pt1_model.cpp#L11-L24):

```cpp
double x_dot = (requested_steering_angle_rad_ - x_vec_[x::position_rad]) / p_.T_PT1;
x_dot = std::clamp(x_dot, -max_rate, max_rate);
```

Local predictor:

```python
steering_rate = ca.fmin(p.steering_rate_limit_radps, ca.fmax(-p.steering_rate_limit_radps,
                    (u[0]-z[10])/p.steering_time_constant_s))
```

The following two `if_else` expressions translate the upstream one-sided angle
bounds. Unlike the original package's smooth `tanh` actuator, this is a clipped
PT1 derivative. The offset affects effective wheel angle, not measured position.

The predictor matches the configuration exposed by `VehicleConfig`: no wind or
additional external loads, equal radius per axle, default constant tire-radius
scaling, zero lift-coefficient lookup correction, and zero pitch-dependent aero
center translation. Native low-level parameter setters expose more settings;
changing those directly does not automatically update the symbolic predictor.
Use the supported configuration mapping to keep the two consistent.

## 6. Reuse the road workflow and adapt prediction/control

| File | What it contributes and where it comes from |
| --- | --- |
| [`__init__.py`](__init__.py) | Local public exports for configuration, vehicle, and speed planner; no dynamics equations. |
| [`planning.py`](planning.py) | Imports the existing `driving.planning.CurvatureSpeedPlan` directly. Curvature envelopes, speed limits, and acceleration/braking passes are shared. |
| [`controller.py`](controller.py) | Local eleven-state CasADi/IPOPT tracking MPC. It replaces the original seven-state explicit prediction with two-stage Radau IIA collocation for stiff wheel-slip dynamics, while retaining road-width constraints, tracking objectives, warm starts, and two-command control. |
| [`simulation.py`](simulation.py) | Local closed-loop orchestration: sample road friction at axle positions, solve the MPC, hold a command while advancing the native plant, log full and observed states, and project position to measure progress. Projection never replaces the integrated XY position. |
| [`validation.py`](validation.py) | Reuses original road-projection and footprint helpers; obtains force/slip diagnostics from the native evaluator. Replays recorded controls at half the native interval to measure timestep sensitivity; this is not another controller run or real-vehicle validation. |
| [`artifacts.py`](artifacts.py) | Reuses the CSV writer and saves eleven-state raw data, seven-state display data, diagnostics, solver/prediction logs, configurations, native parameters, source manifest, library hash, metrics, and the timestep check. |
| [`replay.py`](replay.py) | Reuses `driving/replay_template.html`, inserts recorded data and predictions, changes model descriptions, and adds the static offset to the drawn steering angle. The browser displays results; it does not simulate vehicle dynamics. |
| [`plots.py`](plots.py) | Adapts the existing plotting workflow to native results, labels, and effective steering. Generates trajectory, speed, steering/sideslip, and error PNGs. |
| [`../../run_drive_OCD.py`](../../run_drive_OCD.py) | Local CLI entry: read config, construct the existing `PeriodicTrack`, simulate, save results, create plots, and optionally open the HTML. |
| [`../../configs/default_drive_OCD.json`](../../configs/default_drive_OCD.json) | Reproducible demonstration parameters for this package; changing it does not edit the native source. |
| [`../../requirements-driving.txt`](../../requirements-driving.txt) | Shared Python dependencies, including CasADi for MPC. C++ build prerequisites are separate. |

The predictor uses Radau IIA stage equations
`Z1 = z + h*(5*f1/12 - f2/12)` and
`z_next = z + h*(3*f1/4 + f2/4)`. Its larger prediction interval is distinct from
the native plant's small Dormand-Prince substep. The full model equations,
including the original package's tire and steering formulas, are documented in
the [shared driving reference](../../docs/DRIVING.md) and
[native model equations](MODEL.md#native-model-equations).

`data.py`, `interpolation.py`, `geometry.py`, `road.py`, the surface model, and
the input CSV are shared through the existing APIs. The OCD integration does
not substitute a TUM road, coordinate system, spline, or projection algorithm.

## 7. Verify identity/equations and retain attribution

[`../../tests/test_driving_OCD.py`](../../tests/test_driving_OCD.py) checks source
hashes and manifest identity, library identity, exact parameter mapping, separate evaluator
state, state round trips, kinematics, torque-to-wheel coupling, slip guards,
linear forces and road scaling, lack of a force cap, PT1 limits, stationary
behavior, symbolic/native derivatives including parameter overrides, numerical
refinement, shared planning, and short closed-loop output generation. Derivative
agreement is tested at sample states; it does not establish bitwise equivalence
of the native integrator's cached stages and the predictor's collocation.

[`native/NOTICE.txt`](native/NOTICE.txt) distinguishes upstream implementations
from local adapter/controller/predictor additions. License copies are
[`LICENSE-Open-Car-Dynamics.txt`](native/LICENSE-Open-Car-Dynamics.txt),
[`LICENSE-tam-common.txt`](native/LICENSE-tam-common.txt),
[`LICENSE-parameter-management.txt`](native/LICENSE-parameter-management.txt),
and [`LICENSE-time-series-logging.txt`](native/LICENSE-time-series-logging.txt).
The downloaded source retains its original notices. This documentation explains the
integration and setup; it is not an upstream specification.

[`native/.gitignore`](native/.gitignore) and the [root ignore file](../../.gitignore)
exclude compiled libraries, compiler output, local build metadata, downloaded
source cache, and generated simulation results. `upstream.json`, the bridge,
CMake file, notices, licenses, Python code, and input/configuration files are
source artifacts retained in Git. `build_info.json` describes a local build;
`provenance.json` describes a generated run.
