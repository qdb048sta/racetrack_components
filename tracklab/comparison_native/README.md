# Native OCD double-track reference

This package compiles and runs the actual TUM Open-Car-Dynamics composition
`WHEEL_TORQUE / PT1 / DOUBLE_TRACK / MF_SIMPLE / DEFAULT` at commit
`94f8fb187fb0ed22bba1d809bd74f66d1ff75af4`. It has **33 continuous states**.
The existing `tracklab.driving_OCD` package remains the separate 11-state
single-track/linear-tire composition.

## Build and run

```bash
python -m tracklab.comparison_native.build_native
python -m unittest tracklab.comparison_native.test_native -v
```

The build requires Linux/WSL or macOS, Git, a C++20 compiler, and Eigen3 headers.
It can use the Eigen headers packaged with CasADi. ROS and CMake are unnecessary.
Use `--source /path/to/Open-Car-Dynamics --no-fetch` with an existing checkout.
Every upstream source and dependency is checked against the existing
[pinned manifest](../driving_OCD/native/upstream.json). The native build writes
the compiler command, binary hash, adapter hash, and source provenance to the
ignored `native/build_info.json`. The downloaded source and compiled binary are
reproducible local build products.

```python
from tracklab.comparison_native.vehicle import (
    NativeDoubleTrack, normalized_parameters,
)

with NativeDoubleTrack(normalized_parameters()) as plant:
    # X, Y, yaw, longitudinal speed, lateral speed, yaw rate, front steer
    plant.reset([0, 0, 0, 8, 0, 0, 0])
    observation = plant.step([40, 40, 40, 40], steering=0.03, dt=0.1)
    snapshot = plant.state.copy()      # All 33 native states
    four_wheel_loads = plant.diagnostics["tire_force_vertical_N"]
    plant.reset(snapshot)             # Exact state restoration
```

Torque and friction order is FL, FR, RL, RR. `step` returns seven common
observations. `.state` and `.full_state` return all 33 states; `.observation`
returns the seven observations. Commands are wheel torques, not tire forces.
The controller's allocation of an acceleration command to torques is outside
the native plant. The adapter accepts per-wheel friction multipliers, while
wind, road height and external force inputs remain zero.

`NativeDoubleTrack()` retains the upstream vehicle parameters.
`normalized_parameters()` explicitly changes eight tire peak coefficients
from 1.7 to 1.0 and steering magnitude from 0.3 to 0.5 rad for the comparison.
This is a normalized demonstration car, not a measured tire/vehicle calibration.

## Exact state decomposition

| Native component | States | Count |
|---|---|---:|
| Planar motion | $X,Y,\psi,v_x,v_y,r$ | 6 |
| Body heave, roll, pitch | $z,\dot z,\phi,\dot\phi,\theta,\dot\theta$ | 6 |
| Unsprung wheel heave | $z_{w,i},\dot z_{w,i}$, four wheels | 8 |
| Tire relaxation | $\bar\kappa_i,\bar\alpha_i$, four wheels | 8 |
| Wheel-torque drivetrain | $\omega_i$, four wheels | 4 |
| Steering actuator | $\delta$ | 1 |
| Total | 28 vehicle-dynamics + 4 drivetrain + 1 steering | 33 |

The exact storage order is `native.STATE_NAMES`, which follows the upstream
[double-track state declaration][states]. These are state counts; they should
not be described as 33 mechanical degrees of freedom.

## Equations and conventions

The following is a readable summary of the pinned implementation, with its
signs and regularizations retained. The [equations header][equations] is the
executable specification. Let wheel coordinates in the body frame be
$(a_i,b_i)\in\{(l_f,\pm t_f/2),(-l_r,\pm t_r/2)\}$.

$$
\begin{bmatrix}v^t_{x,i}\\v^t_{y,i}\end{bmatrix}
=R(-\delta_i)
\begin{bmatrix}v_x-rb_i\\v_y+ra_i\end{bmatrix},\qquad
\begin{bmatrix}F^b_{x,i}\\F^b_{y,i}\end{bmatrix}
=R(\delta_i)\begin{bmatrix}F^t_{x,i}\\F^t_{y,i}\end{bmatrix}.
$$

Front steering is parallel, $\delta_{FL}=\delta_{FR}=\delta$; rear steer is
zero under the default PT1 actuator. Toe offsets are zero in this benchmark.

$$
\dot X=v_x\cos\psi-v_y\sin\psi,\quad
\dot Y=v_x\sin\psi+v_y\cos\psi,\quad \dot\psi=r,
$$
$$
\dot v_x=\frac{\sum_iF^b_{x,i}+F_{a,x}}m+rv_y,\qquad
\dot v_y=\frac{\sum_iF^b_{y,i}+F_{a,y}}m-rv_x,
$$
$$
\dot r=\frac{\sum_i(a_iF^b_{y,i}-b_iF^b_{x,i})+M_{a,z}}{I_z}.
$$

The extra left/right longitudinal-force yaw moment is retained. Instantaneous
slips are regularized exactly as in the source:

$$
\kappa_i=\operatorname{clip}\left(
\frac{R_i\omega_i-v^t_{x,i}}{\max(3,v^t_{x,i})},-1,1\right),\qquad
\alpha_i=-\operatorname{atan2}(v^t_{y,i},\max(1,v^t_{x,i})).
$$

The tire forces use **relaxed** slips, which are native dynamic states:

$$
\dot{\bar\kappa}_i=(\kappa_i-\bar\kappa_i)/T_{x,i},\quad
T_{x,i}=0.3/\operatorname{clip}(|v^t_{x,i}|,10,100),
$$
$$
\dot{\bar\alpha}_i=(\alpha_i-\bar\alpha_i)/T_{y,i},\quad
T_{y,i}=0.2/\operatorname{clip}(|v^t_{x,i}|,10,100).
$$

For $f(s;B,C,D,E)=D\sin(C\arctan(Bs-E(Bs-\arctan(Bs))))$, the selected
[MF_SIMPLE tire source][mf] evaluates

$$
F^t_{x,i}=\lambda_{\mu,i}F_{z,i}f(\bar\kappa_i;B_x,C_x,D_x,E_x),\qquad
F^t_{y,i}=\lambda_{\mu,i}F_{z,i}f(\tan\bar\alpha_i;B_y,C_y,D_y,E_y).
$$

**MF_SIMPLE has independent pure-slip longitudinal and lateral forces.** It
does not impose a friction circle or a combined-slip reduction. Therefore
$\sqrt{F_x^2+F_y^2}/(\mu F_z)$ can exceed one without violating an implemented
constraint. That diagnostic must not be mistaken for a native constraint.
OCD's separately named MF_SIMPLE_EXTENDED and MF52 tires are different choices.

Vertical loads are dynamic, not algebraic weight transfer. In the source's
vertical convention the tire spring force is negative at static compression:

$$
F_{t,i}=\min\{c_{t,i}(z_{w,i}-z^0_{t,i}),-0.001\},\qquad F_{z,i}=-F_{t,i}.
$$

The suspension force $S_i$ combines spring, damper, antiroll-bar and geometric
axle-jacking forces. For example, the front-left spring deflection is
$-s^0_{FL}+z+\phi t_f/2-\theta l_{f,s}-z_{w,FL}$. The source uses the sprung
mass center for suspension deflections and the total-mass center in the
torque balances. It then integrates

$$
m_s\ddot z=-\sum_iS_i-m_sg+F_{a,z},\qquad
m_{w,i}\ddot z_{w,i}=S_i-F_{t,i}-m_{w,i}g,
$$
$$
I_x\ddot\phi=h\sum_iF^b_{y,i}-\sum_i b_iS_i+M_{a,x},
$$
$$
I_y\ddot\theta=-h\sum_iF^b_{x,i}+\sum_i a_iS_i
+m_sg(l_f-l_{f,s})+M_{a,y}.
$$

All static preload and antiroll/jacking laws are evaluated by unmodified C++.
The report does not substitute a quasi-static load-transfer formula for them.

The [wheel drivetrain][drivetrain] and [steering actuator][steering] give

$$
\dot\omega_i=\frac{T_i-R_i(F^t_{x,i}+c_{rr}F_{z,i})}{J_i+J_{low}},\quad
J_{low}=\max\{0,15J_r(1-\operatorname{mean}(\omega_i)/20)\},
$$
$$
\dot\delta=\operatorname{clip}((\delta_{cmd}-\delta)/0.06,-0.5,0.5).
$$

At a steering magnitude limit, outward steering rates are set to zero. When
$\omega_i<10^{-5}$, negative wheel angular acceleration is suppressed. The
rolling-resistance implementation assumes forward motion.

## Shared physical parameters

| Parameter | Value used before explicit overrides |
|---|---:|
| Total / sprung mass | 800 / 740 kg |
| Per-wheel unsprung mass | 15 kg |
| Yaw / pitch / roll inertia | 1000 / 500 / 100 kg m² |
| Front / rear CG distance | 1.724 / 1.476 m |
| Front / rear track width | 1.6 / 1.5 m |
| CG / roll center / pitch center height | 0.3 / 0.2 / 0.2 m |
| Wheel radius; front / rear spin inertia | 0.3 m; 1.15 / 1.35 kg m² |
| Suspension stiffness front / rear | 120000 / 90000 N/m |
| Suspension damping front / rear | 1300 / 900 Ns/m |
| Antiroll virtual stiffness front / rear | 12000 / 5000 N/m |
| Tire vertical stiffness | 250000 N/m per wheel |
| Air density; aero area; drag / lift coefficient | 1.225 kg/m³; 1 m²; 1 / −1.5 |
| Rolling resistance coefficient | 0.025 |
| Longitudinal MF $B,C,D,E$ | 20, 2, **1 normalized**, 0.7 |
| Lateral MF $B,C,D,E$ | 10, 1.6, **1 normalized**, −1.9 |
| Steering time constant / rate / normalized angle | 0.06 s / 0.5 rad/s / 0.5 rad |

The [parameter declarations][parameters] define upstream values; the MF and
steering normalizations above are explicit comparison choices.

## Initial conditions and numerical boundaries

For a seven-value reset, all native vertical velocities and tire slip rates
start from the constructed initial state. The adapter solves seven native
vertical/roll/pitch equilibrium equations for body heave/roll/pitch and four
wheel heights while keeping the requested planar pose, velocity, yaw rate and
wheel speed fixed. Longitudinal slip starts at zero; there is no claim of a
longitudinal force equilibrium. At 8 m/s, zero lateral motion and the default
aerodynamics, this gives total normal load 7906.8 N = 7848 N weight + 58.8 N
downforce. Full-state resets restore a snapshot directly without re-trimming.

The adapter uses upstream fixed-step DoPri integration with 0.0005 s substeps
by default. Upstream composition uses the previous RHS evaluation's cached
wheel speeds and its public output describes the last RK stage. The bridge
performs an upstream zero-duration step at reset/evaluation/output boundaries
to synchronize output and caches without changing the continuous state.
Physical substeps retain the upstream composition. A half-step check is
included; this numerical detail prevents claiming a pure textbook
Dormand–Prince convergence order for the assembled upstream implementation.

The selected source logs a road-height input but does not use that quantity in
its vertical tire spring law. This adapter consequently benchmarks a flat road
and makes no claim about bumps or banking. Tire contact is floored at 0.001 N;
there is no discrete flight/contact solver. The implementation regularizes
low-speed slips and prevents wheel reversal. Tests cover vertical equilibrium,
mirrored steering symmetry, snapshot continuation, independent-slip tire
behavior, and half-step agreement.

## Source and license

The local bridge adapts the repository's existing C interface to the native
double-track template. No upstream equations are copied into the Python plant.
Upstream copyrights and Apache-2.0 license are preserved in the downloaded
checkout; the repository also contains the [upstream license copy][license]
and [dependency notices][notice]. Track-data attribution is unchanged.

[states]: https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_vehicle_dynamics_double_track_cpp/include/ocd_vehicle_dynamics_double_track_cpp/states.hpp
[equations]: https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_vehicle_dynamics_double_track_cpp/include/ocd_vehicle_dynamics_double_track_cpp/vehicle_dynamics_equations_impl.hpp
[parameters]: https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_vehicle_dynamics_double_track_cpp/include/ocd_vehicle_dynamics_double_track_cpp/vehicle_dynamics_model_impl.hpp
[mf]: https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_tire_models_cpp/src/mf_simple.cpp
[drivetrain]: https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_drivetrain_wheel_torque_cpp/src/drivetrain_direct_torque_eqns.cpp
[steering]: https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_steering_actuator_pt1_cpp/src/steering_actuator_pt1_model.cpp
[license]: ../driving_OCD/native/LICENSE-Open-Car-Dynamics.txt
[notice]: ../driving_OCD/native/NOTICE.txt
