> Original five-model experiment. See [updated full equation reference and screenshot-thesis comparison](README.md).

# Double-track car and tire models: equations and Norisring experiment

This study adds five verified doctoral references and an executable comparison
of the **actual native OCD double-track/MF_SIMPLE composition** with four
normalized planar alternatives. It uses this repository's existing Norisring
CSV, periodic cubic road geometry, width interpolation and CasADi installation.

Start with [recorded results and figures](recorded/RESULTS.md), the
[doctoral-thesis comparison](LITERATURE.md), and the
[native model specification](../../tracklab/comparison_native/README.md).
The offline [replay](recorded/replay.html) shows recorded motion on the track.

## What is implemented

| Run ID | Vehicle dynamics | Tire forces | Normal loads | State count | Relationship to sources |
|---|---|---|---|---:|---|
| `native_ocd` | Actual C++ OCD double-track: planar motion, body heave/roll/pitch, four unsprung vertical motions, wheel spin and steering | Native `MF_SIMPLE` plus eight relaxed-slip states; independent longitudinal and lateral forces | Dynamic tire/suspension/antiroll/jacking equations | **33** | Unmodified upstream C++ at `94f8fb1`; explicitly normalized parameters |
| `mf_independent` | Planar four-wheel Newton–Euler, four wheel speeds, steering PT1 | Same pure-slip MF shapes, evaluated instantaneously | Algebraic quasi-static longitudinal/lateral transfer | **11** | Reduced OCD-family comparison; isolates omitted transient mechanics |
| `mf_ellipse` | Same planar four-wheel dynamics | Pure MF with Berntorp Eq. (4.9) longitudinal-priority friction ellipse | Same quasi-static transfer | **11** | Source equation implemented with the common OCD parameter set; not his full calibrated chassis/controller |
| `fiala` | Same planar four-wheel dynamics | Standard saturated Fiala lateral law and MF longitudinal force, with remaining circle capacity | Same quasi-static transfer | **11** | Normalized Fiala adaptation motivated by Bertipaglia; not his extended post-peak tire or MPCC |
| `mf_static` | Same planar four-wheel dynamics | Independent MF | Static axle distribution plus aerodynamic load; no acceleration-induced transfer | **11** | Controlled ablation of the load-transfer assumption |

The native state count is not a count of mechanical degrees of freedom.
The original `run_drive_OCD.py` still selects its separate native single-track
and linear-tire composition. The new launcher is `run_model_comparison.py`.

## Reproduce the experiment

Use Python 3.12 on Linux or WSL, a C++20 compiler, Git and Eigen3 headers. The
build can use Eigen bundled with CasADi; ROS and CMake are not required.

```bash
python -m pip install -r requirements-driving.txt
python -m tracklab.comparison_native.build_native

# Both speed schedules, all five plants, one complete lap each.
python run_model_comparison.py --scenario both

# Separate model discrepancy from numerical integration error.
python -m tracklab.model_comparison.validation --output outputs/model_comparison

# Regenerate the tables, figures, published CSVs and offline animation.
python -m tracklab.model_comparison.report --input outputs/model_comparison
```

For an individual run:

```bash
python run_model_comparison.py --models native_ocd --scenario demanding
python run_model_comparison.py --models mf_ellipse --scenario nominal
```

The launcher also supports a different `--track`, `--laps`, `--maximum-time`
and `--output`. Its default output is ignored build data. Publishing the
recorded report requires all ten model/scenario runs to have completed, so a
partial or failed experiment cannot silently become the finished table.

The simulation saves 50 Hz full state/force arrays, exact applied commands,
solver residuals, predictions and metadata. The repository includes readable
10 Hz trajectory CSVs, summaries, figures and validation tables. Full raw
arrays and native binaries are regenerated locally. Raw-array SHA-256 values
and the complete parameter/source manifest accompany the recorded results.

## Shared equations and coordinate conventions

Coordinates are forward $x$, left $y$, with positive counterclockwise yaw.
Wheel order is FL, FR, RL, RR. Let

$$
(a_i,b_i)\in\{(l_f,t_f/2),(l_f,-t_f/2),(-l_r,t_r/2),(-l_r,-t_r/2)\}.
$$

Front wheels have parallel steering $\delta$, rear wheels zero steering.
The tire-frame contact velocity and body-frame force follow

$$
v_i^t=R(-\delta_i)
\begin{bmatrix}v_x-rb_i\\v_y+ra_i\end{bmatrix},\qquad
F_i^b=R(\delta_i)F_i^t.
$$

All five plants retain the following planar force and moment balances:

$$
\dot X=v_x\cos\psi-v_y\sin\psi,\qquad
\dot Y=v_x\sin\psi+v_y\cos\psi,\qquad \dot\psi=r,
$$
$$
m(\dot v_x-rv_y)=\sum_i F^b_{x,i}+F_{a,x},\qquad
m(\dot v_y+rv_x)=\sum_i F^b_{y,i}+F_{a,y},
$$
$$
I_z\dot r=\sum_i(a_iF^b_{y,i}-b_iF^b_{x,i})+M_{a,z}.
$$

The yaw equation includes the moment of unequal left/right longitudinal
forces. With the chosen aerodynamic center, $M_{a,z}=0$. Body-frame drag is
$F_{a,x}=-\tfrac12\rho C_dA v_x|v_x|$ and similarly in the lateral direction.
Aerodynamic downforce is $D_a=-\tfrac12\rho C_lA v_x^2$.

### Wheel rotation, slip and steering

The torque command is not applied directly as tire force. Each wheel obeys

$$
(J_i+J_{low})\dot\omega_i=T_i-R_i(F^t_{x,i}+c_{rr}F_{z,i}),
$$
$$
J_{low}=\max\{0,15J_r(1-\operatorname{mean}(\omega_i)/20)\}.
$$

The source's forward-motion regularizations are preserved:

$$
\kappa_i=\operatorname{clip}\left(
\frac{R_i\omega_i-v^t_{x,i}}{\max(3,v^t_{x,i})},-1,1\right),\qquad
\alpha_i=-\operatorname{atan2}(v^t_{y,i},\max(1,v^t_{x,i})).
$$

Thus positive $\alpha$ produces positive lateral force. Some cited theses
use the opposite slip-angle sign; their polynomial signs must be transformed
before comparison. The denominators are numerical regularizations measured
in m/s, not physical tire parameters.

Native OCD integrates relaxed slips

$$
\dot{\bar\kappa}_i=(\kappa_i-\bar\kappa_i)/T_{x,i},\qquad
\dot{\bar\alpha}_i=(\alpha_i-\bar\alpha_i)/T_{y,i},
$$

with $T_x=0.3/\operatorname{clip}(|v_x^t|,10,100)$ and
$T_y=0.2/\operatorname{clip}(|v_x^t|,10,100)$. The reduced plants use the
instantaneous slips. Consequently, recorded `tire_slip_angle_rad` and
`tire_longitudinal_slip` mean **the slips entering each force law**, not
identical state definitions across the native and reduced models.

Steering in all plants has the same PT1 time constant and rate saturation:

$$
\dot\delta=\operatorname{clip}((\delta_{cmd}-\delta)/0.06,-0.5,0.5),
\qquad |\delta|\le0.5.
$$

Outward rates are suppressed at the steering limit; negative wheel angular
acceleration is suppressed at approximately zero wheel speed.

## Tire equations compared

Write the unit-peak Magic Formula shape as

$$
f(q;B,C,E)=\sin\{C\arctan[Bq-E(Bq-\arctan(Bq))]\}.
$$

| Tire choice | Longitudinal force | Lateral force | Combined-slip implication |
|---|---|---|---|
| Independent MF | $F_x=\mu F_z f(\kappa;B_x,C_x,E_x)$ | $F_y=\mu F_z f(\tan\alpha;B_y,C_y,E_y)$ | No coupling; the resultant can exceed $\mu F_z$ |
| Berntorp-style ellipse | Same $F_{x0}$ | $F_y=F_{y0}\sqrt{\max(0,1-(F_{x0}/(\mu F_z))^2)}$ | Preserves longitudinal force and reduces lateral capacity |
| Standard Fiala adaptation | Same $F_{x0}$ | Saturated brush polynomial below, using $Q=\sqrt{\max(0,(\mu F_z)^2-F_{x0}^2)}$ | Available lateral capacity decreases during braking/traction |

For Fiala, $C_\alpha=16\mu F_z$ and $q=C_\alpha\tan\alpha/(3Q)$:

$$
F_y=\begin{cases}
Q(3q-3q|q|+q^3),&|q|<1,\\
Q\operatorname{sign}(q),&|q|\ge1.
\end{cases}
$$

The implementation handles $Q=0$ without division by zero. Its small-slip
slope matches the normalized native MF. The extended Fiala thesis law adds
load-dependent stiffness and a post-peak branch, which are not reproduced by
this default. The optional load-sensitivity switch in the code is disabled
for this experiment and is not a calibration to Bertipaglia's measured car.

All default force laws scale linearly with normal load. Real tire load
sensitivity is not established by this normalization. Matching a pure-slip
peak and initial slope does not make different combined-slip surfaces equal.
See [the source-level comparison](LITERATURE.md) before using these variants
as a thesis reproduction or selecting a calibrated prediction model.

## Dynamic versus quasi-static normal loads

Native OCD integrates body and wheel vertical motion, roll/pitch, springs,
dampers, antiroll bars and jacking terms. Its exact equations, 33-state order
and equilibrium initialization are documented in the
[native specification](../../tracklab/comparison_native/README.md).

The reduced models use $W=mg+D_a$ and, with zero aero-center offset,

$$
F_{zf}=W\frac{l_r}{L}-\frac{h}{L}\sum_iF^b_{x,i},\qquad F_{zr}=W-F_{zf}.
$$

If $G_y=\sum_iF^b_{y,i}$ and $\eta_f=l_r/L$, lateral redistribution is

$$
F_{z,FL/FR}=\frac{F_{zf}}2\mp\frac{h\eta_fG_y}{t_f},\qquad
F_{z,RL/RR}=\frac{F_{zr}}2\mp\frac{h(1-\eta_f)G_y}{t_r}.
$$

This explicitly assigns front/rear roll-moment shares using the static axle
weight fractions; it does not reproduce the native suspension's dynamic roll
distribution. Forces and loads are algebraically coupled. Because the default
tire forces are homogeneous in $F_z$, the implementation solves their two
force-balance unknowns through an exact 2×2 linear system at each ODE stage.
The static ablation removes both acceleration-induced terms but retains
aerodynamic load. Contact-limit clipping is flagged as leaving the assumed
four-contact operating domain.

## Initial, boundary, path and actuator conditions

These terms describe different parts of the experiment and should not be
interchanged.

| Condition | Implemented value or equation | Role |
|---|---|---|
| Road geometry | Existing `data/Norisring.csv`; periodic cubic centerline, cyclic PCHIP widths; length **2296.3123672779016 m** | Common continuous track, used directly by MPC and diagnostics |
| Road periodicity | $c(0)=c(L)$ with periodic spline derivatives | Geometric boundary condition; it does not impose a periodic vehicle trajectory |
| Surface | Flat road, zero wind, per-wheel friction multiplier 1 | Assumed environment, not measured CSV surface information |
| Initial planar state | Centerline at $s=0$, tangent heading, $v_x=8$ m/s, $v_y=r=\delta=0$ | Identical common observations |
| Initial wheel speeds | $\omega_i=v^t_{x,i}/0.3$, zero initial longitudinal slip | Identical rolling wheel initial conditions |
| Native internal initialization | Solve body/wheel heave, roll and pitch equilibrium; zero corresponding velocities | Avoids arbitrary suspension settling; longitudinal force trim is not claimed |
| Lap termination | First interpolated crossing of $s=L$; one lap | An initial-value simulation; no $x(t_f)=x(0)$ constraint |
| Nominal speed schedule | Reference cap 18 m/s; planning lateral budget 3 m/s² | Reference construction, not a hard measured-speed/acceleration bound |
| Demanding speed schedule | Reference cap 30 m/s; planning lateral budget 6 m/s² | Same controller and vehicle, larger target speeds |
| Reference speed reachability | Forward/backward limits 1.5 m/s² acceleration, 2.0 m/s² braking | Common speed envelope; no minimum-time optimization |
| MPC initial equality | Predicted five-state vector equals current measured observation projection | Reinitialized at each 0.1 s control instant |
| MPC road constraint | $-w_R+2.4\le e_y\le w_L-2.4$ at predicted cross-sections | Hard predicted center corridor |
| MPC terminal condition | Same path bounds at terminal node; tracking penalty multiplied by 3 | No terminal set, invariant tube or periodic endpoint constraint |
| Steering input/state | $\lvert\delta_{cmd}\rvert,\lvert\delta\rvert\le0.5$ rad | Shared amplitude limits |
| Steering rate | Plant rate ≤0.5 rad/s; predicted PT1 slope bounded through $\lvert\delta_{cmd}-\delta\rvert\le0.03$ rad | Common actuator constraints |
| Requested gross force | $-4\le a_{cmd}\le2.5$ m/s²; $T_i=mR\eta_i a_{cmd}$ | Bounds requested torque-equivalent acceleration, not measured acceleration |
| Torque allocation | $\eta=[l_r/(2L),l_r/(2L),l_f/(2L),l_f/(2L)]$ | Identical allocation; no torque-vectoring optimizer |
| MPC speed domain | $1\le v\le\max(40,v_{ref,max}+10)$ m/s | Broad prediction-domain bounds, distinct from the reference cap |
| Tire-force constraint in MPC | None | Force ratio is an evaluated diagnostic, not an MPC constraint |
| Actual road check | 40 points on a 4.4 m × 1.9 m body perimeter every 0.02 s | Sampled containment evidence, independent of frozen MPC cross-sections |
| Numerical integration | Native fixed-step DoPri 0.0005 s; reduced RK4 0.001 s | Checked separately by halving the steps |

The CSV SHA-256 is
`8857d3c362ad2923c1f93c8d257498f50459770b9021adcc7969b71085c31d9a`.

## Common car and controller

Shared parameters: $m=800$ kg, $I_z=1000$ kg m², $l_f=1.724$ m,
$l_r=1.476$ m, $t_f=1.6$ m, $t_r=1.5$ m, $h=0.3$ m, wheel radius
0.3 m and front/rear wheel inertias 1.15/1.35 kg m². Aerodynamic settings
are $\rho=1.225$, $C_dA=1$, $C_lA=-1.5$; rolling resistance is 0.025.
MF shapes are $(B_x,C_x,E_x)=(20,2,0.7)$ and
$(B_y,C_y,E_y)=(10,1.6,-1.9)$. Native tire peaks are explicitly changed
from upstream $D=1.7$ to $D=1$, and steering magnitude from 0.3 to 0.5 rad.
This is a normalized demonstration, not a fitted original vehicle dataset.

The same five-state **kinematic tracking NMPC** predicts
$[X,Y,\psi,v,\delta]$, uses a 25-step horizon at 0.1 s, and solves with
CasADi/IPOPT. It includes steering lag and drag in prediction. The objective
weights, speed-envelope construction and input allocation are fixed across
plants. It does not use the four alternative plant equations as four different
prediction models. This isolates a plant comparison under a shared controller;
it does not compare the theses' MPC algorithms or reproduce their guarantees.

## Evidence and its limits

There are two complementary comparisons:

1. **Closed-loop full laps:** common controller and references, with tracking,
   slip, load, tire-force and sampled body-clearance metrics. Feedback produces
   different commands for different plants, so these results alone do not
   isolate open-loop model error.
2. **Held-input one-second predictions:** twelve fixed track-station windows
   per speed schedule, starting from the native planar and wheel states and
   replaying exactly its recorded controls. Native snapshot replay is checked
   separately; half-step refinement uses each model's own state and inputs.

The tests cover physical signs, load conservation, moment balance, saturation,
symmetry, initialization, state restoration and numerical refinement. The
existing road and Python-driving suites are also run. The separate legacy
single-track native tests require their own original native library build.

OCD is a reference simulator, not measured ground truth. A smaller deviation
from it may simply mean sharing its assumptions. In particular, retaining its
independent-slip law can improve agreement while omitting combined-slip tire
physics. Lap-time differences under prescribed tracking targets are not a
ranking of physical accuracy. CPU timings include implementation language,
integration method and host effects. The thesis table and full equations are
provided so these distinctions remain reviewable.
