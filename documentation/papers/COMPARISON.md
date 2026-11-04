# Four thesis models compared with racetrack_components

Baseline: `master` at `e67066071572d9dbda95604239cc7a43d9615208`.
The four branches start from that same commit. Each contains one independently
runnable paper experiment, its tests, and an equation-to-code analysis. These are
new implementations of selected mathematical models, not the authors' software
or reproductions of their experimental results. The original PDFs are not copied
into this public repository.

## What each source actually studies

| Source | Vehicle models | Tire models | MPC formulations | Executable branch scope |
|---|---|---|---|---|
| Craig E. Beal, *Applications of Model Predictive Control to Vehicle Dynamics for Active Safety and Stability*, Stanford, 2011 | Four-wheel yaw-roll simulation; constant-speed bicycle; affine force-input (AFI) prediction; linearized roll extension | Two-coefficient nonlinear brush; combined slip; friction-circle derating; linear tires for driver intent/estimation | Convex envelope-control MPC using front lateral force, rear tire linearization, soft yaw/slip limits and inversion back to steering; later roll-control extension | Brush force/inverse, bicycle, AFI/Tustin QP, delayed steering envelope control on this road |
| Yiqi Gao, *Model Predictive Control for Autonomous and Semiautonomous Vehicles*, Berkeley, 2014 | Ten-state four-wheel plant; six-state four-wheel predictor; nonlinear/linear bicycle; point mass; spatial bicycle; center-of-percussion reduction | Pacejka including combined slip in experimental models; modified Fiala as simpler nonlinear option; linear option | One-level and hierarchical NMPC; clothoid planning; spatial high-level and temporal low-level MPC; tube-based robust NMPC | Chapter 4 hierarchy using a spatial bicycle and temporal four-wheel chassis, with disclosed Fiala/actuation/solver adaptations |
| Hao Sun, *Model Predictive Control for Future Interconnected and Green Vehicles*, UCL, 2025 | Longitudinal force balance; spatial kinetic-energy/time-headway model; five-state kinematic bicycle for 2D coordination | No lateral tire-force law in the kinematic bicycle. Longitudinal tire effects are rolling resistance, wheel radius and torque-to-force conversion | Learned ecological following MPC; tube-based distributed platoon MPC; APF planner plus distributed kinematic tracking MPC | Nominal spatial following with relaxation diagnostics/physical fallback; kinematic tracking with supplied road/neighbor references |
| Zhe Liu, *A novel ring model for tire dynamics and its application to uncertainty quantification*, TUM, accepted 2023 | Tire/rim mechanical system, not a full car model | Extensible 3D flexible ring; coupled rigid-flexible ring/contact model; longitudinal force relaxation | No MPC algorithm. Instead: modal analysis, contact solve, time integration, parameter identification, generalized polynomial chaos/PCM | Nonrotating in-plane modal matrices/dynamics; rigid-ring ODE; relaxation; uniform-parameter Legendre PCM |

The publication years come from the title pages, not search-engine crawl dates.
Beal and Gao are older than the MPC/Gerdts book editions previously discussed.
Newer does not mean a more detailed lateral handling model: Sun's Chapter 5
vehicle is deliberately simpler than this repository's dynamic bicycle.

## Baseline implementation

| Repository part | Existing formulation | Consequence for swapping models |
|---|---|---|
| `tracklab/driving/vehicle.py` | Seven states `[X,Y,psi,vx,vy,r,delta]`; static axle loads, load-scaled tanh lateral force, combined-force cap, drag, steering lag/rate limit | A new algebraic tire can fit `lateral(alpha,load,fx,mu,ops)`. The sign is opposite to the Beal/Gao slip convention; map it explicitly. |
| `tracklab/driving/controller.py` | Tracking NMPC; fixed seven-state allocation, two controls, symbolic dynamics, RK4 prediction | Beal's force-input QP and Sun's five-state kinematics require new controller layouts, not only a changed tire formula. |
| `tracklab/driving_OCD/native/bridge.cpp` | Native single-track body, four wheel-speed states, steering state: eleven total; selected `Linear` tire, wheel-torque drivetrain and PT1 steering | The tire class is interchangeable in native templates, but the Python bridge/controller state mapping and symbolic predictor remain specialized. |
| `tracklab/driving_OCD/vehicle.py` and `controller.py` | Separate CasADi transcription, eleven-state prediction, implicit Radau collocation | A native tire change alone does not change MPC's model. |
| `PeriodicTrack` | CSV center/width data, periodic interpolation, arc-length geometry, closest-point projection, independent surface sampling | Reused directly by Beal/Gao/Sun demonstrations. It is not itself a chassis or tire law. Flat CSV road data do not provide Liu's contact patch, cleat profile, or calibrated elastic parameters. |

The existing Python vehicle defaults are 800 kg, 1000 kg m^2 yaw inertia,
3.2 m wheelbase and nominal friction 1.0. New handling examples use explicit,
illustrative 1500 kg/2500 kg m^2/2.7 m/0.7 settings, shared between Beal/Gao.
They are **not matched-parameter performance benchmarks** against the baseline.
Sun's geometric model and Liu's material parameters answer different questions.
Use the dataclasses to establish a common calibration before comparing performance.

## What 'solving the tire model' means here

For the baseline, Beal and Gao, a steady tire law is evaluated algebraically at
each ODE stage. The vehicle force/moment equations are integrated over time.
Beal additionally solves an inverse force-to-slip mapping and a convex finite
horizon optimization problem. Gao's spatial model divides time derivatives by
road progress `ds/dt`; its optimization is advanced in metres, then interpolated
back to time for the tracker.

Sun's kinematic bicycle contains no tire-force evaluation. Its geometric beta is
not a dynamic tire slip state. In the longitudinal model, differentiating
`E=m*v^2/2` with respect to road station makes `dE/ds` equal to net force. The
reciprocal-speed/headway equality is the nonlinear part targeted by convex
relaxation.

Liu has actual tire deformation and force-relaxation states. Spatial vibration
equations can be reduced to modal ODEs, while steady contact is a separate
algebraic/iterative computation. The full coupled model repeatedly exchanges
contact forces and ring/rim positions. It cannot fit the current stateless
`TireModelBase.evaluate(slip,alpha,Fz,gamma)` contract without extra states,
terrain/contact geometry and a vehicle/suspension adapter.

## How to compare branches

Install the existing `requirements-driving.txt` (Python 3.12 and CasADi 3.7.2
were used for validation). Check out one `paper/...` branch and run its root
`run_paper_*.py` script. Its `--help` describes arguments. Every run writes
`trajectory.csv`, `summary.json` and `trajectory.png` under ignored `outputs/`.
Liu also writes `modes.csv`.

The driving runners are explicit composition points: construct `PeriodicTrack`,
the paper vehicle, the paper controller, then repeatedly solve and propagate.
They do not route new states into the existing fixed-dimension `TrackingMPC`.
Both old launchers and the native bridge remain unchanged.

Tests check physical signs, force saturation, coordinate transformations,
independent numerical integrations and solver constraints. Smoke runs are short
experiments, not full-lap certification or demonstrations of recursive feasibility.
An optimizer success flag alone is not a control-theoretic guarantee. Native OCD
was not rebuilt because none of its code is changed by these branches.

## Sources

- Beal: <https://purl.stanford.edu/rv794bn8004>
- Gao: <https://escholarship.org/content/qt8xd0b56h/qt8xd0b56h_noSplash_6469c4e93fb947f62914e1d03570224f.pdf>
- Sun: <https://discovery.ucl.ac.uk/id/eprint/10203332/1/Sun_10203332_Thesis.pdf>
- Liu: <https://mediatum.ub.tum.de/doc/1690000/1690000.pdf>

Each branch's named analysis gives printed/PDF page numbers, equation identifiers,
source-file SHA-256, implemented components, and omitted contributions.
