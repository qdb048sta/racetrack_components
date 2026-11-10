# Doctoral literature and model provenance

This document adds five verified doctoral theses to the earlier project record and explains their relationship to the models executed in this branch. Sources were checked on **9 November 2026**. The new entries exclude the previously discussed Liniger, Gao, Beal, Hao Sun, Zhe Liu, Rowold, Reiter and Edward Liu Zhu theses.

**The simulations compare normalized model structures. They do not reproduce five thesis vehicles or five thesis controllers.** Native Open-Car-Dynamics (OCD) is a reference simulator; agreement with it is not experimental validation against a real car. The code uses one common tracking controller so changes in the simulated plant can be examined under the same controller design.

## 1. Additional PhD theses, newest first

| Year | Verified doctoral thesis | Relevance to double-track and tire modeling | MPC predictor versus validation plant | Evidence inspected |
|---|---|---|---|---|
| **2025** | **Alberto Bertipaglia**, *Control of Evasive Manoeuvres for Automated Driving: Solving the Edge Cases*, TU Delft [B] | Four-wheel planar model; longitudinal/lateral force coupling; load-dependent cornering stiffness; extended Fiala lateral tires. | The Chapter 4 double-track model is used **inside MPCC** for evasive maneuvers and torque vectoring. | University doctoral record; indexed thesis passages; complete associated author paper [BP]. The complete doctoral PDF could not be downloaded in this session. |
| **2023** | **Alexander Wischnewski**, *Robust and Data-Driven Control for Autonomous Racing*, TUM [W] | TUM autonomous-racing context; combined-slip Pacejka and nonlinear dual-track hardware-in-loop simulation. | The proposed **Tube-MPC uses a friction-limited point-mass predictor** with low-level acceleration control. Double-track/Pacejka is the validation plant. | Full doctoral PDF downloaded and inspected. |
| **2019** | **Matteo Veneri**, *Minimum-lap-time of race vehicles*, University of Padova [V] | Quasi-steady and transient double-track racing models; aerodynamic forces, Pacejka tires, lateral load transfer and steering geometry. | The verified record describes **minimum-time optimal control**. It does not establish a receding-horizon MPC implementation. | Official doctoral record and abstract. Full-text equations and numerical parameters were not inspected. |
| **2017** | **Nicola Dal Bianco**, *Optimal control of road vehicles: theory and applications*, University of Padova [D] | Multibody GP2/kart models, chassis/suspension motion and tire-load dynamics. The related GP2 paper describes 14 DOF [DP]. | Minimum-lap-time optimal control; no claim here that the doctoral model is used in online MPC. | Official doctoral record, indexed doctoral title page and related original paper abstract. Full thesis equations were not inspected. |
| **2014** | **Karl Berntorp**, *Particle Filtering and Optimal Control for Vehicles and Robots*, Lund University [K] | Detailed two-track derivation with roll/pitch, wheel spin, tire relaxation, Magic Formula and alternative combined-slip laws. | Chapter 10 compares models in optimal-control maneuvers; Chapter 11 develops hierarchical nonlinear MPC. Those are distinct experiments. | Full doctoral PDF downloaded and inspected, including model and parameter sections. |

The TU Delft record gives Bertipaglia's award date as **24 February 2025**. The filename `Bertipaglia_2024_PhD_Dissertation.pdf` is not its publication year. Veneri's doctoral record is also distinct from his similarly named earlier **master's** thesis.

For this project, Wischnewski is the closest TUM application reference, Berntorp provides the clearest mathematical comparison methodology, and Bertipaglia offers the strongest recent alternative for a double-track MPC prediction model. This is an assessment of fit to the project, not a ranking of experimentally demonstrated accuracy.

## 2. Equations and constraints: what is verified

An ODE simulation has an **initial condition**. An optimal-control problem can additionally impose terminal, road and actuator constraints. These should not all be called the car model's “boundary conditions”: a road boundary belongs to the driving problem, while a tire-force limit belongs to its physical model or admissible operating set.

| Source/model | Dynamic equations and tire closure | Initial/terminal conditions | Path and input constraints | Qualification |
|---|---|---|---|---|
| **Bertipaglia double-track MPCC** | Six planar mechanical states plus progress, steering and four longitudinal-force integrators: 12 prediction states in [BP]. Four separate wheel forces; extended Fiala, remaining lateral capacity and load-dependent stiffness. Thesis §4.3; [BP] Eqs. (1)–(8). | Receding-horizon prediction from the measured state. A periodic racing-lap terminal equality is not established by the inspected material. | Thesis p73, §4.4.2: actuator magnitudes/rates, friction safety factor (4.14), road boundary (4.15), limits on redundant left/right force differences (4.16). | No dynamic roll/pitch. This is an obstacle-avoidance study, not the Norisring experiment. |
| **Wischnewski Tube-MPC and HiL plant** | Predictor: friction-limited point-mass dynamics plus acceleration loops. Validation: dual-track, longitudinal/lateral/combined Pacejka, actuator and sensor models. Tire exposition: (2.11)–(2.12), p21. | MPC recedes from the current estimate. No claim of a periodic lap terminal equality. | Tube-MPC tightened corridor/acceleration constraints; the appended HiL study uses $d_{\max}=1\,\mathrm m$ and $a_{x,\max}=a_{y,\max}=12.5\,\mathrm{m/s^2}$. | Source-study settings are not the branch's benchmark settings. This earlier plant is not proven byte-identical to the pinned OCD implementation. |
| **Veneri double-track optimal control** | University record verifies a quasi-steady model and a transient extension, Pacejka, aero and lateral load transfer. | Not extracted from full text. | A fixed brake ratio is reported; exact path/input/endpoint formulas were not inspected. | Literature comparison only; no equation-level reproduction claimed. |
| **Dal Bianco multibody racing model** | University record verifies suspension/chassis/aero modeling and tire-load dynamics. Related GP2 model includes chassis motion, suspension travel and wheel spin [DP]. | Not extracted from full text. | Not extracted from full text. | Literature comparison only; the 14-DOF count is supported by the related paper, not an inspection of every thesis model. |
| **Berntorp two-track optimal control** | Chassis (4.22)–(4.27), pp77–79; Magic Formula (4.8), p69; combined-slip ellipse (4.9), p70, or weighting functions (4.11), p71; wheel spin and relaxation (10.2), p207. | (10.6), p212: prescribed initial state and a selected subset of terminal-state conditions; final time minimized. | Road set $\Gamma(X,Y)\le0$; steering/torque magnitudes and rates. Flat road and continuous wheel contact. Steering limits in that study: 30° and 60°/s; rear-drive torque limits (10.7), p215. | Full two-track model includes roll/pitch. The branch borrows a tire closure while using a reduced chassis. |

### Exact page conventions

| Source | Confirmed location | Printed page / PDF page |
|---|---|---|
| Berntorp | Pure-slip MF (4.8) | 69 / 73 |
| Berntorp | Friction ellipse (4.9), weighting functions (4.11) | 70–71 / 74–75 |
| Berntorp | Two-track derivation (4.22)–(4.27) | 77–79 / 81–83 |
| Berntorp | Relaxation (10.2), load transfer (10.4) | 207–208 / 211–212 |
| Berntorp | Model calibration for comparison | 209–211 / 213–215 |
| Berntorp | Optimal-control constraints (10.6) | 212 / 216 |
| Berntorp | Chassis/tire Tables A.1–A.3 | 324–325 / 328–329 |
| Wischnewski | Single-track exposition (2.6) | 18 / 21 |
| Wischnewski | MF (2.11), combined-force limit (2.12) | 21 / 24 |
| Wischnewski | Appended Tube-MPC paper, §4.1, HiL plant | Appended paper p12 / thesis PDF p97 |
| Bertipaglia | Table 4.2; (4.6)–(4.8) | Thesis p69; PDF index not independently established |
| Bertipaglia | §4.4.2; (4.14)–(4.16) | Thesis p73; PDF index not independently established |

No exact equation or page references are assigned to the unread Veneri and Dal Bianco thesis text.

## 3. The implemented model families

The implementation is in [models.py](../../tracklab/model_comparison/models.py), with the native adapter in [vehicle.py](../../tracklab/comparison_native/vehicle.py). All executed variants share the benchmark's declared mass **800 kg**, yaw inertia **1000 kg m²**, wheelbase **3.2 m**, road, friction normalization and controller. These values are not the thesis calibration datasets.

| Executed model | States and load model | Tire law | Relationship to literature |
|---|---|---|---|
| `native_ocd` | Native 33-state OCD system | Native `MF_SIMPLE`, with explicitly normalized peak coefficients | Actual pinned native source; reference simulator |
| `mf_independent` | 11 states; planar chassis, four wheel speeds, steering; quasi-static load transfer | Independent longitudinal/lateral MF laws | Reduced OCD-family comparison; no combined-force ellipse enforced |
| `mf_ellipse` | Same 11-state structure | Same pure-slip MF, then Berntorp (4.9) lateral reduction | **Exact (4.9) closure applied to normalized benchmark forces**, not a complete Berntorp-model reproduction |
| `fiala` | Same 11-state structure | MF longitudinal force plus classic saturated Fiala lateral force and remaining circle capacity | Normalized Fiala-family adaptation motivated by Bertipaglia; **not his extended Fiala model** |
| `mf_static` | Same 11 states, but no acceleration-induced load transfer; aero load retained | Same independent MF | Ablation isolating the chosen quasi-static transfer mechanism |

The reduced state is

$$
z=[X,Y,\psi,v_x,v_y,r,\omega_{FL},\omega_{FR},\omega_{RL},\omega_{RR},\delta]^\top.
$$

Its planar mechanical core is

$$
\begin{aligned}
\dot X&=v_x\cos\psi-v_y\sin\psi,&
\dot Y&=v_x\sin\psi+v_y\cos\psi,&\dot\psi&=r,\\
\dot v_x&=\frac{\sum_i F^b_{x,i}+F_{x,\mathrm{aero}}}{m}+rv_y,&
\dot v_y&=\frac{\sum_i F^b_{y,i}+F_{y,\mathrm{aero}}}{m}-rv_x,\\
\dot r&=\frac{\sum_i(x_iF^b_{y,i}-y_iF^b_{x,i})}{I_z}.
\end{aligned}
$$

Here body-frame wheel forces are obtained by rotating tire-frame forces through each wheel's steering angle. Wheel spin follows applied torque minus tire and rolling-resistance torques, divided by wheel inertia with the native-style low-speed inertia regularization retained. Steering is a bounded, rate-limited first-order actuator. The reduced models omit dynamic suspension, roll/pitch and tire-relaxation states. Their quasi-static roll-moment allocation follows static axle weight fractions, an explicit benchmark assumption rather than an identified anti-roll-bar distribution.

### Magic Formula and Berntorp's combined-slip alternatives

Define the unit force shape

$$
\mathcal M(q;B,C,E)=
\sin\!\left[C\arctan\!\left(Bq-E(Bq-\arctan Bq)\right)\right].
$$

Berntorp (4.8) uses $F_0=D\mathcal M(q)$, $D=\mu F_z$. The benchmark keeps OCD's lateral argument convention $q=\tan\alpha$, longitudinal regularization and shape coefficients. Consequently, the overall MF implementation is an OCD-normalized variant even when its combined-slip closure is exactly (4.9):

$$
F_y=F_{y0}\sqrt{\max\{0,1-(F_x/(\mu F_z))^2\}}.
$$

This closure gives longitudinal force priority. It does not model the reduction in longitudinal force caused by lateral slip. Berntorp also presents the more coupled weighting law (4.11):

$$
\begin{aligned}
F_x&=F_{x0}\cos\!\left[C_\alpha\arctan(H_\alpha\alpha)\right],
&H_\alpha&=B_{\alpha1}\cos\!\left[\arctan(B_{\alpha2}\kappa)\right],\\
F_y&=F_{y0}\cos\!\left[C_\kappa\arctan(H_\kappa\kappa)\right],
&H_\kappa&=B_{\kappa1}\cos\!\left[\arctan(B_{\kappa2}\alpha)\right].
\end{aligned}
$$

The weighting law and the thesis's lateral relaxation equation

$$
\frac{\sigma}{v_x}\dot\alpha+\alpha=-\arctan(v_y/v_x)
\qquad\text{(10.2)}
$$

are documented comparison possibilities; they are **not implemented in the reduced runs**. Native OCD retains its own source-defined transient states.

### Standard Fiala versus Bertipaglia's extension

The benchmark uses the sign convention in which positive slip angle produces positive lateral force. With

$$
Q=\sqrt{\max\{0,(\mu F_z)^2-F_x^2\}},\qquad
C_\alpha=\mu B_yC_yF_z,\qquad
q=\frac{C_\alpha\tan\alpha}{3Q},
$$

its classic saturated law is

$$
F_y=\begin{cases}
Q(3q-3q|q|+q^3),&|q|<1,\\
Q\operatorname{sign}(q),&|q|\ge1,
\end{cases}
$$

with $F_y=0$ when $Q=0$. This matches the normalized MF's small-slip lateral stiffness and force capacity. It does not make the complete force-slip curves equal. Using a threshold on $\tan\alpha$ makes the piecewise transition explicit.

Bertipaglia's extension additionally changes stiffness with normal and longitudinal force, and modifies the post-peak branch. The source's load dependence is

$$
C_y(F_z)=c_1F_{z0}\sin\!\left[2\arctan\!\left(\frac{F_z}{c_2F_{z0}}\right)\right].
$$

The dissertation's (4.6) is

$$
C_{ym}=\tfrac12(\mu F_z-F_x)+
\left[1-\left(\frac{|F_x|}{\mu F_z}\right)^{c_3}\right]^{1/c_3}
\left[C_y(F_z)-\tfrac12\mu F_z\right].
$$

Table 4.2 gives $c_1=49.3,c_2=3.5,c_3=4.1,\zeta=0.87,F_{z0}=4300\,\mathrm N,\mu=0.95$. These fitted values and the extended post-peak law are **not the default benchmark Fiala calibration**. An optional load-sensitivity implementation exists in the code but is disabled in the comparison runs. The earlier arXiv HTML prints an inconsistent exponent in its analogous stiffness equation; this document follows the dissertation's $1/c_3$. The code derives yaw moments from wheel position and force vectors rather than copying ambiguous expanded component expressions.

## 4. Reading the simulation results correctly

1. **Plant comparison is not predictor selection by itself.** Every plant is driven by the same five-state kinematic tracking NMPC. A tracking failure can reveal a controller–plant mismatch; it does not prove that a vehicle model is physically inaccurate. The branch does not reproduce the theses' MPCC, Tube-MPC or optimal-lap algorithms.
2. **Native OCD is not measured ground truth.** Lower trajectory discrepancy to OCD measures agreement with the chosen simulator. Establishing physical accuracy requires vehicle measurements, identified parameters and independent maneuvers.
3. **A common controller is not a common input trace.** Feedback produces different steering/torque sequences for different plants. Closed-loop lap metrics should be complemented by held-input or short-horizon replay checks when attributing differences specifically to equations.
4. **Combined-slip assumptions matter.** Independent pure-slip forces can exceed a circular force envelope. Report this as a model property and a circle-utilization diagnostic, not an optimization-constraint failure where no such constraint was imposed. An actual tire may have a noncircular measured envelope.
5. **Matching basic parameters does not remove every modeling difference.** Load-transfer dynamics, roll-moment distribution, slip regularization and relaxation remain different. The static-load ablation removes one selected mechanism; it does not represent every quasi-steady thesis model.
6. **The nominal lap has limited identification power.** Small-slip running can make several nonlinear tires look alike. Higher-demand runs and combined braking/cornering are more informative, while still being limited by the common controller and the declared force/steering bounds.
7. **Runtimes and boundaries have practical limits.** Native C++ versus Python timing includes implementation and host effects; it is not a like-for-like MPC solver complexity benchmark. Logged perimeter-clearance samples support sampled road containment, not a mathematical guarantee between samples. The simulation terminates at a lap crossing; it does not solve a periodic-state boundary-value problem.

## 5. Primary bibliography

- **[B]** Bertipaglia, A. (2025). *Control of Evasive Manoeuvres for Automated Driving: Solving the Edge Cases*. Doctoral dissertation, TU Delft. [University record](https://research.tudelft.nl/en/publications/control-of-evasive-manoeuvres-for-automated-driving-solving-the-e/) · [DOI](https://doi.org/10.4233/uuid:05ba52a6-72ed-4259-9541-bf2b09c9a8b6) · [Doctoral PDF](https://research.tudelft.nl/files/235083711/Bertipaglia_2024_PhD_Dissertation.pdf).
- **[BP]** Bertipaglia, A., et al. (2024). *Model Predictive Contouring Control for Vehicle Obstacle Avoidance at the Limit of Handling Using Torque Vectoring*. [Author preprint and equations](https://arxiv.org/html/2405.10847v1). Associated paper supporting [B]; not a separate thesis.
- **[W]** Wischnewski, A. (2023). *Robust and Data-Driven Control for Autonomous Racing*. Doctoral dissertation, TUM. [University PDF](https://mediatum.ub.tum.de/doc/1693646/zgtie2ycz2g2af9xfi7q0zsfq.DissertationWischnewskiRobustAndDataDrivenControl.pdf). Submitted 22 December 2022; accepted 2 May 2023.
- **[V]** Veneri, M. (2019). *Minimum-lap-time of race vehicles*. Doctoral dissertation, University of Padova. [University doctoral record](https://www.research.unipd.it/handle/11577/3425794). Defence date: 30 September 2019.
- **[D]** Dal Bianco, N. (2017). *Optimal control of road vehicles: theory and applications*. Doctoral dissertation, University of Padova. [University doctoral record](https://www.research.unipd.it/handle/11577/3424690) · [Doctoral PDF](https://www.research.unipd.it/retrieve/e14fb26f-deea-3de1-e053-1705fe0ac030/Dal_Bianco_Nicola_tesi.pdf). Date: 31 October 2017.
- **[DP]** Dal Bianco, N., et al. *Minimum time optimal control simulation of a GP2 race car*. [Original paper](https://doi.org/10.1177/0954407017728158). Used only for the related model's explicit 14-DOF description.
- **[K]** Berntorp, K. (2014). *Particle Filtering and Optimal Control for Vehicles and Robots*. Doctoral thesis TFRT-1101, Lund University. [University doctoral record](https://portal.research.lu.se/en/publications/particle-filtering-and-optimal-control-for-vehicles-and-robots/) · [Full doctoral PDF](https://lucris.lub.lu.se/ws/files/3352295/4407278.pdf). Defence date: 23 May 2014.
- **[OCD]** TUMFTM. *Open-Car-Dynamics*, pinned revision [94f8fb187fb0ed22bba1d809bd74f66d1ff75af4](https://github.com/TUMFTM/Open-Car-Dynamics/tree/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4). Its source is authoritative for the native model executed here.

Thesis PDFs are not redistributed in this repository.
