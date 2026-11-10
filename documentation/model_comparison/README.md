<!-- Generated from canonical equation source by tools/render_vehicle_reference.py -->

Editable equation source: [EQUATIONS.md](EQUATIONS.md). Equations are self-contained SVG images so they also display inside GitHub tables. Each image has a white background for legibility in light and dark themes.

# Vehicle and tire models: equations, source audit, and Norisring comparison

**Case Study - updated 11 November 2026.** This reference checks the exact native TUM Open-Car-Dynamics composition against the supplied internal report, the supplied Lienkamp slides, and the five doctoral theses in the screenshot. It publishes the mathematical equations, initial and operating conditions, executable adaptations, and fresh simulations.

## Direct answers

**The native OCD double-track model is not the same complete model as the internal report.** Both represent four contact patches and share planar Newton-Euler balances. The report has six planar states and an exponential lateral tire characteristic; the selected native OCD composition has 33 continuous states, dynamic suspension and normal loads, four wheel-spin states, eight relaxed slips, PT1 steering, and MF_SIMPLE tires.

**The uploaded Lienkamp excerpt does not establish an identical OCD model.** Its 90 slides cover chapter 2.1: a hierarchy of single-track models. Double-track chapters appear in the agenda but their content is absent. The internal report cites Lienkamp; that is a source relationship, not proof that every state, tire law, load closure, or parameter is identical. Claims about the professor's other implementations would require those implementations.

The native composition is pinned to `94f8fb187fb0ed22bba1d809bd74f66d1ff75af4`:

`WHEEL_TORQUE / PT1 / DOUBLE_TRACK / MF_SIMPLE / DEFAULT`.

This is distinct from the repository's older `run_drive_OCD.py`, which selects a native single-track model with linear tires. To run the double-track/thesis comparison, use `run_thesis_comparison.py`.

## What is compared

| Selected model | Executed states | Car and tire structure | Source fidelity and limits |
| --- | ---: | --- | --- |
| Native OCD reference | 33 | Four wheels, suspension, body heave/roll/pitch, unsprung motion, relaxed slips, MF_SIMPLE | Actual pinned C++ component equations; nine explicit parameter normalizations |
| Liniger 2018 | 7 | Nonlinear bicycle; simplified lateral Pacejka; rear longitudinal force | Six original states plus steering actuator; common full-size calibration replaces RC-car fit |
| Gao 2014 | 11 | Planar four-wheel car; four wheel rates; static loads; published pure-slip MF | Ten original states plus steering actuator; the unprinted combined-slip implementation is not reproduced |
| Beal 2011 | 13 | Nonlinear yaw-roll chassis; coupled brush tires; four wheel rates | Common-car roll/load/drivetrain closures explicitly supplied; not calibrated P1 hardware |
| Sun 2025 | 5 | CG kinematic bicycle with speed and steering states | No lateral tire force or wheel-slip dynamics; original steering-rate control receives a shared actuator map |
| Lienkamp supplied slides | 7 | Scheduled linear bicycle lateral/yaw model | Basic two-state lateral model augmented with pose, longitudinal motion, and steering for laps |
| Liu 2023 | N/A for a whole car | Structural tire-ring PDEs and rigid/flexible contact subsystem | Equations documented; no unsupported whole-car lap or handling-controller runtime |
| Internal report excerpt | N/A for exact reproduction | Planar double-track with linear or exponential lateral tire option | Load closure, sliding definition, calibration, and referenced code are missing |

The comparison changes the **simulation plant** while using one fixed tracking MPC design. Feedback generates different commands for different plants. Separate identical-command prediction windows therefore measure short-horizon model disagreement. Neither test establishes experimental accuracy against a real vehicle.

The earlier three-book comparison concerned different mathematical layers: Gerdts supplies the optimal-control/ODE-DAE framework; Grune and Pannek the receding-horizon formulation and analysis; Nocedal and Wright numerical optimization methods. Here the changing object is chiefly the physical right-hand side and its force closures. The thesis authors' complete controllers, costs, estimators, terminal guarantees, and experimental vehicles are not being compared as implementations of one algorithm.

## Files and reproduction

The repository README uses locally rendered equation images so display math and table formulas do not depend on GitHub math parsing. The exact editable mathematical source is `EQUATIONS.md`; `equations/equation_manifest.json` maps each image to its LaTeX expression. Original comparison material is retained in `PLANAR_COMPARISON.md`, `LITERATURE.md`, and `recorded/`.

```bash
python -m pip install -r requirements-driving.txt
python -m tracklab.comparison_native.build_native
python run_thesis_comparison.py --scenario both
python -m tracklab.model_comparison.thesis_validation --output outputs/thesis_comparison
python -m tracklab.model_comparison.thesis_report --input outputs/thesis_comparison
```

Run on Linux/WSL with a C++20 compiler and Eigen headers. See the repository setup guide for the environment used by the existing project. Raw data, dependencies and compiled binaries are regenerated locally; compact recorded results, CSVs, provenance, figures and replay are versioned. Source PDFs and screenshots are not uploaded to the public repository.



## Fresh performance comparison

The following results were generated from twelve complete laps in this update. Every run completed with zero failed MPC solves and zero sampled body points outside the road. Minimum sampled body clearance over all runs was 3.535 m. All models follow a prescribed speed profile, so small lap-time differences should not be interpreted as minimum-lap-time performance or proof of physical accuracy.

### Nominal: speed cap 18 m/s, lateral planning budget 3 m/s squared

| Model | Lap time (s) | RMS lateral error (m) | Peak lateral error (m) | Peak contact force ratio |
| --- | ---: | ---: | ---: | ---: |
| Native OCD | 156.332 | 0.0214 | 0.2072 | 0.8841 |
| Liniger bicycle | 156.815 | 0.0172 | 0.1235 | N/A |
| Gao four-wheel | 156.246 | 0.0234 | 0.2331 | 0.9431 |
| Beal yaw-roll | 156.359 | 0.0247 | 0.2389 | 0.7945 |
| Sun kinematic | 156.748 | 0.0007 | 0.0094 | N/A |
| Lienkamp linear | 156.654 | 0.0188 | 0.1520 | N/A |

![Nominal trajectories and signals](recorded_theses/nominal_comparison.png)

### Demanding: speed cap 30 m/s, lateral planning budget 6 m/s squared

| Model | Lap time (s) | RMS lateral error (m) | Peak lateral error (m) | Peak contact force ratio |
| --- | ---: | ---: | ---: | ---: |
| Native OCD | 120.802 | 0.0658 | 0.4473 | 1.0078 |
| Liniger bicycle | 121.291 | 0.0574 | 0.4128 | N/A |
| Gao four-wheel | 120.568 | 0.0833 | 0.5946 | 1.0138 |
| Beal yaw-roll | 121.030 | 0.0905 | 0.5812 | 0.9694 |
| Sun kinematic | 120.938 | 0.0034 | 0.0454 | N/A |
| Lienkamp linear | 120.957 | 0.0539 | 0.4289 | N/A |

![Demanding trajectories and signals](recorded_theses/demanding_comparison.png)

### Identical-command one-second predictions

Twelve predetermined station windows per scenario replay the same native commands without reinitializing within a window. All represented states are copied; Beal converts native CG velocities to its roll-base velocities. Sun necessarily projects the initial motion onto its kinematic manifold. Endpoint position RMSE is the square root of the mean squared Euclidean endpoint error across the twelve windows.

| Model | Nominal endpoint RMSE (m) | Demanding endpoint RMSE (m) | Demanding worst endpoint (m) |
| --- | ---: | ---: | ---: |
| Liniger bicycle | 0.033534 | 0.051566 | 0.061032 |
| Gao four-wheel | 0.009113 | 0.017329 | 0.047456 |
| Beal yaw-roll | 0.011628 | 0.035565 | 0.120858 |
| Sun kinematic | 0.087134 | 0.222911 | 0.572959 |
| Lienkamp linear | 0.094138 | 0.161473 | 0.532203 |

All 120 prediction windows and 36 integration-refinement windows succeeded. Maximum same-step full-state replay position error was 9.342e-12 m. Halving the integration step changed one-second endpoints by at most 9.205e-6 m for native OCD and 5.309e-9 m for the adapters. Refinement includes each model's strongest recorded yaw-rate window. These are local numerical checks, not a global integration-error bound.

### Measured computational cost

The units below are milliseconds of elapsed plant computation per simulated second, including integration. A smaller number is faster in this implementation. The native model is compiled C++; adapters are Python and use a different integrator, so this is not a language-independent complexity ranking. MPC timings use the same predictor/solver and are not the original theses' solver benchmarks.

| Model | Nominal plant ms/s | Demanding plant ms/s | Demanding median MPC solve (ms) |
| --- | ---: | ---: | ---: |
| Native OCD | 9.07 | 8.80 | 9.78 |
| Liniger bicycle | 50.48 | 49.10 | 9.71 |
| Gao four-wheel | 157.52 | 156.39 | 10.35 |
| Beal yaw-roll | 328.33 | 321.62 | 10.43 |
| Sun kinematic | 26.90 | 26.75 | 9.33 |
| Lienkamp linear | 48.60 | 48.21 | 9.77 |

### What the results support

Gao's normalized four-wheel/pure-MF adapter has the smallest short-horizon position mismatch to the selected native OCD reference in these windows. That is consistent with their related planar four-wheel/MF structure, but it is not an experimental accuracy result. Beal explicitly couples longitudinal/lateral force demand; its higher discrepancy includes different tires, load closure and roll assumptions.

Sun has the smallest closed-loop tracking errors because its kinematic equations closely match the common MPC predictor. It cannot establish correct handling-limit tire behavior: it has no tire force or saturation law. Lienkamp's linear tires also have no force saturation. Neither receives a physical contact-force utilization score. Liniger's source-style rear force includes resistance, so its effective net axle-force ratio is retained separately in machine-readable summaries and is N/A in the common contact-force column.

Independent MF_SIMPLE does not enforce a friction circle. The native ratio of 1.0078 and Gao ratio of 1.0138 are therefore expected possible outputs of those independent force closures, not numerical violations of an implemented friction-circle constraint. Beal's normalized brush supplies a coupled peak and stays below it in this run.

For the Case Study architecture, the results support retaining native OCD as the detailed reference plant and using a smaller dynamic model inside MPC when model evaluation cost matters. A Liniger-type dynamic bicycle is a useful starting structure for steering/speed MPC; wheel-specific braking or torque allocation requires separate wheel forces. Choosing a combined-slip law, fitting parameters, and validating against vehicle measurements remain separate requirements before calling any candidate the most accurate physical model.

The [recorded results](recorded_theses/RESULTS.md), [CSV](recorded_theses/summary.csv), [validation](recorded_theses/validation/summary.json), [provenance](recorded_theses/experiment_provenance.json) and [offline replay](recorded_theses/replay.html) make these claims reviewable. Download the HTML replay and open it locally to animate the saved trajectories.


## Conditions and exact benchmark formulation

### Track, initial state, and lap event

All plants use the existing `data/Norisring.csv`, the same periodic cubic centerline, cyclic PCHIP widths, and arc-length parameterization. Track length is 2296.3123672779016 m. The CSV does not measure friction, banking, road height, or tire parameters. The benchmark uses a flat road and unit per-wheel friction multipliers.

The common initial observation is centerline position at station zero, tangent heading, longitudinal speed 8 m/s, zero lateral speed/yaw rate/steering. Four-wheel plants initialize rolling wheel rates; native OCD solves its seven vertical/roll/pitch equilibrium equations. Beal starts with zero roll/rate; its state velocities are converted from the common CG observation. Sun's geometric sideslip is zero at the straight initial steering.

<p align="center"><img src="equations/eq_51c333a4fed94afc.svg" width="554" alt="(X_0,Y_0)=c(0),\quad \psi_0=\arg c&#x27;(0),\quad (v_{x0},v_{y0},r_0,\delta_0)=(8,0,0,0),\qquad \omega_{i0}=8/0.3."></p>

The last equation applies only to models that contain wheel spin. Native initialization and the remaining internal coordinates are defined in its full-equation section.

Road periodicity is a geometric condition. Completing the lap is a first crossing of the unwrapped progress event, linearly interpolated between the last two 20 ms observations:

<p align="center"><img src="equations/eq_e0b0d783364e77be.svg" width="281" alt="c(s+L_{\rm tr})=c(s),\qquad s(T)=s(0)+L_{\rm tr}."></p>

There is no imposed equality of terminal and initial vehicle states, and there is no minimum-time optimization in these runs. A periodic-lap optimal-control problem would require additional endpoint equalities, with yaw understood modulo a full turn.

The Frenet coordinates used for projection satisfy

<p align="center"><img src="equations/eq_e46f3dff8133aa7c.svg" width="488" alt="\dot s=\frac{v_x\cos e_\psi-v_y\sin e_\psi}{1-\kappa(s)e_y},\quad \dot e_y=v_x\sin e_\psi+v_y\cos e_\psi,\quad \dot e_\psi=r-\kappa(s)\dot s."></p>

The map requires a locally unique projection and a nonzero denominator. The simulation evolves global Cartesian states; the road-coordinate equations explain the projection geometry and are not extra integrated vehicle states.

### Common speed schedule

The nominal target has an 18 m/s cap and a 3 m/s squared lateral planning budget. The demanding target has a 30 m/s cap and a 6 m/s squared budget. These are reference-generation settings, not enforced physical tire limits or measured speed caps.

On the approximately 0.5 m grid, let <img src="equations/eq_9a9931ca20a83c8e.svg" width="20" alt="K_j" align="middle"> be the maximum absolute curvature in the cyclic approximately plus/minus 1 m neighborhood. Initially,

<p align="center"><img src="equations/eq_18a77caf70e2d16a.svg" width="244" alt="v_j^2=\min\left(v_{\max}^2,\frac{a_{y,{\rm plan}}}{\max(K_j,10^{-8})}\right)."></p>

Periodic forward/backward sweeps impose reachability:

<p align="center"><img src="equations/eq_f451311795705ac2.svg" width="555" alt="v_{j+1}^2\le v_j^2+2a_{\rm plan}\Delta s,\qquad v_j^2\le v_{j+1}^2+2b_{\rm plan}\Delta s, \quad a_{\rm plan}=1.5,\ b_{\rm plan}=2.0\ {\rm m/s^2}."></p>

The horizon references advance with the trapezoidal integral of reference speed and the same acceleration/deceleration limits. No individual plant receives a retuned speed schedule.

### Common tracking MPC

Prediction state is <img src="equations/eq_da11c6e640a78b6f.svg" width="113" alt="z=(X,Y,\psi,V,\delta)" align="middle">; control is <img src="equations/eq_9b5d3fabb8b9b0e8.svg" width="73" alt="u=(\delta_c,a_c)" align="middle">. Set

<p align="center"><img src="equations/eq_fa6863364c89fe62.svg" width="375" alt="\beta_g=\arctan\left(\frac{l_r}{L}\tan\delta\right),\qquad W(V)=mg-\tfrac12\rho C_l A V^2,"></p>

<p align="center"><img src="equations/eq_f535e290bf0ffbad.svg" width="233" alt="a_{\rm res}(V)=\frac{c_{rr}W(V)+\tfrac12\rho C_dA V^2}{m}."></p>

The common predictor is

<p align="center"><img src="equations/eq_48eb1a34116bd909.svg" width="292" alt="\dot z=\begin{pmatrix} V\cos(\psi+\beta_g)\\ V\sin(\psi+\beta_g)\\ V\cos\beta_g\tan\delta/L\\ a_c-a_{\rm res}(V)\\ (\delta_c-\delta)/T_\delta \end{pmatrix},\qquad T_\delta=0.06\ {\rm s}."></p>

Four RK4 substeps of 0.025 s give the discrete map <img src="equations/eq_3d3bb73e12ae84bc.svg" width="18" alt="F_h" align="middle"> for each 0.1 s control interval. The horizon has 25 intervals, hence 2.5 s. Initial prediction state is the current observed pose, speed magnitude, and steering. At each stage define lateral/along-track errors by projections onto reference normal/tangent. The steering and body-heading references are

<p align="center"><img src="equations/eq_8a8850995878c31e.svg" width="479" alt="\delta_j^{\rm ref}=\operatorname{clip}(\arctan(L\kappa(s_j)),-0.5,0.5),\qquad \beta_j^{\rm ref}=\arctan\left(\frac{l_r}{L}\tan\delta_j^{\rm ref}\right),"></p>

<p align="center"><img src="equations/eq_bb99d8f035d9bb8b.svg" width="483" alt="\psi_j^{\rm ref}=\operatorname{unwrap}(\psi_{\rm track}(s_j))-\beta_j^{\rm ref},\quad e_{\psi,j}=\psi_j-\psi_j^{\rm ref},\quad e_{\delta,j}=\delta_j-\delta_j^{\rm ref}."></p>

The unwrapped reference heading is aligned to the observed local angle branch. Speed error is relative to the common horizon speed reference. The exact objective is

<p align="center"><img src="equations/eq_9d16fa9a3616523e.svg" width="379" alt="\begin{aligned} \min_{z_j,u_j}\quad &amp;\sum_{j=0}^{N}\gamma_j\left( 40e_{y,j}^2+0.3e_{s,j}^2+8e_{\psi,j}^2 +2e_{V,j}^2+0.2e_{\delta,j}^2\right)\\ &amp;+\sum_{j=0}^{N-1}\left( 6(\Delta\delta_{c,j})^2+0.06(\Delta a_{c,j})^2+0.01a_{c,j}^2\right), \end{aligned}"></p>

where <img src="equations/eq_a6a5b51f6bf77728.svg" width="47" alt="\gamma_N=3" align="middle"> and all other <img src="equations/eq_b4f03bf10883cad6.svg" width="42" alt="\gamma_j=1" align="middle">. The first input difference uses the last applied control. Constraints are

<p align="center"><img src="equations/eq_e59785a5e3891b9a.svg" width="228" alt="z_0=z_{\rm obs},\qquad z_{j+1}=F_h(z_j,u_j),"></p>

<p align="center"><img src="equations/eq_f2db2b451ddbf4cb.svg" width="261" alt="-w_R(s_j)+2.4\le e_{y,j}\le w_L(s_j)-2.4,"></p>

<p align="center"><img src="equations/eq_0ecea8901ecbc88f.svg" width="418" alt="&#124;\delta_j&#124;\le0.5,\quad &#124;\delta_{c,j}&#124;\le0.5,\quad &#124;\delta_{c,j}-\delta_j&#124;\le0.03,\quad &#124;\Delta\delta_{c,j}&#124;\le0.05,"></p>

<p align="center"><img src="equations/eq_0e35e3bbc3592371.svg" width="306" alt="-4\le a_{c,j}\le2.5\ {\rm m/s^2},\qquad 1\le V_j\le40\ {\rm m/s}."></p>

The 0.03 rad target-state difference makes the initial PT1 slope at most 0.5 rad/s. The held target then gives a decaying slope in the predictor. Steering target changes are also bounded by 0.5 rad/s times the 0.1 s sample. The native and adapted plants separately impose their rate-limited actuators. Terminal cost is present; no terminal invariant-set constraint is imposed.

The problem is solved by CasADi/IPOPT with a warm start, tolerance <img src="equations/eq_689c95e33bf74646.svg" width="32" alt="10^{-7}" align="middle"> and a maximum of 100 iterations. A failed solve or unacceptable constraint residual stops and records that run; the runner does not silently replace the controller. Identical controller settings do not imply identical realized inputs across feedback experiments.

### Command allocation and physical-domain checks

The acceleration command requests gross force, before resistance. Four equal-left/right wheel torques use static axle weight shares:

<p align="center"><img src="equations/eq_f1a1088c966abcfb.svg" width="303" alt="\boldsymbol\eta=\left(\frac{l_r}{2L},\frac{l_r}{2L}, \frac{l_f}{2L},\frac{l_f}{2L}\right),\qquad T_i=mR a_c\eta_i."></p>

Liniger's adapter sums these into its rear-axle force. Sun and Lienkamp convert the sum into their longitudinal command; those models do not suddenly acquire four independent driven tires by accepting the common interface.

| Condition | Where it belongs | What is actually used |
| --- | --- | --- |
| Initial state | ODE/IVP | Common observation and model-specific internal initialization |
| Closed ring periodicity | Liu PDE spatial boundary | Fields and required spatial derivatives match at zero and a full turn |
| Road corridor | MPC task constraint | Hard predicted center corridor with 2.4 m margin |
| Physical vehicle boundary | Recorded diagnostic | 40 perimeter points of a 4.4 m by 1.9 m rectangle, checked every 20 ms |
| Steering and steering rate | Actuator and MPC | 0.5 rad magnitude; 0.5 rad/s rate |
| Wheel nonreversal | Native derivative clamp / adapter projection | Explicit implementation limits, not a road boundary |
| Normal load | Contact model | Native 0.001 N floor; static or declared algebraic loads in reduced models |
| Friction-circle ratio | Model-dependent diagnostic | Coupled brush has a bounded resultant; native independent MF does not impose a circle |
| Terminal lap event | Experiment stopping condition | One first interpolated crossing, no periodic-state equality |

The sampled rectangle clearance is not a proof of continuous-time collision avoidance. Native integration uses its source DoPri implementation with 0.0005 s substeps; the adapted models use RK4 at 0.001 s with wheel-speed and steering projections. The validation includes smaller-step repeats and identical-command forecast windows. Timings compare these implementations on this host, including Python versus C++, not algorithmic complexity in isolation.



## Complete native OCD33 equations at pinned source

Audit pin: `94f8fb187fb0ed22bba1d809bd74f66d1ff75af4`, composition `WHEEL_TORQUE / PT1 / DOUBLE_TRACK / MF_SIMPLE / DEFAULT`.
This section gives all derivative groups and algebraic force closures for the selected native composition. It retains the signs, state limits, low-speed regularization, and parameter distinctions in the pinned source.

Source files:

1. `lib/ocd_vehicle_dynamics_double_track_cpp/include/ocd_vehicle_dynamics_double_track_cpp/states.hpp`
2. `lib/ocd_vehicle_dynamics_double_track_cpp/include/ocd_vehicle_dynamics_double_track_cpp/vehicle_dynamics_equations_impl.hpp`
3. `lib/ocd_tire_models_cpp/src/mf_simple.cpp`
4. `lib/ocd_aerodynamics_models_cpp/src/default.cpp`
5. `lib/ocd_drivetrain_wheel_torque_cpp/src/drivetrain_direct_torque_eqns.cpp`
6. `lib/ocd_steering_actuator_pt1_cpp/src/steering_actuator_pt1_model.cpp`

### 1. Coordinates, states, inputs

Body <img src="equations/eq_409da6b793edcac0.svg" width="10" alt="x" align="middle"> forward, <img src="equations/eq_6e5361dc65d65446.svg" width="9" alt="y" align="middle"> left; yaw <img src="equations/eq_ace5938c45794529.svg" width="12" alt="\psi" align="middle"> counterclockwise. Heave coordinates positive upward. Roll and pitch use the implemented small-angle suspension geometry. Define <img src="equations/eq_d5b5ba1d5fe76a72.svg" width="119" alt="p=\dot\phi,q=\dot\theta,r=\dot\psi" align="middle">.

For wheels <img src="equations/eq_88a6254d1d0c0769.svg" width="143" alt="i=(FL,FR,RL,RR)" align="middle">, define
<p align="center"><img src="equations/eq_ff736790a644a77d.svg" width="463" alt="a=(l_f,l_f,-l_r,-l_r),\quad b=(t_f/2,-t_f/2,t_r/2,-t_r/2),\quad L=l_f+l_r."></p>
All four-entry vectors below follow that wheel order. Exact native continuous-state storage:
<p align="center"><img src="equations/eq_8abf5f6232622606.svg" width="428" alt="x=(X,Y,z,\psi,\phi,\theta,v_x,v_y,v_z,r,p,q, \mathbf z_w,\mathbf v_w,\bar{\boldsymbol\kappa}, \bar{\boldsymbol\alpha},\boldsymbol\omega,\delta)^\top\in\mathbb R^{33}."></p>
The first 28 entries are the vehicle dynamics model; four <img src="equations/eq_6a08719b78dba377.svg" width="15" alt="\omega_i" align="middle"> are the wheel-torque drivetrain and one <img src="equations/eq_66fb6569c98ef674.svg" width="9" alt="\delta" align="middle"> is the PT1 steering actuator. This is 33 **continuous states**, not 33 mechanical degrees of freedom.

Driver input:
<p align="center"><img src="equations/eq_b3258ae92e41d9f6.svg" width="236" alt="u=(T_{FL},T_{FR},T_{RL},T_{RR},\delta_{\rm cmd})^\top."></p>
Environment inputs are four nonnegative tire-force scale multipliers <img src="equations/eq_d8c1da888bacfaa1.svg" width="22" alt="\lambda_{\mu i}" align="middle">, wind <img src="equations/eq_99df7129128d8926.svg" width="14" alt="\mathbf w" align="middle">, external body force <img src="equations/eq_a9fd183c9527552b.svg" width="18" alt="\mathbf F_e" align="middle">, and external torque <img src="equations/eq_acd6ff504439b64c.svg" width="24" alt="\mathbf M_e" align="middle">. The comparison keeps wind/external force/torque zero. The source logs a road-height input but the selected tire vertical law below does not use it.

### 2. Dependent masses, geometry, preloads

Let <img src="equations/eq_937d3b4a2329eed8.svg" width="14" alt="m" align="middle"> be total vehicle mass, including the four unsprung masses <img src="equations/eq_15ce21dda9d620ad.svg" width="65" alt="m_{wf},m_{wr}" align="middle"> per front/rear wheel:
<p align="center"><img src="equations/eq_df8fd69f23d2b719.svg" width="486" alt="m_s=m-2m_{wf}-2m_{wr},\qquad l_{r,s}=\frac{m l_r-2m_{wf}L}{m_s},\qquad l_{f,s}=L-l_{r,s}."></p>
Use <img src="equations/eq_e6691d3b512aa481.svg" width="166" alt="a_s=(l_{f,s},l_{f,s},-l_{r,s},-l_{r,s})" align="middle">.

Tire spring preload compressions:
<p align="center"><img src="equations/eq_fa76ef5c55002414.svg" width="227" alt="z^0_{t,f}=\frac{mgl_r}{2Lc_{t,f}},\qquad z^0_{t,r}=\frac{mgl_f}{2Lc_{t,r}}."></p>
Suspension spring preload compressions:
<p align="center"><img src="equations/eq_a0175ba7d08aebe3.svg" width="227" alt="s^0_f=\frac{m_sg l_{r,s}}{2Lc_f},\qquad s^0_r=\frac{m_sg l_{f,s}}{2Lc_r}."></p>
Both sides of an axle use the corresponding same preload. <img src="equations/eq_e37d8f9aa77cd58c.svg" width="92" alt="g=9.81\,{\rm m\,s^{-2}}" align="middle">. Zero heave states therefore still produce static weight support.

### 3. Effective steering, wheel velocities, slips

Define
<p align="center"><img src="equations/eq_3747c198ecc43313.svg" width="185" alt="R(\gamma)= \begin{pmatrix}\cos\gamma&amp;-\sin\gamma\\ \sin\gamma&amp;\cos\gamma\end{pmatrix}."></p>
For PT1 state <img src="equations/eq_66fb6569c98ef674.svg" width="9" alt="\delta" align="middle">, static steering offset <img src="equations/eq_9589c9b09a98b2c1.svg" width="15" alt="o_\delta" align="middle">, and per-wheel toe-out parameter <img src="equations/eq_50fe428e91ee54fd.svg" width="13" alt="\tau_i" align="middle">,
<p align="center"><img src="equations/eq_44ef7584ffdafcbc.svg" width="329" alt="\boldsymbol\delta_w= (\delta+o_\delta+\tau_{FL},\ \delta+o_\delta-\tau_{FR},\ \tau_{RL},-\tau_{RR})."></p>
Thus front steering is parallel; rear steering only has configured toe offsets. The comparison has all offsets zero.

Dynamic rolling radius:
<p align="center"><img src="equations/eq_8b53fb56f21c70b8.svg" width="119" alt="R_i=R_{0i}s_R(v_x),"></p>
where <img src="equations/eq_4681494f0706e908.svg" width="18" alt="s_R" align="middle"> is the source's tabulated linear interpolation. For increasing knots <img src="equations/eq_b61c3a73f23f47a8.svg" width="47" alt="(x_j,f_j)" align="middle">, the interpolation is

<p align="center"><img src="equations/eq_45416b8678f797d4.svg" width="371" alt="I(x)=\begin{cases} f_0,&amp;x\le x_0,\\ f_j+\dfrac{x-x_j}{x_{j+1}-x_j}(f_{j+1}-f_j),&amp;x_j&lt;x&lt;x_{j+1},\\ f_n,&amp;x\ge x_n. \end{cases}"></p>

The default radius table is <img src="equations/eq_ce7742d0f6c82788.svg" width="87" alt="(0,1),(100,1)" align="middle">, so <img src="equations/eq_8217dce9dac7cd3d.svg" width="70" alt="R_i=0.3\,{\rm m}" align="middle"> throughout the comparison. The aero corrections below use the same endpoint-clamped interpolation.

For each wheel:
<p align="center"><img src="equations/eq_37ab3c3a88e529c2.svg" width="211" alt="\binom{U_i}{V_i} =R(-\delta_{w,i})\binom{v_x-rb_i}{v_y+ra_i}."></p>
Instantaneous target slips:
<p align="center"><img src="equations/eq_a2d00662348a60a7.svg" width="450" alt="\kappa_i^\star= \operatorname{clip}\left( \frac{R_i\omega_i-U_i}{\max(3,U_i)},-1,1\right),\qquad \alpha_i^\star=-\operatorname{atan2}(V_i,\max(1,U_i))."></p>
Here 3 and 1 are speeds in <img src="equations/eq_282f8f135daa0de0.svg" width="27" alt="{\rm m/s}" align="middle">; the denominators do not use absolute speed.

Define <img src="equations/eq_6d28f8f966921350.svg" width="138" alt="\nu_i=\operatorname{clip}(&#124;U_i&#124;,10,100)" align="middle">, also in <img src="equations/eq_282f8f135daa0de0.svg" width="27" alt="{\rm m/s}" align="middle">. Tire-relaxation time constants are
<p align="center"><img src="equations/eq_884656ae3c2512e4.svg" width="183" alt="T_{\kappa i}=\frac{0.3}{\nu_i},\qquad T_{\alpha i}=\frac{0.2}{\nu_i}."></p>
The numerators are relaxation lengths in metres.

### 4. Vertical tire loads and MF_SIMPLE

The signed tire spring force is
<p align="center"><img src="equations/eq_fbdd513a232e1c4f.svg" width="369" alt="F_{t,i}=\min\{c_{t,i}(z_{w,i}-z^0_{t,i}),-0.001\},\qquad F_{z,i}=-F_{t,i}."></p>
Hence <img src="equations/eq_6dda5a65cc68e13c.svg" width="90" alt="F_{z,i}\ge0.001\,{\rm N}" align="middle">. This floor is numerical regularization, not a resolved airborne/contact model. There is no tire vertical damper in this selected closure.

Define
<p align="center"><img src="equations/eq_e41ec881ccaaab77.svg" width="431" alt="\mathcal M(s;B,C,D,E)=D\sin\!\left( C\arctan\!\left[Bs-E(Bs-\arctan Bs)\right]\right)."></p>
The selected tire law gives
<p align="center"><img src="equations/eq_30448a795177a3df.svg" width="274" alt="F^t_{x,i}=\lambda_{\mu i}F_{z,i} \mathcal M(\bar\kappa_i;B_{xi},C_{xi},D_{xi},E_{xi}),"></p>
<p align="center"><img src="equations/eq_349a8ce4d0334985.svg" width="298" alt="F^t_{y,i}=\lambda_{\mu i}F_{z,i} \mathcal M(\tan\bar\alpha_i;B_{yi},C_{yi},D_{yi},E_{yi})."></p>
Right tire inputs and lateral outputs are both mirrored in the C++ call; because this particular MF is odd, the displayed law applies on both sides. Camber is passed to the interface but ignored by MF_SIMPLE.

At rest the following override takes precedence:
<p align="center"><img src="equations/eq_c1e0db79d1c51d2e.svg" width="475" alt="v_x^2&lt;0.001,\quad v_y^2&lt;0.001,\quad \omega_i^2&lt;0.001\ \forall i \quad\Longrightarrow\quad F^t_{x,i}=F^t_{y,i}=0\ \forall i."></p>
Otherwise the MF equations apply, including when relaxed slips are nonzero.

Body-frame forces:
<p align="center"><img src="equations/eq_fcce523f3c9f6e54.svg" width="180" alt="\binom{F^b_{x,i}}{F^b_{y,i}} =R(\delta_{w,i})\binom{F^t_{x,i}}{F^t_{y,i}}."></p>

**No combined-slip constraint is included in MF_SIMPLE.** Longitudinal and lateral forces are evaluated independently. A diagnostic resultant ratio <img src="equations/eq_c4bdb9a0f9a95906.svg" width="139" alt="\sqrt{F_x^2+F_y^2}/(\mu F_z)&gt;1" align="middle"> can occur without violating any implemented force constraint. Do not silently impose a friction-circle projection and still label it native MF_SIMPLE.

### 5. Suspension spring, damper, antiroll, and geometric axle forces

For each wheel:
<p align="center"><img src="equations/eq_fdcb6b8608e5803e.svg" width="452" alt="d_i=-s^0_i+z+\phi b_i-\theta a_{s,i}-z_{w,i},\qquad \dot d_i=v_z+p b_i-q a_{s,i}-v_{w,i}."></p>
Spring and damping forces:
<p align="center"><img src="equations/eq_14a7289dc607a559.svg" width="233" alt="F_{{\rm sp},i}=c_i d_i,\qquad F_{{\rm damp},i}=d_{v,i}\dot d_i."></p>
The damping symbol <img src="equations/eq_3a311a48dd4efedd.svg" width="23" alt="d_{v,i}" align="middle"> here denotes the parameter called `d_f/d_r`, to avoid confusion with displacement <img src="equations/eq_ba855d5db47429b0.svg" width="14" alt="d_i" align="middle">.

Antiroll forces:
<p align="center"><img src="equations/eq_8c8645c6372565be.svg" width="321" alt="A^{\rm ar}_{FL}=\tfrac12 c_{{\rm ar},f}(d_{FL}-d_{FR}), \qquad A^{\rm ar}_{FR}=-A^{\rm ar}_{FL},"></p>
<p align="center"><img src="equations/eq_4744352c9c18e6b9.svg" width="319" alt="A^{\rm ar}_{RL}=\tfrac12 c_{{\rm ar},r}(d_{RL}-d_{RR}), \qquad A^{\rm ar}_{RR}=-A^{\rm ar}_{RL}."></p>

To preserve the exact jacking/anti-squat/anti-dive source branch, define
<p align="center"><img src="equations/eq_0d1bd0cd5e930348.svg" width="389" alt="\epsilon=(-1,-1,1,1),\qquad \eta=(1,-1,1,-1),\qquad \xi_i=\epsilon_iF^b_{x,i}."></p>
Use <img src="equations/eq_115a1ba8dbbdc540.svg" width="44" alt="t_i=t_f" align="middle"> and <img src="equations/eq_764665624cef5e21.svg" width="43" alt="\ell_i=l_f" align="middle"> at the front; <img src="equations/eq_6170886c19fcf8d0.svg" width="90" alt="t_i=t_r,\ell_i=l_r" align="middle"> at the rear:
<p align="center"><img src="equations/eq_35520b1de7fba480.svg" width="347" alt="k_{{\rm sl},i}=\frac{2h_{\rm rc}}{t_i}, \qquad k_{{\rm sa},i}(\xi_i)= \begin{cases} h_{{\rm pc},{\rm accel}}/\ell_i,&amp;\xi_i&gt;0,\\ h_{{\rm pc},{\rm decel}}/\ell_i,&amp;\xi_i\le0. \end{cases}"></p>
Then
<p align="center"><img src="equations/eq_a640bf38fb4d21fa.svg" width="445" alt="A^{\rm ax}_i=k_{{\rm sl},i}\eta_iF^b_{y,i} -k_{{\rm sa},i}(\xi_i)\xi_i, \qquad S_i=c_i d_i+d_{v,i}\dot d_i+A^{\rm ar}_i+A^{\rm ax}_i."></p>
The source tests <img src="equations/eq_5fb6358d423ce7b7.svg" width="13" alt="\xi_i" align="middle">, not body acceleration; at the front <img src="equations/eq_6cd6c1412f500a35.svg" width="18" alt="F_x" align="middle"> has already been negated before the branch. Its default pitch-center heights are equal, both <img src="equations/eq_93e43e11052889e2.svg" width="20" alt="0.2" align="middle"> m, so this choice does not change the default benchmark coefficients.

### 6. Default aerodynamics and total forces/moments

Let <img src="equations/eq_54cd90d7a2e83285.svg" width="181" alt="u_x=v_x-w_x,\ u_y=v_y-w_y" align="middle">. The default aero source uses:
<p align="center"><img src="equations/eq_0aa1d7e224c1dcf3.svg" width="356" alt="F_{a,x}=-\tfrac12\rho A C_d\,u_x&#124;u_x&#124;,\qquad F_{a,y}=-\tfrac12\rho A C_d\,u_y&#124;u_y&#124;,"></p>
<p align="center"><img src="equations/eq_600ad21efed041da.svg" width="208" alt="F_{a,z}=\tfrac12\rho A[C_l+\Delta C_l(u_x)]u_x^2,"></p>
<p align="center"><img src="equations/eq_bc8549b9d1949561.svg" width="350" alt="M_{a,x}=M_{a,z}=0,\qquad M_{a,y}=-F_{a,z}[d_a+\Delta d_a(\theta)]."></p>
Both corrections are source interpolation tables. The benchmark defaults make them zero, along with <img src="equations/eq_9146ededa03b17e6.svg" width="16" alt="d_a" align="middle"> and wind. Negative <img src="equations/eq_fadb515d2f1dd82b.svg" width="16" alt="C_l" align="middle"> means downforce. Body height is unused by this selected default aero module. The implementation contains no extra drag-height pitching moment.

External-force attenuation:
<p align="center"><img src="equations/eq_6eda2590bd9ff3c8.svg" width="183" alt="\chi=\operatorname{clip}(0.2(v_x-2),0,1)."></p>
Complete totals:
<p align="center"><img src="equations/eq_8d51e1ed4cd12c5a.svg" width="433" alt="F_x=\sum_iF^b_{x,i}+F_{a,x}+\chi F_{e,x},\qquad F_y=\sum_iF^b_{y,i}+F_{a,y}+\chi F_{e,y},"></p>
<p align="center"><img src="equations/eq_a15e3a9e1fedaa94.svg" width="267" alt="F_z^{\rm body}=-\sum_iS_i-m_sg+F_{a,z}+F_{e,z},"></p>
<p align="center"><img src="equations/eq_9d4b66aac845287a.svg" width="287" alt="M_z=\sum_i(a_iF^b_{y,i}-b_iF^b_{x,i})+M_{a,z}+M_{e,z},"></p>
<p align="center"><img src="equations/eq_edae87e6a78defc8.svg" width="305" alt="M_x=h_{\rm cg}\sum_iF^b_{y,i}-\sum_i b_iS_i+M_{a,x}+M_{e,x},"></p>
<p align="center"><img src="equations/eq_f1b8efaf16b56796.svg" width="429" alt="M_y=-h_{\rm cg}\sum_iF^b_{x,i}+\sum_i a_iS_i +m_sg(l_f-l_{f,s})+M_{a,y}+M_{e,y}."></p>
The attenuation <img src="equations/eq_9c3f192833f55b79.svg" width="11" alt="\chi" align="middle"> applies only to external x/y forces, not tire forces, aero, external vertical force, or external torques. The suspension deflections use sprung-CG distances <img src="equations/eq_ce1aaa225816aa35.svg" width="15" alt="a_s" align="middle">, while moment totals use whole-vehicle distances <img src="equations/eq_a113df440195c2ea.svg" width="9" alt="a" align="middle">, with the gravity correction shown.

### 7. All 33 continuous-state derivatives

#### Body: twelve equations
<p align="center"><img src="equations/eq_0d57d2b92fc606de.svg" width="435" alt="\dot X=v_x\cos\psi-v_y\sin\psi,\qquad \dot Y=v_x\sin\psi+v_y\cos\psi,\qquad \dot\psi=r,"></p>
<p align="center"><img src="equations/eq_70d49a19915cef57.svg" width="392" alt="\dot v_x=F_x/m+r v_y,\qquad \dot v_y=F_y/m-r v_x,\qquad \dot r=M_z/I_z,"></p>
<p align="center"><img src="equations/eq_99dff8abecb744cf.svg" width="197" alt="\dot z=v_z,\qquad \dot v_z=F_z^{\rm body}/m_s,"></p>
<p align="center"><img src="equations/eq_99e43a0f85e4a703.svg" width="346" alt="\dot\phi=p,\qquad \dot p=M_x/I_x,\qquad \dot\theta=q,\qquad \dot q=M_y/I_y."></p>
This source does not use a general Euler-angle derivative map or gyroscopic cross-coupling. Roll/pitch influence the planar force calculation through suspension/load dynamics.

#### Unsprung vertical motion: eight equations
For each of four wheels:
<p align="center"><img src="equations/eq_1670d47027ea51df.svg" width="288" alt="\dot z_{w,i}=v_{w,i},\qquad \dot v_{w,i}=\frac{S_i-F_{t,i}-m_{w,i}g}{m_{w,i}}."></p>

#### Slip relaxation: eight equations
For each wheel:
<p align="center"><img src="equations/eq_d071437a1e0ca0f7.svg" width="229" alt="\dot{\bar\kappa}_i=\frac{\kappa_i^\star-\bar\kappa_i}{T_{\kappa i}}, \qquad \dot{\bar\alpha}_i=\frac{\alpha_i^\star-\bar\alpha_i}{T_{\alpha i}}."></p>
Only instantaneous target <img src="equations/eq_5563026e9137359a.svg" width="17" alt="\kappa^\star" align="middle"> is clipped; an arbitrary restored relaxed state is not separately projected.

#### Wheel spin: four equations
<p align="center"><img src="equations/eq_f2161a303acbeaef.svg" width="305" alt="F_{{\rm rr},i}=c_{\rm rr}F_{z,i},\qquad T_{{\rm load},i}=R_i(F^t_{x,i}+F_{{\rm rr},i}),"></p>
<p align="center"><img src="equations/eq_378e13e69746f556.svg" width="344" alt="\bar\omega=\tfrac14\sum_i\omega_i,\qquad J_{\rm low}=\max\{0,15J_r(1-\bar\omega/20)\},"></p>
<p align="center"><img src="equations/eq_c10de61b760e8954.svg" width="383" alt="g_{\omega i}=\frac{T_i-T_{{\rm load},i}}{J_i+J_{\rm low}}, \qquad \dot\omega_i= \begin{cases} \max(0,g_{\omega i}),&amp;\omega_i&lt;10^{-5},\\ g_{\omega i},&amp;\omega_i\ge10^{-5}. \end{cases}"></p>
The thresholds 20 and <img src="equations/eq_b83960e577c6b2ac.svg" width="32" alt="10^{-5}" align="middle"> are in rad/s. <img src="equations/eq_64ad267f222af704.svg" width="49" alt="J_i=J_f" align="middle"> at the front and <img src="equations/eq_b7c51ac009188bfe.svg" width="16" alt="J_r" align="middle"> at the rear. Rolling resistance is positive for this forward-motion model and enters wheel torque; it is **not** also subtracted directly in <img src="equations/eq_6cd6c1412f500a35.svg" width="18" alt="F_x" align="middle">. An engine-inertia parameter is declared but does not enter these selected wheel-torque equations. The wheel boundary is derivative clipping, not exact finite-step state projection.

#### Steering: one equation
<p align="center"><img src="equations/eq_3df885a39cde9308.svg" width="247" alt="g_\delta=\operatorname{clip}\left( \frac{\delta_{\rm cmd}-\delta}{T_\delta}, -\dot\delta_{\max},\dot\delta_{\max}\right),"></p>
<p align="center"><img src="equations/eq_282d407433754fad.svg" width="217" alt="\dot\delta= \begin{cases} \max(0,g_\delta),&amp;\delta\le-\delta_{\max},\\ \min(0,g_\delta),&amp;\delta\ge\delta_{\max},\\ g_\delta,&amp;&#124;\delta&#124;&lt;\delta_{\max}. \end{cases}"></p>
The physical angle is <img src="equations/eq_521ae7ccc26e2fed.svg" width="40" alt="\delta+o_\delta" align="middle"> before toe correction; measured feedback is <img src="equations/eq_66fb6569c98ef674.svg" width="9" alt="\delta" align="middle">. Native steering load torque is zero. No steering rack inertia is present.

### 8. Initialization and numerical/physical boundaries

Upstream zero states, benchmark moving-car trim, and a restored full-state snapshot are distinct operations.

Benchmark initial position is the centerline start, initial yaw is its tangent, <img src="equations/eq_771896e48299bd89.svg" width="70" alt="v_x=8\,{\rm m/s}" align="middle">, <img src="equations/eq_d4f1505ab0d088d8.svg" width="94" alt="v_y=r=\delta=0" align="middle">. Four wheel rates start at <img src="equations/eq_d091d1bcb4f44c09.svg" width="71" alt="8/0.3\,{\rm rad/s}" align="middle">; all relaxed slips are zero. Vertical coordinates are solved from:
<p align="center"><img src="equations/eq_a41ecbec7284d577.svg" width="353" alt="\dot v_z=\dot p=\dot q=\dot v_{w,FL} =\dot v_{w,FR}=\dot v_{w,RL}=\dot v_{w,RR}=0."></p>
The unknowns are <img src="equations/eq_23896c8ceb5be170.svg" width="210" alt="z,\phi,\theta,z_{w,FL},z_{w,FR},z_{w,RL},z_{w,RR}" align="middle">. All vertical/roll/pitch rates begin zero. This is a vertical trim, not full longitudinal/cornering equilibrium.

For a general positive-speed common-observation reset, the adapter takes
<p align="center"><img src="equations/eq_8b80be728f954631.svg" width="420" alt="\omega_i(0)=\max(U_i(0),0)/R_{0i},\qquad \bar\kappa_i(0)=0,\qquad \bar\alpha_i(0)=\alpha_i^\star(0),"></p>
under zero-toe/unit-radius-scale comparison assumptions, then performs that vertical trim. A supplied full 33-state snapshot is restored directly.

Source physical domains: positive masses/inertias/stiffnesses, <img src="equations/eq_da73d6bf1fb9e26c.svg" width="69" alt="0&lt;l_f&lt;L" align="middle">, positive time constants, sensible interpolation tables. Numerical guards: contact floor, slip floors/clipping, rest override, nonnegative-wheel derivative boundary, PT1 steering rate/magnitude derivative boundary.

The native plant has no track corridor, terminal equality, periodic lap condition, wheel torque magnitude/rate bound, engine power/battery bound, or combined tire-friction circle in MF_SIMPLE. Those must be assigned to controller/test setup if used.

The bridge's fixed integration substep is 0.0005 s by default. Native composition caches wheel-speed output between derivative evaluations; the bridge zero-duration synchronization at reset/evaluate/output boundaries preserves upstream physical stepping. Therefore equations above are exact **component derivative laws**; do not claim a mathematically pure explicit-RK assembled evaluation without acknowledging caching.

### 9. Parameter audit

Prior report parameter table matches source declarations:
- <img src="equations/eq_0a76881f0bec4a97.svg" width="126" alt="m=800,\ m_s=740" align="middle"> kg; each <img src="equations/eq_4f17bd59bba58475.svg" width="65" alt="m_{w,i}=15" align="middle"> kg.
- <img src="equations/eq_bc4aa55bd2341714.svg" width="225" alt="I_z=1000,I_y=500,I_x=100\ {\rm kg\,m^2}" align="middle">.
- <img src="equations/eq_7508d595707626ae.svg" width="189" alt="L=3.2,l_f=1.724,l_r=1.476" align="middle"> m; tracks <img src="equations/eq_bb3056c6c3a91a39.svg" width="45" alt="1.6,1.5" align="middle"> m.
- <img src="equations/eq_e98dac98efdb9587.svg" width="259" alt="h_{\rm cg}=0.3,h_{\rm rc}=h_{\rm pc,accel}=h_{\rm pc,decel}=0.2" align="middle"> m.
- <img src="equations/eq_41a7b9c2eb67a46f.svg" width="318" alt="c_f=120000,c_r=90000,c_{t,f}=c_{t,r}=250000\ {\rm N/m}" align="middle">.
- <img src="equations/eq_eab9d92f63b3dddd.svg" width="186" alt="d_{v,f}=1300,d_{v,r}=900\ {\rm Ns/m}" align="middle">; <img src="equations/eq_fc6aa87ab3cab841.svg" width="200" alt="c_{{\rm ar},f}=12000,c_{{\rm ar},r}=5000\ {\rm N/m}" align="middle">.
- <img src="equations/eq_9b20b60ca92bd228.svg" width="55" alt="R_i=0.3" align="middle"> m, <img src="equations/eq_6e1b50ec29834217.svg" width="169" alt="J_f=1.15,J_r=1.35\ {\rm kg\,m^2}" align="middle">.
- <img src="equations/eq_5d98a6ffa3e440ce.svg" width="319" alt="\rho=1.225,\ A=1,\ C_d=1,\ C_l=-1.5,\ c_{\rm rr}=0.025" align="middle"> in SI.
- MF upstream: <img src="equations/eq_9ce1a014f55087cc.svg" width="213" alt="(B_x,C_x,D_x,E_x)=(20,2,1.7,0.7)" align="middle">; <img src="equations/eq_4161c6d77e04085f.svg" width="235" alt="(B_y,C_y,D_y,E_y)=(10,1.6,1.7,-1.9)" align="middle">.
- Comparison explicitly changes all eight MF <img src="equations/eq_75280bd3ec535a76.svg" width="14" alt="D" align="middle"> peaks to 1.0. Since small-slip stiffness is <img src="equations/eq_aef4088d89b6a302.svg" width="53" alt="F_zBCD" align="middle">, this also changes stiffness; it is not only a cap adjustment.
- Steering: <img src="equations/eq_80ef388a59b9655e.svg" width="61" alt="T_\delta=0.06" align="middle"> s, <img src="equations/eq_bb426626ba76fc02.svg" width="68" alt="\dot\delta_{\max}=0.5" align="middle"> rad/s; native <img src="equations/eq_4eb34bdcd42ac9c8.svg" width="68" alt="\delta_{\max}=0.3" align="middle"> rad, comparison override <img src="equations/eq_6d732b9d45d2de01.svg" width="20" alt="0.5" align="middle"> rad.



## Source identity audit  -  internal report, Lienkamp, native OCD

The supplied PDFs and pinned native implementation were checked directly. The page references below distinguish printed page numbers from PDF page indices.

### Source files and pagination

- Internal report: `upload/Car model - internal report partly - check it before use (1)(2).pdf`. Six PDF pages only: PDF 1=printed 2, PDF 2=printed 3, PDF 3=printed 4, PDF 4=printed 5, PDF 5=printed 6, PDF 6=printed 20 (references). The referenced Listings (4)/(5) are absent.
- Lienkamp: `upload/Lienkamp - 2019 - Dynamik der Straßenfahrzeuge (1)(2).pdf`. Ninety PDF pages, mostly printed slide `2.1 - n` matching PDF page n. Only chapter 2.1, including linear/nonlinear single-track and trailer. Slide 2 agenda lists double-track chapters 2.2-2.6, and multi-body/FEM 2.7; those chapters are not in this upload.
- Native source pin: `TUMFTM/Open-Car-Dynamics@94f8fb187fb0ed22bba1d809bd74f66d1ff75af4`.
  - `lib/ocd_vehicle_dynamics_double_track_cpp/include/ocd_vehicle_dynamics_double_track_cpp/states.hpp`
  - `.../vehicle_dynamics_equations_impl.hpp`
  - `lib/ocd_tire_models_cpp/src/mf_simple.cpp`.
  Checked against the pinned source checkout.

### Conclusions

1. **Native OCD is not the internal-report model.** Both are double-track at the geometric level (four contact patches), but the internal report is a planar six-state yaw/sideslip/speed model with zero CG height, a lumped body-axis driving force, no wheel spin, no tire relaxation, no suspension or roll/pitch/heave. Its nonlinear lateral tire law is of Burckhardt form, not MF_SIMPLE.
2. **The supplied Lienkamp document is not native OCD either.** It teaches a hierarchy of single-track models. We cannot identify the absent double-track chapters with native OCD merely from the course title, affiliation, or agenda. The internal report explicitly cites Lienkamp as reference [1], demonstrating pedagogical lineage, not numerical identity.
3. **Do not claim a fully faithful executable internal-report reproduction.** Missing <img src="equations/eq_51fb18a2bfa6c174.svg" width="20" alt="S_Y" align="middle"> definition, normal-load equations, <img src="equations/eq_f2d05ef4c3b38bf1.svg" width="65" alt="C_1,C_2,C_3" align="middle"> values, vehicle parameters, numerical regularization, and referenced MATLAB code prevent exact reproduction from the provided excerpt.
4. **Do not call every Lienkamp equation exact without correction.** Slide 65 has two visible algebraic inconsistencies compared with slide 64 (see below).
5. Native selected composition has 28 vehicle states plus four wheel angular velocities and one steering state, i.e. 33. A six-state reduction is a separate model.

### Common audited convention

Use body <img src="equations/eq_409da6b793edcac0.svg" width="10" alt="x" align="middle"> forward, <img src="equations/eq_6e5361dc65d65446.svg" width="9" alt="y" align="middle"> left, yaw <img src="equations/eq_ace5938c45794529.svg" width="12" alt="\psi" align="middle"> counterclockwise, <img src="equations/eq_a10965fc6849e195.svg" width="38" alt="r=\dot\psi" align="middle">. Let <img src="equations/eq_48af6b2891f8ec4e.svg" width="43" alt="u=v_x" align="middle">, <img src="equations/eq_336708cc8c9ffdd2.svg" width="78" alt="v_y=V\sin\beta" align="middle">, speed <img src="equations/eq_0e0f3c52fe96ddfb.svg" width="93" alt="V=\sqrt{u^2+v_y^2}" align="middle">, <img src="equations/eq_8a4c5e6484d6f23f.svg" width="74" alt="u=V\cos\beta" align="middle">. Wheel contact coordinates <img src="equations/eq_e5dc3ec74789eeb6.svg" width="44" alt="(x_i,y_i)" align="middle"> are:
<p align="center"><img src="equations/eq_8fdba849a4cfa4c1.svg" width="337" alt="(l_f,w_f/2),(l_f,-w_f/2),(-l_r,w_r/2),(-l_r,-w_r/2)"></p>
in FL,FR,RL,RR order. The force transform is
<p align="center"><img src="equations/eq_245df3bc35e43cd7.svg" width="452" alt="\binom{F_{xi}^b}{F_{yi}^b}= \begin{pmatrix}\cos\delta_i&amp;-\sin\delta_i\\ \sin\delta_i&amp;\cos\delta_i\end{pmatrix} \binom{F_{xi}^t}{F_{yi}^t}, \qquad M_z=\sum_i(x_iF_{yi}^b-y_iF_{xi}^b)."></p>

### Internal report  -  supplied equations

#### Planar force balances: printed p.3/PDF 2, Eqs.(1)-(3)

Let <img src="equations/eq_a53fe1db9aca87e3.svg" width="70" alt="L_i=F_{YT,i}" align="middle">, front force sum <img src="equations/eq_bb782baf88bb1c8c.svg" width="100" alt="L_f=L_{fl}+L_{fr}" align="middle">, rear sum <img src="equations/eq_71972bdf0cd9b19c.svg" width="97" alt="L_r=L_{rl}+L_{rr}" align="middle">.
<p align="center"><img src="equations/eq_977f0b8e250e9947.svg" width="471" alt="ma_x=-L_f\sin\delta_f-L_r\sin\delta_r+F_{\rm dr}, \qquad ma_y=L_f\cos\delta_f+L_r\cos\delta_r."></p>
The report prints:
<p align="center"><img src="equations/eq_e67ab66c12a48531.svg" width="439" alt="\begin{aligned} I_z\dot r={}&amp;L_{fl}(l_f\cos\delta_f-\tfrac{w_f}{2}\sin\delta_f) +L_{fr}(l_f\cos\delta_f+\tfrac{w_f}{2}\sin\delta_f)\\ &amp;+L_{rl}(-l_r\cos\delta_r-\tfrac{w_r}{2}\sin\delta_r) +L_{rr}(-l_r\cos\delta_r+\tfrac{w_r}{2}\sin\delta_r). \end{aligned}"></p>
**Audit:** The four track-width <img src="equations/eq_fd68dca960bb51bc.svg" width="29" alt="\sin\delta" align="middle"> signs are inconsistent with the forward wheel-velocity/slip denominators printed in Eqs.(10)-(13), when read in one common body convention. A Newton/Euler consistent implementation should use the cross-product equation above, which gives <img src="equations/eq_2dda05eb47eb839e.svg" width="54" alt="+\tfrac w2\sin\delta" align="middle"> for left and <img src="equations/eq_f62459c8f7a573a3.svg" width="54" alt="-\tfrac w2\sin\delta" align="middle"> for right. Preserve the printed form when quoting, mark corrected form when implementing. The Figure 2 y-axis points page-right while force <img src="equations/eq_285b4fd6da42f647.svg" width="18" alt="F_y" align="middle"> points page-left; the illustration is itself convention-confusing.

#### Six-state ODE: printed p.4/PDF 3, Eqs.(4)-(8)

States <img src="equations/eq_67d38a460daccbc9.svg" width="92" alt="X,Y,\psi,\beta,r,V" align="middle">, inputs <img src="equations/eq_ae0f4c7a3193f963.svg" width="62" alt="\delta_f,\delta_r,F_{\rm dr}" align="middle">:
<p align="center"><img src="equations/eq_4600dabb1c18d15c.svg" width="335" alt="\dot X=V\cos(\psi+\beta),\quad \dot Y=V\sin(\psi+\beta),\quad \dot\psi=r,"></p>
<p align="center"><img src="equations/eq_52dad0ef354ecd58.svg" width="371" alt="\dot\beta=-r+\frac{L_f\cos(\delta_f-\beta)+L_r\cos(\delta_r-\beta)-F_{\rm dr}\sin\beta}{mV},"></p>
<p align="center"><img src="equations/eq_db551e4eff1fbcc9.svg" width="444" alt="\dot r=M_z/I_z,\qquad \dot V=\frac{L_f\sin(\beta-\delta_f)+L_r\sin(\beta-\delta_r)+F_{\rm dr}\cos\beta}{m}."></p>
The paper repeats its printed yaw form as Eq.(7). The <img src="equations/eq_803a702cde8a210a.svg" width="38" alt="\dot\psi=r" align="middle"> equation is implicit but required to form a first-order system.

#### Slip and linear tire: printed p.5/PDF 4, Eqs.(9)-(13)

<p align="center"><img src="equations/eq_6a71b3586e2443f8.svg" width="325" alt="L_i=c_{\alpha i}\alpha_i,\qquad \alpha_i=\delta_i-\arctan\frac{V\sin\beta+r x_i}{V\cos\beta-r y_i}."></p>
The report explicitly gives all four wheel denominators <img src="equations/eq_317bbf32e8e066d9.svg" width="115" alt="V\cos\beta\mp w_{f/r}r/2" align="middle">. It uses plain <img src="equations/eq_f74a85b94324604d.svg" width="42" alt="\arctan" align="middle"> of a quotient, not a quadrant-aware <img src="equations/eq_769928ef7af762f7.svg" width="37" alt="\operatorname{atan2}" align="middle">. An implementation must either restrict positive wheel-longitudinal speed or state its quadrant/low-speed modification.

#### Nonlinear tire: printed p.6/PDF 5, Eqs.(14)-(17)

<p align="center"><img src="equations/eq_9a96f2e23d8b48e1.svg" width="418" alt="L_i=L_i(\alpha_i,\mu,F_{zi}),\qquad S_{{\rm res},i}=\sqrt{2-2\cos\alpha_i}=2&#124;\sin(\alpha_i/2)&#124;,"></p>
<p align="center"><img src="equations/eq_8736464c5d80c37b.svg" width="422" alt="\mu_{{\rm res},i}=C_1(1-e^{-C_2S_{{\rm res},i}})-C_3S_{{\rm res},i}, \qquad L_i=\frac{S_{Y,i}}{S_{{\rm res},i}}\mu_{{\rm res},i}F_{zi}."></p>
This is **not Pacejka**. Calling it Burckhardt-type describes the functional form; the excerpt does not name it.
- <img src="equations/eq_51fb18a2bfa6c174.svg" width="20" alt="S_Y" align="middle"> is described in words as lateral sliding, but no equation is supplied.
- <img src="equations/eq_2292dabff13900c8.svg" width="18" alt="F_z" align="middle"> is said to depend on <img src="equations/eq_f8230d746ea8cd84.svg" width="64" alt="F_y=ma_y" align="middle">, but no load closure is supplied. This could create an algebraic feedback requiring explicit solution, but the excerpt does not say how.
- Setting CG height to zero (p.3) removes ordinary height-induced quasi-static load transfer in the planar mechanics. Its earlier prose about differential wheel load therefore does not by itself supply a valid load-transfer model.
- At <img src="equations/eq_17dca607a0b33b75.svg" width="37" alt="\alpha=0" align="middle">, <img src="equations/eq_4be691b13684f747.svg" width="52" alt="S_{\rm res}=0" align="middle"> and the written force ratio is <img src="equations/eq_84742adc7c87d9ad.svg" width="23" alt="0/0" align="middle">. A continuous extension must be specified. If one additionally assumes <img src="equations/eq_53bc91aa14bad05a.svg" width="69" alt="S_Y=\sin\alpha" align="middle">, then the limit is zero and near-zero stiffness is <img src="equations/eq_7235ed910cba8b59.svg" width="97" alt="(C_1C_2-C_3)F_z" align="middle">, but that assumption is **not in the excerpt**.
- No combined-slip longitudinal force model, wheel torque equation, or friction ellipse is provided.
- No initial values, boundary conditions, actuator bounds, or optimal-control constraints are supplied in the excerpt.

#### Other factual wording errors

Printed p.4 describes yaw rate as "angular momentum ... also torque." These are different quantities: <img src="equations/eq_2f05b2276c5b569f.svg" width="9" alt="r" align="middle"> is angular velocity, <img src="equations/eq_0bc7608f1a233f27.svg" width="22" alt="I_zr" align="middle"> is angular momentum, <img src="equations/eq_89e8cfe27fb7bb22.svg" width="22" alt="I_z\dot r" align="middle"> is torque in the planar model. Large yaw rate alone is not a general instability criterion; steady tight turning can have high yaw rate. Large sideslip alone is likewise not a complete stability test.

### Lienkamp  -  audited equations and source map

#### Assumptions and notation

- Slide/PDF 10: combine left/right wheels by axle, CG at road level, no roll/pitch/vertical dynamics, no left/right normal-load difference, constant longitudinal speed in the basic model, small angles.
- Slide/PDF 13: states <img src="equations/eq_61b4427000f9f15e.svg" width="35" alt="(\beta,r)" align="middle">, inputs vehicle speed <img src="equations/eq_aa5c572ff666e727.svg" width="13" alt="V" align="middle"> and steering <img src="equations/eq_b6a8371ae1fa1d0b.svg" width="15" alt="\delta_f" align="middle">.
- Slide/PDF 14: counterclockwise angles positive; slip angle convention chosen so positive slip induces positive lateral force.
- Slides/PDF 17-18: Newton/Euler planar balances.
- Slide/PDF 20 Eqs.(2-4),(2-5): approximate slips <img src="equations/eq_89e007097eeb591b.svg" width="244" alt="\alpha_f=\delta-\beta-l_fr/V,\ \alpha_r=-\beta+l_rr/V" align="middle">. The footnote says this small-angle slip approximation is used also in the nonlinear single-track treatment, while the exact relation is used later in the double-track model.
- Slides/PDF 23,58,60,63 summarize four variants, not one universal "Lienkamp model."

#### Constant-speed linear bicycle: slide/PDF 24, Eqs.(2-6)-(2-9)

<p align="center"><img src="equations/eq_7de5971fb0ea5597.svg" width="376" alt="F_{yf}=C_f(\delta-\beta-l_fr/V),\qquad F_{yr}=C_r(-\beta+l_rr/V),"></p>
<p align="center"><img src="equations/eq_4f29e88d1f8a5d52.svg" width="397" alt="\dot\beta=-r+(F_{yf}+F_{yr})/(mV),\qquad \dot r=(l_fF_{yf}-l_rF_{yr})/I_z."></p>
Equivalently
<p align="center"><img src="equations/eq_00ee02f4a05ab5ae.svg" width="561" alt="\binom{\dot\beta}{\dot r}= \begin{pmatrix} -(C_f+C_r)/(mV)&amp;-1+(C_rl_r-C_fl_f)/(mV^2)\\ (C_rl_r-C_fl_f)/I_z&amp;-(C_fl_f^2+C_rl_r^2)/(I_zV) \end{pmatrix}\binom{\beta}{r} +\binom{C_f/(mV)}{l_fC_f/I_z}\delta."></p>
Kinematics <img src="equations/eq_a095b2fe11509071.svg" width="46" alt="X,Y,\psi" align="middle">, longitudinal control, steering dynamics etc. added for a lap simulator are **benchmark augmentations**, not states of this original two-state lateral model.

#### Nonlinear bicycle families: slides/PDF 59,62,64

Using <img src="equations/eq_9de269bcb1f9c39e.svg" width="53" alt="F_{yf},F_{yr}" align="middle"> from either linear or empirically varying cornering stiffness, the general variable-speed version on p.64 is
<p align="center"><img src="equations/eq_23325cc2f0c66989.svg" width="512" alt="\dot\beta=-r+\frac{F_{yf}\cos(\delta-\beta)+F_{yr}\cos\beta-F_{\rm dr}\sin\beta}{mV}, \quad \dot r=\frac{l_fF_{yf}\cos\delta-l_rF_{yr}}{I_z},"></p>
<p align="center"><img src="equations/eq_22b02a31a710a15f.svg" width="297" alt="\dot V=\frac{F_{yf}\sin(\beta-\delta)+F_{yr}\sin\beta+F_{\rm dr}\cos\beta}{m}."></p>
- p.59 Eqs.(2-21),(2-22): finite trigonometric body dynamics, linear <img src="equations/eq_fef7036357d26f22.svg" width="43" alt="C_f,C_r" align="middle">, constant speed.
- p.62 Eqs.(2-23),(2-24): <img src="equations/eq_ed6eacd1436fd145.svg" width="95" alt="F_y=c_\alpha(\alpha,?)\alpha" align="middle">, nonlinear tire characteristic, constant speed.
- p.64: adds drive force/variable speed.
- p.65 Eqs.(2-25)-(2-27): expanded stiffness form, with two apparent errors compared to p.64:
  1. Eq.(2-25) prints <img src="equations/eq_63e6cc8556772477.svg" width="65" alt="+F_{\rm dr}\sin\beta" align="middle">; projection of the body-x drive force onto the velocity normal gives **minus**, as already printed on p.64.
  2. Eq.(2-26) prints <img src="equations/eq_20c6aa5c95a02907.svg" width="67" alt="l_rF_{yr}\cos\beta" align="middle">; body-y rear force with unsteered rear wheel has yaw lever <img src="equations/eq_98ac4048d5de027b.svg" width="12" alt="l_r" align="middle">, with **no <img src="equations/eq_29016d3d4c40cbff.svg" width="32" alt="\cos\beta" align="middle">**. p.64 is consistent.
These were checked visually using a page render.
- p.66 uses example parameters <img src="equations/eq_29bcf344582e04a1.svg" width="80" alt="m=1550\,{\rm kg}" align="middle">, <img src="equations/eq_0bab4b0e37cfd075.svg" width="96" alt="l_f=l_r=1.3\,{\rm m}" align="middle">, <img src="equations/eq_fb0aad326e03b903.svg" width="62" alt="c_w=0.25" align="middle">, frontal area <img src="equations/eq_81f51afc288e2f7b.svg" width="30" alt="2\,{\rm m^2}" align="middle">, <img src="equations/eq_db138c394b778dd9.svg" width="68" alt="R=100\,{\rm m}" align="middle">, <img src="equations/eq_3f3cf3f6785478e1.svg" width="152" alt="C_f=C_r=20000\,{\rm N/rad}" align="middle">. Those are teaching-example values, not native OCD vehicle identification.
- The uploaded slides never define MF_SIMPLE, Burckhardt coefficients, a calibrated load-sensitive tire law, or the native 33-state system.

### Native OCD identity details verified at pin

#### Actual state vector

Vehicle subsystem 28 states:
<p align="center"><img src="equations/eq_80bd8aea91e4ed6b.svg" width="372" alt="(X,Y,z,\psi,\phi,\theta,u,v_y,v_z,r,p,q, z_{w,1:4},\dot z_{w,1:4},\bar\kappa_{1:4},\bar\alpha_{1:4})."></p>
Selected full composition adds <img src="equations/eq_341e76b3d8bc1560.svg" width="39" alt="\omega_{1:4},\delta" align="middle">: total 33. Here <img src="equations/eq_956a236dff335069.svg" width="77" alt="p=\dot\phi,q=\dot\theta" align="middle">.

#### Shared planar skeleton

<p align="center"><img src="equations/eq_55a67c939db49ffc.svg" width="391" alt="\dot X=u\cos\psi-v_y\sin\psi,\quad \dot Y=u\sin\psi+v_y\cos\psi,\quad \dot\psi=r,"></p>
<p align="center"><img src="equations/eq_1a5c5529afc13deb.svg" width="349" alt="\dot u=F_x/m+rv_y,\quad \dot v_y=F_y/m-ru,\quad \dot r=M_z/I_z."></p>
All double-track/bicycle versions may share a Newton/Euler skeleton while differing in closure; that does not make them identical.

#### Native slip and delay

Let wheel ground velocity in body frame be <img src="equations/eq_7efe991f1537deab.svg" width="116" alt="(u-r y_i,v_y+r x_i)" align="middle">, rotate to tire frame by <img src="equations/eq_b2f7d2a3112c1fcd.svg" width="62" alt="R(-\delta_i^{eff})" align="middle"> to get <img src="equations/eq_923050ee90835760.svg" width="47" alt="(U_i,V_i)" align="middle">. Toe-out is added with positive left/negative right sign.
<p align="center"><img src="equations/eq_ef7ac3687f365329.svg" width="434" alt="\kappa_i^\star=\operatorname{clip}\left(\frac{R_i\omega_i-U_i}{\max(3,U_i)},-1,1\right),\quad \alpha_i^\star=-\operatorname{atan2}(V_i,\max(1,U_i)),"></p>
<p align="center"><img src="equations/eq_e177e5135b3dcab1.svg" width="289" alt="\dot{\bar\kappa}_i=(\kappa_i^\star-\bar\kappa_i)/T_{\kappa i},\quad \dot{\bar\alpha}_i=(\alpha_i^\star-\bar\alpha_i)/T_{\alpha i},"></p>
<p align="center"><img src="equations/eq_d5f73da13ef5eaaf.svg" width="406" alt="T_{\kappa i}=0.3/\operatorname{clip}(&#124;U_i&#124;,10,100),\quad T_{\alpha i}=0.2/\operatorname{clip}(&#124;U_i&#124;,10,100)."></p>
The constants have the source's SI interpretation (velocity m/s, relaxation lengths m, time seconds). Do not substitute an unregularized textbook definition and label it exact OCD.

#### Native MF_SIMPLE

Define
<p align="center"><img src="equations/eq_a8d1d4e3030abc71.svg" width="440" alt="\mathcal M(s;B,C,D,E)=D\sin\!\left(C\arctan[B s-E(Bs-\arctan(Bs))]\right)."></p>
Then, for symmetric MF_SIMPLE:
<p align="center"><img src="equations/eq_ec99e4bdf1344e21.svg" width="532" alt="F_{xi}^{t}=\lambda_{\mu i}F_{zi}\mathcal M(\bar\kappa_i;B_x,C_x,D_x,E_x), \quad F_{yi}^{t}=\lambda_{\mu i}F_{zi}\mathcal M(\tan\bar\alpha_i;B_y,C_y,D_y,E_y)."></p>
Native evaluates longitudinal and lateral forces independently. **It does not enforce a combined-slip friction circle/ellipse** in MF_SIMPLE. Therefore a resultant force ratio >1 under a circle diagnostic does not prove the simulation is numerically broken; it exposes this tire-law limitation. Native camber argument is supplied but MF_SIMPLE ignores it.
Default declared coefficients: longitudinal <img src="equations/eq_9a6d2ebeee54be2b.svg" width="91" alt="(20,2,1.7,0.7)" align="middle">, lateral <img src="equations/eq_cabde055409ab2d1.svg" width="113" alt="(10,1.6,1.7,-1.9)" align="middle">, but benchmark may override them; do not imply these are benchmark values without checking configuration.
At rest the vehicle subsystem sets tire forces to zero when squared body velocities and all squared wheel speeds are individually <0.001.

#### Native vertical closures

Native has sprung mass <img src="equations/eq_1a1968c4ca91c442.svg" width="155" alt="m_s=m-2m_{uf}-2m_{ur}" align="middle">, roll/pitch/heave, four unsprung vertical oscillators, suspension springs/dampers, antiroll bars, geometric jacking/squat/dive. Its normal load comes from tire spring compression:
<p align="center"><img src="equations/eq_fc6c347d74ac3199.svg" width="456" alt="z^0_{t,f}=\frac{m g l_r}{2L c_{tf}},\quad z^0_{t,r}=\frac{m g l_f}{2L c_{tr}},\qquad F_{zi}=\max\{c_{ti}(z^0_{ti}-z_{wi}),0.001\}."></p>
This is materially different from a static or algebraic quasi-static load estimate.

#### Model identity comparison table

| Feature | Internal report excerpt | Supplied Lienkamp | Selected native OCD |
|---|---|---|---|
| Contact patches | Four | Two equivalent axle contacts | Four |
| Core motion | Planar yaw, speed, sideslip | Two-state lateral or nonlinear bicycle family | Planar + roll/pitch/heave + unsprung vertical motion |
| Full state count | 6 after adding yaw state | 2 for basic constant-speed model | 33 selected composition |
| Normal-load law | Mentioned but omitted | No left/right difference in basic model | Tire/suspension/unsprung dynamics |
| Tire law | Linear or exponential Burckhardt-type lateral law | Linear or unspecified varying stiffness curve | MF_SIMPLE pure longitudinal/lateral |
| Wheel rotation | Absent | Absent in shown bicycle | Four wheel spin states |
| Tire relaxation | Absent | Absent | Eight slip-relaxation states |
| Steering | Front/rear angle inputs | Front steering input | One PT1 steering state in selected composition |
| Aerodynamics | Not in provided equations | Discussion/example resistance | DEFAULT aero module |
| Fully specified supplied implementation? | No, excerpt incomplete | Not for all nonlinear tire curves | Yes, pinned C++ source plus parameter configuration |



## Thesis car and tire equations


The symbols below are harmonized for comparison; equation numbers remain those of the original thesis. A harmonized symbol is not a claim that the source vehicle has the common benchmark parameters.

### Liniger: complete selected nonlinear bicycle

Source: §3.1, Eqs3.1-3.2, printed18-19. Let `a=lf`, `b=lr`, yaw `psi`, yaw rate `r`, body longitudinal/lateral velocities `u,v`. Original state is `(X,Y,psi,u,v,r)` and control is `(d,delta)`.

<p align="center"><img src="equations/eq_cf4aef9309dcb00a.svg" width="422" alt="\begin{aligned} \dot X&amp;=u\cos\psi-v\sin\psi,&amp; \dot Y&amp;=u\sin\psi+v\cos\psi,&amp; \dot\psi&amp;=r,\\ m\dot u&amp;=F_{xr}-F_{yf}\sin\delta+mvr,\\ m\dot v&amp;=F_{yr}+F_{yf}\cos\delta-mur,\\ I_z\dot r&amp;=aF_{yf}\cos\delta-bF_{yr}. \end{aligned}"></p>

<p align="center"><img src="equations/eq_8bd49fd430ffd111.svg" width="495" alt="\begin{aligned} \alpha_f&amp;=\delta-\arctan\!\left(\frac{v+ar}{u}\right),&amp; F_{yf}&amp;=D_f\sin\!\left(C_f\arctan(B_f\alpha_f)\right),\\ \alpha_r&amp;=\arctan\!\left(\frac{br-v}{u}\right),&amp; F_{yr}&amp;=D_r\sin\!\left(C_r\arctan(B_r\alpha_r)\right),\\ F_{xr}&amp;=(C_{m1}-C_{m2}u)d-C_{rr}-C_du^2. \end{aligned}"></p>

The displayed source model is for forward motion `u>0`; `u=0` requires a numerical continuation/another model. It has no explicit vertical-load, wheel-spin, pitch/roll, tire-relaxation or combined-slip state. No terminal/boundary constraints belong intrinsically to this ODE. In the source tracking MPC, the current estimated state initializes each horizon, with track/input admissibility and a trajectory-tracking objective; the benchmark uses its own common tracking controller.

**Benchmark additions:** seventh state `delta`, PT1 steering target-to-angle law, full-size common-car geometry/mass/inertia and axle tire coefficients, a wheel-torque-to-rear-force mapping, and shared aerodynamic/resistance terms instead of the source RC motor calibration. These need be stated in the implementation section rather than attributed to Eq3.2c.

### Gao: complete published selected four-wheel subsystem

Source: §2.2, Eqs2.5-2.11, printed11-13/PDF24-26. Original ten states are body u,v,heading,yaw rate,global X,Y and four wheel rates. Original inputs are one front steering angle and four wheel torques.

For wheel `i` at body coordinates `(x_i,y_i)`:

<p align="center"><img src="equations/eq_9102d40a4fc61b4d.svg" width="583" alt="(x_{fl},y_{fl})=(a,c),\quad(x_{fr},y_{fr})=(a,-c),\quad (x_{rl},y_{rl})=(-b,c),\quad(x_{rr},y_{rr})=(-b,-c)."></p>

<p align="center"><img src="equations/eq_077fd3642e0b3c30.svg" width="416" alt="\begin{aligned} V_{xi}&amp;=u-r y_i,&amp;V_{yi}&amp;=v+r x_i,\\ V_{li}&amp;=V_{xi}\cos\delta_i+V_{yi}\sin\delta_i,&amp; V_{ci}&amp;=-V_{xi}\sin\delta_i+V_{yi}\cos\delta_i,\\ \alpha_i&amp;=\arctan(V_{ci}/V_{li}),&amp; \delta_{fl}&amp;=\delta_{fr}=\delta_f,\quad\delta_{rl}=\delta_{rr}=0. \end{aligned}"></p>

<p align="center"><img src="equations/eq_ff824ca061a40855.svg" width="335" alt="s_i=\begin{cases} R_w\omega_i/V_{li}-1,&amp;V_{li}&gt;R_w\omega_i\quad\text{(braking)},\\ 1-V_{li}/(R_w\omega_i),&amp;V_{li}&lt;R_w\omega_i\quad\text{(driving)}. \end{cases}"></p>

The corresponding nonzero denominators are required. At rolling equality use `s_i=0`. For forward positive speeds this is `(Rw omega_i - Vli)/max(Vli,Rw omega_i)`; it differs from Beal's velocity-denominator slip and from OCD's velocity-denominator target, which is additionally clipped to [-1,1].

<p align="center"><img src="equations/eq_8ed693a7ee73ab9e.svg" width="426" alt="\begin{aligned} F_{xi}&amp;=F_{li}\cos\delta_i-F_{ci}\sin\delta_i,\\ F_{yi}&amp;=F_{li}\sin\delta_i+F_{ci}\cos\delta_i,\\ m\dot u&amp;=mvr+\sum_iF_{xi},\\ m\dot v&amp;=-mur+\sum_iF_{yi},\\ I_z\dot r&amp;=\sum_i(x_iF_{yi}-y_iF_{xi}),\\ \dot X&amp;=u\cos\psi-v\sin\psi,\qquad \dot Y=u\sin\psi+v\cos\psi,\qquad\dot\psi=r,\\ I_w\dot\omega_i&amp;=T_i-R_wF_{li}-b_w\dot\omega_i. \end{aligned}"></p>

The last line is **as printed** in Eq2.5. The coefficient is described as damping although it multiplies `omega_dot`. With `b_w=0`, both literal and conventional viscous-damping interpretations coincide. Otherwise an explicit modeling correction is needed.

<p align="center"><img src="equations/eq_1d76bad85d03826e.svg" width="377" alt="F_{z,fl}=F_{z,fr}=\frac{bmg}{2(a+b)},\qquad F_{z,rl}=F_{z,rr}=\frac{amg}{2(a+b)}."></p>

The source force functions are `Fci=fc(alpha_i,s_i,mu,Fzi)` and `Fli=fl(alpha_i,s_i,mu,Fzi)`. The **published pure-slip** Pacejka formula (§2.4, Eqs2.21-2.22, p16/PDF29) is

<p align="center"><img src="equations/eq_d9e2aea20ba2a02d.svg" width="619" alt="\mathcal F(\xi)=D\sin\!\left[C\arctan\!\left(B\Theta(\xi)\right)\right]+S_v, \qquad \Theta(\xi)=(1-E)(\xi+S_h)+\frac{E}{B}\arctan\!\left(B(\xi+S_h)\right)."></p>

Here `xi=s` for longitudinal force or `xi=alpha` for lateral force, with signs set consistently with the source's opposing lateral force convention. The source explicitly does not print its combined-slip extension. Therefore the complete exact combined-slip Gao plant cannot be reconstructed from these pages alone; independent pure-slip forces are a declared closure of the benchmark.

#### Gao's separate modified-Fiala alternative

Source Eqs2.23-2.24, p18 and21/PDF31 and34. Set

<p align="center"><img src="equations/eq_4318d630ba4594ea.svg" width="556" alt="D_y=\sqrt{(\mu F_z)^2-F_l^2}=\eta\mu F_z,\qquad \eta=\frac{\sqrt{(\mu F_z)^2-F_l^2}}{\mu F_z},\qquad \alpha_{sl}=\arctan\!\left(\frac{3D_y}{C_\alpha}\right)."></p>

The conventional odd-symmetric form is

<p align="center"><img src="equations/eq_f47a442fa3081ed4.svg" width="422" alt="F_c=\begin{cases} -C_\alpha t+\dfrac{C_\alpha^2}{3D_y}&#124;t&#124;t -\dfrac{C_\alpha^3}{27D_y^2}t^3,&amp;&#124;\alpha&#124;&lt;\alpha_{sl},\\ -D_y\operatorname{sgn}\alpha,&amp;&#124;\alpha&#124;\ge\alpha_{sl}, \end{cases} \qquad t=\tan\alpha."></p>

This displayed conventional form makes a **declared correction**: original Eq2.23 visibly prints `t^2` in the final term. Positive `D_y` and `|Fl|<=mu Fz` are required; zero-availability limit must be handled separately. This is a lateral force law taking longitudinal force as an input, not an independent formula for longitudinal slip force. It is not the selected benchmark Gao tire if the benchmark uses the source's published pure MF.

### Beal: complete selected coupled-brush and yaw-roll equations

Source tire Eq2.7, p25/PDF36. Let `mu` be its adhesion coefficient, `mu_s` sliding coefficient, `C_x,C_alpha` longitudinal/cornering stiffness, wheel-frame slip angle `alpha=atan(Vc/Vl)` and

<p align="center"><img src="equations/eq_f6ce5cb6dc1a935c.svg" width="517" alt="\kappa=\frac{R_w\omega-V_l}{V_l},\qquad q_x=C_x\frac{\kappa}{1+\kappa},\qquad q_y=-C_\alpha\frac{\tan\alpha}{1+\kappa},\qquad \gamma=\sqrt{q_x^2+q_y^2}."></p>

<p align="center"><img src="equations/eq_d59211b2682e2e13.svg" width="637" alt="F(\gamma)=\begin{cases} \gamma-\dfrac{2-\mu_s/\mu}{3\mu F_z}\gamma^2 +\dfrac{1-2\mu_s/(3\mu)}{9\mu^2F_z^2}\gamma^3, &amp;\gamma\le3\mu F_z,\\ \mu_sF_z,&amp;\gamma&gt;3\mu F_z, \end{cases} \qquad F_x=\frac{q_x}{\gamma}F(\gamma),\quad F_y=\frac{q_y}{\gamma}F(\gamma)."></p>

At `gamma=0`, force is zero by continuous extension. This form assumes `Vl!=0`, `1+kappa!=0`, and positive contact load. Floors/caps at low speed are numerical implementation choices. Peak normalization for comparisons must use the actual maximum of this polynomial, not just the parameter named `mu`.

Define `Rmu=mu_s/mu`, `q=1/(1-2 Rmu/3)`. The internal-force peak is

<p align="center"><img src="equations/eq_d6175ed14a121dd3.svg" width="316" alt="F_{peak}=\mu F_z\left[q-\frac{2-R_\mu}{3}q^2 +\frac{1-2R_\mu/3}{9}q^3\right]."></p>

For `Rmu=11/12`, bracket=`45/49`; choosing internal `mu=49/45` makes actual resultant peak `Fz`. Then sliding `mu_s=539/540`, not exactly1.

For the full yaw-roll model use source roll-base velocities `U,V`, roll `phi`, roll rate `p`, yaw rate `r`, CG height above the roll axis `h` (source h-prime), mass `m`, principal inertias `Ix,Iy,Iz`, suspension `Kphi,bphi`. Source Eq2.10, p27/PDF38, is equivalently

<p align="center"><img src="equations/eq_598dbad0e616ff9e.svg" width="519" alt="\begin{aligned} \dot r&amp;=\frac{Q_\psi-Q_xh\sin\phi -2(I_y-I_z)rp\sin\phi\cos\phi} {I_y\sin^2\phi+I_z\cos^2\phi},\\ \dot p&amp;=\frac{Q_yh\cos\phi-K_\phi\phi-b_\phi p+mgh\sin\phi +[(I_y-I_z)r^2-mh^2p^2]\sin\phi\cos\phi} {I_x+mh^2\sin^2\phi},\\ \dot U&amp;=\frac{Q_x}{m}+rV-h\dot r\sin\phi-2hrp\cos\phi,\\ \dot V&amp;=\frac{Q_y}{m}-rU+h\dot p\cos\phi-h(r^2+p^2)\sin\phi,\\ \dot\phi&amp;=p,\qquad\dot\psi=r. \end{aligned}"></p>

For parallel front steering and wheel-fixed forces (subscripts `fl,fr,rl,rr`), let `a,b` be front/rear axle distances from the CG and `d` the full source track width. Source generalized forces are

<p align="center"><img src="equations/eq_472e1dd9458d94c3.svg" width="472" alt="\begin{aligned} Q_x&amp;=(F_{x,fl}+F_{x,fr})\cos\delta -(F_{y,fl}+F_{y,fr})\sin\delta+F_{x,rl}+F_{x,rr},\\ Q_y&amp;=(F_{y,fl}+F_{y,fr})\cos\delta +(F_{x,fl}+F_{x,fr})\sin\delta+F_{y,rl}+F_{y,rr},\\ Q_\psi&amp;=a[(F_{y,fl}+F_{y,fr})\cos\delta +(F_{x,fl}+F_{x,fr})\sin\delta]-b(F_{y,rl}+F_{y,rr})\\ &amp;\quad-\frac{d}{2}[(F_{x,fl}-F_{x,fr})\cos\delta -(F_{y,fl}-F_{y,fr})\sin\delta+F_{x,rl}-F_{x,rr}]. \end{aligned}"></p>

The benchmark should publish the adopted wheel-spin ODE, normal-load distribution, aero moments and shared parameters separately: these are not fully specified by Eq2.10. The conversion needed when logging/initializing CG velocities is

<p align="center"><img src="equations/eq_302fb98b484f46b3.svg" width="313" alt="u_{CG}=U+hr\sin\phi,\qquad v_{CG}=V-hp\cos\phi,"></p>

The corresponding CG pose kinematics are

<p align="center"><img src="equations/eq_11a810841c4ff848.svg" width="412" alt="\dot X=u_{CG}\cos\psi-v_{CG}\sin\psi,\qquad \dot Y=u_{CG}\sin\psi+v_{CG}\cos\psi."></p>

The initial state must include consistent roll/base velocities and wheel rates. Source pure model has12 states if global pose plus3 planar velocities plus4 wheel rates plus2 roll states are retained; common PT1 steering gives13.

#### Beal's MPC predictor is a different reduced model

Source Eqs2.15-2.16 pp30/PDF41 and AFI2.18-2.21 p32/PDF43:

<p align="center"><img src="equations/eq_fb3a5d84e2b96e78.svg" width="568" alt="\dot\beta=\frac{F_{yf}+F_{yr}}{mU_x}-r,\qquad \dot r=\frac{aF_{yf}-bF_{yr}}{I_z},\qquad \alpha_f=\beta+\frac{ar}{U_x}-\delta,\qquad \alpha_r=\beta-\frac{br}{U_x}."></p>

For AFI, control is `Fyf` and

<p align="center"><img src="equations/eq_b4eef4e87a736dbe.svg" width="190" alt="F_{yr}=\bar F_{yr}-\widetilde C_{\alpha r}(\alpha_r-\bar\alpha_r)."></p>

The local quantities are updated from the current operating point. For fixed positive speed this gives an affine system in `(beta,r)` and `Fyf`. Envelope constraints, front force/input bounds and a driver-intent objective are central to the original controller. Running the nonlinear Eq2.10 plant with a shared new controller does not reproduce that source controller.

### Sun: complete kinematic model and distinct eco-driving model

Chapter5 Eq5.1, p106, original FIVE-state model:

<p align="center"><img src="equations/eq_810c17efa1cb8107.svg" width="274" alt="\beta=\arctan\!\left(\frac{l_r}{L}\tan\delta\right),\qquad L=l_f+l_r,"></p>

<p align="center"><img src="equations/eq_c1f720e92620b99e.svg" width="335" alt="\begin{aligned} \dot X&amp;=v\cos(\psi+\beta),&amp; \dot Y&amp;=v\sin(\psi+\beta),\\ \dot\psi&amp;=\frac{v\cos\beta\tan\delta}{L},&amp; \dot v&amp;=a,&amp;\dot\delta&amp;=\nu. \end{aligned}"></p>

`a,nu` are acceleration and steering-rate controls. `beta` is algebraic geometry, not an independently evolving tire slip. There is no tire-force law, so measured tire utilization/slip metrics are N/A. Original path constraints are inter-vehicle/obstacle separation, non-position state/input admissible sets, and terminal position ball in Eq5.11 p112. The primary model should not be interpreted as valid at handling saturation solely because it has small tracking error.

Chapter3 uses a separate longitudinal model, Eqs3.4-3.6 pp43-44:

<p align="center"><img src="equations/eq_d057e44673bf071b.svg" width="385" alt="\dot s=v,\qquad m\dot v=\frac{2\eta_tT}{R_w}-C_dv^2-mg\sin\vartheta-mgC_f\cos\vartheta."></p>

<p align="center"><img src="equations/eq_09e66ae9de4db435.svg" width="434" alt="E=\frac12mv^2,\qquad\frac{d}{ds}=\frac1v\frac{d}{dt},\qquad \frac{d\Delta t}{ds}=\frac1{\sqrt{2E/m}}-\frac1{\sqrt{2E_0/m_0}},"></p>

<p align="center"><img src="equations/eq_608fa4e85ccfca8c.svg" width="401" alt="\frac{dE}{ds}=\frac{2\eta_tT}{R_w}-\frac{2C_d}{m}E-mg\sin\vartheta-mgC_f\cos\vartheta, \qquad E&gt;0."></p>

The factor2 reflects the source vehicle's two motors. This `Cd` is the coefficient multiplying speed squared directly; it must not be mistaken for a dimensionless aerodynamic Cd without area/density. Initial energy/headway, torque bounds, speed limits and following-headway bounds close the eco-driving task. These equations alone contain no lateral vehicle motion.

### Liu: full selected 3D elastic ring PDE system

Source §§2.1-2.2 Eqs2.16-2.19, printed20-21/PDF40-41; inspected both text and rendered pages. Fields are radial `u(theta,t)`, circumferential `v(theta,t)`, axial `w(theta,t)`, and circumferential-axis torsional rotation `phi_theta(theta,t)`. A prime is partial differentiation in theta; a dot is time. The subscript on `phi_theta` is its name, not differentiation. Constant rolling speed is `Omega`, angular domain theta in[0,2pi].

Prestress (Eq2.11, printed19/PDF39):

<p align="center"><img src="equations/eq_5ca42532ecafe04d.svg" width="170" alt="\sigma_\theta^0 A=p_0bR+\rho AR^2\Omega^2."></p>

Radial equation2.16:

<p align="center"><img src="equations/eq_d906c05654c30986.svg" width="425" alt="\begin{aligned} \frac{EA}{R^2}(u+v&#x27;)+\frac{EI_z}{R^4}(u^{(4)}-v&#x27;&#x27;&#x27;) +\frac{\sigma_\theta^0 A}{R^2}(u+2v&#x27;-u&#x27;&#x27;)\\ +\rho A(\ddot u-2\Omega\dot v-\Omega^2u) -\frac{bp_0}{R}(u+v&#x27;)+k_uu+c_u\dot u =q_u+\frac1R q_\beta&#x27;. \end{aligned}"></p>

Circumferential equation2.17:

<p align="center"><img src="equations/eq_cbf9fd034636f74b.svg" width="420" alt="\begin{aligned} -\frac{EA}{R^2}(u&#x27;+v&#x27;&#x27;)+\frac{EI_z}{R^4}(u&#x27;&#x27;&#x27;-v&#x27;&#x27;) +\frac{\sigma_\theta^0 A}{R^2}(v-2u&#x27;-v&#x27;&#x27;)\\ +\rho A(\ddot v+2\Omega\dot u-\Omega^2v) +\frac{bp_0}{R}(u&#x27;-v)+k_vv+c_v\dot v =q_v+\frac1R q_\beta. \end{aligned}"></p>

Axial equation2.18:

<p align="center"><img src="equations/eq_e9cb09a8a3747f91.svg" width="333" alt="\begin{aligned} \frac{EI_r}{R^3}\left(\frac{w^{(4)}}R-\phi_\theta&#x27;&#x27;\right) -\frac{GI_\rho}{R^3}\left(\frac{w&#x27;&#x27;}R+\phi_\theta&#x27;&#x27;\right) +\rho A\ddot w\\ -\frac{\rho I_z}{R}\ddot\phi_\theta +\Omega\frac{\rho I_r}{R}\dot\phi_\theta&#x27; +k_ww+c_w\dot w=q_w+\frac1R q_{Mz}&#x27;. \end{aligned}"></p>

Torsion equation2.19:

<p align="center"><img src="equations/eq_6c5589e2a0750c3e.svg" width="330" alt="\begin{aligned} \frac{EI_r}{R^2}\left(\phi_\theta-\frac{w&#x27;&#x27;}R\right) -\frac{GI_\rho}{R^2}\left(\phi_\theta&#x27;&#x27;+\frac{w&#x27;&#x27;}R\right) +\rho I_\rho\ddot\phi_\theta\\ -\frac{\rho I_z}{R}\ddot w+\Omega\frac{\rho I_r}{R}\dot w&#x27; +k_u\frac{b_{op}^2}{4}\phi_\theta +c_u\frac{b_{op}^2}{4}\dot\phi_\theta=q_{Mx}. \end{aligned}"></p>

Here `R` is mean ring radius, `A,rho` area/density, `E,G` moduli, `EIz,EIr` bending rigidities, `GI_rho` torsional rigidity, `ku,kv,kw` foundation stiffness, `cu,cv,cw` foundation damping, `p0` inflation pressure, `b,bop` effective/nominal widths, and `q` distributed applied forces/moments. Source notation `I_rho` is printed in equations while surrounding prose uses `I_p`. Retain it rather than silently changing indices.

A closed ring requires periodic fields and all derivatives needed in the equations:

<p align="center"><img src="equations/eq_24b75d0d148284ed.svg" width="323" alt="q(\theta+2\pi,t)=q(\theta,t),\qquad \partial_\theta^j q(0,t)=\partial_\theta^j q(2\pi,t)"></p>

for the appropriate derivative orders; this `q` denotes any displacement/rotation field, not external loading. Initial fields are

<p align="center"><img src="equations/eq_c3fa79f9e64096ca.svg" width="382" alt="(u,v,w,\phi_\theta)(\theta,0)=q_0(\theta),\qquad (\dot u,\dot v,\dot w,\dot\phi_\theta)(\theta,0)=\dot q_0(\theta)."></p>

External distributed loads and the road/contact problem must be specified. For free-vibration analysis use zero external loads, appropriate equilibrium/prestress and modal fields; for forced rolling contact use the applicable contact law. Initial and periodic conditions are mathematical closure, not a substitute for contact/friction.

#### Liu's separate rigid-ring/rim ODEs

Source3.110-3.114, printed45/PDF65. Rim variables `xa,za,theta_a`; belt/ring variables `xb,zb,theta_b`. At prescribed rolling speed `xa` is supplied, leaving five second-order ODEs:

<p align="center"><img src="equations/eq_65b3636d1798c052.svg" width="515" alt="\begin{aligned} m_b\ddot x_b+c_{bx}(\dot x_b-\dot x_a)+k_{bx}(x_b-x_a) -c_{bx}(\Omega+\dot\theta_a)(z_b-z_a)&amp;=F_{cx},\\ m_b\ddot z_b+c_{bz}(\dot z_b-\dot z_a)+k_{bz}(z_b-z_a) -c_{bz}(\Omega+\dot\theta_a)(x_b-x_a)&amp;=F_{cz},\\ I_{by}\ddot\theta_b+c_{b\theta}(\dot\theta_b-\dot\theta_a) +k_{b\theta}(\theta_b-\theta_a)&amp;=M_{by}+F_{cx}R_e,\\ m_a\ddot z_a+c_{bz}(\dot z_a-\dot z_b)+k_{bz}(z_a-z_b) -c_{bz}(\Omega+\dot\theta_a)(x_a-x_b)&amp;=F_{z0},\\ I_{ay}\ddot\theta_a+c_{b\theta}(\dot\theta_a-\dot\theta_b) +k_{b\theta}(\theta_a-\theta_b)&amp;=M_{ay}. \end{aligned}"></p>

Here `ma,mb` are rim/ring masses; `Iay,Iby` their spin-axis inertias. The `kbx,kbz,kbtheta` and `cbx,cbz,cbtheta` coefficients are sidewall stiffnesses and dampings. `Re` is effective rolling radius, `Fz0` the applied rim vertical load, and `May,Mby` applied rim/ring torques. The contact forces and moment are outputs of the flexible-ring contact subsystem, not arbitrary functions that can be replaced by a Magic Formula without changing the model. A lateral racing-lap model needs a chassis coupling and lateral-contact extension in addition. Thus lap time and handling-MPC runtime are N/A for this thesis subsystem by itself.

### Additional source constraints

Beal's safe handling envelope (Eq3.4, printed47/PDF58) uses the yaw-rate bound (positive force magnitudes)

<p align="center"><img src="equations/eq_fec2462265f49b18.svg" width="675" alt="r_{max}=\frac1{mU_x}\min\!\left\{ F_{yr,max}\left(1+\frac ba\right),\ F_{yf,max}\left(1+\frac ab\right)\right\}, \qquad &#124;r&#124;\le r_{max},\qquad \left&#124;\beta-\frac{b}{U_x}r\right&#124;\le\alpha_{r,max}."></p>

The rear-slip bound is chosen from tire saturation or a deliberately widened envelope; it is not a universal fixed number. Chapter4 implements the envelope with weighted slack, so one must distinguish a designed bound from a guaranteed hard bound in a particular implementation.

Sun kinematic model requires positive wheelbase and a steering range avoiding `cos(delta)=0`. Its formation controller imposes

<p align="center"><img src="equations/eq_61a3956a89415367.svg" width="561" alt="\&#124;p_i-p_j\&#124;\ge d_{ij,min},\qquad \&#124;p_i-p_{obstacle}\&#124;\ge d_{i,obstacle,min},\qquad x_{i,np}\in\mathcal X_i,\quad u_i\in\mathcal U_i,"></p>

with measured-state initialization and terminal position in an admissible reference ball. These are task constraints, whereas the no-slip kinematic relations are the model itself.


## Complete closures of the executed thesis-family adapters

The preceding source equations identify what the authors published. This section fixes every additional closure used in `tracklab/model_comparison/thesis_models.py`. These are explicit normalized modeling choices. They are necessary for reproducibility and are not presented as the authors' vehicle calibration.

### Shared parameters, actuator, integration and observation

All adapters use the common 800 kg car, 3.2 m wheelbase, CG-to-front distance 1.724 m, rear distance 1.476 m, yaw inertia 1000 kg m squared, wheel radius 0.3 m, track widths 1.6/1.5 m, and the native parameter table's drag and rolling coefficients. Static per-wheel gravity loads are

<p align="center"><img src="equations/eq_0f5786ca7eba73aa.svg" width="210" alt="\mathbf F_z^0=mg\left(\frac{l_r}{2L},\frac{l_r}{2L}, \frac{l_f}{2L},\frac{l_f}{2L}\right)."></p>

Per-wheel friction multipliers are one in the recorded experiments. Each adapter accepts bounded steering targets, uses the native section's PT1 rate and magnitude derivative bounds, and publishes observation <img src="equations/eq_7f46e66bbb40bf7b.svg" width="167" alt="(X,Y,\psi,v_{x,CG},v_{y,CG},r,\delta)" align="middle">. Models with wheel rotation share

<p align="center"><img src="equations/eq_8a0769982a111879.svg" width="451" alt="\dot\omega_i=\frac{T_i-R(F_{x,i}^t+c_{rr}F_{z,i})}{J_i+J_{\rm low}}, \qquad J_{\rm low}=\max\left(0,15J_r\left[1-\frac{\overline\omega}{20}\right]\right),"></p>

with the native near-zero negative-derivative suppression. After every RK4 substep the reduced adapter additionally projects wheel speeds to nonnegative values and steering to its magnitude bounds. These finite-step projections are disclosed numerical differences from native derivative clamping. Full internal state persists and can be restored; state reconstruction is not repeated at every time step.

### Liniger adaptation

Use the six body equations of the Liniger source plus the shared steering actuator. With <img src="equations/eq_feb0e05d0c123ed0.svg" width="105" alt="u_+=\max(v_x,1)" align="middle">,

<p align="center"><img src="equations/eq_0b5d58f0b795ef6e.svg" width="410" alt="\alpha_f=\delta-\operatorname{atan2}(v_y+l_fr,u_+),\qquad \alpha_r=\operatorname{atan2}(l_rr-v_y,u_+)."></p>

Let <img src="equations/eq_c3c65d2d80efb813.svg" width="134" alt="F_{zf}^0=F_{z,FL}^0+F_{z,FR}^0" align="middle"> and similarly for the rear; mean axle friction multipliers are <img src="equations/eq_4ea75af3dd434e26.svg" width="39" alt="\lambda_f,\lambda_r" align="middle">. Then

<p align="center"><img src="equations/eq_d9fa88425942c698.svg" width="533" alt="F_{yf}=\mu_0\lambda_f F_{zf}^0\sin[1.6\arctan(10\alpha_f)],\qquad F_{yr}=\mu_0\lambda_r F_{zr}^0\sin[1.6\arctan(10\alpha_r)],"></p>

<p align="center"><img src="equations/eq_f5b00e59cb687c2a.svg" width="348" alt="F_{xr}=\frac{\sum_iT_i}{R}-c_{rr}mg-\tfrac12\rho C_dA v_x&#124;v_x&#124;, \qquad \mu_0=1."></p>

This retains the source's slip-angle argument and zero curvature factor, not the native tangent-of-angle MF. Downforce and acceleration-induced transfer are absent. Rear longitudinal force has no tire-slip saturation; wheel spin does not exist. The logged longitudinal quantity is a net generalized rear force including resistance. Its effective axle-force ratio is reported separately; it is not labeled tire-contact friction utilization.

### Lienkamp adaptation

Use common six-state global/body kinematics with the following scheduled linear force closure and seventh steering state:

<p align="center"><img src="equations/eq_59ab85f791177205.svg" width="486" alt="\alpha_f=\delta-\frac{v_y+l_fr}{u_+},\quad \alpha_r=\frac{l_rr-v_y}{u_+},\quad C_f=16\lambda_fF_{zf}^0,\quad C_r=16\lambda_rF_{zr}^0,"></p>

<p align="center"><img src="equations/eq_c7794503dcd840a6.svg" width="388" alt="F_{yf}=C_f\alpha_f,\qquad F_{yr}=C_r\alpha_r,\qquad W=mg-\tfrac12\rho C_lA v_x^2,"></p>

<p align="center"><img src="equations/eq_624a2fffd71de1a6.svg" width="255" alt="F_x=\frac{\sum_iT_i}{R}-c_{rr}W-\tfrac12\rho C_dA v_x&#124;v_x&#124;,"></p>

<p align="center"><img src="equations/eq_d990fa0f1fdc29f2.svg" width="504" alt="\dot v_x=F_x/m+rv_y,\quad \dot v_y=(F_{yf}+F_{yr})/m-rv_x,\quad \dot r=(l_fF_{yf}-l_rF_{yr})/I_z."></p>

The tire forces remain unsaturated and use static-load-derived stiffness. Downforce affects the added longitudinal rolling-resistance closure only. Speed scheduling, global pose, longitudinal acceleration, and actuator states extend the source's constant-speed lateral model. A contact-circle feasibility claim is not assigned to this linear baseline.

### Gao adaptation

Use four wheel locations from the common native geometry, including different front/rear widths. Rotate base-point contact velocities into each tire frame, giving <img src="equations/eq_923050ee90835760.svg" width="47" alt="(U_i,V_i)" align="middle"> as in the native geometry section. The positive-lateral-force slip convention and the source's bounded driving/braking convention become

<p align="center"><img src="equations/eq_89f3065aff793ead.svg" width="449" alt="\alpha_i^+=-\operatorname{atan2}(V_i,\max(1,U_i)),\qquad \kappa_i=\frac{R\max(\omega_i,0)-U_i} {\max(1,U_i,R\max(\omega_i,0))}."></p>

Use the source's static gravity loads and explicitly independent published pure-slip MF laws:

<p align="center"><img src="equations/eq_b2c3b46f228f291e.svg" width="498" alt="F_{x,i}^t=\lambda_iF_{z,i}^0\mathcal M(\kappa_i;20,2,1,0.7),\qquad F_{y,i}^t=\lambda_iF_{z,i}^0\mathcal M(\alpha_i^+;10,1.6,1,-1.9)."></p>

Here the MF definition is the native section's four-parameter shape; its peak parameter is explicitly one. The lateral argument is the slip angle itself. Horizontal/vertical force shifts are zero. This closes the chosen benchmark model; it does not reconstruct Gao's unprinted combined-slip modification.

Transform forces into body axes and use the six planar Newton-Euler equations, four shared wheel-rate equations and steering equation. Add CG drag in both horizontal directions,

<p align="center"><img src="equations/eq_5de4698028d6bd7a.svg" width="345" alt="F_{a,x}=-\tfrac12\rho C_dA v_x&#124;v_x&#124;,\qquad F_{a,y}=-\tfrac12\rho C_dA v_y&#124;v_y&#124;,"></p>

with no aerodynamic downforce. The printed wheel damping ambiguity is avoided by setting its coefficient to zero. There is no suspension, dynamic load transfer, or tire relaxation in this adapter.

### Beal adaptation: brush and roll-base coordinates

For wheel velocities based on roll-base <img src="equations/eq_de24ac28305c79e0.svg" width="29" alt="U,V" align="middle">, use the same positive-lateral-force angle as above and

<p align="center"><img src="equations/eq_56106f1628705fb2.svg" width="254" alt="\kappa_i=\max\left(-1,\frac{R\max(\omega_i,0)-U_i}{\max(1,U_i)}\right)."></p>

In the source coupled-brush law set

<p align="center"><img src="equations/eq_e5724132fc2bd3c0.svg" width="432" alt="C_{x,i}=40\lambda_iF_{z,i},\quad C_{\alpha,i}=16\lambda_iF_{z,i},\quad \mu_i=\frac{49}{45}\lambda_i,\quad \mu_{s,i}=\frac{11}{12}\mu_i."></p>

Use the source's opposite sign for its slip angle, <img src="equations/eq_ad8ac0cc0ad3322b.svg" width="92" alt="\alpha_i^{\rm source}=-\alpha_i^+" align="middle">. This gives positive lateral force for positive <img src="equations/eq_c0043bc676b18f92.svg" width="20" alt="\alpha_i^+" align="middle">. The radial force peak is exactly <img src="equations/eq_e7ae96e2d0f691ab.svg" width="38" alt="\lambda_iF_{z,i}" align="middle">; sliding force is <img src="equations/eq_023e24fa0bb239c0.svg" width="52" alt="539/540" align="middle"> of that peak. At zero demand the force is zero. At locked wheel, take the finite force-vector direction limit as <img src="equations/eq_00a8731db5a3d0fa.svg" width="57" alt="1+\kappa\downarrow0" align="middle">; do not divide by zero. The implementation forms the direction before this denominator and applies the saturated radial force.

The added roll coefficients are

<p align="center"><img src="equations/eq_e0214b1afe82f56a.svg" width="357" alt="h&#x27;=0.1\ {\rm m},\quad I_x=100,\ I_y=500,\ I_z=1000\ {\rm kg\,m^2},"></p>

<p align="center"><img src="equations/eq_bb71cc7fc68aee08.svg" width="397" alt="K_\phi=\tfrac12(c_f+c_{{\rm ar},f})t_f^2 +\tfrac12(c_r+c_{{\rm ar},r})t_r^2=275835\ {\rm N\,m/rad},"></p>

<p align="center"><img src="equations/eq_da65d9512b3861b2.svg" width="279" alt="b_\phi=\tfrac12d_ft_f^2+\tfrac12d_rt_r^2=2676.5\ {\rm N\,m\,s/rad}."></p>

These numbers come from the common vehicle's spring/damper/antiroll parameters; they are not Beal's P1 calibration. The source roll equations use total benchmark mass. CG observations and pose derivatives use

<p align="center"><img src="equations/eq_e46f94d8af69bbb7.svg" width="341" alt="v_{x,CG}=U+h&#x27;r\sin\phi,\qquad v_{y,CG}=V-h&#x27;p\cos\phi."></p>

For native-to-Beal forecast initialization, copy native pose, yaw rate, roll/rate, four wheel rates and steering, and invert the above velocities. Thus both forecast models start from the same observed CG motion rather than accidentally the same differently defined internal velocities.

#### Beal normal-load closure

The adapter uses mass-conserving quasi-static load transfer independently of roll displacement. Let <img src="equations/eq_72fc1e4591aebd4b.svg" width="86" alt="G_x=\sum_i F_{x,i}^b" align="middle">, <img src="equations/eq_b055eb4b846bfb1a.svg" width="85" alt="G_y=\sum_i F_{y,i}^b" align="middle">, <img src="equations/eq_eee55e48f997f438.svg" width="54" alt="h_c=0.3" align="middle"> m and <img src="equations/eq_2da9b1c97587f333.svg" width="63" alt="\eta_f=l_r/L" align="middle">. Downforce uses CG longitudinal speed:

<p align="center"><img src="equations/eq_dc52504063368869.svg" width="553" alt="D_a=-\tfrac12\rho C_l A v_{x,CG}^2,\quad W=mg+D_a, \quad F_{zf}=W\eta_f-\frac{h_cG_x}{L},\quad F_{zr}=W-F_{zf},"></p>

<p align="center"><img src="equations/eq_c66550be5dc66afa.svg" width="517" alt="F_{z,FL/FR}=F_{zf}/2\mp h_c\eta_fG_y/t_f,\qquad F_{z,RL/RR}=F_{zr}/2\mp h_c(1-\eta_f)G_y/t_r."></p>

This is an added algebraic closure. It does not reproduce the native suspension or a roll-dependent normal-load calculation in Beal's vehicle. The forces and loads depend on each other. At fixed slip, the normalized brush forces are homogeneous in normal load. Write body-force-per-unit-load vectors <img src="equations/eq_a6a8a4e59a79455e.svg" width="36" alt="f_x,f_y" align="middle"> and

<p align="center"><img src="equations/eq_5abfb81d4ccdf1db.svg" width="389" alt="\mathbf F_z=\mathbf F_{z0}+\ell_x a_x^g+\ell_y a_y^g,\qquad a_x^g=G_x/m,\quad a_y^g=G_y/m,"></p>

<p align="center"><img src="equations/eq_9ed53f43ead635c9.svg" width="288" alt="\mathbf F_{z0}=W\boldsymbol\eta,\qquad \ell_x=\frac{mh_c}{2L}(-1,-1,1,1)^\top,"></p>

<p align="center"><img src="equations/eq_b1bebe5aa643cd15.svg" width="290" alt="\ell_y=mh_c\left(-\frac{\eta_f}{t_f},\frac{\eta_f}{t_f}, -\frac{1-\eta_f}{t_r},\frac{1-\eta_f}{t_r}\right)^\top."></p>

Solve the exact two-variable linear system at each ODE stage:

<p align="center"><img src="equations/eq_6943bc24e516e31d.svg" width="361" alt="\begin{pmatrix} 1-f_x^\top\ell_x/m&amp;-f_x^\top\ell_y/m\\ -f_y^\top\ell_x/m&amp;1-f_y^\top\ell_y/m \end{pmatrix} \binom{a_x^g}{a_y^g} =\frac1m\binom{f_x^\top\mathbf F_{z0}}{f_y^\top\mathbf F_{z0}}."></p>

If an unconstrained axle/side load is negative, the axle total is clipped into <img src="equations/eq_f845402be5e20c61.svg" width="39" alt="[0,W]" align="middle"> and left/right transfer is clipped to half its axle total, preserving total weight. A bounded fixed-point iteration then closes the clipped branch; failure is reported. This condition is flagged as leaving the assumed four-contact operating domain. It is not a validated wheel-lift model.

#### Beal generalized aero forces and all derivatives

Apply common quadratic drag and negative vertical lift at the CG. Let <img src="equations/eq_33a9ad00bfbc84d3.svg" width="170" alt="M_{z,t}=\sum_i(a_iF^b_{y,i}-b_iF^b_{x,i})" align="middle">. The generalized loads in the roll-base equations are

<p align="center"><img src="equations/eq_43f802a3947d5831.svg" width="438" alt="Q_x=G_x+F_{a,x},\quad Q_y=G_y+F_{a,y},\quad Q_\psi=M_{z,t}+h&#x27;\sin\phi F_{a,x},"></p>

<p align="center"><img src="equations/eq_52aed861e5d9ebd8.svg" width="246" alt="Q_{\phi,a}=-h&#x27;\cos\phi F_{a,y}-h&#x27;\sin\phi F_{a,z}."></p>

Use the source yaw-roll equations with these <img src="equations/eq_816219676809b89c.svg" width="72" alt="Q_x,Q_y,Q_\psi" align="middle">, and add <img src="equations/eq_d2c752828a0b44fd.svg" width="30" alt="Q_{\phi,a}" align="middle"> to the numerator of the roll-acceleration equation. Then use the displayed CG global pose derivatives, four shared wheel-rate equations, and shared steering derivative. These give all 13 state derivatives. The algebraic load closure uses tire-contact forces, not drag twice. Body drag is not also subtracted from wheel loads as rolling resistance.

### Sun adaptation

Use the complete five-state source kinematic equations, with its controls replaced by

<p align="center"><img src="equations/eq_8135c2d8e982ce00.svg" width="421" alt="a=\frac{\sum_iT_i/R-c_{rr}W(V)-\tfrac12\rho C_dAV&#124;V&#124;}{m}, \qquad \nu=\dot\delta_{\rm shared\ PT1}."></p>

Observation maps the speed into CG body components and geometric yaw rate:

<p align="center"><img src="equations/eq_ca0607988c23554e.svg" width="417" alt="v_{x,CG}=V\cos\beta_g,\quad v_{y,CG}=V\sin\beta_g,\quad r=V\cos\beta_g\tan\delta/L."></p>

A native state generally has independently evolving sideslip/yaw; Sun projects it onto this kinematic manifold using common pose, speed magnitude and steering. This initial model projection is part of the prediction mismatch, not an integration error. Tire-force/slip/load arrays in raw common interfaces are compatibility placeholders and must not be interpreted as physical zero-force predictions. Published tire metrics are N/A.



## Source map and interpretation

The page numbers below refer to the inspected versions, not a generic edition or an assumed PDF offset. Mathematical symbols are harmonized for comparison; source equations and benchmark additions are identified separately.

| Source | Decisive locations | What it establishes |
| --- | --- | --- |
| Uploaded internal report excerpt | Printed 2-6 and 20; six PDF pages. Equations 1-17. | Planar double-track balances and the supplied linear/exponential tire laws; omitted normal-load/sliding/calibration closure remains omitted |
| Uploaded Lienkamp 2019 slides | Chapter 2.1, slides/PDF 1-90. Linear model slide 24; nonlinear variants slides 59, 62, 64-65. | Single-track hierarchy; absent later double-track chapters cannot be identified with OCD |
| Liniger 2018 | Section 3.1; equations 3.1-3.2; printed 17-20, equations on 18-19 | Nonlinear bicycle with simplified lateral Pacejka and RC motor/resistance model |
| Gao 2014 | Equations 2.5-2.11, printed 11-13; pure MF 2.21-2.22, printed 16; separate Fiala 2.23, printed 18 | Four-wheel source plant, source slip definitions/static loads, published pure-slip formulas and limits of combined-slip reproduction |
| Beal 2011 | Coupled brush 2.7, printed 25/PDF 36; yaw-roll 2.10, printed 27/PDF 38; AFI 2.18-2.21, printed 32/PDF 43 | Coupled tire-force law and simulation chassis differ from the reduced MPC predictor |
| Sun 2025 | Longitudinal 3.4-3.6, printed 43-44; kinematic 5.1, printed 106; task constraints 5.11, printed 112 | Separate eco-driving and formation-motion models; no lateral tire law in the kinematic model |
| Liu 2023 | Ring PDEs 2.16-2.19, printed 20-21/PDF 40-41; rigid-ring 3.110-3.114, printed 45/PDF 65 | Structural tire dynamics with genuine periodic spatial conditions; not a complete racing car model |
| Native OCD | Exact pinned commit and component source files below | Executed 33-state component laws and parameter defaults |

For Gao's downloaded no-splash PDF add 13 to the printed page; the repository's splash-page download adds another cover. Beal's inspected PDF offset is 11 and Liu's is 20. Liniger and Sun printed locations were checked in primary PDF text; an unverified viewer-page offset is not needed to locate the numbered equations.

Primary thesis records and full text:

- [Liniger, Path Planning and Control for Autonomous Racing, ETH Zurich, 2018](https://www.research-collection.ethz.ch/bitstreams/6338cc31-793a-422e-9235-c3e87b4ebf61/download).
- [Gao, Model Predictive Control for Autonomous and Semiautonomous Vehicles, UC Berkeley, 2014](https://escholarship.org/uc/item/8xd0b56h).
- [Beal, Applications of Model Predictive Control to Vehicle Dynamics for Active Safety and Stability, Stanford, 2011](https://ddl.stanford.edu/publications/thesis/applications-model-predictive-control-vehicle-dynamics-active-safety-and).
- [Sun, Model Predictive Control for Future Interconnected and Green Vehicles, UCL, 2025](https://discovery.ucl.ac.uk/id/eprint/10203332/).
- [Liu, A Novel Ring Model for Tire Dynamics and Its Application to Uncertainty Quantification, TUM, 2023](https://mediatum.ub.tum.de/doc/1690000/1690000.pdf).

Pinned native executable specification:

- [Vehicle derivative and suspension/load equations](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_vehicle_dynamics_double_track_cpp/include/ocd_vehicle_dynamics_double_track_cpp/vehicle_dynamics_equations_impl.hpp).
- [State definitions](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_vehicle_dynamics_double_track_cpp/include/ocd_vehicle_dynamics_double_track_cpp/states.hpp).
- [Vehicle parameter declarations](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_vehicle_dynamics_double_track_cpp/include/ocd_vehicle_dynamics_double_track_cpp/vehicle_dynamics_model_impl.hpp).
- [MF_SIMPLE tire model](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_tire_models_cpp/src/mf_simple.cpp).
- [Default aerodynamic model](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_aerodynamics_models_cpp/src/default.cpp).
- [Wheel-torque drivetrain](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_drivetrain_wheel_torque_cpp/src/drivetrain_direct_torque_eqns.cpp).
- [PT1 steering actuator](https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_steering_actuator_pt1_cpp/src/steering_actuator_pt1_model.cpp).
