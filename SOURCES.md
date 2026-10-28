# Sources and provenance

Access date: **2026-10-27**. The track files below are public research datasets, not survey certificates or guarantees of the current circuit layout.

1. TUMFTM, **racetrack-database**: https://github.com/TUMFTM/racetrack-database
   Data fields, OpenStreetMap origin, prior centerline smoothing, satellite-derived widths, minimum-curvature racelines, and accuracy caveat.
2. Norisring source file: https://raw.githubusercontent.com/TUMFTM/racetrack-database/master/tracks/Norisring.csv
   The complete source snapshot is included as `Norisring.csv`. The moving branch was accessible, but a commit SHA was not established. Reproducibility is provided by the included snapshot and its local SHA256, not a claimed repository commit.
3. Hockenheim alternative: https://raw.githubusercontent.com/TUMFTM/racetrack-database/master/tracks/Hockenheim.csv
4. Berlin 2018 alternative: https://raw.githubusercontent.com/TUMFTM/global_racetrajectory_optimization/master/inputs/tracks/berlin_2018.csv
5. TUMFTM, global trajectory optimization: https://github.com/TUMFTM/global_racetrajectory_optimization
6. F1TENTH racetracks: https://github.com/f1tenth/f1tenth_racetracks
   This is a downscaled alternative, not direct full-size data; its README specifies 1:10 scaling and fixed 2.20 m track width.
7. Pinned Open-Car-Dynamics external-influence interface: https://raw.githubusercontent.com/TUMFTM/Open-Car-Dynamics/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/lib/ocd_types_cpp/include/ocd_types_cpp/types.hpp
   `lambda_mue` and `z_height_road_m` are declared per-wheel fields. Declaration does not establish that every model fully implements terrain interaction.
8. SciPy CubicSpline: https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html
9. SciPy PCHIP: https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.PchipInterpolator.html
10. SciPy Akima/Makima: https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.Akima1DInterpolator.html
11. SciPy B-spline interpolation: https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.make_interp_spline.html
12. SciPy parametric interpolation tutorial: https://docs.scipy.org/doc/scipy/tutorial/interpolate/1D.html
13. SciPy smoothing tutorial: https://docs.scipy.org/doc/scipy/tutorial/interpolate/smoothing_splines.html
14. TUM's periodic smoothing and width-adjustment implementation: https://raw.githubusercontent.com/TUMFTM/trajectory_planning_helpers/master/trajectory_planning_helpers/spline_approximation.py
15. TUM curvature-continuous cubic construction: https://raw.githubusercontent.com/TUMFTM/trajectory_planning_helpers/master/trajectory_planning_helpers/calc_splines.py
16. E. Bertolazzi and M. Frego, **Fast and accurate G1 fitting of clothoid curves**, 2013: https://arxiv.org/abs/1305.6644
17. FIA, historical Norisring facts illustration (context only, not coordinate source): https://www.fia.com/news/f3-norisrings-key-facts

## Connections to the user's supplied books and lecture

- Gerdts, *Optimal Control of ODEs and DAEs*, second edition, **Example 1.0.1, printed pp. 2–3**: arc-length transformation, squared-curvature bending energy, and a small-slope squared-second-derivative approximation. The regularization used here is a mathematical analogue, not an identical physical bending objective for an arbitrary parameter.
- Nocedal–Wright, *Numerical Optimization*, second edition, **§10.2, p. 250 onward**: linear least squares. A fixed-basis quadratic spline fit is a finite-dimensional quadratic/least-squares problem. **§10.4, p. 265 onward** is relevant when fitting errors in the geometric coordinates themselves (orthogonal-distance regression).
- Grüne–Pannek, *Nonlinear Model Predictive Control*, second edition, **§11.5, p. 359 onward; §12.1, p. 367 onward**: numerical errors and stability; transcription of the prediction/control problem. No guarantee from these sections has been verified for this road or for a closed-loop car simulation here.
- MA3301 *Numerics of Differential Equations* lecture, **B.9, slides 75–80**, especially **Theorem B.28, slide 76**, and the error fact sheet on slide 77: unique polynomial interpolation, representation/evaluation, and limitations of high-degree interpolation.
