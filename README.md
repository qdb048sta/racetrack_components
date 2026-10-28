# Modular Norisring road/interpolation study

This package decomposes the previously delivered `racetrack_interpolation_demo.py` into separate data, interpolation, geometry, surface, road, evaluation, export and adapter components. The original 428-line file is preserved in `original/`.

**This is not a new MPC implementation or a native Open-Car-Dynamics simulation.** The new adapter demonstrates coordinate and per-wheel surface interfaces only. All interpolation/road benchmark formulas and default numerical outputs are preserved. New input checks and optional custom surface behavior are documented in the modules.

## Run

Python 3.10 or later; numerical tests in this session used Python 3.13.5, NumPy 2.3.5 and SciPy 1.17.0.

```bash
python -m pip install -r requirements.txt
python -m tracklab --data data/Norisring.csv --output results
python examples/query_and_adapter.py
python -m unittest discover -s tests -v
```

`python -m tracklab` runs the offline interpolation comparisons; it is not a car simulation. Add `--no-plots` to omit plotting. To rerun both versions and verify non-timing numerical agreement:

```bash
python validation/check_regression.py
```

The recorded run is in `validation/regression_results.json`, with unit-test output in `validation/component_tests.txt`. Timing measurements are intentionally excluded from equivalence assertions. Scripts need no internet access after downloading this package.

## Import only the road

```python
from tracklab import PeriodicTrack

road = PeriodicTrack("data/Norisring.csv")
query = road.sample(100.0)
print(query["center"], query["curvature_1pm"])

# Position 0.25 m to the left of the reference at 100 m:
point = query["center"] + 0.25 * query["normal_left"]
projection = road.project(point, hint_s=100.0)
print(projection)
```

Construction fits the source and prepares the arc-length maps once. Repeated sampling reuses the objects. Importing `tracklab` does not run the benchmark or import Matplotlib.

## Components

| Module | Role |
|---|---|
| `data.py` | Validate ordered `x,y,right_width,left_width` rows; assign source chord-distance labels. |
| `interpolation.py` | `Curve` common wrapper; `make_curve` factory; seven fitting strategies; smoothing matrix. |
| `geometry.py` | Geometric local frame and `ArcLengthMap`; no tire or controller dependency. |
| `surface.py` | `SurfaceModel` interface and explicit `FlatSurface` assumptions. |
| `road.py` | `WidthProfile` and `PeriodicTrack`; compose the above into queries and projection. |
| `evaluation.py` | Withheld-position test, synthetic-noise test, knot/seam metrics, benchmark orchestration. |
| `export.py` | JSON/CSV helpers and legacy road/coefficients export with geometric diagnostics. |
| `plotting.py` | Plot saved results; not used by the numerical road API. |
| `adapter.py` | NEW planar-car observation, Frenet-coordinate rates and wheel-environment mapping. |
| `cli.py` / `__main__.py` | Parse command-line options and run the offline study. |

See `WALKTHROUGH.md` for equations, original line references, evaluation details and the car-interface limitations. `SOURCE_BROWSER.html` shows numbered source lines; `SOURCE_MAP.json` contains machine-readable definition ranges.

## Preserved behavior and deliberate additions

The two original classes are `Curve` and `PeriodicTrack`. Their default evaluations are preserved. They were not originally one class per interpolation algorithm: `make_curve()` selected a SciPy object and wrapped it in `Curve`.

New components are `TrackData`, `ArcLengthMap`, `WidthProfile`, `FlatSurface`, `PlanarState`, `WheelGeometry`, and `RoadCarAdapter`. The numerical core is extracted rather than rewritten. `PeriodicTrack.sample_geometry()` separates geometry from surface assumptions; `sample()` retains the legacy dictionary keys.

The new code rejects nonfinite stations and invalid modeled surface values. Those extra checks change invalid-input behavior, not the original valid-data results. The old `PeriodicTrack` attributes remain as aliases for compatibility. Some filenames and steering diagnostics still say Norisring and use a 3.2 m wheelbase/0.30 rad limit: these are inherited demonstration settings, not a generic vehicle validation.

## Important limitations

The data is the previously downloaded snapshot with SHA-256 `8857d3c362ad2923c1f93c8d257498f50459770b9021adcc7969b71085c31d9a`. There was no new download or new verification of the physical circuit. It contains no measured friction or elevation.

The benchmark uses station labels calculated from the full source before holding out positions. It tests reconstruction at known stations. Its curvature-noise target is a chosen full-data cubic, not ground truth. The delivered `PeriodicTrack` fits every source point independently of benchmark winners; smoothing remains disallowed there until original boundaries are correctly preserved/reprojected.

The closest-point projection is a grid-seeded local scalar minimization. It does not establish a global minimum or unique road coordinates. A positive `1-curvature*offset` is a local regularity condition, not a global injectivity certificate. Point margins do not imply clearance of a finite car footprint.

Arc length and its inverse are numerical approximations; the inverse is PCHIP (C1). Therefore a quintic centerline alone does not make every `sample(s)` field C2 in s. Headings are principal angles and may wrap at +/-pi. Cyclic station labels may numerically represent a seam as zero or length minus a tiny rounding error.

The adapter's native-named payload is a plain dictionary, not a native binding object. Native construction/copying/stepping and tire-force calculations are not performed. These functions are numerical NumPy/SciPy functions, not CasADi symbolic expressions. The older `X`-monotone graph-road MPC cannot use this closed course by a class-name substitution alone.

## Attribution

The raw track snapshot, attribution and license notices from the prior package are retained. See `NOTICE.txt`, `SOURCES.md`, and `licenses/`. This package and its numerical results are not official TUM simulator output. No physical track accuracy, vehicle safety, global lap-time optimum, or feedback stability guarantee is asserted.
