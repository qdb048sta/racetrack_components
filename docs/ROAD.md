# Road API, exports, and component workflow

[Project overview](../README.md) · [Setup](SETUP.md) · [Interpolation equations](INTERPOLATION.md) · [Driving and control](DRIVING.md)

Run commands and examples from the repository root. Both vehicle packages use this road pipeline.

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

# Save a course map without running the interpolation benchmark:
road.export_map("course_map.png")
```

Construction fits the source and prepares the arc-length maps once. Repeated sampling reuses the objects. Importing `tracklab` does not run the benchmark or import Matplotlib.

`road.export_map(path, spacing_m=1.0)` saves an equal-scale map of the centerline, both reconstructed boundaries, and station 0. The filename extension selects the image format (for example, PNG or SVG); the map uses local coordinates in metres.

## CSV to interpolation, modeling, and output

The shared road-building pipeline below is used by the road study and both driving models. Paths are relative to the repository root; arrows show the data passed between stages.

```mermaid
flowchart TD
    CSV["data/Norisring.csv<br/>Ordered x, y, right_width, left_width in metres"]
    LOAD["tracklab/data.py: load_track / TrackData.from_csv<br/>Read with numpy.loadtxt; validate finite rows and positive widths<br/>Remove repeated closing endpoint; reject duplicate points and large closing gaps"]
    DATA["TrackData<br/>rows + cumulative chord-distance stations u<br/>Closed polyline period U, including last-to-first segment"]
    CENTER["tracklab/interpolation.py: make_curve / Curve<br/>Fit x(u), y(u) through all source points<br/>Default: periodic cubic; road API also allows periodic quintic"]
    WIDTH["tracklab/road.py: WidthProfile<br/>make_curve with pchip_cyclic<br/>Interpolate right and left half-widths over u"]
    ARC["tracklab/geometry.py: ArcLengthMap.build<br/>Integrate the norm of the curve derivative<br/>Build numerical u-to-s and s-to-u PCHIP maps; compute lap length"]
    FRAME["tracklab/geometry.py: local_frame<br/>Curve derivatives give tangent, left normal,<br/>heading and curvature"]
    ROAD["tracklab/road.py: PeriodicTrack<br/>sample_geometry(s): wrap station, convert s to u, evaluate geometry and widths<br/>Borders = center minus right_width * normal / plus left_width * normal<br/>project(xy): estimate road station and lateral offset"]
    SURFACE["tracklab/surface.py: FlatSurface or custom SurfaceModel<br/>Default flat height and assumed friction; not measured in CSV"]
    SAMPLE["tracklab/road.py: sample(s)<br/>Road geometry plus modeled surface values"]
    MAP["Optional: PeriodicTrack.export_map(path)<br/>tracklab/export.py: export_road_map<br/>Sample geometry and draw the course map with Matplotlib"]
    MAPFILE["course_map.png (or chosen image path)<br/>Centerline, reconstructed boundaries and station 0"]
    CSV --> LOAD --> DATA
    DATA --> CENTER
    DATA --> WIDTH
    CENTER --> ARC
    CENTER --> FRAME
    ARC --> ROAD
    FRAME --> ROAD
    WIDTH --> ROAD
    ROAD --> SAMPLE
    SURFACE --> SAMPLE
    ROAD --> MAP --> MAPFILE
```

Here `u` is distance along the source polyline, while `s` is numerically integrated distance along the fitted curve. The road object fits and caches its interpolants and maps once, then reuses them for sampling. Road modeling here means constructing continuous geometry, boundaries, and a surface interface; the vehicle dynamics model is a later stage in the driving path.

### Offline interpolation study and road export

Run `python -m tracklab --data data/Norisring.csv --output results`.

```mermaid
flowchart TD
    CLI["tracklab/__main__.py → tracklab/cli.py: main<br/>Parse data, output, holdout, smoothing and noise options"]
    EXP["tracklab/evaluation.py: run_experiment<br/>Load TrackData through tracklab/data.py"]
    FIT["tracklab/evaluation.py: evaluate_reconstruction / evaluate_noise<br/>tracklab/interpolation.py: make_curve<br/>Compare linear, periodic cubic, cyclic PCHIP, Akima, Makima,<br/>periodic quintic and periodic smoothing cubic"]
    METRICS["results/comparison.csv + phase_details.csv<br/>results/noise_comparison.csv + noise_details.csv"]
    ROAD["tracklab/road.py: PeriodicTrack<br/>Build a separate full-data periodic cubic road<br/>using the shared pipeline above"]
    EXPORT["tracklab/export.py: export_road<br/>Sample road at approximately 1 m spacing<br/>Compute separate dense-grid geometric diagnostics"]
    FILES["results/norisring_interpolated_1m.csv<br/>Position, heading, curvature, widths, borders, assumed friction and height<br/>results/norisring_periodic_cubic_model.npz<br/>Spline coefficients, source stations, widths and arc-length maps"]
    META["tracklab/evaluation.py + tracklab/export.py<br/>results/metadata.json: source hash, settings, versions and diagnostics<br/>results/plot_data.npz: sampled comparison curves and curvature"]
    PLOT["tracklab/plotting.py: plot_results<br/>Read plot_data.npz and create track_overview.png,<br/>bend_comparison.png and curvature_comparison.png"]
    CLI --> EXP
    EXP --> FIT --> METRICS
    EXP --> ROAD --> EXPORT --> FILES
    FIT --> META
    EXPORT --> META
    META --> PLOT
```

Benchmark fits use withheld positions and synthetic noise to compare methods. Their scores do not select or modify the exported road: `PeriodicTrack` independently fits every source point. `--no-plots` skips the PNGs.

## Components

For the native TUM single-track car with linear tires, use the separate
[`tracklab/driving_OCD`](../tracklab/driving_OCD/README.md) package. It builds pinned,
unmodified Open Car Dynamics C++ components, retains this road/interpolation
pipeline, and produces a comparable standalone replay with
`python run_drive_OCD.py --output drive_OCD_results` inside the Linux environment
described in the [setup guide](SETUP.md). Its component guides cover the native build, parameter overrides, model comparison, and validation results.

| Module | Role |
|---|---|
| [`data.py`](../tracklab/data.py) | Validate ordered `x,y,right_width,left_width` rows; assign source chord-distance labels. |
| [`interpolation.py`](../tracklab/interpolation.py) | `Curve` common wrapper; `make_curve` factory; seven fitting strategies; smoothing matrix. |
| [`geometry.py`](../tracklab/geometry.py) | Geometric local frame and `ArcLengthMap`; no tire or controller dependency. |
| [`surface.py`](../tracklab/surface.py) | `SurfaceModel` interface and explicit `FlatSurface` assumptions. |
| [`road.py`](../tracklab/road.py) | `WidthProfile` and `PeriodicTrack`; compose the above into queries and projection. |
| [`evaluation.py`](../tracklab/evaluation.py) | Withheld-position test, synthetic-noise test, knot/seam metrics, benchmark orchestration. |
| [`export.py`](../tracklab/export.py) | JSON/CSV helpers, road/coefficients export with geometric diagnostics, and standalone course-map images. |
| [`plotting.py`](../tracklab/plotting.py) | Plot saved results; not used by the numerical road API. |
| [`adapter.py`](../tracklab/adapter.py) | NEW planar-car observation, Frenet-coordinate rates and wheel-environment mapping. |
| `cli.py` / `__main__.py` | Parse command-line options and run the offline study. |

See [WALKTHROUGH.md](../WALKTHROUGH.md) for equations, original line references, evaluation details and the car-interface limitations. [SOURCE_BROWSER.html](../SOURCE_BROWSER.html) shows numbered source lines; [SOURCE_MAP.json](../SOURCE_MAP.json) contains machine-readable definition ranges.

## Preserved behavior and deliberate additions

The two original classes are `Curve` and `PeriodicTrack`. Their default evaluations are preserved. They were not originally one class per interpolation algorithm: `make_curve()` selected a SciPy object and wrapped it in `Curve`.

New components are `TrackData`, `ArcLengthMap`, `WidthProfile`, `FlatSurface`, `PlanarState`, `WheelGeometry`, and `RoadCarAdapter`. The numerical core is extracted rather than rewritten. `PeriodicTrack.sample_geometry()` separates geometry from surface assumptions; `sample()` retains the legacy dictionary keys.

The new code rejects nonfinite stations and invalid modeled surface values. Those extra checks change invalid-input behavior, not the original valid-data results. The old `PeriodicTrack` attributes remain as aliases for compatibility. Some filenames and steering diagnostics still say Norisring and use a 3.2 m wheelbase/0.30 rad limit: these are inherited demonstration settings, not a generic vehicle validation.

## Important limitations

The data is the previously downloaded snapshot with SHA-256 `8857d3c362ad2923c1f93c8d257498f50459770b9021adcc7969b71085c31d9a`. There was no new download or new verification of the physical circuit. It contains no measured friction or elevation.

The benchmark uses station labels calculated from the full source before holding out positions. It tests reconstruction at known stations. Its curvature-noise target is a chosen full-data cubic, not ground truth. The delivered `PeriodicTrack` fits every source point independently of benchmark winners; smoothing remains disallowed there until original boundaries are correctly preserved/reprojected.

The closest-point projection is a grid-seeded local scalar minimization. It does not establish a global minimum or unique road coordinates. A positive `1-curvature*offset` is a local regularity condition, not a global injectivity certificate. Point margins do not imply clearance of a finite car footprint.

Arc length and its inverse are numerical approximations; the inverse is PCHIP (C1). Therefore a quintic centerline alone does not make every `sample(s)` field C2 in s. Headings are principal angles and may wrap at +/-pi. Cyclic station labels may numerically represent a seam as zero or length minus a tiny rounding error.

The `RoadCarAdapter` in `tracklab/adapter.py` returns a plain dictionary for its native-named payload. That road adapter performs no native vehicle construction, stepping, or tire-force calculation; the separate `tracklab.driving_OCD` package performs those operations through its compiled bridge. Road adapter functions use NumPy/SciPy rather than CasADi symbolic expressions. The older `X`-monotone graph-road MPC cannot use this closed course by a class-name substitution alone.

## Attribution

The raw track snapshot, attribution and license notices from the prior package are retained. See `NOTICE.txt`, `SOURCES.md`, and `licenses/`. This package and its numerical results are not official TUM simulator output. No physical track accuracy, vehicle safety, global lap-time optimum, or feedback stability guarantee is asserted.
