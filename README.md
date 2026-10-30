# Modular Norisring road/interpolation study

This package decomposes the previously delivered `racetrack_interpolation_demo.py` into separate data, interpolation, geometry, surface, road, evaluation, export and adapter components. The original 428-line file is preserved in `original/`.

**The road study and the two driving models are separate entry points.**
`tracklab.driving` uses the local Python dynamic single-track model and tracking
NMPC. `tracklab.driving_OCD` uses compiled TUM Open Car Dynamics single-track and
linear-tire components. Both use the same interpolation and road geometry.

## Step-by-step setup from a fresh clone

Use **Python 3.12** for these instructions; `.python-version` already selects it.
The driving requirements pin `casadi==3.7.2`, which has a Python 3.12 wheel but no
Python 3.15 wheel. Creating a 3.15 environment makes uv attempt a source build.
See the [CasADi release files](https://pypi.org/project/casadi/3.7.2/#files).

Choose the environment for what you want to run:

| Workflow | Entry point | Environment used in this guide |
| --- | --- | --- |
| Road/interpolation study and course map | `python -m tracklab` | Windows, Linux, or WSL; Python 3.12 |
| Existing Python vehicle | `python run_drive.py` | Windows, Linux, or WSL; Python 3.12 + CasADi |
| Native TUM single-track + linear tires | `python run_drive_OCD.py` | Ubuntu 24.04 / WSL; Python 3.12 + compiled `.so` |

For native OCD on Windows, you can go directly to **Step 3**. That route also
runs the road study and the existing Python model. Git Bash is a Windows shell;
activating a Windows environment there still uses Windows Python.

### 1. Clone the project

Install [Git](https://git-scm.com/downloads) and
[uv](https://docs.astral.sh/uv/getting-started/installation/) first. On Windows
with WinGet available, run these in PowerShell, then open a new terminal:

```powershell
winget install --id Git.Git -e
winget install --id astral-sh.uv -e
```

Clone in PowerShell, Git Bash, or a Linux shell:

```bash
git clone https://github.com/qdb048sta/racetrack_components.git
cd racetrack_components
```

Run the remaining project commands from the directory containing `README.md`
and `requirements-driving.txt`. Environments, downloaded TUM sources, and native
binaries are created during setup; a fresh clone does not supply a ready-made
environment or compiled library.

### 2. Set up Windows Python for the road study and existing vehicle

In **PowerShell**:

```powershell
uv venv --python 3.12 dev312
.\dev312\Scripts\Activate.ps1
uv pip install -r requirements-driving.txt
python --version
python -c "import casadi; print(casadi.__version__)"
```

In **Git Bash**, use the Bash activation script instead:

```bash
uv venv --python 3.12 dev312
source dev312/Scripts/activate
uv pip install -r requirements-driving.txt
python --version
python -c "import casadi; print(casadi.__version__)"
```

Choose one block; run `uv venv` only when first creating the environment. If an
older environment is active, run `deactivate` first. uv downloads Python 3.12
when needed. The version checks should show `Python 3.12.x` and `3.7.2`.
See [uv's environment guide](https://docs.astral.sh/uv/pip/environments/).

If PowerShell blocks activation, you can use the environment without changing
execution policy:

```powershell
uv pip install --python .\dev312\Scripts\python.exe -r requirements-driving.txt
.\dev312\Scripts\python.exe run_drive.py --laps 0.01 --output drive_smoke
```

If activation remains blocked, replace `python` in all later Windows commands
with `.\dev312\Scripts\python.exe`.

For only the road study, `requirements.txt` is sufficient. The driving
requirements include those base packages plus CasADi. In VS Code, select
`dev312\Scripts\python.exe` using **Python: Select Interpreter**. Keep this
environment separate from `dev/Open-Car-Dynamics`, which is the native source
cache.

Try the road study and then a short Python-model drive:

```bash
python -m tracklab --data data/Norisring.csv --output results
python examples/query_and_adapter.py
python run_drive.py --laps 0.01 --output drive_smoke
```

Open `drive_smoke/norisring_replay.html` in your browser. For a complete lap:

```bash
python run_drive.py --output drive_results
```

This Windows environment runs the existing Python vehicle. Follow Step 3 to
build and run the native TUM vehicle.

### 3. Set up native TUM OCD in Ubuntu / WSL

On Windows, open **PowerShell as administrator** and install Ubuntu under WSL:

```powershell
wsl --install -d Ubuntu-24.04
```

Restart if requested, launch Ubuntu, and finish creating your Linux username
and password. If Ubuntu 24.04 is already installed, just enter it:

```powershell
wsl -d Ubuntu-24.04
```

These are one-time machine setup steps; see
[Microsoft's WSL instructions](https://learn.microsoft.com/en-us/windows/wsl/install).
On native Ubuntu 24.04, start with the Linux commands below.

Run the following **inside Ubuntu**, rather than in the Windows `dev312`
environment:

```bash
sudo apt-get update
sudo apt-get install -y git cmake g++ libeigen3-dev curl ca-certificates
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"

mkdir -p "$HOME/src"
cd "$HOME/src"
git clone https://github.com/qdb048sta/racetrack_components.git
cd racetrack_components

mkdir -p "$HOME/.venvs"
uv venv --python 3.12 "$HOME/.venvs/tracklab-ocd"
source "$HOME/.venvs/tracklab-ocd/bin/activate"
uv pip install -r requirements-driving.txt
python --version
```

This creates a Linux clone at `~/src/racetrack_components` and a persistent
Linux environment at `~/.venvs/tracklab-ocd`. It is separate from any Windows
clone or Windows environment. Install uv inside Linux even if you already have
Windows uv. Do not place the environment in `/tmp`, whose contents can be
cleared. The installer command is from the
[official uv installation guide](https://docs.astral.sh/uv/getting-started/installation/).

If you already cloned the project inside Linux, enter that clone and skip the
`git clone` line. You can also use a Windows clone mounted under `/mnt/c/...`;
change to its actual Linux path instead. The build supports that layout, but a
clone in the Linux filesystem avoids repeated Windows/Linux filesystem access.

Build the native library, then confirm it loads:

```bash
python -m tracklab.driving_OCD.build_native
python -c "from tracklab.driving_OCD.native import NativeVehicle; v = NativeVehicle(); print(v.library_path); v.close()"
```

The build helper downloads the pinned TUM source into `dev/Open-Car-Dynamics`,
fetches its required submodules, verifies their revisions and source hashes,
and compiles `tracklab/driving_OCD/native/tracklab_ocd.so`. You do not need a
separate TUM OCD or ROS installation. Internet access is required for the first
Python/dependency/source downloads. For later builds from the downloaded
source, use `python -m tracklab.driving_OCD.build_native --no-fetch`.

Run a short native smoke test before a complete lap:

```bash
python run_drive_OCD.py --laps 0.01 --output drive_OCD_smoke
python run_drive_OCD.py --config configs/default_drive_OCD.json --output drive_OCD_results
```

Open `drive_OCD_smoke/norisring_replay.html` or
`drive_OCD_results/norisring_replay.html` in a browser. From WSL you can open the
current project folder in Windows Explorer with `explorer.exe .`, then open the
output directory. The replay
embeds its data and does not need a server. `--no-plots` skips PNGs while still
creating the HTML; `--no-independent-check` skips native timestep refinement.

The [native model README](tracklab/driving_OCD/README.md) explains the TUM
components, parameter overrides, and comparison with the existing Python model.

### 4. Run again after opening a new terminal

For the **Windows Python model**, enter the Windows clone and reactivate
`dev312` using the PowerShell or Git Bash activation command from Step 2.

For **native OCD**, open Ubuntu and run:

```bash
cd "$HOME/src/racetrack_components"
source "$HOME/.venvs/tracklab-ocd/bin/activate"
python run_drive_OCD.py --output drive_OCD_results
```

After completing Step 3 with its default paths, you can also launch the Linux
run directly from **PowerShell**:

```powershell
wsl -d Ubuntu-24.04 -- bash -lc 'cd ~/src/racetrack_components && ~/.venvs/tracklab-ocd/bin/python run_drive_OCD.py'
```

Or directly from **Git Bash**:

```bash
MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu-24.04 -- bash -lc \
  'cd ~/src/racetrack_components && ~/.venvs/tracklab-ocd/bin/python run_drive_OCD.py'
```

The prefix prevents Git Bash from rewriting Linux paths. Both commands select
Linux Python explicitly; a Windows environment cannot load the Linux `.so`.
If you chose another clone or environment location, substitute those paths.

### 5. Run the tests

After the native build, run the complete suite **inside the Linux environment**:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  python -m unittest discover -s tests -v
```

This includes the native source/force-law checks and a short closed-loop run.
The recorded integration run passed 49 tests. Test output is printed to the
terminal; generated validation reports are local artifacts and are not committed.

If you only set up Windows Python, run the two non-native suites instead:

```bash
python -m unittest discover -s tests -p test_components.py -v
python -m unittest discover -s tests -p test_driving.py -v
```

Running all tests under Windows also selects the native OCD tests and requires
a Windows `tracklab_ocd.dll`, which this WSL setup does not build. To rerun the
offline interpolation regression comparison, use:

```bash
python validation/check_regression.py
```

That regression command rewrites its recorded outputs under `validation/`.
Timing measurements are excluded from equivalence assertions. Once the Python
dependencies and native sources are installed, the study and simulations run
without internet access.

### 6. Common setup errors

| Symptom | Cause and correction |
| --- | --- |
| `Failed to build casadi==3.7.2` / CMake error with Python 3.15 | Use `uv venv --python 3.12` and install the requirements into that environment. Confirm `python --version`. |
| `tracklab_ocd.dll` is missing | The script is using Windows Python. Run the native vehicle through the Linux environment in Step 3 or 4. |
| `tracklab_ocd.so` is missing | In Linux, install the C++ prerequisites and run the native build helper. Compiled libraries are not included in a fresh clone. |
| `uv: command not found` in WSL | Install Linux uv and add `$HOME/.local/bin` to the Linux `PATH`; Windows uv and Linux uv are separate installations. |
| `/tmp/tracklab-ocd-venv/bin/python` no longer exists | That path belonged to the original temporary test environment. Create the persistent environment in Step 3. |
| WSL receives a Git/Windows path instead of a Linux path | Use the `MSYS_NO_PATHCONV=1` prefix shown for Git Bash. |
| VS Code uses the wrong Python | Select `dev312\Scripts\python.exe` for Windows, or open the Linux clone in VS Code's WSL mode and select `~/.venvs/tracklab-ocd/bin/python`. |

Before sharing a new repository revision, commit and push the application
source, `run_drive_OCD.py`, `tests/test_driving_OCD.py`,
`configs/default_drive_OCD.json`, `.gitignore`, and both READMEs. The native source manifest,
bridge, CMake files, and license notices must be included. Virtual environments,
downloaded upstream checkouts, and compiled libraries are recreated by these
steps and should not be copied between machines.

The root `.gitignore` excludes simulation outputs, interpolation exports,
validation reports, Python environments/caches, downloaded TUM sources, and
native build products. Put custom output folders and map exports under
`results/` or `outputs/` so they are also excluded, for example
`python run_drive_OCD.py --output outputs/ocd`. Keep source data in `data/` and
configuration files in `configs/`; those remain version-controlled. Generated
HTML replays are local outputs, while `replay_template.html` is application source.

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

The shared road-building pipeline below is used by both entry points. Paths are relative to the repository root; arrows show the data passed between stages.

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

### Driving model, closed-loop simulation, and replay

Install `requirements-driving.txt`, then run `python run_drive.py --output drive_results`. To load the supplied configuration explicitly, add `--config configs/default_drive.json`; without `--config`, the dataclass defaults are used.

```mermaid
flowchart TD
    RUN["run_drive.py: main<br/>Parse track, output, configuration and driving overrides"]
    CONFIG["tracklab/driving/config.py: load_config<br/>VehicleConfig + DrivingConfig<br/>Optional configs/default_drive.json or another JSON file"]
    ROAD["tracklab/road.py: PeriodicTrack<br/>CSV → interpolated continuous road<br/>Shared road-building pipeline above"]
    SIM["tracklab/driving/simulation.py: simulate<br/>Initialize vehicle state, plan and controller"]
    PLAN["tracklab/driving/planning.py: CurvatureSpeedPlan<br/>Sample road curvature; build speed envelope<br/>Apply speed, lateral-acceleration, acceleration and braking limits"]
    CTRL["tracklab/driving/controller.py: TrackingMPC.solve<br/>Sample numeric road preview and speed references<br/>CasADi / IPOPT optimize steering target and acceleration command<br/>Keep predicted trajectories; apply first command"]
    PLANT["tracklab/driving/vehicle.py: DynamicBicycle<br/>Tire forces, body dynamics and steering actuation<br/>Integrate actual state over time with plant.step"]
    FEEDBACK["tracklab/driving/simulation.py<br/>Sample surface friction at axles; project actual position onto road<br/>Update progress and pass new state to next controller solve"]
    RESULT["Simulation result<br/>Time, states, applied inputs, progress, solver logs,<br/>predictions, speed plan and configuration"]
    SAVE["tracklab/driving/artifacts.py: save_results"]
    CHECK["tracklab/driving/validation.py<br/>diagnose: tracking, grip and footprint diagnostics<br/>check_recorded_input_replay: independent integration check"]
    FILES["drive_results/<br/>run_raw.npz, diagnostics.npz, trajectory.csv<br/>solver_log.csv, speed_schedule.csv, metrics.json<br/>solves.json, predictions.json, vehicle_config.json, driving_config.json<br/>independent_integration_check.json when enabled"]
    REPLAY["tracklab/driving/replay.py: build_replay<br/>Combine road geometry, sampled states, diagnostics and predictions<br/>Embed JSON into tracklab/driving/replay_template.html"]
    HTML["drive_results/norisring_replay.html<br/>Standalone interactive browser replay"]
    PLOTS["tracklab/driving/plots.py: make_plots<br/>drive_results/trajectory.png, speed.png,<br/>angles.png and tracking_error.png"]
    RUN --> CONFIG
    RUN --> ROAD
    CONFIG --> SIM
    ROAD --> SIM
    SIM --> PLAN --> CTRL
    CTRL --> PLANT --> FEEDBACK
    ROAD --> FEEDBACK
    FEEDBACK -->|next control period| CTRL
    FEEDBACK -->|lap target reached| RESULT
    RESULT --> SAVE --> CHECK --> FILES
    SAVE --> REPLAY --> HTML
    ROAD --> REPLAY
    FILES --> PLOTS
```

The controller tracks the road and planned speed using a dynamic single-track vehicle model; its objective is trajectory tracking. Vehicle states come from time integration of the dynamics. The replay displays those recorded states and controller predictions. `--no-independent-check` skips the independent integration check, and `--no-plots` skips the driving PNGs; the HTML replay is still produced.

## Components

For the native TUM single-track car with linear tires, use the separate
[`tracklab/driving_OCD`](tracklab/driving_OCD/README.md) package. It builds pinned,
unmodified Open Car Dynamics C++ components, retains this road/interpolation
pipeline, and produces a comparable standalone replay with
`python run_drive_OCD.py --output drive_OCD_results` inside the Linux environment
created above. Its README covers the native
build, parameter overrides, comparison with `driving`, and validation results.

| Module | Role |
|---|---|
| `data.py` | Validate ordered `x,y,right_width,left_width` rows; assign source chord-distance labels. |
| `interpolation.py` | `Curve` common wrapper; `make_curve` factory; seven fitting strategies; smoothing matrix. |
| `geometry.py` | Geometric local frame and `ArcLengthMap`; no tire or controller dependency. |
| `surface.py` | `SurfaceModel` interface and explicit `FlatSurface` assumptions. |
| `road.py` | `WidthProfile` and `PeriodicTrack`; compose the above into queries and projection. |
| `evaluation.py` | Withheld-position test, synthetic-noise test, knot/seam metrics, benchmark orchestration. |
| `export.py` | JSON/CSV helpers, road/coefficients export with geometric diagnostics, and standalone course-map images. |
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
