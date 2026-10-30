# Modular Norisring road and driving lab

This project provides a periodic race-course road model, an interpolation study,
and two closed-loop driving simulations. Both cars use the same track data,
interpolation, road geometry, and curvature speed planner.

The original `racetrack_interpolation_demo.py` is preserved in `original/`.
The reusable implementation lives in `tracklab/`.

## Start here

Follow the [step-by-step setup guide](docs/SETUP.md) to clone the project,
create a Python 3.12 environment with uv, install dependencies, and run tests.
It includes PowerShell, Git Bash, and Ubuntu/WSL instructions.
The native OCD vehicle also requires compiling its pinned C++ components.

Run these commands from the repository root after setup:

| Workflow | Command | Main output |
| --- | --- | --- |
| Road/interpolation study | `python -m tracklab --output results` | Road CSV, spline data, comparison metrics and plots |
| Python vehicle | `python run_drive.py --output drive_results` | `drive_results/norisring_replay.html` |
| Native TUM OCD vehicle | `python run_drive_OCD.py --output drive_OCD_results` | `drive_OCD_results/norisring_replay.html` |

Run native OCD with the Linux environment documented in the setup guide.
Git Bash still uses Windows Python unless Linux Python is selected through WSL.
The HTML replays contain their data and open without a server.

## Documentation by component

| Guide | Contents |
| --- | --- |
| [Setup and troubleshooting](docs/SETUP.md) | Fresh clone, uv environments, native build, run commands, and tests |
| [Road API and exports](docs/ROAD.md) | Sampling, projection, course-map export, component roles, and workflow diagrams |
| [Interpolation and geometry](docs/INTERPOLATION.md) | All seven interpolation methods, arc length, curvature, boundaries, and road-coordinate equations |
| [Python driving model and control](docs/DRIVING.md) | Seven-state car, saturating axle tires, shared body equations, speed planning, and MPC integration |
| [Native OCD overview and running](tracklab/driving_OCD/README.md) | Native component entry point and run instructions |
| [Native OCD model](tracklab/driving_OCD/MODEL.md) | Eleven-state car, linear tires, wheel dynamics, steering, parameters, and comparison with the Python car |
| [Native OCD integration](tracklab/driving_OCD/INTEGRATION.md) | File-by-file explanation and paired upstream/local code excerpts |
| [Native OCD validation](tracklab/driving_OCD/VALIDATION.md) | Recorded model comparison, numerical checks, and attribution |

For the original refactor walkthrough, see [WALKTHROUGH.md](WALKTHROUGH.md).
[SOURCE_BROWSER.html](SOURCE_BROWSER.html) and [SOURCE_MAP.json](SOURCE_MAP.json)
describe that original source decomposition.

## Outputs and assumptions

Generated outputs, downloaded TUM source, Python environments, caches, and native
build products are ignored by Git. Put custom exports under `results/` or
`outputs/`; keep input data in `data/` and configurations in `configs/`.

The supplied vehicle and tire parameters are demonstration choices. The track
CSV contains no measured friction or elevation; the default surface is flat.
See the [road limitations](docs/ROAD.md#important-limitations) and
[OCD model details](tracklab/driving_OCD/MODEL.md) before interpreting results.
Source attribution and licenses are retained in [SOURCES.md](SOURCES.md),
[NOTICE.txt](NOTICE.txt), and `licenses/`.
