# Setup from a fresh clone

[Project overview](../README.md) · [Road guide](ROAD.md) · [Python driving](DRIVING.md) · [Native OCD](../tracklab/driving_OCD/README.md)

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

The [native model README](../tracklab/driving_OCD/README.md) explains the TUM
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
`configs/default_drive_OCD.json`, `.gitignore`, and the documentation under `docs/` and the driving packages. The native source manifest,
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
