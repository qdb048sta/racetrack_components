# Driving with TUM Open Car Dynamics

[Overview and running](README.md) · [Model](MODEL.md) · [Integration walkthrough](INTEGRATION.md) · [Validation](VALIDATION.md) · [Project guide](../../README.md)

`tracklab.driving_OCD` runs the car through compiled C++ components from
[TUMFTM/Open-Car-Dynamics](https://github.com/TUMFTM/Open-Car-Dynamics), pinned to
commit [`94f8fb187fb0ed22bba1d809bd74f66d1ff75af4`](https://github.com/TUMFTM/Open-Car-Dynamics/tree/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4).
It retains the existing road, curvature speed planning, closed-loop tracking,
diagnostics, plotting, and standalone HTML replay workflow. The existing
`tracklab.driving` package remains available separately.

The integration uses the original C++ source for the **simulated plant** and a
hand-transcribed CasADi version of its continuous equations for the **MPC
predictor**. The C ABI, Python wrappers, build helper, controller, and output
adaptations are local additions. This is the distinction between compiling OCD
code and rewriting equations for optimization. The [model reference](MODEL.md)
explains the native composition, equations,
parameters, and differences from the existing car. The [integration
walkthrough](INTEGRATION.md) compares upstream code with each local adapter.
See [shared driving mathematics](../../docs/DRIVING.md),
[road interpolation](../../docs/INTERPOLATION.md), and
[recorded validation](VALIDATION.md) for their separate references.

## Run

For a new machine, follow the setup guide's
[fresh-clone setup](../../docs/SETUP.md),
especially **Step 3** for the native build. It includes installing Ubuntu 24.04
under WSL, Git, a C++20 compiler, CMake, Eigen, Linux uv, and Python 3.12. This
component does not need ROS or a separate TUM OCD installation.

Run the commands below from the repository root **inside Ubuntu / WSL**.
They assume Git/C++ prerequisites and Linux uv have been installed as described
in that guide. A Windows `dev312` environment, including one activated in Git
Bash, cannot load the Linux `.so` produced by the documented build.

### First build

If you already created the environment in the setup guide, skip `uv venv` and
activate that existing environment before installing or building.

```bash
mkdir -p "$HOME/.venvs"
uv venv --python 3.12 "$HOME/.venvs/tracklab-ocd"
source "$HOME/.venvs/tracklab-ocd/bin/activate"
uv pip install -r requirements-driving.txt
python --version
python -m tracklab.driving_OCD.build_native
python -c "from tracklab.driving_OCD.native import NativeVehicle; v = NativeVehicle(); print(v.library_path); v.close()"
```

The first build fetches the pinned upstream repository and its three required
submodules under `dev/Open-Car-Dynamics`, verifies the pinned inputs, then
creates `native/tracklab_ocd.so`. Neither the downloaded source cache nor a
compiled library is supplied by a fresh clone. Python 3.12 avoids the
`casadi==3.7.2` source-build failure seen with Python 3.15.

The Linux environment is persistent under your Linux home directory. The
original recorded test run used `/tmp/tracklab-ocd-venv`; that temporary path is
not a prerequisite and should not be used for a new installation.

### Smoke test, full lap, and tests

```bash
python run_drive_OCD.py --laps 0.01 --output drive_OCD_smoke
python run_drive_OCD.py --config configs/default_drive_OCD.json --output drive_OCD_results
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  python -m unittest discover -s tests -v
```

Open `drive_OCD_smoke/norisring_replay.html` first, then the full-lap output at
`drive_OCD_results/norisring_replay.html`. Each replay embeds its data and works
without a server. In WSL, `explorer.exe .` opens the current project directory in
Windows Explorer. The output also contains all eleven native states, the seven
replay states, trajectory/solver CSVs, the speed schedule, diagnostics,
configuration, provenance, and PNG plots.

`--no-plots` omits PNGs while retaining the HTML. `--no-independent-check` skips
the open-loop native timestep-refinement check. `--config` accepts another JSON
file with the same vehicle/driving schema as `configs/default_drive_OCD.json`.

### Later runs

In a new Ubuntu terminal, using the default clone path from the setup guide:

```bash
cd "$HOME/src/racetrack_components"
source "$HOME/.venvs/tracklab-ocd/bin/activate"
python run_drive_OCD.py --output drive_OCD_results
```

From PowerShell after that setup:

```powershell
wsl -d Ubuntu-24.04 -- bash -lc 'cd ~/src/racetrack_components && ~/.venvs/tracklab-ocd/bin/python run_drive_OCD.py'
```

From Git Bash after that setup:

```bash
MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu-24.04 -- bash -lc \
  'cd ~/src/racetrack_components && ~/.venvs/tracklab-ocd/bin/python run_drive_OCD.py'
```

Substitute your own clone/environment paths if you changed their locations.
`MSYS_NO_PATHCONV=1` prevents Git Bash from rewriting Linux paths. The `wsl`
command must select Linux Python: a Windows Python process requires a Windows
`tracklab_ocd.dll`, which has not been built or tested here. The optional
`build_native --wsl` shortcut builds a Linux library; it does not produce that
Windows DLL or make Windows Python able to load the `.so`.

Simulation outputs and native build products are ignored by Git. For custom
output names, use an ignored parent such as `--output outputs/ocd`. Track CSV
input data, configuration JSON, source code, and the native manifest/bridge
remain part of the project; generated files do not need to be uploaded.

### Rebuild and compare

Once the sources are downloaded, rebuild without network access using:

```bash
python -m tracklab.driving_OCD.build_native --no-fetch
```

`--source` selects an existing checkout that passes the same commit/hash checks;
the documented full test suite expects the default `dev/Open-Car-Dynamics`
source cache. To rerun the existing Python vehicle for comparison in the same
Linux environment:

```bash
python run_drive.py --output drive_comparison_results/current --no-independent-check
```
