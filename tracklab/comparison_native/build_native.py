"""Build the unmodified pinned OCD double-track/MF_SIMPLE components.

Linux/WSL: a C++20 compiler and Eigen3 headers are required. The existing
driving_OCD source manifest verifies every upstream file and dependency.
CMake and ROS are not needed. Eigen bundled with CasADi is also supported.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from ..driving_OCD.build_native import fetch_source, verify_source

HERE = Path(__file__).resolve().parent
NATIVE = HERE / "native"
ROOT = HERE.parents[1]


def eigen_include(explicit=None):
    """Return the directory containing eigen3/Eigen/Dense."""
    candidates = [Path(explicit)] if explicit else []
    if os.environ.get("OCD_EIGEN_INCLUDE_DIR"):
        candidates.append(Path(os.environ["OCD_EIGEN_INCLUDE_DIR"]))
    candidates.extend([Path("/usr/include"), Path("/usr/local/include")])
    try:
        import casadi
        candidates.append(Path(casadi.__file__).resolve().parent / "include")
    except ImportError:
        pass
    for path in candidates:
        if (path / "eigen3/Eigen/Dense").is_file():
            return path.resolve()
    raise RuntimeError("Install Eigen3 headers (e.g. libeigen3-dev) or CasADi; "
                       "--eigen-include must contain eigen3/Eigen/Dense.")


def build_native(source=None, fetch=True, eigen=None, compiler=None):
    if sys.platform == "win32":
        raise RuntimeError("Run this C++20 native benchmark build and simulation inside WSL.")
    source = Path(source or ROOT / "dev/Open-Car-Dynamics").resolve()
    if fetch:
        fetch_source(source)
    manifest = verify_source(source)
    compiler = compiler or os.environ.get("CXX") or shutil.which("g++") or shutil.which("c++")
    if not compiler:
        raise RuntimeError("A C++20 compiler is required (g++ or c++).")
    common = source / "submodules/tam__common"
    param = common / "submodules/parameter_management/param_management_cpp"
    includes = [p / "include" for p in sorted((source / "lib").iterdir()) if (p / "include").is_dir()]
    includes += [param / "include", common / "submodules/time_series_logging/tsl_logger_cpp/include",
                 common / "tum_helpers_cpp/include", common / "tum_types_cpp/include", eigen_include(eigen)]
    sources = [NATIVE / "bridge.cpp"]
    sources += [source / p for p in (
        "lib/ocd_aerodynamics_models_cpp/src/default.cpp",
        "lib/ocd_tire_models_cpp/src/mf_simple.cpp",
        "lib/ocd_drivetrain_wheel_torque_cpp/src/drivetrain_direct_torque_model.cpp",
        "lib/ocd_drivetrain_wheel_torque_cpp/src/drivetrain_direct_torque_eqns.cpp",
        "lib/ocd_steering_actuator_pt1_cpp/src/steering_actuator_pt1_model.cpp")]
    sources += [param / "src" / p for p in (
        "param_manager_composer.cpp", "init_backend.cpp", "initialization.cpp",
        "param_reference_manager.cpp", "param_value_manager.cpp")]
    suffix = ".dylib" if sys.platform == "darwin" else ".so"
    target = NATIVE / ("tracklab_ocd_double_track" + suffix)
    temporary = target.with_name(target.name + ".new")
    command = [str(compiler), "-std=c++20", "-O3", "-DNDEBUG", "-fPIC", "-shared"]
    if sys.platform != "darwin":
        command.append("-Wl,--no-undefined")
    command += ["-I" + str(path) for path in includes]
    command += [str(path) for path in sources] + ["-o", str(temporary)]
    print("Compiling pinned native DOUBLE_TRACK / MF_SIMPLE (C++20)", flush=True)
    subprocess.run(command, check=True)
    os.replace(temporary, target)
    info = {
        **manifest,
        "composition": "WHEEL_TORQUE:PT1:DOUBLE_TRACK:MF_SIMPLE:DEFAULT",
        "state_dimension": 33,
        "source_directory": str(source),
        "library": target.name,
        "library_sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
        "adapter_sha256": hashlib.sha256((NATIVE / "bridge.cpp").read_bytes()).hexdigest(),
        "compiler": subprocess.check_output([str(compiler), "--version"], text=True).splitlines()[0],
        "compile_command": command,
        "synchronization": "upstream zero-duration step at reset/evaluation/output boundaries",
    }
    (NATIVE / "build_info.json").write_text(json.dumps(info, indent=2) + "\n")
    print(target, flush=True)
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path)
    parser.add_argument("--no-fetch", action="store_true")
    parser.add_argument("--eigen-include", type=Path)
    parser.add_argument("--compiler")
    args = parser.parse_args()
    build_native(args.source, not args.no_fetch, args.eigen_include, args.compiler)


if __name__ == "__main__":
    main()
