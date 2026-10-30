"""Fetch and compile the pinned TUM OCD components; no ROS installation needed.

Linux/WSL prerequisites: git, cmake, a C++20 compiler and Eigen headers, e.g.
``sudo apt-get install git cmake g++ libeigen3-dev``. Then run
``python -m tracklab.driving_OCD.build_native``. From Windows, ``--wsl`` builds
the Linux shared library in an installed WSL distribution. Simulations using
that library must also run inside WSL.
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
import tarfile
import tempfile

HERE = Path(__file__).resolve().parent
NATIVE = HERE / "native"
MANIFEST = NATIVE / "upstream.json"
WORKSPACE = HERE.parents[1]


def _run(*args, cwd=None, capture=False):
    return subprocess.run([str(a) for a in args], cwd=cwd, check=True,
                          text=True, stdout=subprocess.PIPE if capture else None).stdout


def verify_source(source: str | Path) -> dict:
    """Reject a different revision, altered compiled sources or dependencies."""
    source = Path(source).resolve()
    expected = json.loads(MANIFEST.read_text(encoding="utf-8"))
    locations = {".": expected["commit"], **expected["submodules"]}
    for relative, commit in locations.items():
        location = source / relative
        actual = _run("git", "rev-parse", "HEAD", cwd=location, capture=True).strip()
        if actual != commit:
            raise RuntimeError(f"Wrong upstream revision at {location}: {actual}, expected {commit}")
    # Verify every C++ source/header in lib and each supporting include tree,
    # plus parameter-management sources, against content pinned in this repo.
    # This checks the actual compiled text and allows Windows CRLF checkouts.
    # It also avoids recursively scanning unrelated ROS submodules in WSL.
    for relative, digest in expected["sha256_lf"].items():
        actual = hashlib.sha256((source / relative).read_bytes().replace(b"\r\n", b"\n")).hexdigest()
        if actual != digest:
            raise RuntimeError(f"Pinned source hash mismatch: {relative}")
    return expected


def fetch_source(source: str | Path) -> Path:
    """Fetch only the upstream submodules needed by this native model."""
    source = Path(source).resolve()
    expected = json.loads(MANIFEST.read_text(encoding="utf-8"))
    if not source.exists():
        source.parent.mkdir(parents=True, exist_ok=True)
        _run("git", "clone", "--filter=blob:none", "--no-checkout", expected["repository"], source)
        _run("git", "checkout", expected["commit"], cwd=source)
    _run("git", "submodule", "update", "--init", "submodules/tam__common", cwd=source)
    _run("git", "submodule", "update", "--init", "submodules/parameter_management",
         "submodules/time_series_logging", cwd=source / "submodules/tam__common")
    verify_source(source)
    return source


def build_native(source: str | Path | None = None, build_dir: str | Path | None = None,
                 jobs: int = 2, fetch: bool = True) -> Path:
    """Build for the current OS, returning the ctypes-loadable library path."""
    if not shutil.which("git") or not shutil.which("cmake"):
        raise RuntimeError("Install git and CMake first. On Windows with WSL, add --wsl to this build command.")
    source = Path(source or WORKSPACE / "dev/Open-Car-Dynamics").resolve()
    if fetch:
        fetch_source(source)
    manifest = verify_source(source)
    build_dir = Path(build_dir or NATIVE / ("_build-" + sys.platform)).resolve()
    if jobs < 1:
        raise ValueError("jobs must be positive")
    # Repeated C++ header reads across a WSL Windows mount can take many minutes.
    # A verified source snapshot on the Linux filesystem avoids that overhead.
    staging = None
    compile_source = source
    if sys.platform.startswith("linux") and str(source).startswith("/mnt/"):
        staging = tempfile.TemporaryDirectory(prefix="tracklab-ocd-source-")
        compile_source = Path(staging.name)
        source_directories = (
            "lib", "submodules/tam__common/tum_helpers_cpp", "submodules/tam__common/tum_types_cpp",
            "submodules/tam__common/submodules/parameter_management/param_management_cpp",
            "submodules/tam__common/submodules/time_series_logging/tsl_logger_cpp",
        )
        windows_tar = Path("/mnt/c/Windows/System32/tar.exe")
        if windows_tar.is_file():
            # Windows tar reads its local files much faster than many separate
            # filesystem requests across /mnt/c. Extract the one archive locally.
            with tempfile.NamedTemporaryFile(dir=source.parent, suffix=".tar.gz", delete=False) as stream:
                archive = Path(stream.name)
            try:
                source_windows = _run("wslpath", "-w", source, capture=True).strip()
                archive_windows = _run("wslpath", "-w", archive, capture=True).strip()
                _run(windows_tar, "-czf", archive_windows, "-C", source_windows, *source_directories)
                with tarfile.open(archive) as snapshot:
                    snapshot.extractall(compile_source, filter="data")
            finally:
                archive.unlink(missing_ok=True)
        else:
            for relative in source_directories:
                shutil.copytree(source / relative, compile_source / relative)
    try:
        _run("cmake", "-S", NATIVE, "-B", build_dir,
             f"-DOCD_SOURCE_DIR={compile_source}", "-DCMAKE_BUILD_TYPE=Release")
        _run("cmake", "--build", build_dir, "--config", "Release", "--parallel", jobs)
    finally:
        if staging:
            staging.cleanup()
    suffix = ".dll" if sys.platform == "win32" else ".dylib" if sys.platform == "darwin" else ".so"
    built = list(build_dir.rglob("tracklab_ocd" + suffix))
    if not built:
        raise RuntimeError(f"Build did not create tracklab_ocd{suffix}")
    target = NATIVE / built[0].name
    # Existing simulations can retain their loaded image while a rebuild finishes.
    temporary_target = target.with_name(target.name + ".new")
    shutil.copy2(built[0], temporary_target)
    os.replace(temporary_target, target)
    build_info = {
        **manifest,
        "source_directory": str(source),
        "library": target.name,
        "library_sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
        "adapter_sha256": hashlib.sha256((NATIVE / "bridge.cpp").read_bytes()).hexdigest(),
        "synchronization": "upstream zero-duration step at reset/evaluation/output boundaries",
    }
    info_text = json.dumps(build_info, indent=2) + "\n"
    (build_dir / "build_info.json").write_text(info_text, encoding="utf-8")
    (NATIVE / "build_info.json").write_text(info_text, encoding="utf-8")
    print(f"Built pinned TUM OCD model: {target}")
    return target


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, help="Existing pinned upstream source checkout")
    parser.add_argument("--build-dir", type=Path)
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--no-fetch", action="store_true")
    parser.add_argument("--wsl", action="store_true", help="On Windows, build a Linux library using WSL")
    args = parser.parse_args(argv)
    if args.wsl:
        if sys.platform != "win32":
            parser.error("--wsl is only needed from Windows")
        if args.source or args.build_dir:
            parser.error("When using --wsl, run custom source/build paths directly inside WSL")
        script = _run("wsl", "--exec", "wslpath", "-a", str(Path(__file__).resolve()), capture=True).strip()
        command = ["wsl", "--exec", "python3", script, "--jobs", str(args.jobs)]
        if args.no_fetch:
            command.append("--no-fetch")
        _run(*command)
        print("The Linux library is ready. Run the simulation with your WSL Python environment.")
        return
    build_native(args.source, args.build_dir, args.jobs, not args.no_fetch)


if __name__ == "__main__":
    main()
