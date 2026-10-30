"""ctypes boundary to the pinned, unmodified TUM Open Car Dynamics C++ model.

The selected upstream template is WHEEL_TORQUE / PT1 / SINGLE_TRACK / LINEAR /
DEFAULT. There is deliberately no Python simulation fallback. Build it with
``python -m tracklab.driving_OCD.build_native`` before creating NativeVehicle.

Upstream's fixed DoPri step retains wheel speeds from its previous RHS call,
and its public output describes its last RK stage. The adapter calls that same
upstream step with zero duration at reset, evaluation and returned-output
boundaries. This leaves the physical state unchanged and synchronizes output
and wheel caches. Between physical substeps upstream composition is untouched.
"""
from __future__ import annotations

import ctypes as ct
import os
from pathlib import Path
import sys
from typing import Mapping, Sequence

import numpy as np

SOURCE_COMMIT = "94f8fb187fb0ed22bba1d809bd74f66d1ff75af4"
MODEL_NAME = "WHEEL_TORQUE:PT1:SINGLE_TRACK:LINEAR:DEFAULT"
STATE_NAMES = (
    "x_m", "y_m", "psi_rad", "v_x_mps", "v_y_mps", "psi_dot_radps",
    "omega_FL_radps", "omega_FR_radps", "omega_RL_radps", "omega_RR_radps",
    "position_rad",
)
_NATIVE_DIR = Path(__file__).with_name("native")
_SUFFIX = ".dll" if sys.platform == "win32" else ".dylib" if sys.platform == "darwin" else ".so"
DEFAULT_LIBRARY_PATH = _NATIVE_DIR / ("tracklab_ocd" + _SUFFIX)
_DPTR = ct.POINTER(ct.c_double)


def _array(value: Sequence[float], size: int, name: str) -> np.ndarray:
    out = np.ascontiguousarray(value, dtype=np.float64)
    if out.shape != (size,) or not np.all(np.isfinite(out)):
        raise ValueError(f"{name} must contain {size} finite values")
    return out


def _pointer(value: np.ndarray):
    return value.ctypes.data_as(_DPTR)


def _diagnostics(values: np.ndarray) -> dict:
    return {
        "acceleration_mps2": values[:3].copy(),
        "yaw_acceleration_radps2": float(values[3]),
        "tire_slip_angle_rad": values[4:8].copy(),
        "tire_longitudinal_slip": values[8:12].copy(),
        "tire_force_longitudinal_N": values[12:16].copy(),
        "tire_force_lateral_N": values[16:20].copy(),
        "tire_force_vertical_N": values[20:24].copy(),
        "wheel_load_torque_Nm": values[24:28].copy(),
        "steering_angle_rad": values[28:32].copy(),
    }


class NativeVehicle:
    """Own an actual upstream C++ vehicle instance and a separate RHS evaluator.

    State order is ``STATE_NAMES``; torque and friction order is FL, FR, RL, RR.
    ``dt`` in :meth:`step` is the complete requested time interval, divided into
    ``substeps`` upstream fixed DoPri steps. :meth:`evaluate` does not disturb
    the evolving plant state. The only external influences set by this adapter
    are per-wheel upstream ``lambda_mue`` multipliers (zero wind/road height).
    """

    def __init__(self, parameters: Mapping[str, float | Sequence[float]] | None = None,
                 library_path: str | Path | None = None):
        path = Path(library_path or os.environ.get("TRACKLAB_OCD_LIBRARY", DEFAULT_LIBRARY_PATH))
        if not path.is_file():
            raise RuntimeError(
                f"TUM OCD native library is missing: {path}. Build it with "
                "python -m tracklab.driving_OCD.build_native. On Windows, the "
                "build helper can use an installed WSL distribution."
            )
        try:
            self._lib = ct.CDLL(str(path.resolve()))
        except OSError as error:
            raise RuntimeError(f"Cannot load TUM OCD native library {path}: {error}") from error
        self.library_path = path.resolve()
        self._declare_functions()
        identity = self._lib.ocd_identity().decode()
        if identity != f"{SOURCE_COMMIT}:{MODEL_NAME}":
            raise RuntimeError(f"Wrong TUM OCD native model: {identity}")
        self._handle = self._lib.ocd_create()
        if not self._handle:
            raise RuntimeError(self._lib.ocd_error().decode())
        try:
            for name, value in (parameters or {}).items():
                self.set_parameter(name, value)
            self.reset(np.zeros(11))
        except Exception:
            self.close()
            raise

    def _declare_functions(self):
        declarations = {
            "ocd_error": (ct.c_char_p, []),
            "ocd_identity": (ct.c_char_p, []),
            "ocd_create": (ct.c_void_p, []),
            "ocd_destroy": (None, [ct.c_void_p]),
            "ocd_parameter_count": (ct.c_int, [ct.c_void_p]),
            "ocd_parameter_name": (ct.c_char_p, [ct.c_void_p, ct.c_int]),
            "ocd_parameter_get": (ct.c_int, [ct.c_void_p, ct.c_char_p, _DPTR, ct.c_int, ct.POINTER(ct.c_int)]),
            "ocd_parameter_set": (ct.c_int, [ct.c_void_p, ct.c_char_p, _DPTR, ct.c_int, ct.c_int]),
            "ocd_reset": (ct.c_int, [ct.c_void_p, _DPTR, _DPTR, _DPTR]),
            "ocd_step": (ct.c_int, [ct.c_void_p, _DPTR, ct.c_double, ct.c_double, ct.c_int, _DPTR, _DPTR]),
            "ocd_state": (ct.c_int, [ct.c_void_p, _DPTR]),
            "ocd_diagnostics": (ct.c_int, [ct.c_void_p, _DPTR]),
            "ocd_evaluate": (ct.c_int, [ct.c_void_p, _DPTR, _DPTR, ct.c_double, _DPTR, _DPTR, _DPTR]),
        }
        try:
            for name, (restype, argtypes) in declarations.items():
                fn = getattr(self._lib, name)
                fn.restype, fn.argtypes = restype, argtypes
        except AttributeError as error:
            raise RuntimeError("The selected library is not the tracklab TUM OCD native adapter") from error

    def _check(self, result: int):
        if result:
            raise RuntimeError(self._lib.ocd_error().decode())

    def _live(self):
        if not getattr(self, "_handle", None):
            raise RuntimeError("NativeVehicle is closed")
        return self._handle

    def set_parameter(self, name: str, value: float | Sequence[float]):
        array = not np.isscalar(value)
        values = np.ascontiguousarray(np.atleast_1d(value), dtype=np.float64)
        if values.ndim != 1 or not values.size or not np.all(np.isfinite(values)):
            raise ValueError("Parameter values must be a finite scalar or nonempty vector")
        self._check(self._lib.ocd_parameter_set(self._live(), name.encode(), _pointer(values), len(values), int(array)))

    def parameters(self) -> dict:
        values = np.empty(1024, dtype=np.float64)
        count = ct.c_int()
        out = {}
        for i in range(self._lib.ocd_parameter_count(self._live())):
            name = self._lib.ocd_parameter_name(self._handle, i)
            self._check(self._lib.ocd_parameter_get(self._handle, name, _pointer(values), len(values), ct.byref(count)))
            out[name.decode()] = float(values[0]) if count.value == 1 else values[:count.value].tolist()
        return out

    def reset(self, state: Sequence[float], friction: Sequence[float] = (1, 1, 1, 1)) -> np.ndarray:
        x = _array(state, 11, "state")
        mu = _array(friction, 4, "friction")
        if np.any(mu < 0):
            raise ValueError("friction multipliers must be nonnegative")
        out = np.empty(11)
        self._check(self._lib.ocd_reset(self._live(), _pointer(x), _pointer(mu), _pointer(out)))
        return out

    def evaluate(self, state: Sequence[float], torques: Sequence[float], steering: float,
                 friction: Sequence[float] = (1, 1, 1, 1)) -> tuple[np.ndarray, dict]:
        x, torque = _array(state, 11, "state"), _array(torques, 4, "torques")
        mu = _array(friction, 4, "friction")
        if not np.isfinite(steering) or np.any(mu < 0):
            raise ValueError("steering must be finite and friction multipliers nonnegative")
        derivative, diag = np.empty(11), np.empty(32)
        self._check(self._lib.ocd_evaluate(self._live(), _pointer(x), _pointer(torque), steering,
                                         _pointer(mu), _pointer(derivative), _pointer(diag)))
        return derivative, _diagnostics(diag)

    def step(self, torques: Sequence[float], steering: float, dt: float, substeps: int = 1,
             friction: Sequence[float] = (1, 1, 1, 1)) -> np.ndarray:
        torque, mu = _array(torques, 4, "torques"), _array(friction, 4, "friction")
        if (not np.isfinite(dt) or dt <= 0 or isinstance(substeps, bool)
                or int(substeps) != substeps or substeps < 1
                or not np.isfinite(steering) or np.any(mu < 0)):
            raise ValueError("Use finite positive dt, positive integer substeps, finite steering and nonnegative friction")
        out = np.empty(11)
        self._check(self._lib.ocd_step(self._live(), _pointer(torque), steering, dt, int(substeps),
                                     _pointer(mu), _pointer(out)))
        if not np.all(np.isfinite(out)):
            raise FloatingPointError("The upstream TUM OCD integration produced nonfinite states; decrease the native substep")
        return out

    @property
    def state(self) -> np.ndarray:
        out = np.empty(11)
        self._check(self._lib.ocd_state(self._live(), _pointer(out)))
        return out

    @property
    def diagnostics(self) -> dict:
        out = np.empty(32)
        self._check(self._lib.ocd_diagnostics(self._live(), _pointer(out)))
        return _diagnostics(out)

    def close(self):
        if getattr(self, "_handle", None):
            self._lib.ocd_destroy(self._handle)
            self._handle = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __del__(self):
        self.close()
