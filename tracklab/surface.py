"""Surface properties are modeling assumptions, separate from road geometry.

The supplied Norisring CSV does not contain friction or elevation. This module
adds an explicit interface for those properties without inferring them from curvature.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Protocol
import numpy as np
from .data import FloatArray


class SurfaceModel(Protocol):
    def sample_xy(self, xy: Any) -> dict[str, FloatArray]:
        """xy has final axis length 2; values have shape xy.shape[:-1]."""
        ...


@dataclass(frozen=True)
class FlatSurface:
    """Constant, modeled values. A multiplier of 1 is not a measured friction coefficient."""
    friction_multiplier: float = 1.0
    height_m: float = 0.0

    def __post_init__(self) -> None:
        if not np.isfinite(self.friction_multiplier) or self.friction_multiplier <= 0:
            raise ValueError("Friction multiplier must be finite and positive.")
        if not np.isfinite(self.height_m):
            raise ValueError("Height must be finite.")

    def sample_xy(self, xy: Any) -> dict[str, FloatArray]:
        xy = np.asarray(xy, dtype=float)
        if xy.ndim < 1 or xy.shape[-1] != 2 or not np.isfinite(xy).all():
            raise ValueError("Surface queries require finite (..., 2) coordinates.")
        shape = xy.shape[:-1]
        return {"friction_multiplier": np.full(shape, self.friction_multiplier),
                "height_m": np.full(shape, self.height_m)}
