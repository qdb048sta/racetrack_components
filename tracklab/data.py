"""Read and validate an ordered TUM track snapshot; never sort points by x.

The legacy loader is retained from original lines 47-64. TrackData only gives its
three outputs meaningful names. The input widths are ordered RIGHT, then LEFT.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from numpy.typing import NDArray
FloatArray = NDArray[np.float64]

SOURCE_URL = 'https://raw.githubusercontent.com/TUMFTM/racetrack-database/master/tracks/Norisring.csv'

def load_track(path: str | Path) -> tuple[FloatArray, FloatArray, float]:
    """Return unclosed rows, chord-distance labels u, and closed polyline length U."""
    a = np.loadtxt(path, delimiter=',', comments='#', ndmin=2)
    if a.shape[1] < 4 or a.shape[0] < 8 or not np.isfinite(a).all():
        raise ValueError('Need at least eight finite rows: x,y,right_width,left_width.')
    a = a[:, :4].astype(float)
    if np.linalg.norm(a[0,:2] - a[-1,:2]) < 1e-8:
        if not np.allclose(a[0,2:], a[-1,2:], atol=1e-6):
            raise ValueError('Repeated endpoint has inconsistent widths.')
        a = a[:-1]
    if np.any(a[:,2:] <= 0):
        raise ValueError('All half-widths must be positive.')
    d = np.linalg.norm(np.roll(a[:,:2], -1, axis=0) - a[:,:2], axis=1)
    if np.any(d < 1e-8):
        raise ValueError('Remove consecutive duplicate coordinates before fitting.')
    if d[-1] > 5*np.median(d[:-1]):
        raise ValueError('Closing gap is unexpectedly large: check ordering/closure.')
    return a, np.r_[0.0,np.cumsum(d[:-1])], float(d.sum())


@dataclass(frozen=True)
class TrackData:
    """Arrays are shared (not deep-frozen); treat them as read-only after loading."""
    rows: FloatArray
    station_u: FloatArray
    period_u: float

    @classmethod
    def from_csv(cls, path: str | Path) -> "TrackData":
        return cls(*load_track(path))

    @property
    def xy(self) -> FloatArray:
        return self.rows[:, :2]

    @property
    def widths_right_left(self) -> FloatArray:
        return self.rows[:, 2:4]
