"""A continuous road API assembled from data, centerline, widths and surface.

The original public sample/project/export names and default numerical output are
preserved. This is composition: the road HAS an interpolant; it is not a subclass
of every possible interpolation algorithm.
"""
from __future__ import annotations
from pathlib import Path
from typing import Any
import math
import numpy as np
from scipy.optimize import minimize_scalar
from .data import TrackData, FloatArray
from .interpolation import make_curve
from .geometry import ArcLengthMap, local_frame
from .surface import FlatSurface, SurfaceModel
from .export import export_road, export_road_map


class WidthProfile:
    """Two scalar cyclic PCHIPs, ordered right/left; no geometric curvature API."""
    def __init__(self, data: TrackData):
        self.interpolant = make_curve("pchip_cyclic", data.station_u,
                                     data.widths_right_left, data.period_u)

    def __call__(self, u: Any, derivative: int = 0) -> FloatArray:
        return self.interpolant(u, derivative)


class PeriodicTrack:
    """Exact periodic cubic/quintic centerline plus cyclic-PCHIP widths.

    Smoothing is kept in the benchmark, not silently applied to the drivable
    corridor: moving the reference curve requires reprojecting/preserving borders.
    The curve and widths are numerical NumPy/SciPy objects, not CasADi expressions.
    """
    def __init__(self, path: str | Path, method: str = "periodic_cubic",
                 surface: SurfaceModel | None = None):
        if method not in ("periodic_cubic", "periodic_quintic"):
            raise ValueError("Road API only uses exact periodic cubic/quintic; see boundary warning.")
        self.dataset = TrackData.from_csv(path)
        self.data = self.dataset.rows
        self.u = self.dataset.station_u
        self.U = self.dataset.period_u
        self.curve = make_curve(method, self.u, self.dataset.xy, self.U)
        self.width = WidthProfile(self.dataset)
        self.arc = ArcLengthMap.build(self.curve, self.u, self.U)
        self.surface = FlatSurface() if surface is None else surface
        # Legacy attributes retained for the original export/projection API.
        self.s_grid = self.arc.s_grid
        self.u_grid = self.arc.u_grid
        self.length = self.arc.length
        self.u_from_s = self.arc.u_from_s
        self.s_from_u = self.arc.s_from_u

    def sample_geometry(self, s: Any) -> dict[str, FloatArray]:
        """Geometry only: no surface assumption and no car-state update."""
        s = np.asarray(s, dtype=float)
        if not np.isfinite(s).all():
            raise ValueError("Stations must be finite.")
        smod = np.mod(s, self.length)
        u = self.u_from_s(smod)
        q = local_frame(self.curve, u)
        widths = self.width(u)
        wr, wl = widths[..., 0], widths[..., 1]
        c, n = q["center"], q["normal_left"]
        return {"s_m": smod, "u_chord_m": u, **q,
                "width_right_m": wr, "width_left_m": wl,
                "boundary_right": c - wr[..., None]*n,
                "boundary_left": c + wl[..., None]*n}

    def sample(self, s: Any) -> dict[str, FloatArray]:
        """Legacy dictionary with modeled surface values at the reference point.

        Use the adapter's wheel_environment() for per-wheel surface sampling.
        These defaults are not friction/height observations from the CSV.
        """
        q = self.sample_geometry(s)
        surface = self.surface.sample_xy(q["center"])
        return {**q, "friction_multiplier_assumed": surface["friction_multiplier"],
                "height_m_assumed": surface["height_m"]}

    def project(self,xy: Any,hint_s: float|None=None,window_m: float=60.0) -> dict[str,float]:
        """Numerical closest-point projection; hint reduces branch switching.

        Returns station unwrapped close to hint_s when provided. This is a local
        numerical projection, not a theorem of globally unique Frenet coordinates.
        """
        point=np.asarray(xy,dtype=float)
        if point.shape!=(2,) or not np.isfinite(point).all(): raise ValueError('xy must be two finite numbers.')
        if hint_s is None:
            sg=np.linspace(0,self.length,math.ceil(self.length),endpoint=False)
        else:
            sg=np.linspace(hint_s-window_m,hint_s+window_m,max(50,math.ceil(2*window_m)))
        c=self.sample(sg)['center'];i=int(np.argmin(np.sum((c-point)**2,axis=1)))
        step=float(sg[1]-sg[0]);g0=float(sg[i])
        res=minimize_scalar(lambda s:float(np.sum((self.sample(s)['center']-point)**2)),
                            bounds=(g0-2*step,g0+2*step),method='bounded',options={'xatol':1e-9})
        q=self.sample(res.x);e=float(np.dot(point-q['center'],q['normal_left']))
        station=float(res.x if hint_s is not None else res.x%self.length)
        return {'s_m':station,'e_left_m':e,'distance_m':float(np.sqrt(res.fun))}

    def export(self, path: Path, spacing_m: float = 1.0) -> dict[str, float]:
        return export_road(self, Path(path), spacing_m)

    def export_map(self, path: str | Path, spacing_m: float = 1.0) -> Path:
        """Save a course map (e.g. PNG or SVG); import Matplotlib only on use."""
        return export_road_map(self, path, spacing_m)
