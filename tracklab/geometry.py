"""Geometry derived from a fitted curve, independently of car and surface models."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Any
import numpy as np
from scipy.interpolate import PchipInterpolator
from .data import FloatArray
from .interpolation import Curve

def wrap_angle(a: Any) -> FloatArray:
    return np.arctan2(np.sin(a),np.cos(a))


def local_frame(curve: Curve, u: Any) -> dict[str, FloatArray]:
    """Position, unit tangent, left normal, principal-angle heading and curvature.

    Derivatives are with respect to u. Curvature uses the general parameter formula,
    so the input parameter need not be true arc length.
    """
    center = curve(u)
    derivative = curve(u, 1)
    speed = np.linalg.norm(derivative, axis=-1)
    if np.any(speed < 1e-8):
        raise RuntimeError("Degenerate tangent in interpolated track.")
    tangent = derivative / speed[..., None]
    normal = np.stack([-tangent[..., 1], tangent[..., 0]], axis=-1)
    return {
        "center": center,
        "tangent": tangent,
        "normal_left": normal,
        "heading_rad": np.arctan2(tangent[..., 1], tangent[..., 0]),
        "curvature_1pm": curve.curvature(u),
    }


@dataclass
class ArcLengthMap:
    """Numerical maps between source chord station u and fitted-curve distance s.

    Preserves the original quadrature: 8 subintervals per original segment, each
    with 5-point Gauss-Legendre integration. Inverse maps are PCHIP approximations.
    They are not analytically exact reparameterizations of the spline.
    """
    u_grid: FloatArray
    s_grid: FloatArray
    u_from_s: Any
    s_from_u: Any
    length: float

    @classmethod
    def build(cls, curve: Curve, source_u: FloatArray, period_u: float) -> "ArcLengthMap":
        xc = np.r_[source_u, period_u]
        uq = np.concatenate([
            a + (b-a)*np.arange(8)/8
            for a, b in zip(xc[:-1], xc[1:])
        ] + [np.array([period_u])])
        g, w = np.polynomial.legendre.leggauss(5)
        h = np.diff(uq)
        mid = (uq[:-1] + uq[1:])/2
        nodes = (mid[:, None] + h[:, None]/2*g).ravel()
        speed = np.linalg.norm(curve(nodes, 1), axis=1).reshape(-1, 5)
        seg = h/2*(speed@w)
        s_grid = np.r_[0, np.cumsum(seg)]
        if np.any(np.diff(s_grid) <= 0):
            raise ValueError("Arc-length map must be strictly increasing.")
        return cls(uq, s_grid, PchipInterpolator(s_grid, uq),
                   PchipInterpolator(uq, s_grid), float(s_grid[-1]))

    def to_parameter(self, s: Any) -> FloatArray:
        return np.asarray(self.u_from_s(np.mod(s, self.length)))
