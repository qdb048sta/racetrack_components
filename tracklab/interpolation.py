"""The seven fitted-curve strategies and their shared numerical interface.

Numerical formulas are preserved from original lines 35-43 and 67-141.
Curve is a wrapper around a fitted SciPy object, not a vehicle trajectory.
The argument u is a spatial chord-distance parameter, NOT a control input.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Any
import numpy as np
from scipy.interpolate import CubicSpline, PchipInterpolator, Akima1DInterpolator, make_interp_spline
from scipy.linalg import cho_factor, cho_solve
from .data import FloatArray

METHODS = ('linear', 'periodic_cubic', 'pchip_cyclic', 'akima_cyclic',
           'makima_cyclic', 'periodic_quintic', 'periodic_smoothing_cubic')
LABELS = {'linear': 'Piecewise linear', 'periodic_cubic': 'Periodic cubic',
          'pchip_cyclic': 'Cyclic PCHIP', 'akima_cyclic': 'Cyclic Akima',
          'makima_cyclic': 'Cyclic Makima', 'periodic_quintic': 'Periodic quintic',
          'periodic_smoothing_cubic': 'Periodic smoothing cubic'}
SMOOTHNESS = {'linear':'C0', 'periodic_cubic':'C2', 'pchip_cyclic':'C1',
              'akima_cyclic':'C1', 'makima_cyclic':'C1', 'periodic_quintic':'C4',
              'periodic_smoothing_cubic':'C2'}

@dataclass
class Curve:
    implementation: Any
    origin: float
    period: float
    method: str

    def __call__(self, u: Any, derivative: int=0) -> FloatArray:
        u = np.asarray(u, dtype=float)
        q = self.origin + np.mod(u-self.origin, self.period)
        if self.method == 'linear' and derivative >= 2:
            # Zero on open segments, but curvature is NOT defined at the corners.
            return np.zeros(q.shape + (2,))
        return np.asarray(self.implementation(q, nu=derivative))

    def curvature(self, u: Any) -> FloatArray:
        if self.method == 'linear':
            return np.full(np.asarray(u).shape, np.nan)
        a,b = self(u,1), self(u,2)
        speed = np.linalg.norm(a,axis=-1)
        cross = a[...,0]*b[...,1]-a[...,1]*b[...,0]
        return np.divide(cross, speed**3, out=np.full_like(cross,np.nan), where=speed>1e-10)

def smoothing_factor(x: FloatArray, period: float, lam: float) -> tuple[Any, FloatArray]:
    """Exact periodic cubic bending penalty, in a nodal/cardinal spline basis.

    Minimize sum_i ||v_i-y_i||^2 + lam * integral ||C_v''(u)||^2 du.
    C_v is the periodic cubic interpolant through unknown fitted values v_i.
    Two-point Gauss integration is exact for products of linear second derivatives.
    The parameter u is chord-distance in metres, so lam has units m^3 here.
    This is not FITPACK's residual-budget s and is not a curvature-energy penalty.
    """
    if lam < 0:
        raise ValueError('Smoothing lambda must be nonnegative.')
    n=len(x); closed_x=np.r_[x,x[0]+period]
    eye=np.eye(n)
    cardinal=CubicSpline(closed_x,np.vstack([eye,eye[0]]),bc_type='periodic',axis=0)
    h=np.diff(closed_x); mid=(closed_x[:-1]+closed_x[1:])/2
    nodes=(mid[:,None]+h[:,None]/2*np.array([-1,1])[None,:]/np.sqrt(3)).ravel()
    weights=np.repeat(h/2,2)
    D=cardinal(nodes,2)
    penalty=D.T @ (weights[:,None]*D)
    penalty=(penalty+penalty.T)/2
    return cho_factor(np.eye(n)+lam*penalty, lower=True, check_finite=False), closed_x

def make_curve(method: str, x: FloatArray, y: FloatArray, period: float,
               lam: float=100.0, factor: Any=None) -> Curve:
    """x contains unique points in increasing cyclic order, without repeated endpoint."""
    if method not in METHODS:
        raise ValueError(f'Unknown interpolation method: {method}')
    xc=np.r_[x,x[0]+period]; yc=np.vstack([y,y[0]])
    if method == 'linear':
        f=make_interp_spline(xc,yc,k=1,axis=0)
    elif method == 'periodic_cubic':
        f=CubicSpline(xc,yc,bc_type='periodic',axis=0)
    elif method == 'periodic_quintic':
        f=make_interp_spline(xc,yc,k=5,bc_type='periodic',axis=0)
    elif method == 'periodic_smoothing_cubic':
        if factor is None: factor=smoothing_factor(x,period,lam)[0]
        fitted=cho_solve(factor,y,check_finite=False)
        f=CubicSpline(xc,np.vstack([fitted,fitted[0]]),bc_type='periodic',axis=0)
    else:
        # Cyclic neighbours are needed to compute the END slopes. Merely wrapping
        # evaluation points with modulo does not make ordinary endpoint slopes match.
        k=3
        xp=np.r_[x[-k:]-period,x,x[:k]+period]
        yp=np.vstack([y[-k:],y,y[:k]])
        if method == 'pchip_cyclic':
            f=PchipInterpolator(xp,yp,axis=0)
        else:
            f=Akima1DInterpolator(xp,yp,axis=0,
                                  method='makima' if method=='makima_cyclic' else 'akima')
    return Curve(f,float(x[0]),float(period),method)
