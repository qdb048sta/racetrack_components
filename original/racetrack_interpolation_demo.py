#!/usr/bin/env python3
"""Closed-racetrack interpolation study and road-query API.

Data: TUMFTM/racetrack-database, tracks/Norisring.csv (snapshot 2026-10-27).
This is road geometry analysis, NOT an executed MPC or native Open-Car-Dynamics run.

Run:
    python racetrack_interpolation_demo.py --data Norisring.csv --output results
Dependencies: Python >=3.10, NumPy, SciPy >=1.13, Matplotlib (plots only).

Seven methods are evaluated on the SAME chord-distance station labels. Those labels
are computed from the entire source dataset before withholding positions. This is a
known-station interpolation experiment, not recovery of unknown station labels.
Errors quantify agreement with already smoothed source data, not physical survey accuracy.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
import platform
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any
import numpy as np
import scipy
from numpy.typing import NDArray
from scipy.interpolate import CubicSpline, PchipInterpolator, Akima1DInterpolator, make_interp_spline
from scipy.linalg import cho_factor, cho_solve
from scipy.optimize import minimize_scalar

SOURCE_URL = 'https://raw.githubusercontent.com/TUMFTM/racetrack-database/master/tracks/Norisring.csv'
METHODS = ('linear', 'periodic_cubic', 'pchip_cyclic', 'akima_cyclic',
           'makima_cyclic', 'periodic_quintic', 'periodic_smoothing_cubic')
LABELS = {'linear': 'Piecewise linear', 'periodic_cubic': 'Periodic cubic',
          'pchip_cyclic': 'Cyclic PCHIP', 'akima_cyclic': 'Cyclic Akima',
          'makima_cyclic': 'Cyclic Makima', 'periodic_quintic': 'Periodic quintic',
          'periodic_smoothing_cubic': 'Periodic smoothing cubic'}
SMOOTHNESS = {'linear':'C0', 'periodic_cubic':'C2', 'pchip_cyclic':'C1',
              'akima_cyclic':'C1', 'makima_cyclic':'C1', 'periodic_quintic':'C4',
              'periodic_smoothing_cubic':'C2'}
FloatArray = NDArray[np.float64]


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


def wrap_angle(a: Any) -> FloatArray:
    return np.arctan2(np.sin(a),np.cos(a))


def seam_and_knots(curve: Curve, x: FloatArray) -> dict[str,float]:
    h=np.diff(np.r_[x,x[0]+curve.period])
    eps=min(h)*1e-7
    am,ap=curve(x-eps,1),curve(x+eps,1)
    jumps=np.abs(wrap_angle(np.arctan2(am[:,1],am[:,0])-np.arctan2(ap[:,1],ap[:,0])))
    out={'max_heading_jump_rad':float(jumps.max()),
         'seam_heading_jump_rad':float(jumps[0])}
    if curve.method != 'linear':
        kjump=np.abs(curve.curvature(x-eps)-curve.curvature(x+eps))
        out.update(max_curvature_jump_1pm=float(np.nanmax(kjump)),
                   seam_curvature_jump_1pm=float(kjump[0]))
    else:
        out.update(max_curvature_jump_1pm=float('nan'),seam_curvature_jump_1pm=float('nan'))
    return out


def json_clean(obj: Any) -> Any:
    if isinstance(obj,dict): return {k:json_clean(v) for k,v in obj.items()}
    if isinstance(obj,(list,tuple)): return [json_clean(v) for v in obj]
    if isinstance(obj,np.ndarray): return json_clean(obj.tolist())
    if isinstance(obj,np.integer): return int(obj)
    if isinstance(obj,(float,np.floating)):
        return float(obj) if np.isfinite(obj) else None
    return obj


def write_csv(path: Path, rows: list[dict[str,Any]]) -> None:
    with path.open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader()
        w.writerows(json_clean(rows))


class PeriodicTrack:
    """Road geometry API for replacing an open graph road y=c(X).

    Uses all source points, an exact periodic cubic (or quintic), and cyclic PCHIP
    half-widths. Friction multiplier=1, height=0 are EXPLICIT demonstration defaults,
    not measurements. Exact interpolation still doesn't validate physical borders.
    General centreline smoothing is deliberately disallowed here because changing
    the reference curve requires preserving/reprojecting the original boundaries.
    """
    def __init__(self, path: str | Path, method: str='periodic_cubic'):
        if method not in ('periodic_cubic','periodic_quintic'):
            raise ValueError('Road API only uses exact periodic cubic/quintic; see boundary warning.')
        self.data,self.u,self.U=load_track(path)
        self.curve=make_curve(method,self.u,self.data[:,:2],self.U)
        self.width=make_curve('pchip_cyclic',self.u,self.data[:,2:],self.U)
        xc=np.r_[self.u,self.U]
        # Eight subintervals per original segment, five-point Gauss quadrature each.
        uq=np.concatenate([a+(b-a)*np.arange(8)/8 for a,b in zip(xc[:-1],xc[1:])]+[np.array([self.U])])
        g,w=np.polynomial.legendre.leggauss(5)
        h=np.diff(uq);mid=(uq[:-1]+uq[1:])/2
        nodes=(mid[:,None]+h[:,None]/2*g).ravel()
        speed=np.linalg.norm(self.curve(nodes,1),axis=1).reshape(-1,5)
        seg=h/2*(speed@w)
        self.s_grid=np.r_[0,np.cumsum(seg)];self.u_grid=uq
        self.length=float(self.s_grid[-1])
        self.u_from_s=PchipInterpolator(self.s_grid,uq)
        self.s_from_u=PchipInterpolator(uq,self.s_grid)

    def sample(self,s: Any) -> dict[str,FloatArray]:
        s=np.asarray(s,dtype=float);smod=np.mod(s,self.length)
        u=self.u_from_s(smod)
        c=self.curve(u);d=self.curve(u,1);speed=np.linalg.norm(d,axis=-1)
        if np.any(speed<1e-8): raise RuntimeError('Degenerate tangent in interpolated track.')
        t=d/speed[...,None];n=np.stack([-t[...,1],t[...,0]],axis=-1)
        widths=self.width(u);wr,wl=widths[...,0],widths[...,1]
        return {'s_m':smod,'u_chord_m':u,'center':c,'tangent':t,'normal_left':n,
                'heading_rad':np.arctan2(t[...,1],t[...,0]),
                'curvature_1pm':self.curve.curvature(u),'width_right_m':wr,'width_left_m':wl,
                'boundary_right':c-wr[...,None]*n,'boundary_left':c+wl[...,None]*n,
                'friction_multiplier_assumed':np.ones_like(s),'height_m_assumed':np.zeros_like(s)}

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

    def export(self,path: Path,spacing_m: float=1.0) -> dict[str,float]:
        if spacing_m<=0:raise ValueError('spacing must be positive.')
        s=np.linspace(0,self.length,math.ceil(self.length/spacing_m)+1)
        q=self.sample(s);c=q['center'];br=q['boundary_right'];bl=q['boundary_left']
        arr=np.column_stack([s,q['u_chord_m'],c,q['heading_rad'],q['curvature_1pm'],
                             q['width_right_m'],q['width_left_m'],br,bl,
                             q['friction_multiplier_assumed'],q['height_m_assumed']])
        np.savetxt(path,arr,delimiter=',',fmt='%.10f',header=
          's_m,u_chord_m,x_m,y_m,heading_rad,curvature_1pm,width_right_m,width_left_m,'
          'right_x_m,right_y_m,left_x_m,left_y_m,friction_multiplier_assumed,height_m_assumed')
        # Diagnostic density is independent of the 1 m export. Include ALL knots:
        # a coarse export alone can miss a sharp curvature extremum at a join.
        ud=np.unique(np.r_[self.u,np.linspace(0,self.U,math.ceil(self.U/.02)+1)])
        k=self.curve.curvature(ud);widths=self.width(ud);wr,wl=widths[:,0],widths[:,1]
        im=int(np.nanargmax(np.abs(k)));peak=float(abs(k[im]))
        if self.curve.method == 'periodic_cubic':
            # Cubic power coefficients for independent reconstruction. On interval i:
            # c(u)=sum(coefficients[j,i,:]*(u-breakpoints[i])**(3-j),j=0..3).
            np.savez_compressed(path.with_name('norisring_periodic_cubic_model.npz'),
                                breakpoints=self.curve.implementation.x,
                                coefficients=self.curve.implementation.c,
                                source_u=self.u,source_widths_right_left=self.data[:,2:],
                                arc_s=self.s_grid,arc_u=self.u_grid)
        return {'export_rows_including_repeated_endpoint':len(s),'spacing_m':float(s[1]-s[0]),
                'arc_length_m':self.length,'diagnostic_points_including_all_knots':len(ud),
                'diagnostic_uniform_u_spacing_upper_bound_m':.02,
                'peak_abs_curvature_1pm':peak,'peak_curvature_source_u_m':float(ud[im]),
                'min_radius_m':1/peak,
                'max_no_slip_steering_for_3p2m_wheelbase_rad':float(np.arctan(3.2*peak)),
                'no_slip_min_radius_with_3p2m_wheelbase_and_0p30rad_steering_m':float(3.2/np.tan(.30)),
                'min_left_offset_jacobian_1_minus_kappa_wleft':float(np.min(1-k*wl)),
                'min_right_offset_jacobian_1_plus_kappa_wright':float(np.min(1+k*wr))}


def run_experiment(data: str|Path,output: str|Path, stride: int=4,lam: float=100,
                   noise_std: float=.2,noise_repeats: int=10) -> dict[str,Any]:
    data=Path(data);out=Path(output);out.mkdir(parents=True,exist_ok=True)
    a,u,U=load_track(data);n=len(u)
    if stride<2 or n//stride<8: raise ValueError('stride must leave at least eight training points.')
    if noise_std<0 or noise_repeats<1:raise ValueError('Invalid noise parameters.')
    details=[];summaries=[];q=np.linspace(0,U,20000,endpoint=False)
    for method in METHODS:
        errors=[];fiterrors=[];phase_results=[]
        for phase in range(stride):
            ids=np.arange(phase,n,stride);test=np.ones(n,dtype=bool);test[ids]=False
            curve=make_curve(method,u[ids],a[ids,:2],U,lam)
            e=np.linalg.norm(curve(u[test])-a[test,:2],axis=1)
            fiterr=np.linalg.norm(curve(u[ids])-a[ids,:2],axis=1)
            errors.extend(e);fiterrors.extend(fiterr)
            v=np.linalg.norm(curve(q,1),axis=1)
            p=curve(np.r_[q,U]);length=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum())
            k=curve.curvature(q)
            row={'method':method,'phase':phase,'training_points':len(ids),'test_points':int(test.sum()),
                 'rmse_m':float(np.sqrt(np.mean(e**2))),'max_error_m':float(e.max()),
                 'max_fit_error_m':float(fiterr.max()),'length_m':length,
                 'min_parameter_speed':float(v.min()),
                 'peak_abs_curvature_1pm':float(np.nanmax(np.abs(k))) if method!='linear' else float('nan'),
                 **seam_and_knots(curve,u[ids])}
            details.append(row);phase_results.append(row)
        e=np.asarray(errors)
        ids=np.arange(0,n,stride);times=[]
        for _ in range(5):
            t=time.perf_counter();last=make_curve(method,u[ids],a[ids,:2],U,lam);times.append((time.perf_counter()-t)*1000)
        eq=np.linspace(0,U,10000,endpoint=False);times_eval=[]
        for _ in range(9):
            t=time.perf_counter();last(eq);times_eval.append((time.perf_counter()-t)*1000)
        summaries.append({'method':method,'continuity':SMOOTHNESS[method],
                          'rmse_m':float(np.sqrt(np.mean(e**2))),'p95_error_m':float(np.quantile(e,.95)),
                          'max_error_m':float(e.max()),'max_fit_error_m':float(np.max(fiterrors)),
                          'mean_length_m':float(np.mean([r['length_m'] for r in phase_results])),
                          'max_curvature_jump_1pm':max(r['max_curvature_jump_1pm'] for r in phase_results),
                          'max_heading_jump_rad':max(r['max_heading_jump_rad'] for r in phase_results),
                          'max_peak_abs_curvature_1pm':max(r['peak_abs_curvature_1pm'] for r in phase_results),
                          'fit_median_ms':float(np.median(times)),
                          'position_eval_10000_median_ms':float(np.median(times_eval))})
    write_csv(out/'comparison.csv',summaries);write_csv(out/'phase_details.csv',details)
    # Synthetic-noise sensitivity experiment: all original stations, no positional
    # withholding. The clean source itself is used as the assumed baseline only.
    rng=np.random.default_rng(20261027);noise_records=[]
    clean=make_curve('periodic_cubic',u,a[:,:2],U);uq=(u+np.diff(np.r_[u,U])/2)%U
    base_k=clean.curvature(uq);fac=smoothing_factor(u,U,lam)[0]
    for trial in range(noise_repeats):
        noisy=a[:,:2]+rng.normal(0,noise_std,size=(n,2))
        for method in METHODS:
            f=make_curve(method,u,noisy,U,lam,fac if method=='periodic_smoothing_cubic' else None)
            e=np.linalg.norm(f(u)-a[:,:2],axis=1)
            kval=f.curvature(uq)
            noise_records.append({'method':method,'trial':trial,'rmse_to_clean_samples_m':float(np.sqrt(np.mean(e**2))),
                                  'curvature_rmse_to_clean_cubic_1pm':float(np.sqrt(np.mean((kval-base_k)**2))) if method!='linear' else float('nan'),
                                  'peak_abs_curvature_at_midpoints_1pm':float(np.max(abs(kval))) if method!='linear' else float('nan')})
    noise_summary=[]
    for method in METHODS:
        rows=[r for r in noise_records if r['method']==method]
        noise_summary.append({'method':method,**{key:float(np.mean([r[key] for r in rows])) for key in rows[0] if key not in ('method','trial')}})
    write_csv(out/'noise_comparison.csv',noise_summary);write_csv(out/'noise_details.csv',noise_records)
    track=PeriodicTrack(data);road_stats=track.export(out/'norisring_interpolated_1m.csv')
    # Test cubic power and B-spline bases represent the SAME interpolant.
    xc=np.r_[u,U];yc=np.vstack([a[:,:2],a[0,:2]])
    bs=make_interp_spline(xc,yc,k=3,bc_type='periodic',axis=0)
    equivalence=float(np.max(np.linalg.norm(clean(q)-bs(q),axis=1)))
    smooth0=make_curve('periodic_smoothing_cubic',u[::stride],a[::stride,:2],U,0)
    cubic0=make_curve('periodic_cubic',u[::stride],a[::stride,:2],U)
    assert np.max(abs(smooth0(q)-cubic0(q)))<1e-8
    assert equivalence<1e-8
    qs=track.sample(np.linspace(0,track.length,1001))
    assert np.max(abs(np.linalg.norm(qs['tangent'],axis=1)-1))<1e-10
    assert np.all(qs['width_left_m']>0) and np.all(qs['width_right_m']>0)
    for s in (0.,11.7,523.1,track.length-.5):
        z=track.sample(s);proj=track.project(z['center']+.25*z['normal_left'],hint_s=s)
        assert abs(proj['s_m']-s)<.01 and abs(proj['e_left_m']-.25)<1e-5
    meta={'source_url':SOURCE_URL,'source_accessed':'2026-10-27',
          'snapshot_sha256':hashlib.sha256(data.read_bytes()).hexdigest(),
          'source_points':n,'closed_polyline_length_m':U,
          'median_original_spacing_m':float(np.median(np.diff(np.r_[u,U]))),
          'total_width_range_m':[float(np.min(a[:,2:].sum(axis=1))),float(np.max(a[:,2:].sum(axis=1)))],
          'holdout_stride':stride,'holdout_phases':stride,'training_points_phase0':len(u[::stride]),
          'test_points_phase0':n-len(u[::stride]),'smoothing_lambda_m3':lam,
          'noise_std_per_coordinate_m':noise_std,'noise_repeats':noise_repeats,
          'cubic_bspline_equivalence_max_difference_m':equivalence,
          'versions':{'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__},
          'road':road_stats,
          'limitations':['Source centerline is already smoothed and not survey truth.',
                         'Holdout station labels computed from the entire original polyline and kept fixed.',
                         'Widths, altitude, friction and bank not inferred from geometry. Only widths are supplied.',
                         'Noise test is synthetic; its curvature baseline is the clean full-data cubic, not measured curvature.',
                         'Clothoids and global polynomial methods are discussed, not benchmarked.',
                         'No car simulation, lap-time optimization, or native C++ integration performed.',
                         'Dense-grid checks are diagnostic, not continuous collision/regularity certificates.']}
    (out/'metadata.json').write_text(json.dumps(json_clean(meta),indent=2),encoding='utf-8')
    # Numerical arrays for independently inspectable plots.
    plot_u=np.linspace(0,U,15000,endpoint=False)
    curves={m:make_curve(m,u[::stride],a[::stride,:2],U,lam) for m in METHODS}
    np.savez_compressed(out/'plot_data.npz',u=plot_u,source_u=u,source=a,training_indices=np.arange(0,n,stride),
                        **{m:curves[m](plot_u) for m in METHODS},
                        **{m+'_curvature':curves[m].curvature(plot_u) for m in METHODS})
    return {'metadata':meta,'comparison':summaries,'noise':noise_summary}


def plot_results(data: str|Path,output: str|Path) -> list[Path]:
    """Each chart is a separate figure; no fixed colors or styles are selected."""
    import matplotlib.pyplot as plt
    out=Path(output);d=np.load(out/'plot_data.npz');a=d['source'];u=d['u'];ids=d['training_indices']
    track=PeriodicTrack(data);s=np.linspace(0,track.length,10000);q=track.sample(s)
    files=[]
    fig,ax=plt.subplots(figsize=(9,7))
    ax.plot(q['boundary_left'][:,0],q['boundary_left'][:,1],label='Left boundary (reconstructed)',lw=1)
    ax.plot(q['boundary_right'][:,0],q['boundary_right'][:,1],label='Right boundary (reconstructed)',lw=1)
    ax.plot(q['center'][:,0],q['center'][:,1],label='Full-data periodic cubic',lw=1)
    ax.scatter(a[::10,0],a[::10,1],s=8,label='Source points (every 10th)')
    ax.set(aspect='equal',xlabel='Local x [m]',ylabel='Local y [m]',title='Norisring: online source geometry and reconstructed road')
    ax.legend(fontsize=8);ax.grid(True,alpha=.25);fig.tight_layout()
    p=out/'track_overview.png';fig.savefig(p,dpi=180);plt.close(fig);files.append(p)
    fig,ax=plt.subplots(figsize=(9,6))
    region=(u>1580)&(u<1740)
    for m in ('linear','pchip_cyclic','akima_cyclic','periodic_cubic','periodic_quintic'):
        xy=d[m];ax.plot(xy[region,0],xy[region,1],label=LABELS[m],lw=1.4)
    inside=(d['source_u']>1580)&(d['source_u']<1740)
    ax.scatter(a[inside,0],a[inside,1],marker='x',s=25,label='All source samples')
    train=ids[(d['source_u'][ids]>1580)&(d['source_u'][ids]<1740)]
    ax.scatter(a[train,0],a[train,1],s=60,marker='+',label='Retained training points')
    ax.set(aspect='equal',xlabel='Local x [m]',ylabel='Local y [m]',title='Tight bend: reconstructing from approximately 20 m spacing')
    ax.legend(fontsize=8);ax.grid(True,alpha=.25);fig.tight_layout()
    p=out/'bend_comparison.png';fig.savefig(p,dpi=180);plt.close(fig);files.append(p)
    fig,ax=plt.subplots(figsize=(10,5))
    for m in ('pchip_cyclic','akima_cyclic','periodic_cubic','periodic_quintic','periodic_smoothing_cubic'):
        ax.plot(u[region],d[m+'_curvature'][region],label=LABELS[m],lw=1.3)
    ax.set(xlabel='Source chord-distance parameter u [m]',ylabel='Signed curvature [1/m]',
           title='Similar-looking geometry can produce different curvature')
    ax.legend(fontsize=8);ax.grid(True,alpha=.25);fig.tight_layout()
    p=out/'curvature_comparison.png';fig.savefig(p,dpi=180);plt.close(fig);files.append(p)
    return files


def main() -> None:
    p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--data',type=Path,default=Path(__file__).with_name('Norisring.csv'))
    p.add_argument('--output',type=Path,default=Path('results'))
    p.add_argument('--stride',type=int,default=4)
    p.add_argument('--smoothing-lambda',type=float,default=100.)
    p.add_argument('--noise-std',type=float,default=.2)
    p.add_argument('--noise-repeats',type=int,default=10)
    p.add_argument('--no-plots',action='store_true')
    args=p.parse_args()
    result=run_experiment(args.data,args.output,args.stride,args.smoothing_lambda,args.noise_std,args.noise_repeats)
    if not args.no_plots:plot_results(args.data,args.output)
    print(json.dumps(json_clean(result),indent=2))

if __name__=='__main__':main()
