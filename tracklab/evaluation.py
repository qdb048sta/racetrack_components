"""Offline experiments: deliberately independent of the vehicle/controller loop.

The numerical experiments are unchanged from original lines 275-376. Code is
split into reconstruction, synthetic-noise, original self-tests and orchestration.
Timings remain local position-only microbenchmarks, not controller timings.
"""
from __future__ import annotations
from pathlib import Path
from typing import Any
import hashlib
import json
import platform
import time
import numpy as np
import scipy
from scipy.interpolate import make_interp_spline
from .data import TrackData, FloatArray, SOURCE_URL
from .interpolation import Curve, METHODS, SMOOTHNESS, make_curve, smoothing_factor
from .geometry import wrap_angle
from .road import PeriodicTrack
from .export import write_csv, json_clean

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

def evaluate_reconstruction(data: TrackData, stride: int = 4,
                            lam: float = 100.0) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Known-station held-out positions; all stride phases, not geometric ground truth."""
    a, u, U = data.rows, data.station_u, data.period_u
    n = len(u)
    if stride < 2 or n//stride < 8:
        raise ValueError("stride must leave at least eight training points.")
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
    return summaries, details

def evaluate_noise(data: TrackData, lam: float = 100.0, noise_std: float = .2,
                   noise_repeats: int = 10) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Synthetic noise with the source coordinates as assumed clean baseline.

    The curvature target is a chosen all-point cubic, not surveyed curvature.
    """
    a, u, U = data.rows, data.station_u, data.period_u
    n = len(u)
    if noise_std < 0 or noise_repeats < 1:
        raise ValueError("Invalid noise parameters.")
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
    return noise_summary, noise_records

def validate_original_properties(track: PeriodicTrack, stride: int = 4) -> float:
    """Original interpolation, unit-vector, width and selected-projection assertions."""
    a, u, U = track.data, track.u, track.U
    clean = make_curve("periodic_cubic", u, a[:, :2], U)
    q = np.linspace(0, U, 20000, endpoint=False)
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
    return equivalence

def run_experiment(data: str | Path, output: str | Path, stride: int = 4,
                   lam: float = 100, noise_std: float = .2,
                   noise_repeats: int = 10) -> dict[str, Any]:
    """Run offline benchmarks, export the separate all-point road, write provenance."""
    data = Path(data)
    out = Path(output)
    out.mkdir(parents=True, exist_ok=True)
    dataset = TrackData.from_csv(data)
    a, u, U = dataset.rows, dataset.station_u, dataset.period_u
    n = len(u)
    summaries, details = evaluate_reconstruction(dataset, stride, lam)
    write_csv(out / "comparison.csv", summaries)
    write_csv(out / "phase_details.csv", details)
    noise_summary, noise_records = evaluate_noise(dataset, lam, noise_std, noise_repeats)
    write_csv(out / "noise_comparison.csv", noise_summary)
    write_csv(out / "noise_details.csv", noise_records)
    track = PeriodicTrack(data)
    road_stats = track.export(out / "norisring_interpolated_1m.csv")
    equivalence = validate_original_properties(track, stride)
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
