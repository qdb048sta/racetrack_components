"""Serialization and road export; numerical diagnostics retained from original.

The ~1 m CSV is an inspection/export grid, not an MPC discretization or a proof
of intersample clearance. The diagnostic grid is independently much denser.
"""
from __future__ import annotations
import csv
import math
from pathlib import Path
from typing import Any, TYPE_CHECKING
import numpy as np
if TYPE_CHECKING:
    from .road import PeriodicTrack

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

def export_road_map(track: "PeriodicTrack", path: str | Path, spacing_m: float = 1.0) -> Path:
    """Save a course map from geometry alone; no benchmark or surface sampling."""
    if not np.isfinite(spacing_m) or spacing_m <= 0:
        raise ValueError("spacing must be finite and positive.")
    import matplotlib.pyplot as plt

    path = Path(path)
    s = np.linspace(0, track.length, max(3, math.ceil(track.length / spacing_m)) + 1)
    q = track.sample_geometry(s)
    fig, ax = plt.subplots(figsize=(9, 7))
    try:
        for key, label in (("boundary_left", "Left boundary"),
                           ("boundary_right", "Right boundary")):
            ax.plot(q[key][:, 0], q[key][:, 1], lw=1, label=label)
        ax.plot(q["center"][:, 0], q["center"][:, 1], "--", lw=1, label="Centerline")
        ax.plot(*q["center"][0], "o", label="Station 0")
        ax.set(aspect="equal", xlabel="Local x [m]", ylabel="Local y [m]", title="Race course")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=.25)
        fig.tight_layout()
        fig.savefig(path, dpi=180)
    finally:
        plt.close(fig)
    return path


def export_road(track: "PeriodicTrack",path: Path,spacing_m: float=1.0) -> dict[str,float]:
    if spacing_m<=0:raise ValueError('spacing must be positive.')
    s=np.linspace(0,track.length,math.ceil(track.length/spacing_m)+1)
    q=track.sample(s);c=q['center'];br=q['boundary_right'];bl=q['boundary_left']
    arr=np.column_stack([s,q['u_chord_m'],c,q['heading_rad'],q['curvature_1pm'],
                         q['width_right_m'],q['width_left_m'],br,bl,
                         q['friction_multiplier_assumed'],q['height_m_assumed']])
    np.savetxt(path,arr,delimiter=',',fmt='%.10f',header=
      's_m,u_chord_m,x_m,y_m,heading_rad,curvature_1pm,width_right_m,width_left_m,'
      'right_x_m,right_y_m,left_x_m,left_y_m,friction_multiplier_assumed,height_m_assumed')
    # Diagnostic density is independent of the 1 m export. Include ALL knots:
    # a coarse export alone can miss a sharp curvature extremum at a join.
    ud=np.unique(np.r_[track.u,np.linspace(0,track.U,math.ceil(track.U/.02)+1)])
    k=track.curve.curvature(ud);widths=track.width(ud);wr,wl=widths[:,0],widths[:,1]
    im=int(np.nanargmax(np.abs(k)));peak=float(abs(k[im]))
    if track.curve.method == 'periodic_cubic':
        # Cubic power coefficients for independent reconstruction. On interval i:
        # c(u)=sum(coefficients[j,i,:]*(u-breakpoints[i])**(3-j),j=0..3).
        np.savez_compressed(path.with_name('norisring_periodic_cubic_model.npz'),
                            breakpoints=track.curve.implementation.x,
                            coefficients=track.curve.implementation.c,
                            source_u=track.u,source_widths_right_left=track.data[:,2:],
                            arc_s=track.s_grid,arc_u=track.u_grid)
    return {'export_rows_including_repeated_endpoint':len(s),'spacing_m':float(s[1]-s[0]),
            'arc_length_m':track.length,'diagnostic_points_including_all_knots':len(ud),
            'diagnostic_uniform_u_spacing_upper_bound_m':.02,
            'peak_abs_curvature_1pm':peak,'peak_curvature_source_u_m':float(ud[im]),
            'min_radius_m':1/peak,
            'max_no_slip_steering_for_3p2m_wheelbase_rad':float(np.arctan(3.2*peak)),
            'no_slip_min_radius_with_3p2m_wheelbase_and_0p30rad_steering_m':float(3.2/np.tan(.30)),
            'min_left_offset_jacobian_1_minus_kappa_wleft':float(np.min(1-k*wl)),
            'min_right_offset_jacobian_1_plus_kappa_wright':float(np.min(1+k*wr))}
