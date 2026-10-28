"""Separate conventional figures, each in its own file; default Matplotlib palette."""
from pathlib import Path
import numpy as np


def make_plots(road,result_directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out=Path(result_directory);raw=np.load(out/'run_raw.npz');d=np.load(out/'diagnostics.npz')
    t=raw['time_s'];z=raw['states'];u=raw['inputs']
    q=road.sample_geometry(np.linspace(0,road.length,3000))
    files=[]
    fig,ax=plt.subplots(figsize=(10,8))
    ax.plot(*q['boundary_left'].T,label='Reconstructed left boundary',linewidth=1)
    ax.plot(*q['boundary_right'].T,label='Reconstructed right boundary',linewidth=1)
    ax.plot(*q['center'].T,'--',label='Reference centerline',linewidth=.8)
    ax.plot(z[:,0],z[:,1],label='Integrated vehicle trajectory',linewidth=1.3)
    ax.set(aspect='equal',xlabel='Local X [m]',ylabel='Local Y [m]',title='Norisring: actual simulated trajectory')
    ax.legend();ax.grid(alpha=.2);fig.tight_layout();p=out/'trajectory.png';fig.savefig(p,dpi=160);plt.close(fig);files.append(p)
    from .planning import CurvatureSpeedPlan
    from .config import DrivingConfig
    import json
    plan=CurvatureSpeedPlan(road,DrivingConfig(**json.loads((out/'driving_config.json').read_text())))
    fig,ax=plt.subplots(figsize=(11,4))
    ax.plot(t,3.6*d['speed_mps'],label='Actual vehicle speed')
    ax.plot(t,3.6*plan(d['progress_m']),'--',label='Curvature speed schedule')
    ax.set(xlabel='Simulation time [s]',ylabel='Speed [km/h]',title='Acceleration and braking over one lap')
    ax.legend();ax.grid(alpha=.2);fig.tight_layout();p=out/'speed.png';fig.savefig(p,dpi=160);plt.close(fig);files.append(p)
    fig,ax=plt.subplots(figsize=(11,4))
    ax.plot(t,np.rad2deg(z[:,6]),label='Actual front steering δ')
    ax.plot(t,np.rad2deg(d['sideslip_rad']),label='Vehicle sideslip β')
    ax.set(xlabel='Simulation time [s]',ylabel='Angle [degrees]',title='Steering and sideslip are different states')
    ax.legend();ax.grid(alpha=.2);fig.tight_layout();p=out/'angles.png';fig.savefig(p,dpi=160);plt.close(fig);files.append(p)
    fig,ax=plt.subplots(figsize=(11,4))
    ax.plot(t,100*d['cross_track_error_m'])
    ax.set(xlabel='Simulation time [s]',ylabel='Cross-track error [cm]',title='Nominal model-matched run — no measurement noise or disturbances')
    ax.grid(alpha=.2);fig.tight_layout();p=out/'tracking_error.png';fig.savefig(p,dpi=160);plt.close(fig);files.append(p)
    return files
