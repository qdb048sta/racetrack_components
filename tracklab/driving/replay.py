"""Standalone offline browser replay; records are embedded, no CDN or server."""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np
from scipy.signal import find_peaks


def build_replay(road,raw,diagnostics,metrics,predictions,vehicle,driving,path):
    ts=raw['time_s'];z=raw['states'];u=raw['inputs']
    stride=max(1,int(round(.05/(ts[1]-ts[0]))))
    indices=np.unique(np.r_[np.arange(0,len(ts),stride),len(ts)-1])
    from .planning import CurvatureSpeedPlan
    from .config import DrivingConfig
    plan=CurvatureSpeedPlan(road,DrivingConfig(**driving))
    cols=np.column_stack([ts,z,diagnostics['progress_m'],diagnostics['cross_track_error_m'],
                          diagnostics['sideslip_rad'],plan(diagnostics['progress_m']),
                          diagnostics['ax_body_mps2'],diagnostics['ay_body_mps2'],
                          np.maximum(diagnostics['front_grip_utilization'],diagnostics['rear_grip_utilization']),u])
    ss=np.linspace(0,road.length,int(np.ceil(road.length/1.5))+1)
    q=road.sample_geometry(ss)
    geometry=np.column_stack([q['center'],q['boundary_left'],q['boundary_right']])
    # Predicted trajectories are stored at actual solve instants, not synthesized later.
    pred=[[round(float(p['time_s']),5),*np.round(p['xy_m'],5).tolist()] for p in predictions]
    ka=np.abs(q['curvature_1pm']);peaks,_=find_peaks(ka,distance=int(180/1.5))
    chosen=sorted(peaks[np.argsort(ka[peaks])[-3:]])
    corners=[float(np.interp(ss[i],diagnostics['progress_m'],ts)) for i in chosen]
    payload={'rows':np.round(cols[indices],7).tolist(),'geometry':np.round(geometry,5).tolist(),
             'predictions':pred,'metrics':metrics,'vehicle':vehicle,'driving':driving,'corner_times':corners,
             'columns':['time_s','x_m','y_m','yaw_rad','vx_mps','vy_mps','yaw_rate_radps','steering_rad',
                        'progress_m','cross_track_error_m','sideslip_rad','reference_speed_mps',
                        'ax_body_mps2','ay_body_mps2','grip_utilization','steering_target_rad','force_acceleration_mps2']}
    text = Path(__file__).with_name('replay_template.html').read_text(encoding='utf-8')
    text=text.replace('__PAYLOAD__',json.dumps(payload,separators=(',',':'),allow_nan=False).replace('</','<\\/'))
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text,encoding='utf8')
    return path
