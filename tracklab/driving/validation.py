"""Numerical diagnostics, not a physical safety certificate or stability proof."""
from __future__ import annotations
import json
import math
import time
from pathlib import Path
import numpy as np
from scipy.integrate import solve_ivp, trapezoid
from ..geometry import wrap_angle
from .vehicle import DynamicBicycle
from .config import VehicleConfig
from .simulation import axle_friction


def project_near_batch(road,xy,hints,iterations=10):
    """Independent vectorized local projection in the original spline parameter.

    Newton solves (c(u)-p).c'(u)=0 starting from known progress. It does NOT
    establish global nearest-point uniqueness. Check the tangential residual.
    """
    xy=np.asarray(xy,dtype=float);hints=np.asarray(hints,dtype=float)
    u=np.asarray(road.u_from_s(np.mod(hints,road.length)),dtype=float)
    for _ in range(iterations):
        c=road.curve(u);d=road.curve(u,1);dd=road.curve(u,2)
        residual=np.sum((c-xy)*d,axis=-1)
        denominator=np.sum(d*d+(c-xy)*dd,axis=-1)
        if np.any(denominator<=0.05):
            raise ValueError('Local projection lost a positive second derivative.')
        u=np.mod(u-np.clip(residual/denominator,-3.,3.),road.U)
    c=road.curve(u);d=road.curve(u,1)
    tangent=d/np.linalg.norm(d,axis=-1)[...,None]
    normal=np.stack([-tangent[...,1],tangent[...,0]],axis=-1)
    e=np.sum((xy-c)*normal,axis=-1)
    tangential=np.sum((xy-c)*tangent,axis=-1)
    swrapped=road.s_from_u(u)
    s=swrapped+road.length*np.rint((hints-swrapped)/road.length)
    return s,e,np.abs(tangential),u


def footprint_points(config,points_per_edge=10):
    a=config.body_length_m/2;b=config.body_width_m/2
    corners=np.array([[a,b],[a,-b],[-a,-b],[-a,b]])
    # Sample the perimeter, including corners. This is not a continuous-body proof.
    return np.concatenate([corners[i]+np.arange(points_per_edge)[:,None]/points_per_edge*
                        (corners[(i+1)%4]-corners[i]) for i in range(4)])


def diagnose(road,raw,plant,solves):
    ts=raw['time_s'];zs=raw['states'];us=raw['inputs'];hint=raw['progress_m']
    s,e,resid,u=project_near_batch(road,zs[:,:2],hint)
    q=road.sample_geometry(s)
    heading=wrap_angle(zs[:,2]-q['heading_rad'])
    beta=np.arctan2(zs[:,4],zs[:,3]);v=np.hypot(zs[:,3],zs[:,4])
    p=plant.config;bp=footprint_points(p)
    cs=np.cos(zs[:,2]);sn=np.sin(zs[:,2])
    min_body=np.inf;projection_residual=float(np.max(resid));outside_samples=0
    # Batch in time to bound memory; independently project all 40 perimeter samples.
    for start in range(0,len(ts),500):
        end=min(start+500,len(ts));n=end-start
        rot=np.stack([np.stack([cs[start:end],-sn[start:end]],axis=-1),
                      np.stack([sn[start:end],cs[start:end]],axis=-1)],axis=1)
        points=np.einsum('nij,kj->nki',rot,bp)+zs[start:end,None,:2]
        point_hints=np.broadcast_to(s[start:end,None],(n,len(bp)))
        _,be,rr,bu=project_near_batch(road,points,point_hints)
        ww=road.width(bu)
        margin=np.minimum(ww[...,1]-be,ww[...,0]+be)
        min_body=min(min_body,float(np.min(margin)))
        outside_samples+=int(np.count_nonzero(margin<0))
        projection_residual=max(projection_residual,float(np.max(rr)))
    diagnostics=[]
    for z,input_ in zip(zs,us):
        mf,mr=axle_friction(plant,road,z)
        diagnostics.append(plant.diagnostics(z,input_,mf,mr))
    keys=list(diagnostics[0]);dyn={k:np.array([x[k] for x in diagnostics]) for k in keys}
    metrics={
        'completed_laps_by_projection':float((s[-1]-s[0])/road.length),
        'reference_length_m':float(road.length),
        'actual_distance_integrated_m':float(trapezoid(v,ts)),
        'maximum_speed_mps':float(np.max(v)), 'minimum_speed_mps':float(np.min(v)),
        'maximum_abs_cross_track_error_m':float(np.max(np.abs(e))),
        'rms_cross_track_error_m':float(np.sqrt(np.mean(e*e))),
        'maximum_abs_sideslip_deg':float(np.rad2deg(np.max(np.abs(beta)))),
        'maximum_abs_steering_deg':float(np.rad2deg(np.max(np.abs(zs[:,6])))),
        'maximum_abs_yaw_rate_radps':float(np.max(np.abs(zs[:,5]))),
        'maximum_abs_lateral_acceleration_mps2':float(np.max(np.abs(dyn['ay_body_mps2']))),
        'maximum_abs_front_slip_deg':float(np.rad2deg(np.max(np.abs(dyn['alpha_front_rad'])))),
        'maximum_abs_rear_slip_deg':float(np.rad2deg(np.max(np.abs(dyn['alpha_rear_rad'])))),
        'maximum_grip_utilization':float(max(np.max(dyn['front_grip_utilization']),np.max(dyn['rear_grip_utilization']))),
        'minimum_center_cross_section_margin_m':float(np.min(np.minimum(q['width_left_m']-e,q['width_right_m']+e))),
        'minimum_sampled_body_cross_section_margin_m':min_body,
        'body_perimeter_samples_per_state':len(bp),
        'body_samples_outside_local_road':outside_samples,
        'maximum_projection_tangential_residual_m':projection_residual,
        'minimum_center_frenet_jacobian':float(np.min(1-q['curvature_1pm']*e)),
        'number_of_integrated_states':len(ts), 'number_of_mpc_solves':len(solves),
        'successful_mpc_solves':sum(x['success'] for x in solves),
        'max_solver_constraint_violation':max(x['constraint_violation'] for x in solves),
        'median_mpc_solve_time_s':float(np.median([x['solve_seconds'] for x in solves])),
        'max_mpc_solve_time_s':float(max(x['solve_seconds'] for x in solves)),
        'diagnostic_limitations':'Sampled local-width checks; not continuous collision or real-road safety certification.',
    }
    diagnostics_out={'progress_m':s,'cross_track_error_m':e,'heading_error_rad':heading,
                     'sideslip_rad':beta,'speed_mps':v,**dyn}
    return metrics,diagnostics_out


def check_recorded_input_replay(road,raw,plant,control_period_s):
    """Replay the already chosen controls with DOP853; do not reoptimize them.

    This checks the RK4 plant discretization, NOT controller robustness or native
    simulator agreement. The finer run is open-loop replay of the fixed inputs.
    """
    ts=raw['time_s'];zs=raw['states'];us=raw['inputs']
    n=int(round(control_period_s/(ts[1]-ts[0])))
    cur=zs[0].copy();maximum=np.zeros(7);position=0.;start=time.perf_counter()
    intervals=0
    for i in range(0,len(ts)-1,n):
        j=min(i+n,len(ts)-1);input_=us[i]
        def rhs(t,z):
            return plant.rhs(z,input_,*axle_friction(plant,road,z))
        sol=solve_ivp(rhs,(ts[i],ts[j]),cur,method='DOP853',rtol=2e-10,atol=2e-11,
                      max_step=control_period_s/2)
        if not sol.success:raise RuntimeError(sol.message)
        cur=sol.y[:,-1];delta=np.abs(cur-zs[j]);maximum=np.maximum(maximum,delta)
        position=max(position,float(np.linalg.norm(cur[:2]-zs[j,:2])));intervals+=1
    return {'method':'DOP853, rtol=2e-10, atol=2e-11, recorded inputs held constant',
            'control_intervals':intervals,'maximum_endpoint_position_difference_m':position,
            'maximum_endpoint_state_component_differences':maximum.tolist(),
            'wall_seconds':time.perf_counter()-start,
            'limitations':'No controller reoptimization; not comparison to the native C++ model.'}
