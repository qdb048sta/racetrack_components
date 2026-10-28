"""Closed-loop simulation. Actual states evolve through tire/body ODEs.

No state is placed on the reference curve after initialization. The independent
variable is TIME, so a closed circuit can double back in global X without issues.
"""
from __future__ import annotations
import math
import time
from pathlib import Path
from dataclasses import asdict
import numpy as np
from ..geometry import wrap_angle
from .config import VehicleConfig,DrivingConfig
from .vehicle import DynamicBicycle
from .planning import CurvatureSpeedPlan
from .controller import TrackingMPC


def axle_friction(plant,road,z):
    p=plant.config;direction=np.array([np.cos(z[2]),np.sin(z[2])])
    points=np.array([z[:2]+p.lf_m*direction,z[:2]-p.lr_m*direction])
    mu=road.surface.sample_xy(points)['friction_multiplier']
    return float(mu[0]),float(mu[1])


def simulate(road,vehicle_config=None,driving_config=None,progress_callback=None):
    p=VehicleConfig() if vehicle_config is None else vehicle_config
    d=DrivingConfig() if driving_config is None else driving_config
    p.validate();d.validate();plant=DynamicBicycle(p);plan=CurvatureSpeedPlan(road,d)
    controller=TrackingMPC(plant,road,plan,d)
    q0=road.sample_geometry(0.0)
    xy=q0['center']+d.initial_lateral_offset_m*q0['normal_left']
    z=np.array([*xy,float(q0['heading_rad']),min(d.initial_speed_mps,float(plan(0))),0,0,0.])
    previous=np.zeros(2);states=[z.copy()];times=[0.];commands=[previous.copy()]
    progresses=[0.];solves=[];predictions=[];t=0.;s=0.;target=road.length*d.laps
    s_last_report=-1e3;start=time.perf_counter();finished=False
    count=int(math.ceil(d.maximum_time_s/d.control_period_s))
    for k in range(count):
        u,info=controller.solve(z,s,previous)
        commands[-1]=u.copy()  # Input applied from this boundary onward.
        info.update(time_s=t,progress_m=s);solves.append({a:b for a,b in info.items() if a not in ('predicted_xy_m','reference_s_m')})
        predictions.append({'time_s':t,'xy_m':info['predicted_xy_m']})
        nsub=int(math.ceil(d.control_period_s/d.integration_step_s));h=d.control_period_s/nsub
        s_start=s;progress_rate=max(0.1,float(np.hypot(z[3],z[4])))
        for j in range(nsub):
            mu=axle_friction(plant,road,z)
            z=plant.step(z,u,h,*mu);t+=h
            if z[3]<.5:raise RuntimeError('Forward-speed operating range violated.')
            if abs(z[4])>8 or abs(z[5])>3.:raise RuntimeError('Vehicle left educational operating range.')
            states.append(z.copy());times.append(t);commands.append(u.copy())
            progresses.append(s_start+(j+1)*h*progress_rate)
        projected=road.project(z[:2],hint_s=s+progress_rate*d.control_period_s,window_m=d.projection_window_m)
        new_s=projected['s_m']
        if abs(new_s-s)>max(10,3*progress_rate*d.control_period_s):
            raise RuntimeError('Unexpected projection branch jump; stopping simulation.')
        # Refine intermediate progress bookkeeping by projected endpoints. No state changes.
        for j in range(nsub):
            progresses[-nsub+j]=s+(new_s-s)*(j+1)/nsub
        s=new_s;previous=u
        if abs(projected['e_left_m'])>min(float(road.sample_geometry(s)['width_left_m']),
                                               float(road.sample_geometry(s)['width_right_m']))+1.:
            raise RuntimeError(f'Car left road neighborhood at {s:.2f} m.')
        if progress_callback and (s-s_last_report>200 or s>=target):
            progress_callback({'progress_m':s,'target_m':target,'time_s':t,'speed_mps':float(np.hypot(z[3],z[4])),
                               'e_left_m':projected['e_left_m'],'solves':len(solves)})
            s_last_report=s
        if s>=target:
            finished=True;break
    if not finished:raise RuntimeError(f'Time limit reached at {s:.1f}/{target:.1f} m.')
    ts=np.asarray(times);zs=np.asarray(states);us=np.asarray(commands);ss=np.asarray(progresses)
    # Truncate only the logged crossing time, preserving the integrated trajectory as run.
    crossing=np.flatnonzero(ss>=target)[0]
    lap_time=float(np.interp(target,ss[crossing-1:crossing+1],ts[crossing-1:crossing+1]))
    result={'time_s':ts,'states':zs,'inputs':us,'progress_m':ss,'solves':solves,
            'predictions':predictions,'speed_plan':plan,'plant':plant,
            'vehicle_config':asdict(p),'driving_config':asdict(d),
            'lap_crossing_time_s':lap_time,'simulation_wall_seconds':time.perf_counter()-start}
    return result
