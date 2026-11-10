"""Reproducible, matched-parameter Norisring plant comparison.

Detailed logs retain the actual plant motion. No trajectory is replaced by a
road-centerline trace. Failures are recorded as failures, never omitted from
the summary or silently continued with a different controller.
"""
from __future__ import annotations

from dataclasses import asdict, replace
from pathlib import Path
import csv
import hashlib
import json
import math
import platform
import subprocess
import time

import numpy as np

from tracklab import PeriodicTrack
from tracklab.driving.planning import CurvatureSpeedPlan
from tracklab.driving.validation import project_near_batch
from .controller import CommonMPC, Experiment

ROOT = Path(__file__).resolve().parents[2]
MODEL_NAMES = ("native_ocd", "mf_independent", "mf_ellipse", "fiala", "mf_static")
LABELS = {
    "native_ocd": "Native OCD double-track / MF_SIMPLE",
    "mf_independent": "Planar double-track / independent MF / load transfer",
    "mf_ellipse": "Planar double-track / MF + friction circle / load transfer",
    "fiala": "Planar double-track / standard Fiala / load transfer",
    "mf_static": "Planar double-track / independent MF / static loads",
}


def json_write(path, obj):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(obj,indent=2,allow_nan=False)+"\n",encoding="utf-8")


def make_plant(name, cfg):
    if name == "native_ocd":
        from tracklab.comparison_native.vehicle import NativeDoubleTrack, normalized_parameters
        return NativeDoubleTrack(normalized_parameters())
    from .models import ReducedDoubleTrack
    if name not in MODEL_NAMES:
        raise ValueError(f"Unknown model {name}")
    return ReducedDoubleTrack(tire_model="mf_independent" if name=="mf_static" else name,
                              load_transfer="static" if name=="mf_static" else "quasistatic",
                              integration_step_s=cfg.reduced_step_s)


def project(road, z, hint):
    s,e,residual,u=project_near_batch(road,np.asarray(z[:2])[None,:],np.array([hint]),iterations=6)
    if residual[0] > 1e-4:
        raise RuntimeError(f"Projection tangential residual {residual[0]:.3g} m")
    return float(s[0]),float(e[0])


def torque_allocation(acceleration):
    # Identical nominal static-load allocation; no per-model torque vectoring.
    return 800.*float(acceleration)*.3*np.array([1.476/3.2/2]*2+[1.724/3.2/2]*2)


def plant_step(plant, name, torque, steering, dt, cfg):
    h=cfg.native_step_s if name=="native_ocd" else cfg.reduced_step_s
    return np.asarray(plant.step(torque,float(steering),float(dt),
                                substeps=math.ceil(dt/h-1e-10)),dtype=float)


def boundary_metrics(road, states, hints, logging_step_s=.02):
    s,e,resid,u=project_near_batch(road,states[:,:2],hints)
    q=road.sample_geometry(s)
    # Forty perimeter points, independently projected at every logged 20 ms.
    corners=np.array([[2.2,.95],[2.2,-.95],[-2.2,-.95],[-2.2,.95]])
    body=np.concatenate([corners[i]+np.arange(10)[:,None]/10*(corners[(i+1)%4]-corners[i])
                         for i in range(4)])
    minimum=float("inf");outside=0;max_res=float(np.max(resid))
    for start in range(0,len(states),500):
        z=states[start:start+500];ss=s[start:start+500]
        cs,sn=np.cos(z[:,2]),np.sin(z[:,2])
        rot=np.stack([np.stack([cs,-sn],axis=-1),np.stack([sn,cs],axis=-1)],axis=1)
        points=np.einsum("nij,kj->nki",rot,body)+z[:,None,:2]
        _,be,rr,bu=project_near_batch(road,points,np.broadcast_to(ss[:,None],(len(ss),40)))
        widths=road.width(bu)
        margins=np.minimum(widths[...,1]-be,widths[...,0]+be)
        minimum=min(minimum,float(np.min(margins)));outside+=int(np.sum(margins<0))
        max_res=max(max_res,float(np.max(rr)))
    return s,e,{"minimum_sampled_body_margin_m":minimum,"body_samples_outside_road":outside,
        "body_perimeter_points_per_state":40,"boundary_sample_period_s":logging_step_s,
        "maximum_projection_tangential_residual_m":max_res,
        "minimum_frenet_jacobian":float(np.min(1-q["curvature_1pm"]*e))}


def run_one(name, cfg: Experiment, output, track=ROOT/"data/Norisring.csv", progress=True,
            plant_factory=None, label=None):
    output=Path(output)/cfg.name/name;output.mkdir(parents=True,exist_ok=True)
    road=PeriodicTrack(track,method="periodic_cubic")
    plan=CurvatureSpeedPlan(road,cfg)
    controller=CommonMPC(road,plan,cfg)
    plant=(plant_factory or make_plant)(name,cfg)
    q0=road.sample_geometry(0.)
    initial=np.r_[q0["center"],float(q0["heading_rad"]),cfg.initial_speed_mps,0.,0.,0.]
    z=np.asarray(plant.reset(initial),dtype=float)
    if z.shape != (7,):
        z=np.asarray(plant.observation,dtype=float)
    t=s=0.;u=np.array([0.,0.]);solve_records=[];predictions=[]
    times=[t];states=[z.copy()];inputs=[u.copy()];hints=[s]
    diagnostics=[plant.diagnostics];internal=[np.asarray(plant.state).copy()]
    control_rows=[];failure=None;plant_wall=0.;wall_start=time.perf_counter();next_report=0.
    if progress: print(f"{cfg.name}/{name}: starting {cfg.laps:g} lap(s)",flush=True)
    try:
        for _ in range(math.ceil(cfg.maximum_time_s/cfg.control_period_s)):
            s,e=project(road,z,s)
            if s>=cfg.laps*road.length: break
            u,info=controller.solve(z,s,u)
            solve_records.append({"time_s":t,"station_m":s,**{k:v for k,v in info.items() if k!="predicted_xy_m"}})
            if len(solve_records)%10==1:
                predictions.append({"time_s":t,"xy_m":info["predicted_xy_m"]})
            torque=torque_allocation(u[1]);control_rows.append([t,*u,*torque])
            # Inputs[i] is the command applied on [time[i], time[i+1]).
            inputs[-1]=u.copy()
            for _ in range(round(cfg.control_period_s/cfg.logging_step_s)):
                start=time.perf_counter()
                z=plant_step(plant,name,torque,u[0],cfg.logging_step_s,cfg)
                plant_wall+=time.perf_counter()-start
                t+=cfg.logging_step_s
                s,e=project(road,z,s+cfg.logging_step_s*max(z[3],0.))
                times.append(t);states.append(z.copy());inputs.append(u.copy());hints.append(s)
                diagnostics.append(plant.diagnostics);internal.append(np.asarray(plant.state).copy())
                if not np.all(np.isfinite(z)):raise FloatingPointError("Nonfinite plant state")
                if z[3]<.5:raise RuntimeError("Forward speed fell below the experiment's 0.5 m/s stop threshold")
                if abs(e)>10.:raise RuntimeError("Cross-track deviation exceeded 10 m")
                if s>=cfg.laps*road.length: break
            if progress and s>=next_report:
                print(f"{cfg.name}/{name}: {100*s/road.length:5.1f}% t={t:.2f}s v={np.hypot(z[3],z[4]):.2f}m/s e={e:+.3f}m",flush=True)
                next_report=s+road.length/8
            if s>=cfg.laps*road.length: break
    except (RuntimeError,ValueError,FloatingPointError) as error:
        failure=str(error)
    finally:
        if hasattr(plant,"close"):plant.close()
    elapsed=time.perf_counter()-wall_start
    ts=np.asarray(times);zs=np.asarray(states);us=np.asarray(inputs);hints=np.asarray(hints)
    dyn={k:np.asarray([d[k] for d in diagnostics]) for k in (
        "acceleration_mps2","tire_slip_angle_rad","tire_longitudinal_slip",
        "tire_force_longitudinal_N","tire_force_lateral_N","tire_force_vertical_N")}
    boundary_error=None
    try:
        station,e,bounds=boundary_metrics(road,zs,hints,cfg.logging_step_s)
    except (RuntimeError,ValueError,FloatingPointError) as error:
        station=hints;e=np.full_like(hints,np.nan);bounds={};boundary_error=str(error)
    completed=bool(station[-1]>=cfg.laps*road.length)
    if not completed and failure is None:failure="Maximum simulation time reached before the lap completed"
    # Interpolate the final crossing inside the last logged interval.
    finish=(float(ts[-2]+(cfg.laps*road.length-station[-2])/(station[-1]-station[-2])*(ts[-1]-ts[-2]))
            if completed and len(ts)>1 else None)
    fz=dyn["tire_force_vertical_N"]
    utilization=np.hypot(dyn["tire_force_longitudinal_N"],dyn["tire_force_lateral_N"])/np.maximum(fz,1e-6)
    speed=np.hypot(zs[:,3],zs[:,4]);beta=np.arctan2(zs[:,4],zs[:,3])
    summaries={"model":name,"label":label or LABELS.get(name,name),"experiment":cfg.name,
        "completed":completed,"failure":failure,"boundary_diagnostic_error":boundary_error,
        "simulation_end_time_s":float(ts[-1]),"lap_time_s":finish,
        "completed_laps":float(station[-1]/road.length),"reference_length_m":float(road.length),
        "max_abs_cross_track_m":float(np.nanmax(np.abs(e))) if np.any(np.isfinite(e)) else None,
        "rms_cross_track_m":float(np.sqrt(np.nanmean(e**2))) if np.any(np.isfinite(e)) else None,
        "max_abs_sideslip_deg":float(np.rad2deg(np.max(np.abs(beta)))),
        "max_abs_tire_slip_deg":float(np.rad2deg(np.max(np.abs(dyn["tire_slip_angle_rad"])))),
        "max_abs_longitudinal_slip":float(np.max(np.abs(dyn["tire_longitudinal_slip"]))),
        "max_abs_yaw_rate_radps":float(np.max(np.abs(zs[:,5]))),
        "max_lateral_acceleration_mps2":float(np.max(np.abs(dyn["acceleration_mps2"][:,1]))),
        "max_speed_mps":float(np.max(speed)),"max_friction_circle_utilization":float(np.max(utilization)),
        "min_wheel_load_N":float(np.min(fz)),"max_wheel_load_N":float(np.max(fz)),
        "max_force_circle_excess_N":float(np.max(np.maximum(0.,np.hypot(dyn["tire_force_longitudinal_N"],dyn["tire_force_lateral_N"])-fz))),
        "mpc_solves":len(solve_records),"mpc_failures":int(failure is not None and "MPC" in failure),
        "median_mpc_solve_ms":float(1000*np.median([r["solve_seconds"] for r in solve_records])) if solve_records else None,
        "max_mpc_constraint_violation":float(max([r["constraint_violation"] for r in solve_records],default=0)),
        "plant_wall_seconds":plant_wall,"plant_wall_ms_per_sim_second":1000*plant_wall/max(ts[-1],1e-9),
        "total_wall_seconds":elapsed,"sample_count":len(ts),**bounds}
    metadata={"settings":cfg.to_dict(),"track":str(Path(track).relative_to(ROOT)) if Path(track).is_relative_to(ROOT) else str(track),
        "track_sha256":hashlib.sha256(Path(track).read_bytes()).hexdigest(),
        "interpolation":"periodic_cubic centerline; cyclic PCHIP widths; unchanged CSV",
        "surface":"flat; per-wheel lambda_mue=1; normalized peak friction D=1",
        "initial_observation":initial.tolist(),"common_controller":"five-state kinematic tracking NMPC; fixed settings for all plants",
        "force_input_slip_kind":"native relaxed slip states" if name=="native_ocd" else "instantaneous kinematic slips",
        "torque_allocation":"static axle load fractions, equal left/right; mass800kg radius0.3m",
        "python":platform.python_version(),"numpy":np.__version__,
        "limitations":"Normalized model-family comparison, not an exact thesis/controller reproduction or real-car validation; timing depends on implementation/language and host."}
    json_write(output/"summary.json",summaries);json_write(output/"metadata.json",metadata)
    json_write(output/"solves.json",solve_records)
    json_write(output/"predictions.json",predictions)
    # Exact 50 Hz samples and internal state are available for replay/refinement.
    np.savez_compressed(output/"raw.npz",time_s=ts,states=zs,inputs=us,station_m=station,
                        cross_track_m=e,internal_state=np.asarray(internal),**dyn)
    # Small readable 10 Hz table; metrics above use the full 50 Hz arrays.
    stride=max(1,round(cfg.control_period_s/cfg.logging_step_s))
    ids=np.unique(np.r_[np.arange(0,len(ts),stride),len(ts)-1])
    cols=["time_s","station_m","X_m","Y_m","yaw_rad","vx_mps","vy_mps","yaw_rate_radps","steering_rad",
          "target_steering_rad","gross_acceleration_mps2","cross_track_m","sideslip_rad"]
    values=np.column_stack([ts,station,zs,us,e,beta])
    for key,prefix in (("tire_force_vertical_N","Fz"),("tire_force_longitudinal_N","Fx"),
                       ("tire_force_lateral_N","Fy"),("tire_slip_angle_rad","alpha"),("tire_longitudinal_slip","kappa")):
        cols.extend([f"{prefix}_{wheel}" for wheel in ("fl","fr","rl","rr")]);values=np.column_stack([values,dyn[key]])
    np.savetxt(output/"trajectory.csv",values[ids],delimiter=",",header=",".join(cols),comments="",fmt="%.9g")
    with (output/"controls.csv").open("w",newline="") as f:
        writer=csv.writer(f);writer.writerow(["time_s","steering_target_rad","gross_acceleration_mps2","torque_fl_Nm","torque_fr_Nm","torque_rl_Nm","torque_rr_Nm"]);writer.writerows(control_rows)
    if progress:print(json.dumps(summaries,indent=2),flush=True)
    return summaries


def collect(output):
    output=Path(output)
    rows=[json.loads(p.read_text()) for p in sorted(output.glob("*/*/summary.json"))]
    json_write(output/"summary.json",rows)
    if rows:
        keys=list(dict.fromkeys(k for row in rows for k in row))
        with (output/"summary.csv").open("w",newline="") as f:
            writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerows(rows)
    return rows
