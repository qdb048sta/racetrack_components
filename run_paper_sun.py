"""Run either Sun kinematic road tracking or nominal spatial car following."""
import argparse
import json
import numpy as np
from tracklab import PeriodicTrack
from tracklab.paper_models.sun2025 import KinematicBicycle,KinematicMPC,Longitudinal,SpatialFollowingMPC
from tracklab.paper_models.common import road_reference,write_run,rk4


def run(track="data/Norisring.csv",steps=30,output="outputs/papers/sun2025",mode="kinematic"):
    if steps<1:raise ValueError("steps must be positive")
    road=PeriodicTrack(track);rows=[];residual=[]
    if mode=="kinematic":
        plant=KinematicBicycle();controller=KinematicMPC(plant);speed=8.
        q=road.sample_geometry(0.);z=np.r_[q["center"],q["heading_rad"],speed,0.];station=0.
        for j in range(steps):
            pr=road.project(z[:2],hint_s=station);station=pr["s_m"]
            q,heading=road_reference(road,station,speed,controller.dt,controller.N,z[2])
            width=np.vstack([q["width_right_m"]-1.,q["width_left_m"]-1.])
            u,info=controller.solve(z,q["center"].T,q["normal_left"].T,width,heading)
            rows.append([j*controller.dt,*z,*u,pr["e_left_m"]]);residual.append(info["residual"])
            z=plant.step(z,u,controller.dt)
        summary={"experiment":"Sun 2025 kinematic tracking core","steps":steps,"solves":steps,
                 "max_constraint_residual":max(residual),"max_cross_track_m":float(np.max(np.abs(np.array(rows)[:,-1]))),
                 "scope":"Supplied track reference; no APF, multi-agent protocol or tire-force model"}
        return write_run(output,["time_s","x_m","y_m","yaw_rad","speed_mps","steer_rad","accel_mps2","steer_rate_radps","cross_track_m"],rows,summary,road,[1,2])
    if mode!="following":raise ValueError("Unknown mode")
    plant=Longitudinal();controller=SpatialFollowingMPC(plant);x=np.array([1.5,.5*plant.p.mass*10**2])
    prior=plant.resistance(10.)*plant.p.radius/(plant.p.motors*plant.p.efficiency);gaps=[];fallbacks=0;candidate_gaps=[]
    for j in range(steps):
        stations=(j+np.arange(controller.N))*controller.ds
        xy=road.sample_geometry(stations)["center"]
        height=road.surface.sample_xy(xy)["height_m"]
        slopes=np.arctan(np.gradient(height,controller.ds))
        leader=np.full(controller.N+1,10.) # explicit supplied predictor
        torque,info=controller.solve(x,leader,slopes,prior)
        fallbacks+=int(info["physical_fallback"]);candidate_gaps.append(info["relaxed_candidate_gap_max"])
        gap=float(np.max(np.abs(info["relaxation_gap"])));gaps.append(gap)
        if gap>1e-4:raise RuntimeError(f"Convex relaxation is not tight: {gap:g} s/m")
        rows.append([j*controller.ds,x[0],x[1],np.sqrt(2*x[1]/plant.p.mass),torque,gap,int(info["physical_fallback"]),info["relaxed_candidate_gap_max"]])
        # Propagate the physical equality, not the virtual xi control.
        f=lambda xx,uu:plant.spatial_rhs(xx,uu,leader[0],slopes[0])
        x=rk4(f,x,torque,controller.ds);prior=torque;residual.append(info["residual"])
    summary={"experiment":"Sun 2025 nominal spatial following","steps":steps,"solves":steps,
             "max_constraint_residual":max(residual),"max_relaxation_gap_s_per_m":max(gaps),
             "physical_equality_fallbacks":fallbacks,"total_optimization_solves":steps+fallbacks,
             "max_relaxed_candidate_gap_s_per_m":max(candidate_gaps),
             "scope":"Supplied constant leader; no learned predictor or robust tube guarantee"}
    return write_run(output,["station_m","time_headway_s","energy_j","speed_mps","motor_torque_nm","relaxation_gap_s_per_m","physical_fallback","relaxed_candidate_gap_s_per_m"],rows,summary)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--track",default="data/Norisring.csv")
    p.add_argument("--steps",type=int,default=30);p.add_argument("--output",default="outputs/papers/sun2025")
    p.add_argument("--mode",choices=["kinematic","following"],default="kinematic");a=p.parse_args()
    print(json.dumps(run(a.track,a.steps,a.output,a.mode),indent=2))
