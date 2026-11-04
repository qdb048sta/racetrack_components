"""Run the spatial/temporal Gao MPC adaptation using PeriodicTrack geometry."""
import argparse
import json
import numpy as np
from tracklab import PeriodicTrack
from tracklab.paper_models.gao2014 import Parameters,FourWheel,SpatialMPC,TemporalMPC
from tracklab.paper_models.common import wrap,write_run


def run(track="data/Norisring.csv",steps=40,output="outputs/papers/gao2014",speed=10.,obstacle=False):
    if steps<1:raise ValueError("steps must be positive")
    road=PeriodicTrack(track);p=Parameters();plant=FourWheel(p);planner=SpatialMPC(p);tracker=TemporalMPC(plant)
    q=road.sample_geometry(0.);z=np.r_[q["center"],q["heading_rad"],speed,0.,0.]
    station=0.;u=np.zeros(3);previous_high=np.zeros(2);rows=[];residual=[];slacks=[];violations=[]
    # Actual required center offset is 1 m; add 0.4 m heuristic tracking margin.
    obstacles=[(25.,35.,1.4,"left")] if obstacle else []
    for j in range(steps):
        projection=road.project(z[:2],hint_s=station);station=projection["s_m"]
        q=road.sample_geometry(station)
        spatial=np.array([z[4],z[3],z[5],wrap(z[2]-q["heading_rad"]),projection["e_left_m"],0.])
        previous_high=np.array([u[0],.5*(u[1]+u[2])])
        path=planner.solve(road,station,spatial,speed,previous_high,obstacles)
        u,info=tracker.solve(z,path,u);previous_high=path["controls"][:,0]
        actual_violation=max(0.,1.-projection["e_left_m"]) if obstacle and 25<=station<=35 else 0.
        slacks.append(path["corridor_slack_max"]);violations.append(actual_violation)
        rows.append([j*tracker.dt,*z,*u,projection["e_left_m"],path["residual"],info["residual"],station,path["corridor_slack_max"],actual_violation])
        residual.extend([path["residual"],info["residual"]]);z=plant.step(z,u,tracker.dt)
    summary={"experiment":"Gao 2014 spatial/temporal MPC adaptation","steps":steps,"solves":2*steps,
             "max_constraint_residual":max(residual),"max_cross_track_m":float(np.max(np.abs(np.array(rows)[:,10]))),
             "max_planned_corridor_slack_m":max(slacks),"max_sampled_obstacle_violation_m":max(violations),
             "final_station_m":float(station),
             "obstacle_enabled":obstacle,"scope":"Fiala substitution; ideal side forces; nonlinear temporal tracker; no robust tube"}
    return write_run(output,["time_s","x_m","y_m","yaw_rad","vx_mps","vy_mps","yaw_rate_radps",
                             "steer_rad","left_effort","right_effort","cross_track_m","spatial_residual","temporal_residual",
                             "station_m","corridor_slack_m","sampled_obstacle_violation_m"],rows,summary,road,[1,2])


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--track",default="data/Norisring.csv")
    p.add_argument("--steps",type=int,default=40);p.add_argument("--output",default="outputs/papers/gao2014")
    p.add_argument("--speed",type=float,default=10.);p.add_argument("--obstacle",action="store_true");a=p.parse_args()
    print(json.dumps(run(a.track,a.steps,a.output,a.speed,a.obstacle),indent=2))
