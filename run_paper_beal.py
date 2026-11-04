"""Run the Beal envelope-control adaptation on the repository's interpolated road."""
import argparse
import json
import numpy as np
from tracklab import PeriodicTrack
from tracklab.paper_models.beal2011 import Parameters,Bicycle,EnvelopeMPC,brush
from tracklab.paper_models.common import wrap,write_run


def run(track="data/Norisring.csv",steps=200,output="outputs/papers/beal2011",speed=12.):
    if steps<1:raise ValueError("steps must be positive")
    road=PeriodicTrack(track);p=Parameters(speed=speed);plant=Bicycle(p);controller=EnvelopeMPC(plant)
    q=road.sample_geometry(0.);z=np.r_[q["center"],q["heading_rad"],0.,0.]
    station=0.;steer=0.;rows=[];slack=[];residual=[]
    for j in range(steps):
        projection=road.project(z[:2],hint_s=station);station=projection["s_m"]
        look=road.sample_geometry(station+speed*.5)
        driver=float(np.clip((p.a+p.b)*look["curvature_1pm"]-1.2*wrap(z[2]-look["heading_rad"])-.04*projection["e_left_m"],-.25,.25))
        force=float(brush(z[3]+p.a*z[4]/speed-steer,p.cf,p.mu*p.loads[0],p.sliding_ratio))
        command,info=controller.solve(z,driver,force)
        rows.append([j*controller.dt,*z,driver,steer,command,info["force_n"],info["slack_max"],projection["e_left_m"]])
        z=plant.step(z,steer,controller.dt);steer=command
        slack.append(info["slack_max"]);residual.append(info["constraint_residual"])
    summary={"experiment":"Beal 2011 envelope MPC adaptation","steps":steps,"solves":steps,
             "dt_s":controller.dt,"max_envelope_slack":max(slack),"max_constraint_residual":max(residual),
             "max_cross_track_m":float(np.max(np.abs(np.array(rows)[:,-1]))),
             "scope":"Chapter 4 lateral envelope; synthetic road-following driver; no roll/friction estimator"}
    return write_run(output,["time_s","x_m","y_m","yaw_rad","beta_rad","yaw_rate_radps","driver_steer_rad",
                             "applied_steer_rad","next_steer_rad","next_force_n","slack_max","cross_track_m"],rows,summary,road,[1,2])


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--track",default="data/Norisring.csv")
    p.add_argument("--steps",type=int,default=200);p.add_argument("--output",default="outputs/papers/beal2011")
    p.add_argument("--speed",type=float,default=12.);a=p.parse_args()
    print(json.dumps(run(a.track,a.steps,a.output,a.speed),indent=2))
