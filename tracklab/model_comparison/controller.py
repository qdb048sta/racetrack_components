"""One fixed NMPC tracker shared by every comparison plant.

This is a deliberately common kinematic prediction model, not the MPC from any
of the cited theses. Keeping the controller, road, weights and bounds fixed
lets the closed-loop experiment expose the consequences of changing the plant.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import time
import numpy as np


@dataclass(frozen=True)
class Experiment:
    name: str = "nominal"
    maximum_speed_mps: float = 18.0
    lateral_acceleration_budget_mps2: float = 3.0
    planning_acceleration_mps2: float = 1.5
    planning_braking_mps2: float = 2.0
    speed_grid_spacing_m: float = .5
    initial_speed_mps: float = 8.0
    control_period_s: float = .1
    logging_step_s: float = .02
    horizon_steps: int = 25
    maximum_time_s: float = 600.0
    laps: float = 1.0
    corridor_center_margin_m: float = 2.4
    steering_limit_rad: float = .5
    steering_rate_limit_radps: float = .5
    steering_time_constant_s: float = .06
    acceleration_limit_mps2: float = 2.5
    braking_limit_mps2: float = 4.0
    native_step_s: float = .0005
    reduced_step_s: float = .001

    def __post_init__(self):
        values=asdict(self)
        if not self.name or any(not np.isfinite(v) or v<=0 for k,v in values.items() if k!="name"):
            raise ValueError("Experiment numeric settings must be finite and positive")
        ratio=self.control_period_s/self.logging_step_s
        if abs(ratio-round(ratio))>1e-9:
            raise ValueError("Control period must be an integer multiple of logging step")
        if isinstance(self.horizon_steps,bool) or int(self.horizon_steps)!=self.horizon_steps:
            raise ValueError("Horizon steps must be a positive integer")

    def to_dict(self):
        return asdict(self)


class CommonMPC:
    """Five prediction states [X,Y,psi,speed,delta], two held commands.

    The acceleration command is requested *gross* force divided by mass;
    the predictor subtracts rolling resistance and aerodynamic drag. Wheel
    torque allocation belongs to the common experiment, outside this tracker.
    """

    def __init__(self, road, speed_plan, experiment: Experiment, *, mass=800.,
                 lf=1.724, lr=1.476, rho=1.225, drag_area=1.,
                 downforce_area=1.5, rolling_coefficient=.025, gravity=9.81):
        import casadi as ca
        self.road, self.plan, self.cfg = road, speed_plan, experiment
        self.mass, self.lf, self.lr = mass, lf, lr
        self.resist = lambda v: (rolling_coefficient *
            (mass*gravity+.5*rho*downforce_area*v*v)+.5*rho*drag_area*v*v)/mass
        c = experiment
        N, dt = c.horizon_steps, c.control_period_s
        op = ca.Opti()
        X, U = op.variable(5, N+1), op.variable(2, N)
        x0, ref = op.parameter(5), op.parameter(5, N+1)
        normals, widths = op.parameter(2, N+1), op.parameter(2, N+1)
        previous = op.parameter(2)

        def rhs(x, u):
            beta = ca.atan(lr/(lf+lr)*ca.tan(x[4]))
            # Smooth PT1 prediction. A stage inequality below bounds its
            # initial slope; the slope then decays under the held target.
            rate = (u[0]-x[4])/c.steering_time_constant_s
            return ca.vertcat(x[3]*ca.cos(x[2]+beta),
                              x[3]*ca.sin(x[2]+beta),
                              x[3]*ca.cos(beta)*ca.tan(x[4])/(lf+lr),
                              u[1]-self.resist(x[3]), rate)

        op.subject_to(X[:, 0] == x0)
        objective = 0
        for j in range(N):
            op.subject_to(op.bounded(-c.steering_rate_limit_radps*c.steering_time_constant_s,
                                    U[0,j]-X[4,j],
                                    c.steering_rate_limit_radps*c.steering_time_constant_s))
            # Resolve the 60 ms PT1 actuator within a 100 ms control interval.
            x = X[:, j]
            for _ in range(4):
                h = dt/4
                k1=rhs(x,U[:,j]); k2=rhs(x+h*k1/2,U[:,j])
                k3=rhs(x+h*k2/2,U[:,j]); k4=rhs(x+h*k3,U[:,j])
                x = x+h*(k1+2*k2+2*k3+k4)/6
            op.subject_to(X[:, j+1] == x)
            du = U[:, j]-(previous if j == 0 else U[:, j-1])
            objective += 6*du[0]**2 + .06*du[1]**2 + .01*U[1,j]**2
            op.subject_to(op.bounded(-c.steering_rate_limit_radps*dt,
                                    du[0], c.steering_rate_limit_radps*dt))
        for j in range(N+1):
            dx = X[:2,j]-ref[:2,j]
            e = ca.dot(normals[:,j],dx)
            along = ca.dot(ca.vertcat(normals[1,j],-normals[0,j]),dx)
            objective += (3 if j == N else 1) * (
                40*e**2+.3*along**2+8*(X[2,j]-ref[2,j])**2+
                2*(X[3,j]-ref[3,j])**2+.2*(X[4,j]-ref[4,j])**2)
            op.subject_to(op.bounded(-widths[0,j]+c.corridor_center_margin_m,
                                    e, widths[1,j]-c.corridor_center_margin_m))
        op.subject_to(op.bounded(-c.steering_limit_rad,U[0,:],c.steering_limit_rad))
        op.subject_to(op.bounded(-c.braking_limit_mps2,U[1,:],c.acceleration_limit_mps2))
        op.subject_to(op.bounded(-c.steering_limit_rad,X[4,:],c.steering_limit_rad))
        op.subject_to(op.bounded(1.,X[3,:],max(40.,c.maximum_speed_mps+10)))
        op.minimize(objective)
        op.solver("ipopt",{"expand":True,"print_time":False},
                  {"print_level":0,"sb":"yes","max_iter":100,"tol":1e-7,
                   "acceptable_tol":1e-6,"acceptable_iter":3,
                   "warm_start_init_point":"yes","mu_init":.01})
        self.op,self.X,self.U,self.x0,self.ref = op,X,U,x0,ref
        self.normals,self.widths,self.previous = normals,widths,previous
        self.last_global = self.last_controls = None

    def solve(self, observation, station, previous_input):
        z=np.asarray(observation,dtype=float); c=self.cfg; N=c.horizon_steps
        speed=float(np.hypot(z[3],z[4]))
        ss,vv=self.plan.references(station,speed,N,c.control_period_s)
        q=self.road.sample_geometry(ss)
        origin=q["center"][0]; angle=float(q["heading_rad"][0])
        cs,sn=np.cos(angle),np.sin(angle)
        rot=np.array([[cs,-sn],[sn,cs]])
        delta=np.clip(np.arctan((self.lf+self.lr)*q["curvature_1pm"]),
                      -c.steering_limit_rad,c.steering_limit_rad)
        beta=np.arctan(self.lr/(self.lf+self.lr)*np.tan(delta))
        heading=np.unwrap(q["heading_rad"]-angle)-beta
        heading-=2*np.pi*round(heading[0]/(2*np.pi))
        ref=np.column_stack([(q["center"]-origin)@rot,heading,vv,delta]).T
        init=np.r_[(z[:2]-origin)@rot,np.arctan2(np.sin(z[2]-angle),np.cos(z[2]-angle)),speed,z[6]]
        ref[2]+=2*np.pi*round((init[2]-ref[2,0])/(2*np.pi))
        op=self.op
        op.set_value(self.x0,init);op.set_value(self.ref,ref)
        op.set_value(self.normals,(q["normal_left"]@rot).T)
        op.set_value(self.widths,np.vstack([q["width_right_m"],q["width_left_m"]]))
        op.set_value(self.previous,previous_input)
        if self.last_global is None:
            guess=ref.copy(); controls=np.vstack([delta[:-1],self.resist(vv[:-1])])
        else:
            old=np.column_stack([self.last_global[:,1:],self.last_global[:,-1]])
            guess=old.copy();guess[:2]=((old[:2].T-origin)@rot).T
            guess[2]=np.unwrap(old[2]-angle)
            guess[2]+=2*np.pi*round((init[2]-guess[2,0])/(2*np.pi))
            controls=np.column_stack([self.last_controls[:,1:],self.last_controls[:,-1]])
        guess[:,0]=init
        op.set_initial(self.X,guess);op.set_initial(self.U,controls)
        start=time.perf_counter()
        try:
            solution=op.solve()
        except RuntimeError as error:
            raise RuntimeError(f"Common MPC failed at {station:.2f} m: {op.stats().get('return_status')}") from error
        wall=time.perf_counter()-start
        states=np.asarray(solution.value(self.X));controls=np.asarray(solution.value(self.U))
        g=np.asarray(solution.value(op.g)).ravel()
        lo=np.asarray(solution.value(op.lbg)).ravel();hi=np.asarray(solution.value(op.ubg)).ravel()
        violation=float(max(0,np.max(lo-g),np.max(g-hi)))
        if violation>2e-5 or not np.isfinite(states).all():
            raise RuntimeError(f"Common MPC constraint residual {violation}")
        glob=states.copy();glob[:2]=(states[:2].T@rot.T+origin).T;glob[2]=states[2]+angle
        self.last_global,self.last_controls=glob,controls
        return controls[:,0].copy(),{"solve_seconds":wall,"constraint_violation":violation,
            "iterations":int(op.stats()["iter_count"]),"status":op.stats()["return_status"],
            "predicted_xy_m":glob[:2].T.tolist()}
