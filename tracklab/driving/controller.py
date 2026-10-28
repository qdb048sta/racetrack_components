"""Nonlinear trajectory-tracking MPC with numeric road preview.

Road samples are PARAMETERS, not symbolic SciPy calls. The predicted vehicle is
integrated in a local Cartesian frame. Rebuild the reference every 0.1 s and apply
only the first optimized input. The objective is tracking, NOT minimum lap time.
"""
from __future__ import annotations
import time
import math
import numpy as np
from ..geometry import wrap_angle
from .config import DrivingConfig

class TrackingMPC:
    def __init__(self,plant,road,speed_plan,config:DrivingConfig):
        import casadi as ca
        self.ca=ca;self.plant=plant;self.road=road;self.speed_plan=speed_plan;self.config=config
        p=plant.config;N=config.horizon_steps;dt=config.control_period_s
        op=ca.Opti();X=op.variable(7,N+1);U=op.variable(2,N)
        init=op.parameter(7);ref=op.parameter(7,N+1)
        normals=op.parameter(2,N+1);widths=op.parameter(2,N+1)
        mus=op.parameter(2,N);previous=op.parameter(2)
        f=plant.symbolic_rhs()
        op.subject_to(X[:,0]==init)
        objective=0
        for j in range(N):
            xnext=X[:,j];h=dt/config.prediction_substeps
            for _ in range(config.prediction_substeps):
                a=f(xnext,U[:,j],mus[:,j]);b=f(xnext+h*a/2,U[:,j],mus[:,j])
                c=f(xnext+h*b/2,U[:,j],mus[:,j]);d=f(xnext+h*c,U[:,j],mus[:,j])
                xnext=xnext+h*(a+2*b+2*c+d)/6
            op.subject_to(X[:,j+1]==xnext)
            difference=U[:,j]-(previous if j==0 else U[:,j-1])
            objective += 5*difference[0]**2+0.03*difference[1]**2
            objective += .02*U[0,j]**2+.008*U[1,j]**2
        for j in range(N+1):
            dx=X[:2,j]-ref[:2,j];normal=normals[:,j]
            e=ca.dot(normal,dx);along=ca.dot(ca.vertcat(normal[1],-normal[0]),dx)
            scale=3 if j==N else 1
            objective+=scale*(35*e**2+.5*along**2+12*(X[2,j]-ref[2,j])**2
                         +1.8*(X[3,j]-ref[3,j])**2+.15*(X[4,j]-ref[4,j])**2
                         +.8*(X[5,j]-ref[5,j])**2+.15*(X[6,j]-ref[6,j])**2)
            # Frozen reference cross sections. Actual geometry checked independently.
            margin=config.corridor_center_margin_m
            op.subject_to(e<=widths[1,j]-margin)
            op.subject_to(e>=-widths[0,j]+margin)
        op.subject_to(op.bounded(-p.steering_limit_rad,U[0,:],p.steering_limit_rad))
        op.subject_to(op.bounded(-p.braking_limit_mps2,U[1,:],p.acceleration_limit_mps2))
        op.subject_to(op.bounded(-p.steering_limit_rad,X[6,:],p.steering_limit_rad))
        op.subject_to(op.bounded(1.,X[3,:],max(25.,config.maximum_speed_mps+5)))
        op.subject_to(op.bounded(-3.,X[5,:],3.))
        op.minimize(objective)
        op.solver('ipopt',{'expand':True,'print_time':False},
                  {'print_level':0,'sb':'yes','max_iter':config.maximum_solver_iterations,
                   'tol':1e-6,'acceptable_tol':1e-5,'acceptable_iter':3,
                   'warm_start_init_point':'yes','mu_init':.01})
        self.op=op;self.X=X;self.U=U;self.init=init;self.ref=ref
        self.normals=normals;self.widths=widths;self.mus=mus;self.previous=previous
        self.last_global_prediction=None;self.last_inputs=None

    def solve(self,z,s,previous_input):
        p=self.plant.config;d=self.config;N=d.horizon_steps
        ss,vv=self.speed_plan.references(s,np.hypot(z[3],z[4]),N,d.control_period_s)
        q=self.road.sample_geometry(ss)
        origin=q['center'][0];theta=float(q['heading_rad'][0])
        ct,st=math.cos(theta),math.sin(theta)
        rot=np.array([[ct,-st],[st,ct]])
        centers=(q['center']-origin)@rot
        normals=q['normal_left']@rot
        headings=np.unwrap(q['heading_rad']-theta)
        headings-=2*np.pi*round(headings[0]/(2*np.pi))
        kap=q['curvature_1pm']
        # Approximate feedforward equilibrium, used only as a soft tracking target.
        beta=np.arctan(p.lr_m*kap)-vv*vv*kap/(p.gravity_mps2*p.cornering_coefficient_per_rad)
        steer=np.clip(np.arctan(p.wheelbase_m*kap),-p.steering_limit_rad,p.steering_limit_rad)
        reference=np.column_stack([centers,headings-beta,vv*np.cos(beta),vv*np.sin(beta),vv*kap,steer]).T
        initial=np.asarray(z,dtype=float).copy();initial[:2]=(z[:2]-origin)@rot
        initial[2]=float(wrap_angle(z[2]-theta))
        # Keep reference headings in the same local angular branch as the state.
        reference[2]+=2*np.pi*round((initial[2]-reference[2,0])/(2*np.pi))
        axes=q['tangent'][:-1]
        front=q['center'][:-1]+p.lf_m*axes;rear=q['center'][:-1]-p.lr_m*axes
        mf=self.road.surface.sample_xy(front)['friction_multiplier']
        mr=self.road.surface.sample_xy(rear)['friction_multiplier']
        op=self.op
        op.set_value(self.init,initial);op.set_value(self.ref,reference)
        op.set_value(self.normals,normals.T)
        op.set_value(self.widths,np.vstack([q['width_right_m'],q['width_left_m']]))
        op.set_value(self.mus,np.vstack([mf,mr]));op.set_value(self.previous,previous_input)
        if self.last_global_prediction is None:
            guess=reference.copy();guess[:,0]=initial
            controls=np.vstack([steer[:-1],np.zeros(N)])
        else:
            glob=np.column_stack([self.last_global_prediction[:,1:],self.last_global_prediction[:,-1]])
            guess=glob.copy();guess[:2]=((glob[:2].T-origin)@rot).T
            guess[2]=np.unwrap(glob[2]-theta)
            guess[2]+=2*np.pi*round((initial[2]-guess[2,0])/(2*np.pi))
            guess[:,0]=initial
            controls=np.column_stack([self.last_inputs[:,1:],self.last_inputs[:,-1]])
        op.set_initial(self.X,guess);op.set_initial(self.U,controls)
        start=time.perf_counter()
        try:
            solution=op.solve()
        except RuntimeError as exc:
            raise RuntimeError(f'MPC failed at progress {s:.2f} m: {op.stats().get("return_status")}') from exc
        elapsed=time.perf_counter()-start
        states=np.asarray(solution.value(self.X));inputs=np.asarray(solution.value(self.U))
        gv=np.asarray(solution.value(op.g)).ravel()
        lower=np.asarray(solution.value(op.lbg)).ravel();upper=np.asarray(solution.value(op.ubg)).ravel()
        violation=float(max(0.,np.max(lower-gv),np.max(gv-upper)))
        if violation>2e-5 or not np.isfinite(states).all():
            raise RuntimeError(f'Unacceptable MPC residual {violation} at progress {s}.')
        glob=states.copy();glob[:2]=(states[:2].T@rot.T+origin).T;glob[2]=states[2]+theta
        self.last_global_prediction=glob;self.last_inputs=inputs
        info={'success':bool(op.stats()['success']),'status':op.stats()['return_status'],
              'iterations':int(op.stats()['iter_count']),'solve_seconds':elapsed,
              'constraint_violation':violation,'objective':float(solution.value(op.f)),
              'predicted_xy_m':glob[:2].T.tolist(),'reference_s_m':ss.tolist(),
              'reference_speed_mps':float(self.speed_plan(s))}
        return inputs[:,0].copy(),info
