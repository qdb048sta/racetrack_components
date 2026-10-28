"""A NEW reduced single-track plant, not native Open-Car-Dynamics bindings.

State z = [X,Y,psi,vx,vy,r,delta]; input = [requested steer, requested Fx/m].
Static axle loads; tanh lateral tire law with a combined-force cap; front steer;
ideal load-proportional longitudinal force allocation; no suspension/wheel RPM.
The same equation builder supports NumPy plant integration and CasADi prediction.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np
from .config import VehicleConfig

STATE_COLUMNS=('x_m','y_m','yaw_rad','vx_mps','vy_mps','yaw_rate_radps','steering_rad')

@dataclass(frozen=True)
class Actuation:
    steering_target_rad: float
    force_acceleration_mps2: float
    def array(self):
        return np.array([self.steering_target_rad,self.force_acceleration_mps2],dtype=float)

class SaturatingAxleTire:
    """Load-scaled tanh lateral force with remaining grip after longitudinal force.

    Not Pacejka, not the upstream linear.cpp function, and not fitted tire data.
    Linear small-slip limit: Fy ~= c_alpha * Fz * alpha.
    """
    def __init__(self, coefficient_per_rad=14.0):
        self.coefficient=coefficient_per_rad

    def lateral(self, alpha, load, fx, mu, ops=np):
        peak=mu*load
        # Guard is inactive for the validated default command/friction range.
        cap=ops.sqrt(ops.fmax(peak*peak-fx*fx,0.0025*peak*peak))
        return cap*ops.tanh(self.coefficient*load*alpha/cap)

class DynamicBicycle:
    def __init__(self, config: VehicleConfig):
        config.validate();self.config=config
        self.tire=SaturatingAxleTire(config.cornering_coefficient_per_rad)

    def evaluate(self,z,u,mu_front=1.0,mu_rear=1.0,ops=np):
        p=self.config
        vx,vy,r,delta=z[3],z[4],z[5],z[6]
        sf,cf=ops.sin(delta),ops.cos(delta)
        # Exact forward-frame slip-angle convention with a low-speed regularizer.
        vden=ops.sqrt(vx*vx+p.velocity_regularization_mps**2)
        af=delta-ops.arctan2(vy+p.lf_m*r,vden)
        ar=-ops.arctan2(vy-p.lr_m*r,vden)
        fzf=p.mass_kg*p.gravity_mps2*p.lr_m/p.wheelbase_m
        fzr=p.mass_kg*p.gravity_mps2*p.lf_m/p.wheelbase_m
        # mu_front/rear are ROAD MULTIPLIERS; nominal mu is a separate assumption.
        muf=p.nominal_friction*mu_front;mur=p.nominal_friction*mu_rear
        fxf=fzf*u[1]/p.gravity_mps2;fxr=fzr*u[1]/p.gravity_mps2
        # Respect available longitudinal grip even for an explicitly changed surface.
        fxf=ops.fmax(-0.9*muf*fzf,ops.fmin(0.9*muf*fzf,fxf))
        fxr=ops.fmax(-0.9*mur*fzr,ops.fmin(0.9*mur*fzr,fxr))
        fyf=self.tire.lateral(af,fzf,fxf,muf,ops)
        fyr=self.tire.lateral(ar,fzr,fxr,mur,ops)
        frontx=cf*fxf-sf*fyf;fronty=sf*fxf+cf*fyf
        drag=0.5*p.air_density_kgpm3*p.drag_coefficient_area_m2*vx*ops.fabs(vx)
        fx=frontx+fxr-drag;fy=fronty+fyr
        mz=p.lf_m*fronty-p.lr_m*fyr
        # Smooth bounded actuator: first-order lag near zero error, bounded rate.
        ddelta=p.steering_rate_limit_radps*ops.tanh(
                  (u[0]-delta)/(p.steering_time_constant_s*p.steering_rate_limit_radps))
        cp,sp=ops.cos(z[2]),ops.sin(z[2])
        rhs=[vx*cp-vy*sp,vx*sp+vy*cp,r,fx/p.mass_kg+r*vy,
             fy/p.mass_kg-r*vx,mz/p.yaw_inertia_kgm2,ddelta]
        details={'alpha_front_rad':af,'alpha_rear_rad':ar,'fy_front_n':fyf,
                 'fy_rear_n':fyr,'fx_front_n':fxf,'fx_rear_n':fxr,
                 'fz_front_n':fzf,'fz_rear_n':fzr,'ax_body_mps2':fx/p.mass_kg,
                 'ay_body_mps2':fy/p.mass_kg,'yaw_moment_nm':mz,
                 'front_grip_utilization':ops.sqrt(fxf*fxf+fyf*fyf)/(muf*fzf),
                 'rear_grip_utilization':ops.sqrt(fxr*fxr+fyr*fyr)/(mur*fzr)}
        return rhs,details

    def rhs(self,z,u,mu_front=1.,mu_rear=1.):
        z=np.asarray(z,dtype=float);u=np.asarray(u,dtype=float)
        return np.asarray(self.evaluate(z,u,mu_front,mu_rear)[0],dtype=float)

    def diagnostics(self,z,u,mu_front=1.,mu_rear=1.):
        return {k:float(v) for k,v in self.evaluate(z,u,mu_front,mu_rear)[1].items()}

    def step(self,z,u,dt,mu_front=1.,mu_rear=1.):
        """One explicit RK4 plant step, with actuation held constant."""
        if dt<=0:raise ValueError('dt must be positive.')
        z=np.asarray(z,dtype=float);u=np.asarray(u,dtype=float)
        p=self.config
        if np.any(np.abs(z)>1e8) or not np.isfinite(z).all():
            raise FloatingPointError('Invalid vehicle state.')
        if abs(u[0])>p.steering_limit_rad+1e-7 or not -p.braking_limit_mps2-1e-7<=u[1]<=p.acceleration_limit_mps2+1e-7:
            raise ValueError('Actuation outside configured limits.')
        f=lambda q:self.rhs(q,u,mu_front,mu_rear)
        k1=f(z);k2=f(z+dt*k1/2);k3=f(z+dt*k2/2);k4=f(z+dt*k3)
        out=z+dt*(k1+2*k2+2*k3+k4)/6
        if not np.isfinite(out).all():raise FloatingPointError('Nonfinite integrated state.')
        return out

    def symbolic_rhs(self):
        import casadi as ca
        # Small adapter for numpy spellings shared by evaluate().
        class Ops:
            sin=ca.sin;cos=ca.cos;sqrt=ca.sqrt;tanh=ca.tanh
            arctan2=ca.atan2;fmax=ca.fmax;fmin=ca.fmin;fabs=ca.fabs
        z=ca.SX.sym('z',7);u=ca.SX.sym('u',2);mu=ca.SX.sym('mu',2)
        rhs,_=self.evaluate(z,u,mu[0],mu[1],Ops)
        return ca.Function('dynamic_bicycle_rhs',[z,u,mu],[ca.vertcat(*rhs)])
