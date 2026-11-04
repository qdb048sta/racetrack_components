"""Gao (2014): Chapter 4 spatial planner and temporal tracking architecture.

The executable adaptation uses the modified Fiala option from Sec. 2.4 at all
four wheels, since the dissertation omits its combined-slip Pacejka calibration.
Inputs to the temporal model are steer and left/right normalized wheel forces.
They are ideal force commands, not calibrated brake torques or wheel-spin states.
"""
from dataclasses import dataclass
import numpy as np
import casadi as ca
from .common import rk4, ipopt_options, solve_checked


@dataclass(frozen=True)
class Parameters:
    mass: float = 1500.
    iz: float = 2500.
    a: float = 1.2
    b: float = 1.5
    half_track: float = .8
    cf_axle: float = 80000.
    cr_axle: float = 90000.
    mu: float = .7
    gravity: float = 9.81

    def __post_init__(self):
        if min(vars(self).values()) <= 0:
            raise ValueError("All vehicle parameters must be positive")


def magic_formula(x, B, C, D, E=0., Sh=0., Sv=0., ops=np):
    """Pure-slip Eqs. 2.21-2.22; no invented combined-slip calibration."""
    y=x+Sh
    return D*ops.sin(C*ops.arctan(B*y-E*(B*y-ops.arctan(B*y))))+Sv


def fiala(alpha, stiffness, load, mu, fx, symbolic=False):
    """Eq. 2.23 corrected to tan^3 in the last term, with Eq. 2.24 threshold.

    Printed tan^2 violates odd symmetry and saturation continuity. The correction
    is explicit in documentation/papers/gao2014.md and has regression tests.
    """
    o=ca if symbolic else np
    if not symbolic and (stiffness<=0 or load<=0 or mu<=0 or np.any(np.abs(fx)>mu*load+1e-8)):
        raise ValueError("Invalid tire parameters or force outside friction circle")
    cap=o.sqrt(o.fmax((mu*load)**2-fx**2,0.))
    safe=o.fmax(cap,1e-9)
    tangent=o.tan(alpha)
    polynomial=-stiffness*tangent+stiffness**2/(3*safe)*o.fabs(tangent)*tangent-stiffness**3/(27*safe**2)*tangent**3
    choose=ca.if_else if symbolic else np.where
    return choose(o.fabs(tangent)<3*cap/stiffness,polynomial,-cap*o.sign(tangent))


class FourWheel:
    """Six chassis states [X,Y,psi,vx,vy,r]; static per-wheel normal loads."""
    def __init__(self,p=Parameters()):
        self.p=p
        x=ca.SX.sym("x",6);u=ca.SX.sym("u",3)
        self.symbolic=ca.Function("gao_fourwheel",[x,u],[self._rhs(x,u)])

    def _rhs(self,x,u):
        p=self.p;delta=u[0];vx,vy,r=x[3],x[4],x[5]
        totalx=0;totaly=0;moment=0
        for longitudinal,front,load,c in [(p.a,True,p.mass*p.gravity*p.b/(2*(p.a+p.b)),p.cf_axle/2),
                                         (-p.b,False,p.mass*p.gravity*p.a/(2*(p.a+p.b)),p.cr_axle/2)]:
            steer=delta if front else 0
            for lateral,effort in [(p.half_track,u[1]),(-p.half_track,u[2])]:
                wx=vx-r*lateral;wy=vy+r*longitudinal
                vl=wx*ca.cos(steer)+wy*ca.sin(steer)
                vc=wy*ca.cos(steer)-wx*ca.sin(steer)
                alpha=ca.atan2(vc,vl)
                fx=effort*p.mu*load
                fy=fiala(alpha,c,load,p.mu,fx,True)
                bx=fx*ca.cos(steer)-fy*ca.sin(steer)
                by=fx*ca.sin(steer)+fy*ca.cos(steer)
                totalx+=bx;totaly+=by;moment+=longitudinal*by-lateral*bx
        return ca.vertcat(vx*ca.cos(x[2])-vy*ca.sin(x[2]),vx*ca.sin(x[2])+vy*ca.cos(x[2]),r,
                          totalx/p.mass+r*vy,totaly/p.mass-r*vx,moment/p.iz)

    def rhs(self,x,u):
        return np.array(self.symbolic(x,u)).ravel()

    def step(self,x,u,dt):
        if np.asarray(x).shape!=(6,) or np.asarray(u).shape!=(3,):
            raise ValueError("Expected six chassis states and three controls")
        if x[3]<=1 or abs(u[0])>.5 or max(abs(np.asarray(u)[1:]))>.9+1e-8:
            raise ValueError("Outside the validated forward-driving/actuation domain")
        n=max(1,int(np.ceil(dt/.005)));h=dt/n
        for _ in range(n):x=rk4(self.rhs,x,u,h)
        return x


class SpatialBicycle:
    """State [vy,vx,r,e_psi,e_y,elapsed_time], independent variable road station.

    Eqs. 4.4-4.6 with axle-lumped tires. FourWheel's zero-track counterpart
    supplies exactly the nonlinear bicycle chassis balances (2.13-2.14).
    """
    def __init__(self,p=Parameters()):
        self.p=p
        z=ca.SX.sym("z",6);u=ca.SX.sym("u",2);k=ca.SX.sym("curvature")
        vy,vx,r,ep,ey=z[0],z[1],z[2],z[3],z[4]
        delta,beta=u[0],u[1]
        fzf=p.mass*p.gravity*p.b/(p.a+p.b);fzr=p.mass*p.gravity*p.a/(p.a+p.b)
        fyf=fiala(ca.atan2(vy+p.a*r,vx)-delta,p.cf_axle,fzf,p.mu,beta*p.mu*fzf,True)
        fyr=fiala(ca.atan2(vy-p.b*r,vx),p.cr_axle,fzr,p.mu,beta*p.mu*fzr,True)
        fxf=beta*p.mu*fzf;fxr=beta*p.mu*fzr
        bx=fxf*ca.cos(delta)-fyf*ca.sin(delta)+fxr
        by=fxf*ca.sin(delta)+fyf*ca.cos(delta)+fyr
        yaw=(p.a*(fxf*ca.sin(delta)+fyf*ca.cos(delta))-p.b*fyr)/p.iz
        sd=(vx*ca.cos(ep)-vy*ca.sin(ep))/(1-k*ey)
        rhs=ca.vertcat(by/p.mass-r*vx,bx/p.mass+r*vy,yaw,r-k*sd,
                       vx*ca.sin(ep)+vy*ca.cos(ep),1)/sd
        self.symbolic=ca.Function("gao_spatial",[z,u,k],[rhs])

    def rhs(self,z,u,curvature):
        if 1-curvature*z[4]<=.2 or z[1]*np.cos(z[3])-z[0]*np.sin(z[3])<=1:
            raise ValueError("Spatial formulation requires positive forward progress and nonsingular road coordinates")
        return np.array(self.symbolic(z,u,curvature)).ravel()


class SpatialMPC:
    """Chapter 4 architecture with fixed passing side and logged soft corridor.

    Track widths remain hard. Obstacle corridor slack is an explicit adaptation
    inspired by Chapter 5's soft constraints, without its robust-set guarantee.
    """
    def __init__(self,p=Parameters(),horizon=15,ds=1.5):
        if horizon<3 or ds<=0:raise ValueError("Invalid spatial horizon")
        self.model=SpatialBicycle(p);self.N=horizon;self.ds=ds
        op=self.op=ca.Opti();N=horizon
        self.X=X=op.variable(6,N+1);self.U=U=op.variable(2,N)
        self.S=S=op.variable(2,N+1)
        self.x0=op.parameter(6);self.k=op.parameter(1,N+1)
        self.lower=op.parameter(1,N+1);self.upper=op.parameter(1,N+1)
        self.road_lower=op.parameter(1,N+1);self.road_upper=op.parameter(1,N+1)
        self.target=op.parameter();self.previous=op.parameter(2)
        op.subject_to(X[:,0]==self.x0)
        op.subject_to(op.bounded(2.,X[1,:],30.))
        op.subject_to(op.bounded(-.6,X[3,:],.6))
        op.subject_to(op.bounded(self.road_lower,X[4,:],self.road_upper))
        op.subject_to(ca.vec(S)>=0)
        op.subject_to(X[4,:]>=self.lower-S[0,:])
        op.subject_to(X[4,:]<=self.upper+S[1,:])
        op.subject_to(op.bounded(-np.deg2rad(10),U[0,:],np.deg2rad(10)))
        op.subject_to(op.bounded(-.8,U[1,:],.8))
        cost=1e4*ca.sumsqr(S)+1000*ca.sum1(ca.vec(S))
        for j in range(N):
            z=X[:,j];u=U[:,j];curv=self.k[j]
            f=lambda zz,uu:self.model.symbolic(zz,uu,curv)
            op.subject_to(X[:,j+1]==rk4(f,z,u,ds))
            # Conservative local domain, including the terminal node below.
            op.subject_to(1-curv*z[4]>=.2)
            op.subject_to(z[1]*ca.cos(z[3])-z[0]*ca.sin(z[3])>=1.)
            delta=u-(self.previous if j==0 else U[:,j-1])
            steer_change=np.deg2rad(17)*(.05 if j==0 else ds/30.)
            op.subject_to(op.bounded(-steer_change,delta[0],steer_change))
            op.subject_to(op.bounded(-.3,delta[1],.3))
            cost+=(z[1]-self.target)**2+(z[2]-curv*self.target)**2+20*z[3]**2+4*z[4]**2
            cost+=50*ca.sumsqr(u)+.1*ca.sumsqr(delta)
            if j%3!=0 or j>=12:op.subject_to(u==U[:,j-1])
        op.subject_to(1-self.k[N]*X[4,N]>=.2)
        op.subject_to(X[1,N]*ca.cos(X[3,N])-X[0,N]*ca.sin(X[3,N])>=1.)
        op.minimize(cost);op.solver("ipopt",*ipopt_options());self.last=None

    def solve(self,road,station,z,speed=10.,previous=(0.,0.),obstacles=()):
        q=road.sample_geometry(station+self.ds*np.arange(self.N+1))
        low=-q["width_right_m"]+1.;high=q["width_left_m"]-1.
        road_low,road_high=low.copy(),high.copy()
        for start,end,edge,side in obstacles:
            mask=(station+self.ds*np.arange(self.N+1)>=start)&(station+self.ds*np.arange(self.N+1)<=end)
            if side=="left":low[mask]=np.maximum(low[mask],edge)
            elif side=="right":high[mask]=np.minimum(high[mask],edge)
            else:raise ValueError("Obstacle side must be left or right")
        if np.any(low>=high):raise ValueError("No feasible corridor")
        for param,val in [(self.x0,z),(self.k,q["curvature_1pm"]),(self.lower,low),(self.upper,high),
                          (self.target,speed),(self.previous,previous),(self.road_lower,road_low),(self.road_upper,road_high)]:self.op.set_value(param,val)
        self.op.set_initial(self.S,0)
        if self.last is None:
            guess=np.tile(np.asarray(z)[:,None],(1,self.N+1))
            guess[5]=np.arange(self.N+1)*self.ds/max(z[1],2.)
            self.op.set_initial(self.X,guess);self.op.set_initial(self.U,0)
        else:
            gx,gu=self.last;gx=gx.copy();gx[5]-=gx[5,0]
            self.op.set_initial(self.X,gx);self.op.set_initial(self.U,gu)
        sol,res=solve_checked(self.op);x=np.array(sol.value(self.X));u=np.array(sol.value(self.U))
        self.last=x,u
        heading=np.unwrap(q["heading_rad"])+x[3]
        xy=q["center"]+x[4,:,None]*q["normal_left"]
        world=np.vstack([xy.T,heading,x[1],x[0],x[2]])
        return {"states":x,"controls":u,"world":world,"time":x[5],"residual":res,
                "corridor_slack_max":float(np.max(sol.value(self.S)))}


class TemporalMPC:
    """Temporal nonlinear tracker with four-wheel balances and differential effort.

    This uses nonlinear multiple shooting, not the thesis's LTV/SQP implementation.
    """
    def __init__(self,plant,horizon=5,dt=.05):
        self.plant,self.N,self.dt=plant,horizon,dt
        op=self.op=ca.Opti();N=horizon
        self.X=X=op.variable(6,N+1);self.U=U=op.variable(3,N)
        self.initial=op.parameter(6);self.ref=op.parameter(6,N+1);self.previous=op.parameter(3)
        self.feedforward=op.parameter(3,N)
        op.subject_to(X[:,0]==self.initial);op.subject_to(op.bounded(2,X[3,:],30))
        op.subject_to(op.bounded(-np.deg2rad(10),U[0,:],np.deg2rad(10)))
        op.subject_to(op.bounded(-.8,U[1:,:],.8));cost=0
        for j in range(N):
            q=X[:,j];u=U[:,j]
            op.subject_to(X[:,j+1]==rk4(plant.symbolic,q,u,dt))
            e=q-self.ref[:,j];du=u-(self.previous if j==0 else U[:,j-1])
            cost+=20*ca.sumsqr(e[:2])+20*e[2]**2+e[3]**2+e[5]**2+.1*ca.sumsqr(u)+ca.sumsqr(du)
            cost+=50*ca.sumsqr(u-self.feedforward[:,j])
            op.subject_to(op.bounded(-np.deg2rad(17)*dt,du[0],np.deg2rad(17)*dt))
            op.subject_to(op.bounded(-.25,du[1:],.25))
        cost+=20*ca.sumsqr(X[:2,N]-self.ref[:2,N])
        op.minimize(cost);op.solver("ipopt",*ipopt_options());self.last=None

    def solve(self,z,path,previous):
        times=np.arange(self.N+1)*self.dt
        if path["time"][-1]<times[-1]:raise ValueError("Spatial reference too short")
        ref=np.vstack([np.interp(times,path["time"],row) for row in path["world"]])
        control=np.vstack([np.interp(times[:-1],path["time"][:-1],row) for row in path["controls"]])
        feedforward=np.vstack([control[0],control[1],control[1]])
        ref[2]+=2*np.pi*np.round((z[2]-ref[2,0])/(2*np.pi))
        for param,val in [(self.initial,z),(self.ref,ref),(self.previous,previous),(self.feedforward,feedforward)]:self.op.set_value(param,val)
        self.op.set_initial(self.X,ref);self.op.set_initial(self.U,np.tile(previous,(self.N,1)).T)
        sol,res=solve_checked(self.op)
        return np.array(sol.value(self.U[:,0])).ravel(),{"residual":res,"prediction":np.array(sol.value(self.X))}
