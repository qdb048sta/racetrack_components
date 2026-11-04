"""Sun (2025) model comparison: longitudinal energy and kinematic bicycle.

Includes executable nominal spatial MPC and Chapter 5 tracking MPC. It does NOT
claim the learned MFST predictor, robust tube/observer proof, or APF formation
planner. Predecessor predictions and collision-free references are explicit inputs.
"""
from dataclasses import dataclass
import numpy as np
import casadi as ca
from .common import rk4, ipopt_options, solve_checked


@dataclass(frozen=True)
class LongitudinalParameters:
    mass: float = 1500.
    radius: float = .31
    efficiency: float = .95
    drag: float = .4
    rolling: float = .012
    motors: int = 2
    gravity: float = 9.81

    def __post_init__(self):
        if min(self.mass,self.radius,self.efficiency,self.gravity)<=0 or self.drag<0 or self.rolling<0 or self.motors<1:
            raise ValueError("Invalid longitudinal parameters")


class Longitudinal:
    def __init__(self,p=LongitudinalParameters()):self.p=p

    def force(self,torque):
        return self.p.motors*self.p.efficiency*torque/self.p.radius

    def resistance(self,v,slope=0.):
        p=self.p
        return p.drag*v*v+p.mass*p.gravity*(np.sin(slope)+p.rolling*np.cos(slope))

    def time_rhs(self,x,torque,slope=0.):
        """Eq. 3.4; motors=1 gives the total-torque convention in Eq. 4.1."""
        return np.array([x[1],(self.force(torque)-self.resistance(x[1],slope))/self.p.mass])

    def spatial_rhs(self,x,torque,leader_speed,slope=0.):
        """Physical Eq. 3.6/4.5: state [time headway, kinetic energy]."""
        if x[1]<=0 or leader_speed<=0:raise ValueError("Spatial dynamics require positive speed/energy")
        speed=np.sqrt(2*x[1]/self.p.mass)
        return np.array([1/speed-1/leader_speed,self.force(torque)-self.resistance(speed,slope)])


class SpatialFollowingMPC:
    """Nominal convex relaxation (3.7)/(4.9), not the full robust DMPC (4.26).

    Energy is scaled by m*10^2/2 to condition the optimization. A positive linear
    xi cost encourages tightness; every solution reports xi - 1/v. Tightness is
    checked and is NOT assumed for arbitrary weights/scenarios. A loose result
    triggers a reported physical-equality nonlinear solve by default.
    """
    def __init__(self,plant,horizon=20,ds=2.,gap=1.5,relaxed=True):
        self.plant,self.N,self.ds,self.gap=plant,horizon,ds,gap
        self.relaxed=relaxed;self.physical_solver=None
        self.scale=.5*plant.p.mass*100
        op=self.op=ca.Opti();N=horizon;p=plant.p
        self.X=X=op.variable(2,N+1);self.U=U=op.variable(2,N) # torque/500, xi [s/m]
        self.initial=op.parameter(2);self.leader=op.parameter(1,N+1);self.slope=op.parameter(1,N)
        self.prior=op.parameter()
        op.subject_to(X[:,0]==self.initial)
        op.subject_to(op.bounded(.5,X[0,:],3.))
        op.subject_to(op.bounded(.25,X[1,:],9.)) # speeds 5..30 m/s
        op.subject_to(op.bounded(-1,U[0,:],1));op.subject_to(op.bounded(1/30,U[1,:],1/5))
        cost=0
        for j in range(N):
            E=X[1,j]*self.scale;v=ca.sqrt(2*E/p.mass);torque=500*U[0,j];xi=U[1,j]
            resistance=2*p.drag/p.mass*E+p.mass*p.gravity*(ca.sin(self.slope[j])+p.rolling*ca.cos(self.slope[j]))
            op.subject_to(X[0,j+1]==X[0,j]+ds*(xi-1/self.leader[j]))
            op.subject_to(X[1,j+1]==X[1,j]+ds*(p.motors*p.efficiency*torque/p.radius-resistance)/self.scale)
            op.subject_to(xi>=1/v if relaxed else xi==1/v)
            du=U[0,j]-(self.prior if j==0 else U[0,j-1])
            op.subject_to(op.bounded(-.3,du,.3))
            target=.5*p.mass*self.leader[j]**2/self.scale
            cost+=20*(X[0,j]-gap)**2+2*(X[1,j]-target)**2+.02*U[0,j]**2+.1*du**2+20*xi
        cost+=20*(X[0,N]-gap)**2+2*(X[1,N]-.5*p.mass*self.leader[N]**2/self.scale)**2
        op.minimize(cost);op.solver("ipopt",*ipopt_options())

    def solve(self,x,leader,slope=None,previous_torque=0.,allow_physical_fallback=True):
        leader=np.asarray(leader,dtype=float)
        if leader.shape!=(self.N+1,) or np.any(leader<=0):raise ValueError("Provide N+1 positive predecessor speeds")
        slope=np.zeros(self.N) if slope is None else np.asarray(slope)
        if slope.shape!=(self.N,):raise ValueError("Provide N slopes")
        state=np.array([x[0],x[1]/self.scale])
        for param,val in [(self.initial,state),(self.leader,leader),(self.slope,slope),(self.prior,previous_torque/500)]:self.op.set_value(param,val)
        self.op.set_initial(self.X,np.tile(state,(self.N+1,1)).T)
        self.op.set_initial(self.U,np.vstack([np.full(self.N,previous_torque/500),1/leader[:-1]]))
        sol,res=solve_checked(self.op);X=np.array(sol.value(self.X));U=np.array(sol.value(self.U))
        physical=X.copy();physical[1]*=self.scale
        slack=U[1]-1/np.sqrt(2*physical[1,:-1]/self.plant.p.mass)
        gap=float(np.max(np.abs(slack)))
        if self.relaxed and gap>1e-4 and allow_physical_fallback:
            # A documented implementation adaptation: do not apply an unrealizable
            # virtual-time command when this nominal relaxation is not tight.
            if self.physical_solver is None:
                self.physical_solver=SpatialFollowingMPC(self.plant,self.N,self.ds,self.gap,relaxed=False)
            torque,info=self.physical_solver.solve(x,leader,slope,previous_torque)
            info.update(physical_fallback=True,relaxed_candidate_gap_max=gap)
            return torque,info
        return 500*U[0,0],{"prediction":physical,"controls":U,"relaxation_gap":slack,"residual":res,
                           "physical_fallback":False,"relaxed_candidate_gap_max":gap}


@dataclass(frozen=True)
class KinematicParameters:
    lf: float = 1.2
    lr: float = 1.5
    steer_limit: float = .5
    steer_rate: float = .5
    acceleration: float = 3.

    def __post_init__(self):
        if min(vars(self).values())<=0:raise ValueError("Invalid kinematic parameters")


class KinematicBicycle:
    """Eq. 5.1, [X,Y,heading,v,delta], u=[acceleration,steering rate].

    beta is a geometric velocity-direction angle, not a tire slip-force state.
    """
    def __init__(self,p=KinematicParameters()):
        self.p=p;x=ca.SX.sym("x",5);u=ca.SX.sym("u",2)
        beta=ca.atan(p.lr/(p.lf+p.lr)*ca.tan(x[4]))
        f=ca.vertcat(x[3]*ca.cos(x[2]+beta),x[3]*ca.sin(x[2]+beta),
                     x[3]*ca.tan(x[4])*ca.cos(beta)/(p.lf+p.lr),u[0],u[1])
        self.symbolic=ca.Function("sun_bicycle",[x,u],[f])

    def rhs(self,x,u):return np.array(self.symbolic(x,u)).ravel()

    def step(self,x,u,dt):
        n=max(1,int(np.ceil(dt/.01)));h=dt/n
        for _ in range(n):x=rk4(self.rhs,x,u,h)
        return x


class KinematicMPC:
    """OCP 5 tracking core, Euler dynamics, terminal disk, optional neighbors.

    Receives a reference from PeriodicTrack instead of the thesis's APF planner.
    Uses disk separation directly, not the paper's consistency-set assumption.
    """
    def __init__(self,plant,horizon=20,dt=.1,neighbors=0,terminal_radius=1.):
        if horizon<2 or dt<=0 or terminal_radius<=0 or neighbors<0:raise ValueError("Invalid MPC settings")
        self.plant,self.N,self.dt,self.neighbors=plant,horizon,dt,neighbors
        op=self.op=ca.Opti();N=horizon;p=plant.p
        self.X=X=op.variable(5,N+1);self.U=U=op.variable(2,N)
        self.initial=op.parameter(5);self.ref=op.parameter(2,N+1)
        self.normal=op.parameter(2,N+1);self.width=op.parameter(2,N+1)
        self.other=[op.parameter(2,N+1) for _ in range(neighbors)];self.distance=op.parameter(neighbors) if neighbors else None
        op.subject_to(X[:,0]==self.initial)
        op.subject_to(op.bounded(0,X[3,:],20.));op.subject_to(op.bounded(-p.steer_limit,X[4,:],p.steer_limit))
        op.subject_to(op.bounded(-p.acceleration,U[0,:],p.acceleration));op.subject_to(op.bounded(-p.steer_rate,U[1,:],p.steer_rate))
        cost=0
        for j in range(N+1):
            e=X[:2,j]-self.ref[:,j];offset=ca.dot(e,self.normal[:,j])
            op.subject_to(op.bounded(-self.width[0,j],offset,self.width[1,j]))
            for other,d in zip(self.other,range(neighbors)):
                op.subject_to(ca.sumsqr(X[:2,j]-other[:,j])>=self.distance[d]**2)
            if j<N:
                op.subject_to(X[:,j+1]==X[:,j]+dt*plant.symbolic(X[:,j],U[:,j]))
                # Smoothed Euclidean norm and small regularizer are documented adaptations.
                cost+=ca.sqrt(ca.sumsqr(e)+1e-8)+.0001*ca.sumsqr(U[:,j])
        op.subject_to(ca.sumsqr(X[:2,N]-self.ref[:,N])<=terminal_radius**2)
        op.minimize(cost);op.solver("ipopt",*ipopt_options())

    def solve(self,z,reference,normal,width,heading=None,neighbor_paths=(),distances=()):
        if len(neighbor_paths)!=self.neighbors or len(distances)!=self.neighbors:
            raise ValueError("Neighbor data must match controller dimension")
        for param,val in [(self.initial,z),(self.ref,reference),(self.normal,normal),(self.width,width)]:self.op.set_value(param,val)
        for param,val in zip(self.other,neighbor_paths):self.op.set_value(param,val)
        if self.neighbors:self.op.set_value(self.distance,distances)
        guess=np.tile(np.asarray(z)[:,None],(1,self.N+1));guess[:2]=reference
        if heading is not None:guess[2]=heading
        self.op.set_initial(self.X,guess);self.op.set_initial(self.U,0)
        sol,res=solve_checked(self.op)
        return np.array(sol.value(self.U[:,0])).ravel(),{"prediction":np.array(sol.value(self.X)),"residual":res}
