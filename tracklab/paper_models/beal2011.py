"""Beal (2011), Ch. 2 brush/AFI models and Ch. 4 envelope MPC.

Paper equations use alpha = wheel-velocity angle minus steer, Fy ~ -C*alpha.
The paper's two lateral states are embedded in world pose for PeriodicTrack.
No yaw-roll chassis, wheel spin, or online friction estimator is implemented.
"""
from dataclasses import dataclass
import numpy as np
from scipy.optimize import brentq
from .common import rk4, ipopt_options, solve_checked


@dataclass(frozen=True)
class Parameters:
    mass: float = 1500.
    iz: float = 2500.
    a: float = 1.2
    b: float = 1.5
    cf: float = 80000.
    cr: float = 90000.
    mu: float = .7
    sliding_ratio: float = .9
    speed: float = 12.
    gravity: float = 9.81

    def __post_init__(self):
        if min(self.mass,self.iz,self.a,self.b,self.cf,self.cr,self.mu,self.speed) <= 0:
            raise ValueError("Physical parameters and forward speed must be positive")
        if not 0 < self.sliding_ratio <= 1:
            raise ValueError("sliding_ratio must be in (0,1]")

    @property
    def loads(self):
        return self.mass*self.gravity*np.array([self.b,self.a])/(self.a+self.b)


def brush(alpha, stiffness, capacity, sliding_ratio=1.):
    """Eq. 2.1 and signed sliding limit; threshold is |tan(alpha)|=3D/C.

    Eq. 2.2 prints alpha_slide=3D/C (small-angle form). Applying that to
    tan(alpha) preserves continuity of Eq. 2.1; see the documented correction.
    """
    if stiffness <= 0 or capacity < 0 or not 0 < sliding_ratio <= 1:
        raise ValueError("Invalid tire parameters")
    alpha = np.asarray(alpha)
    if np.any(np.abs(alpha) >= np.pi/2):
        raise ValueError("Brush model requires |alpha| < pi/2")
    if capacity == 0:
        return np.zeros_like(alpha, dtype=float)
    t = np.tan(alpha); c = stiffness; d = capacity; r = sliding_ratio
    adhesion = -c*t + c*c/(3*d)*(2-r)*t*np.abs(t) - c**3/(9*d*d)*(1-2*r/3)*t**3
    return np.where(np.abs(t) < 3*d/c, adhesion, -r*d*np.sign(t))


def derated_capacity(mu, load, longitudinal_force):
    """Eq. 2.8; requests outside the friction circle are rejected."""
    if mu <= 0 or load <= 0 or abs(longitudinal_force) > mu*load:
        raise ValueError("Infeasible longitudinal tire force")
    return np.sqrt(max(0., (mu*load)**2-longitudinal_force**2))


def peak(stiffness, capacity, ratio):
    angle = np.arctan(capacity/(stiffness*(1-2*ratio/3)))
    return angle, float(abs(brush(angle, stiffness, capacity, ratio)))


def inverse_brush(force, stiffness, capacity, ratio):
    angle, limit = peak(stiffness, capacity, ratio)
    if abs(force) > limit+1e-5:
        raise ValueError("Requested force exceeds the peak")
    force = np.clip(force, -limit, limit)
    if abs(force) < 1e-12:
        return 0.
    if abs(abs(force)-limit) < 1e-7:
        return -np.sign(force)*angle
    return brentq(lambda a: float(brush(a, stiffness, capacity, ratio))-force, -angle, angle)


class Bicycle:
    """World state [X,Y,psi,beta,r], steering input; fixed longitudinal speed."""
    def __init__(self, p=Parameters()):
        self.p = p

    def rhs(self, z, steer):
        p = self.p; beta, r = z[3:5]; fz, rz = p.loads
        af, ar = beta+p.a*r/p.speed-steer, beta-p.b*r/p.speed
        fyf = brush(af,p.cf,p.mu*fz,p.sliding_ratio)
        fyr = brush(ar,p.cr,p.mu*rz,p.sliding_ratio)
        return np.array([p.speed*(np.cos(z[2])-beta*np.sin(z[2])),
                         p.speed*(np.sin(z[2])+beta*np.cos(z[2])), r,
                         (fyf+fyr)/(p.mass*p.speed)-r, (p.a*fyf-p.b*fyr)/p.iz])

    def step(self, z, steer, dt):
        return rk4(self.rhs, np.asarray(z), steer, dt)

    def afi(self, lateral_state):
        """Eq. 2.21, rear force Taylor expansion at the measured alpha_r."""
        p = self.p; ar = lateral_state[0]-p.b*lateral_state[1]/p.speed
        d = p.mu*p.loads[1]; rmu = p.sliding_ratio; t = np.tan(ar)
        if abs(t) >= 3*d/p.cr:
            local_c = 0.
        else:
            slope = (-p.cr + 2*p.cr**2/(3*d)*(2-rmu)*abs(t)
                     - p.cr**3/(3*d*d)*(1-2*rmu/3)*t*t)*(1+t*t)
            local_c = -slope
        offset = float(brush(ar,p.cr,d,rmu))+local_c*ar
        A = np.array([[-local_c/(p.mass*p.speed),p.b*local_c/(p.mass*p.speed**2)-1],
                      [p.b*local_c/p.iz,-p.b*p.b*local_c/(p.speed*p.iz)]])
        B = np.array([1/(p.mass*p.speed),p.a/p.iz])
        c = np.array([offset/(p.mass*p.speed),-p.b*offset/p.iz])
        return A, B, c

    def linear_driver_model(self):
        p = self.p
        A = np.array([[-(p.cf+p.cr)/(p.mass*p.speed),-(p.a*p.cf-p.b*p.cr)/(p.mass*p.speed**2)-1],
                      [-(p.a*p.cf-p.b*p.cr)/p.iz,-(p.a*p.a*p.cf+p.b*p.b*p.cr)/(p.speed*p.iz)]])
        return A, np.array([p.cf/(p.mass*p.speed),p.a*p.cf/p.iz])


def tustin(A, B, c, dt):
    left = np.eye(len(A))-.5*dt*A
    return (np.linalg.solve(left,np.eye(len(A))+.5*dt*A),
            np.linalg.solve(left,dt*B),np.linalg.solve(left,dt*c))


class EnvelopeMPC:
    """Eq. 4.2 with Tustin dynamics, one-step delay, force/rate limits and slack.

    CasADi/IPOPT solves this convex QP; not the thesis's generated embedded solver.
    Front forces are scaled by their bound solely for numerical conditioning.
    """
    def __init__(self, plant, horizon=15, dt=.01):
        if horizon < 3 or dt <= 0:
            raise ValueError("Need horizon >= 3 and positive dt")
        self.plant,self.N,self.dt = plant,horizon,dt
        import casadi as ca
        self.op=op=ca.Opti(); N=horizon
        self.X=X=op.variable(2,N+1); self.V=V=op.variable(1,N-1); self.S=S=op.variable(2,N)
        self.x0=op.parameter(2); self.prev=op.parameter(); self.ref=op.parameter(2,N)
        self.A=op.parameter(2,2); self.B=op.parameter(2); self.c=op.parameter(2)
        self.limit=op.parameter(); self.bounds=op.parameter(2)
        p=plant.p; C=ca.DM([[0,1],[1,-p.b/p.speed]])
        op.subject_to(X[:,0]==self.x0)
        op.subject_to(X[:,1]==self.A@X[:,0]+self.B*self.prev+self.c)
        op.subject_to(op.bounded(-1,V,1)); op.subject_to(ca.vec(S)>=0)
        cost=0
        for j in range(N):
            err=X[:,j+1]-self.ref[:,j]
            cost+=30*err[0]**2+err[1]**2+100*ca.sum1(S[:,j])
            op.subject_to(C@X[:,j+1] <= self.bounds+S[:,j])
            op.subject_to(-C@X[:,j+1] <= self.bounds+S[:,j])
            if j<N-1:
                force=self.limit*V[0,j]
                op.subject_to(X[:,j+2]==self.A@X[:,j+1]+self.B*force+self.c)
                previous=self.prev if j==0 else self.limit*V[0,j-1]
                op.subject_to(op.bounded(-.3,(force-previous)/self.limit,.3))
                cost+=.0001*V[0,j]**2
        op.minimize(cost);op.solver("ipopt",*ipopt_options())

    def solve(self, z, driver_steer, previous_force=0.):
        p=self.plant.p; op=self.op; x=np.asarray(z)[3:5]
        A,B,c=tustin(*self.plant.afi(x),self.dt)
        al,bl=self.plant.linear_driver_model(); ad,bd,_=tustin(al,bl,np.zeros(2),self.dt)
        ref=[]; q=x.copy()
        for _ in range(self.N):
            q=ad@q+bd*driver_steer;ref.append(q.copy())
        _,limit=peak(p.cf,p.mu*p.loads[0],p.sliding_ratio)
        rear_angle,_=peak(p.cr,p.mu*p.loads[1],p.sliding_ratio)
        for param,value in [(self.x0,x),(self.prev,previous_force),(self.ref,np.array(ref).T),
                            (self.A,A),(self.B,B),(self.c,c),(self.limit,limit),
                            (self.bounds,[p.mu*p.gravity/p.speed,rear_angle])]:op.set_value(param,value)
        op.set_initial(self.X,np.column_stack([x,*ref]))
        op.set_initial(self.V,previous_force/limit);op.set_initial(self.S,0)
        sol,residual=solve_checked(op)
        force=float(sol.value(self.V)[0])*limit
        # The command is applied after the one-step computational delay.
        next_x=np.array(sol.value(self.X[:,1])).ravel()
        alpha=inverse_brush(force,p.cf,p.mu*p.loads[0],p.sliding_ratio)
        steer=next_x[0]+p.a*next_x[1]/p.speed-alpha
        return steer,{"force_n":force,"slack_max":float(np.max(sol.value(self.S))),
                      "constraint_residual":residual,"prediction":np.array(sol.value(self.X))}
