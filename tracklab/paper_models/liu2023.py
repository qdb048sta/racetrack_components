"""Liu (2023) executable submodels: modal ring, rigid ring, relaxation and PCM.

This is a tire laboratory, not a vehicle handling model. In particular it does
not implement the full steady contact solve (3.96-3.102) or a lateral slip law.
External contact forces are inputs to RigidRing, as in Eqs. 3.110-3.114.
"""
from dataclasses import dataclass
from itertools import product
import numpy as np
from scipy.linalg import eigh
from scipy.integrate import solve_ivp
from numpy.polynomial.legendre import legvander


@dataclass(frozen=True)
class RingParameters:
    """Illustrative SI parameters, not the identified tire in the dissertation."""
    radius: float = .30
    width: float = .20
    pressure: float = 220000.
    membrane: float = 2e6       # EA [N]
    bending: float = 10.       # EI [N m^2]
    mass_per_length: float = 5. # rho A [kg/m]
    radial_foundation: float = 2e5 # ku [N/m^2]
    tangential_foundation: float = 1e5
    damping: float = 60.       # proportional per-coordinate damping [N s/m^2]

    def __post_init__(self):
        vals=vars(self).copy();vals.pop("damping")
        if min(vals.values())<=0 or self.damping<0:raise ValueError("Invalid ring parameters")


class ModalRing:
    """Nonrotating in-plane modal reduction, Eq. 2.20.

    n=0 gives breathing/tangential uniform coordinates; each n>0 matrix is one
    sine/cosine phase pair. The other spatial phase has the same eigenvalues.
    M qdd + C qd + K q = modal forcing. No out-of-plane/rotation terms here.
    """
    def __init__(self,p=RingParameters()):self.p=p

    def matrices(self,n):
        if not isinstance(n,(int,np.integer)) or n<0:raise ValueError("Mode number must be a nonnegative integer")
        p=self.p;r=p.radius;ea=p.membrane/r**2;ei=p.bending/r**4;pressure=p.pressure*p.width/r
        coupling=-ea*n-ei*n**3-pressure*n
        K=np.array([[ea+ei*n**4+pressure*n*n+p.radial_foundation,coupling],
                    [coupling,ea*n*n+ei*n*n+pressure*n*n+p.tangential_foundation]])
        return p.mass_per_length*np.eye(2),p.damping*np.eye(2),K

    def modes(self,n):
        M,_,K=self.matrices(n);values,vectors=eigh(K,M)
        if np.any(values<=0):raise ValueError("Nonpositive modal stiffness")
        return np.sqrt(values)/(2*np.pi),vectors

    def integrate(self,n,times,initial,forcing=lambda t:np.zeros(2)):
        M,C,K=self.matrices(n)
        def rhs(t,x):return np.r_[x[2:],np.linalg.solve(M,forcing(t)-C@x[2:]-K@x[:2])]
        result=solve_ivp(rhs,(times[0],times[-1]),initial,t_eval=times,method="Radau",rtol=1e-9,atol=1e-11)
        if not result.success:raise RuntimeError(result.message)
        return result.y.T


def relaxation_step(force,nominal_force,belt_speed,relaxation_length,dt):
    """Exact held-input integration of Eq. 3.105 (sigma_t=a under full adhesion)."""
    if relaxation_length<=0 or dt<0:raise ValueError("Need positive relaxation length and nonnegative step")
    return nominal_force+(force-nominal_force)*np.exp(-abs(belt_speed)*dt/relaxation_length)


@dataclass(frozen=True)
class RigidParameters:
    ring_mass: float = 8.
    rim_mass: float = 20.
    ring_inertia: float = .7
    rim_inertia: float = 1.2
    kx: float = 150000.
    kz: float = 200000.
    kt: float = 15000.
    cx: float = 500.
    cz: float = 700.
    ct: float = 50.
    radius: float = .30

    def __post_init__(self):
        if min(vars(self).values())<=0:raise ValueError("Rigid-ring parameters must be positive")


class RigidRing:
    """Eqs. 3.110-3.114, x=[xb,zb,theta_b,za,theta_a, their five velocities].

    z is downward. Fcz must be supplied as a signed force, normally negative.
    Rim longitudinal position/velocity xa,va and carrier speed Omega are inputs.
    The printed gyroscopic/damping signs are preserved; stability is not assumed
    for arbitrary Omega/parameter combinations. The demo uses Omega=0.
    """
    def __init__(self,p=RigidParameters()):self.p=p

    def rhs(self,t,x,forcing):
        p=self.p;xb,zb,tb,za,ta,vx,vz,wt,vza,wta=x
        f=forcing(t);xa=f.get("xa",0.);va=f.get("va",0.);omega=f.get("omega",0.)
        fx=f.get("fx",0.);fz=f.get("fz",0.);mz=f.get("moment",0.)
        axle_load=f.get("axle_load",0.);axle_torque=f.get("axle_torque",0.)
        dx=xb-xa;dz=zb-za;dt=tb-ta
        ax=(fx-p.cx*(vx-va)-p.kx*dx+p.cx*(omega+wta)*dz)/p.ring_mass
        az=(fz-p.cz*(vz-vza)-p.kz*dz+p.cz*(omega+wta)*dx)/p.ring_mass
        at=(mz+fx*p.radius-p.ct*(wt-wta)-p.kt*dt)/p.ring_inertia
        aza=(axle_load-p.cz*(vza-vz)+p.kz*dz-p.cz*(omega+wta)*dx)/p.rim_mass
        ata=(axle_torque-p.ct*(wta-wt)+p.kt*dt)/p.rim_inertia
        return np.array([vx,vz,wt,vza,wta,ax,az,at,aza,ata])

    def integrate(self,times,initial,forcing):
        out=solve_ivp(lambda t,x:self.rhs(t,x,forcing),(times[0],times[-1]),initial,
                      t_eval=times,method="Radau",rtol=1e-8,atol=1e-10,
                      first_step=min(.0001,times[-1]-times[0]),max_step=.001)
        if not out.success:raise RuntimeError(out.message)
        return out.y.T


class LegendrePCM:
    """Nonintrusive regression PCM, Eqs. 5.137-5.139, for independent U[-1,1].

    Uses least squares via SVD rather than explicitly inverting normal equations.
    This is one Askey-family case, not all distributions in the dissertation.
    """
    def __init__(self,dimension,degree):
        if dimension<1 or degree<0:raise ValueError("Invalid polynomial basis")
        self.dimension,self.degree=dimension,degree
        self.indices=[i for i in product(range(degree+1),repeat=dimension) if sum(i)<=degree]
        self.coefficients=None

    def design(self,points):
        points=np.asarray(points)
        if points.ndim!=2 or points.shape[1]!=self.dimension or np.any(np.abs(points)>1):
            raise ValueError("Expected points in the standardized [-1,1] parameter box")
        univariate=[legvander(points[:,d],self.degree) for d in range(self.dimension)]
        return np.column_stack([np.prod([univariate[d][:,i] for d,i in enumerate(index)],axis=0) for index in self.indices])

    def fit(self,points,values):
        A=self.design(points)
        if len(A)<len(self.indices):raise ValueError("Underdetermined collocation fit")
        coefficients,_,rank,_=np.linalg.lstsq(A,np.asarray(values),rcond=None)
        if rank<len(self.indices):raise ValueError("Rank-deficient collocation points")
        self.coefficients=coefficients
        return self

    def predict(self,points):
        if self.coefficients is None:raise ValueError("Fit the surrogate first")
        return self.design(points)@self.coefficients

    def moments(self):
        if self.coefficients is None:raise ValueError("Fit the surrogate first")
        norms=np.array([np.prod([1/(2*i+1) for i in index]) for index in self.indices])
        return self.coefficients[0],np.tensordot(norms[1:],self.coefficients[1:]**2,axes=(0,0))
