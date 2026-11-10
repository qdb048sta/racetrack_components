"""Source-family vehicle adapters on the common OCD demonstration vehicle.

These are normalized structural comparisons, not calibrated thesis reproductions.
Every added actuator, drivetrain, aerodynamic and load closure is exposed in
model_info. Source equations are cited in documentation/model_comparison.
Coordinates: x forward, y left; positive yaw counterclockwise; FL FR RL RR.
Public observation always [X,Y,psi,vx_CG,vy_CG,r,steering].
"""
from __future__ import annotations
from dataclasses import asdict
import math
import numpy as np
from .models import VehicleParameters, ReducedDoubleTrack, magic_formula, _finite_vector

P = VehicleParameters
NATIVE_BODY = [0, 1, 3, 6, 7, 9]


def steering_rate(delta, target, p):
    rate = float(np.clip((target-delta)/p.steering_time_constant_s,
                         -p.steering_rate_limit_radps, p.steering_rate_limit_radps))
    if delta >= p.steering_limit_rad: rate = min(0., rate)
    if delta <= -p.steering_limit_rad: rate = max(0., rate)
    return rate


def resistance(speed, p, downforce=True):
    down = -.5*p.air_density_kgpm3*p.lift_coefficient_area_m2*speed*speed if downforce else 0.
    return (p.rolling_resistance_coefficient*(p.mass_kg*p.gravity_mps2+down)
            +.5*p.air_density_kgpm3*p.drag_coefficient_area_m2*speed*abs(speed))


class Integrator:
    """Forward-operation RK4 adapter; all internal states persist between calls."""
    state_dimension = 7
    steering_index = 6
    wheel_indices = None

    def __init__(self, parameters=None, integration_step_s=.001):
        self.parameters = parameters or P()
        self.parameters.validate()
        if not np.isfinite(integration_step_s) or integration_step_s <= 0:
            raise ValueError('integration_step_s must be positive')
        self.integration_step_s = float(integration_step_s)
        self._state = np.zeros(self.state_dimension)
        self._torques = np.zeros(4)
        self._steering = 0.
        self._friction = np.ones(4)
        p=self.parameters
        self.static_loads=p.mass_kg*p.gravity_mps2*np.array([p.lr_m/p.wheelbase_m/2]*2+[p.lf_m/p.wheelbase_m/2]*2)

    @property
    def state(self): return self._state.copy()
    @property
    def full_state(self): return self.state
    @property
    def observation(self): return self.observe(self._state)
    @property
    def diagnostics(self):
        return self._evaluate(self._state,self._torques,self._steering,self._friction,True)[1]

    def observe(self,z): return np.asarray(z,dtype=float).copy()

    def reset(self,state,steering=None,friction=(1.,1.,1.,1.)):
        z=np.asarray(state,dtype=float)
        if z.shape==(6,): z=np.r_[z,0. if steering is None else steering]
        z=self._from_observation(z) if z.shape==(7,) and self.state_dimension!=7 else z.copy()
        z=_finite_vector(z,self.state_dimension,'state')
        if abs(z[self.steering_index]) > self.parameters.steering_limit_rad+1e-10:
            raise ValueError('Initial steering outside limits')
        if self.wheel_indices is not None and np.any(z[self.wheel_indices]<0):
            raise ValueError('Negative initial wheel speed')
        self._state=z.copy(); self._torques=np.zeros(4); self._steering=float(z[self.steering_index])
        self._friction=ReducedDoubleTrack._check_friction(friction)
        return self.observation

    def reset_from_native(self,x):
        x=_finite_vector(x,33,'native state')
        return self.reset(np.r_[x[NATIVE_BODY],x[32]])

    def evaluate(self,state,torques,steering,friction=(1.,1.,1.,1.)):
        z=_finite_vector(state,self.state_dimension,'state')
        t=_finite_vector(torques,4,'torques'); mu=ReducedDoubleTrack._check_friction(friction)
        return self._evaluate(z,t,float(steering),mu,True)

    def step(self,torques,steering,dt,substeps=None,friction=(1.,1.,1.,1.)):
        t=_finite_vector(torques,4,'torques'); mu=ReducedDoubleTrack._check_friction(friction)
        p=self.parameters
        if not np.isfinite(dt) or dt<=0 or not np.isfinite(steering) or abs(steering)>p.steering_limit_rad+1e-10:
            raise ValueError('Positive dt and bounded steering required')
        if substeps is None: substeps=max(1,math.ceil(dt/self.integration_step_s-1e-12))
        if isinstance(substeps,bool) or int(substeps)!=substeps or substeps<1: raise ValueError('Invalid substeps')
        h=dt/int(substeps);z=self._state.copy()
        for _ in range(int(substeps)):
            k1=self._evaluate(z,t,steering,mu)
            k2=self._evaluate(z+h*k1/2,t,steering,mu)
            k3=self._evaluate(z+h*k2/2,t,steering,mu)
            k4=self._evaluate(z+h*k3,t,steering,mu)
            z+=h*(k1+2*k2+2*k3+k4)/6
            if self.wheel_indices is not None: z[self.wheel_indices]=np.maximum(z[self.wheel_indices],0.)
            z[self.steering_index]=np.clip(z[self.steering_index],-p.steering_limit_rad,p.steering_limit_rad)
        if not np.isfinite(z).all(): raise FloatingPointError('Nonfinite integrated state')
        self._state=z;self._torques=t.copy();self._steering=float(steering);self._friction=mu.copy()
        return self.observation

    def _diagnostic(self,ax,ay,alpha,slip,fx,fy,fz,**extra):
        return dict(acceleration_mps2=np.array([ax,ay,0.]),tire_slip_angle_rad=np.asarray(alpha),
            tire_longitudinal_slip=np.asarray(slip),tire_force_longitudinal_N=np.asarray(fx),
            tire_force_lateral_N=np.asarray(fy),tire_force_vertical_N=np.asarray(fz),**extra)

    def close(self): pass

    @property
    def model_info(self):
        return dict(model=self.name,state_dimension=self.state_dimension,parameters=asdict(self.parameters),
                    source=self.source,adaptations=self.adaptations,capabilities=self.capabilities,
                    operating_domain='Forward motion; numerical denominator floor 1 m/s; RK4 post-step steering projection, plus nonnegative wheel projection where wheel states exist')


class LinigerBicycle(Integrator):
    name='liniger_bicycle'
    source='Liniger 2018, section 3.1, equations 3.1-3.2, printed pp.17-20'
    adaptations='Common 800 kg vehicle and static-load axle peaks; motor PWM map replaced by sum(T)/R at rear axle with rolling and drag; added bounded PT1 steering; no wheel spin, longitudinal tire saturation, load transfer or downforce'
    capabilities=dict(tire_forces=True,wheel_spin=False,longitudinal_slip=False,independent_wheels=False,force_circle_metric=False)

    def _evaluate(self,z,torques,steering,mu,diagnostics=False):
        p=self.parameters;u,v,r,d=z[3:7];uf=max(u,1.)
        af=d-math.atan2(v+p.lf_m*r,uf); ar=math.atan2(p.lr_m*r-v,uf)
        fz=self.static_loads
        fyf=p.nominal_friction*float(mu[:2].mean())*sum(fz[:2])*math.sin(p.lateral_C*math.atan(p.lateral_B*af))
        fyr=p.nominal_friction*float(mu[2:].mean())*sum(fz[2:])*math.sin(p.lateral_C*math.atan(p.lateral_B*ar))
        fxr=float(sum(torques))/p.wheel_radius_m-resistance(u,p,False)
        ax=(fxr-fyf*math.sin(d))/p.mass_kg; ay=(fyr+fyf*math.cos(d))/p.mass_kg
        dr=(p.lf_m*fyf*math.cos(d)-p.lr_m*fyr)/p.yaw_inertia_kgm2
        c,s=math.cos(z[2]),math.sin(z[2])
        rhs=np.array([u*c-v*s,u*s+v*c,r,ax+r*v,ay-r*u,dr,steering_rate(d,steering,p)])
        if not diagnostics:return rhs
        return rhs,self._diagnostic(ax,ay,[af,af,ar,ar],np.zeros(4),[0,0,fxr/2,fxr/2],
            [fyf/2,fyf/2,fyr/2,fyr/2],fz,**self.capabilities)


class LienkampLinear(LinigerBicycle):
    name='lienkamp_linear'
    source='Lienkamp 2019 supplied notes, slide 24, equations 2-8 and 2-9'
    adaptations='Constant-speed linear lateral/yaw model scheduled using max(vx,1); added global pose, longitudinal force, drag/downforce-related rolling resistance, bounded PT1 steering. Static axle stiffness C=16*Fz; no lateral saturation'
    capabilities=dict(tire_forces=True,wheel_spin=False,longitudinal_slip=False,independent_wheels=False,force_circle_metric=False)

    def _evaluate(self,z,torques,steering,mu,diagnostics=False):
        p=self.parameters;u,v,r,d=z[3:7];uf=max(u,1.)
        af=d-(v+p.lf_m*r)/uf; ar=(p.lr_m*r-v)/uf
        fz=self.static_loads
        fyf=16.*float(mu[:2].mean())*sum(fz[:2])*af
        fyr=16.*float(mu[2:].mean())*sum(fz[2:])*ar
        fx=float(sum(torques))/p.wheel_radius_m-resistance(u,p,True)
        ax=fx/p.mass_kg;ay=(fyf+fyr)/p.mass_kg
        dr=(p.lf_m*fyf-p.lr_m*fyr)/p.yaw_inertia_kgm2
        c,s=math.cos(z[2]),math.sin(z[2])
        rhs=np.array([u*c-v*s,u*s+v*c,r,ax+r*v,ay-r*u,dr,steering_rate(d,steering,p)])
        if not diagnostics:return rhs
        return rhs,self._diagnostic(ax,ay,[af,af,ar,ar],np.zeros(4),fx*fz/sum(fz),
            [fyf/2,fyf/2,fyr/2,fyr/2],fz,**self.capabilities)


class SunKinematic(Integrator):
    name='sun_kinematic';state_dimension=5;steering_index=4
    source='Sun 2025, equation 5.1, printed p.106 (chapter 3 is a separate longitudinal eco-driving model)'
    adaptations='CG kinematic bicycle with common gross-acceleration force adapter, drag/rolling resistance and bounded PT1 steering; source steering-rate input replaced by actuator target. No tire contact or friction dynamics'
    capabilities=dict(tire_forces=False,wheel_spin=False,longitudinal_slip=False,independent_wheels=False,force_circle_metric=False)

    def _from_observation(self,z): return np.r_[z[:3],math.hypot(z[3],z[4]),z[6]]
    def observe(self,z):
        p=self.parameters;b=math.atan(p.lr_m/p.wheelbase_m*math.tan(z[4]));v=z[3]
        return np.r_[z[:3],v*math.cos(b),v*math.sin(b),v*math.cos(b)*math.tan(z[4])/p.wheelbase_m,z[4]]
    def _evaluate(self,z,torques,steering,mu,diagnostics=False):
        p=self.parameters;v=z[3];d=z[4];b=math.atan(p.lr_m/p.wheelbase_m*math.tan(d))
        a=(float(sum(torques))/p.wheel_radius_m-resistance(v,p,True))/p.mass_kg
        rate=steering_rate(d,steering,p);r=v*math.cos(b)*math.tan(d)/p.wheelbase_m
        rhs=np.array([v*math.cos(z[2]+b),v*math.sin(z[2]+b),r,a,rate])
        if not diagnostics:return rhs
        db=p.lr_m/p.wheelbase_m/(math.cos(d)**2*(1+(p.lr_m/p.wheelbase_m*math.tan(d))**2))*rate
        ax=a*math.cos(b)-v*math.sin(b)*(db+r);ay=a*math.sin(b)+v*math.cos(b)*(db+r)
        return rhs,self._diagnostic(ax,ay,np.zeros(4),np.zeros(4),np.zeros(4),np.zeros(4),np.zeros(4),**self.capabilities)


def coupled_brush(alpha_positive,kappa,loads,friction,p,normalize_peak=True):
    """Beal Eq 2.7, source mu_s/mu=11/12, exact locked-wheel limit.

    Small-slip stiffness 40*mu_nom*Fz and 16*mu_nom*Fz matches native.
    The source radial peak is 45/49 of internal mu*Fz, so internal mu is
    scaled by 49/45 to compare equal *actual* peak resultant forces.
    """
    loads=np.asarray(loads,dtype=float);friction=np.asarray(friction,dtype=float)
    peak=p.nominal_friction*friction*loads
    ratio=11/12; internal=peak*(49/45 if normalize_peak else 1.)
    nx=40*peak*kappa;ny=16*peak*np.tan(alpha_positive)
    norm=np.hypot(nx,ny);den=np.maximum(1.+kappa,0.)
    gamma=np.divide(norm,den,out=np.full_like(norm,np.inf),where=den>1e-12)
    q=np.minimum(np.divide(gamma,3*internal,out=np.ones_like(norm),where=internal>0),1.)
    f=internal*(3*q-3*(2-ratio)*q*q+(3-2*ratio)*q*q*q)
    fx=np.divide(nx*f,norm,out=np.zeros_like(norm),where=norm>0)
    fy=np.divide(ny*f,norm,out=np.zeros_like(norm),where=norm>0)
    return fx,fy


class GaoFourWheel(Integrator):
    name='gao_fourwheel';state_dimension=11;steering_index=10;wheel_indices=slice(6,10)
    source='Gao 2014, equations 2.5-2.11 and 2.21-2.22'
    adaptations='Common geometry/parameters; source static gravity loads; zero MF shifts and common coefficients. Only published pure-slip MF retained independently: source combined-slip modification is unspecified. Wheel damping bw=0 avoids printed typo; common rolling wheel torque, low-speed inertia and bounded PT1 steering added; CG body drag added; no downforce'
    capabilities=dict(tire_forces=True,wheel_spin=True,longitudinal_slip=True,independent_wheels=True,force_circle_metric=True)
    load_transfer=False

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);p=self.parameters
        self.xw=np.array([p.lf_m,p.lf_m,-p.lr_m,-p.lr_m]);self.yw=np.array([p.front_track_m/2,-p.front_track_m/2,p.rear_track_m/2,-p.rear_track_m/2])
        self.inertia=np.array([p.wheel_inertia_front_kgm2]*2+[p.wheel_inertia_rear_kgm2]*2)
        self._loads=ReducedDoubleTrack(parameters=p)
    def _from_observation(self,z):
        vl,vc=self._wheel_velocity(z[3],z[4],z[5],z[6])
        return np.r_[z[:6],np.maximum(vl,0.)/self.parameters.wheel_radius_m,z[6]]
    def observe(self,z): return np.r_[z[:6],z[10]]
    def reset_from_native(self,x):
        x=_finite_vector(x,33,'native state')
        return self.reset(np.r_[x[NATIVE_BODY],x[28:32],x[32]])
    def _wheel_velocity(self,u,v,r,delta):
        vx=u-r*self.yw;vy=v+r*self.xw;c,s=math.cos(delta),math.sin(delta)
        vl=vx.copy();vc=vy.copy();vl[:2]=c*vx[:2]+s*vy[:2];vc[:2]=-s*vx[:2]+c*vy[:2]
        return vl,vc
    def _slips(self,z,vl,vc):
        rw=self.parameters.wheel_radius_m*np.maximum(z[6:10],0.)
        return -np.arctan2(vc,np.maximum(1.,vl)),(rw-vl)/np.maximum(1.,np.maximum(vl,rw))
    def tire_forces(self,alpha,kappa,fz,mu):
        p=self.parameters;peak=p.nominal_friction*mu*fz
        return peak*magic_formula(kappa,20.,2.,.7),peak*magic_formula(alpha,10.,1.6,-1.9)
    def _forces(self,z,mu):
        p=self.parameters;delta=z[10];c,s=math.cos(delta),math.sin(delta)
        vl,vc=self._wheel_velocity(z[3],z[4],z[5],delta);alpha,kappa=self._slips(z,vl,vc)
        obs=self.observe(z);ucg,vcg=obs[3:5]
        if not self.load_transfer:
            fz=self.static_loads;fx,fy=self.tire_forces(alpha,kappa,fz,mu)
            bx=fx.copy();by=fy.copy();bx[:2]=c*fx[:2]-s*fy[:2];by[:2]=s*fx[:2]+c*fy[:2]
            outside=False;residual=0.
        else:
            ux,uy=self.tire_forces(alpha,kappa,np.ones(4),mu)
            ubx=ux.copy();uby=uy.copy();ubx[:2]=c*ux[:2]-s*uy[:2];uby[:2]=s*ux[:2]+c*uy[:2]
            fz0,_=self._loads.normal_loads(0.,0.,ucg)
            lx,ly=self._loads._load_dx,self._loads._load_dy
            aa=1-float(ubx@lx)/p.mass_kg;ab=-float(ubx@ly)/p.mass_kg
            ba=-float(uby@lx)/p.mass_kg;bb=1-float(uby@ly)/p.mass_kg
            ra=float(ubx@fz0)/p.mass_kg;rb=float(uby@fz0)/p.mass_kg;det=aa*bb-ab*ba
            if abs(det)<1e-10:raise FloatingPointError('Singular normal-load closure')
            agx,agy=(bb*ra-ab*rb)/det,(aa*rb-ba*ra)/det
            fz,outside=self._loads.normal_loads(agx,agy,ucg)
            if outside:
                for _ in range(40):
                    ax,ay=float(ubx@fz)/p.mass_kg,float(uby@fz)/p.mass_kg
                    nxt,flag=self._loads.normal_loads(ax,ay,ucg)
                    if np.max(np.abs(nxt-fz))<1e-6: fz=nxt;break
                    fz=nxt
                else:raise FloatingPointError('Contact closure failed')
            fx,fy=ux*fz,uy*fz;bx,by=ubx*fz,uby*fz
            ax,ay=float(sum(bx))/p.mass_kg,float(sum(by))/p.mass_kg
            exact,_=self._loads.normal_loads(ax,ay,ucg);residual=float(np.max(np.abs(exact-fz)))
        drag=np.array([-.5*p.air_density_kgpm3*p.drag_coefficient_area_m2*ucg*abs(ucg),
                       -.5*p.air_density_kgpm3*p.drag_coefficient_area_m2*vcg*abs(vcg),
                       .5*p.air_density_kgpm3*p.lift_coefficient_area_m2*ucg*ucg if self.load_transfer else 0.])
        return fz,fx,fy,bx,by,alpha,kappa,drag,outside,residual
    def _wheel_rhs(self,z,torques,fx,fz):
        p=self.parameters;extra=max(0.,15*p.wheel_inertia_rear_kgm2*(1-float(np.mean(z[6:10]))/20))
        rhs=(torques-p.wheel_radius_m*(fx+p.rolling_resistance_coefficient*fz))/(self.inertia+extra)
        return np.where(z[6:10]<1e-5,np.maximum(rhs,0.),rhs)
    def _evaluate(self,z,torques,steering,mu,diagnostics=False):
        p=self.parameters;fz,fx,fy,bx,by,alpha,kappa,drag,outside,residual=self._forces(z,mu)
        ax=(float(sum(bx))+drag[0])/p.mass_kg;ay=(float(sum(by))+drag[1])/p.mass_kg
        mz=float(self.xw@by-self.yw@bx);u,v,r=z[3:6];c,s=math.cos(z[2]),math.sin(z[2])
        rhs=np.empty(11);rhs[:6]=[u*c-v*s,u*s+v*c,r,ax+r*v,ay-r*u,mz/p.yaw_inertia_kgm2]
        rhs[6:10]=self._wheel_rhs(z,torques,fx,fz);rhs[10]=steering_rate(z[10],steering,p)
        if not diagnostics:return rhs
        return rhs,self._diagnostic(ax,ay,alpha,kappa,fx,fy,fz,contact_load_limited=outside,
            load_balance_residual_N=residual,**self.capabilities)


class BealCoupledBrush(GaoFourWheel):
    name='beal_brush';state_dimension=13;load_transfer=True
    source='Beal 2011, equations 2.6-2.10 and Table 4.2'
    adaptations='Source yaw-roll and coupled-brush structure, common vehicle coefficients; actual radial peak normalized via internal mu=49/45 for mu_s/mu=11/12. OCD-derived roll inertia/stiffness/damping and quasi-static load closure independent of roll states, CG aero and common wheel drivetrain/PT1 added. Not calibrated P1 suspension reproduction'

    def __init__(self,*args,normalize_peak_friction=True,**kwargs):
        super().__init__(*args,**kwargs);self.normalize_peak_friction=bool(normalize_peak_friction)
        self.roll_lever=.1;self.Ix=100.;self.Iy=500.;self.Kroll=275835.;self.Broll=2676.5
    def _from_observation(self,z): return np.r_[super()._from_observation(z),0.,0.]
    def observe(self,z):
        h=self.roll_lever;phi,p=z[11:13]
        return np.r_[z[:3],z[3]+h*z[5]*math.sin(phi),z[4]-h*p*math.cos(phi),z[5],z[10]]
    def reset_from_native(self,x):
        x=_finite_vector(x,33,'native state');h=self.roll_lever;phi=x[4];p=x[10]
        body=x[NATIVE_BODY].copy();body[3]-=h*body[5]*math.sin(phi);body[4]+=h*p*math.cos(phi)
        return self.reset(np.r_[body,x[28:32],x[32],phi,p])
    def _slips(self,z,vl,vc):
        rw=self.parameters.wheel_radius_m*np.maximum(z[6:10],0.)
        return -np.arctan2(vc,np.maximum(1.,vl)),np.maximum(-1.,(rw-vl)/np.maximum(1.,vl))
    def tire_forces(self,alpha,kappa,fz,mu):
        return coupled_brush(alpha,kappa,fz,mu,self.parameters,self.normalize_peak_friction)
    @property
    def model_info(self):
        info=super().model_info
        info.update(roll_parameters=dict(h_prime_m=self.roll_lever,Ix=self.Ix,Iy=self.Iy,Kroll=self.Kroll,Broll=self.Broll),
            brush_peak_normalization=dict(enabled=self.normalize_peak_friction,sliding_static_ratio=11/12,
                source_actual_peak_ratio=45/49,internal_coefficient_multiplier=49/45 if self.normalize_peak_friction else 1.,
                actual_sliding_to_nominal_peak=539/540 if self.normalize_peak_friction else 11/12),
            reference_point='State U,V is roll-base; public velocities and global position are CG',
            load_closure='Quasi-static force/load equilibrium at CG height0.3m; roll does not feed normal loads')
        return info
    def _evaluate(self,z,torques,steering,mu,diagnostics=False):
        pp=self.parameters;fz,fx,fy,bx,by,alpha,kappa,drag,outside,residual=self._forces(z,mu)
        m=pp.mass_kg;Iz=pp.yaw_inertia_kgm2;h=self.roll_lever;U,V,r=z[3:6];phi,p=z[11:13]
        sn,cs=math.sin(phi),math.cos(phi)
        qx=float(sum(bx))+drag[0];qy=float(sum(by))+drag[1]
        qpsi=float(self.xw@by-self.yw@bx)+h*sn*drag[0]
        qphi_a=-h*cs*drag[1]-h*sn*drag[2]
        rdot=(qpsi-qx*h*sn-2*(self.Iy-Iz)*r*p*sn*cs)/(self.Iy*sn*sn+Iz*cs*cs)
        pdot=(qy*h*cs-self.Kroll*phi-self.Broll*p+m*pp.gravity_mps2*h*sn+
              ((self.Iy-Iz)*r*r-m*h*h*p*p)*sn*cs+qphi_a)/(self.Ix+m*h*h*sn*sn)
        udot=qx/m+r*V-h*rdot*sn-2*h*r*p*cs
        vdot=qy/m-r*U+h*pdot*cs-h*(r*r+p*p)*sn
        obs=self.observe(z);u,v=obs[3:5];c,s=math.cos(z[2]),math.sin(z[2])
        rhs=np.empty(13);rhs[:6]=[u*c-v*s,u*s+v*c,r,udot,vdot,rdot]
        rhs[6:10]=self._wheel_rhs(z,torques,fx,fz);rhs[10]=steering_rate(z[10],steering,pp)
        rhs[11:13]=[p,pdot]
        if not diagnostics:return rhs
        return rhs,self._diagnostic(qx/m,qy/m,alpha,kappa,fx,fy,fz,roll_rad=phi,roll_rate_radps=p,
            contact_load_limited=outside,load_balance_residual_N=residual,**self.capabilities)


MODEL_CLASSES={cls.name:cls for cls in (LinigerBicycle,GaoFourWheel,BealCoupledBrush,SunKinematic,LienkampLinear)}
MODEL_NAMES=('native_ocd',*MODEL_CLASSES)
LABELS=dict(native_ocd='Native OCD double-track / MF_SIMPLE',liniger_bicycle='Liniger nonlinear bicycle (normalized)',
    gao_fourwheel='Gao four-wheel / published pure-slip MF (normalized)',beal_brush='Beal yaw-roll / coupled brush (normalized)',
    sun_kinematic='Sun CG kinematic bicycle (normalized)',lienkamp_linear='Lienkamp scheduled linear bicycle (normalized)')


def make_plant(name,cfg):
    if name=='native_ocd':
        from tracklab.comparison_native.vehicle import NativeDoubleTrack,normalized_parameters
        return NativeDoubleTrack(normalized_parameters(),integration_step_s=cfg.native_step_s)
    return MODEL_CLASSES[name](integration_step_s=cfg.reduced_step_s)
