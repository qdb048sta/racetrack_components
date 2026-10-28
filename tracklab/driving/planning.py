"""Conservative curvature speed schedule, NOT a minimum-lap-time optimizer."""
from __future__ import annotations
import numpy as np
from scipy.ndimage import maximum_filter1d
from .config import DrivingConfig

class CurvatureSpeedPlan:
    def __init__(self,road,config:DrivingConfig):
        self.road=road;self.config=config
        n=int(np.ceil(road.length/config.speed_grid_spacing_m))
        self.s=np.linspace(0,road.length,n,endpoint=False);self.ds=road.length/n
        k=np.abs(road.sample_geometry(self.s)['curvature_1pm'])
        # Conservative +/- 1 m curvature envelope; no modification of road geometry.
        k=maximum_filter1d(k,size=2*int(np.ceil(1./self.ds))+1,mode='wrap')
        v2=np.minimum(config.maximum_speed_mps**2,
                      config.lateral_acceleration_budget_mps2/np.maximum(k,1e-8))
        self.curvature_envelope=k
        self.raw_limit=np.sqrt(v2)
        for _ in range(100):
            old=v2.copy()
            for i in range(n):
                j=(i+1)%n
                v2[j]=min(v2[j],v2[i]+2*config.planning_acceleration_mps2*self.ds)
            for i in range(n-1,-1,-1):
                j=(i+1)%n
                v2[i]=min(v2[i],v2[j]+2*config.planning_braking_mps2*self.ds)
            if np.max(np.abs(v2-old))<1e-10:break
        self.speed=np.sqrt(v2)

    def __call__(self,s):
        return np.interp(np.mod(s,self.road.length),np.r_[self.s,self.road.length],
                         np.r_[self.speed,self.speed[0]])

    def references(self,s0,speed0,steps,dt):
        ss=[float(s0)];vv=[min(max(float(speed0),1.0),self.config.maximum_speed_mps)]
        for _ in range(steps):
            target=float(self(ss[-1]+vv[-1]*dt))
            # Reach the planned envelope with bounded reference acceleration.
            vnext=min(target,vv[-1]+self.config.planning_acceleration_mps2*dt)
            vnext=max(vnext,vv[-1]-self.config.planning_braking_mps2*dt)
            ss.append(ss[-1]+.5*(vv[-1]+vnext)*dt);vv.append(vnext)
        return np.asarray(ss),np.asarray(vv)
