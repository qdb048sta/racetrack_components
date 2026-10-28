"""New dynamics/controller-interface checks; existing geometry tests are retained."""
from __future__ import annotations
import unittest
from pathlib import Path
import numpy as np
from scipy.integrate import solve_ivp
from tracklab import PeriodicTrack
from tracklab.driving.config import VehicleConfig,DrivingConfig
from tracklab.driving.vehicle import DynamicBicycle,SaturatingAxleTire
from tracklab.driving.planning import CurvatureSpeedPlan
from tracklab.driving.validation import project_near_batch,footprint_points

ROOT=Path(__file__).resolve().parents[1]

class DrivingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params=VehicleConfig();cls.plant=DynamicBicycle(cls.params)
        cls.road=PeriodicTrack(ROOT/'data'/'Norisring.csv')

    def test_positive_speed_initialization(self):
        with self.assertRaises(ValueError):DrivingConfig(initial_speed_mps=0).validate()

    def test_invalid_geometry(self):
        with self.assertRaises(ValueError):VehicleConfig(lf_m=4.).validate()

    def test_zero_lateral_motion_straight(self):
        z=np.array([0,0,.2,10,0,0,0.]);d=self.plant.rhs(z,[0,0])
        self.assertEqual(d[4],0);self.assertEqual(d[5],0)
        np.testing.assert_allclose(d[:2],[10*np.cos(.2),10*np.sin(.2)])

    def test_positive_steer_gives_positive_yaw_acceleration(self):
        z=np.array([0,0,0,10,0,0,.05]);d=self.plant.rhs(z,[.05,0])
        self.assertGreater(d[5],0)

    def test_lateral_slip_restoring_force(self):
        z=np.array([0,0,0,10,.5,0,0]);d=self.plant.rhs(z,[0,0])
        self.assertLess(d[4],0)

    def test_tire_force_circle(self):
        tire=SaturatingAxleTire();load=4000.;mu=.8
        for slip in np.linspace(-.5,.5,41):
            for fx in (-.8*mu*load,0,.8*mu*load):
                fy=tire.lateral(slip,load,fx,mu)
                self.assertLessEqual(np.hypot(fx,fy),mu*load+1e-8)

    def test_steering_actuator_limit(self):
        rhs=self.plant.rhs([0,0,0,8,0,0,0],[.5,0])
        self.assertLessEqual(rhs[6],self.params.steering_rate_limit_radps)
        self.assertGreater(rhs[6],.99*self.params.steering_rate_limit_radps)

    def test_symbolic_numeric_agreement(self):
        try: import casadi
        except ImportError: self.skipTest('CasADi not installed; install requirements-driving.txt')
        f=self.plant.symbolic_rhs();z=np.array([1,2,.3,6,.4,.18,.12]);u=np.array([.15,-1.2])
        np.testing.assert_allclose(self.plant.rhs(z,u,.9,.95),np.asarray(f(z,u,[.9,.95])).ravel(),atol=1e-12)

    def test_rk4_independent_solver(self):
        z=np.array([1.,2.,.3,6.,.4,.18,.12]);u=np.array([.15,-1.2]);start=z.copy()
        for _ in range(10):z=self.plant.step(z,u,.01)
        sol=solve_ivp(lambda t,q:self.plant.rhs(q,u),(0,.1),start,method='DOP853',rtol=1e-11,atol=1e-12)
        np.testing.assert_allclose(z,sol.y[:,-1],atol=2e-6)

    def test_speed_plan_limits(self):
        cfg=DrivingConfig();plan=CurvatureSpeedPlan(self.road,cfg);v2=plan.speed**2
        self.assertLessEqual(float(np.max(plan.speed)),cfg.maximum_speed_mps+1e-8)
        self.assertLessEqual(float(np.max(v2*plan.curvature_envelope)),cfg.lateral_acceleration_budget_mps2+1e-8)
        self.assertLessEqual(float(np.max(np.roll(v2,-1)-v2)),2*cfg.planning_acceleration_mps2*plan.ds+1e-8)
        self.assertLessEqual(float(np.max(v2-np.roll(v2,-1))),2*cfg.planning_braking_mps2*plan.ds+1e-8)

    def test_projection_round_trip(self):
        s=np.array([0.,99.,440.,1655.,self.road.length+3.])
        q=self.road.sample_geometry(s);xy=q['center']+.5*q['normal_left']
        ss,e,resid,_=project_near_batch(self.road,xy,s)
        np.testing.assert_allclose(e,.5,atol=1e-8);np.testing.assert_allclose(ss,s,atol=1e-4)
        self.assertLess(float(np.max(resid)),1e-8)

    def test_footprint_sample_count(self):
        # Source-hash check performed separately in validation/source_integrity.json.
        self.assertEqual(footprint_points(self.params).shape,(40,2))

    def test_mpc_solves_nonzero_offset(self):
        try: import casadi
        except ImportError: self.skipTest('CasADi not installed')
        from tracklab.driving.controller import TrackingMPC
        cfg=DrivingConfig(horizon_steps=10)
        plan=CurvatureSpeedPlan(self.road,cfg);q=self.road.sample_geometry(100.)
        xy=q['center']+.5*q['normal_left']
        z=np.array([*xy,float(q['heading_rad']),8,0,0,0])
        mpc=TrackingMPC(self.plant,self.road,plan,cfg)
        u,info=mpc.solve(z,100.,np.zeros(2))
        self.assertTrue(info['success']);self.assertLess(info['constraint_violation'],2e-5)
        self.assertTrue(np.isfinite(u).all());self.assertLess(u[0],0) # turns back toward path

if __name__=='__main__':unittest.main()
