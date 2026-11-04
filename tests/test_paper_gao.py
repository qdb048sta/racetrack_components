import unittest
import numpy as np
import casadi as ca
from scipy.integrate import solve_ivp
from tracklab import PeriodicTrack
from tracklab.paper_models.gao2014 import FourWheel,SpatialBicycle,SpatialMPC,TemporalMPC,fiala,magic_formula


class GaoTests(unittest.TestCase):
    def test_fiala_symmetry_saturation_and_friction_circle(self):
        for fx in [0,1000,2500]:
            a=np.linspace(-.4,.4,101);fy=fiala(a,40000,4000,.8,fx)
            np.testing.assert_allclose(fy,-fiala(-a,40000,4000,.8,fx),atol=1e-10)
            self.assertLessEqual(np.max(np.hypot(fy,fx)),3200+1e-8)
            cap=np.sqrt(3200**2-fx**2);threshold=np.arctan(3*cap/40000)
            self.assertAlmostEqual(float(fiala(threshold-1e-8,40000,4000,.8,fx)),-cap,places=5)
        self.assertEqual(float(fiala(.1,40000,4000,.8,3200)),0.)

    def test_fiala_symbolic_and_numeric(self):
        a=ca.SX.sym("a");f=ca.Function("tire",[a],[fiala(a,40000,4000,.8,1000,True)])
        for x in [-.3,-.01,0,.01,.3]:self.assertAlmostEqual(float(f(x)),float(fiala(x,40000,4000,.8,1000)),places=8)

    def test_magic_formula_small_slip_slope(self):
        self.assertAlmostEqual(magic_formula(1e-8,10,1.3,4000,.1)/1e-8,52000,delta=.001)

    def test_yaw_force_signs_and_symmetry(self):
        p=FourWheel();x=np.array([0,0,0,10,0,0.])
        self.assertGreater(p.rhs(x,[.05,0,0])[5],0)
        self.assertAlmostEqual(p.rhs(x,[0,.1,.1])[5],0.)
        self.assertGreater(p.rhs(x,[0,0,.1])[5],0.)
        self.assertLess(p.rhs([0,0,0,10,1,0],[0,0,0])[4],0.)

    def test_spatial_chain_rule_on_straight_road(self):
        s=SpatialBicycle();z=np.array([0,10,0,0,0,0.]);f=s.rhs(z,[0,0],0)
        np.testing.assert_allclose(f,[0,0,0,0,0,.1],atol=1e-12)
        with self.assertRaises(ValueError):s.rhs([0,0,0,0,0,0],[0,0],0)

    def test_plant_integrator_independent(self):
        p=FourWheel();x=np.array([0,0,.1,10,.2,.1]);u=np.array([.04,-.1,-.1])
        actual=p.step(x,u,.05)
        expected=solve_ivp(lambda t,z:p.rhs(z,u),(0,.05),x,rtol=1e-11,atol=1e-12,method="DOP853").y[:,-1]
        np.testing.assert_allclose(actual,expected,atol=2e-7)

    def test_two_optimizations_share_a_road_reference(self):
        road=PeriodicTrack("data/Norisring.csv");q=road.sample_geometry(0)
        plan=SpatialMPC().solve(road,0,np.array([0,10,0,0,0,0.]))
        x=np.r_[q["center"],q["heading_rad"],10,0,0]
        u,info=TemporalMPC(FourWheel()).solve(x,plan,np.zeros(3))
        self.assertTrue(np.isfinite(u).all());self.assertLess(info["residual"],1e-5)
        self.assertTrue(np.all(np.diff(plan["time"])>0));self.assertLess(plan["residual"],1e-5)


if __name__=="__main__":unittest.main()
