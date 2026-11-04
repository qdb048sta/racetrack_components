import unittest
import numpy as np
from scipy.integrate import solve_ivp
from tracklab import PeriodicTrack
from tracklab.paper_models.sun2025 import Longitudinal,KinematicBicycle,KinematicMPC,SpatialFollowingMPC
from tracklab.paper_models.common import road_reference


class SunTests(unittest.TestCase):
    def test_energy_transformation_chain_rule(self):
        m=Longitudinal();speed=12.;E=.5*m.p.mass*speed**2
        temporal=m.time_rhs([0,speed],150.,.02)
        spatial=m.spatial_rhs([1.5,E],150.,10.,.02)
        self.assertAlmostEqual(spatial[1],m.p.mass*temporal[1])
        self.assertAlmostEqual(spatial[0],1/speed-1/10)
        with self.assertRaises(ValueError):m.spatial_rhs([1,0],0,10)

    def test_straight_and_constant_steering_circle(self):
        p=KinematicBicycle();v=8.;delta=.1;beta=np.arctan(p.p.lr/(p.p.lf+p.p.lr)*np.tan(delta))
        f=p.rhs([0,0,0,v,delta],[0,0]);self.assertAlmostEqual(np.hypot(f[0],f[1]),v)
        self.assertAlmostEqual(f[2],v*np.sin(beta)/p.p.lr)
        np.testing.assert_allclose(p.rhs([0,0,0,v,0],[0,0]),[v,0,0,0,0])

    def test_time_integration_independent(self):
        p=KinematicBicycle();x=np.array([0,0,0,8,.1]);u=np.array([.3,.02])
        truth=solve_ivp(lambda t,z:p.rhs(z,u),(0,.2),x,rtol=1e-11,atol=1e-12).y[:,-1]
        np.testing.assert_allclose(p.step(x,u,.2),truth,atol=1e-8)

    def test_spatial_relaxation_reports_and_attains_tightness(self):
        p=Longitudinal();mpc=SpatialFollowingMPC(p);x=[1.5,.5*p.p.mass*100]
        steady=p.resistance(10)*p.p.radius/(p.p.motors*p.p.efficiency)
        torque,info=mpc.solve(x,np.full(mpc.N+1,10.),previous_torque=steady)
        self.assertLess(info["residual"],1e-5)
        self.assertLess(np.max(np.abs(info["relaxation_gap"])),1e-4)
        self.assertTrue(np.isfinite(torque))

    def test_kinematic_road_mpc_and_terminal_disk(self):
        road=PeriodicTrack("data/Norisring.csv");p=KinematicBicycle();mpc=KinematicMPC(p)
        q,heading=road_reference(road,0,8,mpc.dt,mpc.N)
        z=np.r_[q["center"][0],heading[0],8,0]
        u,info=mpc.solve(z,q["center"].T,q["normal_left"].T,np.vstack([q["width_right_m"]-1,q["width_left_m"]-1]),heading)
        self.assertLess(info["residual"],1e-5)
        self.assertLessEqual(np.linalg.norm(info["prediction"][:2,-1]-q["center"][-1]),1.00001)
        self.assertTrue(np.isfinite(u).all())

    def test_loose_relaxation_uses_explicit_physical_fallback(self):
        p=Longitudinal();mpc=SpatialFollowingMPC(p)
        _,info=mpc.solve([1.2,.5*p.p.mass*12**2],np.full(mpc.N+1,10.),previous_torque=40.)
        self.assertTrue(info["physical_fallback"])
        self.assertGreater(info["relaxed_candidate_gap_max"],1e-4)
        self.assertLess(np.max(np.abs(info["relaxation_gap"])),1e-6)


if __name__=="__main__":unittest.main()
