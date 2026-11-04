import unittest
import numpy as np
from scipy.integrate import solve_ivp
from tracklab.paper_models.beal2011 import Parameters,Bicycle,EnvelopeMPC,brush,peak,inverse_brush,derated_capacity


class BealTests(unittest.TestCase):
    def test_brush_symmetry_small_slip_and_slide_continuity(self):
        C,D,R=80000.,4000.,.8
        a=np.linspace(-.4,.4,101)
        np.testing.assert_allclose(brush(a,C,D,R),-brush(-a,C,D,R),atol=1e-10)
        self.assertAlmostEqual(float(brush(1e-8,C,D,R))/1e-8,-C,delta=.1)
        threshold=np.arctan(3*D/C)
        self.assertAlmostEqual(float(brush(threshold-1e-9,C,D,R)),-R*D,places=4)
        self.assertAlmostEqual(float(brush(threshold+1e-9,C,D,R)),-R*D,places=4)

    def test_inverse_chooses_low_slip_branch(self):
        a,limit=peak(80000,4000,.8)
        for target in np.linspace(-limit,limit,31):
            alpha=inverse_brush(target,80000,4000,.8)
            self.assertLessEqual(abs(alpha),a+1e-12)
            self.assertAlmostEqual(float(brush(alpha,80000,4000,.8)),target,places=5)

    def test_combined_force_derating(self):
        D=derated_capacity(.8,4000.,2000.)
        self.assertAlmostEqual(np.hypot(D,2000.),3200.)
        with self.assertRaises(ValueError):derated_capacity(.8,4000.,4000.)

    def test_afi_matches_nonlinear_derivative_at_operating_point(self):
        plant=Bicycle();p=plant.p;x=np.array([.04,.3]);A,B,c=plant.afi(x)
        u=.1;fy=float(brush(x[0]+p.a*x[1]/p.speed-u,p.cf,p.mu*p.loads[0],p.sliding_ratio))
        np.testing.assert_allclose(A@x+B*fy+c,plant.rhs(np.r_[0,0,0,x],u)[3:],atol=1e-12)

    def test_rk4_against_independent_integration(self):
        plant=Bicycle();initial=np.array([0.,0.,.2,.03,.1]);x=initial.copy()
        for _ in range(100):x=plant.step(x,.08,.001)
        truth=solve_ivp(lambda t,z:plant.rhs(z,.08),(0,.1),initial,rtol=1e-11,atol=1e-12,method="DOP853")
        np.testing.assert_allclose(x,truth.y[:,-1],atol=1e-8)

    def test_envelope_solve_is_feasible_and_delay_is_present(self):
        plant=Bicycle();mpc=EnvelopeMPC(plant);z=np.zeros(5)
        steer,info=mpc.solve(z,.1,0.)
        self.assertTrue(np.isfinite(steer));self.assertLess(info["constraint_residual"],1e-5)
        np.testing.assert_allclose(info["prediction"][:,1],0,atol=1e-9)
        self.assertGreater(info["force_n"],0)


if __name__=="__main__":unittest.main()
