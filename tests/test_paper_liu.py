import unittest
from dataclasses import replace
import numpy as np
from tracklab.paper_models.liu2023 import ModalRing,RigidRing,LegendrePCM,relaxation_step


class LiuTests(unittest.TestCase):
    def test_generalized_eigenproblem(self):
        ring=ModalRing()
        for n in [0,1,2,8]:
            M,_,K=ring.matrices(n);f,V=ring.modes(n)
            np.testing.assert_allclose(K@V,M@V@np.diag((2*np.pi*f)**2),rtol=1e-10,atol=1e-7)
            self.assertTrue(np.all(f>0))

    def test_free_modal_motion_matches_analytic_solution(self):
        ring=ModalRing(replace(ModalRing().p,damping=0));f,V=ring.modes(2)
        times=np.linspace(0,.02,80);q=V[:,0]*1e-3
        x=ring.integrate(2,times,np.r_[q,0,0])
        np.testing.assert_allclose(x[:,:2],np.cos(2*np.pi*f[0]*times)[:,None]*q,atol=1e-9)

    def test_relaxation_time_constant_and_rest(self):
        self.assertAlmostEqual(relaxation_step(0,1000,10,.1,.01),1000*(1-np.exp(-1)))
        self.assertEqual(relaxation_step(5,1000,0,.1,.1),5)
        self.assertEqual(relaxation_step(5,1000,10,.1,0),5)

    def test_rigid_ring_static_force_balance(self):
        ring=RigidRing();x=np.zeros(10);x[1]=-1000/ring.p.kz
        np.testing.assert_allclose(ring.rhs(0,x,lambda t:{"fz":-1000,"axle_load":1000}),0,atol=1e-12)

    def test_pcm_recovers_polynomial_and_moments(self):
        points=np.linspace(-1,1,12)[:,None]
        model=LegendrePCM(1,2).fit(points,2+3*points[:,0]+4*(3*points[:,0]**2-1)/2)
        np.testing.assert_allclose(model.coefficients,[2,3,4],atol=1e-12)
        mean,var=model.moments();self.assertAlmostEqual(mean,2);self.assertAlmostEqual(var,9/3+16/5)
        with self.assertRaises(ValueError):LegendrePCM(1,2).fit(np.zeros((5,1)),np.ones(5))


if __name__=="__main__":unittest.main()
