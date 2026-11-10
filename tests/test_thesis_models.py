"""Physics/source-convention checks for the normalized thesis model adapters."""
import unittest
import math
import numpy as np
from tracklab.model_comparison.thesis_models import (MODEL_CLASSES,LinigerBicycle,GaoFourWheel,
    BealCoupledBrush,SunKinematic,LienkampLinear,coupled_brush)
from tracklab.model_comparison.models import VehicleParameters


class SourceModelTests(unittest.TestCase):
    def test_gao_drive_brake_slip(self):
        model=GaoFourWheel();model.reset([0,0,0,10,0,0,0]);x=model.state
        x[6:10]=20/.3
        _,d=model.evaluate(x,[0]*4,0)
        np.testing.assert_allclose(d['tire_longitudinal_slip'],.5)
        x[6:10]=5/.3
        _,d=model.evaluate(x,[0]*4,0)
        np.testing.assert_allclose(d['tire_longitudinal_slip'],-.5)
    def test_gao_static_load(self):
        model=GaoFourWheel();model.reset([0,0,0,20,1,.2,.1]);_,d=model.evaluate(model.state,[100]*4,.1)
        np.testing.assert_allclose(d['tire_force_vertical_N'],[1809.945,1809.945,2114.055,2114.055])
    def test_gao_lateral_mf_alpha_not_tangent(self):
        m=GaoFourWheel();alpha=np.ones(4)*.4;fx,fy=m.tire_forces(alpha,np.zeros(4),np.ones(4),np.ones(4))
        from tracklab.model_comparison.models import magic_formula
        np.testing.assert_allclose(fy,magic_formula(alpha,10,1.6,-1.9))
    def test_liniger_rear_torque_collapse(self):
        m=LinigerBicycle();z=np.array([0,0,0,10,.2,.1,.05])
        np.testing.assert_allclose(m.evaluate(z,[100,0,0,0],.05)[0],m.evaluate(z,[0,0,0,100],.05)[0])
    def test_liniger_lateral_source_form(self):
        m=LinigerBicycle();z=np.array([0,0,0,10,0,0,.15]);_,d=m.evaluate(z,[0]*4,.15)
        expected=sum(m.static_loads[:2])*math.sin(1.6*math.atan(10*.15))
        self.assertAlmostEqual(sum(d['tire_force_lateral_N'][:2]),expected)
    def test_brush_peak_and_sliding_limit(self):
        p=VehicleParameters();k=np.linspace(0,2,100001);a=np.zeros_like(k);fz=np.ones_like(k)
        fx,fy=coupled_brush(a,k,fz,fz,p)
        self.assertAlmostEqual(float(max(fx)),1.,places=7)
        self.assertAlmostEqual(float(fx[-1]),539/540,places=12)
        fx,fy=coupled_brush(np.ones(4)*.1,-np.ones(4),np.ones(4),np.ones(4),p)
        self.assertTrue(np.isfinite(fx).all());np.testing.assert_allclose(np.hypot(fx,fy),539/540)
    def test_brush_combined_bound_and_slopes(self):
        rng=np.random.default_rng(403);a=rng.uniform(-.5,.5,10000);k=rng.uniform(-1,3,10000)
        fx,fy=coupled_brush(a,k,np.ones_like(a),np.ones_like(a),VehicleParameters())
        self.assertLessEqual(float(np.max(np.hypot(fx,fy))),1.+1e-12)
        fx,fy=coupled_brush(np.array([0.,1e-9]),np.array([1e-9,0.]),np.ones(2),np.ones(2),VehicleParameters())
        self.assertAlmostEqual(fx[0]/1e-9,40,places=5);self.assertAlmostEqual(fy[1]/1e-9,16,places=5)
    def test_snapshot_continuation_and_symmetry(self):
        for cls in MODEL_CLASSES.values():
            with self.subTest(model=cls.name):
                m=cls();n=cls();initial=[0,0,0,8,0,0,0];m.reset(initial);n.reset(initial)
                a=m.step([40]*4,.05,.3);b=n.step([40]*4,-.05,.3)
                np.testing.assert_allclose(a*[1,-1,-1,1,-1,-1,-1],b,atol=1e-10)
                snapshot=m.state;expected=m.step([45]*4,-.02,.1);m.reset(snapshot)
                np.testing.assert_allclose(m.step([45]*4,-.02,.1),expected,atol=1e-12)
    def test_beal_cg_newton_consistency(self):
        m=BealCoupledBrush();m.reset([0,0,.2,12,.4,.15,.06]);z=m.state;z[11:]=[.02,.08]
        dz,d=m.evaluate(z,[60]*4,.07);eps=1e-7
        derivative=(m.observe(z+eps*dz)-m.observe(z-eps*dz))/(2*eps)
        obs=m.observe(z);expected=np.array([derivative[3]-obs[5]*obs[4],derivative[4]+obs[5]*obs[3]])
        np.testing.assert_allclose(expected,d['acceleration_mps2'][:2],atol=1e-7)
    def test_beal_native_cg_map(self):
        x=np.zeros(33);x[[0,1,3,6,7,9,4,10,32]]=[2,3,.5,10,.4,.2,.03,.1,.06];x[28:32]=32
        m=BealCoupledBrush();z=m.reset_from_native(x)
        np.testing.assert_allclose(z,[2,3,.5,10,.4,.2,.06],atol=1e-13)
    def test_beal_load_mass_balance(self):
        m=BealCoupledBrush();m.reset([0,0,0,20,1,.2,.1]);_,d=m.evaluate(m.state,[100]*4,.1)
        self.assertAlmostEqual(sum(d['tire_force_vertical_N']),800*9.81+.5*1.225*1.5*400,places=8)
        self.assertLess(d['load_balance_residual_N'],1e-8)
    def test_sun_kinematic_no_slip_manifold(self):
        m=SunKinematic();m.reset([0,0,.1,8,0,0,.1]);o=m.observation;p=m.parameters
        self.assertAlmostEqual(o[4]-p.lr_m*o[5],0.)
        self.assertFalse(m.model_info['capabilities']['tire_forces'])
    def test_local_refinement(self):
        for cls in MODEL_CLASSES.values():
            with self.subTest(model=cls.name):
                a=cls();b=cls();a.reset([0,0,0,10,0,0,0]);b.reset([0,0,0,10,0,0,0])
                za=a.step([60]*4,.06,1.,substeps=1000);zb=b.step([60]*4,.06,1.,substeps=2000)
                np.testing.assert_allclose(za,zb,atol=1e-5,rtol=0)

if __name__=='__main__':unittest.main()
