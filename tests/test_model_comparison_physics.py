"""Independent physical invariants for the normalized double-track plants."""
from dataclasses import replace
import unittest

import numpy as np

from tracklab.model_comparison.models import (
    ReducedDoubleTrack, VehicleParameters, fiala_lateral, magic_formula,
)


class TirePhysicsTest(unittest.TestCase):
    def test_small_slip_stiffness_and_signs(self):
        p = VehicleParameters()
        load = np.full(4, 2000.)
        eps = 1e-8
        for tire in ("mf_independent", "mf_ellipse", "fiala"):
            plant = ReducedDoubleTrack(tire_model=tire)
            fx, fy = plant.tire_forces(np.full(4, eps), np.full(4, eps), load)
            np.testing.assert_allclose(fx/eps, load*40, rtol=1e-5)
            np.testing.assert_allclose(fy/eps, load*16, rtol=1e-5)
            self.assertTrue(np.all(fx > 0) and np.all(fy > 0))

    def test_combined_force_constraint_and_native_independence(self):
        load = np.array([1000., 2000., 3000., 4000.])
        max_independent = 0.
        for alpha in np.linspace(-.5, .5, 17):
            for kappa in np.linspace(-.2, .2, 17):
                for tire in ("mf_independent", "mf_ellipse", "fiala"):
                    plant = ReducedDoubleTrack(tire_model=tire)
                    fx, fy = plant.tire_forces(np.full(4, alpha), np.full(4, kappa), load)
                    utilization = np.hypot(fx, fy)/load
                    if tire == "mf_independent":
                        max_independent = max(max_independent, float(np.max(utilization)))
                    else:
                        self.assertLessEqual(float(np.max(utilization)), 1.+1e-12)
        # Regression against silently adding a friction circle to MF_SIMPLE.
        self.assertGreater(max_independent, 1.35)

    def test_fiala_saturation_is_continuous_and_odd(self):
        stiffness, capacity = 32000., 2000.
        threshold = np.arctan(3*capacity/stiffness)
        angles = np.array([threshold-1e-7, threshold, threshold+1e-7])
        forces = fiala_lateral(angles, stiffness, capacity)
        np.testing.assert_allclose(forces, capacity, atol=1e-8)
        np.testing.assert_allclose(fiala_lateral(-angles, stiffness, capacity), -forces)
        self.assertEqual(float(fiala_lateral(.3, stiffness, 0.)), 0.)

    def test_zero_friction_gives_zero_tire_force(self):
        for tire in ("mf_independent", "mf_ellipse", "fiala"):
            plant = ReducedDoubleTrack(tire_model=tire)
            fx, fy = plant.tire_forces(np.full(4, .2), np.full(4, .1), np.full(4, 2000.), np.zeros(4))
            np.testing.assert_array_equal(fx, np.zeros(4))
            np.testing.assert_array_equal(fy, np.zeros(4))


class ChassisPhysicsTest(unittest.TestCase):
    def test_weight_conservation_and_load_transfer_direction(self):
        plant = ReducedDoubleTrack()
        p = plant.parameters
        static, _ = plant.normal_loads(0., 0., 0.)
        np.testing.assert_allclose(static.sum(), p.mass_kg*p.gravity_mps2)
        loads, outside = plant.normal_loads(3., 4., 20.)
        downforce = -.5*p.air_density_kgpm3*p.lift_coefficient_area_m2*20**2
        self.assertAlmostEqual(loads.sum(), p.mass_kg*p.gravity_mps2+downforce)
        self.assertFalse(outside)
        self.assertLess(loads[0], loads[1])  # Left turn loads the right wheels.
        self.assertLess(loads[2], loads[3])
        self.assertLess(loads[:2].sum()/(loads.sum()), p.front_weight_fraction)
        self.assertAlmostEqual((loads[1]-loads[0])*p.front_track_m/2 +
                               (loads[3]-loads[2])*p.rear_track_m/2,
                               p.mass_kg*4*p.cg_height_m)

    def test_contact_limit_is_reported_and_weight_remains_conserved(self):
        plant = ReducedDoubleTrack()
        loads, outside = plant.normal_loads(100., 100., 0.)
        self.assertTrue(outside)
        self.assertGreaterEqual(loads.min(), 0.)
        self.assertAlmostEqual(loads.sum(), plant.parameters.mass_kg*plant.parameters.gravity_mps2)

    def test_symmetry_and_force_moment_balance(self):
        p = replace(VehicleParameters(), drag_coefficient_area_m2=0.,
                    lift_coefficient_area_m2=0., rolling_resistance_coefficient=0.)
        plant = ReducedDoubleTrack(parameters=p)
        plant.reset([0., 0., 0., 15., 0., 0., 0.])
        z = plant.state
        z[6:10] *= 1.03
        rhs, d = plant.evaluate(z, np.zeros(4), 0.)
        self.assertAlmostEqual(rhs[4], 0., places=12)
        self.assertAlmostEqual(rhs[5], 0., places=12)
        self.assertAlmostEqual(p.mass_kg*rhs[3], d["tire_force_longitudinal_N"].sum(), places=8)
        self.assertLess(d["load_balance_residual_mps2"], 1e-5)
        # Positive steer must generate leftward force and positive yaw acceleration.
        z[6:10] = 15/p.wheel_radius_m
        z[10] = .025
        rhs, _ = plant.evaluate(z, np.zeros(4), .025)
        self.assertGreater(rhs[4], 0.)
        self.assertGreater(rhs[5], 0.)

    def test_asymmetric_longitudinal_force_has_correct_yaw_moment(self):
        p = replace(VehicleParameters(), drag_coefficient_area_m2=0.,
                    lift_coefficient_area_m2=0., rolling_resistance_coefficient=0.)
        plant = ReducedDoubleTrack(parameters=p, load_transfer="static")
        plant.reset([0., 0., 0., 15., 0., 0., 0.])
        z = plant.state
        z[7] *= 1.01  # Front-right traction increases positive yaw moment.
        rhs, d = plant.evaluate(z, np.zeros(4), 0.)
        expected = p.front_track_m/2*d["tire_force_longitudinal_N"][1]
        self.assertAlmostEqual(p.yaw_inertia_kgm2*rhs[5], expected, places=8)
        self.assertGreater(rhs[5], 0.)

    def test_step_refinement_and_snapshot_replay(self):
        for tire in ("mf_independent", "mf_ellipse", "fiala"):
            a = ReducedDoubleTrack(tire_model=tire, integration_step_s=.001)
            b = ReducedDoubleTrack(tire_model=tire, integration_step_s=.0005)
            initial = [0., 0., .2, 12., 0., 0., 0.]
            a.reset(initial); b.reset(initial)
            torques = np.full(4, 70.)
            out_a = a.step(torques, .04, .15)
            out_b = b.step(torques, .04, .15)
            np.testing.assert_allclose(out_a, out_b, atol=2e-6, rtol=1e-6)
            snap = a.state
            expected = a.step(torques, .01, .02)
            a.reset(snap)
            actual = a.step(torques, .01, .02)
            np.testing.assert_allclose(actual, expected, atol=1e-13, rtol=0)


if __name__ == "__main__":
    unittest.main()
