"""Checks against the compiled, pinned TUM model rather than a substitute plant.

Run these in the environment used to build driving_OCD's native library. A
missing compiler/library is an error: this suite must not silently validate the
older driving model instead.
"""
from __future__ import annotations

from dataclasses import replace
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.integrate import solve_ivp

from tracklab import PeriodicTrack
from tracklab.driving_OCD.config import VehicleConfig, DrivingConfig
from tracklab.driving_OCD.vehicle import OCDVehicle
from tracklab.driving_OCD.native import NativeVehicle, SOURCE_COMMIT, MODEL_NAME


ROOT = Path(__file__).resolve().parents[1]


class OCDNativeBridgeTests(unittest.TestCase):
    def test_pinned_equation_source_hashes(self):
        # Commit/submodule and source-hash verification precedes the native
        # build. Recheck the pinned source/header files directly here so the
        # runtime suite does not traverse Git metadata on a mounted drive.
        manifest = json.loads((ROOT/'tracklab'/'driving_OCD'/'native'/'upstream.json').read_text())
        self.assertEqual(manifest['commit'], SOURCE_COMMIT)
        self.assertEqual(manifest['model'], MODEL_NAME)
        self.assertIn('lib/ocd_tire_models_cpp/src/linear.cpp', manifest['sha256_lf'])
        for relative, expected in manifest['sha256_lf'].items():
            with self.subTest(source=relative):
                source = (ROOT/'dev'/'Open-Car-Dynamics'/relative).read_bytes().replace(b'\r\n', b'\n')
                self.assertEqual(hashlib.sha256(source).hexdigest(), expected)

    def test_missing_library_is_an_error(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'missing_tum_ocd_library'
            with self.assertRaisesRegex(RuntimeError, 'native library is missing'):
                NativeVehicle(library_path=path)

    def test_compiled_identity_and_parameter_mapping(self):
        cfg = VehicleConfig()
        with NativeVehicle(cfg.native_parameters()) as native:
            self.assertEqual(native._lib.ocd_identity().decode(), f'{SOURCE_COMMIT}:{MODEL_NAME}')
            parameters = native.parameters()
            for key, expected in cfg.native_parameters().items():
                with self.subTest(parameter=key):
                    self.assertEqual(parameters[key], expected)
            with self.assertRaises(RuntimeError):
                native.set_parameter('not.a.real.upstream.parameter', 1.)

    def test_rhs_evaluator_preserves_evolving_native_state(self):
        z = np.array([1., 2., .2, 8., 0., 0., 8./.3, 8./.3, 8./.3, 8./.3, 0.])
        with NativeVehicle(VehicleConfig().native_parameters()) as native:
            native.reset(z)
            sample = z.copy()
            sample[4], sample[10] = .2, .1
            derivative, _ = native.evaluate(sample, [10., 10., 10., 10.], .1)
            self.assertTrue(np.isfinite(derivative).all())
            np.testing.assert_array_equal(native.state, z)


class OCDVehicleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params = VehicleConfig()
        cls.plant = OCDVehicle(cls.params)

    @classmethod
    def tearDownClass(cls):
        cls.plant.native.close()

    def state(self, vx=8., vy=0., yaw_rate=0., steer=0.):
        return self.plant.initial_state(np.array([1., 2., .2, vx, vy, yaw_rate, steer]))

    def test_full_native_state_round_trip(self):
        body = np.array([1., 2., .2, 8., .3, .1, .12])
        state = self.plant.initial_state(body)
        self.assertEqual(state.shape, (11,))
        np.testing.assert_array_equal(self.plant.observe(state), body)
        self.assertTrue(np.all(state[6:10] > 0.))

    def test_global_kinematics_and_restoring_slip(self):
        z = self.state(vy=.4)
        rhs = self.plant.rhs(z, [0., 0.])
        np.testing.assert_allclose(rhs[:2], [8.*np.cos(.2)-.4*np.sin(.2),
                                            8.*np.sin(.2)+.4*np.cos(.2)], atol=1e-12)
        self.assertLess(rhs[4], 0.)
        z = self.state(steer=.05)
        self.assertGreater(self.plant.rhs(z, [.05, 0.])[5], 0.)

    def test_torque_does_not_bypass_wheel_rotation(self):
        z = self.state(vy=.2, yaw_rate=.03, steer=.08)
        coast = self.plant.rhs(z, [.08, 0.])
        accelerate = self.plant.rhs(z, [.08, 1.2])
        # At this instant wheel slip and therefore body force are unchanged.
        # The torque command first accelerates the wheels, changing later slip.
        np.testing.assert_allclose(accelerate[:6], coast[:6], atol=1e-12)
        self.assertTrue(np.all(accelerate[6:10] > coast[6:10]))

    def test_upstream_slip_angle_definition_at_low_speed(self):
        z = self.state(vx=.7, vy=.12, yaw_rate=.03, steer=.2)
        d = self.plant.diagnostics(z, [.2, 0.])
        tire_vx = np.cos(.2)*.7 + np.sin(.2)*(.12+self.params.lf_m*.03)
        tire_vy = -np.sin(.2)*.7 + np.cos(.2)*(.12+self.params.lf_m*.03)
        expected = -np.arctan2(tire_vy, max(1., tire_vx))
        self.assertAlmostEqual(d['alpha_front_rad'], expected, places=12)

    def test_linear_tire_force_and_external_scale(self):
        z = self.state(vy=.45, yaw_rate=.06, steer=.12)
        for mu_front, mu_rear in ((1., 1.), (.65, 1.15)):
            with self.subTest(mu=(mu_front, mu_rear)):
                d = self.plant.diagnostics(z, [.12, 0.], mu_front, mu_rear)
                for axle, mu in (('front', mu_front), ('rear', mu_rear)):
                    expected = (self.params.cornering_coefficient_per_rad *
                                d[f'alpha_{axle}_rad'] * d[f'fz_{axle}_n'] * mu)
                    self.assertAlmostEqual(d[f'fy_{axle}_n'], expected, places=8)

    def test_linear_tire_has_no_added_friction_circle(self):
        z = self.state(vx=8., vy=2.)
        d = self.plant.diagnostics(z, [0., 0.])
        # Upstream Linear is unbounded in alpha. Diagnostics may exceed unity;
        # a friction-circle clamp would silently replace the requested model.
        self.assertGreater(abs(d['fy_front_n']), d['fz_front_n'])
        self.assertGreater(abs(d['fy_rear_n']), d['fz_rear_n'])

    def test_upstream_pt1_rate_and_angle_limits(self):
        z = self.state()
        rhs = self.plant.rhs(z, [self.params.steering_limit_rad, 0.])
        self.assertAlmostEqual(rhs[10], self.params.steering_rate_limit_radps, places=12)
        z[10] = self.params.steering_limit_rad
        self.assertEqual(self.plant.rhs(z, [self.params.steering_limit_rad, 0.])[10], 0.)
        self.assertLess(self.plant.rhs(z, [-self.params.steering_limit_rad, 0.])[10], 0.)
        z[10] = .1
        rhs = self.plant.rhs(z, [.101, 0.])
        self.assertAlmostEqual(rhs[10], .001/self.params.steering_time_constant_s, places=12)

    def test_stationary_native_guards(self):
        z = self.state(vx=0.)
        rhs = self.plant.rhs(z, [0., 0.])
        np.testing.assert_allclose(rhs, np.zeros(11), atol=1e-12)
        d = self.plant.diagnostics(z, [0., 0.])
        self.assertEqual(d['fy_front_n'], 0.)
        self.assertEqual(d['fy_rear_n'], 0.)

    def test_casadi_prediction_matches_native_derivatives(self):
        symbolic = self.plant.symbolic_rhs()
        samples = [
            (self.state(), [0., 0.], [1., 1.]),
            (self.state(vx=6., vy=.4, yaw_rate=.18, steer=.12), [.15, -1.2], [.9, .95]),
            (self.state(vx=.7, vy=.12, yaw_rate=.03, steer=-.2), [-.25, .6], [.6, 1.1]),
            (self.state(vx=0.), [0., 0.], [1., 1.]),
        ]
        clipped = self.state(vx=4., vy=-.15, yaw_rate=-.08, steer=-.09)
        clipped[6:10] = [0., 200., 0., 250.]
        samples.append((clipped, [-.15, -1.5], [.7, 1.2]))
        for z, u, mu in samples:
            with self.subTest(vx=z[3], steer=z[10], mu=mu):
                expected = self.plant.rhs(z, u, *mu)
                actual = np.asarray(symbolic(z, u, mu)).ravel()
                np.testing.assert_allclose(actual, expected, rtol=2e-11, atol=2e-9)

    def test_symbolic_native_agreement_with_parameter_overrides(self):
        cfg = replace(self.params, aero_center_offset_m=.2, steering_static_offset_rad=.03,
                      cornering_coefficient_per_rad=18., longitudinal_slip_coefficient=150.,
                      wheel_radius_m=.28, rolling_resistance_coefficient=.018)
        plant = OCDVehicle(cfg)
        try:
            state = plant.initial_state([1., 2., -.3, 12., -.2, .1, -.08])
            state[6:10] *= np.array([.98, 1.02, .99, 1.01])
            u, mu = [-.13, -.7], [.8, 1.1]
            expected = plant.rhs(state, u, *mu)
            actual = np.asarray(plant.symbolic_rhs()(state, u, mu)).ravel()
            np.testing.assert_allclose(actual, expected, rtol=2e-11, atol=2e-9)
        finally:
            plant.native.close()

    def test_native_step_refines_toward_independent_component_rhs(self):
        z = self.state(vx=8., vy=.1, yaw_rate=.03, steer=.05)
        u = [.08, .7]
        actual = self.plant.step(z, u, .02)
        independent = solve_ivp(lambda t, q: self.plant.rhs(q, u), (0., .02), z,
                                method='DOP853', rtol=1e-10, atol=1e-11,
                                max_step=.0002)
        self.assertTrue(independent.success)
        reference = independent.y[:, -1]
        # Upstream's cached wheel speeds lag between internal RK evaluations.
        # Its finite native step therefore differs from integrating a pure RHS.
        # Check the small error and its reduction when the native step shrinks.
        np.testing.assert_allclose(actual[:6], reference[:6], rtol=2e-6, atol=3e-6)
        np.testing.assert_allclose(actual[6:10], reference[6:10], rtol=1e-5, atol=2e-4)
        fine_plant = OCDVehicle(self.params, integration_step_s=.00005)
        try:
            fine = fine_plant.step(z, u, .02)
        finally:
            fine_plant.native.close()
        coarse_error = np.linalg.norm(actual-reference)
        fine_error = np.linalg.norm(fine-reference)
        self.assertLess(fine_error, .6*coarse_error)


class OCDDrivingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.road = PeriodicTrack(ROOT/'data'/'Norisring.csv')

    def test_configuration_validation(self):
        with self.assertRaises(ValueError):
            DrivingConfig(initial_speed_mps=0.).validate()
        with self.assertRaises(ValueError):
            VehicleConfig(lf_m=4.).validate()

    def test_speed_plan_on_original_road(self):
        from tracklab.driving_OCD.planning import CurvatureSpeedPlan
        cfg = DrivingConfig()
        plan = CurvatureSpeedPlan(self.road, cfg)
        v2 = plan.speed**2
        self.assertLessEqual(float(np.max(plan.speed)), cfg.maximum_speed_mps+1e-8)
        self.assertLessEqual(float(np.max(v2*plan.curvature_envelope)),
                             cfg.lateral_acceleration_budget_mps2+1e-8)
        self.assertLessEqual(float(np.max(np.roll(v2, -1)-v2)),
                             2*cfg.planning_acceleration_mps2*plan.ds+1e-8)
        self.assertLessEqual(float(np.max(v2-np.roll(v2, -1))),
                             2*cfg.planning_braking_mps2*plan.ds+1e-8)

    def test_closed_loop_native_simulation_smoke(self):
        from tracklab.driving_OCD.artifacts import save_results
        from tracklab.driving_OCD.simulation import simulate
        cfg = replace(DrivingConfig(), laps=.001, maximum_time_s=3., horizon_steps=10)
        result = simulate(self.road, VehicleConfig(), cfg)
        try:
            self.assertEqual(result['states'].shape[1], 7)
            self.assertEqual(result['native_states'].shape[1], 11)
            np.testing.assert_array_equal(result['native_states'][:, :6], result['states'][:, :6])
            np.testing.assert_array_equal(result['native_states'][:, 10], result['states'][:, 6])
            self.assertGreaterEqual(result['progress_m'][-1], self.road.length*cfg.laps)
            self.assertTrue(all(info['success'] for info in result['solves']))
            self.assertTrue(np.all(np.isfinite(result['native_states'])))
            with tempfile.TemporaryDirectory() as directory:
                output = Path(directory)
                metrics = save_results(self.road, result, output, independent_check=False)
                self.assertEqual(metrics['native_model'], MODEL_NAME)
                with np.load(output/'run_raw.npz') as saved:
                    np.testing.assert_array_equal(saved['native_states'], result['native_states'])
                provenance = json.loads((output/'provenance.json').read_text())
                self.assertEqual(provenance['commit'], SOURCE_COMMIT)
                self.assertEqual(len(provenance['native_state_columns']), 11)
                self.assertEqual(len(provenance['native_library_sha256']), 64)
                html = (output/'norisring_replay.html').read_text(encoding='utf-8')
                payload = json.loads(html.split('<script id="payload" type="application/json">', 1)[1]
                                    .split('</script>', 1)[0])
                self.assertEqual(payload['metrics']['native_model'], MODEL_NAME)
                self.assertAlmostEqual(payload['rows'][-1][0], result['time_s'][-1], places=7)
                self.assertNotIn('<script src=', html)
        finally:
            result['plant'].native.close()


if __name__ == '__main__':
    unittest.main()
