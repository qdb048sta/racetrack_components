"""Focused integration checks against the actual compiled OCD plant."""
import unittest
import numpy as np

from .native import DEFAULT_LIBRARY_PATH
from .vehicle import NativeDoubleTrack, normalized_parameters


@unittest.skipUnless(DEFAULT_LIBRARY_PATH.is_file(), "build native double-track adapter first")
class NativeDoubleTrackTests(unittest.TestCase):
    def make(self):
        model = NativeDoubleTrack(normalized_parameters())
        self.addCleanup(model.close)
        return model

    def test_vertical_equilibrium_and_weight(self):
        model = self.make()
        observation = np.array([3., -2., .4, 8., 0., 0., 0.])
        np.testing.assert_allclose(model.reset(observation), observation, atol=1e-14)
        dx, diagnostic = model.native.evaluate(model.state, [0.]*4, 0.)
        np.testing.assert_allclose(dx[[8, 10, 11, 16, 17, 18, 19]], 0, atol=1e-9)
        self.assertAlmostEqual(sum(diagnostic["tire_force_vertical_N"]),
                               800*9.81 + .5*1.225*1.5*8**2, places=8)

    def test_full_state_snapshot_continuation(self):
        model = self.make()
        model.reset([0, 0, 0, 8, 0, 0, 0])
        model.step([35.]*4, .04, .3)
        state = model.state.copy()
        expected = model.step([40.]*4, -.02, .2)
        model.reset(state)
        np.testing.assert_array_equal(model.state, state)
        actual = model.step([40.]*4, -.02, .2)
        np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_left_right_symmetry(self):
        left, right = self.make(), self.make()
        for model in (left, right):
            model.reset([0, 0, 0, 8, 0, 0, 0])
        zl = left.step([40.]*4, .05, 2.)
        zr = right.step([40.]*4, -.05, 2.)
        np.testing.assert_allclose(zl * [1, -1, -1, 1, -1, -1, -1], zr, atol=1e-10)
        np.testing.assert_allclose(left.diagnostics["tire_force_vertical_N"][[1, 0, 3, 2]],
                                   right.diagnostics["tire_force_vertical_N"], atol=1e-8)
        self.assertGreater(zl[5], 0)

    def test_mf_simple_has_independent_pure_slip_forces(self):
        model = self.make()
        model.reset([0, 0, 0, 10, 0, 0, 0])
        x = model.state
        x[24:28] = .08
        _, no_long = model.native.evaluate(x, [0.]*4, 0.)
        x[20:24] = .15
        _, combined = model.native.evaluate(x, [0.]*4, 0.)
        np.testing.assert_allclose(no_long["tire_force_lateral_N"], combined["tire_force_lateral_N"], atol=1e-10)
        self.assertGreater(np.max(combined["tire_force_longitudinal_N"]), 100)

    def test_half_step_agreement(self):
        coarse, fine = self.make(), self.make()
        for model in (coarse, fine):
            model.reset([0, 0, 0, 8, 0, 0, 0])
        zc = coarse.step([40.]*4, .08, 1., substeps=2000)
        zf = fine.step([40.]*4, .08, 1., substeps=4000)
        np.testing.assert_allclose(zc, zf, atol=5e-4, rtol=0)


if __name__ == "__main__":
    unittest.main()
