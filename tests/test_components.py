"""Run: python -m unittest discover -s tests -v"""
from pathlib import Path
import sys
import tempfile
import unittest
import importlib.util
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tracklab import TrackData, PeriodicTrack, make_curve, METHODS, FlatSurface
from tracklab.adapter import PlanarState, WheelGeometry, RoadCarAdapter

spec = importlib.util.spec_from_file_location("legacy_road",
                    ROOT / "original" / "racetrack_interpolation_demo.py")
legacy = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = legacy
spec.loader.exec_module(legacy)
DATA = ROOT / "data" / "Norisring.csv"


class ComponentsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = TrackData.from_csv(DATA)
        cls.road = PeriodicTrack(DATA)
        cls.old = legacy.PeriodicTrack(DATA)

    def test_loader_same_arrays(self):
        a, u, period = legacy.load_track(DATA)
        np.testing.assert_array_equal(self.data.rows, a)
        np.testing.assert_array_equal(self.data.station_u, u)
        self.assertEqual(self.data.period_u, period)
        self.assertEqual(len(a), 460)

    def test_all_methods_match_original_all_phases(self):
        for method in METHODS:
            for phase in range(4):
                with self.subTest(method=method, phase=phase):
                    ids = np.arange(phase, len(self.data.rows), 4)
                    args = (method, self.data.station_u[ids], self.data.xy[ids], self.data.period_u)
                    old, new = legacy.make_curve(*args), make_curve(*args)
                    u = np.r_[self.data.station_u, -2.0, self.data.period_u + 2]
                    for order in (0, 1, 2):
                        np.testing.assert_allclose(new(u, order), old(u, order), atol=1e-12, rtol=1e-12)
                    np.testing.assert_allclose(new.curvature(u), old.curvature(u),
                                               atol=1e-12, rtol=1e-12, equal_nan=True)

    def test_sample_matches_original_cubic_and_quintic(self):
        for method in ("periodic_cubic", "periodic_quintic"):
            new, old = PeriodicTrack(DATA, method), legacy.PeriodicTrack(DATA, method)
            self.assertEqual(new.length, old.length)
            s = np.r_[-.2, np.linspace(0, new.length, 1001), new.length + 23]
            for value in (s, 100.0):
                actual, expected = new.sample(value), old.sample(value)
                self.assertEqual(set(actual), set(expected))
                for key in actual:
                    np.testing.assert_allclose(actual[key], expected[key], atol=1e-12, rtol=1e-12)

    def test_projection_matches_original_and_keeps_lap(self):
        for s in (0.0, 100.0, 523.1, self.road.length-.3, 2*self.road.length + 100):
            q = self.road.sample(s)
            xy = q["center"] + .25*q["normal_left"]
            actual, expected = self.road.project(xy, s), self.old.project(xy, s)
            for key in actual:
                self.assertAlmostEqual(actual[key], expected[key], places=10)
            self.assertAlmostEqual(actual["s_m"], s, places=4)

    def test_frames_orthonormal(self):
        q = self.road.sample(np.linspace(0, self.road.length, 1001))
        np.testing.assert_allclose(np.linalg.norm(q["tangent"], axis=1), 1, atol=1e-13)
        np.testing.assert_allclose(np.linalg.norm(q["normal_left"], axis=1), 1, atol=1e-13)
        np.testing.assert_allclose(np.sum(q["tangent"]*q["normal_left"], axis=1), 0, atol=1e-13)

    def test_boundary_offsets(self):
        q = self.road.sample(np.linspace(0, self.road.length, 1001))
        np.testing.assert_allclose(np.linalg.norm(q["boundary_left"]-q["center"], axis=1),
                                   q["width_left_m"], atol=1e-12)
        np.testing.assert_allclose(np.linalg.norm(q["boundary_right"]-q["center"], axis=1),
                                   q["width_right_m"], atol=1e-12)

    def test_periodic_queries(self):
        s = np.array([0.0, 100.0, 900.2])
        for key, value in self.road.sample(s).items():
            other = self.road.sample(s+3*self.road.length)[key]
            # Floating modulo may represent the seam as 0 or period-epsilon.
            # Compare cyclic labels modulo their period rather than as real numbers.
            if key in ("s_m", "u_chord_m"):
                period = self.road.length if key == "s_m" else self.road.U
                difference = (value-other+period/2) % period-period/2
                np.testing.assert_allclose(difference, 0, atol=1e-10)
            else:
                np.testing.assert_allclose(value, other, atol=1e-10)

    def test_arc_length_roundtrip(self):
        s = np.linspace(0, self.road.length-.001, 5000)
        recovered = self.road.s_from_u(self.road.u_from_s(s))
        self.assertLess(float(np.max(abs(recovered-s))), 1e-4)

    def test_linear_curvature_not_zero_certificate(self):
        curve = make_curve("linear", self.data.station_u, self.data.xy, self.data.period_u)
        self.assertTrue(np.isnan(curve.curvature([0, 10])).all())

    def test_smoothing_zero_matches_cubic(self):
        args = (self.data.station_u[::4], self.data.xy[::4], self.data.period_u)
        smooth = make_curve("periodic_smoothing_cubic", *args, lam=0)
        cubic = make_curve("periodic_cubic", *args)
        np.testing.assert_allclose(smooth(self.data.station_u), cubic(self.data.station_u), atol=1e-10)

    def test_surface_separate_from_geometry(self):
        wet = PeriodicTrack(DATA, surface=FlatSurface(.6, .02))
        q, base = wet.sample(100), self.road.sample(100)
        np.testing.assert_array_equal(q["center"], base["center"])
        self.assertEqual(float(q["friction_multiplier_assumed"]), .6)
        self.assertEqual(float(q["height_m_assumed"]), .02)

    def test_disallow_unvalidated_smoothing_in_road(self):
        with self.assertRaises(ValueError):
            PeriodicTrack(DATA, "periodic_smoothing_cubic")

    def test_nonfinite_station_rejected(self):
        with self.assertRaises(ValueError):
            self.road.sample(float('nan'))

    def test_loader_rejects_nonpositive_width(self):
        rows = self.data.rows.copy(); rows[0,2] = 0
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'bad.csv'; np.savetxt(path, rows, delimiter=',')
            with self.assertRaises(ValueError): TrackData.from_csv(path)

    def test_export_equivalence(self):
        with tempfile.TemporaryDirectory() as directory:
            left, right = Path(directory)/'a', Path(directory)/'b'
            left.mkdir(); right.mkdir()
            actual = self.road.export(left/'road.csv', spacing_m=2)
            expected = self.old.export(right/'road.csv', spacing_m=2)
            self.assertEqual(actual, expected)
            self.assertEqual((left/'road.csv').read_bytes(), (right/'road.csv').read_bytes())

    def test_adapter_observation_and_rates(self):
        q = self.road.sample(100)
        xy = q['center']+.25*q['normal_left']
        state = PlanarState(*map(float, xy), float(q['heading_rad']), 10., 0., .1)
        adapter = RoadCarAdapter(self.road)
        obs = adapter.observe(state, hint_s=100)
        self.assertAlmostEqual(obs['e_left_m'], .25, places=7)
        self.assertAlmostEqual(obs['heading_error_rad'], 0, places=6)
        rate = adapter.frenet_rates(state, obs)
        k = float(obs['road']['curvature_1pm'])
        self.assertAlmostEqual(rate['s_dot_mps'], 10/(1-k*.25), places=7)
        self.assertAlmostEqual(rate['e_dot_mps'], 0, places=6)
        self.assertAlmostEqual(rate['heading_error_dot_radps'], .1-k*rate['s_dot_mps'], places=10)

    def test_wheel_positions_rotation_and_order(self):
        state = PlanarState(10, 20, np.pi/2, 0, 0, 0)
        adapter = RoadCarAdapter(self.road, WheelGeometry(2, 1, 2, 2))
        np.testing.assert_allclose(adapter.wheel_positions(state),
                                   [[9,22], [11,22], [9,19], [11,19]], atol=1e-12)
        env = adapter.wheel_environment(state)
        self.assertEqual(list(env['lambda_mue']),
                         ['front_left','front_right','rear_left','rear_right'])
        self.assertTrue(all(x == 1 for x in env['lambda_mue'].values()))

    def test_frenet_singularity_rejected(self):
        state = PlanarState(0,0,0,10,0,0)
        obs = {'e_left_m':10, 'heading_error_rad':0, 'road':{'curvature_1pm':.1}}
        with self.assertRaises(ValueError):
            RoadCarAdapter(self.road).frenet_rates(state, obs)


if __name__ == '__main__':
    unittest.main()
