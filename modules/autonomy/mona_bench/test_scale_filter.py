"""Regression test against the actual staged upstream scale callbacks."""
import ast
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import numpy as np


class ScaleFilterRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = Path(sys.argv[1]) / "src/scale_recovery/scripts/scale_recovery_planner.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        original = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "ScaleRecovery")
        callbacks = [n for n in original.body if isinstance(n, ast.FunctionDef)
                     and n.name in ("scale_factor_callback", "metric_depth_callback")]
        namespace = {"np": np}
        exec(compile(ast.Module(body=callbacks, type_ignores=[]), str(source), "exec"), namespace)
        cls.scale = staticmethod(namespace["scale_factor_callback"])
        cls.depth = staticmethod(namespace["metric_depth_callback"])

    def setUp(self):
        self.outputs = []
        self.state = SimpleNamespace(
            _SCALE_FACTOR_FILTER=np.array([]), _filter_shape=10, _cam_link="camera_link",
            relative_height_estimate=lambda _: -0.5,
            _bridge=SimpleNamespace(imgmsg_to_cv2=lambda _: np.full((2, 3), 2.0, dtype=np.float32),
                                    cv2_to_imgmsg=lambda array, _: SimpleNamespace(data=array, header=SimpleNamespace())),
            _depth_pub=SimpleNamespace(publish=self.outputs.append))

    def sample(self, height=1.0, points=50):
        self.scale(self.state, SimpleNamespace(width=points, height=1), SimpleNamespace(range=height))
        self.depth(self.state, SimpleNamespace(header=SimpleNamespace(stamp=123)))

    def test_metric_depth_continues_after_initialization(self):
        for _ in range(30):
            self.sample()
        self.assertEqual(len(self.state._SCALE_FACTOR_FILTER), 10)
        self.assertEqual(len(self.outputs), 21)
        np.testing.assert_allclose(self.outputs[-1].data, 4.0)
        self.assertEqual(self.outputs[-1].header.stamp, 123)

    def test_invalid_ranges_and_empty_ground_do_not_initialize(self):
        for value in (0, -1, float("nan"), float("inf"), 8):
            self.sample(value)
        self.state.relative_height_estimate = lambda _: float("nan")
        self.sample()
        self.state.relative_height_estimate = lambda _: 0
        self.sample()
        self.assertEqual(len(self.state._SCALE_FACTOR_FILTER), 0)
        self.assertEqual(self.outputs, [])

    def test_large_jump_does_not_pollute_calibration(self):
        for _ in range(10):
            self.sample()
        self.sample(3.0)
        np.testing.assert_allclose(self.state._SCALE_FACTOR_FILTER, 2.0)

    def test_plane_above_camera_is_never_used(self):
        self.state.relative_height_estimate = lambda _: 0.065  # e.g. ceiling seen by RANSAC
        for _ in range(20):
            self.sample(1.5)
        self.assertEqual(len(self.state._SCALE_FACTOR_FILTER), 0)
        self.assertEqual(self.outputs, [])

    def test_small_ground_patch_is_ignored(self):
        for _ in range(20):
            self.sample(points=5)
        self.assertEqual(len(self.state._SCALE_FACTOR_FILTER), 0)

    def test_band_is_relative_to_scale(self):
        self.state.relative_height_estimate = lambda _: -0.05
        for _ in range(10):
            self.sample(1.0)   # scale 20
        self.sample(1.15)      # scale 23 (+15%) is within the 20% band
        self.assertAlmostEqual(self.state._SCALE_FACTOR_FILTER[-1], 23.0)
        self.sample(1.5)       # scale 30 (+50%) is outside it
        self.assertAlmostEqual(self.state._SCALE_FACTOR_FILTER[-1], 23.0)

    def test_sustained_consistent_disagreement_reseeds(self):
        for _ in range(10):
            self.sample(3.0)   # wrong lock: scale 6
        for _ in range(9):
            self.sample(1.0)   # scale 2, consistently
        np.testing.assert_allclose(self.state._SCALE_FACTOR_FILTER, 6.0)
        self.sample(1.0)
        np.testing.assert_allclose(self.state._SCALE_FACTOR_FILTER, 2.0)

    def test_scattered_outliers_do_not_reseed(self):
        for _ in range(10):
            self.sample()      # scale 2
        for i in range(30):
            self.sample(3.0 if i % 2 else 0.4)  # scales 6 and 0.8 disagree with each other
        np.testing.assert_allclose(self.state._SCALE_FACTOR_FILTER, 2.0)

    def test_accepted_sample_resets_disagreement_run(self):
        for _ in range(10):
            self.sample()
        for _ in range(9):
            self.sample(3.0)
        self.sample()          # agrees with the lock: run starts over
        for _ in range(9):
            self.sample(3.0)
        np.testing.assert_allclose(np.median(self.state._SCALE_FACTOR_FILTER), 2.0)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: test_scale_filter.py STAGED_WORKSPACE")
    unittest.main(argv=[sys.argv[0]], verbosity=2)
