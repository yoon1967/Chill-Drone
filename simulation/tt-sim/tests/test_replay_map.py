"""비행 기록 다시 돌리기(tools/replay_map.py) 검사. 실행: python -m unittest discover -s tests"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from autonomy.camera_map import CameraMapParams  # noqa: E402
from autonomy.scan_match import LocalizerParams  # noqa: E402
from replay_map import replay, set_param  # noqa: E402
from test_camera_map import BOUNDS, HFOV, _box, _world, render_depth  # noqa: E402


def _flight(drift=0.02):
    """방 안을 x로 왕복하며 본 깊이 영상. 기록된 추정 위치는 1초마다 drift m씩 y로 틀어진다."""
    world = _world(_box(4.5, 1.0, 0.6, 0.6, 0.6, 1.2))
    ts, true, raw, depth = [], [], [], []
    for i in range(20):
        t = float(i)
        x = 1.5 + 0.15 * (i if i < 10 else 19 - i)
        yaw = 0.0 if i < 10 else math.pi
        ts.append(t)
        true.append((x, 2.0, 1.0, yaw))
        raw.append((x, 2.0 + drift * i, 1.0, yaw))
        depth.append(render_depth(world, (x + 0.05 * math.cos(yaw), 2.0, 1.0), yaw).astype(np.float32))
    rec = {"t": np.array(ts), "pose": np.array(raw), "depth": np.stack(depth),
           "tof_t": np.zeros(0), "tof_pose": np.zeros((0, 4)), "tof_range": np.zeros(0),
           "meta": {"hfov": HFOV, "sensor_x": 0.05, "tof_max": 1.2, "bounds": BOUNDS, "height": 1.0}}
    a = np.array(true)
    truth = {"t": np.array(ts), "x": a[:, 0], "y": a[:, 1], "z": a[:, 2], "yaw": np.unwrap(a[:, 3])}
    return rec, truth


class ReplayTest(unittest.TestCase):
    def test_without_localize_error_is_recorded_drift(self):
        rec, truth = _flight()
        res = replay(rec, truth, map_params=CameraMapParams(stride=2))
        np.testing.assert_allclose(res["err"], res["err_raw"])
        self.assertAlmostEqual(float(res["err"][-1]), 0.02 * 19, places=6)
        self.assertTrue(res["cmap"].occupied().any(), "지도가 만들어진다")

    def test_truth_pose_has_no_error(self):
        rec, truth = _flight()
        res = replay(rec, truth, pose_source="truth", map_params=CameraMapParams(stride=2))
        self.assertLess(float(res["err"].max()), 1e-6)

    def test_localizer_runs_and_counts(self):
        rec, truth = _flight()
        lp = LocalizerParams(ref_age=5.0, snapshot=1.0, period=0.5)
        res = replay(rec, truth, localize=True, loc_params=lp, map_params=CameraMapParams(stride=2))
        self.assertGreater(res["localizer"].tries, 0)
        self.assertEqual(len(res["corr"]), len(rec["t"]))

    def test_set_param_nested_and_tuple(self):
        lp = LocalizerParams()
        set_param(lp, "gain", "0.3")
        set_param(lp, "match.sigma", "0.15")
        set_param(lp, "rng", "1.0,3.0")
        self.assertEqual((lp.gain, lp.match.sigma, lp.rng), (0.3, 0.15, (1.0, 3.0)))


if __name__ == "__main__":
    unittest.main()
