"""카메라 지도로 위치 바로잡기(4단계) 검사. 실행: python -m unittest discover -s tests"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from autonomy.camera_map import CameraMap, CameraMapParams  # noqa: E402
from autonomy.scan_match import Correction, ScanMatcher, depth_points  # noqa: E402
from test_camera_map import BOUNDS, HFOV, _box, _world, render_depth  # noqa: E402

PARAMS = CameraMapParams(stride=2, hit_far=0.85, hit_max_range=4.0)   # 정답 깊이를 방향마다 한 장면만 넣는 시험 (test_camera_map 참고)


def _reference(world, poses, params=PARAMS):
    """정답 위치에서 본 깊이로 만든 평면 막힘 지도와 본 칸 (set_reference 인자 순서)."""
    cmap = CameraMap(BOUNDS, params)
    for x, y, yaw in poses:
        cmap.integrate_depth(render_depth(world, (x, y, 1.0), yaw), (x, y, 1.0), yaw, HFOV)
    return (cmap.occupied()[:, :, 3:18].any(axis=2), cmap.origin, params.res,
            cmap.seen_band()[:, :, 3:18].any(axis=2))


class ScanMatchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = _world(_box(4.5, 1.0, 0.6, 0.6, 0.6, 1.2), _box(2.0, 3.2, 0.5, 0.8, 0.5, 1.0))
        cls.ref = _reference(cls.world, [(3.0, 2.0, a) for a in np.arange(0, 2 * math.pi, math.pi / 4)])

    def _match(self, true_pose, est_pose):
        x, y, yaw = true_pose
        depth = render_depth(self.world, (x, y, 1.0), yaw)
        ex, ey, eyaw = est_pose
        pts = depth_points(depth, (ex, ey, 1.0), eyaw, HFOV, stride=2)
        m = ScanMatcher()
        m.set_reference(*self.ref)
        return m.match(pts, (ex, ey))

    def test_recovers_offset(self):
        true = (2.6, 1.8, 0.3)
        est = (2.6 + 0.2, 1.8 - 0.15, 0.3 + math.radians(1))
        res = self._match(true, est)
        self.assertIsNotNone(res)
        dx, dy, dyaw, _ = res
        corr = Correction()
        corr.nudge(dx, dy, dyaw, est[:2])
        x, y, _, yaw = corr.apply((*est[:2], 1.0, est[2]))
        self.assertLess(math.hypot(x - true[0], y - true[1]), 0.07, f"보정 뒤 {x:.2f}, {y:.2f}")
        self.assertLessEqual(abs(yaw - true[2]), math.radians(1.0) + 1e-9, "방향은 나빠지지 않는다")

    def test_recovers_yaw(self):
        # Tello yaw는 분당 약 1°씩 틀어진다. 방향만 3° 틀렸을 때
        res = self._match((2.6, 1.8, 0.3), (2.6, 1.8, 0.3 + math.radians(3)))
        self.assertIsNotNone(res)
        self.assertAlmostEqual(math.degrees(res[2]), -3.0, delta=1.0)

    def test_wide_search_for_large_offset(self):
        # 0.45 m 틀어지면 보통 범위(±0.3 m)로는 끝에 걸려 버리고, 출발점 맞추기 범위(±0.6 m)로는 찾는다
        from autonomy.scan_match import ScanMatchParams
        true, est = (2.6, 1.8, 0.3), (2.6 + 0.45, 1.8, 0.3)
        depth = render_depth(self.world, (true[0], true[1], 1.0), true[2])
        pts = depth_points(depth, (est[0], est[1], 1.0), est[2], HFOV, stride=2)
        narrow, wide = ScanMatcher(), ScanMatcher(ScanMatchParams(search_xy=0.6, step_xy=0.1, max_spread=0.15))
        narrow.set_reference(*self.ref)
        wide.set_reference(*self.ref)
        self.assertIsNone(narrow.match(pts, est[:2]))
        res = wide.match(pts, est[:2])
        self.assertIsNotNone(res)
        self.assertAlmostEqual(res[0], -0.45, delta=0.1)

    def test_no_correction_when_already_right(self):
        res = self._match((2.6, 1.8, 0.3), (2.6, 1.8, 0.3))
        if res is not None:   # 10 cm 칸이라 반 칸 정도는 흔들린다
            self.assertLess(math.hypot(res[0], res[1]), 0.08)

    def test_refuses_without_overlap(self):
        m = ScanMatcher()
        m.set_reference(np.zeros_like(self.ref[0]), *self.ref[1:3])
        depth = render_depth(self.world, (3.0, 2.0, 1.0), 0.0)
        self.assertIsNone(m.match(depth_points(depth, (3.0, 2.0, 1.0), 0.0, HFOV, stride=2), (3.0, 2.0)))

    def _corridor(self, ref_poses):
        # 긴 복도(y 1.5–2.5)를 비스듬히(옆벽이 보이게) 보며, 추정이 x로 0.2 m, y로 0.1 m 틀렸다
        world = _world(_box(3.0, 1.45, 1.25, 6.0, 0.1, 2.5), _box(3.0, 2.55, 1.25, 6.0, 0.1, 2.5))
        m = ScanMatcher()
        m.set_reference(*_reference(world, ref_poses, CameraMapParams(stride=2, max_range=2.0)))
        depth = render_depth(world, (3.0, 2.0, 1.0), 0.6)
        depth[depth > 2.0] = np.inf
        return m.match(depth_points(depth, (3.2, 2.1, 1.0), 0.6, HFOV, stride=2, rng=(0.3, 2.0)), (3.2, 2.1))

    def test_corridor_does_not_slide_along(self):
        # 복도 방향(x)으로 틀린 것은 알 수 없으니 그쪽으로는 고치지 않고, 가로지르는 쪽(y)만 고친다
        poses = [(x, 2.0, a) for x in (1.5, 2.5, 3.5, 4.5) for a in (0.6, -0.6, math.pi - 0.6, math.pi + 0.6)]
        res = self._corridor(poses)
        self.assertIsNotNone(res)
        dx, dy, _, _ = res
        # 가로지르는 쪽은 바른 쪽으로 고친다 (10 cm 칸이라 한 번에 다 고치지는 못한다. 비행 중에는 되풀이한다)
        self.assertTrue(-0.15 < dy < -0.03, f"dy {dy:.3f}")
        self.assertLess(abs(dx), 0.1, "복도 방향으로 멋대로 끌려가지 않는다")

    def test_partial_reference_does_not_drag(self):
        # 예전 지도에 벽 일부만 있으면 점이 빽빽한 쪽을 그 구간에 겹치려고 끌려가기 쉽다
        res = self._corridor([(3.0, 2.0, 0.0), (3.0, 2.0, math.pi)])
        if res is not None:
            self.assertLess(abs(res[0]), 0.15)


class DriftFilterTest(unittest.TestCase):
    def test_learns_speed_scale_and_heading(self):
        # 속도 적분 위치가 출발점 (1, 2)을 중심으로 1.06배 늘고 3° 돌아간 꼴로 틀어졌다. 맞추기가 (잡음 섞인) 그때그때의
        # 보정을 내면, 거르개는 배율·방향을 배워 다른 자리에서도 맞는 위치를 내야 한다
        from autonomy.scan_match import Correction, LocalizerParams, MapLocalizer

        true = Correction(scale=1 / 1.06, yaw=math.radians(-3), px=1.0, py=2.0)
        loc = MapLocalizer(LocalizerParams(model="drift"))
        loc.correction.set_pivot(1.0, 2.0)
        loc.pivot_set = True
        rng = np.random.default_rng(0)
        for i in range(200):
            raw = (1.0 + 4 * math.cos(i * 0.3), 2.0 + 3 * math.sin(i * 0.2), 1.0, 0.0)
            want, have = true.apply(raw), loc.correction.apply(raw)
            z = (want[0] - have[0] + rng.normal(0, 0.1), want[1] - have[1] + rng.normal(0, 0.1),
                 want[3] - have[3] + rng.normal(0, 0.02))
            loc._drift_update(0.5 * i, (*z, {"weak": [], "yaw_known": True}), raw)
        far = (6.0, -1.0, 1.0, 0.0)                 # 지나가지 않은 자리
        a, b = true.apply(far), loc.correction.apply(far)
        self.assertLess(math.hypot(a[0] - b[0], a[1] - b[1]), 0.1)
        self.assertAlmostEqual(loc.correction.scale, 1 / 1.06, delta=0.01)
        self.assertAlmostEqual(math.degrees(loc.correction.yaw), -3.0, delta=0.5)

    def test_rejects_wild_proposal(self):
        from autonomy.scan_match import LocalizerParams, MapLocalizer

        loc = MapLocalizer(LocalizerParams(model="drift"))
        loc.correction.set_pivot(0.0, 0.0)
        ok = loc._drift_update(0.0, (1.5, 0.0, 0.0, {"weak": [], "yaw_known": True}), (1.0, 0.0, 1.0, 0.0))
        self.assertFalse(ok, "1.5 m 보정은 처음 불확실성(약 0.1 m)에 비해 엉뚱하다")
        self.assertEqual(loc.rejected, 1)


class CorrectionTest(unittest.TestCase):
    def test_set_pivot_keeps_mapping(self):
        corr = Correction(tx=0.3, ty=-0.1, yaw=0.05, scale=1.04, px=1.0, py=2.0)
        before = corr.apply((4.0, -1.0, 1.0, 0.2))
        corr.set_pivot(-2.0, 5.0)
        after = corr.apply((4.0, -1.0, 1.0, 0.2))
        for a, b in zip(before, after):
            self.assertAlmostEqual(a, b)

    def test_nudge_composes_about_center(self):
        corr = Correction()
        corr.nudge(0.1, -0.2, math.radians(5), (1.0, 1.0))
        x, y, _, yaw = corr.apply((1.0, 1.0, 1.0, 0.0))     # 회전 중심은 이동만 받는다
        self.assertAlmostEqual(x, 1.1)
        self.assertAlmostEqual(y, 0.8)
        self.assertAlmostEqual(yaw, math.radians(5))
        corr.nudge(-0.1, 0.2, -math.radians(5), (1.1, 0.8))  # 되돌리면 원래대로
        x, y, _, yaw = corr.apply((3.0, -2.0, 1.0, 0.4))
        self.assertAlmostEqual(x, 3.0)
        self.assertAlmostEqual(y, -2.0)
        self.assertAlmostEqual(yaw, 0.4)


if __name__ == "__main__":
    unittest.main()
