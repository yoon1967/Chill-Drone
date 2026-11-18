"""autonomy/shape.py: 깊이 장면에서 앞이 기둥인지 벽인지."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autonomy.shape import (front_points, front_shape, rotate_scene, scene_ranges, see_through,  # noqa: E402
                            segment_points, shape_points)

HFOV = 1.4416


def synth_depth(objects, w=240, h=180, hfov=HFOV, cam_z=1.0, far=6.0):
    """objects: (가로 위치 y[m, 왼쪽 +], 앞 거리 x[m], 폭[m], 아래 z, 위 z) 상자들을 보는 깊이 영상(광축 거리)."""
    f = (w / 2) / math.tan(hfov / 2)
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    left = -(us - w / 2) / f
    up = -(vs - h / 2) / f
    depth = np.full((h, w), far)
    for oy, ox, ow, z0, z1 in objects:
        lat, z = ox * left, cam_z + ox * up
        hit = (np.abs(lat - oy) <= ow / 2) & (z >= z0) & (z <= z1)
        depth[hit] = np.minimum(depth[hit], ox)
    floor = up < -1e-6
    depth = np.minimum(depth, np.where(floor, -cam_z / np.where(floor, up, -1.0), far))
    return depth.astype(np.float32)


class SceneTest(unittest.TestCase):
    def test_pillar_ahead_is_narrow_with_free_sides(self):
        c, r = scene_ranges(synth_depth([(0.0, 1.5, 0.4, 0.0, 1.8)]), HFOV, 1.0, 1.0)
        s = front_shape(c, r, 1.5)
        self.assertEqual(s.kind, "pillar")
        self.assertLess(abs(s.width - 0.4), 0.2)
        self.assertGreater(min(s.left_open, s.right_open), 3)
        self.assertIsNotNone(s.pass_angle())
        self.assertIn("기둥", s.text())

    def test_wall_fills_the_view(self):
        c, r = scene_ranges(synth_depth([(0.0, 1.5, 8.0, 0.0, 2.5)]), HFOV, 1.0, 1.0)
        s = front_shape(c, r, 1.5)
        self.assertEqual(s.kind, "wall")
        self.assertTrue(s.edge_left and s.edge_right)
        self.assertIsNone(s.free_side)
        self.assertIn("벽", s.text())

    def test_wide_panel_is_wall_even_if_sides_visible(self):
        c, r = scene_ranges(synth_depth([(0.0, 2.0, 2.0, 0.0, 2.0)]), HFOV, 1.0, 1.0)
        s = front_shape(c, r, 2.0)
        self.assertEqual(s.kind, "wall")
        self.assertGreater(s.width, 1.5)

    def test_pillar_with_wall_close_on_one_side(self):
        # 기둥에 벽이 맞닿아 있으면 한 덩어리(벽), 1.1 m 틈이 있으면 기둥이고 더 넓게 빈 쪽은 왼쪽
        c, r = scene_ranges(synth_depth([(0.0, 1.5, 0.4, 0.0, 1.8), (-1.45, 1.5, 2.5, 0.0, 2.5)]), HFOV, 1.0, 1.0)
        self.assertEqual(front_shape(c, r, 1.5).kind, "wall")
        c, r = scene_ranges(synth_depth([(0.0, 1.5, 0.4, 0.0, 1.8), (-1.6, 1.5, 0.6, 0.0, 2.5)]), HFOV, 1.0, 1.0)
        s = front_shape(c, r, 1.5)
        self.assertEqual((s.kind, s.free_side), ("pillar", "left"))
        self.assertGreater(s.pass_angle(), s.a_left)

    def test_clear_and_low_object(self):
        c, r = scene_ranges(synth_depth([]), HFOV, 1.0, 1.0)
        self.assertEqual(front_shape(c, r, 0.8).kind, "clear")
        c, r = scene_ranges(synth_depth([(0.0, 1.5, 3.0, 0.0, 0.5)]), HFOV, 1.0, 1.0)   # 낮은 탁자: 띠 밖
        self.assertEqual(front_shape(c, r, 1.5).kind, "clear")

    def test_rotate_and_points(self):
        np.testing.assert_allclose(rotate_scene([-0.5, 0.0, 0.5], 1.0, 0.8), [-0.3, 0.2, 0.7], atol=1e-9)
        pts = front_points((0.0, 0.0, 1.0), 0.0, 2.0, -0.2, 0.2, 0.7, 1.3, 0.1)
        self.assertTrue(np.all(np.abs(pts[:, 0] - 2.0) < 1e-9))
        self.assertLess(abs(pts[:, 1].min() + 2 * math.tan(0.2)), 0.06)
        self.assertAlmostEqual(pts[:, 2].min(), 0.7)
        seg = segment_points((1.0, 1.0), (1.0, 2.0), 0.7, 1.3, 0.1, extra=0.3)
        self.assertAlmostEqual(seg[:, 1].min(), 0.7, places=6)
        self.assertAlmostEqual(seg[:, 1].max(), 2.3, places=6)

    def test_oblique_corridor_wall_is_painted_on_the_wall(self):
        """11/03 Gazebo corridor_L: 폭 1.4 m 복도(서쪽 벽 x=0.7) 안 (1.27, 1.29)에서 기수 129°로 서쪽 벽을 51° 비스듬히 보고,
        화면 오른쪽에는 복도 끝 벽(y=2.4)이 보여 양쪽 화면 끝까지 막힘 → 벽. 기수에 수직인 직선(front_points)은 끝이 복도 안으로
        0.5 m 넘게 들어오고, 면을 따라 찍는 shape_points는 두 벽 위에만 남는다."""
        hfov, bins = 1.4416, 27
        origin, yaw = (1.27, 1.29, 1.0), math.radians(129)
        centers = -hfov / 2 + hfov / bins * (np.arange(bins) + 0.5)
        truth = []
        for c in centers:   # 각 열의 광선이 서쪽 벽(x=0.7)·끝 벽(y=2.4) 중 먼저 닿는 곳까지의 수평 거리
            dx, dy = math.cos(yaw + c), math.sin(yaw + c)
            tx = (0.7 - origin[0]) / dx if dx < -1e-6 else 9.0
            ty = (2.4 - origin[1]) / dy if dy > 1e-6 else 9.0
            truth.append(min(tx, ty, 4.0))
        truth = np.array(truth)
        ranges = truth * 0.8   # 단안 깊이가 20% 짧게 읽힌 장면; 정면 ToF는 원뿔(±12.5°) 안에서 가장 가까운 참값
        cone = math.radians(12.5)
        dist = float(truth[np.abs(centers) <= cone].min())
        shape = front_shape(centers, ranges, dist)
        self.assertEqual(shape.kind, "wall")
        pts = shape_points(origin, yaw, centers, ranges, shape, dist, 0.65, 1.35, 0.1, extra=0.3, cone=cone)
        off = np.minimum(np.abs(pts[:, 0] - 0.7), np.abs(pts[:, 1] - 2.4))   # 두 벽 면에서 벗어난 거리
        self.assertLess(off.max(), 0.1, f"벽에서 벗어남: 최대 {off.max():.2f} m")
        west = pts[np.abs(pts[:, 0] - 0.7) < 0.1]
        self.assertGreater(west[:, 1].max() - west[:, 1].min(), 0.8)   # 서쪽 벽을 따라 0.8 m 넘게 이어진다
        self.assertAlmostEqual(pts[:, 2].min(), 0.65)
        old = front_points(origin, yaw, dist, shape.a_right, shape.a_left, 0.65, 1.35, 0.1, extra=0.3)
        off_old = np.minimum(np.abs(old[:, 0] - 0.7), np.abs(old[:, 1] - 2.4))
        inside = (off_old > 0.15) & (old[:, 0] > 0.7) & (old[:, 1] < 2.4)
        self.assertTrue(inside.any())                 # 예전 방식(기수에 수직인 직선)은 두 벽에서 0.15 m 넘게 떨어진 복도 안까지 들어온다
        # 열마다 깊이가 제멋대로 8–36% 짧게 읽혀도(기록 재생에서 본 사분위) 직선 맞춤은 벽에서 0.2 m 안에 머문다
        rng = np.random.default_rng(3)
        noisy = truth * rng.uniform(0.64, 0.92, truth.shape)
        shape2 = front_shape(centers, noisy, dist)
        pts2 = shape_points(origin, yaw, centers, noisy, shape2, dist, 0.65, 1.35, 0.1, extra=0.3, cone=cone)
        off2 = np.minimum(np.abs(pts2[:, 0] - 0.7), np.abs(pts2[:, 1] - 2.4))
        self.assertLess(off2.max(), 0.2, f"흔들린 깊이에서 벽에서 벗어남: 최대 {off2.max():.2f} m")

    def test_see_through_only_when_every_cone_column_is_far(self):
        # 11/04 실기체 62 s: 흰 벽을 ToF가 0.40 m에서 재는데 원뿔 안 깊이 열은 1.06–1.43 m (꿰뚫어 봄)
        cone = math.radians(12.5)
        centers = np.radians(np.linspace(-26, 26, 27))
        ranges = np.full(27, 1.2)
        self.assertAlmostEqual(see_through(centers, ranges, 0.40, cone), 1.2)
        chair = ranges.copy()
        chair[14] = 0.45                       # 원뿔 한쪽 열에만 걸린 등받이를 깊이도 봤다
        self.assertIsNone(see_through(centers, chair, 0.40, cone))
        self.assertIsNone(see_through(centers, ranges, 0.90, cone), "0.3 m 차이는 단안 깊이 오차 안")
        self.assertIsNone(see_through(centers, ranges, math.inf, cone), "ToF가 못 잡으면 판단하지 않는다")


if __name__ == "__main__":
    unittest.main()
