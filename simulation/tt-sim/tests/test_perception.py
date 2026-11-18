"""탐지 보조 함수 검사 (YOLO 없이 돈다). 실행: python -m unittest discover -s tests"""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from perception import Detection, EventThrottle, camera_matrix, project  # noqa: E402

W, H, HFOV = 960, 720, 1.4416


class ProjectionTest(unittest.TestCase):
    def test_focal_length_matches_fov(self):
        f, cx, cy = camera_matrix(W, H, HFOV)
        self.assertAlmostEqual(2 * math.atan(cx / f), HFOV)
        self.assertEqual((cx, cy), (480, 360))

    def test_point_straight_ahead_is_image_center(self):
        u, v = project((0, 0, 1), 0.0, (3, 0, 1), W, H, HFOV)
        self.assertAlmostEqual(u, 480)
        self.assertAlmostEqual(v, 360)

    def test_left_and_up_move_pixel_left_and_up(self):
        u, v = project((0, 0, 1), 0.0, (3, 0.5, 1.5), W, H, HFOV)
        self.assertLess(u, 480)
        self.assertLess(v, 360)

    def test_yaw_is_respected_and_behind_is_none(self):
        u, v = project((0, 0, 1), math.pi / 2, (0, 4, 1), W, H, HFOV)   # 북쪽을 보고 북쪽 점
        self.assertAlmostEqual(u, 480)
        self.assertIsNone(project((0, 0, 1), 0.0, (-2, 0, 1), W, H, HFOV))

    def test_edge_of_fov_hits_image_edge(self):
        x = 2.0
        y = x * math.tan(HFOV / 2)
        u, _ = project((0, 0, 1), 0.0, (x, y, 1), W, H, HFOV)
        self.assertAlmostEqual(u, 0.0, places=6)


class DetectionTest(unittest.TestCase):
    def test_contains_with_margin(self):
        d = Detection(0, "person", 0.9, 100, 50, 200, 300)
        self.assertTrue(d.contains(150, 100))
        self.assertFalse(d.contains(210, 100))
        self.assertTrue(d.contains(210, 100, margin=15))

    def test_to_dict_rounds(self):
        d = Detection(0, "person", 0.876543, 1.234, 2.345, 3.456, 4.567).to_dict()
        self.assertEqual(d["conf"], 0.877)
        self.assertEqual(d["x1"], 1.2)
        self.assertNotIn("polygons", d)                      # 상자만 낸 모델
        d = Detection(0, "person", 0.9, 1, 2, 3, 4, polygons=[[(1.26, 2.0), (3.0, 2.0), (3.0, 4.0)]],
                      mask=object()).to_dict()
        self.assertEqual(d["polygons"], [[[1.3, 2.0], [3.0, 2.0], [3.0, 4.0]]])   # 윤곽은 반올림해 내고
        self.assertNotIn("mask", d)                                                 # 마스크는 JSON에 넣지 않는다


class ThrottleTest(unittest.TestCase):
    def test_suppresses_repeats_within_cooldown(self):
        t = EventThrottle(cooldown=2.0)
        self.assertTrue(t.ready(0, 10.0))
        self.assertFalse(t.ready(0, 11.0))
        self.assertTrue(t.ready(1, 11.0))   # 다른 종류는 따로 센다
        self.assertTrue(t.ready(0, 12.1))


class TofDepthScaleTest(unittest.TestCase):
    """11/04 12:57 실기체: 깊이 노드의 깊이(배율 0.85)는 ToF와 0.99배로 맞았는데 바닥 배율 1.30이 곱해져 30% 멀어졌다."""

    def test_starts_at_one_and_holds_until_enough_samples(self):
        from perception.depth import TofDepthScale
        s = TofDepthScale()
        self.assertEqual(s.value(0.0), 1.0)
        for i in range(4):                        # 4개로는 아직 바꾸지 않는다
            s.add(0.80, 1.00, i * 1.0)
        self.assertEqual(s.value(4.0), 1.0)
        s.add(0.80, 1.00, 4.0)
        # 5개로는 1/3만 따라간다 (0.80^(5/15) = 0.93): 11/11 09:32 실기체는 처음 5개로 1.00 → 0.70이 됐다
        self.assertAlmostEqual(s.value(4.0), 0.80 ** (5 / 15), places=4)
        for i in range(5, 40):                    # 같은 비가 계속 나오면 그 값에 다가간다 (40개면 0.80^0.8 = 0.84)
            s.add(0.80, 1.00, i * 1.0)
        self.assertAlmostEqual(s.value(40.0), 0.80 ** (40 / 50), places=4)
        self.assertLess(s.value(40.0), 0.85)

    def test_few_wild_samples_do_not_swing_the_scale(self):
        # 실기체 가방 6개: 표본 하나하나가 ±22%씩 흩어진다. 예전(60초 중앙값·0.7–1.4)은 한 비행에서 0.70 ↔ 1.29로 튀었다
        from perception.depth import TofDepthScale
        s = TofDepthScale()
        for i, (tof, depth) in enumerate([(0.70, 1.0), (0.72, 1.0), (0.68, 1.0), (0.75, 1.0), (0.66, 1.0)]):
            s.add(tof, depth, float(i))
        self.assertGreater(s.value(5.0), 0.87, "표본 5개가 모두 0.7 근처여도 0.89쯤까지만")
        for i in range(6):                        # 2분 뒤 다른 면에서 1.3 근처 6개: 창이 300초라 앞의 것과 함께 본다
            s.add(1.30, 1.0, 120.0 + i)
        v = s.value(126.0)
        self.assertGreater(v, 0.95)
        self.assertLess(v, 1.2, "1.3으로 튀지 않는다 (11개 중앙값 1.30 → 1.30^(11/21) = 1.15)")
        old = TofDepthScale(window=60.0, limits=(0.7, 1.4), shrink=0.0)   # 예전 설정이면 그대로 튄다
        for i in range(5):
            old.add(0.70, 1.0, float(i))
        self.assertAlmostEqual(old.value(5.0), 0.70)
        for i in range(6):
            old.add(1.30, 1.0, 120.0 + i)
        self.assertAlmostEqual(old.value(126.0), 1.30)

    def test_one_sample_per_gap_and_rejects_outliers(self):
        from perception.depth import TofDepthScale
        s = TofDepthScale()
        self.assertTrue(s.add(0.5, 0.5, 0.0))
        self.assertFalse(s.add(0.5, 0.5, 0.2), "초당 10번 오는 ToF를 다 받으면 한 면이 배율을 정한다")
        self.assertFalse(s.add(0.58, 1.10, 1.0), "비 0.53: 유리·흰 벽을 꿰뚫어 본 것")
        self.assertFalse(s.add(1.0, 0.5, 2.0), "비 2.0: 다른 물체")
        self.assertFalse(s.add(float("inf"), 1.0, 3.0))
        self.assertEqual(s.used, 1)

    def test_median_is_clamped_and_kept_after_samples_expire(self):
        from perception.depth import TofDepthScale
        s = TofDepthScale(window=60.0, shrink=0.0)
        for i in range(8):
            s.add(0.62, 1.00, i * 1.0)            # 비 0.62 → 하한 0.8
        self.assertAlmostEqual(s.value(8.0), 0.8)
        self.assertAlmostEqual(s.value(200.0), 0.8, msg="표본이 창 밖으로 나가도 지난 값을 쓴다")
        for i in range(6):
            s.add(1.00, 0.99, 300.0 + i)          # 12:57처럼 깊이와 ToF가 맞으면 1.0 근처로 돌아온다
        self.assertAlmostEqual(s.value(306.0), 1.0 / 0.99, places=3)
        hi = TofDepthScale(shrink=0.0)
        for i in range(8):
            hi.add(1.50, 1.00, i * 1.0)           # 비 1.5 → 상한 1.25
        self.assertAlmostEqual(hi.value(8.0), 1.25)


if __name__ == "__main__":
    unittest.main()
