"""autonomy/speed.py: 앞이 비어 있을 때 빠르게."""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autonomy.speed import boosted, fast_factor, straight_ahead  # noqa: E402


class SpeedTest(unittest.TestCase):
    def test_factor_ramps_between_from_and_full(self):
        self.assertEqual(fast_factor(1.0), 0.0)
        self.assertEqual(fast_factor(1.5), 0.0)
        self.assertAlmostEqual(fast_factor(2.25), 0.5)
        self.assertEqual(fast_factor(3.0), 1.0)
        self.assertEqual(fast_factor(math.inf), 1.0)
        self.assertEqual(fast_factor(None), 0.0)

    def test_boost_scales_toward_fast_and_never_past_it(self):
        self.assertAlmostEqual(boosted(0.2, 0.2, 0.4, 1.0), 0.4)
        self.assertAlmostEqual(boosted(0.2, 0.2, 0.4, 0.5), 0.3)
        self.assertAlmostEqual(boosted(0.1, 0.2, 0.4, 1.0), 0.2)      # 회전 중 줄인 명령은 같은 비율로
        self.assertEqual(boosted(0.2, 0.2, 0.0, 1.0), 0.2)           # fast 0 = 끔
        self.assertEqual(boosted(0.0, 0.2, 0.4, 1.0), 0.0)
        self.assertEqual(boosted(0.2, 0.2, 0.4, 0.0), 0.2)

    def test_straight_ahead_stops_at_a_bend(self):
        pts = [(1.0, 0.0), (2.5, 0.0), (2.5, 2.0)]
        self.assertAlmostEqual(straight_ahead(0.0, 0.0, 0.0, pts), 2.5)      # 세 번째 점은 90° 꺾임
        self.assertAlmostEqual(straight_ahead(0.0, 0.0, math.pi / 2, pts), 0.0)   # 기수가 옆을 봄
        self.assertAlmostEqual(straight_ahead(0.0, 0.0, 0.0, [(3.0, 0.5)]), math.hypot(3.0, 0.5))   # 9°: 곧은 길로 본다
        self.assertEqual(straight_ahead(0.0, 0.0, 0.0, []), 0.0)


if __name__ == "__main__":
    unittest.main()
