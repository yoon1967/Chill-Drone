"""rc 변환·가감속·전방 ToF 새 값 판정 시험 (표준 라이브러리만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rc_mixer  # noqa: E402
from rc_mixer import SlewLimiter, apply_deadzone, cmd_vel_to_rc, operator_axes_to_rc  # noqa: E402
from tof_fresh import FreshFilter  # noqa: E402


class CmdVelToRcTest(unittest.TestCase):
    def test_signs_follow_tello_rc(self):
        # x 앞 → rc 앞뒤 +, y 왼쪽 → rc 좌우 -, z 위 → +, yaw 반시계 → rc 회전 -
        self.assertEqual(cmd_vel_to_rc(0.1, 0, 0, 0), (0, 10, 0, 0))
        self.assertEqual(cmd_vel_to_rc(0, 0.1, 0, 0), (-10, 0, 0, 0))
        self.assertEqual(cmd_vel_to_rc(0, 0, 0.1, 0), (0, 0, 10, 0))
        self.assertEqual(cmd_vel_to_rc(0, 0, 0, 0.2), (0, 0, 0, -11))  # 0.2 × 57 = 11.4

    def test_limit(self):
        self.assertEqual(cmd_vel_to_rc(1.0, 0, -1.0, 5.0), (0, 30, -30, -30))

    def test_diagonal_keeps_direction(self):
        lr, fb, _, _ = cmd_vel_to_rc(0.8, -0.4, 0, 0)  # 앞 2 : 오른쪽 1
        self.assertLessEqual(math.hypot(lr, fb), 30.5)
        self.assertAlmostEqual(fb / lr, 2.0, delta=0.1)
        # 따로 잘랐다면 (30, 30)이 되어 45°로 바뀐다
        self.assertNotEqual((lr, fb), (30, 30))

    def test_rejects_non_finite(self):
        self.assertIsNone(cmd_vel_to_rc(float("nan"), 0, 0, 0))
        self.assertIsNone(cmd_vel_to_rc(None, 0, 0, 0))
        self.assertIsNone(cmd_vel_to_rc(0, float("inf"), 0, 0))

    def test_max_speed(self):
        self.assertAlmostEqual(rc_mixer.max_speed_mps(), 0.3)


class OperatorAxesTest(unittest.TestCase):
    def test_deadzone_and_rescale(self):
        self.assertEqual(apply_deadzone([0.1, -0.15, 1.0, -1.0]), (0.0, 0.0, 1.0, -1.0))
        self.assertAlmostEqual(apply_deadzone([0.575, 0, 0, 0])[0], 0.5)

    def test_full_stick_is_rc_limit(self):
        self.assertEqual(operator_axes_to_rc([1, -1, 0.05, 2]), (30, -30, 0, 30))

    def test_bad_values_are_zero(self):
        self.assertEqual(operator_axes_to_rc([float("nan"), None, "x", 0]), (0, 0, 0, 0))


class SlewLimiterTest(unittest.TestCase):
    def test_ramps_up_and_stops_at_once(self):
        s = SlewLimiter(150)
        self.assertEqual(s.step((0, 30, 0, 0), 0.0), (0, 0, 0, 0))
        self.assertEqual(s.step((0, 30, 0, 0), 0.05), (0, 8, 0, 0))   # 7.5 → 8
        s.step((0, 30, 0, 0), 0.10)
        s.step((0, 30, 0, 0), 0.15)
        self.assertEqual(s.step((0, 30, 0, 0), 0.20), (0, 30, 0, 0))
        self.assertEqual(s.step((0, 0, 0, 0), 0.25), (0, 0, 0, 0))    # 손 떼면 바로 0
        s.step((0, 30, 0, 0), 0.30)

    def test_reverse_goes_through_zero(self):
        s = SlewLimiter(150)
        s.last, s._last_t = (0, 30, 0, 0), 0.0
        self.assertEqual(s.step((0, -30, 0, 0), 0.05), (0, 0, 0, 0))
        self.assertEqual(s.step((0, -30, 0, 0), 0.10), (0, -8, 0, 0))

    def test_decrease_is_immediate(self):
        s = SlewLimiter(150)
        s.last, s._last_t = (20, 30, 0, 0), 0.0
        self.assertEqual(s.step((5, 10, 0, 0), 0.05), (5, 10, 0, 0))

    def test_zero_rate_disables(self):
        s = SlewLimiter(0)
        self.assertEqual(s.step((0, 30, 0, 0), 0.0), (0, 30, 0, 0))


class FreshFilterTest(unittest.TestCase):
    def reading(self, m):
        return {"front_tof_m": m, "valid": True, "reason": None}

    def test_same_value_not_sent_twice_within_hold(self):
        f = FreshFilter(0.2)
        self.assertEqual(f.update(self.reading(0.5), 0.0), 1)
        self.assertIsNone(f.update(self.reading(0.5), 0.1))   # 10Hz 조회, 센서는 아직 그대로
        self.assertEqual(f.update(self.reading(0.48), 0.15), 2)  # 바뀌면 바로

    def test_still_obstacle_keeps_coming(self):
        f = FreshFilter(0.2)
        seqs = [f.update(self.reading(0.3), t / 10) for t in range(11)]  # 1초 동안 같은 값
        self.assertEqual(len([s for s in seqs if s]), 6)  # 0, 0.2, ..., 1.0 → 약 5Hz

    def test_invalid_counts_as_value(self):
        f = FreshFilter(0.2)
        f.update(self.reading(0.3), 0.0)
        self.assertEqual(f.update({"front_tof_m": None, "valid": False, "reason": "out_of_range"}, 0.05), 2)


if __name__ == "__main__":
    unittest.main()
