"""오래된 위치·전방 ToF로 움직이지 않기 (autonomy/fresh.py, 11/04 외부 리뷰)."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autonomy import fresh  # noqa: E402

GO = (0.3, 0.05, 0.1, 0.2)


class GuardTest(unittest.TestCase):
    def test_fresh_inputs_pass(self):
        self.assertEqual(fresh.guard(GO, 0.1, 0.2), (GO, None))
        self.assertEqual(fresh.guard(GO, 0.1, None), (GO, None), "ToF를 받은 적 없으면(ToF 없는 시뮬레이션) 보지 않는다")

    def test_old_pose_stops_everything(self):
        # 리뷰 재현: 1초 전 위치로 전진 명령을 만들었다
        cmd, why = fresh.guard(GO, 1.0, 0.1)
        self.assertEqual(cmd, (0.0, 0.0, 0.0, 0.0))
        self.assertIn("위치", why)

    def test_old_tof_blocks_only_forward(self):
        # 리뷰 재현: 5초 전 ToF로 전진 명령을 만들었다. 옆·오르내림·회전과 뒤로는 그대로 (ToF는 앞만 본다)
        cmd, why = fresh.guard(GO, 0.1, 5.0)
        self.assertEqual(cmd, (0.0, 0.05, 0.1, 0.2))
        self.assertIn("ToF", why)
        back = (-0.1, 0.0, 0.0, 0.0)
        self.assertEqual(fresh.guard(back, 0.1, 5.0), (back, None))
        self.assertEqual(fresh.guard(GO, 0.1, 0.9)[1], None, "응답 한 번 놓친 정도(약 0.9초)는 그대로")


class LostTest(unittest.TestCase):
    def test_lands_only_after_long_gap_and_enough_steps(self):
        self.assertIsNone(fresh.lost(0.3, 3, 0.3, 3))
        self.assertIn("위치", fresh.lost(2.5, 40, 0.3, 3))
        self.assertIsNone(fresh.lost(2.5, 2, 0.3, 3), "긴 계산 직후(제어 주기 2번) 못 읽은 메시지는 '안 옴'이 아니다")
        self.assertIn("ToF", fresh.lost(0.1, 2, 3.5, 70))
        self.assertIsNone(fresh.lost(0.1, 2, None, 999), "ToF가 없는 시뮬레이션")


if __name__ == "__main__":
    unittest.main()
