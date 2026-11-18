"""자율 비행 높이 지켜보기(height_watch.py): 아래가 가까운가, 자율 비행 높이 추정이 잰 높이와 어긋나는가(알리기만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_height_watch.py" -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from height_watch import GAP_S, LOW_S, HeightWatch  # noqa: E402

FLOOR = 36.22   # 바닥 기압(m, Tello baro)


class HeightWatchTest(unittest.TestCase):
    def setUp(self):
        self.w = HeightWatch()
        self.t = 0.0
        self.events = []

    def fly(self, seconds, real, est, tof=None, noise=0.0, flying=True, short=False):
        """0.1초마다 상태 하나: 실제 높이 real(기압·하방), 자율 비행 추정 est. tof를 주면 하방은 그것(가구 위 등)."""
        for i in range(int(round(seconds / 0.1))):
            self.t += 0.1
            wobble = noise if i % 2 else -noise   # 기압은 ±0.1 m쯤 흔들린다
            self.events += self.w.update(self.t, flying, None if short else (real if tof is None else tof), short,
                                         FLOOR + real + wobble, est)
        return self.events

    def what(self):
        return [(e["what"], e["on"]) for e in self.events]

    def test_normal_flight_is_quiet(self):   # 11/16·11/17 정상 비행: 추정과 잰 높이가 0.2~0.3 m 안
        self.fly(10, 1.0, 1.0, noise=0.1)                  # 바닥 기압을 모은다
        self.fly(20, 1.2, 1.45, noise=0.1)                 # 0.25 m 어긋나도 조용
        self.fly(5, 1.0, 1.0, tof=0.75)                    # 책상 위: 하방만 짧다(아래가 가깝지도 않음)
        self.assertEqual(self.events, [])
        self.assertAlmostEqual(self.w.baro_ref(), FLOOR, places=1)

    def test_estimate_stuck_high_like_the_crash(self):   # 11/17 10:23: 추정 1.5 m, 실제 0.95 m(기압·하방 같음)
        self.fly(10, 1.0, 1.0)
        self.fly(GAP_S + 0.5, 0.95, 1.5, noise=0.1)
        self.assertEqual(self.what(), [("gap", True)])
        gap = self.w.state()["gap"]
        self.assertEqual(gap["est_m"], 1.5)
        self.assertTrue(0.45 <= gap["gap_m"] <= 0.65, gap)   # 0.55 ± 기압 흔들림(중앙값)
        self.fly(GAP_S + 0.5, 1.0, 1.05)                  # 다시 맞음
        self.assertEqual(self.what(), [("gap", True), ("gap", False)])

    def test_over_furniture_is_not_a_low_estimate(self):   # 가구 위: 하방만 짧고 기압은 추정과 같다
        self.fly(10, 1.0, 1.0)
        self.fly(GAP_S + 1, 1.0, 1.0, tof=0.55)
        self.assertEqual(self.events, [])

    def test_estimate_low_needs_both_sensors(self):   # 낮게 봄: 기압·하방 둘 다 추정보다 높아야
        self.fly(10, 1.0, 1.0)
        self.fly(GAP_S + 0.5, 1.6, 1.0)
        self.assertEqual(self.what(), [("gap", True)])
        self.assertLess(self.w.state()["gap"]["gap_m"], -0.4)

    def test_low_above_the_floor_or_a_chair(self):
        self.fly(10, 1.0, 1.0)
        self.fly(LOW_S + 0.3, 1.0, 1.0, tof=0.3)           # 의자 위 0.3 m
        self.assertEqual(self.what(), [("low", True)])
        self.assertEqual(self.w.state()["low"]["tof_m"], 0.3)
        self.fly(LOW_S + 0.3, 1.0, 1.0)
        self.assertEqual(self.what(), [("low", True), ("low", False)])
        self.fly(LOW_S + 0.3, 1.0, 1.0, short=True)        # 0.1 m 아래(Tello 무효)도 가깝다
        self.assertEqual(self.what()[-1], ("low", True))

    def test_landing_ends_the_alerts_and_starts_over(self):
        self.fly(10, 1.0, 1.0)
        self.fly(LOW_S + 0.3, 0.3, 1.0)
        self.fly(0.1, 0.3, 1.0, flying=False)              # 착륙 단계·땅: 끝(다시 정상이 된 것은 아님)
        self.assertEqual([e.get("end") for e in self.events if not e["on"]], [True] * sum(not e["on"] for e in self.events))
        self.assertIsNone(self.w.baro_ref())                # 다음 비행은 바닥 기압부터 새로

    def test_no_floor_pressure_no_gap(self):   # 바닥 기압을 모으기 전(추정과 하방이 안 맞음)에는 어긋남을 안 본다
        self.fly(GAP_S + 1, 0.5, 1.5)
        self.assertNotIn(("gap", True), self.what())


if __name__ == "__main__":
    unittest.main()
