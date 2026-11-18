"""전방 ToF(확장 키트 EXT tof?)를 게이트웨이가 묻는다: 상태 drone_state.front_tof와 자율 front_tof (가짜 Tello).

계약 AUTONOMY_BRIDGE_V0_1 3장 "전방 ToF: 새 값일 때만 보낸다". 확장 키트가 없으면 묻기를 멈춘다.

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_front_tof.py" -v
"""

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import gateway  # noqa: E402
from test_gateway import GatewayCase, wait_until  # noqa: E402


def queries(fake) -> int:
    return sum(1 for c in list(fake.received) if c.lower() == "ext tof?")


class FrontTofTest(GatewayCase):
    """확장 키트가 달린 기체(RoboMaster TT). 가짜 Tello는 늘 `tof 481`로 답한다."""

    def setUp(self):
        super().setUp()
        self.fake.ext_kit = True
        self.gw._forget_ext_kit()   # 시작하자마자 키트 없이 물었을 수 있어 처음부터 다시 본다
        self.to_autonomy = []

    def test_ground_reads_front_distance(self):
        self.assertTrue(wait_until(lambda: self.gw.drone_state()["front_tof"]["valid"]), self.gw.drone_state()["front_tof"])
        tof = self.gw.drone_state()["front_tof"]
        self.assertEqual((tof["m"], tof["reason"]), (0.481, None))
        self.assertLess(tof["age_s"], gateway.FRONT_TOF_STALE_S)

    def test_new_values_go_to_autonomy(self):
        self.assertTrue(self.gw.autonomy_open("au-1", self.to_autonomy.append))

        def sent():
            return [m for m in list(self.to_autonomy) if m["type"] == "front_tof"]
        self.assertTrue(wait_until(lambda: len(sent()) >= 2))
        a, b = sent()[:2]
        self.assertEqual((a["m"], a["valid"], a["reason"], b["seq"] - a["seq"]), (0.481, True, None, 1))
        self.assertGreaterEqual(a["age_s"], 0)

    def test_faster_while_flying(self):
        self.assertTrue(wait_until(lambda: self.gw.drone_state()["front_tof"]["valid"]))
        n = queries(self.fake)
        time.sleep(1.5)
        ground = queries(self.fake) - n
        self.take_off()
        n = queries(self.fake)
        time.sleep(1.5)
        air = queries(self.fake) - n
        self.assertLessEqual(ground, 4)   # 땅에서 0.5초마다
        self.assertGreaterEqual(air, 8)   # 떠 있을 때 0.1초마다(왕복 시간만큼 덜)

    def test_quiet_during_takeoff_landing_and_video_commands(self):
        self.assertTrue(wait_until(self.gw._front_tof_due))
        self.gw.flight.busy = "takeoff"   # 늦게 온 tof 응답을 이륙 응답으로 읽지 않게
        self.assertFalse(self.gw._front_tof_due())
        self.gw.flight.busy = None
        self.gw._video_busy = True
        self.assertFalse(self.gw._front_tof_due())
        self.gw._video_busy = False
        self.assertTrue(self.gw._front_tof_due())

    def test_old_value_is_stale(self):
        self.gw._stop.set()   # 묻기를 멈추고 시각을 직접 넣는다
        self.gw._tof_thread.join(timeout=2)
        reading = {"front_tof_m": 0.5, "valid": True, "reason": None, "raw": "tof 500"}
        self.gw._ext_kit = True
        self.gw._tof = (reading, self.gw.clock() - gateway.FRONT_TOF_STALE_S - 0.5)
        self.assertEqual((self.gw._front_tof_state()["valid"], self.gw._front_tof_state()["reason"]), (False, "stale"))
        self.gw._tof = (reading, self.gw.clock())
        self.assertEqual(self.gw._front_tof_state()["m"], 0.5)


class NoExtKitTest(GatewayCase):
    """확장 키트가 없는 기체(기본 Tello). 가짜 Tello는 EXT 명령에 error로 답한다."""

    def test_stops_asking_without_ext_kit(self):
        self.assertTrue(wait_until(lambda: self.gw.drone_state()["front_tof"]["reason"] == "no_ext_kit"),
                        self.gw.drone_state()["front_tof"])
        n = queries(self.fake)
        self.assertEqual(n, gateway.EXT_KIT_PROBES)
        to_autonomy = []
        self.assertTrue(self.gw.autonomy_open("au-1", to_autonomy.append))
        time.sleep(1.2)
        self.assertEqual(queries(self.fake), n)   # 다시 묻는 것은 땅에서 30초마다
        self.assertFalse([m for m in list(to_autonomy) if m["type"] == "front_tof"])

    def test_reconnect_checks_again(self):   # 끊긴 동안 확장 키트가 달린 기체로 바꿔 끼웠다
        self.assertTrue(wait_until(lambda: self.gw.drone_state()["front_tof"]["reason"] == "no_ext_kit"))
        self.fake.ext_kit = True
        self.gw._forget_ext_kit()
        self.assertTrue(wait_until(lambda: self.gw.drone_state()["front_tof"]["valid"]))


if __name__ == "__main__":
    unittest.main()
