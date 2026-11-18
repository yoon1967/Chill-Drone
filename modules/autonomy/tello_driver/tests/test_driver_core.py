"""가짜 Tello(tello_link/fake_tello.py)로 자율비행 드라이버를 시험한다 (표준 라이브러리만).

    python -B -m unittest discover -s modules/autonomy/tello_driver/tests -v
"""

import json
import math
import os
import sys
import tempfile
import time
import unittest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tello_link"))

from driver_core import RcScale, TelloDriver, raw_to_text, slew, to_input  # noqa: E402
from fake_tello import FakeTello  # noqa: E402
from tello_link import TelloLink, parse_state  # noqa: E402


def wait_until(pred, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.02)
    return False


class ConversionTest(unittest.TestCase):
    def test_signs(self):
        s = RcScale(v_full=1.0, vz_full=1.0, yaw_full=math.radians(100))
        lr, fb, ud, yaw = to_input(s, 30, 0.2, 0.1, 0.05, math.radians(10))
        self.assertAlmostEqual(fb, 0.2)      # 앞 + → fb +
        self.assertAlmostEqual(lr, -0.1)     # 왼쪽 + → lr - (Tello는 오른쪽 +)
        self.assertAlmostEqual(ud, 0.05)
        self.assertAlmostEqual(yaw, -0.1)    # 반시계 + → yaw - (Tello는 시계 +)

    def test_horizontal_limit_keeps_direction(self):
        lr, fb, _, _ = to_input(RcScale(), 30, 0.4, -0.1, 0.0, 0.0)
        self.assertAlmostEqual(fb, 0.3)
        self.assertAlmostEqual(lr / fb, 0.1 / 0.4)   # 축마다 자르면 0.1/0.3이 된다

    def test_dead_zone_lifts_small_commands_keeps_direction(self):
        # 2026-10-29 실기체: rc 20 → 약 0.12 m/s, rc 40 → 약 0.35 m/s (v_dead 0.095, v_full 1.03)
        s = RcScale(v_full=1.03, v_dead=0.095)
        _, fb, _, _ = to_input(s, 80, 0.12, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(fb, 0.2, delta=0.01)
        _, fb, _, _ = to_input(s, 80, 0.35, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(fb, 0.40, delta=0.01)
        lr, fb, _, _ = to_input(s, 80, 0.2, -0.1, 0.0, 0.0)
        self.assertAlmostEqual(lr / fb, 0.1 / 0.2)
        self.assertEqual(to_input(s, 80, 0.0, 0.0, 0.0, 0.0)[:2], (0.0, 0.0))   # 서 있으라면 0

    def test_vertical_dead_zone_lifts_small_climb_but_holds_near_target(self):
        # 11/02: 높이 8 cm 모자람 → 0.064 m/s = rc 6을 Tello가 무시해 0.92 m에 머물렀다. 무시 구간 0.1이면 rc 10을 넘겨 보낸다
        s = RcScale(vz_dead=0.1)
        self.assertAlmostEqual(to_input(s, 80, 0, 0, 0.064, 0)[2], 0.1 + 0.064 * 0.9)
        self.assertAlmostEqual(to_input(s, 80, 0, 0, -0.064, 0)[2], -(0.1 + 0.064 * 0.9))
        self.assertEqual(to_input(s, 80, 0, 0, 0.02, 0)[2], 0.0)      # 4 cm 안은 0 (오르내리지 않게)
        self.assertAlmostEqual(to_input(RcScale(), 80, 0, 0, 0.064, 0)[2], 0.064)   # 기본은 보정 없음

    def test_slew_ramps_up_but_stops_and_brakes_at_once(self):
        # 키우는 쪽만 단계적으로: 0 → 0.8을 0.15씩. 줄이거나 반대로 미는 것은 바로 (정지·제동이 늦으면 안 된다)
        self.assertAlmostEqual(slew(0.0, 0.8, 0.15), 0.15)
        self.assertAlmostEqual(slew(0.15, 0.8, 0.15), 0.30)
        self.assertAlmostEqual(slew(0.75, 0.8, 0.15), 0.80)
        self.assertAlmostEqual(slew(0.5, 0.0, 0.15), 0.0)
        self.assertAlmostEqual(slew(0.5, -0.4, 0.15), -0.4)
        self.assertAlmostEqual(slew(-0.2, -0.8, 0.15), -0.35)

    def test_vertical_and_yaw_clamped(self):
        _, _, ud, yaw = to_input(RcScale(), 30, 0.0, 0.0, 2.0, -5.0)
        self.assertEqual((ud, yaw), (0.3, 0.3))

    def test_state_text_round_trip(self):
        packet = "mid:-1;x:-100;y:-100;z:-100;mpry:0,0,0;pitch:1;roll:-2;yaw:-37;vgx:3;vgy:-1;vgz:0;templ:58;temph:61;" \
                 "tof:96;h:80;bat:87;baro:12.51;time:3;agx:-2.00;agy:1.00;agz:-998.00;\r\n"
        raw = parse_state(packet).raw
        self.assertEqual(parse_state(raw_to_text(raw)).raw, raw)
        self.assertIn("mpry:0,0,0;", raw_to_text(raw))


class DriverTest(unittest.TestCase):
    def setUp(self):
        self.fake = FakeTello(cmd_port=48889, state_port=48890, video_port=41111, send_video=False)
        self.fake.motion_delay = 0.3
        self.fake.temp_override = 50
        self.fake.start()
        self.tmp = tempfile.TemporaryDirectory()
        link = TelloLink(host="127.0.0.1", cmd_port=48889, local_cmd_port=0, state_port=48890, bind_ip="127.0.0.1",
                         default_timeout=1.0, allow_flight=True, log_root=self.tmp.name, run_id="driver_test",
                         source="fake").start()
        self.drv = TelloDriver(link, rc_limit=30, cmd_timeout=0.6, log=lambda *_: None).start()
        self.assertTrue(wait_until(lambda: self.drv.connected and link.latest_state() is not None))

    def tearDown(self):
        self.drv.close()
        self.fake.close()
        self.tmp.cleanup()

    def take_off(self):
        self.drv.ctrl.confirm_operator(True)
        self.assertTrue(self.drv.request("takeoff"))
        self.assertTrue(wait_until(lambda: self.drv.ctrl.flying))

    def rc_sent(self):
        return [c for c in self.fake.received if c.startswith("rc ")]

    def test_takeoff_needs_operator_confirmation(self):
        self.assertTrue(self.drv.request("takeoff"))     # 받아들이지만 이륙 전 점검에서 막힌다
        self.assertTrue(wait_until(lambda: self.drv.last_result is not None))
        self.assertFalse(self.drv.ctrl.flying)
        self.assertIn("operator", self.drv.last_result["failed"])
        self.assertNotIn("takeoff", self.fake.received)

    def test_velocity_becomes_rc(self):
        self.take_off()
        for _ in range(10):
            self.drv.on_cmd_vel(0.2, 0.1, 0.0, 0.0)
            time.sleep(0.03)
        self.assertTrue(wait_until(lambda: "rc -10 20 0 0" in self.rc_sent()))
        with open(os.path.join(self.tmp.name, "driver_test", "manifest.json"), encoding="utf-8") as f:
            self.assertFalse(json.load(f)["operator_controlled"])   # 자율비행 회차는 사람 조종으로 남기지 않는다

    def test_command_timeout_lands(self):
        self.take_off()
        self.drv.on_cmd_vel(0.0, 0.0, 0.0, 0.0)

        def ticked():
            self.drv.tick()
            return self.drv.landing_reason == "cmd_timeout"
        self.assertTrue(wait_until(ticked))
        self.assertTrue(wait_until(lambda: not self.drv.ctrl.flying))
        self.assertIn("land", self.fake.received)

    def test_operator_confirmation_by_request(self):
        self.assertTrue(self.drv.request("confirm"))
        self.assertTrue(self.drv.ctrl.operator_confirmed)
        self.take_off()
        self.assertFalse(self.drv.request("confirm"))       # 떠 있는 동안에는 바꾸지 않는다
        self.assertTrue(self.drv.request("land"))
        self.assertTrue(wait_until(lambda: not self.drv.ctrl.flying))
        self.assertFalse(self.drv.ctrl.operator_confirmed)  # 착륙하면 다시 확인해야 한다

    def test_land_when_not_flying_is_refused(self):
        self.assertFalse(self.drv.request("land"))
        self.assertFalse(self.drv.request("flip f"))

    def test_keepalive_on_ground_only(self):
        import driver_core
        driver_core.KEEPALIVE_S, saved = 0.2, driver_core.KEEPALIVE_S
        try:
            count = lambda: self.fake.received.count("command")  # noqa: E731
            before = count()
            for _ in range(10):
                self.drv.tick()
                time.sleep(0.05)
            self.assertTrue(wait_until(lambda: count() >= before + 2))   # 땅에서는 command를 다시 보낸다
            self.take_off()
            self.drv.on_cmd_vel(0.0, 0.0, 0.0, 0.0)
            in_air = count()
            for _ in range(10):
                self.drv.on_cmd_vel(0.0, 0.0, 0.0, 0.0)
                self.drv.tick()
                time.sleep(0.05)
            self.assertEqual(count(), in_air)    # 떠 있으면 rc가 대신한다
        finally:
            driver_core.KEEPALIVE_S = saved

    def test_state_is_forwarded_once(self):
        self.assertTrue(wait_until(lambda: self.drv.poll_state() is not None))
        self.assertIsNone(self.drv.poll_state())   # 같은 패킷은 다시 넘기지 않는다
        self.assertTrue(wait_until(lambda: self.drv.poll_state() is not None))


class FrontTofTest(unittest.TestCase):
    """확장 키트 전방 ToF: 떠 있을 때만 묻고, 새 값만 넘긴다."""

    def setUp(self):
        self.fake = FakeTello(cmd_port=48989, state_port=48990, video_port=41211, send_video=False)
        self.fake.motion_delay = 0.3
        self.fake.temp_override = 50
        self.fake.ext_kit = True
        self.fake.start()
        self.tmp = tempfile.TemporaryDirectory()
        link = TelloLink(host="127.0.0.1", cmd_port=48989, local_cmd_port=0, state_port=48990, bind_ip="127.0.0.1",
                         default_timeout=1.0, allow_flight=True, log_root=self.tmp.name, run_id="tof_test",
                         source="fake").start()
        self.got = []
        self.drv = TelloDriver(link, rc_limit=30, cmd_timeout=5.0, log=lambda *_: None, front_tof=True,
                               on_front_tof=lambda r, mono: self.got.append(r)).start()
        self.assertTrue(wait_until(lambda: self.drv.connected and link.latest_state() is not None))

    def tearDown(self):
        self.drv.close()
        self.fake.close()
        self.tmp.cleanup()

    def tof_queries(self):
        return [c for c in self.fake.received if c.lower() == "ext tof?"]

    def test_asks_once_on_ground_then_only_while_flying(self):
        self.assertTrue(wait_until(lambda: self.drv.front_tof_last is not None))
        self.assertEqual(self.drv.front_tof_last["front_tof_m"], 0.481)   # 연결 때 한 번 물어 보여 준다
        time.sleep(0.4)
        self.assertEqual(len(self.tof_queries()), 1)
        self.assertEqual(self.got, [])          # 땅에서는 비행 노드에 넘기지 않는다
        self.drv.ctrl.confirm_operator(True)
        self.assertTrue(self.drv.request("takeoff"))
        self.assertTrue(wait_until(lambda: self.drv.ctrl.flying))
        self.drv.on_cmd_vel(0.0, 0.0, 0.0, 0.0)
        self.assertTrue(wait_until(lambda: self.got))
        self.assertAlmostEqual(self.got[0], 0.481)

    def test_fresh_values_out_of_range_and_errors(self):
        self.fake.front_tof_mm = 900
        self.assertAlmostEqual(self.drv.poll_front_tof(), 0.9)
        self.assertIsNone(self.drv.poll_front_tof())          # 0.2초 안의 같은 값은 넘기지 않는다
        time.sleep(0.25)
        self.assertAlmostEqual(self.drv.poll_front_tof(), 0.9)  # 멈춘 장애물도 0.2초마다 다시 넘긴다
        self.fake.front_tof_mm = 8190
        self.assertEqual(self.drv.poll_front_tof(), math.inf)  # 범위 밖은 inf
        self.fake.ext_kit = False                               # 키트가 'error'로 답하면 버린다
        self.assertIsNone(self.drv.poll_front_tof())
        self.assertEqual(self.drv.front_tof_last["reason"], "error_response")
        self.assertEqual(self.got, [0.9, 0.9, math.inf])


if __name__ == "__main__":
    unittest.main()
