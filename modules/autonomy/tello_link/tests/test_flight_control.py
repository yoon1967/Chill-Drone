"""가짜 Tello로 FlightController의 안전 장치를 시험한다 (표준 라이브러리만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -v
"""

import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import flight_control as fc  # noqa: E402
from fake_tello import FakeTello  # noqa: E402
from flight_control import FlightController  # noqa: E402
from tello_link import TelloLink  # noqa: E402


def wait_until(pred, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.05)
    return False


class FlightControlTest(unittest.TestCase):
    def setUp(self):
        self.fake = FakeTello(cmd_port=38889, state_port=38890, video_port=31111, send_video=False)
        self.fake.motion_delay = 0.3
        self.fake.temp_override = 50
        self.fake.start()
        self.tmp = tempfile.TemporaryDirectory()
        self.link = TelloLink(host="127.0.0.1", cmd_port=38889, local_cmd_port=0, state_port=38890,
                              bind_ip="127.0.0.1", default_timeout=1.0, allow_flight=True,
                              log_root=self.tmp.name, run_id="flight_test", source="fake").start()
        self.link.send("command")
        self.assertTrue(wait_until(lambda: self.link.latest_state() is not None))
        self.ctrl = FlightController(self.link).start()

    def tearDown(self):
        self.ctrl._stop.set()
        self.ctrl._thread.join(timeout=20)
        self.link.close()
        self.fake.close()
        self.tmp.cleanup()

    def rc_sent(self):
        return [c for c in self.fake.received if c.startswith("rc ")]

    def takeoff(self):
        self.ctrl.confirm_operator(True)
        r = self.ctrl.takeoff()
        self.assertTrue(r["ok"], r)
        return r

    def test_requires_allow_flight(self):
        link = TelloLink(host="127.0.0.1", allow_flight=False)
        with self.assertRaises(ValueError):
            FlightController(link)

    def test_takeoff_refused_without_operator_confirm(self):
        r = self.ctrl.takeoff()
        self.assertFalse(r["ok"])
        self.assertIn("operator", r["failed"])
        self.assertNotIn("takeoff", self.fake.received)

    def autonomy_ctrl(self, **limits):
        """자율비행 드라이버(tello_driver)가 넘기는 기준으로 만든 FlightController (기본값은 수동 조종 게이트웨이용이다)."""
        base = dict(temp_max_takeoff_c=80, temp_land_c=85, battery_min_takeoff_pct=0, battery_land_pct=10)
        return FlightController(self.link, **{**base, **limits})

    def test_autonomy_temperature_limits_block_and_can_be_raised(self):
        self.fake.temp_override = 85   # temph 88
        si = lambda: (self.link.latest_state() or {}).get("si") or {}  # noqa: E731
        self.assertTrue(wait_until(lambda: si().get("temp_high_c") == 88))
        temp = lambda ctrl: next(c for c in ctrl.preflight() if c.key == "temp")  # noqa: E731
        auto = self.autonomy_ctrl()
        self.assertFalse(temp(auto).ok)                        # 자율비행: 이륙 80°C 미만
        self.assertTrue(temp(auto).blocking)
        self.assertEqual(auto._safety_reason(), "overheat")    # 자율비행: 85°C 이상이면 착륙
        hot = self.autonomy_ctrl(temp_max_takeoff_c=90, temp_land_c=90)
        self.assertTrue(temp(hot).ok)
        self.assertIsNone(hot._safety_reason())
        self.assertFalse(temp(self.ctrl).blocking)             # 기본: 알리기만 하고 이륙은 막지 않는다
        self.assertIsNone(self.ctrl._safety_reason())          # 기본: 고온 자동 착륙 없음

    def test_autonomy_battery_has_no_takeoff_limit_but_lands_at_10_or_less(self):
        # 2026-11-02: 자율비행은 이륙 기준 없음. 10% 이하만 거절(이륙하자마자 착륙하므로)·비행 중 착륙
        auto = self.autonomy_ctrl()
        self.fake.bat = 11
        time.sleep(0.3)
        checks = {c.key: c for c in auto.preflight()}
        self.assertTrue(checks["battery"].ok)
        self.assertIsNone(auto._safety_reason())
        self.fake.bat = 10
        time.sleep(0.3)
        checks = {c.key: c for c in auto.preflight()}
        self.assertFalse(checks["battery"].ok)
        self.assertEqual(auto._safety_reason(), "battery_low")
        strict = self.autonomy_ctrl(battery_min_takeoff_pct=30)   # 기준을 주면 그대로 쓴다
        self.fake.bat = 20
        time.sleep(0.3)
        self.assertFalse({c.key: c for c in strict.preflight()}["battery"].ok)
        self.assertIsNone(strict._safety_reason())

    def test_autonomy_takeoff_refused_on_low_battery_and_heat(self):
        auto = self.autonomy_ctrl()
        auto.confirm_operator(True)
        self.fake.bat = 10
        self.fake.temp_override = 85
        time.sleep(0.3)
        r = auto.takeoff()
        self.assertFalse(r["ok"])
        self.assertIn("battery", r["failed"])
        self.assertIn("temp", r["failed"])
        self.assertNotIn("takeoff", self.fake.received)

    def test_default_battery_lands_at_or_below_limit(self):
        # 기본(수동 조종 게이트웨이): BATTERY_LAND 이하면 착륙, 그 위는 그대로
        self.fake.bat = fc.BATTERY_LAND + 1
        time.sleep(0.3)
        self.assertIsNone(self.ctrl._safety_reason())
        self.fake.bat = fc.BATTERY_LAND
        time.sleep(0.3)
        self.assertEqual(self.ctrl._safety_reason(), "battery_low")

    def test_takeoff_refused_on_low_battery_not_heat(self):
        self.ctrl.confirm_operator(True)
        self.fake.bat = fc.BATTERY_MIN_TAKEOFF - 1
        self.fake.temp_override = 85
        time.sleep(0.3)
        r = self.ctrl.takeoff()
        self.assertFalse(r["ok"])
        self.assertEqual(r["failed"], ["battery"])  # 온도는 경고만, 이륙 실패 사유가 아니다

    def test_takeoff_marks_operator_controlled_and_sends_rc(self):
        self.takeoff()
        self.assertTrue(self.ctrl.flying)
        self.assertTrue(self.link.manifest["operator_controlled"])
        self.assertTrue(wait_until(lambda: len(self.rc_sent()) >= 5))

    def test_rc_is_clamped_to_limit(self):
        self.takeoff()
        self.ctrl.set_input(1, -1, 0, 1, source="test")
        self.assertTrue(wait_until(lambda: "rc 30 -30 0 30" in self.rc_sent(), timeout=1))

    def test_deadman_returns_to_hover(self):
        self.takeoff()
        self.ctrl.set_input(0, 1, 0, 0, source="test")
        self.assertTrue(wait_until(lambda: "rc 0 30 0 0" in self.rc_sent(), timeout=1))
        time.sleep(fc.DEADMAN_S + 0.3)  # 입력을 더 보내지 않으면
        self.assertEqual(self.rc_sent()[-1], "rc 0 0 0 0")

    def test_height_limit_blocks_climb(self):
        self.takeoff()
        self.fake.h_cm = 160
        time.sleep(0.3)
        for _ in range(5):
            self.ctrl.set_input(0, 0, 1, 0, source="test")
            time.sleep(0.05)
        self.assertEqual(self.rc_sent()[-1], "rc 0 0 0 0")
        self.ctrl.set_input(0, 0, -1, 0, source="test")  # 하강은 허용
        self.assertTrue(wait_until(lambda: self.rc_sent()[-1] == "rc 0 0 -30 0", timeout=1))

    def climb_blocked(self):
        for _ in range(5):
            self.ctrl.set_input(0, 0, 1, 0, source="test")
            time.sleep(0.05)
        return self.rc_sent()[-1] == "rc 0 0 0 0"

    def test_height_limit_uses_down_tof_when_h_is_wrong(self):
        # 실기체에서 0.78m에 떠 있는데 h가 0~10cm로 나온 경우(허병우 10/27)
        self.takeoff()
        self.fake.h_override = 5
        self.fake.tof_override = 160
        time.sleep(0.3)
        self.assertTrue(self.climb_blocked())
        self.ctrl.set_input(0, 0, -1, 0, source="test")
        self.assertTrue(wait_until(lambda: self.rc_sent()[-1] == "rc 0 0 -30 0", timeout=1))

    def test_invalid_down_tof_blocks_climb(self):
        self.takeoff()
        self.fake.tof_override = 6553  # 반사 없음
        time.sleep(0.3)
        self.assertTrue(self.climb_blocked())
        self.fake.tof_override = 80    # 다시 유효해지면 상승 허용
        time.sleep(0.3)
        self.ctrl.set_input(0, 0, 1, 0, source="test")
        self.assertTrue(wait_until(lambda: self.rc_sent()[-1] == "rc 0 0 30 0", timeout=1))

    def test_state_loss_triggers_auto_land(self):
        self.takeoff()
        self.fake.send_state = False
        self.assertTrue(wait_until(lambda: "land" in self.fake.received, timeout=3))
        self.assertTrue(wait_until(lambda: not self.ctrl.flying, timeout=3))
        self.assertEqual(self.ctrl.last_auto_land["reason"], "state_lost")

    def test_hot_drone_can_take_off_with_warning(self):
        # 지상에서 뜨거워진 Tello는 띄워야 식는다. 온도는 알리기만 하고 이륙을 막지 않는다.
        self.fake.temp_override = 88
        time.sleep(0.3)
        temp = [c for c in self.ctrl.preflight() if c.key == "temp"][0]
        self.assertFalse(temp.ok)
        self.assertFalse(temp.blocking)
        self.takeoff()

    def test_no_auto_land_for_heat(self):
        self.takeoff()
        self.fake.temp_override = 92
        time.sleep(0.6)
        self.assertNotIn("land", self.fake.received)
        self.assertTrue(self.ctrl.flying)

    def test_low_battery_blocks_takeoff(self):
        self.fake.bat = fc.BATTERY_MIN_TAKEOFF - 1
        time.sleep(0.3)
        self.ctrl.confirm_operator(True)
        r = self.ctrl.takeoff()
        self.assertEqual((r["ok"], r["failed"]), (False, ["battery"]))
        self.fake.bat = fc.BATTERY_MIN_TAKEOFF
        time.sleep(0.3)
        self.assertTrue(self.ctrl.takeoff()["ok"])

    def test_emergency_during_takeoff_is_not_blocked(self):
        import threading
        self.fake.motion_delay = 2.0
        self.ctrl.confirm_operator(True)
        t = threading.Thread(target=self.ctrl.takeoff)
        t.start()
        time.sleep(0.3)  # takeoff 응답을 기다리는 중
        t0 = time.monotonic()
        self.ctrl.emergency(source="test")
        self.assertLess(time.monotonic() - t0, 0.2)
        self.assertTrue(wait_until(lambda: "emergency" in self.fake.received, timeout=1))
        self.assertTrue(wait_until(lambda: not self.fake.flying, timeout=1))
        t.join(timeout=5)

    def test_land_error_on_ground_counts_as_landed(self):
        # 부딪혀 떨어지면 기체는 바닥인데 우리는 떠 있다고 믿고, land에 error가 와서 끝없이 다시 보냈다 (10/29)
        self.takeoff()
        self.fake.flying, self.fake.h_cm = False, 0.0
        self.assertTrue(wait_until(lambda: ((self.link.latest_state() or {}).get("si") or {}).get("tof_valid") is False,
                                   timeout=2))
        r = self.ctrl.land()
        self.assertFalse(r["ok"])
        self.assertFalse(self.ctrl.flying)

    def test_crash_tilt_then_ground_stops_flying(self):
        # 2026-10-29: 부딪혀 pitch 40°로 기울며 떨어졌는데 떠 있다고 믿고 2초 넘게 rc를 보냈다
        self.takeoff()
        self.fake.pitch = 40
        time.sleep(0.3)
        self.fake.flying, self.fake.h_cm, self.fake.pitch = False, 0.0, 0
        self.assertTrue(wait_until(lambda: not self.ctrl.flying, timeout=2.5))
        self.assertIn("land", self.fake.received)
        self.assertEqual((self.ctrl.last_auto_land or {}).get("reason"), "crash")

    def test_flipped_stops_motors_without_waiting_for_ground(self):
        # 2026-11-03 14:32: roll −173°로 뒤집혔는데 하방 tof가 위를 봐 바닥 판정이 안 났고 land는 error였다
        self.takeoff()
        self.fake.pitch = 170
        self.assertTrue(wait_until(lambda: not self.ctrl.flying, timeout=2.5))
        self.assertIn("emergency", self.fake.received)
        self.assertEqual((self.ctrl.last_auto_land or {}).get("reason"), "flipped")
        self.fake.pitch = 0

    def test_on_ground_without_tilt_counts_as_grounded(self):
        # 2026-10-30 복도: 기울지 않고 스스로 내려앉았다. 충돌(crash)이 아니라 내려앉음(grounded)이고,
        # 하방 tof와 h가 둘 다 바닥이면 0.8 s에 판정한다
        self.takeoff()
        self.fake.flying, self.fake.h_cm = False, 0.0
        self.assertTrue(wait_until(lambda: not self.ctrl.flying, timeout=2.0))
        self.assertIn("land", self.fake.received)
        self.assertEqual((self.ctrl.last_auto_land or {}).get("reason"), "grounded")

    def test_tof_only_ground_waits_longer_then_grounded(self):
        self.takeoff()
        self.fake.tof_override = 10     # 하방 tof만 바닥 (h는 떠 있음): 낮은 가구 위일 수 있어 1.5 s 기다린다
        time.sleep(1.0)
        self.assertTrue(self.ctrl.flying)
        self.assertTrue(wait_until(lambda: not self.ctrl.flying, timeout=1.5))
        self.assertEqual((self.ctrl.last_auto_land or {}).get("reason"), "grounded")

    def test_ground_hold_stops_pushing_before_verdict(self):
        # 바닥에 닿은 뒤 판정(0.8 s)까지 전진 입력을 그대로 보내 카펫 위에서 밀었다 → 0.3 s부터 rc 0 0 0 0
        self.takeoff()
        self.fake.flying, self.fake.h_cm = False, 0.0
        t_end = time.monotonic() + 1.5
        while time.monotonic() < t_end and self.ctrl.flying:
            self.ctrl.set_input(0, 0.3, 0, 0, source="test")
            time.sleep(0.05)
        self.assertFalse(self.ctrl.flying)
        land_i = self.fake.received.index("land")
        before_land = [c for c in self.fake.received[:land_i] if c.startswith("rc ")]
        self.assertIn("rc 0 30 0 0", before_land)                 # 처음 0.3 s는 입력대로
        self.assertEqual(before_land[-3:], ["rc 0 0 0 0"] * 3)    # 판정 직전은 멈춰 있음

    def test_land_zeroes_stick_before_land_command(self):
        # 2026-11-12 13:58 실기체: 뒤쪽 제동(rc 0 -36 0 0) 중에 착륙 요청. land()가 입력을 0으로 정해 두기만 하고 rc 루프를
        # 세워 0이 나가지 않았고, Tello는 마지막 스틱대로 내려오는 3.5초 동안 0.83 m 뒤로 갔다 → land 바로 앞의 rc는 0
        self.takeoff()
        for _ in range(6):
            self.ctrl.set_input(0, -0.3, 0, 0, source="test")
            time.sleep(0.05)
        self.assertEqual(self.rc_sent()[-1], "rc 0 -30 0 0")
        self.ctrl.set_input(0, -0.3, 0, 0, source="test")
        self.assertTrue(self.ctrl.land()["ok"])
        land_i = self.fake.received.index("land")
        before_land = [c for c in self.fake.received[:land_i] if c.startswith("rc ")]
        self.assertIn("rc 0 -30 0 0", before_land)
        self.assertEqual(before_land[-fc.LAND_ZERO_RC:], ["rc 0 0 0 0"] * fc.LAND_ZERO_RC)
        after_land = [c for c in self.fake.received[land_i:] if c.startswith("rc ")]
        self.assertEqual([c for c in after_land if c != "rc 0 0 0 0"], [])
        self.assertEqual(self.ctrl.last_rc, (0, 0, 0, 0))

    def test_sink_against_climb_leaves_warning(self):
        # 상승 입력(rc 30)을 보내는데 하방 tof와 Tello h가 같이 0.6 s에 0.2 m 넘게 떨어지면 경고만 남긴다 (착륙시키지 않는다)
        self.takeoff()
        self.fake.tof_override, self.fake.h_override = 95, 95
        steps = iter([85, 75, 60, 45, 40, 40, 40, 40])
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            self.ctrl.set_input(0, 0, 0.3, 0, source="test")
            v = next(steps, 40)
            self.fake.tof_override, self.fake.h_override = v, v
            time.sleep(0.1)
        self.assertTrue(wait_until(lambda: self.ctrl.last_warning is not None, timeout=1.0))
        self.assertEqual(self.ctrl.last_warning["type"], "sink_against_climb")
        self.assertGreaterEqual(self.ctrl.last_warning["detail"]["drop_m"], 0.2)
        self.assertTrue(self.ctrl.flying)
        self.assertNotIn("land", self.fake.received)

    def test_furniture_below_while_climbing_is_not_sink(self):
        # 11/05–11/09 실기체: 가구 위로 넘으려 오르는 동안 밑에 가구가 들어와 하방 tof만 0.3–0.6 m 줄고 h는 그대로였다 → 경고 없음
        self.takeoff()
        self.fake.tof_override, self.fake.h_override = 120, 100
        steps = iter([110, 95, 80, 70, 65, 65, 65, 65])
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            self.ctrl.set_input(0, 0, 0.3, 0, source="test")
            self.fake.tof_override = next(steps, 65)
            time.sleep(0.1)
        time.sleep(0.3)
        self.assertIsNone(self.ctrl.last_warning)
        self.assertTrue(self.ctrl.flying)

    def test_climb_stalled_lowers_cap(self):
        # 11/09 17:07: 상승 입력 rc 37을 2.7초 보내도 h가 1.9 m 그대로(천장에 눌림) → 이번 비행 상한을 h − 0.3으로 낮추고 잠깐 내려온다
        self.takeoff()
        self.fake.tof_override, self.fake.h_override = 130, 130
        t_end = time.monotonic() + fc.CLIMB_STALL_S + 0.6
        while time.monotonic() < t_end:
            self.ctrl.set_input(0, 0, 1, 0, source="test")
            time.sleep(0.05)
        self.assertTrue(wait_until(lambda: (self.ctrl.last_warning or {}).get("type") == "climb_stalled", timeout=1.0))
        self.assertAlmostEqual(self.ctrl.ceiling_cap_m, 1.0, places=2)
        self.assertIn(f"rc 0 0 {fc.CEILING_DROP_UD} 0", self.rc_sent())
        time.sleep(fc.CEILING_DROP_S)
        self.assertTrue(self.climb_blocked())   # 1.3 m는 낮춘 상한 1.0 m 이상
        self.assertTrue(self.ctrl.flying)
        self.ctrl.land()
        self.assertIsNone(self.ctrl.ceiling_cap_m)   # 다음 비행에는 남기지 않는다

    def test_normal_climb_is_not_stall(self):
        self.takeoff()
        self.fake.tof_override = 120
        h = 120.0
        t_end = time.monotonic() + fc.CLIMB_STALL_S + 0.6
        while time.monotonic() < t_end:   # 약 0.15 m/s로 오른다 (높이 상한 1.5 m에 닿으면 상승이 막힌다)
            self.ctrl.set_input(0, 0, 1, 0, source="test")
            self.fake.h_override = int(h)
            h += 0.75
            time.sleep(0.05)
        self.assertIsNone(self.ctrl.last_warning)
        self.assertIsNone(self.ctrl.ceiling_cap_m)

    def test_short_low_reading_is_not_a_crash(self):
        self.takeoff()
        self.fake.tof_override = 15     # 0.3초만 바닥처럼 (낮은 가구 위를 스침)
        time.sleep(0.3)
        self.fake.tof_override = None
        time.sleep(1.8)
        self.assertTrue(self.ctrl.flying)
        self.assertNotIn("land", self.fake.received)

    def test_emergency_cancels_a_takeoff_waiting_for_the_command_socket(self):
        import threading
        self.ctrl.confirm_operator(True)
        result = []
        takeoff = threading.Thread(target=lambda: result.append(self.ctrl.takeoff()))
        with self.link._cmd_lock:
            takeoff.start()
            self.assertTrue(wait_until(lambda: self.ctrl.busy == "takeoff"))
            self.ctrl.emergency(source="test")
        takeoff.join(timeout=2)
        self.assertFalse(takeoff.is_alive())
        self.assertFalse(result[0]["ok"])
        self.assertFalse(self.ctrl.flying)
        self.assertNotIn("takeoff", self.fake.received)

    def test_land_requires_reconfirm_for_next_takeoff(self):
        self.takeoff()
        r = self.ctrl.land()
        self.assertTrue(r["ok"])
        self.assertFalse(self.ctrl.flying)
        again = self.ctrl.takeoff()
        self.assertFalse(again["ok"])
        self.assertIn("operator", again["failed"])


if __name__ == "__main__":
    unittest.main()
