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

    def test_takeoff_refused_on_low_battery_and_heat(self):
        self.ctrl.confirm_operator(True)
        self.fake.bat = 20
        self.fake.temp_override = 85
        time.sleep(0.3)
        r = self.ctrl.takeoff()
        self.assertFalse(r["ok"])
        self.assertIn("battery", r["failed"])
        self.assertIn("temp", r["failed"])

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

    def test_state_loss_triggers_auto_land(self):
        self.takeoff()
        self.fake.send_state = False
        self.assertTrue(wait_until(lambda: "land" in self.fake.received, timeout=3))
        self.assertTrue(wait_until(lambda: not self.ctrl.flying, timeout=3))
        self.assertEqual(self.ctrl.last_auto_land["reason"], "state_lost")

    def test_overheat_triggers_auto_land(self):
        self.takeoff()
        self.fake.temp_override = 90
        self.assertTrue(wait_until(lambda: "land" in self.fake.received, timeout=3))
        self.assertEqual(self.ctrl.last_auto_land["reason"], "overheat")

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
