"""가짜 Tello로 게이트웨이를 시험한다: 화면 메시지 → 판정기 → FlightController → 가짜 기체 (표준 라이브러리만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -v
"""

import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fake_tello import FakeTello  # noqa: E402
import gateway  # noqa: E402
from gateway import Gateway  # noqa: E402
from tello_link import TelloLink  # noqa: E402


def wait_until(pred, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.05)
    return False


class GatewayCase(unittest.TestCase):
    def make_video(self):
        return None

    def setUp(self):
        self.fake = FakeTello(cmd_port=48889, state_port=48890, video_port=41111, send_video=False)
        self.fake.motion_delay = 0.3
        self.fake.temp_override = 50
        self.fake.start()
        self.tmp = tempfile.TemporaryDirectory()
        self.link = TelloLink(host="127.0.0.1", cmd_port=48889, local_cmd_port=0, state_port=48890,
                              bind_ip="127.0.0.1", default_timeout=1.0, allow_flight=True,
                              log_root=self.tmp.name, run_id="gateway_test", source="fake").start()
        self.gw = Gateway(self.link, source="fake", video=self.make_video())
        self.sent = []
        self.gw.listeners.append(self.sent.append)
        self.gw.start()
        self.assertTrue(wait_until(lambda: self.gw.arbiter.mode == "grounded"), "SDK 진입 후 지상이어야 한다")

    def tearDown(self):
        self.gw._stop.set()
        self.gw._thread.join(timeout=2)
        self.gw.flight._stop.set()
        self.gw.flight._thread.join(timeout=20)
        self.link.close()
        self.fake.close()
        self.tmp.cleanup()

    def cmd(self, name, client="monitor", **args):
        return self.gw.handle_message({"type": "cmd", "id": f"t-{name}", "name": name, "args": args, "client": client}, "t")

    def rc(self, axes, client="monitor"):
        self.gw.handle_message({"type": "rc", "axes": axes, "client": client}, "t")

    def rc_sent(self):
        return [c for c in self.fake.received if c.startswith("rc ")]

    def take_off(self):
        self.assertTrue(self.cmd("confirm_area", confirmed=True)["ok"])
        self.assertTrue(self.cmd("takeoff")["ok"])
        self.assertTrue(wait_until(lambda: self.gw.arbiter.mode == "manual"), self.gw.arbiter.mode)


class GatewayTest(GatewayCase):
    def test_state_follows_contract(self):
        st = self.gw.drone_state()
        for key in ("schema_version", "type", "at", "time_source", "source", "connected", "mode", "authority",
                    "allowed", "warnings", "battery_pct", "height_m", "temp_high_c", "front_tof", "down_tof",
                    "mission", "video", "last_transition", "operator_interventions"):
            self.assertIn(key, st)
        self.assertEqual((st["type"], st["mode"], st["source"]), ("drone_state", "grounded", "fake"))
        self.assertIsNone(st["height_m"])               # 지상: 하방 tof 무효 → 높이 없음(0이 아님)
        self.assertFalse(st["front_tof"]["valid"])
        self.assertIn("takeoff", st["allowed"])

    def test_takeoff_needs_area_confirm(self):
        r = self.cmd("takeoff")
        self.assertEqual((r["ok"], r["reason"]), (False, "preflight_failed"))
        self.assertIn("operator", r["failed"])
        self.assertNotIn("takeoff", self.fake.received)

    def test_manual_flight_rc_and_land(self):
        self.take_off()
        self.rc([0, 1, 0, 0])
        self.assertTrue(wait_until(lambda: any(c.startswith("rc 0 ") and c != "rc 0 0 0 0" for c in self.rc_sent())))
        self.assertTrue(wait_until(lambda: "rc 0 30 0 0" in self.rc_sent()), self.rc_sent()[-5:])  # 가감속 후 30
        time.sleep(0.5)  # 입력이 끊기면(데드맨) 제자리
        self.assertEqual(self.rc_sent()[-1], "rc 0 0 0 0")
        self.assertTrue(self.cmd("land")["ok"])
        self.assertTrue(wait_until(lambda: self.gw.arbiter.mode == "grounded"), self.gw.arbiter.mode)
        self.assertIn("land", self.fake.received)

    def test_emergency_goes_out_and_settles(self):
        self.take_off()
        self.assertTrue(self.cmd("emergency", client="control")["ok"])
        self.assertTrue(wait_until(lambda: "emergency" in self.fake.received), self.fake.received)  # 응답 없이 보내므로 도착을 기다린다
        self.assertEqual(self.gw.arbiter.mode, "emergency")
        self.assertTrue(wait_until(lambda: self.gw.arbiter.mode == "grounded", timeout=5), self.gw.arbiter.mode)

    def test_screen_disconnect_zeroes_rc(self):
        self.take_off()
        self.rc([0, 1, 0, 0])
        self.gw.client_closed("t")
        time.sleep(0.2)
        self.assertEqual(self.rc_sent()[-1], "rc 0 0 0 0")

    def test_events_reach_screens(self):
        self.take_off()
        kinds = [m["detail"]["to"] for m in self.sent if m.get("type") == "event" and m.get("kind") == "mode_changed"]
        self.assertIn("manual", kinds)

    def test_unknown_message(self):
        r = self.gw.handle_message({"type": "nope"})
        self.assertEqual(r["reason"], "unknown_message")

    def test_not_allowed_command(self):
        r = self.cmd("pause")
        self.assertEqual((r["ok"], r["reason"]), (False, "not_allowed_in_mode"))

    def test_video_command_without_receiver(self):
        r = self.cmd("video_on")
        self.assertEqual((r["ok"], r["reason"]), (False, "video_disabled"))   # --no-video
        self.assertEqual(self.gw.video_frame(), (None, "video_disabled"))
        self.assertNotIn("streamon", self.fake.received)


class StubVideo:
    """TelloVideo 대신. 수신 소켓·PyAV 없이 켜짐 여부만 흉내 낸다."""
    def __init__(self):
        self.running = False
        self.size = (960, 720)
        self.snap = None   # (jpeg, 디코딩 monotonic 시각, wall 시각)

    def push(self, jpeg=b"jpg"):  # Windows monotonic은 약 15ms 단위라 연달아 넣어도 다른 프레임이 되게 한다
        stamp = max(time.monotonic(), self.snap[1] + 0.001 if self.snap else 0.0)
        self.snap = (jpeg, stamp, time.time())

    def jpeg_snapshot(self):
        return self.snap

    def start(self):
        self.running = True

    def stop(self):
        self.running = False

    def stats(self):
        return {"running": self.running, "fps": None, "frame_age_s": None}


class GatewayVideoTest(GatewayCase):
    """영상은 자동으로 켜지 않는다: 화면 요청·이륙 시 켜고, 지상에서 오래 켜져 있으면 경고 후 끈다."""
    def make_video(self):
        self.video = StubVideo()
        return self.video

    def setUp(self):
        self._saved = (gateway.VIDEO_GROUND_WARN_S, gateway.VIDEO_GROUND_OFF_S)
        gateway.VIDEO_GROUND_WARN_S, gateway.VIDEO_GROUND_OFF_S = 0.5, 1.0
        super().setUp()

    def tearDown(self):
        super().tearDown()
        gateway.VIDEO_GROUND_WARN_S, gateway.VIDEO_GROUND_OFF_S = self._saved

    def video_cmds(self):
        return [c for c in self.fake.received if c in ("streamon", "streamoff")]

    def test_no_auto_start(self):
        self.assertTrue(wait_until(lambda: "streamoff" in self.fake.received), self.fake.received)  # 시작 시 꺼진 상태로 맞춤
        time.sleep(0.3)
        self.assertNotIn("streamon", self.fake.received)
        st = self.gw.drone_state()
        self.assertEqual((st["video"]["enabled"], st["video"]["on"], st["video"]["available"]), (True, False, False))
        self.assertNotIn("video_stale", st["warnings"])

    def test_operator_on_off(self):
        r = self.cmd("video_on")
        self.assertTrue(r["ok"], r)
        self.assertTrue(self.video.running)
        st = self.gw.drone_state()
        self.assertEqual((st["video"]["on"], st["video"]["reason"]), (True, "operator"))
        self.assertTrue(self.cmd("video_off")["ok"])
        self.assertEqual(self.video_cmds()[-2:], ["streamon", "streamoff"])
        self.assertFalse(self.gw.drone_state()["video"]["on"])
        changed = [m["detail"] for m in self.sent if m.get("kind") == "video_changed"]
        self.assertEqual(changed, [{"on": True, "reason": "operator"}, {"on": False, "reason": "operator"}])

    def test_takeoff_turns_video_on(self):
        self.assertTrue(wait_until(lambda: "streamoff" in self.fake.received))
        self.take_off()
        self.assertTrue(wait_until(lambda: self.gw.drone_state()["video"]["on"]), self.fake.received)
        self.assertEqual(self.gw.drone_state()["video"]["reason"], "takeoff")
        received = self.fake.received
        self.assertGreater(received.index("streamon"), received.index("takeoff"))  # 이륙 뒤에 켠다
        time.sleep(1.3)  # 비행 중에는 지상 시간을 세지 않는다
        st = self.gw.drone_state()
        self.assertTrue(st["video"]["on"])
        self.assertIsNone(st["video"]["ground_s"])

    def test_ground_timeout_turns_video_off(self):
        self.assertTrue(self.cmd("video_on")["ok"])
        self.assertTrue(wait_until(lambda: "video_on_ground" in self.gw.drone_state()["warnings"], timeout=2))
        self.assertTrue(self.gw.drone_state()["video"]["on"])
        self.assertTrue(wait_until(lambda: not self.gw.drone_state()["video"]["on"], timeout=3))
        st = self.gw.drone_state()
        self.assertEqual(st["video"]["reason"], "ground_timeout")
        self.assertNotIn("video_on_ground", st["warnings"])
        self.assertEqual(self.video_cmds()[-1], "streamoff")
        self.assertIn({"on": False, "reason": "ground_timeout"},
                      [m["detail"] for m in self.sent if m.get("kind") == "video_changed"])

    def test_frame_ids(self):
        self.assertEqual(self.gw.video_frame(), (None, "video_off"))
        self.video.push(b"before")              # 켜기 전 수신기에 남은 프레임은 내주지 않는다
        self.assertTrue(self.cmd("video_on")["ok"])
        self.assertEqual(self.gw.video_frame(), (None, "no_frame"))
        self.video.push(b"a")
        f1, _ = self.gw.video_frame()
        self.assertEqual((f1["frame_id"], f1["jpeg"], f1["width"]), ("frame-000001", b"a", 960))
        self.assertTrue(f1["input_id"].endswith("-video1"))
        self.assertIs(self.gw.video_frame()[0], f1)   # 같은 프레임을 다시 가져가면 같은 번호
        self.video.push(b"b")
        f2, _ = self.gw.video_frame()
        self.assertEqual(f2["frame_id"], "frame-000002")
        self.assertIs(self.gw.frames.get(f1["input_id"], "frame-000001"), f1)
        saved, gateway.VIDEO_STALE_S = gateway.VIDEO_STALE_S, 0.05
        try:
            time.sleep(0.1)                     # 새 프레임이 끊김
            self.assertEqual(self.gw.video_frame(), (None, "video_stale"))
        finally:
            gateway.VIDEO_STALE_S = saved
        self.assertTrue(self.cmd("video_off")["ok"])
        self.assertEqual(self.gw.video_frame(), (None, "video_off"))
        self.assertTrue(self.cmd("video_on")["ok"])   # 다시 켜면 새 세션, 이전 프레임은 찾을 수 없다
        self.video.push(b"c")
        f3, _ = self.gw.video_frame()
        self.assertEqual((f3["frame_id"], f3["input_id"].endswith("-video2")), ("frame-000001", True))
        self.assertIsNone(self.gw.frames.get(f1["input_id"], "frame-000001"))


if __name__ == "__main__":
    unittest.main()
