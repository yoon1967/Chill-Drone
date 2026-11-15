"""자율 비행(ROS) 보기: 게이트웨이 보기 모드(ros_view.py, 서버 --ros-view)를 ROS 없이 시험한다.

게이트웨이는 드론에 붙지 않고 보기 노드(ros_view_node.py)에서 상태·영상을 받아 화면에 보인다. 명령은 받지 않는다(1단계).
보기 노드는 시험용으로 ros_view_core의 서버를 직접 채운다(test_ros_view_node.Tap).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_ros_view.py" -v
"""

import argparse
import json
import os
import shutil
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ros_view  # noqa: E402
from test_ros_view_node import CHECKS, FLYING, GROUND, Clock, Tap, can_encode, image  # noqa: E402


class GatewayTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.tap = Tap(self)
        self.clock = Clock()
        self.gw = ros_view.RosViewGateway(self.tap.url, log_root=self.tmp, clock=self.clock)
        self.addCleanup(self.gw.link.close)
        self.sent = []
        self.gw.listeners.append(self.sent.append)

    def step(self, dt=0.1):
        self.clock.t += dt
        self.gw.tick()
        return self.gw.drone_state()

    def test_modes_follow_the_autonomy_driver(self):
        st = self.step()
        self.assertEqual((st["mode"], st["ros_view"]["reason"]), ("disconnected", "autonomy_driver_silent"))   # 노드는 있고 드라이버 소식 없음
        self.tap.state(GROUND)
        st = self.step()
        self.assertEqual((st["mode"], st["source"], st["battery_pct"], st["height_m"]), ("grounded", "ros", 87, None))
        self.assertEqual(st["preflight"], CHECKS)      # 자율 드라이버의 점검(그 기준) 그대로
        self.assertEqual(st["allowed"], [])
        self.tap.state(GROUND, busy="takeoff")
        self.assertEqual(self.step()["flight_busy"], "takeoff")
        self.tap.state(FLYING, flying=True)
        st = self.step()
        self.assertEqual((st["mode"], st["authority"], st["height_m"], st["preflight"]), ("auto", "autonomy", 0.92, None))
        self.tap.state(FLYING, flying=True, busy="land")
        self.assertEqual(self.step()["mode"], "landing")
        self.tap.state(GROUND, connected=False)
        self.assertEqual((self.step()["mode"], self.gw.mode_reason), ("disconnected", "drone_not_connected"))
        changes = [m["detail"]["to"] for m in self.sent if m.get("kind") == "mode_changed"]
        self.assertEqual(changes, ["grounded", "auto", "landing", "disconnected"])

    def test_state_lost_and_node_gone(self):
        self.tap.state(GROUND)
        self.assertEqual(self.step()["mode"], "grounded")
        self.assertEqual(self.step(ros_view.STATE_STALE_S + 0.5)["mode"], "disconnected")   # 같은 원문만 오면 상태가 끊긴 것
        self.assertEqual(self.gw.mode_reason, "state_lost")
        self.tap.close()
        self.step(ros_view.TAP_STALE_S + 0.5)
        self.assertEqual((self.gw.mode, self.gw.mode_reason), ("disconnected", "ros_view_unreachable"))

    def test_front_tof(self):
        self.tap.state(FLYING, flying=True)
        self.tap.view.on_front_tof(0.613)
        tof = self.step()["front_tof"]
        self.assertEqual((tof["m"], tof["valid"], tof["reason"]), (0.613, True, None))
        self.tap.view.on_front_tof(float("inf"))
        tof = self.step()["front_tof"]
        self.assertEqual((tof["valid"], tof["reason"]), (False, "out_of_range"))

    def test_commands_are_refused(self):
        r = self.gw.handle_message({"type": "cmd", "id": "c1", "name": "land", "client": "monitor"})
        self.assertEqual((r["type"], r["id"], r["ok"], r["reason"]), ("result", "c1", False, "ros_view_read_only"))
        self.assertIsNone(self.gw.handle_message({"type": "rc", "axes": [0, 1, 0, 0]}))
        self.assertFalse(self.gw.autonomy_open("au-1", print, "127.0.0.1"))

    def test_run_folder_can_be_replayed(self):
        from replay import Recording
        self.tap.state(GROUND)
        self.step()
        self.tap.state(FLYING, flying=True)
        self.step()
        self.gw.link.close()
        run_dir = self.gw.link.log.dir
        with open(os.path.join(run_dir, "manifest.json"), encoding="utf-8") as f:
            self.assertEqual(json.load(f)["source"], "ros")
        rec = Recording(run_dir)
        self.assertEqual(len(rec.sensor), 2)
        self.assertEqual([m["to"] for m in rec.modes], ["grounded", "auto"])
        with open(os.path.join(run_dir, "metrics.json"), encoding="utf-8") as f:
            self.assertEqual(json.load(f)["states"], 2)

    def test_video_frames_for_detection(self):
        if not can_encode():
            self.skipTest("OpenCV·Pillow 없음")
        self.gw.clock = time.monotonic   # 영상 나이는 실제 시각으로 본다
        self.gw.video.clock = time.monotonic
        self.gw.video.start()
        self.addCleanup(self.gw.video.stop)
        deadline = time.monotonic() + 5
        while self.gw.video.jpeg() is None and time.monotonic() < deadline:
            self.tap.view.on_image(8, 6, "rgb8", image())
            time.sleep(0.05)
        self.assertIsNotNone(self.gw.video.jpeg())
        frame, reason = self.gw.video_frame()
        self.assertIsNone(reason)
        self.assertTrue(frame["input_id"].startswith(self.gw.link.run_id))
        self.assertTrue(self.gw.drone_state()["video"]["on"])


class ServerTest(unittest.TestCase):
    def test_state_route_and_autonomy_busy(self):
        try:
            from fastapi.testclient import TestClient
        except (ImportError, RuntimeError) as e:
            self.skipTest(f"TestClient 없음: {e}")
        import gateway_server
        tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, tmp, True)
        args = argparse.Namespace(ros_view="http://127.0.0.1:9", log_root=tmp, fake=False, replay=None, no_video=False,
                                  allow_remote_autonomy=True)
        app = gateway_server.build_app(args)
        self.addCleanup(app.state.gateway.link.close)
        client = TestClient(app)   # with 없이: 보기 노드를 묻는 스레드를 켜지 않는다
        st = client.get("/api/drone/state").json()
        self.assertEqual((st["source"], st["mode"]), ("ros", "disconnected"))
        with client.websocket_connect("/ws/autonomy") as ws:
            self.assertEqual(ws.receive_json()["reason"], "busy")


if __name__ == "__main__":
    unittest.main()
