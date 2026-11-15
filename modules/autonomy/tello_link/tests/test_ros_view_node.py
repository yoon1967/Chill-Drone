"""자율 비행(ROS) 보기 노드의 ROS 없는 부분(ros_view_core.py)과 가짜 노드(fake_ros_view.py)를 시험한다.

보기 노드는 자율 비행 프로그램 토픽을 그대로 받아 HTTP로 내주기만 한다(게이트웨이 --ros-view가 가져감).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_ros_view_node.py" -v
"""

import ast
import json
import math
import os
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import fake_ros_view  # noqa: E402
from ros_view_core import BOUNDARY, ViewState, encode_jpeg, make_server, scan_range  # noqa: E402
from tello_link import parse_state  # noqa: E402

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GROUND = "mid:-1;x:-100;y:-100;z:-100;mpry:0,0,0;pitch:0;roll:0;yaw:5;vgx:0;vgy:0;vgz:0;templ:60;temph:63;tof:10;h:0;bat:87;baro:12.50;time:0;agx:0.00;agy:0.00;agz:-1000.00;"  # noqa: E501
FLYING = GROUND.replace("tof:10;h:0;", "tof:92;h:90;").replace("time:0;", "time:12;")
CHECKS = [{"key": "battery", "title": "배터리 10% 초과 (이하면 바로 착륙)", "ok": True, "detail": "87%", "blocking": True}]


def can_encode() -> bool:
    return encode_jpeg(2, 2, "rgb8", bytes(12)) is not None


def image(w=8, h=6, pad=0) -> bytes:
    row = bytes((x * 30) % 256 for x in range(w * 3)) + bytes(pad)
    return row * h


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class Tap:
    """시험용 보기 노드: ViewState를 시험이 직접 채운다."""

    def __init__(self, testcase):
        self.view = ViewState()
        self.server = make_server(self.view, "127.0.0.1", 0)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        testcase.addCleanup(self.close)

    def close(self):
        if self.server is not None:
            self.server.stopping.set()
            self.server.shutdown()
            self.server.server_close()
            self.server = None

    def state(self, raw, flying=False, busy=None, connected=True, **extra):
        self.view.on_state(json.dumps({"stamp": time.monotonic(), "raw": raw}))
        self.view.on_flight_status(json.dumps({"flying": flying, "busy": busy, "connected": connected,
                                               "preflight": CHECKS, "rc_limit": 80, **extra}))


class CoreTest(unittest.TestCase):
    def test_scan_range(self):
        self.assertIsNone(scan_range([]))
        self.assertIsNone(scan_range([float("nan")]))
        self.assertTrue(math.isinf(scan_range([float("inf")])))   # 범위 밖은 inf 그대로
        self.assertEqual(scan_range([0.48]), 0.48)

    def test_encode_jpeg(self):
        self.assertIsNone(encode_jpeg(8, 6, "mono8", image()))        # 모르는 형식
        self.assertIsNone(encode_jpeg(8, 6, "rgb8", image()[:-1]))    # 모자란 바이트
        if not can_encode():
            self.skipTest("OpenCV·Pillow 없음")
        self.assertTrue(encode_jpeg(8, 6, "rgb8", image()).startswith(b"\xff\xd8"))
        self.assertTrue(encode_jpeg(8, 6, "bgr8", image(pad=4), step=8 * 3 + 4).startswith(b"\xff\xd8"))   # 줄 끝 여분

    def test_state_and_snapshot(self):
        clock = Clock()
        v = ViewState(clock=clock)
        v.on_state("not json")
        v.on_state(json.dumps({"stamp": 1.5, "raw": GROUND}))
        v.on_flight_status(json.dumps({"flying": False, "connected": True}))
        v.on_front_tof(float("inf"))
        clock.t += 0.25
        snap = v.snapshot()
        self.assertEqual((snap["bad"], snap["state"]["raw"], snap["state"]["ros_stamp"], snap["state"]["age_s"]), (1, GROUND, 1.5, 0.25))
        self.assertEqual((snap["flight_status"]["flying"], snap["flight_status"]["age_s"]), (False, 0.25))
        self.assertEqual((snap["front_tof"]["m"], snap["front_tof"]["inf"]), (None, True))
        self.assertEqual(snap["topics"]["/tt/state"]["count"], 1)
        self.assertIsNone(snap["video"])

    def test_jpeg_is_made_once_per_frame(self):
        if not can_encode():
            self.skipTest("OpenCV·Pillow 없음")
        v = ViewState()
        self.assertIsNone(v.jpeg())
        v.on_image(8, 6, "rgb8", image())
        a, b = v.jpeg(), v.jpeg()
        self.assertEqual(a[0], 1)
        self.assertIs(a[1], b[1])          # 같은 장은 한 번만 바꾼다
        self.assertFalse(v.wait_frame(1, 0.05))
        v.on_image(8, 6, "rgb8", image())
        self.assertTrue(v.wait_frame(1, 0.05))
        self.assertEqual(v.jpeg()[0], 2)


class HttpTest(unittest.TestCase):
    def test_paths(self):
        tap = Tap(self)
        with urllib.request.urlopen(tap.url + "/api/state", timeout=5) as r:
            self.assertEqual(json.loads(r.read())["schema_version"], "ros-view/0.1")
        with self.assertRaises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(tap.url + "/frame.jpg", timeout=5)
        self.assertEqual(e.exception.code, 503)                  # 아직 그림 없음
        with self.assertRaises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(tap.url + "/nope", timeout=5)
        self.assertEqual(e.exception.code, 404)

    def test_mjpeg_sends_each_new_frame(self):
        if not can_encode():
            self.skipTest("OpenCV·Pillow 없음")
        tap = Tap(self)
        tap.view.on_image(8, 6, "rgb8", image())
        with urllib.request.urlopen(tap.url + "/frame.jpg", timeout=5) as r:
            self.assertEqual((r.headers["X-Frame-Seq"], r.read()[:2]), ("1", b"\xff\xd8"))
        with urllib.request.urlopen(tap.url + "/video.mjpg", timeout=5) as r:
            self.assertIn(BOUNDARY, r.headers["Content-Type"])
            threading.Timer(0.2, lambda: tap.view.on_image(8, 6, "rgb8", image())).start()
            seqs = []
            while len(seqs) < 2:
                line = r.readline()
                if line.startswith(b"X-Frame-Seq:"):
                    seqs.append(int(line.split(b":")[1]))
        self.assertEqual(seqs, [1, 2])


class FakeNodeTest(unittest.TestCase):
    def test_fake_values_look_like_the_driver(self):
        self.assertEqual(fake_ros_view.phase_at(1.0)[0], "ground")
        self.assertEqual(fake_ros_view.phase_at(fake_ros_view.GROUND_S + 0.5)[0], "takeoff")
        sample = parse_state(fake_ros_view.tello_raw(20.0, "flying", 5.0))
        self.assertTrue(sample.valid)
        self.assertGreater(sample.si["tof_m"], 0.8)
        fs = fake_ros_view.flight_status("takeoff", 80, 60)
        self.assertEqual((fs["flying"], fs["busy"]), (False, "takeoff"))   # FlightController처럼 이륙 중엔 아직 안 떠 있음

    def test_ros_node_subscribes_to_the_driver_topics(self):   # rclpy가 없어 불러오지 않고 읽기만 한다
        with open(os.path.join(HERE, "ros_view_node.py"), encoding="utf-8") as f:
            src = f.read()
        ast.parse(src)
        for topic in ("/tt/state", "/tt/flight_status", "/tt/tof_front", "/tt/camera/image_raw"):
            self.assertIn(topic, src)
        self.assertNotIn("create_publisher", src)   # 보기만 한다(1단계)


if __name__ == "__main__":
    unittest.main()
