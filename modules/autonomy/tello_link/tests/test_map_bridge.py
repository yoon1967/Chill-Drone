"""지도 연결기(gateway_map_bridge.py)를 가짜 지도 서버(HTTP)에 붙여 새 버전만 넘기는지 본다 (표준 라이브러리만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_map_bridge.py" -v
"""

import os
import sys
import threading
import unittest
import urllib.request
from http.server import ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fake_map_server import BOXES, FakeMap, make_handler  # noqa: E402
from gateway_map_bridge import MapBridge, compact_preview  # noqa: E402
from map_inbox import MapInbox  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class MapBridgeTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.fake = FakeMap(BOXES, clock=self.clock)
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(self.fake))
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"
        self.inbox = MapInbox(clock=self.clock)
        self.sent = []

        def deliver(msg):
            self.sent.append(msg["kind"])
            return self.inbox.receive(msg)
        self.bridge = MapBridge(self.url, deliver, log=lambda text: None, clock=self.clock)

    def tearDown(self):
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()

    def test_root_page_explains_fake_server(self):   # 관제 화면 "지도 콘솔" 보기가 이 주소를 열면 빈 오류 대신 안내
        with urllib.request.urlopen(self.url + "/", timeout=5) as r:
            self.assertTrue(r.headers["Content-Type"].startswith("text/html"))
            self.assertIn("가짜 지도 서버", r.read().decode("utf-8"))

    def test_no_map_until_session_then_new_revisions_only(self):
        self.bridge.poll_once()
        self.assertEqual((self.inbox.status()["upstream"], self.inbox.latest()), ("no_map", None))   # 지도 만들기 전
        self.fake.start()                    # 지도 화면에서 드론 지도를 시작한 상태
        self.bridge.poll_once()
        first = self.inbox.latest()
        self.assertEqual((first["source"]["revision"], first["scale"]["unit"], first["up"]), (1, "arbitrary", [0.0, 0.0, 1.0]))
        self.bridge.poll_once()              # 같은 버전은 점군을 다시 보내지 않는다
        self.assertEqual(self.sent.count("snapshot"), 1)
        self.clock.t += 5
        self.bridge.poll_once()
        self.assertEqual(self.inbox.latest()["source"]["revision"], 6)
        self.assertGreater(len(self.inbox.latest()["points"]), len(first["points"]))   # 지나간 만큼 벽 점이 늘어난다
        self.assertEqual(self.inbox.status()["upstream"], "ok")

    def test_too_many_points_are_cut(self):
        self.fake.start()
        self.clock.t += 10
        bridge = MapBridge(self.url, self.inbox.receive, max_points=50, log=lambda text: None, clock=self.clock)
        bridge.poll_once()
        snap = self.inbox.latest()
        self.assertEqual(len(snap["points"]), 50)
        self.assertGreater(snap["points_total"], 50)

    def test_metric_units_survive_a_tello_pose_gap(self):
        preview = {"map_id": 0, "map_unit": "m", "tracking": "sensor_wait",
                   "points": [[0, 0, 1, 80, 90, 100]]}
        message = compact_preview(preview, {}, self.url, 50)
        self.assertEqual(message["scale"]["unit"], "m")
        self.assertEqual(message["source"]["pose_source"], "tello_sensor_rgbd")
        self.assertEqual(message["points"], preview["points"])

    def test_unreachable_is_reported(self):
        self.server.shutdown()
        self.server.server_close()
        self.server = None
        self.bridge.poll_once()
        st = self.inbox.status()
        self.assertEqual((st["upstream"], st["alive"]), ("unreachable", True))   # 연결기는 살아 있고 지도 서버가 꺼짐

    def test_video_input_state_reaches_the_gateway(self):   # 관제 화면 "영상 기다리는 중"
        self.fake.start()
        self.clock.t += 3
        self.bridge.poll_once()
        st = self.inbox.status()
        self.assertEqual((st["input_state"], st["input_age_s"]), ("receiving", 0.0))
        self.fake.input_state, self.fake.input_age_s = "reconnecting", 7.46   # 드론 영상이 끊겨 지도 서버가 기다림
        self.clock.t += 2   # 깊이 지도 복원 중에도 2초 안에는 알게 된다
        self.bridge.poll_once()
        st = self.inbox.status()
        self.assertEqual((st["input_state"], st["input_age_s"]), ("reconnecting", 7.5))
        self.fake.input_state = "receiving"
        self.clock.t += 2
        self.bridge.poll_once()
        self.assertEqual(self.inbox.status()["input_state"], "receiving")


class DepthMapBridgeTest(MapBridgeTest):
    """지금 지도 서버(-365 깊이 지도): 복원이 끝나야 결과가 생기고, 그 전에는 진행 중 미리보기만 있다."""

    def setUp(self):
        super().setUp()
        self.fake.depth = True

    def test_no_map_until_session_then_new_revisions_only(self):
        self.bridge.poll_once()
        self.assertEqual((self.inbox.status()["upstream"], self.inbox.latest()), ("no_map", None))
        self.fake.start()
        self.bridge.poll_once()               # 복원 중: 미리보기에서 점군을 받는다
        first = self.inbox.latest()
        self.assertEqual((first["source"]["schema_version"], first["source"]["revision"], first["up"]),
                         ("live-depth-preview", 1, [0.0, -1.0, 0.0]))
        self.assertEqual(len(first["points"][0]), 6)
        self.bridge.poll_once()               # 같은 버전은 다시 보내지 않는다
        self.assertEqual(self.sent.count("snapshot"), 1)
        self.clock.t += 5
        self.bridge.poll_once()
        self.assertEqual(self.inbox.latest()["source"]["revision"], 6)
        self.assertGreater(len(self.inbox.latest()["points"]), len(first["points"]))
        self.assertEqual(self.inbox.status()["upstream"], "ok")

    def test_final_result_after_session_ends(self):
        sid = self.fake.start()["session_id"]
        self.clock.t += 3
        self.bridge.poll_once()
        self.assertEqual(self.inbox.latest()["source"]["schema_version"], "live-depth-preview")
        self.fake.stop(sid)                   # 끝나면 map_available → 전체 결과(/run). 2초를 안 기다리고 바로 넘어간다
        self.bridge.poll_once()
        self.assertEqual(self.inbox.latest()["source"]["schema_version"], "phone-sfm-result/1.2")

    def test_state_is_read_every_2s_while_previews_every_poll(self):
        # 지도 서버는 상태를 줄 때마다 복원 진행 파일을 읽어서, Windows에서 복원 쪽이 그 파일을 못 바꾸는 일이 생긴다(WinError 5).
        # 복원 중에는 상태를 2초에 한 번만 읽고, 미리보기(그 파일을 안 읽음)는 매번 본다.
        calls = []
        state, preview = self.fake.state, self.fake.preview
        self.fake.state = lambda: (calls.append("state"), state())[1]
        self.fake.preview = lambda job, since: (calls.append("preview"), preview(job, since))[1]
        self.fake.start()
        for _ in range(4):
            self.bridge.poll_once()
            self.clock.t += 0.4
        self.assertEqual((calls.count("state"), calls.count("preview")), (1, 4))
        self.clock.t += 0.5   # 처음 읽은 지 2초가 넘음
        self.bridge.poll_once()
        self.assertEqual((calls.count("state"), calls.count("preview")), (2, 5))
        self.assertEqual(self.inbox.latest()["source"]["revision"], 3)   # 미리보기는 그동안에도 새 버전을 받는다

    def test_too_many_points_are_cut(self):
        self.fake.start()
        self.clock.t += 10
        bridge = MapBridge(self.url, self.inbox.receive, max_points=50, log=lambda text: None, clock=self.clock)
        bridge.poll_once()
        snap = self.inbox.latest()
        self.assertLessEqual(len(snap["points"]), 50)
        self.assertGreater(snap["points_total"], 50)

if __name__ == "__main__":
    unittest.main()
