"""지도 연결기(gateway_map_bridge.py)를 가짜 지도 서버(HTTP)에 붙여 새 버전만 넘기는지 본다 (표준 라이브러리만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_map_bridge.py" -v
"""

import os
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fake_map_server import BOXES, FakeMap, make_handler  # noqa: E402
from gateway_map_bridge import MapBridge  # noqa: E402
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

    def test_unreachable_is_reported(self):
        self.server.shutdown()
        self.server.server_close()
        self.server = None
        self.bridge.poll_once()
        st = self.inbox.status()
        self.assertEqual((st["upstream"], st["alive"]), ("unreachable", True))   # 연결기는 살아 있고 지도 서버가 꺼짐


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
        self.fake.stop(sid)                   # 끝나면 map_available → 전체 결과(/run)
        self.bridge.poll_once()
        self.assertEqual(self.inbox.latest()["source"]["schema_version"], "phone-sfm-result/1.2")

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
