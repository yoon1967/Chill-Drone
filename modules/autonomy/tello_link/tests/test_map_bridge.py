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
from gateway_map_bridge import DEFAULT_MAX_POINTS, MapBridge, compact_preview, compact_scene  # noqa: E402
from map_inbox import MAX_POINTS, MapInbox  # noqa: E402


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
        self.gateway_up = True

        def deliver(msg):
            self.sent.append(msg["kind"])
            if not self.gateway_up:   # 꺼진 게이트웨이: 따로 실행의 deliver처럼 연결 실패는 상태 0
                return 0, None
            return self.inbox.receive(msg)
        self.bridge = MapBridge(self.url, deliver, log=lambda text: None, clock=self.clock)

    def tearDown(self):
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()

    def test_root_page_explains_fake_server(self):   # 관제 화면 "지도 콘솔" 보기가 이 주소를 열면 빈 오류 대신 안내
        # 지도 콘솔 탭은 진짜 콘솔의 영상 복원 화면(/reconstruct.html)을 연다(-415). 화면 주소는 모두 안내로 답한다
        for page in ("/", "/reconstruct.html", "/reconstruct.html?from=controller"):
            with urllib.request.urlopen(self.url + page, timeout=5) as r:
                self.assertTrue(r.headers["Content-Type"].startswith("text/html"), page)
                self.assertIn("가짜 지도 서버", r.read().decode("utf-8"))
        with self.assertRaises(urllib.error.HTTPError) as e:   # 모르는 API 주소는 그대로 404
            urllib.request.urlopen(self.url + "/api/nope", timeout=5)
        self.assertEqual(e.exception.code, 404)

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

    def settle(self, sid):
        """가짜 카메라가 복도 끝까지 가서 지도 버전이 더 안 바뀌게 한다. 세션은 살려 둔다(깊이 지도는 복원 중 그대로)."""
        while self.fake.revision() < len(self.fake.path):
            self.clock.t += 10
            self.fake.heartbeat(sid)

    def test_restarted_gateway_gets_the_map_again(self):
        # 게이트웨이만 다시 켜면 받은 지도가 없다. 지도 버전이 더 안 바뀌어도(복원 끝·저장 지도) 지금 지도를 다시 보낸다.
        self.settle(self.fake.start()["session_id"])
        self.bridge.poll_once()
        first = self.inbox.latest()
        self.inbox = MapInbox(clock=self.clock)   # 두 번 읽는 사이에 다시 켬: 보내기 실패가 한 번도 없었다
        self.clock.t += 2                         # 상태를 알릴 때 → 새 게이트웨이가 받은 지도가 없다고 답한다
        self.bridge.poll_once()
        self.assertIsNone(self.inbox.latest())
        self.bridge.poll_once()                   # 다음 차례에 지금 지도를 처음처럼 다시 보낸다
        again = self.inbox.latest()
        self.assertIsNotNone(again)
        self.assertEqual((again["source"]["revision"], len(again["points"])),
                         (first["source"]["revision"], len(first["points"])))
        self.clock.t += 2
        self.bridge.poll_once()                   # 받은 게이트웨이에는 다시 보내지 않는다
        self.assertEqual(self.sent.count("snapshot"), 2)

    def test_map_waits_while_gateway_is_down_then_is_sent_again(self):
        self.settle(self.fake.start()["session_id"])
        self.bridge.poll_once()
        first = self.inbox.latest()
        self.gateway_up = False                   # 게이트웨이가 5초 꺼짐
        for _ in range(10):
            self.clock.t += 0.5
            self.bridge.poll_once()
        self.assertGreater(self.sent.count("status"), 0)
        self.assertEqual(self.sent.count("snapshot"), 1)   # 꺼진 동안은 지도를 다시 읽어 보내려 하지 않는다(상태만 2초마다)
        self.inbox, self.gateway_up = MapInbox(clock=self.clock), True
        for _ in range(6):                        # 다시 켠 뒤 첫 상태 응답(지도 없음) → 다음 차례에 한 번 보낸다
            self.clock.t += 0.5
            self.bridge.poll_once()
        self.assertEqual(self.sent.count("snapshot"), 2)
        self.assertEqual(self.inbox.latest()["source"]["revision"], first["source"]["revision"])


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
        self.assertEqual(len(snap["points"]), 50)   # 전에는 간격으로 솎아 50보다 적게 갔다
        self.assertGreater(snap["points_total"], 50)


class PickSpreadTest(unittest.TestCase):
    """점이 많을 때 고르기(11/17): 앞에서부터 자르지 않고 지도 전체에서 고르게 고른다."""

    @staticmethod
    def depth_scene(n):
        # 실시간 깊이 지도 저장 결과처럼: 오차는 모두 0, 관측 수는 모두 1, 점은 x 차례로 놓임
        return {"schema_version": "phone-sfm-result/1.2", "segment_id": 0, "revision": 9,
                "scale": {"status": "tello_metric_estimate", "unit": "m"},
                "points": [[i, i * 0.01, 0.0, 1.0, 100, 100, 100, 0.0, 1] for i in range(n)], "cameras": []}

    def test_equal_errors_are_spread_over_the_whole_map(self):
        msg = compact_scene(self.depth_scene(1000), {"session_id": "s"}, "http://m", "live", max_points=100)
        xs = [p[0] for p in msg["points"]]
        self.assertEqual((len(xs), msg["points_total"]), (100, 1000))
        self.assertLess(min(xs), 0.5)          # 전에는 앞의 100개(x 0~0.99)만 갔다
        self.assertGreater(max(xs), 9.4)
        self.assertEqual(xs, sorted(xs))       # 고른 점은 원래 차례대로
        self.assertLess(max(b - a for a, b in zip(xs, xs[1:])), 0.3)   # 크게 빈 곳이 없다(고른 간격이면 0.1)
        self.assertEqual(MapInbox().receive(msg)[0], 201)

    def test_low_error_points_still_come_first(self):
        run = self.depth_scene(1000)
        for p in run["points"]:
            p[7] = 1.0
        for i in (990, 995, 999):              # 끝쪽 세 점만 오차가 작다
            run["points"][i][7] = 0.2
        xs = [p[0] for p in compact_scene(run, {}, "http://m", "live", max_points=10)["points"]]
        for i in (990, 995, 999):
            self.assertIn(round(i * 0.01, 4), xs)
        self.assertLess(min(xs), 5.0)          # 나머지는 앞쪽에서도 고른다

    def test_preview_sends_exactly_the_limit(self):
        preview = {"map_id": 0, "map_unit": "m", "points": [[i * 0.01, 0, 1, 10, 20, 30] for i in range(1000)]}
        xs = [p[0] for p in compact_preview(preview, {}, "http://m", 300)["points"]]
        self.assertEqual(len(xs), 300)         # 전에는 간격 4로 솎아 250개
        self.assertGreater(max(xs) - min(xs), 9.4)

    def test_default_sends_the_whole_live_map_within_the_gateway_limit(self):
        self.assertGreaterEqual(DEFAULT_MAX_POINTS, 34_000)   # 지도 쪽 실시간 최대(정적 3만 2천 + 사람 2천)를 다 보낸다
        self.assertLessEqual(DEFAULT_MAX_POINTS, MAX_POINTS)  # 게이트웨이가 받는 최대 안
        self.assertEqual(MapBridge("http://m", lambda msg: (200, {})).max_points, DEFAULT_MAX_POINTS)


if __name__ == "__main__":
    unittest.main()
