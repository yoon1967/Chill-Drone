"""지도 그림(board.py, /api/board): 진입로·사람 동선을 게이트웨이에 두고 모든 화면에 알린다.

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_board.py" -v
"""

import argparse
import json
import math
import os
import shutil
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import board as board_mod  # noqa: E402
from board import Board  # noqa: E402
from route_plan import BLOCKED  # noqa: E402
from test_route_plan import Room, two_rooms  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class BoardCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.sent = []
        self.grid = None
        self.found = None
        self.clock = Clock()
        self.board = Board(self.tmp, "ros_test", self.sent.append, grid=lambda: self.grid,
                           discovered=lambda: self.found, clock=self.clock)

    def add(self, item, client="monitor"):
        return self.board.apply({"op": "add", "client": client, "item": item})

    def entry(self, x=1.0, y=2.0, **kw):
        status, out = self.add({"kind": "entry", "pts": [[x, y]], **kw})
        self.assertEqual(status, 200, out)
        return out["item"]

    def lines(self):
        with open(os.path.join(self.tmp, "board.jsonl"), encoding="utf-8") as f:
            return [json.loads(line) for line in f]


class BoardTest(BoardCase):
    def test_entry_is_saved_recorded_and_announced(self):
        item = self.entry()
        self.assertEqual((item["id"], item["kind"], item["n"], item["text"], item["pts"]), ("b-0001", "entry", 1, "진입 1", [[1.0, 2.0]]))
        self.assertEqual(self.entry(2.0, 2.0, text="  뒷문\n(잠김) ")["text"], "뒷문 (잠김)")   # 줄바꿈은 띄어쓰기로
        msg = self.sent[0]
        self.assertEqual((msg["type"], msg["op"], msg["rev"], msg["client"], msg["logged"]), ("board", "add", 1, "monitor", True))
        self.assertEqual([m["item"]["id"] for m in self.lines()], ["b-0001", "b-0002"])   # board.jsonl은 ws와 같은 줄
        with open(os.path.join(self.tmp, "board.json"), encoding="utf-8") as f:
            saved = json.load(f)
        self.assertEqual((saved["rev"], len(saved["items"]), saved["frame"]), (2, 2, "map"))
        snap = self.board.snapshot()
        self.assertEqual((snap["type"], snap["run_id"], snap["rev"], snap["width_m"], snap["walk_mps"]),
                         ("board", "ros_test", 2, 0.6, 0.5))

    def test_route_from_an_entry_to_a_person(self):
        self.grid = two_rooms(8).grid(ver=5)
        e = self.entry(1.0, 2.0)
        status, out = self.add({"kind": "route", "from": {"type": "entry", "id": e["id"]},
                                "to": {"type": "person", "n": 2, "x": 5.0, "y": 2.0}})
        self.assertEqual(status, 200, out)
        r = out["item"]
        self.assertEqual((r["kind"], r["n"], r["from"]["label"], r["to"]["label"]), ("route", 1, "진입 1", "사람 2"))
        self.assertEqual((r["from"]["x"], r["from"]["y"]), (1.0, 2.0))       # 진입로 자리를 그대로
        self.assertEqual((r["result"]["state"], r["result"]["grid_ver"]), ("ok", 5))
        self.assertAlmostEqual(r["result"]["length_m"], 4.0, delta=0.2)
        self.assertEqual(r["result"]["walk_s"], round(r["result"]["length_m"] / 0.5, 1))
        status, out = self.add({"kind": "route", "from": {"type": "start", "x": 1.0, "y": 1.0},
                                "to": {"type": "point", "x": 4.0, "y": 1.0}})
        self.assertEqual((out["item"]["n"], out["item"]["from"]["label"], out["item"]["to"]["label"]), (2, "출발", "지정 위치 1"))

    def test_route_waits_for_the_map_then_follows_it(self):   # 지도가 없으면 no_map, 지도가 오면 다시 계산해 알린다
        _, out = self.add({"kind": "route", "from": {"type": "point", "x": 1.0, "y": 2.0},
                           "to": {"type": "point", "x": 5.0, "y": 2.0}})
        self.assertEqual((out["item"]["result"]["state"], out["item"]["result"]["reason"]), ("no_map", "no_grid"))
        self.sent.clear()
        self.grid = two_rooms(8).grid(ver=1)
        self.board.tick()
        self.assertEqual([(m["op"], m["client"], m["logged"], m["item"]["result"]["state"]) for m in self.sent],
                         [("route", "gateway", True, "ok")])
        self.sent.clear()
        self.clock.t += 5
        self.grid = two_rooms(8).grid(ver=2)               # 번호만 바뀌고 길은 같다: 조용히
        self.board.tick()
        self.assertEqual(self.sent, [])
        self.grid = two_rooms(6).grid(ver=3)               # 문이 좁아짐(0.6 m): 막힘. 3초 안에는 다시 계산하지 않는다
        self.board.tick()
        self.assertEqual(self.sent, [])
        self.clock.t += 3.5
        self.board.tick()
        self.assertEqual([(m["op"], m["logged"], m["item"]["result"]["state"]) for m in self.sent], [("route", True, "partial")])
        self.assertEqual(self.board.snapshot()["items"][0]["result"]["state"], "partial")
        self.sent.clear()
        self.clock.t += 5
        self.grid = None                                    # 지도가 잠깐 없으면 마지막 결과를 둔다
        self.board.tick()
        self.assertEqual(self.sent, [])

    def test_found_obstacles_count_as_walls(self):
        self.grid = Room(60, 12).grid(ver=1)                # 폭 1 m 복도
        self.found = {"obstacles": [{"x": 3.0, "y": 0.6, "sx": 0.4, "sy": 0.4}]}
        _, out = self.add({"kind": "route", "from": {"type": "point", "x": 0.5, "y": 0.6},
                           "to": {"type": "point", "x": 5.5, "y": 0.6}})
        self.assertEqual(out["item"]["result"]["state"], "partial")

    def test_delete_hides_but_keeps_the_record(self):
        e = self.entry()
        status, out = self.board.apply({"op": "delete", "client": "control", "id": e["id"]})
        self.assertEqual((status, out["item"]["deleted"]), (200, True))
        self.assertEqual(self.board.snapshot()["items"], [])
        self.assertEqual((self.sent[-1]["op"], self.sent[-1]["client"], self.sent[-1]["logged"]), ("delete", "control", True))
        self.assertEqual([m["op"] for m in self.lines()], ["add", "delete"])
        self.assertEqual(self.board.apply({"op": "delete", "id": e["id"]})[0], 404)
        self.assertEqual(self.board.apply({"op": "delete", "id": "b-9999"})[0], 404)
        self.assertEqual(self.entry()["n"], 2)                # 번호는 지운 것도 센다(화면마다 같은 번호)

    def test_bad_items_are_refused(self):
        e = self.entry()
        self.board.apply({"op": "delete", "id": e["id"]})
        bad = [{"kind": "pen", "pts": [[0, 0]]},
               {"kind": "entry", "pts": [[math.nan, 0]]},
               {"kind": "entry", "pts": [[True, 0]]},
               {"kind": "entry", "pts": [[5000, 0]]},
               {"kind": "entry", "pts": [[0, 0], [1, 1]]},
               {"kind": "entry", "pts": [[0, 0]], "text": "가" * 61},
               {"kind": "route", "from": {"type": "entry", "id": e["id"]}, "to": {"type": "point", "x": 1, "y": 1}},   # 지운 진입로
               {"kind": "route", "from": {"type": "person", "n": 1, "x": 0, "y": 0}, "to": {"type": "point", "x": 1, "y": 1}},
               {"kind": "route", "from": {"type": "start", "x": 0, "y": 0}, "to": {"type": "person", "n": 0, "x": 1, "y": 1}},
               "entry"]
        for item in bad:
            with self.subTest(item=item):
                status, out = self.add(item)
                self.assertEqual((status, out["reason"]), (400, "invalid_item"))
        self.assertEqual(self.board.apply({"op": "move"})[1]["reason"], "invalid_op")
        self.assertEqual(self.board.apply([])[0], 400)
        self.assertEqual(self.board.snapshot()["rev"], 2)     # 거절한 것은 판 번호를 올리지 않는다

    def test_too_many_items(self):
        for _ in range(board_mod.MAX_ITEMS):
            self.entry()
        status, out = self.add({"kind": "entry", "pts": [[0, 0]]})
        self.assertEqual((status, out["reason"]), (413, "too_many_items"))

    def test_store_failure_keeps_screens_and_record_together(self):   # 기록에 못 남기면 넣지 않는다
        broken = Board(os.path.join(self.tmp, "없는 폴더"), "ros_x", self.sent.append)
        status, out = broken.apply({"op": "add", "item": {"kind": "entry", "pts": [[0, 0]]}})
        self.assertEqual((status, out["reason"]), (500, "store_failed"))
        self.assertEqual((broken.snapshot()["rev"], broken.snapshot()["items"], self.sent), (0, [], []))

    def test_routes_recalc_in_the_background(self):
        self.grid = two_rooms(8).grid(ver=1)
        self.add({"kind": "route", "from": {"type": "point", "x": 1.0, "y": 2.0}, "to": {"type": "point", "x": 5.0, "y": 2.0}})
        board_mod.POLL_S, old = 0.01, board_mod.POLL_S
        self.addCleanup(setattr, board_mod, "POLL_S", old)
        self.clock.t += 10
        self.grid = Room(60, 40).box(30, 0, 30, 39, BLOCKED).grid(ver=2)   # 문이 막힘
        self.board.start()
        self.addCleanup(self.board.close)
        for _ in range(200):
            if any(m["op"] == "route" for m in self.sent):
                break
            time.sleep(0.01)
        self.assertEqual([m["item"]["result"]["state"] for m in self.sent if m["op"] == "route"], ["partial"])


class ServerTest(unittest.TestCase):
    def setUp(self):
        try:
            from fastapi.testclient import TestClient
        except (ImportError, RuntimeError) as e:
            self.skipTest(f"TestClient 없음: {e}")
        import gateway_server
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        args = argparse.Namespace(ros_view="http://127.0.0.1:9", log_root=self.tmp, fake=False, replay=None, no_video=False,
                                  allow_remote_autonomy=True)
        self.app = gateway_server.build_app(args)
        self.gw = self.app.state.gateway
        self.addCleanup(self.gw.link.close)
        self.client = TestClient(self.app)   # with 없이: 보기 노드를 묻는 스레드·동선 다시 계산 스레드를 켜지 않는다

    def test_board_round_trip(self):
        c = self.client
        self.assertEqual(c.get("/api/board").json()["items"], [])
        r = c.post("/api/board", json={"op": "add", "client": "monitor", "item": {"kind": "entry", "pts": [[0.0, 0.0]]}})
        self.assertEqual((r.status_code, r.json()["item"]["text"]), (200, "진입 1"))
        self.gw._grid = {**two_rooms(8).grid(ver=7), "frame": "map"}   # 자율 비행 보기가 받은 지도
        r = c.post("/api/board", json={"op": "add", "item": {"kind": "route", "from": {"type": "start", "x": 1.0, "y": 2.0},
                                                             "to": {"type": "person", "n": 1, "x": 5.0, "y": 2.0}}})
        self.assertEqual((r.status_code, r.json()["item"]["result"]["state"], r.json()["item"]["result"]["grid_ver"]),
                         (200, "ok", 7))
        board = c.get("/api/board").json()
        self.assertEqual([it["kind"] for it in board["items"]], ["entry", "route"])
        self.assertEqual(board["run_id"], self.gw.link.run_id)
        events = c.get("/api/events").json()["events"]          # 새로 연 화면의 비행 기록에도
        self.assertEqual([(m["type"], m["op"]) for m in events], [("board", "add"), ("board", "add")])
        self.assertTrue(os.path.exists(os.path.join(self.gw.link.log.dir, "board.jsonl")))
        self.assertEqual(c.post("/api/board", content=b"{").json()["reason"], "invalid_json")
        self.assertEqual(c.post("/api/board", content=b"x" * 40_000).status_code, 413)
        self.assertEqual(c.post("/api/board", json={"op": "add", "item": {"kind": "pen"}}).status_code, 400)


if __name__ == "__main__":
    unittest.main()
