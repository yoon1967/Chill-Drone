"""자율 비행(ROS) 보기 노드의 ROS 없는 부분(ros_view_core.py)과 가짜 노드(fake_ros_view.py)를 시험한다.

보기 노드는 자율 비행 프로그램 토픽을 그대로 받아 HTTP로 내주기만 한다(게이트웨이 --ros-view가 가져감).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_ros_view_node.py" -v
"""

import ast
import base64
import json
import math
import os
import re
import shutil
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import fake_ros_view  # noqa: E402
import ros_view_core  # noqa: E402
from ros_view_core import (BOUNDARY, CLIENT_HEADER, FLIGHT_CMD, MISSION_CMD, ViewState, encode_jpeg,  # noqa: E402
                           make_server, newest_people_file, pose_sigma, quat_yaw, scan_range, watch_people, yaw_span)
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


class Recorder(list):
    """보기 노드가 낸 (토픽, 문자열). 진짜 노드의 publish 자리."""

    def __call__(self, topic, text):
        self.append((topic, text))


class Tap:
    """시험용 보기 노드: ViewState를 시험이 직접 채운다. publish를 주면 명령을 받는다(3단계)."""

    def __init__(self, testcase, publish=None, need_yolo=False):
        self.view = ViewState(publish=publish, need_yolo=need_yolo)
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
        # 드라이버는 상태마다 ROS 시각이 다르다. monotonic은 Windows에서 15.6 ms 단위라 같은 원문이 겹쳐 새 상태로 안 보였다
        self.view.on_state(json.dumps({"stamp": time.perf_counter(), "raw": raw}))
        self.view.on_flight_status(json.dumps({"flying": flying, "busy": busy, "connected": connected,
                                               "preflight": CHECKS, "rc_limit": 80, **extra}))


class CoreTest(unittest.TestCase):
    def test_yaw_span_like_the_flight_node(self):   # 11/17 비행 노드 tello_state.yaw_span과 같은 잣대(땅에서 기수 값이 흐르나)
        self.assertIsNone(yaw_span([(0.0, 5.0), (2.9, 9.0)], 2.9))                             # 3초도 안 쌓임: 모른다
        self.assertEqual(yaw_span([(0.0, 5.0), (1.5, 9.0), (3.0, 6.0)], 3.0), 4.0)
        self.assertEqual(yaw_span([(0.0, 179.0), (1.0, -179.0), (3.0, -178.0)], 3.0), 3.0)     # ±180° 넘김은 이어 붙인다
        self.assertEqual(yaw_span([(0.0, 50.0), (5.0, 5.0), (8.0, 5.0), (15.0, 6.0)], 15.0), 1.0)   # 10초 밖은 보지 않는다

    def test_autonomy_warnings_for_the_gateway(self):   # 11/17 충돌 뒤: /rosout WARN 이상을 번호와 함께, 놓친 것까지
        clock = Clock()
        v = ViewState(clock=clock)
        v.on_log(20, "tt_mission", "정보는 안 모은다")
        v.on_log(30, "tt_mission", "바로 밑 0.32 m에 무엇이 있습니다")
        clock.t += 0.5
        v.on_log(40, "tt_tello_driver", "드라이버 오류")
        snap = v.snapshot()
        self.assertEqual((snap["log"], "logs" in snap), ({"seq": 2}, False))   # 번호만(가져갈 때 log_after)
        snap = v.snapshot(log_after=1)
        self.assertEqual([(x["seq"], x["level"], x["node"], x["text"], x["age_s"]) for x in snap["logs"]],
                         [(2, 40, "tt_tello_driver", "드라이버 오류", 0.0)])
        self.assertEqual(len(v.snapshot(log_after=0)["logs"]), 2)

    def test_mission_note_passes_through(self):   # 11/17 /tt/mission_note: 그대로 두고 받은 지 몇 초만 붙인다
        clock = Clock()
        v = ViewState(clock=clock)
        self.assertIsNone(v.snapshot()["note"])
        note = {"t": 12.5, "state": "FOLLOW", "search": {"elapsed": 30.2, "limit": 120}, "returning": False,
                "why_return": None, "return_home": True, "why_end": None}
        v.on_note(json.dumps(note, ensure_ascii=False))
        clock.t += 0.4
        v.on_note("not json")
        v.on_note("[1, 2]")
        snap = v.snapshot()
        self.assertEqual(snap["note"], {**note, "age_s": 0.4})
        self.assertEqual((snap["bad"], snap["topics"]["/tt/mission_note"]["count"]), (2, 1))

    def test_scan_range(self):
        self.assertIsNone(scan_range([]))
        self.assertIsNone(scan_range([float("nan")]))
        self.assertTrue(math.isinf(scan_range([float("inf")])))   # 범위 밖은 inf 그대로
        self.assertEqual(scan_range([0.48]), 0.48)

    def test_state_history_for_the_gateway(self):   # B16: 게이트웨이가 놓친 상태를 번호로 다시 받는다
        clock = Clock()
        v = ViewState(clock=clock)
        for i in range(3):
            v.on_state(json.dumps({"stamp": i, "raw": f"time:{i};"}))
            clock.t += 0.1
        snap = v.snapshot(states_after=1)
        self.assertEqual(snap["state"]["seq"], 3)
        self.assertEqual([(s["seq"], s["raw"], s["age_s"]) for s in snap["states"]], [(2, "time:1;", 0.2), (3, "time:2;", 0.1)])
        self.assertEqual(snap["states_lost"], 0)
        self.assertNotIn("states", v.snapshot())                     # 묻지 않으면 예전 모양(마지막 상태 하나)
        self.assertEqual(v.snapshot(states_after=3)["states"], [])
        for i in range(ros_view_core.STATE_HISTORY + 5):
            v.on_state(json.dumps({"stamp": 10 + i, "raw": f"time:{10 + i};"}))
        snap = v.snapshot(states_after=3)
        self.assertEqual((len(snap["states"]), snap["states_lost"]), (ros_view_core.STATE_HISTORY, 5))   # 모아 둔 것보다 앞은 빠짐
        tap = Tap(self)
        tap.view.on_state(json.dumps({"stamp": 1, "raw": "time:1;"}))
        with urllib.request.urlopen(tap.url + "/api/state?states_after=0", timeout=5) as r:
            body = json.loads(r.read())
        self.assertEqual([s["seq"] for s in body["states"]], [1])

    def test_obstacle_grid_like_the_autonomy_web(self):   # 11/17: 자율 비행 웹 on_grid와 같은 칸 코드(0 모름·1 본 곳·2 의심·3 막힘)
        clock = Clock()
        v = ViewState(clock=clock)
        self.assertIsNone(v.grid())
        v.on_grid(0.1, 3, 2, -1.0, -2.0, bytes([255, 0, 50, 100, 7, 0]))   # int8 −1·0·50·100, 모르는 값 7
        g = v.grid()
        self.assertEqual(base64.b64decode(g["cells"]), bytes([0, 1, 2, 3, 0, 1]))
        self.assertEqual((g["ver"], g["w"], g["h"], g["x0"], g["y0"], g["counts"]),
                         (1, 3, 2, -1.0, -2.0, {"blocked": 1, "suspect": 1, "seen": 2}))
        snap = v.snapshot()["grid"]
        self.assertEqual((snap["ver"], snap["blocked"], snap["seen"], snap["w"]), (1, 1, 2, 3))   # 칸 전체는 상태에 안 싣는다
        self.assertNotIn("cells", snap)
        v.on_grid(0.1, 3, 3, 0, 0, bytes(6))                                 # 크기가 맞지 않으면 버린다
        v.on_grid(float("nan"), 3, 2, 0, 0, bytes(6))
        self.assertEqual((v.grid()["ver"], v.bad), (1, 2))
        self.assertIsNone(v.snapshot()["discovered"])
        v.on_discovered({"obstacles": [{"x": 1, "y": 2, "sx": 0.5, "sy": 0.4}]}, "f/discovered.json")
        self.assertEqual(v.snapshot()["discovered"]["obstacles"], 1)
        self.assertEqual(v.discovered()["obstacles"][0]["sx"], 0.5)
        # 자율 비행 웹이 그리는 지도 파일(시뮬레이터 known_map.json): 웹 load_map과 같은 키만, 바뀌면 번호가 오른다
        self.assertIsNone(v.snapshot()["world_map"])
        world = {"format": "tt-sim/known-map@2", "bounds": {"xmin": 0, "ymin": 0, "xmax": 12, "ymax": 10},
                 "spawn": {"x": 6.45, "y": 6.25, "yaw": -0.9}, "goal": None, "lintels": [{"x": 1}],
                 "walls": [{"x": 1, "y": 0, "sx": 1.7, "sy": 0.1}], "obstacles": [{"x": 4.6, "y": 3.8, "sx": 1.6, "sy": 1.3, "yaw": 3.14}]}
        v.on_world_map(world, "w/known_map.json")
        v.on_world_map(dict(world), "w/known_map.json")                      # 같은 내용: 번호 그대로
        m = v.world_map()
        self.assertEqual((m["version"], m["spawn"]["x"], len(m["walls"]), m["file"]), (1, 6.45, 1, "w/known_map.json"))
        self.assertNotIn("lintels", m)                                     # 웹이 쓰지 않는 키는 뺀다
        self.assertEqual(v.snapshot()["world_map"], {**v.snapshot()["world_map"], "walls": 1, "obstacles": 1, "open": False})
        # 시뮬레이션 정답 위치(/tt/odometry_gt)
        self.assertIsNone(v.snapshot()["truth"])
        v.on_truth(1.0, 2.0, 0.9, 0.5)
        v.on_truth(float("nan"), 0, 0, 0)
        self.assertEqual({k: v.snapshot()["truth"][k] for k in ("x_m", "y_m", "yaw_rad")}, {"x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.5})
        tap = Tap(self)
        with self.assertRaises(urllib.error.HTTPError) as no:
            urllib.request.urlopen(tap.url + "/api/grid", timeout=5)
        self.assertEqual(no.exception.code, 404)
        with self.assertRaises(urllib.error.HTTPError) as no:
            urllib.request.urlopen(tap.url + "/api/map", timeout=5)      # 지도 파일 없음
        self.assertEqual(no.exception.code, 404)
        tap.view.on_world_map({"bounds": {"xmin": 0, "ymin": 0, "xmax": 3, "ymax": 2}, "open": True}, "f/map.json")
        with urllib.request.urlopen(tap.url + "/api/map", timeout=5) as r:
            self.assertEqual(json.load(r)["open"], True)
        tap.view.on_grid(0.25, 2, 1, 0.0, 0.0, bytes([100, 0]))
        with urllib.request.urlopen(tap.url + "/api/grid", timeout=5) as r:
            self.assertEqual(base64.b64decode(json.loads(r.read())["cells"]), bytes([3, 1]))

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


class AutonomyInfoTest(unittest.TestCase):
    """2단계: 위치·계획 경로·임무 단계·찍은 자리·찾은 사람."""

    def test_quaternion_and_sigma(self):
        self.assertAlmostEqual(quat_yaw(0, 0, math.sin(0.5), math.cos(0.5)), 1.0)   # z축으로 1 rad
        self.assertAlmostEqual(pose_sigma([0.04] + [0] * 6 + [0.09]), 0.3)          # 큰 쪽(y 분산)
        self.assertIsNone(pose_sigma([-1.0] + [0] * 6 + [-1.0]))                    # 위치 추정 노드가 모르면 -1
        self.assertIsNone(pose_sigma(None))

    def test_values_and_versions(self):
        clock = Clock()
        v = ViewState(clock=clock)
        v.on_pose(1.0, 2.0, 0.9, 0.5, 0.12)
        v.on_pose(float("nan"), 0, 0, 0, None)                          # 엉터리 값은 버리고 센다
        v.on_mission_state("  FOLLOW  ")
        v.on_goal(json.dumps({"seq": 3, "state": "going", "text": "찍은 자리로 가는 중", "goal": [2.0, 1.0]}))
        v.on_goal("not json")
        v.on_path([(0, 0, 0.9), (1, 0, 0.9)])
        v.on_path([(0, 0, 0.9), (1, 0, 0.9)])                           # 같은 경로는 version 그대로
        clock.t += 0.5
        snap = v.snapshot()
        self.assertEqual((snap["pose"]["x_m"], snap["pose"]["sigma_m"], snap["pose"]["frame"], snap["pose"]["age_s"]),
                         (1.0, 0.12, "map", 0.5))
        self.assertEqual((snap["mission"]["state"], snap["goal"]["state"], snap["path"]["version"], snap["path"]["points"]),
                         ("FOLLOW", "going", 1, 2))
        self.assertEqual(snap["bad"], 2)
        v.on_path([(0, 0, 0.9), (2, 0, 0.9)])
        self.assertEqual((v.path()["version"], v.path()["points"][1]), (2, [2.0, 0.0, 0.9]))
        self.assertIsNone(v.people())
        v.on_people({"coverage": 0.3, "people": [{"x": 1, "y": 2}], "unconfirmed": []}, "a/people.json")
        v.on_people({"coverage": 0.3, "people": [{"x": 1, "y": 2}], "unconfirmed": []}, "a/people.json")
        self.assertEqual((v.people()["version"], v.people()["file"], v.snapshot()["people"]["confirmed"]), (1, "a/people.json", 1))
        v.on_people(None, None)                                         # 새 회차: 아직 파일 없음
        self.assertIsNone(v.people())

    def test_newest_run_people_file(self):
        root = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, root, True)
        self.assertIsNone(newest_people_file(root))
        os.makedirs(os.path.join(root, "20261113_101500"))
        with open(os.path.join(root, "20261113_101500", "people.json"), "w", encoding="utf-8") as f:
            json.dump({"people": [{"x": 1, "y": 1}], "unconfirmed": []}, f)
        self.assertTrue(newest_people_file(root).endswith(os.path.join("20261113_101500", "people.json")))
        os.makedirs(os.path.join(root, "20261113_120000"))               # 새 회차가 시작됐고 아직 사람 파일이 없다
        self.assertIsNone(newest_people_file(root))                       # 지난 회차 것을 쓰지 않는다

    def test_watch_people_reads_and_clears(self):
        root = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, root, True)
        run = os.path.join(root, "20261113_101500")
        os.makedirs(run)
        with open(os.path.join(run, "people.json"), "w", encoding="utf-8") as f:
            json.dump({"coverage": 0.2, "people": [{"x": 1, "y": 1}], "unconfirmed": []}, f)
        v, stop = ViewState(), threading.Event()
        threading.Thread(target=watch_people, args=(v, root, None, stop, 0.05), daemon=True).start()
        self.addCleanup(stop.set)
        deadline = time.monotonic() + 3
        while v.people() is None and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertEqual(v.people()["coverage"], 0.2)
        os.makedirs(os.path.join(root, "20261113_120000"))               # 새 회차
        deadline = time.monotonic() + 3
        while v.people() is not None and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertIsNone(v.people())

    def test_http_path_and_people(self):
        tap = Tap(self)
        with self.assertRaises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(tap.url + "/api/people", timeout=5)
        self.assertEqual(e.exception.code, 404)                          # 아직 사람 파일 없음
        tap.view.on_path([(0, 0, 0.9), (1, 0.5, 0.9)])
        tap.view.on_people({"coverage": 0.4, "people": [], "unconfirmed": [{"x": 2, "y": 1}]}, "f")
        with urllib.request.urlopen(tap.url + "/api/path", timeout=5) as r:
            self.assertEqual(json.loads(r.read())["points"], [[0.0, 0.0, 0.9], [1.0, 0.5, 0.9]])
        with urllib.request.urlopen(tap.url + "/api/people", timeout=5) as r:
            body = json.loads(r.read())
        self.assertEqual((body["coverage"], body["unconfirmed"][0]["x"], body["frame"]), (0.4, 2, "map"))


class CommandTest(unittest.TestCase):
    """3단계: 관제 화면 명령을 자율 비행 웹(autonomy_web.command)과 같은 토픽·문자열로 낸다."""

    def test_same_strings_as_the_autonomy_web(self):
        rec = Recorder()
        v = ViewState(publish=rec)
        for name, body in (("mission/start", None), ("mission/goto", {"x": 1.5, "y": -2}), ("mission/goto_cancel", None),
                           ("flight/emergency", None), ("flight/confirm", {"confirmed": True}),
                           ("flight/confirm", {"confirmed": False})):
            self.assertTrue(v.command(name, body)["ok"], name)
        self.assertEqual(rec, [(MISSION_CMD, "start"), (MISSION_CMD, "goto 1.50 -2.00"), (MISSION_CMD, "goto cancel"),
                               (FLIGHT_CMD, "emergency"), (FLIGHT_CMD, "emergency"),    # 웹처럼 두 번
                               (FLIGHT_CMD, "confirm"), (FLIGHT_CMD, "unconfirm")])
        self.assertEqual(v.snapshot()["commands"]["count"], 6)

    def test_land_also_tells_the_driver_when_the_mission_is_over(self):   # 웹: 떠 있는데 임무가 없거나 끝·중단이면 드라이버에도
        rec = Recorder()
        v = ViewState(publish=rec)
        v.on_flight_status(json.dumps({"flying": True}))
        v.on_mission_state("FOLLOW")
        self.assertEqual(v.command("mission/land")["sent"], ["land"])
        v.on_mission_state("DONE")
        self.assertEqual(v.command("mission/land")["sent"], ["land", "land"])
        self.assertEqual(rec[-2:], [(MISSION_CMD, "land"), (FLIGHT_CMD, "land")])
        v.on_flight_status(json.dumps({"flying": False}))
        self.assertEqual(v.command("mission/land")["sent"], ["land"])   # 땅이면 비행 노드에만

    def test_refusals(self):
        rec = Recorder()
        v = ViewState(publish=rec)
        for x, y in (("1", 2), (float("nan"), 0), (True, 1), (2000, 0)):
            self.assertEqual(v.command("mission/goto", {"x": x, "y": y}), {"ok": False, "reason": "bad_xy"})
        self.assertEqual(v.command("flight/takeover")["reason"], "unknown")
        self.assertEqual(rec, [])
        self.assertEqual(ViewState().command("mission/start"), {"ok": False, "reason": "commands_disabled"})   # 보기만

    def test_yolo_ready_like_the_web(self):
        clock = Clock()
        v = ViewState(clock=clock, need_yolo=True)
        self.assertEqual((v.snapshot()["yolo"], v.snapshot()["need_yolo"]), (None, True))
        v.on_detections("{}")
        clock.t += 2.9
        self.assertTrue(v.snapshot()["yolo"]["ready"])
        clock.t += 0.2
        self.assertFalse(v.snapshot()["yolo"]["ready"])   # 3초 넘게 YOLO 결과가 없다

    def test_http_post_only_from_the_gateway(self):
        rec = Recorder()
        tap = Tap(self, publish=rec)

        def post(path, body=b"{}", headers=None):
            req = urllib.request.Request(tap.url + path, data=body, method="POST",
                                         headers={"Content-Type": "application/json", **(headers or {})})
            try:
                with urllib.request.urlopen(req, timeout=5) as r:
                    return r.status, json.loads(r.read())
            except urllib.error.HTTPError as e:
                return e.code, json.loads(e.read())

        gw = {CLIENT_HEADER: "1"}
        self.assertEqual(post("/api/mission/start")[0], 403)                            # 헤더 없음(브라우저 페이지)
        self.assertEqual(post("/api/mission/start", headers={**gw, "Host": "evil.example:8094"})[0], 403)   # 다른 이름으로 온 요청
        self.assertEqual(rec, [])
        self.assertEqual(post("/api/mission/start", headers=gw), (200, {"ok": True, "sent": ["start"]}))
        self.assertEqual(post("/api/mission/goto", b'{"x": 1, "y": 2}', gw)[1]["sent"], ["goto 1.00 2.00"])
        self.assertEqual(post("/api/mission/goto", b"not json", gw)[0], 400)
        self.assertEqual(post("/api/flight/takeover", headers=gw)[0], 404)
        self.assertEqual(rec, [(MISSION_CMD, "start"), (MISSION_CMD, "goto 1.00 2.00")])


class FakeNodeTest(unittest.TestCase):
    def test_fake_flight_follows_commands(self):
        clock = Clock()
        f = fake_ros_view.FakeFlight(clock=clock)

        def run(seconds):
            for _ in range(int(seconds / 0.1)):
                clock.t += 0.1
                f.step()

        self.assertEqual(f.mission(), "WAIT_POSE")
        run(3.1)
        self.assertEqual(f.mission(), "READY")
        f.on_command(MISSION_CMD, "start")
        self.assertEqual(f.phase, "ground")                  # 주변 확인 전: 드라이버가 이륙하지 않는다
        f.on_command(FLIGHT_CMD, "confirm")
        f.on_command(MISSION_CMD, "start")
        self.assertEqual((f.phase, f.mission(), f.operator), ("takeoff", "TAKEOFF", False))   # 확인은 이륙에 썼다
        run(2.1)
        self.assertEqual(f.phase, "flying")
        f.on_command(MISSION_CMD, "goto 1.00 1.00")
        self.assertEqual((f.mission(), f.goal_msg()["state"]), ("FOLLOW", "going"))
        run(4.0)
        self.assertEqual((f.mission(), f.goal_msg()["state"]), ("LOOK", "arrived"))
        x, y, _, _ = f.pose()
        self.assertLess(math.hypot(x - 1, y - 1), 0.2)
        run(3.1)
        self.assertEqual(f.goal_msg()["state"], "none")       # 둘러본 뒤 다시 돈다
        f.on_command(MISSION_CMD, "land")
        self.assertEqual(f.phase, "land")
        self.assertTrue(f.note_msg()["why_end"].startswith("착륙 요청을 받았습니다 (지금 단계"))   # 비행 노드: 바로 앞 로그가 내린 이유
        run(2.1)
        self.assertEqual((f.phase, f.mission()), ("ground", "DONE"))
        run(5.1)
        self.assertEqual(f.mission(), "READY")
        f.on_command(FLIGHT_CMD, "confirm")
        f.on_command(MISSION_CMD, "start")
        run(2.5)
        f.on_command(FLIGHT_CMD, "emergency")
        self.assertEqual((f.phase, f.mission()), ("ground", "ABORT"))
        fs = fake_ros_view.flight_status(f.phase, 80, 60, f.operator)
        self.assertEqual((fs["can_takeoff"], [c["ok"] for c in fs["preflight"] if c["key"] == "operator"]), (False, [False]))

    @staticmethod
    def run_fake(f, clock, seconds):
        for _ in range(int(round(seconds / 0.1))):
            clock.t += 0.1
            f.step()

    def test_fake_search_time_returns_home_with_reasons(self):   # 지금 하는 일과 이유: 탐색 시간이 다 되면 출발점으로 와서 내린다
        clock = Clock()
        f = fake_ros_view.FakeFlight(clock=clock)
        self.assertIsNone(f.note_msg()["search"])                                          # 아직 안 날았다
        self.run_fake(f, clock, 3.1)
        f.on_command(FLIGHT_CMD, "confirm")
        f.on_command(MISSION_CMD, "start")
        self.run_fake(f, clock, 2.1 + 10.0)
        n = f.note_msg()
        self.assertEqual((n["search"]["limit"], n["returning"], n["why_end"]), (fake_ros_view.SEARCH_S, False, None))
        self.assertAlmostEqual(n["search"]["elapsed"], 10.0, delta=0.25)
        self.run_fake(f, clock, fake_ros_view.SEARCH_S - 10.0 + 0.3)
        n = f.note_msg()
        self.assertEqual((n["state"], n["returning"], n["why_return"]), ("FOLLOW", True, "탐색 시간(24초)이 다 돼"))
        self.assertEqual(len(f.path()), 11)                                                 # 계획 경로: 출발점으로 곧장
        self.run_fake(f, clock, 4.0)                                                        # 약 1.8 m를 0.5 m/s로 와서 내린다
        n = f.note_msg()
        self.assertEqual((f.phase, n["state"], n["why_end"]), ("land", "LAND", "탐색 시간(24초)이 다 돼 출발점으로 돌아와 내립니다."))
        self.assertLess(math.hypot(*f.pose()[:2]), 0.2)
        self.run_fake(f, clock, 2.1)
        self.assertEqual((f.phase, f.note_msg()["state"]), ("ground", "DONE"))

    def test_fake_goal_replies_like_the_flight_node(self):   # 찍은 자리 답: 새 답마다 번호·시각, 방 밖은 받지 않는다(warn)
        clock = Clock()
        f = fake_ros_view.FakeFlight(clock=clock)
        self.run_fake(f, clock, 3.1)
        f.on_command(FLIGHT_CMD, "confirm")
        f.on_command(MISSION_CMD, "start")
        self.run_fake(f, clock, 2.1)
        f.on_command(MISSION_CMD, "goto 9.00 9.00")                                         # 방 밖
        g = f.goal_msg()
        self.assertEqual((g["state"], g["warn"], g["seq"]), ("none", True, 1))
        self.assertIn("받지 않습니다", g["text"])
        self.run_fake(f, clock, 1.0)
        self.assertEqual(f.goal_msg(), g)                                                   # 같은 답은 같은 번호·시각
        f.on_command(MISSION_CMD, "goto 1.00 1.00")
        g = f.goal_msg()
        self.assertEqual((g["state"], g["warn"], g["seq"], g["click"]), ("going", False, 2, [1.0, 1.0]))
        f.on_command(MISSION_CMD, "goto cancel")
        g = f.goal_msg()
        self.assertEqual((g["state"], g["seq"], g["text"]), ("none", 3, "찍은 자리를 취소했습니다. 자율 탐색을 이어 갑니다."))

    def test_fake_flip(self):   # --flip-after: 11/17 실기체처럼 뒤집혀 드라이버가 모터를 끔 → 임무 중단
        clock = Clock()
        f = fake_ros_view.FakeFlight(clock=clock, flip_after=3.0)
        self.run_fake(f, clock, 3.1)
        f.on_command(FLIGHT_CMD, "confirm")
        f.on_command(MISSION_CMD, "start")
        self.run_fake(f, clock, 2.1 + 2.0)
        self.assertEqual((f.phase, f.last_auto_land), ("flying", None))
        self.run_fake(f, clock, 1.2)
        self.assertEqual((f.phase, f.mission(), f.last_auto_land["reason"]), ("ground", "ABORT", "flipped"))
        self.assertIn("뒤집힘", f.note_msg()["why_end"])
        fs = fake_ros_view.flight_status(f.phase, 80, 60, f.operator, f.last_result, f.last_auto_land)
        self.assertEqual((fs["flying"], fs["last_auto_land"]["reason"]), (False, "flipped"))

    def test_fake_sink(self):   # --sink-after: 실제 높이(하방·기압)만 내려가고 추정은 그대로(11/17 충돌처럼)
        clock = Clock()
        f = fake_ros_view.FakeFlight(clock=clock, sink_after=2.0)
        self.run_fake(f, clock, 3.1)
        f.on_command(FLIGHT_CMD, "confirm")
        f.on_command(MISSION_CMD, "start")
        self.run_fake(f, clock, 2.1 + 1.0)
        self.assertIsNone(f.real_height(clock()))
        self.run_fake(f, clock, 4.0)
        real = f.real_height(clock())
        self.assertEqual(real, 0.35)
        self.assertEqual(f.pose()[2], 0.9)                                                  # 추정 높이는 그대로
        s = parse_state(fake_ros_view.tello_raw(1.0, "flying", 5.0, 0, real))
        self.assertAlmostEqual(s.si["tof_m"], 0.35, delta=0.04)
        self.assertAlmostEqual(s.raw["baro"], 12.85, places=2)

    def test_fake_takeoff_refusal_back_to_ready(self):   # --refuse-takeoff: 드론이 이륙을 거절 → 비행 노드(11/17)는 다시 보내지 않는다
        clock = Clock()
        f = fake_ros_view.FakeFlight(clock=clock, refuse=1)
        self.run_fake(f, clock, 3.1)
        f.on_command(FLIGHT_CMD, "confirm")
        f.on_command(MISSION_CMD, "start")
        self.assertEqual((f.phase, f.mission(), f.operator), ("ground", "READY", True))   # 시작 대기로. 확인은 남는다(드라이버와 같음)
        self.assertEqual((f.last_result["ok"], f.last_result["result"]["response"]), (False, "error"))
        self.run_fake(f, clock, 25.0)
        self.assertEqual((f.phase, f.requests), ("ground", 1))                               # 저절로 다시 보내지 않는다
        f.on_command(MISSION_CMD, "start")
        self.assertEqual((f.phase, f.last_result["ok"], f.requests), ("takeoff", True, 2))   # 다시 시작하면 뜬다

    def test_fake_yaw_drift_holds_the_first_start(self):   # --yaw-drift: 비행 노드 start_blocked처럼 흐르는 동안 첫 시작은 받지 않는다
        clock = Clock()
        f = fake_ros_view.FakeFlight(clock=clock, yaw_drift=8.0)
        self.run_fake(f, clock, 5.0)
        self.assertEqual(f.ground_yaw_now(clock()), 5)                                      # 켠 뒤 1°/초로 흐른다
        f.on_command(FLIGHT_CMD, "confirm")
        f.on_command(MISSION_CMD, "start")
        self.assertEqual((f.phase, f.mission(), f.start_holds, f.requests), ("ground", "READY", 1, 0))
        self.run_fake(f, clock, 1.0)
        f.on_command(MISSION_CMD, "start")                                                  # 30초 안에 한 번 더: 그대로 요청
        self.assertEqual((f.phase, f.requests), ("takeoff", 1))
        f2 = fake_ros_view.FakeFlight(clock=clock, yaw_drift=2.0)
        self.run_fake(f2, clock, 12.5)                                                      # 2초 흐르고 멈춘 지 10초 넘음
        f2.on_command(FLIGHT_CMD, "confirm")
        f2.on_command(MISSION_CMD, "start")
        self.assertEqual((f2.phase, f2.start_holds), ("takeoff", 0))                        # 멈췄으면 바로 받는다

    def test_fake_autonomy_info(self):
        self.assertEqual([fake_ros_view.mission_at("ground", 1), fake_ros_view.mission_at("ground", 5),
                          fake_ros_view.mission_at("takeoff", 1), fake_ros_view.mission_at("land", 1)],
                         ["WAIT_POSE", "READY", "TAKEOFF", "LAND"])
        x, y, z, yaw = fake_ros_view.pose_at("flying", fake_ros_view.FLY_S / 4)   # 4분의 1 바퀴: 앞 1.5 m, 왼쪽 1.5 m, 왼쪽을 봄
        self.assertEqual((round(x, 2), round(y, 2), z, round(yaw, 2)), (1.5, 1.5, 0.9, 1.57))
        p = fake_ros_view.people_at(16)
        self.assertEqual((len(p["people"]), len(p["unconfirmed"])), (1, 1))

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
        for topic in ("/tt/state", "/tt/flight_status", "/tt/tof_front", "/tt/camera/image_raw", "/tt/pose_est", "/tt/path",
                      "/tt/mission_state", "/tt/operator_goal", "/tt/mission_note", "/tt/detections", "/tt/obstacle_grid",
                      "/rosout"):
            self.assertIn(topic, src)
        # 내는 것은 자율 비행 웹과 같은 두 토픽뿐이고, commands:=false면 아무것도 내지 않는다(3단계)
        self.assertEqual(sorted(re.findall(r"create_publisher\(String, (\w+), 10\)", src)), ["FLIGHT_CMD", "MISSION_CMD"])
        self.assertIn('if g("commands"):', src)


if __name__ == "__main__":
    unittest.main()
