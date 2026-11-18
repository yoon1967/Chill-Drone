"""자율 비행(ROS) 보기: 게이트웨이 보기 모드(ros_view.py, 서버 --ros-view)를 ROS 없이 시험한다.

게이트웨이는 드론에 붙지 않고 보기 노드(ros_view_node.py)에서 상태·영상·자율 정보를 받아 화면에 보인다.
화면 버튼은 보기 노드에 넘기고, 보기 노드가 자율 비행 웹과 같은 명령을 낸다(3단계). 여는 조건도 웹과 같다.
보기 노드는 시험용으로 ros_view_core의 서버를 직접 채운다(test_ros_view_node.Tap).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_ros_view.py" -v
"""

import argparse
import base64
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ros_view  # noqa: E402
from ros_view_core import FLIGHT_CMD, MISSION_CMD  # noqa: E402
from test_ros_view_node import CHECKS, FLYING, GROUND, Clock, Recorder, Tap, can_encode, image  # noqa: E402


class GatewayCase(unittest.TestCase):
    """보기 게이트웨이 하나와 가짜 보기 노드(시험 시계로 돈다)."""
    SENSOR_POSE = False

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.tap = Tap(self)
        self.clock = Clock()
        self.gw = ros_view.RosViewGateway(self.tap.url, log_root=self.tmp, clock=self.clock, sensor_pose=self.SENSOR_POSE)
        self.addCleanup(self.gw.link.close)
        self.sent = []
        self.gw.listeners.append(self.sent.append)

    def step(self, dt=0.1):
        self.clock.t += dt
        self.gw.tick()
        return self.gw.drone_state()


class GatewayTest(GatewayCase):
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

    def test_view_only_node_refuses_commands(self):   # 보기 노드가 명령을 받지 않게 켜졌으면(commands:=false) 버튼을 닫는다
        self.tap.state(GROUND)
        st = self.step()
        self.assertEqual((st["allowed"], st["ros_view"]["commands"]), ([], False))
        r = self.gw.handle_message({"type": "cmd", "id": "c1", "name": "land", "client": "monitor"})
        self.assertEqual((r["type"], r["id"], r["ok"], r["reason"]), ("result", "c1", False, "commands_disabled"))
        self.assertIsNone(self.gw.handle_message({"type": "rc", "axes": [0, 1, 0, 0]}))   # 사람 rc 길은 없다
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


class MapVideoTest(unittest.TestCase):
    """지도용 영상: 지도 파트는 Tello 원본 960×720(내장 보정값)만 받는다. 실기체 자율 비행 드라이버는 480×360으로 줄여 낸다."""

    def jpeg(self, w, h):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.skipTest("OpenCV 없음")
        img = np.zeros((h, w, 3), np.uint8)
        img[:, : w // 2] = (0, 128, 255)
        return cv2.imencode(".jpg", img)[1].tobytes()

    def size(self, data):
        import cv2
        import numpy as np
        h, w = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR).shape[:2]
        return w, h

    def test_shrunk_real_drone_video_is_scaled_back_for_the_map(self):
        video = ros_view.ViewVideo("http://127.0.0.1:9")
        small = self.jpeg(480, 360)
        video.size = (480, 360)
        video.put(small)
        out = video.map_snapshot()
        self.assertEqual(self.size(out[0]), (960, 720))
        self.assertEqual(out[1:], video.jpeg_snapshot()[1:])   # 장 시각은 받은 그대로(센서 자세를 그 시각으로 붙인다)
        self.assertIs(video.map_snapshot(), out)      # 같은 장은 한 번만 키운다
        self.assertIs(video.jpeg(), small)            # 화면·사람 탐지는 받은 그대로

    def test_full_size_and_other_aspect_are_left_alone(self):
        video = ros_view.ViewVideo("http://127.0.0.1:9")
        full = self.jpeg(960, 720)
        video.size = (960, 720)                       # 시뮬레이터
        video.put(full)
        self.assertIs(video.map_snapshot()[0], full)
        wide = self.jpeg(640, 360)                    # 화각이 다르면 보정이 안 맞으니 그대로(지도 파트가 거절)
        self.assertIs(ros_view.scale_jpeg(wide, ros_view.MAP_SIZE), wide)


class SensorPoseTest(GatewayCase):
    """지도용 센서 자세: 자율 비행 드라이버가 받은 Tello 상태 원문을 실기체 게이트웨이와 같은 적분(TelloOdometry)으로.
    없으면 지도 파트 실시간 지도가 영상으로만 위치를 맞추다 놓치면 짐작한 위치로 쌓는다.
    실시간으로 도는 드론(실기체·가짜 Tello)만 켠다(--ros-view-sensor-pose, 실행기 autonomy)."""
    SENSOR_POSE = True

    def setUp(self):
        super().setUp()
        self.tap.view.clock = self.clock   # 보기 노드도 시험 시계: 상태를 받은 때 = 보낸 때의 시험 시각(step 전)

    def test_driver_states_become_the_map_sensor_pose(self):
        self.tap.state(GROUND)
        self.step()
        self.step()                                              # 같은 상태는 한 번만 적분한다
        pose = self.gw.link.sensor_pose_at(self.clock.t - 0.1)
        self.assertEqual((pose.segment, round(pose.x, 3), round(pose.z, 3)), (0, 0.0, 0.0))
        forward = FLYING.replace("vgx:0;", "vgx:-5;")            # 켤 때 앞쪽으로 0.5 m/s(Tello vgx는 dm/s·부호 반대)
        for i in range(3):   # 드라이버 원문은 매번 다르다(time·baro…). 여기서는 time만 바꾼다
            self.tap.state(forward.replace("time:12;", f"time:{13 + i};"), flying=True)
            self.step()
        pose = self.gw.link.sensor_pose_at(self.clock.t - 0.1)   # 마지막 상태를 받은 때
        self.assertEqual(pose.segment, 0)
        self.assertAlmostEqual(pose.x, 0.15, places=6)            # 0.2초 동안 0→0.5 m/s, 0.2초 동안 0.5 m/s
        self.assertAlmostEqual(pose.z, 0.92, places=6)            # 높이는 하방 ToF
        self.assertAlmostEqual(self.gw.link.sensor_pose_at(self.clock.t).x, 0.20, places=6)   # 0.1초 앞은 짐작
        self.assertIsNone(self.gw.link.sensor_pose_at(self.clock.t + 0.5))   # 오래 앞은 짐작하지 않는다
        self.clock.t += 0.4                                      # 상태가 0.35초 넘게 안 옴(드라이버·보기 노드 멈춤)
        self.tap.state(forward.replace("time:12;", "time:20;"), flying=True)
        self.step()                                              # 새 구간: 지도 파트가 영상으로 다시 맞춘다
        self.assertEqual(self.gw.link.sensor_pose_at(self.clock.t - 0.1).segment, 1)

    def test_gateway_stall_does_not_lose_states(self):   # B16: 게이트웨이가 0.35초 넘게 늦어도 보기 노드가 모아 둔 상태를 다 받는다
        forward = FLYING.replace("vgx:0;", "vgx:-5;")
        self.tap.state(forward, flying=True)
        self.step(0.0)                                           # 첫 상태(번호를 받는다)
        for i in range(6):                                       # 게이트웨이가 0.6초 못 묻는 사이 드라이버는 0.1초마다 냈다
            self.clock.t += 0.1
            self.tap.state(forward.replace("time:12;", f"time:{13 + i};"), flying=True)
        self.step(0.0)
        self.assertEqual(self.gw.states, 7)
        rows = self.gw.link.sensor_samples()["samples"]
        gaps = [round(b["received_mono"] - a["received_mono"], 6) for a, b in zip(rows, rows[1:])]
        self.assertEqual(gaps, [0.1] * 6)                        # 받은 때는 보기 노드가 받은 때 그대로
        self.assertEqual(self.gw.link.sensor_pose_at(self.clock.t).segment, 0)   # 지도 원점이 새로 잡히지 않는다
        self.assertEqual(self.gw.states_lost, 0)

    def test_sensor_batch_for_map_recording(self):   # GET /api/drone/sensors: 실기체 게이트웨이와 같은 모양
        for raw in (GROUND, FLYING, FLYING.replace("vgx:0;", "vgx:-5;")):
            self.tap.state(raw)
            self.step()
        batch = self.gw.link.sensor_samples()
        self.assertEqual((batch["schema_version"], batch["run_id"], batch["next_sequence"], batch["lost"]),
                         ("tello-sensor-batch/1.0", self.gw.link.run_id, 3, 0))
        rows = batch["samples"]
        self.assertEqual([(r["sequence"], r["time_source"]) for r in rows], [(1, "ros_view"), (2, "ros_view"), (3, "ros_view")])
        self.assertTrue(all(b["received_mono"] > a["received_mono"] for a, b in zip(rows, rows[1:])))
        self.assertAlmostEqual(rows[-1]["received_mono"], self.clock.t - 0.1, places=6)   # 게이트웨이 시계(영상 장 시각과 같은 시계)
        self.assertEqual(self.gw.link.sensor_samples(3)["samples"], [])

    def test_off_by_default_for_the_slow_simulator(self):   # 시뮬레이터(실제보다 느림)는 영상만: 지도 파트가 영상으로 원점을 잡는다
        gw = ros_view.RosViewGateway(self.tap.url, log_root=self.tmp, clock=self.clock)
        self.addCleanup(gw.link.close)
        self.assertFalse(hasattr(gw.link, "sensor_pose_at") or hasattr(gw.link, "sensor_samples"))
        self.tap.state(GROUND)
        gw.tick()                                                # 상태는 그대로 받는다
        self.assertEqual(gw.states, 1)

    def test_receive_order_is_kept(self):   # 붙어 온 두 상태: 늦게 받은 것이 앞 시각으로 적히지 않는다
        self.tap.state(GROUND)
        self.step()
        first = self.gw.link.sensor_samples()["samples"][0]["received_mono"]
        self.tap.state(FLYING, flying=True)
        self.step(0.0)
        self.clock.t -= 0.01                                      # 시계 단위·지연으로 거꾸로 읽힌 때
        self.tap.state(GROUND)
        self.step(0.0)
        stamps = [r["received_mono"] for r in self.gw.link.sensor_samples()["samples"]]
        self.assertEqual(len(stamps), 3)
        self.assertTrue(stamps[0] == first and stamps[0] < stamps[1] < stamps[2])


class GatewayAutonomyTest(GatewayCase):
    """2단계: 보기 노드의 자율 정보가 자율 창구로 가서 2D 지도·발견 지점·기록에 쓰인다."""

    def setUp(self):
        super().setUp()
        self.tap.view.clock = self.clock   # 보기 노드도 같은 시계: 시험이 시간을 돌리면 자율 정보도 그만큼 오래된다

    def fly(self):
        self.tap.state(FLYING, flying=True)
        self.tap.view.on_pose(1.0, 0.5, 0.9, 0.3, 0.1)
        self.tap.view.on_mission_state("LOOK")

    def test_autonomy_opens_on_ground_and_fills_the_map(self):
        self.tap.state(GROUND)
        self.tap.view.on_mission_state("READY")
        self.tap.view.on_pose(0.0, 0.0, 0.0, 0.0, None)
        st = self.step()
        self.assertTrue(st["autonomy"]["connected"])
        self.assertTrue(st["autonomy"]["ground_origin"])                        # 땅에서 연결: 원점 = 출발점
        self.assertEqual((st["mission"]["state"], st["mission"]["phase"]), ("READY", "idle"))
        self.fly()
        self.tap.view.on_path([(1.0, 0.5, 0.9), (2.0, 1.0, 0.9)])
        self.tap.view.on_people({"coverage": 0.35, "people": [{"x": 2.0, "y": 1.0, "conf": 0.9, "hits": 3}],
                                 "unconfirmed": []}, "f")
        st = self.step()
        self.assertEqual((st["pose"]["x_m"], st["pose"]["valid"]), (1.0, True))
        self.assertEqual((st["mission"]["state"], st["mission"]["phase"], st["mission"]["progress"]), ("LOOK", "running", 0.35))
        self.assertTrue(st["autonomy"]["mission_active"])
        snap = self.gw.autonomy.snapshot()
        self.assertEqual(len(snap["path"]["points"]), 2)
        self.assertEqual(snap["people"]["people"][0]["x_m"], 2.0)
        self.tap.view.on_people(None, None)                                    # 새 회차: 사람 파일 없음 → 화면에서 비움
        self.step()
        self.assertEqual(self.gw.autonomy.snapshot()["people"]["people"], [])

    def test_mission_note_like_the_web(self):   # 11/17 /tt/mission_note: 지금 하는 일과 이유. 기록은 돌아가기 시작·내리는 이유만
        note = {"t": 1.0, "state": "FOLLOW", "search": {"elapsed": 30.0, "limit": 120}, "returning": False, "why_return": None,
                "return_home": True, "why_end": None}

        def say(**kw):
            self.tap.view.on_note(json.dumps({**note, **kw}, ensure_ascii=False))
            return self.step()

        self.fly()
        st = say()
        self.assertEqual(st["mission"]["note"], {"state": "FOLLOW", "search": {"elapsed": 30.0, "limit": 120.0}, "returning": False,
                                                 "why_return": None, "return_home": True, "why_end": None})
        self.assertIsNone(say(search={"elapsed": "x", "limit": 120})["mission"]["note"]["search"])   # 모양이 틀리면 없음
        why = "탐색 시간(120초)이 다 돼"
        say(returning=True, why_return=why)
        say(returning=True, why_return=why)                                 # 1초마다 같은 것: 다시 적지 않는다
        end = f"{why} 출발점으로 돌아와 내립니다."
        say(state="LAND", returning=True, why_return=why, why_end=end)
        st = say(state="DONE", returning=True, why_return=why, why_end=end)
        self.assertEqual(st["mission"]["note"]["why_end"], end)
        told = [m["detail"] for m in self.sent if m.get("kind") == "mission_note"]
        self.assertEqual(told, [{"what": "returning", "why_return": why}, {"what": "end", "why_end": end, "state": "LAND"}])
        self.tap.view.on_mission_state("DONE")
        st = self.step(ros_view.NOTE_FRESH_S + 0.5)                       # 비행 노드가 3초 넘게 조용하면 내린 이유만
        self.assertEqual(st["mission"]["note"], {"state": "DONE", "why_end": end, "stale": True})
        self.gw.link.close()
        with open(os.path.join(self.gw.link.log.dir, "events.jsonl"), encoding="utf-8") as f:
            logged = [e["detail"]["what"] for e in map(json.loads, f) if e["type"] == "mission_note"]
        self.assertEqual(logged, ["returning", "end"])

    def test_goal_reply_for_twenty_seconds(self):   # 11/17 찍은 자리 답(받음·거절·그만둠·도착): [여기로 보내기] 옆에 20초, 기록에도
        def goal(seq, state, text, warn=False):
            self.tap.view.on_goal(json.dumps({"seq": seq, "t": float(seq), "state": state, "text": text, "warn": warn,
                                              "click": [1.0, 2.0], "goal": [1.0, 2.0] if state == "going" else None},
                                             ensure_ascii=False))

        def told():
            return [m["detail"] for m in self.sent if m.get("kind") == "goal_note"]

        goal(0, "none", "지난 비행의 답")
        self.clock.t += 10.0                                                    # 게이트웨이가 보기 전에 온 답(10초 전)
        self.fly()
        st = self.step()
        self.assertEqual(st["mission"]["goal_note"], {"text": "지난 비행의 답", "warn": False, "age_s": 10.1})   # 처음 들은 때 = 보기 노드가 받은 때
        self.assertEqual(told(), [])                                           # 지난 답은 기록에 적지 않는다
        no = "찍은 자리 (9.0, 9.0)를 받지 않습니다: 아직 못 본 곳입니다."
        goal(1, "none", no, warn=True)
        st = self.step()
        self.assertEqual((st["mission"]["goal_note"], st["mission"]["goal"]), ({"text": no, "warn": True, "age_s": 0.1}, None))
        for _ in range(20):                                                     # 같은 답(seq, t)이 다시 와도 처음 들은 때부터 센다
            goal(1, "none", no, warn=True)
            self.tap.view.on_mission_state("LOOK")
            st = self.step(1.0)
        self.assertGreater(st["mission"]["goal_note"]["age_s"], 20)          # 화면은 20초가 지나면 보이지 않는다
        go = "찍은 자리 (1.0, 2.0)로 갑니다 (1.2 m 거리). 도착하면 둘러본 뒤 자율 탐색을 이어 갑니다."
        goal(2, "going", go)
        st = self.step()
        self.assertEqual((st["mission"]["goal"]["state"], st["mission"]["goal_note"]["text"]), ("going", go))
        self.tap.view.on_mission_state("LOOK")
        st = self.step(ros_view.GOAL_FRESH_S + 0.5)
        self.assertIsNone(st["mission"]["goal"])                               # 3초 넘게 다시 안 알리면(비행 노드 꺼짐) 지도에서 뺀다
        self.assertEqual(told(), [{"text": no, "warn": True, "state": "none"}, {"text": go, "warn": False, "state": "going"}])

    def test_bad_values_do_not_stop_the_view(self):   # A3: 비행 노드가 이상한 값을 내도 보기는 계속 돈다
        self.fly()
        many = [{"x": 0.1 * i, "y": 0.0, "conf": 0.8, "hits": 2} for i in range(150)]
        self.tap.view.on_people({"coverage": 0.2, "people": many + [{"x": float("nan"), "y": 1.0}],
                                 "unconfirmed": [{"x": 1.0, "y": 1.0}] * 100}, "f")
        self.tap.view.on_path([(1.0, 0.5, 0.9), (float("nan"), 0.0, 0.9)] + [(0.01 * i, 0.0, 0.9) for i in range(6000)])
        self.step()
        snap = self.gw.autonomy.snapshot()
        self.assertEqual((len(snap["people"]["people"]), len(snap["people"]["unconfirmed"])), (150, 50))   # 합쳐 200명, 확정 먼저
        self.assertEqual(len(snap["path"]["points"]), ros_view.MAX_PATH_POINTS)                          # 앞으로 갈 끝쪽
        self.tap.view.on_mission_state("FOLLOW")
        self.assertEqual(self.step()["mission"]["state"], "FOLLOW")                                       # 다음 소식도 들어온다
        self.gw._receive({"schema_version": "x", "type": "pose"})                                        # 자율 창구가 거절해도
        self.assertEqual(self.gw._errors["autonomy_pose"], 1)                                             # 세기만 하고 지나간다

    def test_loop_keeps_running_after_an_error(self):   # A3: 한 바퀴가 예외로 끝나도 스레드는 다음 바퀴를 돈다
        calls = []

        def tick():
            calls.append(1)
            if len(calls) == 1:
                raise RuntimeError("boom")

        self.gw.tick = tick
        t = threading.Thread(target=self.gw._loop, daemon=True)
        t.start()
        time.sleep(0.3)
        self.gw._stop.set()
        t.join(1)
        self.assertGreater(len(calls), 2)
        self.assertEqual(self.gw._errors["tick"], 1)
        self.gw.link.close()
        with open(os.path.join(self.gw.link.log.dir, "events.jsonl"), encoding="utf-8") as f:
            errors = [e["detail"] for e in map(json.loads, f) if e["type"] == "ros_view_error"]
        self.assertEqual((errors[0]["kind"], errors[0]["error"]), ("tick", "RuntimeError: boom"))

    def test_autonomy_map_and_found_obstacles(self):   # 11/17: 자율 비행 웹이 그리는 지도(벽·장애물)를 관제 화면에도
        self.fly()
        st = self.step()
        self.assertEqual((st["autonomy"]["grid"], st["autonomy"]["discovered"]), (None, None))
        self.tap.view.on_grid(0.5, 2, 2, -1.0, -1.0, bytes([255, 0, 50, 100]))   # 모름·본 곳·의심·막힘
        self.tap.view.on_discovered({"obstacles": [{"x": 1.0, "y": 0.5, "sx": 0.4, "sy": 0.3, "sources": ["tof"]},
                                                   {"x": "bad"}], "replans": 2}, "f/discovered.json")
        st = self.step()
        self.assertEqual({k: st["autonomy"]["grid"][k] for k in ("ver", "w", "h", "blocked", "suspect", "seen")},
                         {"ver": 1, "w": 2, "h": 2, "blocked": 1, "suspect": 1, "seen": 1})
        g = self.gw.autonomy_grid()
        self.assertEqual((g["x0"], g["res"], g["frame"], base64.b64decode(g["cells"])), (-1.0, 0.5, "map", bytes([0, 1, 2, 3])))
        self.assertEqual(self.gw.autonomy_discovered()["obstacles"],   # 좌표가 숫자인 것만, 출처 이름은 그대로(화면 이름표)
                         [{"x": 1.0, "y": 0.5, "sx": 0.4, "sy": 0.3, "sources": ["tof"]}])
        self.assertEqual(st["autonomy"]["discovered"]["obstacles"], 1)
        with open(os.path.join(self.gw.autonomy.run_dir, "grid.json"), encoding="utf-8") as f:   # 회차 폴더에 마지막 지도
            self.assertEqual(json.load(f)["w"], 2)
        self.tap.view.on_discovered(None, None)                                     # 새 회차: 아직 파일 없음 → 비움
        self.assertIsNone(self.step()["autonomy"]["discovered"])

    def test_world_map_and_truth_like_the_autonomy_web(self):   # 11/17 "똑같이": 시뮬레이터 세계 지도의 벽·가구·목적지와 정답 위치
        self.fly()
        st = self.step()
        self.assertEqual((st["autonomy"]["world_map"], st["autonomy"]["truth"]), (None, None))
        self.tap.view.on_world_map({"bounds": {"xmin": 0, "ymin": 0, "xmax": 12, "ymax": 10}, "spawn": {"x": 6.45, "y": 6.25},
                                    "goal": {"x": 9, "y": 2}, "description": "세계",
                                    "walls": [{"x": 1, "y": 0, "sx": 1.7, "sy": 0.1}, {"x": "bad"}],
                                    "obstacles": [{"x": 4.6, "y": 3.8, "sx": 1.6, "sy": 1.3, "yaw": 3.1416, "kind": "desk"}]},
                                   "w/known_map.json")
        self.tap.view.on_truth(1.0, 2.0, 0.9, 0.5)
        st = self.step()
        self.assertEqual({k: st["autonomy"]["world_map"][k] for k in ("open", "walls", "obstacles")},
                         {"open": False, "walls": 1, "obstacles": 1})
        m = self.gw.autonomy_world_map()
        self.assertEqual((m["bounds"]["xmax"], m["spawn"], m["goal"], m["walls"], m["obstacles"][0]["yaw"]),
                         (12.0, {"x": 6.45, "y": 6.25}, {"x": 9.0, "y": 2.0}, [{"x": 1.0, "y": 0.0, "sx": 1.7, "sy": 0.1}], 3.1416))
        self.assertEqual(st["autonomy"]["truth"], {"x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.5})
        with open(os.path.join(self.gw.autonomy.run_dir, "map.json"), encoding="utf-8") as f:   # 회차 폴더에도
            self.assertEqual(json.load(f)["description"], "세계")

    def test_autonomy_warnings_go_to_the_flight_log(self):   # 11/17 충돌 뒤: 비행 노드 경고·오류를 비행 기록에
        self.tap.view.on_log(30, "tt_mission", "게이트웨이를 켜기 전 경고")      # 처음 붙을 때 있던 것: 지난 일
        self.tap.state(GROUND)
        self.step()
        self.tap.view.on_log(30, "tt_tello_driver", "영상 패킷이 5초 동안 오지 않습니다")   # 땅의 경고: 안 적음
        self.step()
        self.fly()
        self.step()
        for h in ("0.32", "0.30"):                                                 # 숫자만 다른 같은 말: 30초에 한 번
            self.tap.view.on_log(30, "tt_mission", f"바로 밑 {h} m에 무엇이 있습니다")
            self.step()
        self.tap.view.on_log(30, "tt_mission", "부딪힘, 닿은 쪽 기체 왼쪽 뒤")
        self.step()
        self.tap.state(GROUND)                                                     # 내려앉음
        self.step()
        self.tap.view.on_log(40, "tt_mission", "드라이버가 기체를 내렸습니다 (flipped)")
        self.step(15)                                                              # 오류는 늘 적는다
        self.tap.view.on_log(30, "tt_mission", "내린 뒤 15초의 경고")              # 내린 뒤 10초 지남: 안 적음
        self.step()
        told = [(d["level"], d["text"]) for d in (m["detail"] for m in self.sent if m.get("kind") == "autonomy_log")]
        self.assertEqual(told, [("warn", "바로 밑 0.32 m에 무엇이 있습니다"), ("warn", "부딪힘, 닿은 쪽 기체 왼쪽 뒤"),
                                ("error", "드라이버가 기체를 내렸습니다 (flipped)")])
        self.gw.link.close()
        with open(os.path.join(self.gw.link.log.dir, "events.jsonl"), encoding="utf-8") as f:
            logged = [e["detail"]["node"] for e in map(json.loads, f) if e["type"] == "autonomy_log"]
        self.assertEqual(logged, ["tt_mission"] * 3)

    def test_height_alerts_like_the_crash(self):   # 11/17 충돌: 추정은 높게 굳고 실제로는 바닥 가까이 → 알리기만
        def state(real, est):
            raw = FLYING.replace("tof:92;", f"tof:{round(real * 100)};").replace("baro:12.50", f"baro:{12.5 + real:.2f}")
            self.tap.state(raw, flying=True)
            self.tap.view.on_pose(1.0, 0.5, est, 0.3, 0.1)
            self.tap.view.on_mission_state("FOLLOW")
            return self.step()

        for _ in range(60):                       # 6초: 바닥 기압을 모은다(추정 = 실제)
            st = state(0.92, 0.92)
        self.assertEqual((st["height_watch"]["baro_ref_m"], st["height_watch"]["gap"]), (12.5, None))
        for _ in range(35):                       # 3.5초: 실제 0.35 m인데 추정은 0.92 m
            st = state(0.35, 0.92)
        self.assertIn("low_altitude", st["warnings"])
        self.assertIn("height_mismatch", st["warnings"])
        gap = st["height_watch"]["gap"]
        self.assertEqual((gap["est_m"], gap["baro_m"], gap["tof_m"]), (0.92, 0.35, 0.35))
        self.assertEqual(st["mode"], "auto")      # 비행에는 끼어들지 않는다
        self.tap.state(GROUND)                    # 땅: 알림 끝
        st = self.step()
        self.assertNotIn("height_mismatch", st["warnings"])
        told = [(d["what"], d["on"], d.get("end")) for d in (m["detail"] for m in self.sent if m.get("kind") == "height_watch")]
        self.assertEqual(told, [("low", True, None), ("gap", True, None), ("low", False, True), ("gap", False, True)])

    def test_last_map_and_found_obstacles_stay_in_the_run(self):   # 11/17 실기체: 5초 묶음에 걸린 마지막 판이 회차에 안 남았다
        def grid_file():
            with open(os.path.join(self.gw.autonomy.run_dir, "grid.json"), encoding="utf-8") as f:
                return base64.b64decode(json.load(f)["cells"])

        def keep_open(dt):   # 비행 노드가 단계를 1초마다 내듯 자율 연결을 열어 둔다
            self.tap.view.on_mission_state("LOOK")
            return self.step(dt)

        self.fly()
        self.step()
        self.tap.view.on_grid(0.5, 2, 1, 0.0, 0.0, bytes([0, 0]))
        self.step()
        self.assertEqual(grid_file(), bytes([1, 1]))                 # 처음 판은 바로
        self.tap.view.on_grid(0.5, 2, 1, 0.0, 0.0, bytes([100, 0]))
        self.step()
        self.assertEqual(grid_file(), bytes([1, 1]))                 # 5초 안에 온 판: 아직
        keep_open(2.6)
        keep_open(2.6)
        self.assertEqual(grid_file(), bytes([3, 1]))                 # 새 판이 안 와도 5초 뒤에 남는다
        self.tap.view.on_grid(0.5, 2, 1, 0.0, 0.0, bytes([100, 100]))
        self.tap.view.on_discovered({"obstacles": [{"x": 1.0, "y": 0.0, "sx": 0.4, "sy": 0.4, "sources": ["yolo:chair", 7]}]},
                                    "f/discovered.json")
        keep_open(0.1)
        self.gw.close()
        self.assertEqual(grid_file(), bytes([3, 3]))                 # 끌 때 마지막 판
        with open(os.path.join(self.gw.autonomy.run_dir, "discovered.json"), encoding="utf-8") as f:
            self.assertEqual(json.load(f)["obstacles"], [{"x": 1.0, "y": 0.0, "sx": 0.4, "sy": 0.4, "sources": ["yolo:chair"]}])

    def test_photo_gets_the_pose_and_autonomy_closes_when_silent(self):
        self.fly()
        self.step()
        self.assertEqual(self.gw._brief()["pose"]["x_m"], 1.0)                  # 탐지 사진에 그때 자율 위치
        self.step(ros_view.AUTONOMY_STALE_S + 1)                                # 비행 노드가 꺼져 위치·단계가 안 옴
        st = self.gw.drone_state()
        self.assertFalse(st["autonomy"]["connected"])
        self.assertIsNone(st["mission"])
        self.assertIsNone(self.gw._brief()["pose"])

    def test_mission_phases(self):
        self.assertEqual([ros_view.mission_phase(s) for s in ("WAIT_POSE", "FOLLOW", "DONE", "ABORT")],
                         ["idle", "running", "arrived", "failed"])

    def hold(self, raw, seconds, xy, flying=False, dt=0.1):
        """seconds 동안 dt마다 Tello 상태와 자율 위치를 새로 보낸다."""
        st = None
        for _ in range(round(seconds / dt)):
            self.tap.state(raw, flying=flying)
            self.tap.view.on_pose(xy[0], xy[1], 0.9 if flying else 0.0, 0.0, None)
            st = self.step(dt)
        return st

    def test_flight_time_and_start_point(self):   # 시뮬레이터 Tello는 모터 시간이 늘 0이고, 위치는 지도 좌표라 원점이 출발이 아니다
        self.tap.view.on_mission_state("READY")
        st = self.hold(GROUND, 1.0, (2.0, 1.5))
        self.assertEqual((st["flight_time_s"], st["autonomy"]["start_pose"]), (0.0, {"x_m": 2.0, "y_m": 1.5}))
        st = self.hold(GROUND, 1.0, (2.2, 1.4))                                 # 땅에서 위치를 다시 잡으면 출발도 따라간다
        self.assertEqual(st["autonomy"]["start_pose"], {"x_m": 2.2, "y_m": 1.4})
        st = self.hold(FLYING, 10.0, (3.0, 2.0), flying=True)                  # 첫 0.1초 뒤 auto → 9.9초 떠 있음
        self.assertEqual(st["mode"], "auto")
        self.assertEqual(st["autonomy"]["start_pose"], {"x_m": 2.2, "y_m": 1.4})   # 뜬 뒤에는 처음 뜬 자리 그대로
        self.assertAlmostEqual(st["flight_time_s"], 9.9, delta=0.05)
        st = self.hold(GROUND, 5.0, (2.5, 1.0))                                 # 땅을 본 첫 0.1초까지 센다
        self.assertEqual(st["mode"], "grounded")
        self.assertAlmostEqual(st["flight_time_s"], 10.0, delta=0.05)              # 땅에서는 세지 않는다
        self.assertEqual(st["autonomy"]["start_pose"], {"x_m": 2.2, "y_m": 1.4})   # 내려와도 출발은 처음 뜬 자리
        self.gw.link.close()
        with open(os.path.join(self.gw.link.log.dir, "metrics.json"), encoding="utf-8") as f:
            self.assertAlmostEqual(json.load(f)["flight_time_s"], 10.0, delta=0.05)

    def test_no_start_point_when_autonomy_joins_in_the_air(self):   # 출발을 모르면 화면은 지도 원점에 그린다
        st = self.hold(FLYING, 1.0, (3.0, 2.0), flying=True)
        self.assertEqual((st["autonomy"]["ground_origin"], st["autonomy"]["start_pose"]), (False, None))


class GatewayCommandTest(GatewayCase):
    """3단계: 화면 버튼 → 보기 노드 → 자율 비행 웹과 같은 명령. 버튼을 여는 조건도 웹과 같다."""

    def setUp(self):
        super().setUp()
        self.rec = Recorder()
        self.tap.view.publish = self.rec   # 명령을 받는 보기 노드
        self.tap.view.clock = self.clock

    def cmd(self, name, **args):
        r = self.gw.handle_message({"type": "cmd", "id": "c1", "name": name, "args": args, "client": "monitor"})
        return r["ok"], r["reason"]

    def test_start_opens_like_the_web(self):
        self.tap.state(GROUND, can_takeoff=True)
        self.tap.view.on_mission_state("WAIT_POSE")
        st = self.step()
        self.assertEqual(st["allowed"], ["emergency", "confirm_area"])   # 위치 잡는 중: 시작은 닫힘
        self.assertEqual((st["ros_view"]["commands"], st["ros_view"]["start"]["mission"]), (True, "WAIT_POSE"))
        self.assertEqual(self.cmd("takeoff"), (False, "not_allowed_in_mode"))
        self.tap.state(GROUND, can_takeoff=False)   # 드라이버 점검 미통과(주변 확인 전 등)
        self.tap.view.on_mission_state("READY")
        self.assertNotIn("takeoff", self.step()["allowed"])
        self.tap.state(GROUND, can_takeoff=True)
        self.assertIn("takeoff", self.step()["allowed"])
        self.tap.view.need_yolo = True              # 웹 need_yolo: YOLO 준비 전이면 시작 닫힘
        st = self.step()
        self.assertEqual(("takeoff" in st["allowed"], st["ros_view"]["start"]["yolo"]), (False, False))
        self.tap.view.on_detections("{}")
        self.assertIn("takeoff", self.step()["allowed"])
        self.assertEqual(self.cmd("confirm_area", confirmed=True), (True, None))
        self.assertEqual(self.cmd("takeoff"), (True, None))
        self.assertEqual(self.rec, [(FLIGHT_CMD, "confirm"), (MISSION_CMD, "start")])

    def test_flight_buttons_like_the_web(self):
        self.tap.state(FLYING, flying=True)
        self.tap.view.on_mission_state("FOLLOW")
        self.assertEqual(self.step()["allowed"], ["emergency", "land", "goto"])
        self.assertEqual(self.cmd("goto", x=1.5, y=2.0), (True, None))
        self.assertEqual(self.cmd("goto", x="a", y=2), (False, "bad_xy"))
        self.tap.view.on_goal(json.dumps({"seq": 1, "state": "going", "click": [1.5, 2.0], "goal": [1.4, 1.9], "text": "가는 중"}))
        st = self.step()
        self.assertIn("goto_cancel", st["allowed"])
        self.assertEqual((st["mission"]["goal"]["click"], st["mission"]["goal"]["goal"]), ([1.5, 2.0], [1.4, 1.9]))
        self.assertEqual(self.cmd("goto_cancel"), (True, None))
        self.assertEqual(self.cmd("confirm_area", confirmed=True), (False, "not_allowed_in_mode"))   # 떠 있으면 주변 확인 없음
        self.assertEqual(self.cmd("land"), (True, None))
        self.assertEqual(self.cmd("emergency"), (True, None))
        self.tap.view.on_mission_state("LAND")
        self.assertEqual(self.step()["allowed"], ["emergency", "land", "goto_cancel"])   # 착륙 단계: 찍은 자리는 못 보냄
        self.assertEqual(self.rec, [(MISSION_CMD, "goto 1.50 2.00"), (MISSION_CMD, "goto cancel"), (MISSION_CMD, "land"),
                                    (FLIGHT_CMD, "emergency"), (FLIGHT_CMD, "emergency")])
        self.assertEqual(self.gw.commands, 4)
        self.gw.link.close()
        with open(os.path.join(self.gw.link.log.dir, "events.jsonl"), encoding="utf-8") as f:
            cmds = [e["detail"] for e in map(json.loads, f) if e["type"] == "gateway_cmd"]
        self.assertEqual([(c["name"], c["ok"]) for c in cmds][:3], [("goto", True), ("goto", False), ("goto_cancel", True)])
        self.assertEqual(cmds[0]["sent"], ["goto 1.50 2.00"])

    def test_refusals(self):
        self.tap.state(GROUND, can_takeoff=True)
        self.tap.view.on_mission_state("READY")
        self.step()
        self.assertEqual(self.cmd("pause"), (False, "ros_view_not_available"))   # 자율 비행 웹에 없는 조작
        self.assertEqual(self.cmd("video_on"), (False, "ros_view_not_available"))
        self.assertEqual(self.cmd("confirm_area", confirmed="yes"), (False, "invalid_command_args"))
        self.tap.close()
        self.step(ros_view.TAP_STALE_S + 0.5)
        self.assertEqual(self.cmd("emergency"), (False, "ros_view_unreachable"))
        self.assertEqual(self.rec, [])

    def test_driver_landing_reasons_go_to_the_log(self):
        old = {"at": "t0", "reason": "battery_low"}   # 지난 비행의 자동 착륙
        self.tap.state(GROUND, last_auto_land=old)
        self.step()
        self.tap.state(FLYING, flying=True, last_auto_land=old)
        self.step()
        self.tap.state(FLYING, flying=True, busy="land", last_auto_land=old)
        self.assertEqual(self.step()["last_transition"]["reason"], "autonomy_landing")   # 지난 것은 이번 까닭이 아니다
        self.tap.state(GROUND, last_auto_land=old)
        self.step()
        self.tap.state(FLYING, flying=True, last_auto_land=old)
        self.step()
        self.tap.state(FLYING, flying=True, busy="land", landing_reason="cmd_timeout", last_auto_land=old)
        self.assertEqual(self.step()["last_transition"]["reason"], "cmd_timeout")      # 드라이버: 속도 명령 끊김
        self.tap.state(GROUND, landing_reason="cmd_timeout", last_auto_land=old)
        self.assertEqual(self.step()["last_transition"]["reason"], "cmd_timeout")
        self.tap.state(FLYING, flying=True, last_auto_land=old)
        self.step()
        self.tap.state(GROUND, last_auto_land={"at": "t1", "reason": "flipped"})   # 뒤집힘: 착륙 없이 모터를 끈다
        self.assertEqual(self.step()["last_transition"]["reason"], "flipped")

    REFUSED = {"cmd": "takeoff", "source": "autonomy", "ok": False, "flying": False,   # 11/16 실기체 첫 takeoff
               "result": {"cmd": "takeoff", "response": "error", "ok": False, "reason": "error_response",
                          "sent_at": "2026-11-16T05:56:31.978+00:00"}}

    def told(self):
        return [m["detail"] for m in self.sent if m.get("kind") == "command_rejected"]

    def test_driver_refusal_is_told_once(self):   # 드론이 이륙을 거절하면 화면에 알린다(드론 응답 그대로). 같은 결과는 한 번만
        self.tap.state(GROUND)   # 막 켠 드라이버: 결과 없음 → 다음 결과부터 새것
        self.step()
        self.tap.state(GROUND, last_result=self.REFUSED)
        self.step()
        self.step()
        self.tap.state(GROUND, last_result={"cmd": "takeoff", "source": "autonomy", "ok": False, "reason": "preflight_failed",
                                            "failed": ["operator"]})
        self.step()
        self.tap.state(FLYING, flying=True, last_result={"cmd": "takeoff", "source": "autonomy", "ok": True, "flying": True,
                                                         "result": {"response": "ok", "sent_at": "t2"}})
        self.step()   # 성공은 알리지 않는다
        self.assertEqual([(d["name"], d["reason"], d["by"], d["response"], d["failed"]) for d in self.told()],
                         [("takeoff", "error_response", "autonomy_driver", "error", None),
                          ("takeoff", "preflight_failed", "autonomy_driver", None, ["operator"])])
        self.gw.link.close()
        with open(os.path.join(self.gw.link.log.dir, "events.jsonl"), encoding="utf-8") as f:
            logged = [e["detail"]["reason"] for e in map(json.loads, f) if e["type"] == "driver_result"]
        self.assertEqual(logged, ["error_response", "preflight_failed"])

    def test_old_driver_result_is_not_told(self):   # 게이트웨이를 켜기 전 결과(지난 비행)는 알리지 않는다
        self.tap.state(GROUND, last_result=self.REFUSED)
        self.step()
        self.step()
        self.assertEqual(self.told(), [])
        self.tap.state(GROUND, last_result={**self.REFUSED, "result": {**self.REFUSED["result"], "sent_at": "t3"}})
        self.step()   # 같은 응답이라도 새 요청이면 알린다
        self.assertEqual(len(self.told()), 1)

    def test_yaw_drift_warns_but_does_not_block_the_start(self):   # 11/17 비행 노드: 땅에서 기수 값이 흐르면 첫 시작을 받지 않는다
        def ground(deg):
            self.tap.state(GROUND.replace("yaw:5;", f"yaw:{deg};"), can_takeoff=True)
            return self.step()

        def settle(st):
            return st["ros_view"]["start"]["yaw_span_deg"], st["ros_view"]["start"]["yaw_drifting"]

        self.tap.view.on_mission_state("READY")
        self.assertEqual(settle(ground(5)), (None, False))   # 3초도 안 쌓임: 모른다
        for i in range(40):                                   # 4초 동안 1°/초로 흐른다(Tello는 1° 단위)
            st = ground(5 + (i + 1) // 10)
        self.assertEqual(settle(st), (4.0, True))
        self.assertIn("takeoff", st["allowed"])               # 막지 않는다(받을지는 비행 노드가 정한다: 30초 안 두 번째는 받는다)
        self.assertEqual(self.cmd("takeoff"), (True, None))
        for _ in range(120):                                  # 멈추고 10초 넘게 지나면 흐르지 않음
            st = ground(9)
        self.assertEqual(settle(st), (0.0, False))
        self.tap.state(FLYING.replace("yaw:5;", "yaw:179;"), flying=True)
        self.assertEqual(settle(self.step()), (None, False))   # 뜨면 비운다(자율 비행 웹과 같게)
        self.gw.link.close()
        with open(os.path.join(self.gw.link.log.dir, "events.jsonl"), encoding="utf-8") as f:
            cmds = [e["detail"] for e in map(json.loads, f) if e["type"] == "gateway_cmd"]
        self.assertEqual([(c["name"], c["yaw_span_deg"], c["yaw_drifting"]) for c in cmds], [("takeoff", 4.0, True)])

    def test_emergency_from_this_screen_is_the_reason(self):
        self.tap.state(FLYING, flying=True)
        self.tap.view.on_mission_state("FOLLOW")
        self.step()
        self.assertEqual(self.cmd("emergency"), (True, None))
        self.tap.state(GROUND)   # 모터가 꺼져 땅
        self.assertEqual(self.step()["last_transition"]["reason"], "emergency")


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
        self.assertEqual(client.get("/api/autonomy/grid").status_code, 404)        # 자율 비행 지도를 아직 못 받음
        self.assertEqual(client.get("/api/autonomy/discovered").status_code, 404)
        self.assertEqual(client.get("/api/autonomy/map").status_code, 404)              # 지도 파일을 아직 못 받음
        app.state.gateway._world_map = {"bounds": None, "spawn": {"x": 0.0, "y": 0.0}, "goal": None, "walls": [], "obstacles": [],
                                        "open": True, "description": None, "frame": "map"}
        self.assertTrue(client.get("/api/autonomy/map").json()["open"])
        app.state.gateway._grid = {"ver": 3, "w": 1, "h": 1, "res": 0.1, "x0": 0.0, "y0": 0.0, "cells": "Aw==", "frame": "map"}
        self.assertEqual(client.get("/api/autonomy/grid").json()["cells"], "Aw==")
        self.assertEqual(client.get("/api/autonomy/trail").json()["points"], [])   # 새로 연 화면이 지나온 길을 채우는 곳

    def test_mapping_video_carries_sensor_pose(self):   # 지도 파트가 받는 영상(/video.mjpg?mapping=1): 960×720 장마다 그때 센서 자세
        try:
            import cv2
            import numpy as np
            import gateway_server
        except (ImportError, RuntimeError) as e:
            self.skipTest(f"없음: {e}")
        import asyncio
        from unittest.mock import PropertyMock, patch
        from tello_link import parse_state
        tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, tmp, True)
        def first_part(sensor_pose):
            args = argparse.Namespace(ros_view="http://127.0.0.1:9", log_root=tmp, fake=False, replay=None,
                                      no_video=False, allow_remote_autonomy=True, ros_view_sensor_pose=sensor_pose)
            app = gateway_server.build_app(args)
            gw = app.state.gateway
            self.addCleanup(gw.link.close)
            gw.video.size = (480, 360)                               # 실기체 자율 비행 드라이버가 줄여 낸 크기
            gw.link.note_state(parse_state(FLYING, received_mono=time.monotonic()))
            gw.video.put(cv2.imencode(".jpg", np.zeros((360, 480, 3), np.uint8))[1].tobytes())
            route = next(r for r in app.routes if getattr(r, "path", None) == "/video.mjpg")

            class Client:
                async def is_disconnected(self):
                    return False

            async def read():
                response = await route.endpoint(Client(), True)
                part = await anext(response.body_iterator)
                await response.body_iterator.aclose()
                return response, part

            with patch.object(ros_view.ViewVideo, "running", new_callable=PropertyMock, return_value=True):
                response, part = asyncio.run(read())
            head, body = part.split(b"\r\n\r\n", 1)
            values = dict(line.split(b": ", 1) for line in head.split(b"\r\n")[1:])
            h, w = cv2.imdecode(np.frombuffer(body[:-2], np.uint8), cv2.IMREAD_COLOR).shape[:2]
            return gw, response.headers, values, (w, h)

        gw, headers, values, size = first_part(True)
        self.assertEqual(headers["X-Tello-Sensor-Pose"], "1")       # 지도 파트: 영상으로 원점을 잡지 말고 센서 자세로
        self.assertEqual(json.loads(values[b"X-Tello-Pose"])[3], 0.92)   # [구간, x, y, z, yaw, pitch, roll], 높이는 하방 ToF
        self.assertEqual(json.loads(values[b"X-Tello-Frame"])["run_id"], gw.link.run_id)
        self.assertEqual(size, (960, 720))
        _, headers, values, size = first_part(False)                 # 시뮬레이터: 크기만 맞추고 자세는 없다
        self.assertNotIn("X-Tello-Sensor-Pose", headers)
        self.assertNotIn(b"X-Tello-Pose", values)
        self.assertEqual(size, (960, 720))

    def test_events_route_keeps_this_runs_log(self):   # 새로 연 화면이 비행 기록을 채운다(화면으로 보낸 알림과 같은 모양)
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
        gw = app.state.gateway
        self.addCleanup(gw.link.close)
        client = TestClient(app)
        self.assertEqual(client.get("/api/events").json(), {"events": []})
        gw._emit({"type": "event", "kind": "cmd_ignored", "at": "t1"})        # 1초마다 오는 것은 빼고
        gw._emit({"type": "detection", "event": {}})                          # 탐지 사진도 빼고
        review = {"type": "detection_review", "finding_id": "finding-0001", "review": "confirmed", "at": "t2"}
        gw._emit(review)
        self.assertEqual(client.get("/api/events").json(), {"events": [review]})
        for i in range(gateway_server.EVENT_HISTORY + 5):
            gw._emit({"type": "event", "kind": "video_changed", "at": f"v{i}", "detail": {"on": True}})
        got = client.get("/api/events").json()["events"]
        self.assertEqual(len(got), gateway_server.EVENT_HISTORY)               # 최근 것만, 오래된 순
        self.assertEqual((got[0]["at"], got[-1]["at"]), ("v5", f"v{gateway_server.EVENT_HISTORY + 4}"))
        self.assertNotIn("cmd_ignored", {m.get("kind") for m in got})


if __name__ == "__main__":
    unittest.main()
