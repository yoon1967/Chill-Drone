"""자율 비행 연결(/ws/autonomy)을 게이트웨이·가짜 Tello로 시험한다 (WebSocket 없이 게이트웨이 함수를 바로 부른다).

계약 AUTONOMY_BRIDGE_V0_1 8장의 검증 항목 중 게이트웨이 쪽: auto가 아닐 때 cmd_vel 무시, 넘겨받기 뒤 자율 명령 막힘,
control stop, 끊김(연결 끊기면 paused), 이륙 뒤 연결한 자율의 시작 거절, 두 번째 연결 busy, 위치·경로·찾은 사람 전달.
0.5초·2초 끊김 판정은 test_mode_arbiter.py가 시계를 돌려 본다.

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_autonomy.py" -v
"""

import json
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from autonomy_link import AutonomyLink, tello_raw_text  # noqa: E402
from fake_autonomy import FakeMission  # noqa: E402
from tello_link import parse_state  # noqa: E402
from test_gateway import GatewayCase, wait_until  # noqa: E402

V = "0.1.0"


def msg(kind, **kw):
    return {"schema_version": V, "type": kind, **kw}


class AutonomyLinkTest(unittest.TestCase):
    def test_raw_text_round_trip(self):   # 자율 쪽 해석기가 원문을 읽으므로 같은 값으로 되돌아와야 한다
        packet = "mid:-1;x:-100;y:-100;z:-100;mpry:0,0,0;pitch:-8;roll:1;yaw:54;vgx:0;vgy:0;vgz:0;templ:62;temph:65;tof:10;h:0;bat:100;baro:34.09;time:0;agx:-99.00;agy:-28.00;agz:-1004.00;"
        raw = parse_state(packet).raw
        self.assertEqual(parse_state(tello_raw_text(raw)).raw, raw)
        self.assertIn("agx:-99.00;", tello_raw_text(raw))

    def test_one_client_and_checks(self):
        a = AutonomyLink()
        self.assertTrue(a.open("au-1"))
        self.assertFalse(a.open("au-2"))                 # 두 번째는 busy
        with self.assertRaises(ValueError):
            a.receive({"type": "pose"})                  # schema_version 없음
        with self.assertRaises(ValueError):
            a.receive(msg("pose", x_m=1, y_m=2, yaw_rad=0, frame="body"))   # 위치는 map 기준만
        a.receive(msg("pose", x_m=1.0, y_m=2.0, z_m=1.0, yaw_rad=0.5, frame="map"))
        self.assertEqual(a.pose_state()["x_m"], 1.0)
        self.assertTrue(a.pose_state()["valid"])
        self.assertEqual(a.status()["rejected"], 2)
        self.assertFalse(a.close("au-2"))
        self.assertTrue(a.close("au-1"))
        self.assertEqual(a.pose_state()["reason"], "autonomy_disconnected")   # 마지막 값은 남기되 무효로

    def test_people_revision_only_when_changed(self):
        a = AutonomyLink()
        a.open("au-1")
        person = {"x": 1.0, "y": 2.0, "conf": 0.8, "hits": 2, "gone": False, "t_first": 3.0, "top": 1.6}
        start = a.people_revision
        a.receive(msg("people", frame="map", people=[person], unconfirmed=[]))
        a.receive(msg("people", frame="map", people=[person], unconfirmed=[]))
        self.assertEqual(a.people_revision, start + 1)
        a.receive(msg("people", frame="map", people=[{**person, "gone": True}], unconfirmed=[]))
        self.assertEqual((a.people_revision, a.status()["people"]), (start + 2, {"confirmed": 0, "gone": 1, "unconfirmed": 0}))
        snap = a.snapshot()["people"]["people"][0]
        self.assertEqual((snap["x_m"], snap["y_m"], snap["top_m"], snap["gone"]), (1.0, 2.0, 1.6, True))

    def test_new_connection_starts_empty(self):   # 새 자율은 새 지도. 지난 연결의 경로·찾은 사람을 화면에 남기지 않는다
        a = AutonomyLink()
        a.open("au-1")
        a.receive(msg("path", points=[[0, 0, 1], [2, 0, 1]], frame="map"))
        a.receive(msg("people", frame="map", people=[{"x": 1, "y": 1, "conf": 0.9, "hits": 2}], unconfirmed=[]))
        a.close("au-1")
        revs = (a.path_revision, a.people_revision)
        a.open("au-2")
        snap = a.snapshot()
        self.assertEqual((snap["path"], snap["people"], snap["pose"]), (None, None, None))
        self.assertGreater(a.path_revision, revs[0])   # 화면이 다시 읽어 비운다
        self.assertGreater(a.people_revision, revs[1])


class GatewayAutonomyTest(GatewayCase):
    def setUp(self):
        super().setUp()
        self.to_autonomy = []

    def open(self, key="au-1"):
        return self.gw.autonomy_open(key, self.to_autonomy.append)

    def out(self, kind):   # 게이트웨이 → 자율로 나간 메시지
        return [m for m in self.to_autonomy if m.get("type") == kind]

    def say(self, kind, key="au-1", **kw):
        return self.gw.autonomy_message(key, msg(kind, **kw))

    def start_mission(self):
        self.assertTrue(self.open())
        self.take_off()
        r = self.cmd("auto_start")
        self.assertTrue(r["ok"], r)
        start = self.out("control")[-1]
        self.assertEqual(start["cmd"], "start")
        self.say("ack", id=start["id"], ok=True)
        return start

    def test_hello_state_and_status_go_out(self):
        self.assertTrue(self.open())
        hello = self.out("hello")[0]
        self.assertEqual((hello["source"], hello["max_speed_mps"], hello["height_limit_m"]), ("fake", 0.3, 1.5))
        self.assertTrue(wait_until(lambda: self.out("tello_state") and self.out("flight_status")))
        self.assertIn("bat:", self.out("tello_state")[-1]["raw"])
        self.assertEqual(self.out("flight_status")[-1]["mode"], "grounded")
        self.assertTrue(self.gw.drone_state()["autonomy"]["connected"])
        self.assertTrue(self.gw.drone_state()["autonomy"]["ground_origin"])   # 지상에서 붙었다

    def test_second_client_is_busy(self):
        self.assertTrue(self.open("au-1"))
        self.assertFalse(self.gw.autonomy_open("au-2", self.to_autonomy.append))
        self.assertIsNone(self.say("pose", key="au-2", x_m=0, y_m=0, yaw_rad=0, frame="map"))   # 그 연결의 말은 듣지 않는다

    def test_cmd_vel_only_in_auto_then_takeover_blocks(self):
        self.assertTrue(self.open())
        self.take_off()
        n = len(self.rc_sent())
        self.say("cmd_vel", vx_mps=0.3, vy_mps=0, vz_mps=0, yaw_rate_rps=0, frame="body", seq=1)   # manual: 무시
        self.assertIn("cmd_ignored", [m.get("kind") for m in self.sent_events()])
        self.cmd("auto_start")
        start = self.out("control")[-1]
        self.say("ack", id=start["id"], ok=True)
        for i in range(12):   # 10Hz로 0.6초
            self.say("cmd_vel", vx_mps=0.3, vy_mps=0, vz_mps=0, yaw_rate_rps=0, frame="body", seq=2 + i)
            time.sleep(0.05)
        self.assertTrue(wait_until(lambda: any(c.split()[2] != "0" for c in self.rc_sent()[n:]), timeout=2),
                        self.rc_sent()[n:][-5:])   # 앞으로(rc 둘째 값)가 나갔다
        self.rc([0.6, 0, 0, 0])   # 사람이 조이스틱을 밀면 넘겨받기
        self.assertEqual(self.gw.arbiter.mode, "manual")
        self.assertEqual(self.out("control")[-1]["cmd"], "stop")
        self.say("cmd_vel", vx_mps=0.3, vy_mps=0, vz_mps=0, yaw_rate_rps=0, frame="body", seq=99)
        self.assertEqual(self.gw.arbiter.mode, "manual")   # 자율 명령으로 다시 auto가 되지 않는다

    def sent_events(self):
        return [m for m in self.sent if m.get("type") == "event"]

    def test_disconnect_in_auto_pauses(self):
        self.start_mission()
        self.assertEqual(self.gw.arbiter.mode, "auto")
        self.gw.autonomy_closed("au-1")
        self.assertEqual(self.gw.arbiter.mode, "paused")
        self.assertIn("autonomy_disconnected", self.gw.drone_state()["warnings"])
        self.assertFalse(self.gw.drone_state()["autonomy"]["connected"])

    def test_joined_in_air_cannot_start(self):
        self.take_off()
        self.assertTrue(self.open())
        r = self.cmd("auto_start")
        self.assertEqual((r["ok"], r["reason"]), (False, "autonomy_joined_in_air"))

    def test_mission_end_and_requests(self):
        self.start_mission()
        self.say("mission_state", state="FOLLOW", phase="running", progress=0.5, reason=None)
        self.assertEqual(self.gw.drone_state()["mission"]["progress"], 0.5)
        r = self.say("request", name="takeoff")
        self.assertEqual((r["type"], r["ok"], r["reason"]), ("request_result", False, "takeoff_by_operator_only"))
        self.say("mission_state", state="ARRIVED", phase="arrived", progress=1.0, reason="route_done")
        self.assertEqual(self.gw.arbiter.mode, "paused")   # 도착하면 사람이 착륙을 정한다

    def test_pose_path_people_reach_screen(self):
        self.assertTrue(self.open())
        self.say("pose", x_m=1.2, y_m=-0.4, z_m=1.0, yaw_rad=1.57, sigma_m=0.05, frame="map")
        self.say("path", points=[[0, 0, 1], [2, 0, 1]], frame="map")
        self.say("people", frame="map", people=[{"x": 1.5, "y": 0.5, "conf": 0.9, "hits": 2, "gone": False, "t_first": 1.0}],
                 unconfirmed=[])
        st = self.gw.drone_state()
        self.assertEqual((st["pose"]["x_m"], st["pose"]["frame"], st["pose"]["valid"]), (1.2, "map", True))
        self.assertEqual((st["autonomy"]["path_revision"], st["autonomy"]["people"]["confirmed"]), (2, 1))   # 연결 때 1, 경로 받고 2
        snap = self.gw.autonomy.snapshot()
        self.assertEqual(len(snap["path"]["points"]), 2)
        bad = self.say("path", points=[[0, "x"]], frame="map")
        self.assertEqual((bad["type"], bad["reason"]), ("error", "invalid_path"))

    def test_run_folder_records(self):
        self.start_mission()
        self.say("mission_state", state="FOLLOW", phase="running", progress=0.2, reason=None)
        with open(os.path.join(self.link.log.dir, "autonomy", "events.jsonl"), encoding="utf-8") as f:
            kinds = [json.loads(line)["kind"] for line in f]
        self.assertEqual(kinds[:1], ["connected"])
        self.assertIn("control", kinds)
        self.assertIn("mission_state", kinds)


class FakeMissionTest(unittest.TestCase):
    def run_until_done(self, m, limit=120.0):
        t = 0.0
        while m.phase == "running" and t < limit:
            m.step(0.1, moving=True)
            t += 0.1
        return t

    def test_square_finds_people_and_one_leaves(self):
        m = FakeMission()
        self.assertEqual(m.control("start"), (True, None))
        t = self.run_until_done(m)
        self.assertEqual(m.phase, "arrived")
        self.assertLess(t, 60)
        self.assertLess(abs(m.x) + abs(m.y), 0.2)    # 출발점으로 돌아왔다
        people = m.people()
        self.assertEqual(len(people["people"]), 2)
        self.assertEqual(sorted(q["gone"] for q in people["people"]), [False, True])

    def test_pause_and_reject(self):
        m = FakeMission()
        m.control("start")
        self.assertIsNotNone(m.step(0.1, moving=True))
        self.assertEqual(m.control("pause"), (True, None))
        self.assertIsNone(m.step(0.1, moving=True))   # 멈추면 속도 명령을 보내지 않는다
        self.assertEqual(FakeMission(reject_start=True).control("start"), (False, "pose_not_ready"))

    def test_no_move_unless_auto(self):
        m = FakeMission()
        m.control("start")
        for _ in range(20):
            m.step(0.1, moving=False)
        self.assertEqual((m.x, m.y), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
