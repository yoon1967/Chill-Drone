"""3D 지도 받기·지도 연결기 변환 시험 (표준 라이브러리만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_map_inbox.py" -v
"""

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gateway_map_bridge import SCHEMA, camera_up, compact_scene  # noqa: E402
from map_inbox import MapInbox  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def snapshot(revision=1, n=3, session="s1", segment=0):
    return {"schema_version": SCHEMA, "kind": "snapshot", "upstream": "ok", "source_url": "http://127.0.0.1:8790",
            "mapping_state": "mapping",
            "source": {"kind": "live", "session_id": session, "segment_id": segment, "revision": revision},
            "scale": {"status": "arbitrary", "unit": "arbitrary"},
            "points": [[i * 0.1, 0.0, 1.0, 10, 20, 30] for i in range(n)], "points_total": n,
            "cameras": [[0, 0, 0], [0.1, 0, 0]], "up": [0, -1, 0]}


class MapInboxTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.sent = []
        self.inbox = MapInbox(run_dir=self.tmp.name, emit=self.sent.append, clock=self.clock)

    def tearDown(self):
        self.tmp.cleanup()

    def test_snapshot_stored_sent_and_logged(self):
        status, out = self.inbox.receive(snapshot())
        self.assertEqual((status, out["revision"]), (201, 1))
        self.assertEqual(len(self.inbox.latest()["points"]), 3)
        self.assertEqual((self.sent[-1]["type"], self.sent[-1]["revision"], self.sent[-1]["points"]), ("map", 1, 3))
        with open(os.path.join(self.tmp.name, "map", "latest.json"), encoding="utf-8") as f:
            self.assertEqual(json.load(f)["source"]["revision"], 1)
        with open(os.path.join(self.tmp.name, "map_updates.jsonl"), encoding="utf-8") as f:
            self.assertEqual(json.loads(f.readline())["points"], 3)
        st = self.inbox.status()
        self.assertEqual((st["alive"], st["revision"], st["points"], st["unit"], st["upstream"]), (True, 1, 3, "arbitrary", "ok"))

    def test_same_revision_not_stored_twice(self):
        self.inbox.receive(snapshot())
        status, out = self.inbox.receive(snapshot())
        self.assertEqual((status, out["duplicate"]), (200, True))
        self.assertEqual((self.inbox.updates, len(self.sent)), (1, 1))
        self.assertEqual(self.inbox.receive(snapshot(session="s2"))[0], 201)   # 세션이 바뀌면 같은 버전 번호라도 새 지도

    def test_status_keeps_latest_and_alive(self):
        self.inbox.receive(snapshot())
        self.clock.t += 4
        self.inbox.receive({"schema_version": SCHEMA, "kind": "status", "upstream": "unreachable", "source_url": "x"})
        st = self.inbox.status()
        self.assertEqual((st["alive"], st["upstream"], st["points"]), (True, "unreachable", 3))   # 지도는 마지막 것 유지
        self.clock.t += 6
        self.assertFalse(self.inbox.status()["alive"])

    def test_invalid(self):
        bad = snapshot()
        bad["points"][0] = [0, 0, float("nan"), 1, 2, 3]
        self.assertEqual(self.inbox.receive(bad)[1]["detail"], "invalid_point")
        bad = snapshot()
        bad["points"][0][3] = 300
        self.assertEqual(self.inbox.receive(bad)[1]["detail"], "invalid_point")
        self.assertEqual(self.inbox.receive({**snapshot(), "schema_version": "x"})[0], 400)
        self.assertEqual(self.inbox.receive({**snapshot(), "kind": "x"})[0], 400)
        self.assertEqual(self.inbox.receive([1])[0], 400)
        self.assertEqual((self.inbox.status()["rejected"], self.inbox.latest()), (5, None))

    def test_no_run_dir(self):
        inbox = MapInbox(clock=self.clock)
        self.assertEqual(inbox.receive(snapshot())[0], 201)


class BridgeTest(unittest.TestCase):
    # 지도 서버 scene 한 조각(phone-sfm-result/1.2): 점 행 [id,x,y,z,r,g,b,error_px,track_length]
    RUN = {"schema_version": "phone-sfm-result/1.2", "input_run_id": "run-a", "segment_id": 0, "revision": 7,
           "pose_source": "orbslam3", "scale": {"status": "arbitrary", "unit": "arbitrary"},
           "points": [[1, 0.0, 0.0, 1.0, 255, 0, 0, 0.5, 9], [2, 1.0, 0.0, 1.0, 0, 255, 0, 3.0, 2],
                      [3, 2.0, 0.0, 1.0, 0, 0, 255, 0.5, 2], [4, None, 0, 0, 0, 0, 0, 0.1, 9]],
           "cameras": [{"frame_id": "f1", "position": [0, 0, 0],
                        "world_from_camera_rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]},
                       {"frame_id": "f2", "position": [0.1, 0, 0.2],
                        "world_from_camera_rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}]}
    STATE = {"session_id": "sess", "segment_id": 0, "revision": 7, "map_available": True, "mapping_state": "mapping"}

    def test_compact_keeps_low_error_points(self):
        msg = compact_scene(self.RUN, self.STATE, "http://m", "live", max_points=2)
        self.assertEqual(msg["points_total"], 3)   # 좌표가 잘못된 점은 뺀다
        self.assertEqual(msg["points"], [[0.0, 0.0, 1.0, 255, 0, 0], [2.0, 0.0, 1.0, 0, 0, 255]])   # 오차 0.5 두 개
        self.assertEqual(msg["cameras"], [[0, 0, 0], [0.1, 0, 0.2]])
        self.assertEqual((msg["source"]["revision"], msg["source"]["session_id"], msg["scale"]["unit"]), (7, "sess", "arbitrary"))
        self.assertEqual(msg["up"], [0.0, -1.0, 0.0])   # 카메라 y는 아래 → 위쪽은 -y
        inbox = MapInbox()
        self.assertEqual(inbox.receive(msg)[0], 201)       # 받는 쪽 형식 검사도 통과

    def test_camera_up_none_without_rotation(self):
        self.assertIsNone(camera_up([{"position": [0, 0, 0]}]))


if __name__ == "__main__":
    unittest.main()
