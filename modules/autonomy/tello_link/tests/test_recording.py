"""기록 재생·목업 데이터용 기록 보강을 시험한다: 영상 원본 녹화, 3D 지도 버전 저장, 자율 위치·경로·사람 전부 저장 (표준 라이브러리만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_recording.py" -v
"""

import gzip
import json
import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from autonomy_link import AutonomyLink  # noqa: E402
from map_inbox import MapInbox  # noqa: E402
from test_gateway import GatewayCase, StubVideo, wait_until  # noqa: E402
from test_map_inbox import Clock, snapshot  # noqa: E402
from video_recorder import VideoRecorder  # noqa: E402


def lines(path):
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f]


class VideoRecorderTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def test_bytes_and_index_as_received(self):
        rec = VideoRecorder(self.tmp.name)
        sink = rec.start_part("run-video1", "operator")
        packets = [bytes([i]) * (100 + i) for i in range(5)]
        for i, p in enumerate(packets):
            self.assertTrue(sink(p, 10.0 + i * 0.01))
        sink2 = rec.start_part("run-video2", "takeoff")      # 새 조각을 열면 앞 조각은 닫힌다
        self.assertFalse(sink(b"late", 11.0))                # 닫힌 조각에는 더 안 쓴다
        sink2(b"\x00\x00\x00\x01next", 12.0)
        rec.close()
        d = os.path.join(self.tmp.name, "video")
        with open(os.path.join(d, "part_0000.h264"), "rb") as f:
            self.assertEqual(f.read(), b"".join(packets))   # 받은 그대로 이어 붙임
        index = lines(os.path.join(d, "part_0000.jsonl"))
        self.assertEqual([r["offset"] for r in index], [0, 100, 201, 303, 406])
        self.assertEqual(index[2]["timestamp_s"], 10.02)
        self.assertIn("wall", index[0])
        parts = lines(os.path.join(d, "parts.jsonl"))
        ends = [r for r in parts if r["event"] == "end"]
        self.assertEqual([(r["id"], r["status"], r["packets"]) for r in ends], [(0, "captured", 5), (1, "captured", 1)])
        self.assertEqual([r["input_id"] for r in parts if r["event"] == "start"], ["run-video1", "run-video2"])

    def test_overflow_keeps_prefix_and_marks_partial(self):
        rec = VideoRecorder(self.tmp.name, queue_bytes=10)
        sink = rec.start_part("v", "operator")
        self.assertFalse(sink(b"x" * 20, 1.0))              # 대기 상한을 넘으면 받지 않는다(수신 스레드를 막지 않음)
        rec.close()
        end = [r for r in lines(os.path.join(self.tmp.name, "video", "parts.jsonl")) if r["event"] == "end"][0]
        self.assertEqual((end["status"], end["reason_partial"]), ("partial", "queue_overflow"))


class GatewayRecordTest(GatewayCase):
    def make_video(self):
        self.video = StubVideo()
        self.video.packet_sink = None
        return self.video

    def test_video_on_off_makes_parts(self):
        self.assertTrue(self.cmd("video_on")["ok"])
        self.assertIsNotNone(self.video.packet_sink)
        self.video.packet_sink(b"abc", time.monotonic())
        self.assertTrue(self.cmd("video_off")["ok"])
        self.assertIsNone(self.video.packet_sink)            # 끄면 떼어 낸다
        self.assertTrue(self.cmd("video_on")["ok"])
        self.video.packet_sink(b"defg", time.monotonic())
        self.assertTrue(self.cmd("video_off")["ok"])
        d = os.path.join(self.link.log.dir, "video")
        self.assertTrue(wait_until(lambda: os.path.exists(os.path.join(d, "part_0001.h264"))
                                   and os.path.getsize(os.path.join(d, "part_0001.h264")) == 4))
        with open(os.path.join(d, "part_0000.h264"), "rb") as f:
            self.assertEqual(f.read(), b"abc")
        starts = [r for r in lines(os.path.join(d, "parts.jsonl")) if r["event"] == "start"]
        self.assertEqual([r["input_id"] for r in starts], [f"{self.link.run_id}-video1", f"{self.link.run_id}-video2"])


class MapSnapshotTest(unittest.TestCase):
    def test_every_five_seconds_and_on_kind_change(self):
        with tempfile.TemporaryDirectory() as tmp:
            clock = Clock()
            inbox = MapInbox(run_dir=tmp, clock=clock)
            for rev, dt in ((1, 0), (2, 1), (3, 1), (4, 4)):   # 0초, 1초, 2초, 6초
                clock.t += dt
                self.assertEqual(inbox.receive(snapshot(revision=rev))[0], 201)
            final = snapshot(revision=4)
            final["source"]["schema_version"] = "phone-sfm-result/1.2"   # 최종 지도는 간격과 상관없이 남긴다
            clock.t += 0.5
            inbox.receive(final)
            saved = [r["saved"] for r in lines(os.path.join(tmp, "map_updates.jsonl"))]
            self.assertEqual([s is not None for s in saved], [True, False, False, True, True])
            with gzip.open(os.path.join(tmp, saved[-1]), "rt", encoding="utf-8") as f:
                self.assertEqual(json.load(f)["source"]["schema_version"], "phone-sfm-result/1.2")


class AutonomyRecordTest(unittest.TestCase):
    def test_every_pose_and_path_people_history(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = AutonomyLink(run_dir=tmp)
            a.open("au-1")
            m = lambda kind, **kw: {"schema_version": "0.1.0", "type": kind, **kw}   # noqa: E731
            for i in range(5):
                a.receive(m("pose", x_m=i * 0.1, y_m=0, yaw_rad=0, frame="map"))
            a.receive(m("path", points=[[0, 0, 1], [1, 0, 1]], frame="map"))
            person = {"x": 1, "y": 1, "conf": 0.9, "hits": 2}
            a.receive(m("people", frame="map", people=[person], unconfirmed=[]))
            a.receive(m("people", frame="map", people=[person], unconfirmed=[]))   # 같으면 다시 남기지 않는다
            d = os.path.join(tmp, "autonomy")
            self.assertEqual(len(lines(os.path.join(d, "pose.jsonl"))), 5)
            self.assertEqual(len(lines(os.path.join(d, "path.jsonl"))), 1)
            self.assertEqual(len(lines(os.path.join(d, "people.jsonl"))), 1)


if __name__ == "__main__":
    unittest.main()
