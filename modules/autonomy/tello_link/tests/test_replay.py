"""기록 재생(replay.py)을 작은 가짜 회차 폴더로 시험한다. 영상은 libx264로 몇 장 만들어 녹화 형식으로 남긴다(PyAV 필요).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_replay.py" -v
"""

import argparse
import gzip
import json
import os
import shutil
import sys
import tempfile
import time
import unittest
import uuid

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from replay import ReplayClock, ReplayGateway  # noqa: E402
from tello_link import now_iso  # noqa: E402

T0 = 1_800_000_000.0   # 가짜 회차 시작(UTC epoch 초)


def write_jsonl(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r) + "\n")


def make_video_part(run_dir, start, n=20):
    """libx264로 n장을 만들어 녹화 형식(part_0000.h264 + 색인 + parts.jsonl)으로 남긴다. 30fps."""
    import av
    import numpy as np
    enc = av.CodecContext.create("libx264", "w")
    enc.width, enc.height, enc.pix_fmt, enc.framerate = 320, 240, "yuv420p", 30
    enc.options = {"tune": "zerolatency", "preset": "ultrafast", "g": "10"}
    vdir = os.path.join(run_dir, "video")
    os.makedirs(vdir, exist_ok=True)
    data, index = b"", []
    for i in range(n):
        frame = av.VideoFrame.from_ndarray(np.full((240, 320, 3), i * 10 % 255, np.uint8), format="rgb24")
        for packet in enc.encode(frame):
            b = bytes(packet)
            index.append({"offset": len(data), "size": len(b), "timestamp_s": 0.0, "wall": start + i / 30})
            data += b
    for packet in enc.encode(None):
        b = bytes(packet)
        index.append({"offset": len(data), "size": len(b), "timestamp_s": 0.0, "wall": start + n / 30})
        data += b
    with open(os.path.join(vdir, "part_0000.h264"), "wb") as f:
        f.write(data)
    write_jsonl(os.path.join(vdir, "part_0000.jsonl"), index)
    write_jsonl(os.path.join(vdir, "parts.jsonl"), [{"event": "start", "id": 0, "input_id": "fake-video1", "reason": "operator"},
                                                  {"event": "end", "id": 0, "status": "captured"}])


def make_run(root, video=True):
    run = os.path.join(root, "tello_20270115_080000_abcdef")
    os.makedirs(run)
    with open(os.path.join(run, "manifest.json"), "w") as f:
        json.dump({"run_id": os.path.basename(run)}, f)
    write_jsonl(os.path.join(run, "sensor.jsonl"), [
        {"received_at": now_iso(T0 + i * 0.1), "raw": {"bat": 90 - i // 10}, "valid": True,
         "si": {"battery_pct": 90 - i // 10, "temp_high_c": 60, "tof_m": 0.8 if 20 <= i < 80 else None,
                "tof_valid": 20 <= i < 80, "tof_reason": None if 20 <= i < 80 else "below_min_or_on_ground", "yaw_rad": 0.1}}
        for i in range(101)])
    ev = lambda dt, kind, **d: {"t": now_iso(T0 + dt), "type": kind, "detail": d}   # noqa: E731
    write_jsonl(os.path.join(run, "events.jsonl"), [
        ev(0.05, "gateway_mode_changed", **{"from": "disconnected", "to": "grounded", "by": "gateway", "reason": "state_ok"}),
        ev(0.1, "gateway_autonomy_connected", ground_origin=True),
        ev(2.0, "gateway_mode_changed", **{"from": "grounded", "to": "manual", "by": "operator", "reason": "takeoff"}),
        ev(3.0, "gateway_video", on=True, ok=True, reason="takeoff"),
        ev(7.0, "gateway_video", on=False, ok=True, reason="operator"),
        ev(8.0, "gateway_mode_changed", **{"from": "manual", "to": "grounded", "by": "gateway", "reason": "landed"})])
    eid = str(uuid.uuid4())
    person = {"detection_id": "d1", "class_name": "person", "confidence": 0.8, "bbox_xyxy_px": [1, 2, 3, 4], "polygons_px": []}
    det = lambda dt, e, conf, photo: {"type": "event", "received_at": now_iso(T0 + dt), "frame_time": now_iso(T0 + dt - 0.1),   # noqa: E731
                                      "finding_id": "finding-0001", "latency_ms": 80, "drone": None, "photo": photo,
                                      "event": {"event_id": e, "input_id": "fake-video1", "frame_id": "frame-000001",
                                                "image": {"width_px": 320, "height_px": 240}, "model": {"name": "m"},
                                                "detections": [{**person, "confidence": conf}]}}
    write_jsonl(os.path.join(run, "detections.jsonl"), [det(4.0, eid, 0.8, f"detections/{eid}.jpg"),
                                                         det(4.5, str(uuid.uuid4()), 0.6, None)])
    os.makedirs(os.path.join(run, "detections"))
    with open(os.path.join(run, "detections", f"{eid}.jpg"), "wb") as f:
        f.write(b"\xff\xd8jpg")
    os.makedirs(os.path.join(run, "map", "snapshots"))
    snap = {"source": {"revision": 7, "segment_id": 0, "schema_version": "x"}, "points": [[0, 0, 0, 1, 2, 3]], "cameras": [],
            "scale": {"unit": "arbitrary"}, "received_at": now_iso(T0 + 5)}
    with gzip.open(os.path.join(run, "map", "snapshots", "0000_rev7.json.gz"), "wt", encoding="utf-8") as f:
        json.dump(snap, f)
    write_jsonl(os.path.join(run, "map_updates.jsonl"), [{"received_at": now_iso(T0 + 5), "revision": 7, "segment_id": 0,
                                                          "points": 1, "unit": "arbitrary", "saved": "map/snapshots/0000_rev7.json.gz"}])
    a = os.path.join(run, "autonomy")
    write_jsonl(os.path.join(a, "events.jsonl"), [{"at": now_iso(T0 + 0.1), "kind": "connected"},
                                                  {"at": now_iso(T0 + 0.2), "kind": "hello", "name": "fake_autonomy", "version": "0.1"},
                                                  {"at": now_iso(T0 + 3), "kind": "mission_state", "state": "FOLLOW", "phase": "running", "progress": 0.2}])
    write_jsonl(os.path.join(a, "pose.jsonl"), [{"at": now_iso(T0 + 3 + i * 0.1), "x_m": i * 0.1, "y_m": 0.0, "z_m": 1.0, "yaw_rad": 0.0}
                                                for i in range(30)])
    write_jsonl(os.path.join(a, "people.jsonl"), [{"at": now_iso(T0 + 5), "people": [{"x_m": 1, "y_m": 1, "gone": False}],
                                                   "unconfirmed": [], "frame": "map", "revision": 1}])
    if video:
        make_video_part(run, T0 + 3.0)
    return run


class Mono:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class ReplayStateTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.run = make_run(self.tmp, video=False)
        self.mono = Mono()
        self.gw = ReplayGateway(self.run, mono=self.mono)
        self.sent = []
        self.gw.listeners.append(self.sent.append)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def at(self, pos):
        self.gw.clock.seek(pos)
        return self.gw.drone_state()

    def test_state_follows_record(self):
        st = self.at(2.5)
        self.assertEqual((st["source"], st["mode"], st["authority"], st["allowed"]), ("replay", "manual", "operator", []))
        self.assertEqual((st["battery_pct"], st["height_m"], st["video"]["on"]), (88, 0.8, False))
        self.assertEqual(st["replay"]["duration_s"], 10.0)
        st = self.at(9.0)
        self.assertEqual((st["mode"], st["height_m"]), ("grounded", None))
        self.assertEqual(st["autonomy"]["ground_origin"], True)

    def test_commands_rejected(self):
        r = self.gw.handle_message({"type": "cmd", "id": "c1", "name": "takeoff"})
        self.assertEqual((r["ok"], r["reason"]), (False, "replay_source"))
        self.assertIsNone(self.gw.handle_message({"type": "rc", "axes": [0, 1, 0, 0]}))
        self.assertFalse(self.gw.autonomy_open("au-1", print))
        self.assertEqual(self.gw.detections.receive({})[0], 409)

    def test_detections_map_autonomy_up_to_now(self):
        self.gw.clock.seek(4.2)
        snap = self.gw.detections.snapshot()
        self.assertEqual((len(snap["findings"]), snap["findings"][0]["events"]), (1, 1))
        self.assertTrue(snap["findings"][0]["best"]["photo_url"].endswith(".jpg"))
        self.assertIsNone(self.gw.map.latest())                     # 지도는 5초에 처음 온다
        pose = self.gw.autonomy.pose_state()
        self.assertEqual((round(pose["x_m"], 1), pose["valid"]), (1.2, True))
        self.gw.clock.seek(5.5)
        self.assertEqual(self.gw.map.latest()["source"]["revision"], 7)
        self.assertEqual(self.gw.drone_state()["autonomy"]["people"], {"confirmed": 1, "gone": 0, "unconfirmed": 0})
        self.assertEqual(self.gw.drone_state()["mission"]["state"], "FOLLOW")
        photo = self.gw.detections.photo_path(snap["findings"][0]["best"]["event_id"])
        self.assertTrue(photo and os.path.isfile(photo))
        self.assertIsNone(self.gw.detections.photo_path("../manifest"))   # 회차 폴더 밖 경로는 거절

    def test_tick_emits_in_order_and_restarts(self):
        self.gw.clock.seek(6.0)
        self.gw.tick()
        kinds = [m.get("kind") or m["type"] for m in self.sent]
        self.assertEqual(kinds, ["mode_changed", "autonomy_connected", "mode_changed", "video_changed", "detection", "detection", "map"])
        self.sent.clear()
        self.gw.clock.seek(1.0)                                     # 되감으면 처음부터 다시 센다
        self.gw.tick()
        self.assertEqual([m.get("kind") or m["type"] for m in self.sent], ["replay_restarted", "mode_changed", "autonomy_connected"])

    def test_clock_speed_loop(self):
        mono = Mono()
        c = ReplayClock(T0, T0 + 10, speed=2.0, loop=True, mono=mono)
        mono.t += 3
        self.assertEqual(c.pos(), 6.0)
        mono.t += 3
        self.assertEqual(c.pos(), 2.0)                              # 12초 → 한 바퀴 돌아 2초

    def test_server_builds_replay(self):
        import gateway_server
        args = argparse.Namespace(replay=self.run, replay_speed=1.0, replay_loop=False, replay_start=0.0, no_video=False, fake=False)
        gw, fake = gateway_server.build_gateway(args)
        self.assertEqual((type(gw).__name__, fake), ("ReplayGateway", None))


class ReplayVideoTest(unittest.TestCase):
    def setUp(self):
        try:
            import av  # noqa: F401
            import numpy  # noqa: F401
        except ImportError:
            self.skipTest("PyAV·numpy가 없는 환경")
        self.tmp = tempfile.mkdtemp()
        self.run = make_run(self.tmp, video=True)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_recorded_packets_decode_at_their_time(self):
        gw = ReplayGateway(self.run, start_s=3.0)       # 영상 조각(3초~)부터, 실제 시계로
        self.assertEqual(len(gw.rec.parts), 1)
        gw.start()
        try:
            deadline = time.monotonic() + 5
            while gw.video.frames < 10 and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertGreaterEqual(gw.video.frames, 10)
            self.assertEqual(gw.video.size, (320, 240))
            frame, reason = gw.video_frame()
            self.assertIsNone(reason)
            self.assertEqual(frame["input_id"], f"{gw.rec.run_id}-replay-video1")
            self.assertTrue(gw.drone_state()["video"]["available"])
        finally:
            gw.video.stop()
            gw.close()


if __name__ == "__main__":
    unittest.main()
