"""탐지 결과 받기 시험: 검증·중복·저장·발견 묶기·사진 (표준 라이브러리만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_detection_inbox.py" -v
"""

import json
import os
import sys
import tempfile
import unittest
import uuid

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from detection_inbox import DetectionInbox  # noqa: E402
from frame_store import FrameStore  # noqa: E402

PERSON = {"confidence": 0.8, "xyxy": [100, 100, 200, 400], "polygons": [[[100, 100], [200, 100], [150, 400]]]}


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class DetectionInboxTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.frames = FrameStore("run1", clock=self.clock)
        self.input_id = self.frames.new_session()
        self.sent = []
        self.inbox = DetectionInbox(self.frames, run_dir=self.tmp.name, emit=self.sent.append, clock=self.clock)

    def tearDown(self):
        self.tmp.cleanup()

    def frame(self, jpeg=b"\xff\xd8jpeg"):
        return self.frames.add(jpeg, self.clock.t, 1_800_000_000 + self.clock.t, (960, 720), {"mode": "manual"})

    def event(self, frame=None, people=(), frame_id=None, width=960):
        from modules.perception.inference.detection_event import build_person_detection_event
        return build_person_detection_event(
            input_id=frame["input_id"] if frame else "other-camera", frame_id=frame_id or (frame["frame_id"] if frame else "f-1"),
            detected_at_utc="2027-01-15T08:00:00Z", image_width=width, image_height=720,
            model={"name": "m", "version": "1", "weights_sha256": None}, detections=list(people))

    def lines(self):
        with open(os.path.join(self.tmp.name, "detections.jsonl"), encoding="utf-8") as f:
            return [json.loads(line) for line in f]

    def test_person_on_gateway_frame(self):
        f = self.frame()
        self.clock.t += 0.12
        status, out = self.inbox.receive(self.event(f, [PERSON]))
        self.assertEqual((status, out["frame"], out["finding_id"], out["photo"]), (201, "gateway", "finding-0001", True))
        rec = self.lines()[0]
        self.assertEqual((rec["latency_ms"], rec["drone"]["mode"]), (120, "manual"))
        self.assertTrue(os.path.isfile(self.inbox.photo_path(out["event_id"])))
        msg = self.sent[-1]
        self.assertEqual((msg["type"], msg["finding"]["finding_id"], msg["latency_ms"]), ("detection", "finding-0001", 120))
        self.assertEqual(msg["photo_url"], f"/api/detections/{out['event_id']}.jpg")
        self.assertEqual(msg["finding"]["best"]["photo_url"], msg["photo_url"])

    def test_duplicate(self):
        e = self.event(self.frame(), [PERSON])
        self.assertEqual(self.inbox.receive(e)[0], 201)
        status, out = self.inbox.receive(e)
        self.assertEqual((status, out["duplicate"]), (200, True))
        self.assertEqual((len(self.lines()), self.inbox.events), (1, 1))

    def test_invalid(self):
        bad = self.event(self.frame(), [PERSON])
        bad["detections"][0]["bbox_xyxy_px"] = [100, 100, 2000, 400]
        self.assertEqual(self.inbox.receive(bad)[1]["detail"], "bbox_out_of_bounds")
        self.assertEqual(self.inbox.receive({"schema_version": "x"})[0], 400)
        self.assertEqual(self.inbox.receive([1, 2])[0], 400)
        odd = self.event(self.frame(), [])
        odd["event_id"] = "{" + odd["event_id"].upper() + "}"   # uuid로는 읽히지만 파일 이름에 쓰지 않는 표기
        self.assertEqual(self.inbox.receive(odd)[1]["detail"], "invalid_event_id")
        self.assertEqual(self.inbox.status()["rejected"], 4)

    def test_image_size_mismatch(self):
        status, out = self.inbox.receive(self.event(self.frame(), [], width=640))
        self.assertEqual((status, out["reason"]), (400, "image_size_mismatch"))

    def test_empty_event_logged_not_sent(self):
        status, out = self.inbox.receive(self.event(self.frame(), []))
        self.assertEqual((status, out["finding_id"], out["photo"]), (201, None, False))
        self.assertEqual(self.sent, [])
        self.assertEqual(len(self.lines()), 1)
        st = self.inbox.status()
        self.assertEqual((st["alive"], st["events"], st["person_events"], st["findings"]), (True, 1, 0, 0))
        self.clock.t += 3
        self.assertFalse(self.inbox.status()["alive"])

    def test_findings_grouped_by_gap(self):
        ids = []
        for dt in (0, 1.0, 2.5, 6.0):   # 2.5 → 6.0은 3초 넘게 안 보임
            self.clock.t = 100 + dt
            ids.append(self.inbox.receive(self.event(self.frame(), [PERSON]))[1]["finding_id"])
        self.assertEqual(ids, ["finding-0001"] * 3 + ["finding-0002"])
        listed = self.inbox.snapshot()["findings"]
        self.assertEqual([f["finding_id"] for f in listed], ["finding-0002", "finding-0001"])   # 최신순
        self.assertEqual(listed[1]["events"], 3)

    def test_best_and_photo_rate(self):
        low, high = dict(PERSON, confidence=0.5), dict(PERSON, confidence=0.9)
        a = self.inbox.receive(self.event(self.frame(), [low]))[1]
        self.clock.t += 0.2
        b = self.inbox.receive(self.event(self.frame(), [low]))[1]     # 1초 안, 대표 그대로 → 사진 안 남김
        self.clock.t += 0.2
        c = self.inbox.receive(self.event(self.frame(), [high, low]))[1]  # 대표 바뀜 → 남김
        self.assertEqual((a["photo"], b["photo"], c["photo"]), (True, False, True))
        best = self.inbox.snapshot()["findings"][0]["best"]
        self.assertEqual((best["event_id"], best["confidence"], best["people"]), (c["event_id"], 0.9, 2))

    def test_other_camera_photo_upload(self):
        status, out = self.inbox.receive(self.event(None, [PERSON]))
        self.assertEqual((status, out["frame"], out["photo"]), (201, "unknown", False))
        self.assertEqual(out["upload"], f"/api/detections/{out['event_id']}.jpg")
        self.assertIsNone(self.inbox.snapshot()["findings"][0]["best"]["photo_url"])
        self.assertEqual(self.inbox.put_photo(out["event_id"], b"not a jpeg")[0], 400)
        self.assertEqual(self.inbox.put_photo(str(uuid.uuid4()), b"\xff\xd8x")[0], 404)
        self.assertEqual(self.inbox.put_photo(out["event_id"], b"\xff\xd8x")[0], 201)
        self.assertEqual(self.inbox.put_photo(out["event_id"], b"\xff\xd8y")[1]["duplicate"], True)
        self.assertIsNotNone(self.inbox.snapshot()["findings"][0]["best"]["photo_url"])
        with open(self.inbox.photo_path(out["event_id"]), "rb") as f:
            self.assertEqual(f.read(), b"\xff\xd8x")

    def test_old_session_frame_is_unknown(self):
        f = self.frame()
        self.frames.new_session()   # 영상을 다시 켬
        self.assertEqual(self.inbox.receive(self.event(f, [PERSON]))[1]["frame"], "unknown")


if __name__ == "__main__":
    unittest.main()
