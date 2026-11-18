"""게이트웨이 AI 연결기 시험: 가짜 게이트웨이(HTTP)와 가짜 모델로 주고받기만 본다(모델·GPU 없이).

    python -B -m unittest modules.perception.inference.tests.test_gateway_ai_client -v
"""

import json
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace

import cv2
import numpy as np

from modules.perception.inference.detection_event import validate_person_detection_event
from modules.perception.inference.gateway_ai_client import GatewayAIClient, people_from_result


class Values:
    def __init__(self, values):
        self.values = values

    def cpu(self):
        return self

    def tolist(self):
        return self.values


def result(boxes, polygons, confs=None):
    return SimpleNamespace(boxes=SimpleNamespace(xyxy=Values(boxes), conf=Values(confs or [.9] * len(boxes))),
                           masks=SimpleNamespace(xy=[np.array(p, np.float32) for p in polygons]))


class FakeYolo:
    def __init__(self, res):
        self.res, self.calls = res, 0

    def predict(self, **kwargs):
        self.calls += 1
        return [self.res]


class FakeProcessor:
    """DualSegmentationProcessor처럼 process() 안에서 yolo.predict를 한 번 부르고 그린 그림을 돌려준다."""

    def __init__(self, res):
        self.yolo = FakeYolo(res)
        self.device = "cpu"

    def process(self, frame):
        self.yolo.predict(source=frame, classes=[0])
        out = frame.copy()
        out[:8] = (0, 255, 0)
        return out


class FakeGateway(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        s = self.server
        if s.snapshot_status != 200:
            self.send_response(s.snapshot_status)
            self.end_headers()
            self.wfile.write(b'{"detail":{"reason":"video_off"}}')
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("X-Input-Id", "run-1-video-1")
        self.send_header("X-Frame-Id", s.frame_id)
        self.send_header("X-Frame-Time", "2026-11-03T06:00:00.000+00:00")
        self.end_headers()
        self.wfile.write(s.jpeg)

    def _body(self):
        return self.rfile.read(int(self.headers["Content-Length"]))

    def do_PUT(self):
        self.server.puts.append((self.path, dict(self.headers), self._body()))
        self._ok({"ok": True})

    def do_POST(self):
        self.server.posts.append((self.path, json.loads(self._body())))
        self._ok({"ok": True, "finding_id": "finding-1"})

    def _ok(self, body):
        data = json.dumps(body).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


class GatewayAIClientTest(unittest.TestCase):
    def setUp(self):
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), FakeGateway)
        frame = np.full((72, 96, 3), 90, np.uint8)
        self.server.jpeg = cv2.imencode(".jpg", frame)[1].tobytes()
        self.server.frame_id, self.server.snapshot_status = "run-1-video-1-f000001", 200
        self.server.puts, self.server.posts = [], []
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.base = f"http://127.0.0.1:{self.server.server_address[1]}"
        self.proc = FakeProcessor(result([[10, 8, 40, 60]], [[[10, 8], [40, 8], [40, 60], [10, 60]]]))

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def client(self, **kw):
        return GatewayAIClient(self.base, self.proc, {"name": "fake", "version": None, "weights_sha256": None}, **kw)

    def test_sends_overlay_and_event_for_gateway_frame(self):
        out = self.client().step()
        self.assertEqual((out["status"], out["people"]), ("sent", 1))
        self.assertEqual(self.proc.yolo.model.calls, 1)   # 사람 결과는 합성할 때 돌린 YOLO 결과를 다시 쓴다
        (path, headers, jpeg), = self.server.puts
        self.assertEqual(path, "/api/perception/overlay.jpg")
        self.assertEqual((headers["X-Input-Id"], headers["X-Frame-Id"]), ("run-1-video-1", "run-1-video-1-f000001"))
        drawn = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertGreater(int(drawn[2, 50, 1]), 200)      # 합성된 그림(초록 띠)을 보냈다
        (path, event), = self.server.posts
        self.assertEqual(path, "/api/detections")
        validate_person_detection_event(event)
        self.assertEqual((event["input_id"], event["frame_id"]), ("run-1-video-1", "run-1-video-1-f000001"))
        self.assertEqual(event["detected_at_utc"], "2026-11-03T06:00:00.000Z")
        self.assertEqual(event["detections"][0]["bbox_xyxy_px"], [10.0, 8.0, 40.0, 60.0])

    def test_same_frame_is_not_processed_twice(self):
        c = self.client()
        c.step()
        self.assertEqual(c.step()["status"], "same_frame")
        self.assertEqual((len(self.server.posts), self.proc.yolo.model.calls), (1, 1))

    def test_no_person_still_reports_and_no_overlay_option(self):
        self.proc = FakeProcessor(SimpleNamespace(boxes=None, masks=None))
        out = self.client(overlay=False).step()
        self.assertEqual((out["status"], out["people"], self.server.puts), ("sent", 0, []))
        self.assertEqual(self.server.posts[0][1]["detections"], [])   # 사람이 없어도 탐지가 돈다는 근거로 보낸다

    def test_waits_when_video_is_off(self):
        self.server.snapshot_status = 503
        out = self.client().step()
        self.assertEqual((out["status"], out["code"]), ("no_video", 503))
        self.assertEqual((self.server.puts, self.server.posts), ([], []))

    def test_people_are_clipped_to_image(self):
        res = result([[-5, 2, 120, 70]], [[[-5, 2], [120, 2], [120, 70], [-5, 70]]], [.5])
        (p,) = people_from_result(res, 96, 72)
        self.assertEqual(p["xyxy"], [0.0, 2.0, 96.0, 70.0])
        self.assertTrue(all(0 <= x <= 96 and 0 <= y <= 72 for poly in p["polygons"] for x, y in poly))


if __name__ == "__main__":
    unittest.main()
