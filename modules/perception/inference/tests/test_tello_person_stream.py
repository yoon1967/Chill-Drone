import time
import unittest
import json
from functools import partial
from http.server import ThreadingHTTPServer
import threading
from types import SimpleNamespace
import urllib.request

import cv2
import numpy as np

from modules.perception.inference.tello_person_stream import TelloPersonStream
from apps.console.serve import ViewerHandler


class Values:
    def __init__(self, values):
        self.values = values

    def cpu(self):
        return self

    def tolist(self):
        return self.values


class FakeResult:
    def __init__(self, frame):
        self.frame = frame
        self.boxes = SimpleNamespace(xyxy=Values([[10, 20, 80, 100]]), conf=Values([.91]), cls=Values([0]))
        self.masks = SimpleNamespace(xy=[np.array([[10, 20], [80, 20], [80, 100], [10, 100]])])


class FakeDetector:
    names = {0: "person"}
    task = "segment"

    def __init__(self, model):
        self.model = model

    def predict(self, source, **kwargs):
        return [FakeResult(source)]


class FakeLink:
    def start(self):
        return self

    def send(self, command, timeout=3):
        return SimpleNamespace(ok=True, reason=None, response="ok")

    def latest_state(self):
        return {"valid": True}

    def close(self):
        pass


class FakeVideo:
    def __init__(self):
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        self.jpeg = cv2.imencode(".jpg", frame)[1].tobytes()
        self.stamp = 1.0
        self.running = False

    def start(self):
        self.running = True

    def stop(self):
        self.running = False

    def jpeg_snapshot(self):
        return self.jpeg, self.stamp, 2.0

    def stats(self):
        return {"running": self.running, "fps": 24.0}


class TelloPersonStreamTests(unittest.TestCase):
    def test_defaults_to_person_segmentation_model(self):
        self.assertEqual(TelloPersonStream().model_name, "yolo26n-seg.pt")

    def test_detects_person_and_produces_annotated_jpeg(self):
        stream = TelloPersonStream("fake.pt", link_factory=FakeLink, video_factory=FakeVideo,
                                   detector_factory=FakeDetector)
        self.addCleanup(stream.stop)
        stream.start()
        deadline = time.monotonic() + 2
        while stream.stats()["processed_frames"] == 0 and time.monotonic() < deadline:
            time.sleep(.01)
        stats = stream.stats()
        frame, stamp = stream.latest()
        self.assertEqual(stats["person_count"], 1)
        self.assertEqual(stats["detections"][0]["xyxy"], [10.0, 20.0, 80.0, 100.0])
        self.assertEqual(stats["detections"][0]["polygons"][0][0], [10.0, 20.0])
        self.assertEqual(stats["task"], "segment")
        self.assertEqual(stamp, 1.0)
        self.assertTrue(frame.startswith(b"\xff\xd8"))

    def test_rejects_detection_model_without_segmentation_masks(self):
        class DetectOnly(FakeDetector):
            task = "detect"

        stream = TelloPersonStream("detect.pt", link_factory=FakeLink, video_factory=FakeVideo,
                                   detector_factory=DetectOnly)
        with self.assertRaisesRegex(RuntimeError, "model_task_is_not_segmentation"):
            stream.start()

    def test_console_exposes_tello_start_state_and_stop(self):
        class FakeWebTello:
            running = False

            def stats(self):
                return {"enabled": True, "running": self.running}

            def start(self):
                self.running = True
                return self.stats()

            def stop(self):
                self.running = False
                return self.stats()

        tello = FakeWebTello()
        handler = partial(ViewerHandler, tello=tello)
        with ThreadingHTTPServer(("127.0.0.1", 0), handler) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                root = f"http://127.0.0.1:{server.server_port}"
                with urllib.request.urlopen(root + "/api/tello") as response:
                    self.assertFalse(json.load(response)["running"])
                for path, expected in [("start", True), ("stop", False)]:
                    request = urllib.request.Request(root + f"/api/tello/{path}", data=b"{}", method="POST",
                        headers={"Content-Type": "application/json", "X-Mapping-Client": "1"})
                    with urllib.request.urlopen(request) as response:
                        self.assertEqual(json.load(response)["running"], expected)
            finally:
                server.shutdown()
                thread.join()

    def test_console_exposes_flight_controls_through_tello_owner(self):
        class FakeFlight:
            def __init__(self):
                self.confirmed = False
                self.rc = None

            def status(self):
                return {"flying": False, "can_takeoff": self.confirmed, "preflight": []}

            def confirm_operator(self, confirmed):
                self.confirmed = confirmed

            def takeoff(self, source="web"):
                return {"ok": True, "flying": True}

            def land(self, source="web"):
                return {"ok": True, "flying": False}

            def emergency(self, source="web"):
                return {"ok": True}

            def set_input(self, *values, source="keyboard"):
                self.rc = values

        class FakeWebTello:
            running = True
            flight = FakeFlight()

            def stats(self):
                return {"enabled": True, "running": True, "flight": self.flight.status()}

            def require_flight(self):
                return self.flight

        handler = partial(ViewerHandler, tello=FakeWebTello())
        with ThreadingHTTPServer(("127.0.0.1", 0), handler) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                root = f"http://127.0.0.1:{server.server_port}"
                def send(action, body):
                    request = urllib.request.Request(root + f"/api/tello/flight/{action}",
                        data=json.dumps(body).encode(), method="POST",
                        headers={"Content-Type": "application/json", "X-Mapping-Client": "1"})
                    with urllib.request.urlopen(request) as response:
                        return json.load(response)
                self.assertTrue(send("confirm", {"confirmed": True})["can_takeoff"])
                self.assertTrue(send("takeoff", {})["flying"])
                self.assertEqual(send("rc", {"rc": [1, 0, 0, -1]})["can_takeoff"], True)
                self.assertEqual(handler.keywords["tello"].flight.rc, (1.0, 0.0, 0.0, -1.0))
                self.assertFalse(send("land", {})["flying"])
            finally:
                server.shutdown()
                thread.join()


if __name__ == "__main__":
    unittest.main()
