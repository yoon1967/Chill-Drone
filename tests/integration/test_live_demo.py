"""Recorded input calibration and HTTP metadata; no Docker or real captures required."""
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import threading
import unittest
from urllib.request import urlopen

import cv2
import numpy as np

from apps.console.serve import ViewerHandler, ViewerServer, prepare_demo_camera
from replay_live_slam import replay


class RecordedDemoTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "synthetic.mp4"
        self.camera = self.root / "camera.json"
        self.write_video(1280, 720)
        self.camera.write_text(json.dumps(dict(width=1280, height=720, fx=1000., fy=1000.,
            cx=640., cy=360., distortion=[0.] * 4)), encoding="utf-8")

    def write_video(self, width, height):
        writer = cv2.VideoWriter(str(self.source), cv2.VideoWriter_fourcc(*"mp4v"), 30., (width, height))
        self.assertTrue(writer.isOpened())
        writer.write(np.zeros((height, width, 3), dtype=np.uint8))
        writer.release()

    def test_explicit_camera_scales_to_browser_frame(self):
        camera = prepare_demo_camera(self.source, camera=self.camera)
        self.assertEqual((camera["width"], camera["height"]), (960, 540))
        self.assertEqual(camera["fx"], 750.)
        self.assertEqual(camera["cx"], 480.)

    def test_saved_result_reuses_intrinsics_only(self):
        prior = self.root / "prior"
        prior.mkdir()
        (prior / "scene.json").write_text(json.dumps({"cameras": [dict(model="SIMPLE_RADIAL",
            width=1280, height=720, params=[1000., 640., 360., .01])]}), encoding="utf-8")
        camera = prepare_demo_camera(self.source, calibration_result=prior)
        self.assertEqual(camera["calibration_status"], "prior_self_estimated")
        self.assertEqual(camera["distortion"], [.01, 0., 0., 0.])

    def test_existing_session_camera_remains_supported(self):
        self.assertEqual(prepare_demo_camera(self.source, session=self.root)["width"], 960)

    def test_missing_calibration_fails_before_camera_start(self):
        with self.assertRaisesRegex(ValueError, "Demo requires"):
            prepare_demo_camera(self.source)

    def test_different_crop_is_rejected(self):
        camera = json.loads(self.camera.read_text(encoding="utf-8"))
        camera.update(width=720, height=1280)
        self.camera.write_text(json.dumps(camera), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "orientation_or_crop"):
            prepare_demo_camera(self.source, camera=self.camera)

    def test_portrait_dimensions_match_browser_cap(self):
        self.write_video(720, 1280)
        self.camera.write_text(json.dumps(dict(width=720, height=1280, fx=1000., fy=1000.,
            cx=360., cy=640., distortion=[0.] * 4)), encoding="utf-8")
        camera = prepare_demo_camera(self.source, camera=self.camera)
        self.assertEqual((camera["width"], camera["height"]), (540, 960))
        self.assertEqual(camera["cy"], 480.)

    def test_http_exposes_validated_demo_camera_without_online_seed(self):
        camera = prepare_demo_camera(self.source, camera=self.camera)
        server = ViewerServer(("127.0.0.1", 0), partial(ViewerHandler,
            demo=self.source, demo_camera=camera, live=type("Live", (), {"current": None})()))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with urlopen(f"http://127.0.0.1:{server.server_port}/api/live") as response:
                value = json.load(response)
            self.assertTrue(value["enabled"])
            self.assertTrue(value["demo_available"])
            self.assertEqual(value["demo_camera"], camera)
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_replay_rejects_maps_published_only_while_stopping(self):
        camera = prepare_demo_camera(self.source, camera=self.camera)

        class StoppingOnlyMap(BaseHTTPRequestHandler):
            stopping = False
            revision = 0

            def respond(self, value):
                body = json.dumps(value).encode()
                self.send_response(200)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == "/api/live":
                    self.respond(dict(enabled=True, demo_available=True, demo_camera=camera))
                elif self.path.startswith("/api/live/state?"):
                    if self.stopping:
                        type(self).revision += 1
                    self.respond(dict(session_id="synthetic", status="complete" if self.revision >= 3 else "receiving",
                        transport_ready=True, transport_active=self.revision < 3, transport_stopping=self.stopping,
                        revision=self.revision, tracked_frames=1))
                else:
                    self.respond(dict(revision=self.revision, points=[[1, 0, 0, 0]], cameras=[{}],
                        slam={"timestamp_s": 0.}))

            def do_POST(self):
                self.rfile.read(int(self.headers["Content-Length"]))
                if self.path == "/api/live/stop":
                    type(self).stopping = True
                self.respond(dict(session_id="synthetic"))

            def log_message(self, *args):
                pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), StoppingOnlyMap)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            report_path = self.root / "report.json"
            result = replay(SimpleNamespace(source=self.source, url=f"http://127.0.0.1:{server.server_port}", report=report_path))
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(result, 1)
            self.assertEqual(report["failure_stage"], "slam")
            self.assertGreaterEqual(len(report["map_updates"]), 2)
            self.assertFalse(any(item["during_stream"] for item in report["map_updates"]))
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == "__main__":
    unittest.main()
