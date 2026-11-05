import contextlib
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

import cv2
from cv_bridge import CvBridge
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Range

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from capture_http import capture, valid_sample
from inspect_bag import inspect, RGB, TOF
from summarize_output import summarize


class InputTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)

    def bag(self, mode="valid"):
        path = self.root / (mode + ".bag")
        bridge = CvBridge()
        with rosbag.Bag(str(path), "w") as bag:
            for index in range(20):
                image = bridge.cv2_to_imgmsg(np.zeros((48, 64, 3), dtype=np.uint8), "bgr8")
                image.header.stamp = rospy.Time.from_sec(100 + index * 0.1)
                bag.write(RGB, image, image.header.stamp)
                if mode != "missing_tof":
                    value = Range(min_range=0.1, max_range=6, range=float("nan") if mode == "invalid_tof" else 1.0)
                    value.header.stamp = rospy.Time.from_sec(100 + index * 0.1 + (10 if mode == "clock_mismatch" else 0))
                    bag.write(TOF, value, value.header.stamp)
        return path

    def test_valid_input_passes(self):
        result = inspect(self.bag())
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["rgb_with_bottom_tof_ratio"], 1)

    def test_missing_height_is_reported(self):
        result = inspect(self.bag("missing_tof"))
        self.assertFalse(result["ok"])
        self.assertEqual(result["topics"][TOF]["valid_messages"], 0)

    def test_different_clocks_are_rejected(self):
        result = inspect(self.bag("clock_mismatch"))
        self.assertFalse(result["ok"])
        self.assertEqual(result["rgb_with_bottom_tof_ratio"], 0)

    def test_invalid_tof_is_not_treated_as_clear_space(self):
        result = inspect(self.bag("invalid_tof"))
        self.assertFalse(result["ok"])
        self.assertEqual(result["topics"][TOF]["invalid_messages"], 20)

    def test_stale_state_is_not_recorded(self):
        status = dict(state=dict(valid=True, age_s=2), video=dict(frame_age_s=0))
        with self.assertRaisesRegex(ValueError, "state_stale"):
            valid_sample(status)

    def test_empty_output_is_not_claimed_as_pipeline_success(self):
        path = self.root / "empty.bag"
        with rosbag.Bag(str(path), "w"):
            pass
        result = summarize(path)
        self.assertFalse(result["all_stages_observed"])
        self.assertTrue(all(v["messages"] == 0 for v in result["stages"].values()))

    def test_light_recording_marks_depth_unrecorded_but_still_needs_poses(self):
        path = self.root / "empty_light.bag"
        with rosbag.Bag(str(path), "w"):
            pass
        result = summarize(path, record_mode="light")
        self.assertEqual(result["stages"]["metric_depth"], dict(topic="/sync/depth_metric", messages=None, recorded=False))
        self.assertFalse(result["all_stages_observed"])

    def fake_server(self, paths):
        class Handler(BaseHTTPRequestHandler):
            index = 0
            def log_message(self, *args):
                pass
            def do_GET(self):
                paths.append(self.path)
                if self.path == "/api/status":
                    Handler.index += 1
                    now = datetime.now(timezone.utc).isoformat()
                    payload = json.dumps(dict(mode="fake", now=now,
                        state=dict(valid=True, age_s=0, received_at=now, raw=dict(tof=120)),
                        video=dict(frame_age_s=0))).encode()
                elif self.path == "/snapshot.jpg":
                    frame = np.full((48, 64, 3), Handler.index % 255, dtype=np.uint8)
                    payload = cv2.imencode(".jpg", frame)[1].tobytes()
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.end_headers()
                self.wfile.write(payload)
        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()

        def close():
            server.shutdown()
            server.server_close()
            worker.join(timeout=2)
        return "http://127.0.0.1:%d" % server.server_port, close

    def test_http_capture_only_reads_and_converts_centimeters(self):
        paths = []
        url, close = self.fake_server(paths)
        try:
            output = self.root / "capture"
            with contextlib.redirect_stdout(io.StringIO()):
                code = capture(url, 2, 10, output)
            self.assertEqual(code, 0)
            self.assertEqual(set(paths), {"/api/status", "/snapshot.jpg"})
            with rosbag.Bag(str(output / "input.bag")) as bag:
                heights = [msg.range for _, msg, _ in bag.read_messages(topics=[TOF])]
            self.assertGreaterEqual(len(heights), 10)
            self.assertAlmostEqual(heights[0], 1.2, places=5)
            manifest = json.loads((output / "capture.json").read_text())
            self.assertEqual(manifest["source_modes"], ["fake"])
            self.assertTrue(manifest["read_only"])
            self.assertEqual(manifest["stop_reason"], "time_limit")
        finally:
            close()

    def test_stop_request_ends_early_and_still_saves(self):
        url, close = self.fake_server([])
        try:
            output = self.root / "stopped"
            stop = threading.Event()
            threading.Timer(2.0, stop.set).start()
            started = time.monotonic()
            with contextlib.redirect_stdout(io.StringIO()):
                code = capture(url, 60, 10, output, stop)
            self.assertLess(time.monotonic() - started, 10)
            self.assertEqual(code, 0)
            manifest = json.loads((output / "capture.json").read_text())
            self.assertEqual(manifest["stop_reason"], "stop_requested")
            self.assertTrue(manifest["input_quality"]["ok"])
            with rosbag.Bag(str(output / "input.bag")) as bag:
                self.assertGreaterEqual(bag.get_message_count(topic_filters=[TOF]), 10)
        finally:
            close()


if __name__ == "__main__":
    unittest.main()
