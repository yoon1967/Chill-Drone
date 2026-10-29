"""Saved/live HTTP isolation with synthetic maps and the real manager/transport checks."""
from concurrent.futures import ThreadPoolExecutor
from functools import partial
import hashlib
from http.client import HTTPConnection
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
import uuid

import cv2
import numpy as np

from apps.console.serve import ObservationCache, Timeline, ViewerHandler, ViewerServer
from modules.mapping.localization.orbslam3 import atomic_json
from modules.mapping.localization.orbslam3_live import LiveManager, LiveSession


def scene(run_id, x, unit="arbitrary", revision=None):
    value = {"schema_version": "phone-sfm-result/1.2", "input_run_id": run_id,
             "points": [[1, x, 0, 0, 100, 100, 100, 0., 3]],
             "cameras": [{"image_id": 1, "frame_id": "same.jpg", "timestamp_s": 0.}],
             "coordinate_system": {"native": "synthetic local world"},
             "scale": {"unit": unit, "status": "reference_scaled" if unit == "m" else "arbitrary"}}
    if revision is not None:
        value.update(revision=revision, segment_id=revision * 7, pose_source="orb_slam3",
                     slam={"timestamp_s": float(revision), "timestamp_basis": "snapshot_input_pts", "final": False})
    return value


def write_result(path, value, frame):
    path.mkdir(parents=True, exist_ok=True)
    atomic_json(path / "scene.json", value)
    atomic_json(path / "status.json", {"status": "partial" if value.get("revision") else "complete"})
    images = path / "images"
    images.mkdir(exist_ok=True)
    (images / "same.jpg").write_bytes(frame)
    atomic_json(path / "local_assets.json", {"images_root": str(images), "frames": ["same.jpg"]})
    (path / "points.ply").write_bytes(frame + b" ply")


class SyntheticSession(LiveSession):
    """Replace Docker execution only; use real offer validation and lifecycle stats."""
    def __init__(self, root, camera, image_id, *, input_kind="browser_camera", timestamp_basis="browser_media_time"):
        self.input_kind, self.timestamp_basis = input_kind, timestamp_basis
        self.id = "live_" + uuid.uuid4().hex[:16]
        self.path = root / self.id
        self.path.mkdir(parents=True)
        (self.path / "revisions").mkdir()
        self.camera = camera
        self.condition = threading.Condition()
        self.pending, self.closing, self.done, self.ready, self.error = None, False, False, True, None
        self.reason = "user_stop"
        self.received = self.dropped = self.submitted = 0
        self.last_sequence, self.last_pts, self.ack = -1, -1., -1.
        self.state = {"status": "receiving", "backend": "orb_slam3_monocular", "revision": 0, "map_available": False}
        atomic_json(self.path / "state.json", self.state)

    def publish(self, revision=1):
        directory = self.path / "revisions" / f"{revision:06d}"
        write_result(directory, scene(self.id, 10 * revision, revision=revision), self.id.encode())
        self.state.update(revision=revision, revision_path=f"revisions/{revision:06d}", map_available=True,
                          mapping_state="tracking", tracking_state=2)
        atomic_json(self.path / "state.json", self.state)
        (self.path / "preview.jpg").write_bytes(self.id.encode() + b" preview")

    def stop(self, reason="user_stop"):
        super().stop(reason)
        self.done, self.ready, self.pending = True, False, None
        self.state["status"] = "complete"
        atomic_json(self.path / "state.json", self.state)


class MapSourceTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.saved = self.root / "saved"
        write_result(self.saved, scene("saved-run", 1., "m"), b"saved image")
        self.manager = LiveManager(self.root / "live")
        self.config = dict(width=320, height=180, fps=30, fov=60)
        self.jpeg = cv2.imencode(".jpg", np.zeros((180, 320, 3), dtype=np.uint8))[1].tobytes()
        self.addCleanup(patch.stopall)
        patch("modules.mapping.localization.orbslam3_live.LiveSession", SyntheticSession).start()
        patch("modules.mapping.localization.orbslam3_live.subprocess.check_output", return_value="synthetic-image").start()

    def serve(self, **kwargs):
        parameters = dict(result=self.saved, live=self.manager, timeline=Timeline(), observations=ObservationCache())
        parameters.update(kwargs)
        server = ViewerServer(("127.0.0.1", 0), partial(ViewerHandler, **parameters))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

        def close():
            server.shutdown()
            server.server_close()
            thread.join()

        self.addCleanup(close)
        return server.server_port

    def request(self, port, path, body=None, headers=None):
        headers = {"X-Mapping-Client": "1", **(headers or {})}
        if isinstance(body, dict):
            body = json.dumps(body).encode()
            headers["Content-Type"] = "application/json"
        connection = HTTPConnection("127.0.0.1", port, timeout=5)
        try:
            connection.request("GET" if body is None else "POST", path, body, headers)
            response = connection.getresponse()
            payload = response.read()
            return response.status, json.loads(payload) if "application/json" in response.getheader("Content-Type", "") else payload
        finally:
            connection.close()

    def saved_hashes(self):
        return {str(p.relative_to(self.saved)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in self.saved.rglob("*") if p.is_file()}

    def test_idle_live_never_returns_saved_map(self):
        port = self.serve()
        self.assertEqual(self.request(port, "/api/saved/run")[1]["input_run_id"], "saved-run")
        code, state = self.request(port, "/api/live/state")
        self.assertEqual(code, 200)
        self.assertEqual((state["source"], state["session_status"], state["map_available"]), ("live", "idle", False))
        self.assertIsNone(state["session_id"])
        self.assertEqual(self.request(port, "/api/live/run")[0], 503)
        self.assertEqual(self.request(port, "/api/live/frame/same.jpg")[0], 404)
        self.assertEqual(self.request(port, "/api/live/timeline")[1]["entries"], [])

    def test_restart_preserves_saved_files_and_rejects_stale_session(self):
        port = self.serve()
        before = self.saved_hashes()
        saved_before = self.request(port, "/api/saved/run")[1]
        code, started = self.request(port, "/api/live/start", self.config)
        self.assertEqual(code, 201)
        first = self.manager.current
        self.assertEqual((started["source"], started["session_id"]), ("live", first.id))
        self.assertEqual(self.request(port, "/api/live/start", self.config)[0], 409)
        self.assertIs(self.manager.current, first)
        frame_path = f"/api/live/frame?session={first.id}"
        headers = {"X-Frame-Timestamp": "0", "X-Frame-Sequence": "0"}
        self.assertEqual(self.request(port, frame_path, self.jpeg, headers)[0], 202)
        first.publish()
        self.assertEqual(self.request(port, "/api/live/run")[1]["input_run_id"], first.id)
        code, stopped = self.request(port, "/api/live/stop", {"session_id": first.id})
        self.assertEqual((code, stopped["session_id"], stopped["session_status"]), (202, first.id, "complete"))
        self.assertEqual(self.request(port, "/api/live/state")[1]["session_status"], "complete")
        self.request(port, "/api/live/start", self.config)
        second = self.manager.current
        self.assertNotEqual(first.id, second.id)
        state = self.request(port, "/api/live/state")[1]
        self.assertFalse(state["map_available"])
        self.assertEqual(state["revision"], 0)
        self.assertEqual(self.request(port, "/api/live/run")[0], 503)
        self.assertEqual(self.request(port, f"/api/live/run?session={first.id}&revision=000001")[0], 409)
        self.assertEqual(self.request(port, f"/api/live/frame/same.jpg?session={first.id}")[0], 409)
        self.assertEqual(self.request(port, frame_path, self.jpeg, headers)[0], 409)
        self.assertEqual(self.request(port, "/api/live/stop", {"session_id": first.id})[0], 409)
        self.assertEqual(second.received, 0)
        self.assertFalse(second.done)
        self.assertEqual(self.request(port, "/api/saved/run")[1], saved_before)
        self.assertEqual(self.saved_hashes(), before)
        self.assertTrue((first.path / "revisions/000001/scene.json").is_file())

    def test_missing_saved_result_never_falls_back_to_live(self):
        self.manager.start(self.config)
        self.manager.current.publish()
        port = self.serve(result=None)
        code, state = self.request(port, "/api/saved/state")
        self.assertEqual(code, 200)
        self.assertEqual(state["source"], "saved")
        self.assertFalse(state["map_available"])
        self.assertIsNone(state["scale"])
        self.assertEqual(self.request(port, "/api/saved/run")[0], 503)
        self.assertEqual(self.request(port, "/api/live/run")[0], 200)

    def test_concurrent_reads_keep_maps_units_images_and_exports_separate(self):
        self.manager.start(self.config)
        current = self.manager.current
        current.publish()
        port = self.serve()
        paths = ["/api/saved/run", "/api/live/run", "/api/saved/state", "/api/live/state"] * 4
        with ThreadPoolExecutor(max_workers=8) as executor:
            responses = list(executor.map(lambda path: self.request(port, path), paths))
        for path, (code, value) in zip(paths, responses):
            self.assertEqual(code, 200)
            self.assertEqual(value["source"], "saved" if "/saved/" in path else "live")
            self.assertEqual(value["scale"]["unit"], "m" if "/saved/" in path else "arbitrary")
        self.assertNotIn("stream", responses[0][1])
        self.assertEqual(self.request(port, "/api/saved/frame/same.jpg")[1], b"saved image")
        self.assertEqual(self.request(port, "/api/live/frame/same.jpg")[1], current.id.encode())
        self.assertEqual(self.request(port, "/api/saved/points.ply")[1], b"saved image ply")
        self.assertEqual(self.request(port, "/api/live/points.ply")[1], current.id.encode() + b" ply")
        self.assertEqual(self.request(port, "/api/saved/slam-preview")[0], 404)
        self.assertEqual(self.request(port, "/api/live/slam-preview")[1], current.id.encode() + b" preview")

    def test_saved_objects_do_not_get_attached_to_live_map(self):
        value = json.loads((self.saved / "scene.json").read_text(encoding="utf-8"))
        binding = hashlib.sha256(json.dumps({key: value[key] for key in ("points", "cameras", "scale")},
            sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        objects = self.root / "objects"
        objects.mkdir()
        atomic_json(objects / "objects.json", {"schema_version": "planar-objects/1.0", "scene_binding_sha256": binding,
            "objects": [{"texture_file": "object-0001.jpg"}]})
        (objects / "textures").mkdir()
        (objects / "textures/object-0001.jpg").write_bytes(b"saved texture")
        self.manager.start(self.config)
        self.manager.current.publish()
        port = self.serve(objects=objects)
        self.assertEqual(self.request(port, "/api/saved/object-texture/object-0001.jpg")[1], b"saved texture")
        self.assertEqual(self.request(port, "/api/live/object-texture/object-0001.jpg")[0], 404)
        code, live = self.request(port, "/api/live/run")
        self.assertEqual(code, 200)
        self.assertNotIn("planar_objects", live)

    def test_old_revision_metadata_is_not_mixed_with_current_state(self):
        self.manager.start(self.config)
        current = self.manager.current
        current.publish(1)
        current.publish(2)
        port = self.serve()
        code, value = self.request(port, f"/api/live/run?session={current.id}&revision=000001")
        self.assertEqual(code, 200)
        self.assertEqual(value["revision"], 1)
        self.assertEqual(value["stream"]["revision"], 2)
        self.assertEqual(value["segment_id"], 7)
        self.assertEqual(value["stream"]["segment_id"], 14)
        self.assertEqual(self.request(port, "/api/saved/run?revision=000001")[0], 400)
        self.assertEqual(self.request(port, "/api/live/run?revision=../saved")[0], 404)

    def test_one_request_uses_one_session_even_during_restart(self):
        self.manager.start(self.config)
        first = self.manager.current
        first.publish()
        first.stop()
        self.manager.start(self.config)
        second = self.manager.current
        second.publish()

        class SwitchingManager:
            reads = 0

            @property
            def current(self):
                self.reads += 1
                return first if self.reads == 1 else second

        port = self.serve(live=SwitchingManager())
        code, state = self.request(port, f"/api/live/state?session={first.id}")
        self.assertEqual(code, 200)
        self.assertEqual(state["session_id"], first.id)
        self.assertEqual(state["session_status"], "complete")

    def test_recorded_session_and_saved_result_can_be_loaded_together(self):
        self.manager.start(self.config)
        recorded = self.manager.current
        recorded.publish()
        recorded.stop()
        port = self.serve(live=None, session=recorded.path)
        self.assertEqual(self.request(port, "/api/live/run")[1]["input_run_id"], recorded.id)
        self.assertEqual(self.request(port, "/api/saved/run")[1]["input_run_id"], "saved-run")
        self.assertEqual(self.request(port, "/api/run")[1]["input_run_id"], recorded.id)
        self.assertEqual(self.request(port, "/api/live/start", self.config)[0], 403)

    def test_lifecycle_and_input_sequence_are_explicit(self):
        self.manager.start(self.config)
        current = self.manager.current
        current.ready = False
        self.assertEqual(current.stats()["session_status"], "starting")
        current.ready = True
        self.assertEqual(current.stats()["session_status"], "receiving")
        current.offer(self.jpeg, 0., 0)
        self.assertEqual(current.stats()["last_input_sequence"], 0)
        self.assertEqual(current.stats()["last_input_timestamp_s"], 0.)
        with self.assertRaisesRegex(ValueError, "not_increasing"):
            current.offer(self.jpeg, 0., 0)
        current.closing = True
        self.assertEqual(current.stats()["session_status"], "stopping")
        current.done = True
        self.assertEqual(current.stats()["session_status"], "complete")
        current.error = "synthetic tracker failure"
        self.assertEqual(current.stats()["session_status"], "failed")

    def test_brief_state_file_sharing_violation_is_retried(self):
        self.manager.start(self.config)
        current = self.manager.current
        current.publish()
        port = self.serve()
        original = Path.read_text
        attempts = []

        def read(path, *args, **kwargs):
            if path == current.path / "state.json":
                attempts.append(path)
                if len(attempts) == 1:
                    raise PermissionError("synthetic sharing violation")
            return original(path, *args, **kwargs)

        with patch.object(Path, "read_text", read):
            code, state = self.request(port, "/api/live/state")
        self.assertEqual(code, 200)
        self.assertEqual(state["session_id"], current.id)
        self.assertEqual(len(attempts), 2)

    def test_unreadable_live_state_reports_its_source_without_fallback(self):
        self.manager.start(self.config)
        current = self.manager.current
        current.publish()
        port = self.serve()
        original = Path.read_text

        def read(path, *args, **kwargs):
            if path == current.path / "state.json":
                raise PermissionError("synthetic persistent sharing violation")
            return original(path, *args, **kwargs)

        with patch.object(Path, "read_text", read):
            code, state = self.request(port, "/api/live/state")
            self.assertEqual((code, state["source"], state["error"]), (503, "live", "source_unavailable"))
            self.assertEqual(self.request(port, "/api/saved/run")[0], 200)


class CancelledResponseTests(unittest.TestCase):
    def test_cancelled_json_writes_are_expected_but_other_io_errors_propagate(self):
        # Windows tab cancellation can raise 10053 after all response headers were sent.
        handler = ViewerHandler.__new__(ViewerHandler)
        handler.send_response, handler.send_header, handler.end_headers = Mock(), Mock(), Mock()
        handler.wfile = Mock()
        for error in [BrokenPipeError(), ConnectionResetError(), ConnectionAbortedError(10053, "cancelled")]:
            with self.subTest(error=type(error).__name__):
                handler.wfile.write.side_effect = error
                handler.json_response({"source": "saved"})
                handler.send_response.assert_called_with(200)
        handler.wfile.write.side_effect = OSError("unexpected I/O failure")
        with self.assertRaisesRegex(OSError, "unexpected I/O failure"):
            handler.json_response({"source": "saved"})


if __name__ == "__main__":
    unittest.main()
