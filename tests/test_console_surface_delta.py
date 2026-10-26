"""Synthetic coverage of the optional, session-bound ORB snapshot delta endpoint."""
from copy import deepcopy
from functools import partial
from http.server import ThreadingHTTPServer
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("console_surface_server", ROOT / "apps/console/serve.py")
SERVER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SERVER)


def scene(revision):
    return {"schema_version": "phone-sfm-result/1.2", "pose_source": "orb_slam3",
        "input_run_id": "synthetic", "segment_id": 0, "revision": revision,
        "points": [[i, i / 100, 0, 0, 160, 170, 180, .5, 3] for i in range(128)],
        "cameras": [{"frame_id": str(i)} for i in range(3)], "scale": {"unit": "arbitrary"},
        "point_observations": {"schema_version": "point-observations/1.0", "basis": "keyframes",
            "frame_index_by_point": {str(i): [0, 1, 2] for i in range(128)}}}


class ConsoleDeltaTests(unittest.TestCase):
    def setUp(self):
        self.before, self.after = scene(1), scene(2)
        self.after["points"] = list(reversed(self.after["points"][1:]))
        self.after["points"][0][1] += .1
        del self.after["point_observations"]["frame_index_by_point"]["0"]
        self.after["point_observations"]["frame_index_by_point"]["1"] = [0, 2]

    def test_exact_round_trip_and_no_source_mutation(self):
        original = deepcopy(self.before), deepcopy(self.after)
        delta = SERVER.scene_delta(self.before, self.after)
        rows = {p[0]: p for p in self.before["points"]}
        for key in delta["points_removed"]:
            rows.pop(key)
        rows.update({p[0]: p for p in delta["points_upsert"]})
        frames = deepcopy(self.before["point_observations"]["frame_index_by_point"])
        for key in delta["observation_removed"]:
            del frames[key]
        frames.update(delta["observation_upserts"])
        restored = {**delta["scene"], "points": [rows[i] for i in delta["point_order"]],
            "point_observations": {**delta["observation_metadata"], "frame_index_by_point": frames}}
        self.assertEqual(restored, self.after)
        self.assertEqual(original, (self.before, self.after))

    def test_incompatible_or_missing_bases_require_a_full_snapshot(self):
        for change in ({"segment_id": 1}, {"input_run_id": "other"}, {"pose_source": "sfm"},
                       {"revision": 0}, {"point_observations": None}):
            with self.subTest(change=change):
                self.assertIsNone(SERVER.scene_delta(self.before, {**self.after, **change}))

    def test_http_delta_fallbacks_and_static_whitelist(self):
        with tempfile.TemporaryDirectory(prefix="console-delta-") as directory:
            session = Path(directory) / "synthetic"
            for revision, value in ((1, self.before), (2, self.after)):
                target = session / "revisions" / f"{revision:06d}"
                target.mkdir(parents=True)
                (target / "scene.json").write_text(json.dumps(value), encoding="utf-8")
                (target / "status.json").write_text('{"status":"partial"}', encoding="utf-8")
            (session / "state.json").write_text(json.dumps({"revision": 2, "revision_path": "revisions/000002"}), encoding="utf-8")
            handler = partial(SERVER.ViewerHandler, session=session, timeline=SERVER.Timeline())
            server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            base = f"http://127.0.0.1:{server.server_port}"
            try:
                with urlopen(base + "/api/run?since_revision=000001&session=synthetic") as response:
                    delta = json.load(response)
                self.assertEqual(delta["schema_version"], "phone-map-delta/1.0")
                self.assertEqual(delta["points_removed"], [0])
                with urlopen(base + "/api/live/run?since_revision=000001&revision=000002&session=synthetic") as response:
                    scoped = json.load(response)
                self.assertEqual(scoped["scene"]["source"], "live")
                self.assertEqual(scoped["revision"], 2)
                self.assertEqual(scoped["points_removed"], [0])
                with urlopen(base + "/api/run?since_revision=000099") as response:
                    self.assertEqual(json.load(response)["points"], self.after["points"])
                for path, expected in (("/api/run?since_revision=../", 404),
                                       ("/api/run?session=other", 409), ("/point-surfaces.test.js", 404)):
                    with self.subTest(path=path), self.assertRaises(HTTPError) as caught:
                        urlopen(base + path)
                    self.assertEqual(caught.exception.code, expected)
                    caught.exception.close()
                for path in ("/point-surface-worker.js", "/map-delta.js", "/wide-surfaces.js"):
                    with urlopen(base + path) as response:
                        self.assertEqual(response.status, 200)
                        self.assertIn("javascript", response.headers["Content-Type"])
            finally:
                server.shutdown()
                server.server_close()
                thread.join()


if __name__ == "__main__":
    unittest.main()
