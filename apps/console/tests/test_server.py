from functools import partial
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
from threading import Thread
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from serve import ViewerHandler


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        (self.root / "status.json").write_text('{"status":"complete"}', encoding="utf-8")
        (self.root / "scene.json").write_text('{"ok":true}', encoding="utf-8")
        (self.root / "frame.jpg").write_bytes(b"fixture")
        (self.root / "local_assets.json").write_text(json.dumps({"images_root": str(self.root), "frames": ["frame.jpg"]}), encoding="utf-8")
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), partial(ViewerHandler, result=self.root))
        self.thread = Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        self.temporary.cleanup()

    def test_result_and_registered_frame_only(self):
        with urlopen(self.base + "/api/run") as response:
            self.assertEqual(json.load(response), {"ok": True})
        with urlopen(self.base + "/api/frame/frame.jpg") as response:
            self.assertEqual(response.read(), b"fixture")
        for path in ["/.env", "/data/real_data/", "/api/frame/../scene.json", "/api/frame/%2e%2e%2fscene.json", "/api/frame/other.jpg", "/local_assets.json", "/node_modules/three/package.json"]:
            with self.assertRaises(HTTPError) as error:
                urlopen(self.base + path)
            self.assertEqual(error.exception.code, 404)

    def test_incomplete_result_is_not_served(self):
        (self.root / "status.json").write_text('{"status":"failed"}', encoding="utf-8")
        with self.assertRaises(HTTPError) as error:
            urlopen(self.base + "/api/run")
        self.assertEqual(error.exception.code, 503)

    def test_unrecognized_host_is_rejected(self):
        with self.assertRaises(HTTPError) as error:
            urlopen(Request(self.base + "/api/run", headers={"Host": "untrusted.example"}))
        self.assertEqual(error.exception.code, 403)


if __name__ == "__main__":
    unittest.main()
