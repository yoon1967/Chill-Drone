"""게이트웨이가 내주는 관제 화면 파일에 Cache-Control: no-cache가 붙는지 본다(fastapi가 있는 게이트웨이 환경에서만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_controller_files.py" -v
"""

import asyncio
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from gateway_server import ControllerFiles  # noqa: E402
except ImportError:  # fastapi 없는 환경
    ControllerFiles = None


def scope(path, headers=()):
    return {"type": "http", "method": "GET", "path": path, "root_path": "", "headers": list(headers),
            "query_string": b""}


@unittest.skipIf(ControllerFiles is None, "fastapi가 없는 환경")
class ControllerFilesTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        for name, body in (("index.html", "<!doctype html>"), ("app.js", "console.log(1)")):
            with open(os.path.join(self.tmp.name, name), "w", encoding="utf-8") as f:
                f.write(body)
        self.files = ControllerFiles(directory=self.tmp.name, html=True)

    def tearDown(self):
        self.tmp.cleanup()

    def get(self, path, headers=()):
        return asyncio.run(self.files.get_response(path.lstrip("/"), scope(path, headers)))

    def test_files_are_revalidated(self):
        for path in ("/app.js", "/"):
            r = self.get(path)
            self.assertEqual((r.status_code, r.headers.get("cache-control")), (200, "no-cache"), path)

    def test_unchanged_file_is_304(self):   # 매번 물어봐도 그대로면 본문 없이 304
        etag = self.get("/app.js").headers["etag"]
        r = self.get("/app.js", [(b"if-none-match", etag.encode())])
        self.assertEqual((r.status_code, r.headers.get("cache-control")), (304, "no-cache"))


if __name__ == "__main__":
    unittest.main()
