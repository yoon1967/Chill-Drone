"""Loopback-only viewer: expose result files and registered frames, never a directory."""
from __future__ import annotations

import argparse
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
from urllib.parse import unquote, urlsplit

APP = Path(__file__).resolve().parent


class ViewerHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, result, **kwargs):
        self.result = Path(result).resolve()
        super().__init__(*args, **kwargs)

    def do_GET(self):
        host = self.headers.get("Host", "").split(":", 1)[0]
        if host not in {"127.0.0.1", "localhost"}:
            self.send_error(403)
            return
        path = unquote(urlsplit(self.path).path)
        static = {"/": APP / "index.html", "/app.js": APP / "app.js", "/style.css": APP / "style.css",
                  "/vendor/three.module.js": APP / "node_modules/three/build/three.module.js",
                  "/vendor/three.core.js": APP / "node_modules/three/build/three.core.js",
                  "/vendor/OrbitControls.js": APP / "node_modules/three/examples/jsm/controls/OrbitControls.js"}
        exports = {"/api/run": "scene.json", "/api/points.ply": "points.ply",
                   "/api/poses.json": "poses.json", "/api/trajectory.csv": "trajectory.csv"}
        target = static.get(path)
        if path in exports:
            if not (self.result / "status.json").is_file() or json.loads(
                    (self.result / "status.json").read_text(encoding="utf-8")).get("status") != "complete":
                self.send_error(503, "Result is incomplete")
                return
            target = self.result / exports[path]
        elif path.startswith("/api/frame/"):
            name = path.removeprefix("/api/frame/")
            assets = json.loads((self.result / "local_assets.json").read_text(encoding="utf-8"))
            if name not in assets["frames"] or Path(name).name != name:
                self.send_error(404)
                return
            root = Path(assets["images_root"]).resolve()
            target = (root / name).resolve()
            if target.parent != root:
                self.send_error(404)
                return
        if target is None or not target.is_file():
            self.send_error(404)
            return
        payload = target.read_bytes()
        self.send_response(200)
        content_type = "text/javascript" if target.suffix == ".js" else mimetypes.guess_type(target.name)[0]
        self.send_header("Content-Type", content_type or "application/octet-stream")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        if path in exports and path != "/api/run":
            self.send_header("Content-Disposition", f'attachment; filename="{exports[path]}"')
        self.end_headers()
        self.wfile.write(payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--port", type=int, default=8790)
    args = parser.parse_args()
    result = args.result.resolve()
    if not (result / "status.json").is_file() or json.loads(
            (result / "status.json").read_text(encoding="utf-8")).get("status") != "complete":
        parser.error("Select a completed reconstruction result")
    if not (APP / "node_modules/three/build/three.module.js").is_file():
        parser.error("Install apps/console dependencies first: pnpm install --frozen-lockfile")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), partial(ViewerHandler, result=result))
    print(f"Phone mapping viewer: http://127.0.0.1:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
