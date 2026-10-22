"""Loopback-only viewer: expose result files and registered frames, never a directory."""
from __future__ import annotations

import argparse
import hashlib
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import re
from urllib.parse import parse_qs, unquote, urlsplit

APP = Path(__file__).resolve().parent


class ViewerHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, result=None, session=None, surface=None, **kwargs):
        self.result = Path(result).resolve() if result else None
        self.session = Path(session).resolve() if session else None
        self.surface = Path(surface).resolve() if surface else None
        super().__init__(*args, **kwargs)

    def json_response(self, value, status=200):
        payload = json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def snapshot(self, revision=None):
        if not self.session:
            return self.result, None
        if not (self.session / "state.json").is_file():
            return None, {"status": "starting", "revision": 0, "map_available": False, "frame_drops": 0}
        state = json.loads((self.session / "state.json").read_text(encoding="utf-8"))
        name = f"revisions/{revision}" if revision else state.get("revision_path", "")
        if not re.fullmatch(r"revisions/\d{6}", name):
            return None, state
        result = (self.session / name).resolve()
        if result.parent != (self.session / "revisions").resolve():
            return None, state
        return result, state

    def do_GET(self):
        host = self.headers.get("Host", "").split(":", 1)[0]
        if host not in {"127.0.0.1", "localhost"}:
            self.send_error(403)
            return
        url = urlsplit(self.path)
        path = unquote(url.path)
        try:
            result, state = self.snapshot()
        except (OSError, ValueError):
            self.json_response({"status": "unavailable"}, 503)
            return
        if path == "/api/state":
            self.json_response(state or {"status": "offline", "map_available": True, "revision": 0})
            return
        if path == "/api/run":
            if result is None or not (result / "status.json").is_file() or json.loads(
                    (result / "status.json").read_text(encoding="utf-8")).get("status") != "complete":
                self.json_response(state or {"status": "incomplete"}, 503)
                return
            value = json.loads((result / "scene.json").read_text(encoding="utf-8"))
            if self.surface:
                value["surface"] = json.loads(self.surface.read_text(encoding="utf-8"))
                binding = hashlib.sha256(json.dumps({key: value[key] for key in ("points", "cameras", "scale")},
                    sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
                if value["surface"].get("scene_binding_sha256") != binding:
                    self.json_response({"status": "surface_does_not_match_scene"}, 503)
                    return
            if state:
                value["stream"] = state
            self.json_response(value)
            return
        static = {"/": APP / "index.html", "/app.js": APP / "app.js", "/style.css": APP / "style.css",
                  "/vendor/three.module.js": APP / "node_modules/three/build/three.module.js",
                  "/vendor/three.core.js": APP / "node_modules/three/build/three.core.js",
                  "/vendor/OrbitControls.js": APP / "node_modules/three/examples/jsm/controls/OrbitControls.js"}
        exports = {"/api/points.ply": "points.ply",
                   "/api/poses.json": "poses.json", "/api/trajectory.csv": "trajectory.csv"}
        target = static.get(path)
        if path in exports:
            if result is None or not (result / "status.json").is_file() or json.loads(
                    (result / "status.json").read_text(encoding="utf-8")).get("status") != "complete":
                self.send_error(503, "Result is incomplete")
                return
            target = result / exports[path]
        elif path.startswith("/api/frame/"):
            name = path.removeprefix("/api/frame/")
            requested_revision = parse_qs(url.query).get("revision", [None])[0]
            if self.session and requested_revision:
                if not re.fullmatch(r"\d{6}", requested_revision):
                    self.send_error(404)
                    return
                result, _ = self.snapshot(requested_revision)
            if result is None:
                self.send_error(404)
                return
            if not (result / "local_assets.json").is_file():
                self.send_error(404)
                return
            assets = json.loads((result / "local_assets.json").read_text(encoding="utf-8"))
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
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--result", type=Path)
    source.add_argument("--session", type=Path)
    parser.add_argument("--surface", type=Path, help="Optional observed-surface JSON for an offline result")
    parser.add_argument("--port", type=int, default=8790)
    args = parser.parse_args()
    result = args.result.resolve() if args.result else None
    if result and (not (result / "status.json").is_file() or json.loads(
            (result / "status.json").read_text(encoding="utf-8")).get("status") != "complete"):
        parser.error("Select a completed reconstruction result")
    if not (APP / "node_modules/three/build/three.module.js").is_file():
        parser.error("Install apps/console dependencies first: pnpm install --frozen-lockfile")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), partial(ViewerHandler, result=result,
        session=args.session, surface=args.surface))
    print(f"Phone mapping viewer: http://127.0.0.1:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
