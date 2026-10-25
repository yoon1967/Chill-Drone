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
import sys
import threading
from urllib.parse import parse_qs, unquote, urlsplit

APP = Path(__file__).resolve().parent


class Timeline:
    def __init__(self):
        self.cache = {}
        self.lock = threading.Lock()

    def read(self, session, state):
        if not session:
            return {"schema_version": "orb-map-timeline/1.0", "entries": []}
        path = session / "timeline.json"
        if path.is_file():
            return json.loads(path.read_text(encoding="utf-8"))
        # Old ORB sessions already contain real map revisions, but no timeline index.
        key = (str(session), state.get("revision"))
        with self.lock:
            if key not in self.cache:
                entries = []
                for item in sorted((session / "revisions").glob("[0-9][0-9][0-9][0-9][0-9][0-9]/scene.json")):
                    scene = json.loads(item.read_text(encoding="utf-8"))
                    if not scene.get("slam"):
                        continue
                    slam = scene["slam"]
                    entries.append({"revision": scene["revision"], "map_id": scene["segment_id"],
                        "timestamp_s": slam.get("timestamp_s", max(c["timestamp_s"] for c in scene["cameras"])),
                        "timestamp_basis": slam.get("timestamp_basis", "latest_surviving_keyframe_estimate"),
                        "points3D": len(scene["points"]), "keyframes": len(scene["cameras"]), "final": slam["final"]})
                self.cache = {key: {"schema_version": "orb-map-timeline/1.0", "session_id": session.name, "entries": entries}}
            return self.cache[key]


class ViewerHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, result=None, session=None, objects=None, live=None, timeline=None, demo=None, **kwargs):
        self.result = Path(result).resolve() if result else None
        self.session = Path(session).resolve() if session else None
        self.objects = Path(objects).resolve() if objects else None
        self.live, self.timeline, self.demo = live, timeline, demo
        self.saved_session = self.session
        if live and live.current:
            self.session = live.current.path
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
        state.pop("surface", None)  # Retired point-connectivity mesh statistics in older sessions.
        name = f"revisions/{revision}" if revision else state.get("revision_path", "")
        if not re.fullmatch(r"revisions/\d{6}", name):
            return None, state
        result = (self.session / name).resolve()
        if result.parent != (self.session / "revisions").resolve():
            return None, state
        return result, state

    def do_POST(self):
        # A custom header plus exact Origin/Host checks prevent cross-site camera starts.
        host = self.headers.get("Host", "")
        origin = self.headers.get("Origin")
        if (host not in {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}
                or origin not in {None, "http://" + host} or self.headers.get("X-Mapping-Client") != "1"):
            self.send_error(403)
            return
        if not self.live:
            self.json_response({"error": "camera_input_disabled"}, 403)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 2097152:
                self.json_response({"error": "body_size_out_of_range"}, 413)
                return
            self.connection.settimeout(10)
            data = self.rfile.read(length)
            if len(data) != length:
                raise ValueError("incomplete_body")
            url = urlsplit(self.path)
            if url.path == "/api/live/start":
                self.json_response(self.live.start(json.loads(data)), 201)
            elif url.path == "/api/live/frame":
                query = parse_qs(url.query)
                session = self.live.require(query.get("session", [""])[0])
                self.json_response(session.offer(data, float(self.headers["X-Frame-Timestamp"]), int(self.headers["X-Frame-Sequence"])), 202)
            elif url.path == "/api/live/stop":
                self.live.require(json.loads(data)["session_id"]).stop()
                self.json_response({"status": "stopping"}, 202)
            else:
                self.send_error(404)
        except (ValueError, KeyError, TypeError) as error:
            self.json_response({"error": str(error)}, 400)
        except RuntimeError as error:
            self.json_response({"error": str(error)}, 409)
        except Exception as error:
            self.json_response({"error": "camera_backend_unavailable", "detail": str(error)}, 503)

    def log_message(self, format, *args):
        # Frame/status polling is operational traffic; avoid an unbounded terminal log.
        if args and re.search(r"/api/(?:state|run|timeline|live/frame|slam-preview|frame/)", str(args[0])):
            return
        super().log_message(format, *args)

    def source_video(self):
        if not self.demo or not self.demo.is_file():
            self.send_error(404)
            return
        size = self.demo.stat().st_size
        start, end = 0, size - 1
        range_header = self.headers.get("Range")
        if range_header:
            match = re.fullmatch(r"bytes=(\d+)-(\d*)", range_header)
            if not match:
                self.send_error(416)
                return
            start = int(match[1])
            end = min(int(match[2]) if match[2] else end, end)
            if start > end:
                self.send_error(416)
                return
        self.send_response(206 if range_header else 200)
        self.send_header("Content-Type", "video/mp4")
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Length", str(end - start + 1))
        self.send_header("Cache-Control", "no-store")
        if range_header:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.end_headers()
        with self.demo.open("rb") as video:
            video.seek(start)
            remaining = end - start + 1
            while remaining:
                chunk = video.read(min(remaining, 65536))
                if not chunk:
                    break
                try:
                    self.wfile.write(chunk)
                except (BrokenPipeError, ConnectionResetError):
                    break  # Browsers routinely cancel MP4 range requests while seeking.
                remaining -= len(chunk)

    def object_collection(self, result, scene):
        collection = json.loads((self.objects / "objects.json").read_text(encoding="utf-8")) if self.objects else scene.get("planar_objects")
        if collection is not None:
            if not isinstance(collection, dict):
                raise ValueError("invalid_objects_collection")
            binding = hashlib.sha256(json.dumps({key: scene[key] for key in ("points", "cameras", "scale")},
                sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
            if collection.get("schema_version") != "planar-objects/1.0" or collection.get("scene_binding_sha256") != binding:
                raise ValueError("objects_do_not_match_scene")
            if not isinstance(collection.get("objects"), list) or len(collection["objects"]) > 64:
                raise ValueError("too_many_objects")
            if any(not isinstance(o, dict) or not re.fullmatch(r"object-\d{4,8}\.jpg", str(o.get("texture_file", "")))
                   for o in collection["objects"]):
                raise ValueError("invalid_object_texture")
        return collection

    def do_GET(self):
        host = self.headers.get("Host", "").split(":", 1)[0]
        if host not in {"127.0.0.1", "localhost"}:
            self.send_error(403)
            return
        url = urlsplit(self.path)
        path = unquote(url.path)
        if path == "/api/source-video":
            self.source_video()
            return
        if path == "/api/live":
            camera = None
            if self.demo and self.saved_session and (self.saved_session / "camera.json").is_file():
                camera = json.loads((self.saved_session / "camera.json").read_text(encoding="utf-8"))
            self.json_response({"enabled": self.live is not None, "demo_available": self.demo is not None,
                "demo_camera": camera, "current": self.live.current.stats() if self.live and self.live.current else None})
            return
        try:
            requested = parse_qs(url.query).get("revision", [None])[0] if path == "/api/run" else None
            requested_session = parse_qs(url.query).get("session", [None])[0]
            if requested_session and (not self.session or requested_session != self.session.name):
                self.json_response({"error": "session_changed"}, 409)
                return
            if requested and not re.fullmatch(r"\d{6}", requested):
                self.send_error(404)
                return
            result, state = self.snapshot(requested)
        except (OSError, ValueError):
            self.json_response({"status": "unavailable"}, 503)
            return
        if path == "/api/state":
            if state and self.session:
                state["session_id"] = self.session.name
            if self.live and self.live.current:
                state.update(self.live.current.stats())
            self.json_response(state or {"status": "offline", "map_available": True, "revision": 0})
            return
        if path == "/api/timeline":
            self.json_response(self.timeline.read(self.session, state or {}))
            return
        if path == "/api/run":
            if result is None or not (result / "status.json").is_file() or json.loads(
                    (result / "status.json").read_text(encoding="utf-8")).get("status") not in {"complete", "partial"}:
                self.json_response(state or {"status": "incomplete"}, 503)
                return
            value = json.loads((result / "scene.json").read_text(encoding="utf-8"))
            value.pop("surface", None)  # Read old results without exposing the retired mesh.
            if state:
                value["stream"] = state
            try:
                collection = self.object_collection(result, value)
            except (OSError, ValueError):
                self.json_response({"status": "objects_unavailable_or_mismatched"}, 503)
                return
            if collection:
                value["planar_objects"] = collection
            self.json_response(value)
            return
        static = {"/": APP / "index.html", "/app.js": APP / "app.js", "/objects.js": APP / "objects.js", "/capture.js": APP / "capture.js", "/point-grid.js": APP / "point-grid.js", "/style.css": APP / "style.css",
                  "/vendor/three.module.js": APP / "node_modules/three/build/three.module.js",
                  "/vendor/three.core.js": APP / "node_modules/three/build/three.core.js",
                  "/vendor/OrbitControls.js": APP / "node_modules/three/examples/jsm/controls/OrbitControls.js"}
        exports = {"/api/points.ply": "points.ply",
                   "/api/poses.json": "poses.json", "/api/trajectory.csv": "trajectory.csv"}
        target = static.get(path)
        if path == "/api/slam-preview" and self.session:
            target = self.session / "preview.jpg"
        if path in exports:
            if result is None or not (result / "status.json").is_file() or json.loads(
                    (result / "status.json").read_text(encoding="utf-8")).get("status") not in {"complete", "partial"}:
                self.send_error(503, "Result is incomplete")
                return
            target = result / exports[path]
        elif path.startswith("/api/object-texture/"):
            name = path.removeprefix("/api/object-texture/")
            requested_revision = parse_qs(url.query).get("revision", [None])[0]
            if not re.fullmatch(r"object-\d{4,8}\.jpg", name):
                self.send_error(404)
                return
            if self.session and requested_revision:
                if not re.fullmatch(r"\d{6}", requested_revision):
                    self.send_error(404)
                    return
                result, _ = self.snapshot(requested_revision)
            if result is None:
                self.send_error(404)
                return
            try:
                value = json.loads((result / "scene.json").read_text(encoding="utf-8"))
                collection = self.object_collection(result, value)
                allowed = {o["texture_file"] for o in (collection or {}).get("objects", [])}
            except (OSError, ValueError, KeyError):
                allowed = set()
            if name not in allowed:
                self.send_error(404)
                return
            texture_root = (self.objects / "textures" if self.objects else result / "object_textures").resolve()
            target = (texture_root / name).resolve()
            if target.parent != texture_root:
                self.send_error(404)
                return
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
    parser.add_argument("--objects", type=Path, help="Optional planar object bundle for an offline result")
    parser.add_argument("--port", type=int, default=8790)
    parser.add_argument("--enable-camera", action="store_true", help="Enable local browser camera -> Docker ORB-SLAM3")
    parser.add_argument("--live-root", type=Path, default=APP.parents[1] / "output/phone_mapping")
    parser.add_argument("--demo-source", type=Path, help="One local MP4 explicitly exposed for streaming demonstration")
    args = parser.parse_args()
    result = args.result.resolve() if args.result else None
    if args.session and args.objects:
        parser.error("Online sessions use revision-bound objects")
    if result and (not (result / "status.json").is_file() or json.loads(
            (result / "status.json").read_text(encoding="utf-8")).get("status") not in {"complete", "partial"}):
        parser.error("Select a complete or partial reconstruction result")
    if not (APP / "node_modules/three/build/three.module.js").is_file():
        parser.error("Install apps/console dependencies first: pnpm install --frozen-lockfile")
    live = None
    if args.enable_camera:
        sys.path.insert(0, str(APP.parents[1]))
        from modules.mapping.localization.orbslam3_live import LiveManager
        live = LiveManager(args.live_root)
    demo = args.demo_source.resolve() if args.demo_source else None
    if demo and (not demo.is_file() or demo.suffix.lower() != ".mp4"):
        parser.error("Demo source must be a local MP4")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), partial(ViewerHandler, result=result,
        session=args.session, objects=args.objects, live=live, timeline=Timeline(), demo=demo))
    print(f"Phone mapping viewer: http://127.0.0.1:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if live:
            live.shutdown()


if __name__ == "__main__":
    main()
