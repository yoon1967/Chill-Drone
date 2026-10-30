"""Loopback-only viewer: expose result files and registered frames, never a directory."""
from __future__ import annotations

import argparse
import hashlib
from functools import lru_cache, partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import re
import sys
import threading
import time
import zipfile
from types import SimpleNamespace
from urllib.parse import parse_qs, unquote, urlsplit

APP = Path(__file__).resolve().parent
sys.path.insert(0, str(APP.parents[1]))
from modules.mapping.localization.observations import native_observations
from modules.mapping.data.live_recording import archive_files
from modules.mapping.data.session_import import ImportManager, inside
from modules.mapping.data.camera_profiles import CameraProfiles


def scene_delta(previous, current):
    """ORB IDs are stable only within a session/map. Other producers keep full snapshots."""
    if (previous.get("pose_source") != "orb_slam3" or current.get("pose_source") != "orb_slam3"
            or any(previous.get(k) != current.get(k) for k in ("input_run_id", "segment_id", "schema_version"))
            or previous.get("revision", 0) >= current.get("revision", 0)):
        return None
    old_evidence, evidence = previous.get("point_observations"), current.get("point_observations")
    if (not old_evidence or not evidence or old_evidence.get("schema_version") != "point-observations/1.0"
            or evidence.get("schema_version") != "point-observations/1.0"):
        return None
    old_rows = {p[0]: p for p in previous["points"]}
    rows = {p[0]: p for p in current["points"]}
    old_frames, frames = old_evidence["frame_index_by_point"], evidence["frame_index_by_point"]
    return {"schema_version": "phone-map-delta/1.0", "base_revision": previous["revision"],
        "revision": current["revision"], "session_id": current["input_run_id"], "segment_id": current["segment_id"],
        "scene": {k: v for k, v in current.items() if k not in {"points", "point_observations"}},
        "points_upsert": [p for p in current["points"] if old_rows.get(p[0]) != p],
        "points_removed": [key for key in old_rows if key not in rows], "point_order": [p[0] for p in current["points"]],
        "observation_metadata": {k: v for k, v in evidence.items() if k != "frame_index_by_point"},
        "observation_upserts": {k: v for k, v in frames.items() if old_frames.get(k) != v},
        "observation_removed": [k for k in old_frames if k not in frames]}


def load_json(path):
    # Windows can briefly deny a reader while an atomic publisher replaces the file.
    for attempt in range(4):
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except PermissionError:
            if attempt == 3:
                raise
            time.sleep(.01)


def prepare_demo_camera(source, *, camera=None, calibration_result=None, session=None):
    """Validate a recorded input against its own calibration before serving it."""
    from modules.mapping.localization.orbslam3 import calibration
    from modules.mapping.localization.orbslam3_live import live_camera

    if camera is None and calibration_result is None:
        camera = Path(session) / "camera.json" if session else None
        if camera is None or not camera.is_file():
            raise ValueError("Demo requires --demo-camera, --demo-calibration-result, or a session camera.json")
    value = calibration(SimpleNamespace(source=source, camera=camera,
        calibration_result=calibration_result, width=960))
    # The browser caps both dimensions, including portrait recordings.
    if value["height"] > 960:
        value = calibration(SimpleNamespace(source=source, camera=camera,
            calibration_result=calibration_result, width=round(value["width"] * 960 / value["height"])))
    return live_camera({"width": value["width"], "height": value["height"],
                        "fps": value["fps"], "camera": value})


class ObservationCache:
    """Enrich legacy scenes in memory, without rewriting any saved reconstruction."""
    def __init__(self):
        self.key = None
        self.value = None
        self.lock = threading.Lock()

    def read(self, session, scene):
        if not session or scene.get("pose_source") != "orb_slam3":
            return None
        path = session / "native.json"
        try:
            stat = path.stat()
            key = (str(path), stat.st_mtime_ns, stat.st_size, scene.get("revision"), scene.get("segment_id"))
            with self.lock:
                if key != self.key:
                    value = native_observations(scene, json.loads(path.read_text(encoding="utf-8")))
                    self.key, self.value = key, value
                return self.value
        except (OSError, ValueError, KeyError, TypeError):
            return None


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


class ViewerServer(ThreadingHTTPServer):
    # JPEG uploads and the viewer's parallel state/image requests share this listener.
    request_queue_size = 32


@lru_cache(maxsize=8)
def result_metadata(path, modified_ns, size):
    """Cache only public coordinate metadata, keyed by the actual scene version."""
    scene = load_json(path)
    return {key: scene[key] for key in ("coordinate_system", "scale", "segment_id", "input_run_id") if key in scene}


class ViewerHandler(BaseHTTPRequestHandler):
    tello_input_lock = threading.Lock()
    def __init__(self, *args, result=None, session=None, objects=None, live=None, tello=None, tello_map=None,
                 timeline=None, observations=None, demo=None, demo_camera=None, imports=None, profiles=None, **kwargs):
        self.profiles = profiles
        self.imports = imports
        self.imported = None
        self.saved_result = Path(result).resolve() if result else None
        self.recorded_session = Path(session).resolve() if session else None
        self.saved_objects = Path(objects).resolve() if objects else None
        self.live, self.tello, self.timeline, self.demo = live, tello, timeline, demo
        self.tello_map = tello_map
        self.demo_camera = demo_camera
        self.observations = observations
        # Capture one session per request, even when another request starts a new session.
        self.current = live.current if live else None
        self.select_source()
        super().__init__(*args, **kwargs)

    def select_source(self, source=None):
        self.source = source or ("live" if self.current or self.recorded_session or not self.saved_result else "saved")
        self.session = ((self.current.path if self.current else self.recorded_session) if self.source == 'live' else
                        self.imported.root if self.imported and self.imported.is_session else None)
        self.result = (self.imported.result() if self.imported else self.saved_result) if self.source == 'saved' else None
        self.objects = self.saved_objects if self.source == "saved" and not self.imported else None

    def source_state(self, result, state):
        value = dict(state or {})
        if self.source == "saved" and not self.session:
            value.update(status="offline", session_status=None, session_id=None, revision=0, map_available=False)
        elif not self.session:
            value.update(status="idle", session_status="idle", session_id=None, revision=0, map_available=False)
        else:
            value["session_id"] = self.imported.state['session_id'] if self.imported else self.session.name
            if self.current and self.source == 'live':
                value.update(self.current.stats())
            elif (self.session / 'session.json').is_file():
                try:
                    value['recording'] = {**load_json(self.session / 'session.json')['recording'],
                                          'session_path': str(self.session)}
                except (OSError, ValueError, KeyError, TypeError):
                    pass
            value.setdefault("session_status", value.get("status", "starting"))
        value.update(schema_version="mapping-source-state/1.0", source=self.source,
                     coordinate_system=None, scale=None, map_available=False)
        value.setdefault("segment_id", None)
        if (result and (result / "scene.json").is_file() and (result / "status.json").is_file()
                and load_json(result / "status.json").get("status") in {"complete", "partial"}):
            path = result / "scene.json"
            stat = path.stat()
            metadata = result_metadata(path, stat.st_mtime_ns, stat.st_size)
            value.update(metadata)
            value["map_available"] = True
        elif self.source == "live" and value.get("backend") == "orb_slam3_monocular":
            value.update(scale={"status": "arbitrary", "unit": "arbitrary"}, coordinate_system={
                "native": "ORB-SLAM3 world; camera x-right y-down z-forward",
                "display_rotation": [[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]]})
        return value

    def json_response(self, value, status=200):
        payload = json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.json_payload(payload, status)

    def json_payload(self, payload, status=200):
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.write_payload(payload)

    def write_payload(self, payload):
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass  # Switching tabs cancels in-flight JSON/image requests in the browser.

    def snapshot(self, revision=None, state=None):
        if self.source == 'saved' and self.imported:
            return self.imported.result(revision), dict(self.imported.state)
        if not self.session:
            return self.result, None
        if not (self.session / "state.json").is_file():
            return None, {"status": "starting", "revision": 0, "map_available": False, "frame_drops": 0}
        if state is None:
            state = load_json(self.session / "state.json")
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
        if urlsplit(self.path).path.startswith('/api/import/'):
            self.import_request()
            return
        if urlsplit(self.path).path.startswith('/api/camera-profiles'):
            self.profile_request()
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
            if url.path == "/api/tello/start":
                if not self.tello:
                    self.json_response({"error": "tello_input_disabled"}, 403)
                    return
                with self.tello_input_lock:
                    if self.tello_map and self.tello_map.busy:
                        self.json_response({"error": "tello_mapping_input_is_active"}, 409)
                        return
                    self.json_response(self.tello.start(), 201)
            elif url.path == "/api/tello/stop":
                if not self.tello:
                    self.json_response({"error": "tello_input_disabled"}, 403)
                    return
                self.json_response(self.tello.stop(), 202)
            elif url.path.startswith("/api/tello/flight/"):
                if not self.tello or not self.tello.running:
                    self.json_response({"error": "tello_not_running"}, 409)
                    return
                flight = self.tello.require_flight()
                action = url.path.rsplit("/", 1)[-1]
                body = json.loads(data)
                if action == "confirm":
                    flight.confirm_operator(bool(body.get("confirmed")))
                    self.json_response(flight.status())
                elif action == "takeoff":
                    self.json_response(flight.takeoff(source="web"))
                elif action == "land":
                    self.json_response(flight.land(source="web"))
                elif action == "emergency":
                    self.json_response(flight.emergency(source="web"))
                elif action == "rc":
                    values = body.get("rc")
                    if not isinstance(values, list) or len(values) != 4:
                        raise ValueError("rc_requires_four_values")
                    flight.set_input(*(float(value) for value in values), source="keyboard")
                    self.json_response(flight.status(), 202)
                else:
                    self.send_error(404)
            elif url.path == "/api/live/tello/start":
                if not self.tello_map:
                    self.json_response({"error": "tello_mapping_input_disabled"}, 403)
                    return
                with self.tello_input_lock:
                    if self.tello and self.tello.running:
                        self.json_response({"error": "tello_detection_input_is_active"}, 409)
                        return
                    self.json_response(self.tello_map.start(self.profile_config(json.loads(data), 'tello')), 201)
            elif url.path == "/api/live/tello/heartbeat":
                if not self.tello_map:
                    self.json_response({"error": "tello_mapping_input_disabled"}, 403)
                    return
                self.json_response(self.tello_map.heartbeat(json.loads(data)["session_id"]))
            elif not self.live:
                self.json_response({"error": "camera_input_disabled"}, 403)
            elif url.path == "/api/live/start":
                self.json_response(self.live.start(self.profile_config(json.loads(data), 'camera')), 201)
            elif url.path == "/api/live/frame":
                query = parse_qs(url.query)
                session = self.live.require(query.get("session", [""])[0])
                if session.stats()["input"] != "browser_camera":
                    self.json_response({"error": "input_is_owned_by_tello_adapter"}, 409)
                    return
                self.json_response(session.offer(data, float(self.headers["X-Frame-Timestamp"]), int(self.headers["X-Frame-Sequence"])), 202)
            elif url.path == "/api/live/event":
                body = json.loads(data)
                session = self.live.require(body['session_id'])
                session.record_event(body['event'], body.get('message', ''))
                self.json_response({'recorded': True}, 202)
            elif url.path == "/api/live/stop":
                body = json.loads(data)
                reason = body.get('reason', 'user_stop')
                if reason not in {'user_stop', 'page_hidden', 'client_error', 'browser_media_ended', 'start_cancelled'}:
                    raise ValueError('invalid_stop_reason')
                session = self.live.require(body["session_id"])
                if body.get('error'):
                    session.record_event('client_error', body['error'])
                session.stop(reason)
                self.json_response({**session.stats(), "status": "stopping"}, 202)
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
        if args and re.search(r"/api/(?:(?:saved|live)/)?(?:state|run|timeline|frame|tracking|slam-preview|tello-preview|tello/heartbeat)", str(args[0])):
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
        self.send_header("Content-Type", mimetypes.guess_type(self.demo.name)[0] or 'application/octet-stream')
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
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
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

    def profile_config(self, body, mode, size=None):
        selection = body.get('profile_selection')
        if selection is None:
            return body
        if not self.profiles:
            raise ValueError('camera_profiles_not_enabled')
        width, height = size or ((960, 720) if mode == 'tello' else (body.get('source_width'), body.get('source_height')))
        camera = self.profiles.resolve(selection, mode, width, height)
        return {**body, 'camera': camera}

    def profile_request(self):
        if not self.profiles:
            self.json_response({'error': 'camera_profiles_not_enabled'}, 404)
            return
        try:
            url = urlsplit(self.path)
            if self.command == 'GET':
                if url.path == '/api/camera-profiles':
                    self.json_response(self.profiles.list())
                elif url.path == '/api/camera-profiles/export':
                    key = parse_qs(url.query).get('id', [''])[0]
                    camera = self.profiles.get(key)
                    payload = json.dumps(camera, ensure_ascii=False, indent=2, allow_nan=False).encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json; charset=utf-8')
                    self.send_header('Content-Disposition', f'attachment; filename="{key}-camera.json"')
                    self.send_header('Content-Length', str(len(payload)))
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    self.write_payload(payload)
                else:
                    self.send_error(404)
                return
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 131072:
                raise ValueError('camera_profile_json_too_large')
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError('invalid_profile_request')
            if url.path == '/api/camera-profiles':
                self.json_response(self.profiles.register(body), 201)
            elif url.path == '/api/camera-profiles/default':
                self.json_response(self.profiles.set_default(body['mode'], body.get('id')))
            else:
                self.send_error(404)
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.json_response({'error': str(error)}, 400)

    def import_request(self):
        if not self.imports:
            self.json_response({'error': 'imports_not_enabled'}, 404)
            return
        url = urlsplit(self.path)
        query = parse_qs(url.query)
        action = url.path.removeprefix('/api/import/')
        key = query.get('id', [''])[0]
        try:
            if self.command == 'GET':
                if action == 'library':
                    self.json_response({'items': self.imports.library(), 'initial': getattr(self.imports, 'initial', None)})
                elif action == 'status':
                    self.json_response(self.imports.status(key))
                elif action == 'media':
                    record = self.imports.resolve(key)
                    index = int(query.get('part', ['0'])[0])
                    if not 0 <= index < len(record.media):
                        raise ValueError('unknown_recording')
                    self.demo = inside(record.root, record.media[index]['path'])
                    self.source_video()
                else:
                    self.send_error(404)
                return
            if action == 'upload':
                self.imports.upload(key, query.get('path', [''])[0], self.rfile, int(self.headers.get('Content-Length', '-1')))
                self.json_response({'status': 'uploaded'})
                return
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 <= length <= 65536:
                raise ValueError('invalid_request_size')
            body = json.loads(self.rfile.read(length) or b'{}')
            if action == 'create':
                value = self.imports.create(body.get('name', '불러온 자료'))
            elif action == 'inspect':
                value = self.imports.inspect(key)
            elif action == 'local':
                value = self.imports.open_local(body.get('local_id', ''))
            elif action == 'start':
                if self.current and self.current.stats().get('session_status') not in {'complete', 'failed', 'stopped_or_disconnected'}:
                    raise ValueError('stop_live_mapping_before_reconstruction')
                summary = self.imports.job(key).get('summary', {})
                body = self.profile_config(body, 'file', (summary.get('width'), summary.get('height')))
                value = self.imports.start(key, body)
            elif action == 'cancel':
                value = self.imports.cancel(key)
            else:
                self.send_error(404)
                return
            self.json_response(value, 202)
        except (OSError, ValueError, KeyError, TypeError, zipfile.BadZipFile) as error:
            self.close_connection = True
            self.json_response({'error': str(error)[:400]}, 400)

    def do_GET(self):
        host = self.headers.get("Host", "").split(":", 1)[0]
        if host not in {"127.0.0.1", "localhost"}:
            self.send_error(403)
            return
        url = urlsplit(self.path)
        path = unquote(url.path)
        if path.startswith('/api/camera-profiles'):
            self.profile_request()
            return
        if path.startswith('/api/import/'):
            self.import_request()
            return
        asset = parse_qs(url.query).get('asset', [None])[0]
        if asset and self.imports and path.startswith('/api/saved/'):
            try:
                self.imported = self.imports.resolve(asset)
            except (OSError, ValueError, KeyError, TypeError) as error:
                self.json_response({'error': str(error)}, 404)
                return
        if path in {'/api/live/session', '/api/live/session-download'}:
            session = self.current.path if self.current else self.recorded_session
            if session is None:
                self.json_response({'error': 'no_live_session'}, 404)
                return
            requested = parse_qs(url.query).get('session', [None])[0]
            if requested != session.name:
                self.json_response({'error': 'camera_session_mismatch'}, 409)
                return
            try:
                recorder = getattr(self.current, 'recording', None) if self.current else None
                manifest = recorder.snapshot() if recorder else load_json(session / 'session.json')
            except (OSError, ValueError):
                self.json_response({'error': 'session_recording_unavailable'}, 404)
                return
            if (not isinstance(manifest, dict) or manifest.get('schema_version') != 'mapping-session/1.0'
                    or not isinstance(manifest.get('recording'), dict)):
                self.json_response({'error': 'invalid_session_manifest'}, 400)
                return
            if path == '/api/live/session':
                self.json_response(manifest)
                return
            if (manifest['recording'].get('finalized') is not True or
                    (self.current and not self.current.done)):
                self.json_response({'error': 'session_save_in_progress'}, 409)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'application/zip')
            name = session.name if re.fullmatch(r'[a-zA-Z0-9_-]{1,80}', session.name) else 'mapping-session'
            self.send_header('Content-Disposition', f'attachment; filename="{name}.zip"')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            self.close_connection = True
            try:
                # Stream the completed folder with bounded memory; no second on-disk copy.
                with zipfile.ZipFile(self.wfile, 'w', compression=zipfile.ZIP_STORED) as archive:
                    for item, relative in archive_files(session):
                        if relative != 'session.json':
                            archive.write(item, name + '/' + relative)
                    archive.writestr(name + '/session.json', json.dumps(manifest, ensure_ascii=False))
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                pass
            return
        if path == '/api/live/tracking' and not self.current:
            if self.recorded_session:
                requested = parse_qs(url.query).get('session', [None])[0]
                if requested and requested != self.recorded_session.name:
                    self.json_response({'error': 'camera_session_mismatch'}, 409)
                    return
                progress_path = self.recorded_session / 'tracking.json'
                try:
                    progress = load_json(progress_path) if progress_path.is_file() else {}
                    state = load_json(self.recorded_session / 'state.json')
                except (OSError, ValueError):
                    self.json_response({'source': 'live', 'error': 'source_unavailable'}, 503)
                    return
                self.json_response({**{k: v for k, v in progress.items() if k not in {'features', 'T_cw'}},
                    **state, 'source': 'live', 'session_id': self.recorded_session.name,
                    'preview_ready': (self.recorded_session / 'preview.jpg').is_file()})
            else:
                self.json_response({'source': 'live', 'status': 'idle', 'session_status': 'idle', 'preview_ready': False})
            return
        if path == '/api/live/tracking' and self.current:
            requested_session = parse_qs(url.query).get('session', [None])[0]
            if requested_session and requested_session != self.current.id:
                self.json_response({'error': 'camera_session_mismatch'}, 409)
                return
            with self.current.condition:
                progress = dict(getattr(self.current, 'preview_progress', {}))
            stats = self.current.stats()
            self.json_response({**progress, **stats, 'status': stats['session_status'],
                'preview_ready': bool(progress.get('frame_id'))})
            return
        if path == '/api/live/slam-preview' and self.current and hasattr(self.current, 'previews'):
            query = parse_qs(url.query)
            if query.get('session', [''])[0] != self.current.id:
                self.json_response({'error': 'camera_session_mismatch'}, 409)
                return
            frame = query.get('frame', [''])[0]
            with self.current.condition:
                payload = self.current.previews.get(frame)
            if payload is None:
                self.send_error(404, 'Tracking preview expired or not ready')
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Frame-Id', frame)
            self.end_headers()
            self.write_payload(payload)
            return
        if path == "/api/live/tello-preview.jpg":
            session = parse_qs(url.query).get("session", [""])[0]
            if not self.tello_map or not self.live or not self.live.current or self.live.current.id != session:
                self.json_response({"error": "camera_session_mismatch"}, 409)
                return
            image = self.tello_map.latest(session)
            if image is None:
                self.send_error(404, "No fresh Tello frame")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(image)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.write_payload(image)
            return
        if path == "/api/tello":
            self.json_response(self.tello.stats() if self.tello else {"enabled": False, "running": False})
            return
        if path == "/api/tello/video.mjpg":
            if not self.tello or not self.tello.running:
                self.send_error(503, "Tello stream is not running")
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            stamp = None
            try:
                while self.tello.running:
                    frame, next_stamp = self.tello.latest(stamp)
                    if frame is None or next_stamp == stamp:
                        continue
                    stamp = next_stamp
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                     str(len(frame)).encode("ascii") + b"\r\n\r\n" + frame + b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            return
        if path == "/api/source-video":
            self.source_video()
            return
        if path == "/api/live":
            self.json_response({"enabled": self.live is not None, "demo_available": self.demo is not None,
                                "tello_map_enabled": self.tello_map is not None,
                                "tello_map_mode": ("gateway" if self.tello_map.gateway else "direct") if self.tello_map else None,
                                "tello_camera": self.tello_map.camera if self.tello_map else None,
                "demo_camera": self.demo_camera, "current": self.current.stats() if self.current else None})
            return
        if path.startswith("/api/saved/"):
            self.select_source("saved")
            path = "/api/" + path.removeprefix("/api/saved/")
        elif path.startswith("/api/live/"):
            self.select_source("live")
            path = "/api/" + path.removeprefix("/api/live/")
        try:
            requested = parse_qs(url.query).get("revision", [None])[0]
            requested_session = parse_qs(url.query).get("session", [None])[0]
            identity = self.imported.state.get('session_id') if self.imported else self.session.name if self.session else None
            if requested_session and requested_session != identity:
                self.json_response({"error": "session_changed"}, 409)
                return
            if requested and not re.fullmatch(r"\d{6}", requested):
                self.send_error(404)
                return
            if requested and self.source == "saved" and not self.session:
                self.json_response({"error": "saved_map_has_no_revisions"}, 400)
                return
            result, state = self.snapshot()
            current_result = result
            if requested:
                result, _ = self.snapshot(requested, state)
            state = self.source_state(current_result, state)
        except (OSError, ValueError) as error:
            self.log_error("%s source read failed: %s", self.source, error)
            self.json_response({"status": "unavailable", "source": self.source, "error": "source_unavailable"}, 503)
            return
        if path == "/api/state":
            self.json_response(state)
            return
        if path == "/api/timeline":
            if self.source == 'saved' and self.imported:
                self.json_response({'schema_version': 'orb-map-timeline/1.0', 'entries': self.imported.entries,
                    'source': 'saved', 'session_id': self.imported.state.get('session_id')})
                return
            self.json_response({**self.timeline.read(self.session, state), "source": self.source,
                                "session_id": self.session.name if self.session else None})
            return
        if path == "/api/run":
            if result is None or not (result / "status.json").is_file() or json.loads(
                    (result / "status.json").read_text(encoding="utf-8")).get("status") not in {"complete", "partial"}:
                self.json_response(state or {"status": "incomplete"}, 503)
                return
            value = json.loads((result / "scene.json").read_text(encoding="utf-8"))
            value.pop("surface", None)  # Read old results without exposing the retired mesh.
            value.pop("stream", None)
            value["source"] = self.source
            if not value.get("point_observations") and self.observations and not self.imported:
                evidence = self.observations.read(self.session, value)
                if evidence:
                    value["point_observations"] = evidence
            if self.source == "live" or (self.imported and self.imported.is_session):
                value["stream"] = state
            try:
                collection = self.object_collection(result, value)
            except (OSError, ValueError):
                self.json_response({"status": "objects_unavailable_or_mismatched"}, 503)
                return
            if collection:
                value["planar_objects"] = collection
            since = parse_qs(url.query).get("since_revision", [None])[0]
            if since is not None:
                if not re.fullmatch(r"\d{6}", since):
                    self.send_error(404)
                    return
                if self.session:
                    try:
                        base, _ = self.snapshot(since)
                        previous = json.loads((base / "scene.json").read_text(encoding="utf-8"))
                        delta = scene_delta(previous, value)
                        if delta is not None:
                            delta_payload = json.dumps(delta, ensure_ascii=False, allow_nan=False).encode("utf-8")
                            full_payload = json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8")
                            self.json_payload(delta_payload if len(delta_payload) < len(full_payload) else full_payload)
                            return
                    except (OSError, ValueError, KeyError, TypeError):
                        pass  # Missing/pruned/unsupported bases always fall back to a complete snapshot.
            self.json_response(value)
            return
        static = {"/": APP / "index.html", "/app.js": APP / "app.js", "/import-view.js": APP / 'import-view.js', "/objects.js": APP / "objects.js", "/capture.js": APP / "capture.js", "/recording-view.js": APP / "recording-view.js", "/point-grid.js": APP / "point-grid.js", "/point-surfaces.js": APP / "point-surfaces.js", "/style.css": APP / "style.css",
                  "/point-surface-worker.js": APP / "point-surface-worker.js", "/map-delta.js": APP / "map-delta.js",
                  "/map-source.js": APP / "map-source.js", "/wide-surfaces.js": APP / "wide-surfaces.js",
                  "/frame-latency.js": APP / "frame-latency.js",
                  "/point-display.js": APP / "point-display.js", "/point-worker-state.js": APP / "point-worker-state.js",
                  "/point-spacing-worker.js": APP / "point-spacing-worker.js",
                  "/tracking-preview.js": APP / "tracking-preview.js",
                  '/profile-view.js': APP / 'profile-view.js',
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
            if not re.fullmatch(r"object-\d{4,8}\.jpg", name):
                self.send_error(404)
                return
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
            root = self.imported.image_root(result) if self.imported and self.source == 'saved' else Path(assets["images_root"]).resolve()
            if root is None:
                self.send_error(404)
                return
            target = (root / name).resolve()
            if target.parent != root:
                self.send_error(404)
                return
        if target is None or not target.is_file():
            self.send_error(404)
            return
        if self.imported and path.startswith('/api/'):
            try:
                target = inside(self.imported.root, target.relative_to(self.imported.root).as_posix())
            except ValueError:
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
        self.write_payload(payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, help="Saved reconstruction exposed through /api/saved")
    parser.add_argument("--session", type=Path, help="Initial online session exposed through /api/live")
    parser.add_argument('--import-source', type=Path, help='Explicit local ZIP, session, video or frames to inspect in the saved tab')
    parser.add_argument("--tello-only", action="store_true", help="Open the console for Tello person detection without a map")
    parser.add_argument("--objects", type=Path, help="Optional planar object bundle for an offline result")
    parser.add_argument("--port", type=int, default=8790)
    parser.add_argument("--enable-camera", action="store_true", help="Enable local browser camera -> Docker ORB-SLAM3")
    parser.add_argument("--enable-tello-map", action="store_true", help="Enable Tello camera -> live 3D map (no flight commands)")
    parser.add_argument("--tello-gateway", help="Share video from a fixed loopback HTTP gateway, e.g. http://127.0.0.1:8010")
    parser.add_argument('--tello-camera', type=Path, help='Override measured Tello camera JSON for another camera/crop')
    parser.add_argument("--enable-tello", action="store_true", help="Enable Tello/RoboMaster TT video and person detection")
    parser.add_argument("--tello-flight", action="store_true", help="Enable guarded manual flight controls in the Tello detection view")
    parser.add_argument("--tello-model", default="yolo26n-seg.pt",
                        help="Ultralytics person-segmentation model name or local .pt path")
    parser.add_argument("--tello-confidence", type=float, default=.25)
    parser.add_argument("--live-root", type=Path, default=APP.parents[1] / "output/phone_mapping")
    parser.add_argument('--profiles-root', type=Path, default=APP.parents[1] / 'output/camera_profiles')
    parser.add_argument("--demo-source", type=Path, help="One local MP4 explicitly exposed for streaming demonstration")
    calibration = parser.add_mutually_exclusive_group()
    calibration.add_argument("--demo-camera", type=Path, help="Intrinsics JSON for this exact demo camera/crop")
    calibration.add_argument("--demo-calibration-result", type=Path, help="Reuse only intrinsics from this video's saved reconstruction")
    args = parser.parse_args()
    result = args.result.resolve() if args.result else None
    if not (args.result or args.session or args.import_source or args.enable_camera or args.tello_only or args.enable_tello_map or args.tello_gateway):
        parser.error("Select --result, --session, --enable-camera, --enable-tello-map, or --tello-only")
    if args.objects and not args.result:
        parser.error("--objects requires a saved --result")
    if result and (not (result / "status.json").is_file() or json.loads(
            (result / "status.json").read_text(encoding="utf-8")).get("status") not in {"complete", "partial"}):
        parser.error("Select a complete or partial reconstruction result")
    if not (APP / "node_modules/three/build/three.module.js").is_file():
        parser.error("Install apps/console dependencies first: pnpm install --frozen-lockfile")
    live = None
    if args.enable_camera or args.enable_tello_map or args.tello_gateway:
        from modules.mapping.localization.orbslam3_live import LiveManager
        live = LiveManager(args.live_root)
    tello_map = None
    if args.enable_tello_map or args.tello_gateway:
        from modules.mapping.localization.tello_input import TelloMapInput
        try:
            tello_map = TelloMapInput(live, args.tello_gateway,
                camera=json.loads(args.tello_camera.read_text(encoding='utf-8')) if args.tello_camera else None)
        except ValueError as error:
            parser.error(str(error))
    tello = None
    if args.enable_tello or args.tello_only:
        from modules.perception.inference.tello_person_stream import TelloPersonStream
        tello = TelloPersonStream(args.tello_model, args.tello_confidence, flight_enabled=args.tello_flight)
    demo = args.demo_source.resolve() if args.demo_source else None
    if demo and (not demo.is_file() or demo.suffix.lower() != ".mp4"):
        parser.error("Demo source must be a local MP4")
    if (demo and not args.enable_camera) or ((args.demo_camera or args.demo_calibration_result) and not demo):
        parser.error("Demo calibration requires --demo-source and --enable-camera")
    demo_camera = None
    if demo:
        try:
            demo_camera = prepare_demo_camera(demo, camera=args.demo_camera,
                calibration_result=args.demo_calibration_result, session=args.session)
        except (OSError, ValueError, KeyError, TypeError) as error:
            parser.error(str(error))
    imports = ImportManager(args.live_root)
    profiles = CameraProfiles(args.profiles_root)
    if args.import_source:
        initial = imports.add_local(args.import_source)
        imports.initial = initial
        print(f'Import selected: {initial}', flush=True)
    server = ViewerServer(("127.0.0.1", args.port), partial(ViewerHandler, result=result, imports=imports, profiles=profiles,
        session=args.session, objects=args.objects, live=live, tello=tello, tello_map=tello_map, timeline=Timeline(), observations=ObservationCache(),
        demo=demo, demo_camera=demo_camera))
    print(f"Phone mapping viewer: http://127.0.0.1:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        imports.shutdown()
        if tello_map:
            tello_map.shutdown()
        if live:
            live.shutdown()
        if tello and tello.running:
            tello.stop()


if __name__ == "__main__":
    main()
