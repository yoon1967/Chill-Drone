"""Tello camera -> existing bounded SLAM session; no flight commands.

Direct mode owns the SDK/video sockets. Gateway mode only reads a fixed loopback
gateway and never binds the drone's UDP ports or changes its stream/control state.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

from .orbslam3_live import live_camera
from .orbslam3 import atomic_json, read_json

DEFAULT_CAMERA = Path(__file__).resolve().parents[3] / 'data/mapping/tello/20261028/v1/camera.json'


def retryable(error):
    if isinstance(error, HTTPError):
        return error.code in {408, 429, 500, 502, 503, 504}
    return isinstance(error, (OSError, URLError)) or str(error) in {
        'tello_video_timeout_check_wifi_and_udp', 'tello_sdk_unavailable_check_wifi',
        'tello_streamon_failed', 'tello_gateway_video_off_or_stale',
        'tello_gateway_stream_ended', 'tello_gateway_session_changed'}


class InputError(RuntimeError):
    pass


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def gateway_address(value):
    parsed = urlsplit(value)
    if (parsed.scheme != "http" or parsed.hostname not in {"localhost", "127.0.0.1"}
            or parsed.username or parsed.password or parsed.path not in {"", "/"}
            or parsed.query or parsed.fragment or not parsed.port):
        raise ValueError("tello_gateway_requires_loopback_http_origin_with_port")
    return f"http://127.0.0.1:{parsed.port}"


def mjpeg_frames(response, cancelled):
    """Read bounded JPEGs from both existing gateway MJPEG variants (no length header)."""
    buffer = bytearray()
    while not cancelled():
        chunk = response.read1(16384)
        if not chunk:
            raise InputError("tello_gateway_stream_ended")
        buffer.extend(chunk)
        while True:
            start = buffer.find(b"\xff\xd8")
            if start < 0:
                if len(buffer) > 16384:
                    raise InputError("tello_gateway_invalid_mjpeg")
                break
            if start:
                del buffer[:start]
            end = buffer.find(b"\xff\xd9", 2)
            if end < 0:
                if len(buffer) > 2097152:
                    raise InputError("tello_gateway_frame_too_large")
                break
            if end + 2 > 2097152:
                raise InputError("tello_gateway_frame_too_large")
            jpeg = bytes(buffer[:end + 2])
            del buffer[:end + 2]
            yield jpeg


class DirectCamera:
    time_basis = "pc_decode_receive_monotonic"
    source_label = "tello_sdk_camera"

    def __init__(self, link_factory=None, video_factory=None):
        if link_factory is None:
            from modules.autonomy.tello_link.tello_link import TelloLink
            link_factory = TelloLink
        if video_factory is None:
            from modules.autonomy.tello_link.tello_video import TelloVideo
            video_factory = TelloVideo
        self.link_factory, self.video_factory = link_factory, video_factory
        self.link = self.video = None
        self.stream_requested = False
        self.recording = self.recording_sink = None

    def set_recording(self, recording):
        self.recording = recording

    def open(self, cancelled):
        self.link = self.link_factory(allow_flight=False)
        self.link.start()
        if cancelled():
            return
        if not self.link.send("command", timeout=3, source="mapping").ok:
            raise InputError("tello_sdk_unavailable_check_wifi")
        if cancelled():
            return
        self.video = self.video_factory()
        if self.recording:
            self.recording_sink = self.recording.open_part('h264', 'pc_udp_receive_monotonic')
            if self.recording_sink:
                self.video.packet_sink = self.recording_sink.offer
        self.video.start()
        self.stream_requested = True
        if not self.link.send("streamon", timeout=3, source="mapping").ok:
            raise InputError("tello_streamon_failed")

    def frames(self, cancelled):
        last, last_new = None, time.monotonic()
        while not cancelled():
            snapshot = self.video.jpeg_snapshot()
            if snapshot and snapshot[1] != last and time.monotonic() - snapshot[1] < 1:
                jpeg, last, _ = snapshot
                last_new = time.monotonic()
                yield jpeg, last
            elif time.monotonic() - last_new > 5:
                raise InputError("tello_video_timeout_check_wifi_and_udp")
            elif time.monotonic() - last_new > .75:
                yield None, time.monotonic()
                time.sleep(.05)
            else:
                time.sleep(.015)

    def close(self):
        try:
            if self.link and self.stream_requested:
                self.link.send("streamoff", timeout=1, source="mapping")
        finally:
            try:
                if self.video:
                    self.video.stop()
            finally:
                if self.recording_sink:
                    self.recording_sink.close(self.video.stats() if self.video else None)
                    self.recording_sink = None
                if self.link:
                    self.link.close()
                self.link = self.video = None
                self.stream_requested = False


class GatewayCamera:
    time_basis = "mapping_bridge_receive_monotonic"

    def __init__(self, address):
        self.address = gateway_address(address)
        self.opener = build_opener(ProxyHandler({}), NoRedirect())
        self.response = None
        self.state_path = "/api/drone/state"
        self.checked = -math.inf
        self.run_id = None
        self.source_label = "gateway_unspecified"
        self.recording = self.recording_sink = None

    def set_recording(self, recording):
        self.recording = recording

    def check(self):
        try:
            with self.opener.open(self.address + self.state_path, timeout=2) as response:
                data = response.read(65537)
        except HTTPError as error:
            if error.code == 404 and self.state_path == "/api/drone/state":
                self.state_path = "/api/status"
                return self.check()
            raise
        if len(data) > 65536:
            raise InputError("tello_gateway_status_too_large")
        state = json.loads(data)
        video = state.get("video") or {}
        on = video.get("on", state.get("stream_on", False))
        age = video.get("frame_age_s")
        if (not on or video.get("available") is False or not isinstance(age, (int, float))
                or isinstance(age, bool) or not math.isfinite(age) or not 0 <= age < 1):
            raise InputError("tello_gateway_video_off_or_stale")
        run = state.get("run_id")
        if self.run_id is not None and run != self.run_id:
            raise InputError("tello_gateway_session_changed")
        self.run_id, self.checked = run, time.monotonic()
        mode = state.get("source", state.get("mode"))
        self.source_label = "gateway_reported_fake" if mode == "fake" else "gateway_reported_tello" if mode == "tello" else "gateway_unspecified"

    def open(self, cancelled):
        self.check()
        if cancelled():
            return
        self.response = self.opener.open(Request(self.address + "/video.mjpg"), timeout=2)
        if "multipart/x-mixed-replace" not in self.response.headers.get("Content-Type", ""):
            raise InputError("tello_gateway_invalid_content_type")
        if self.recording:
            self.recording_sink = self.recording.open_part('received_jpeg', self.time_basis)

    def frames(self, cancelled):
        for jpeg in mjpeg_frames(self.response, cancelled):
            # Windows monotonic() can have a coarse tick. Several JPEGs may arrive
            # in one socket read, so use the high-resolution monotonic counter.
            stamp = time.perf_counter()
            if self.recording_sink:
                self.recording_sink.offer(jpeg, stamp)
            if time.monotonic() - self.checked >= .25:
                self.check()
            yield jpeg, stamp

    def close(self):
        if self.response:
            self.response.close()
            self.response = None
        if self.recording_sink:
            self.recording_sink.close()
            self.recording_sink = None


class TelloMapInput:
    def __init__(self, live, gateway=None, source_factory=None, *, camera=None, recovery_seconds=30):
        self.live, self.gateway = live, gateway_address(gateway) if gateway else None
        self.source_factory = source_factory or (lambda: GatewayCamera(self.gateway) if self.gateway else DirectCamera())
        self.lock = threading.Lock()
        self.worker = self.session = None
        self.preview = None
        self.lease_until = 0
        self.camera = json.loads(DEFAULT_CAMERA.read_text(encoding='utf-8')) if camera is None else camera
        self.recovery_seconds = recovery_seconds

    @property
    def busy(self):
        return bool(self.worker and self.worker.is_alive())

    def start(self, config):
        # The current Tello receiver has one fixed full-frame 960x720 camera mode.
        config = {**config, "camera": self.camera if config.get('camera') is None else config['camera'],
                  "width": 960, "height": 720, "fps": 24}
        live_camera(config)  # reject bad calibration before any SDK or Docker action
        with self.lock:
            if self.worker and self.worker.is_alive():
                raise InputError("tello_input_cleanup_pending")
            source = self.source_factory()
            state = self.live.start(config, input_kind="tello_camera", timestamp_basis=source.time_basis)
            session = self.live.require(state["session_id"])
            self.session, self.preview = session, None
            self.lease_until = time.monotonic() + 45
            self.worker = threading.Thread(target=self._run, args=(session, source), daemon=True,
                                           name="tello-map-input")
            self.worker.start()
            return {**state, "input_mode": "gateway" if self.gateway else "direct"}

    def heartbeat(self, session_id):
        with self.lock:
            if not self.session or self.session.id != session_id:
                raise InputError("camera_session_mismatch")
            if not self.session.closing and not self.session.done:
                self.lease_until = time.monotonic() + 45
            return {"session_id": session_id}

    def _run(self, session, source):
        def cancelled():
            if session.closing or session.done:
                return True
            if time.monotonic() > self.lease_until:
                session.stop("tello_client_timeout")
                return True
            return False
        base, sequence, last_stamp, missing_since = None, 0, -math.inf, None
        session.input_state = 'connecting'
        try:
            while not cancelled():
                try:
                    if hasattr(source, 'set_recording'):
                        source.set_recording(getattr(session, 'recording', None))
                    source.open(cancelled)
                    if cancelled():
                        break
                    session.input_origin = getattr(source, "source_label", "test_source")
                    settings = session.path / "settings.json"
                    if settings.is_file():
                        atomic_json(settings, {**read_json(settings), "input_origin": session.input_origin,
                            "tello_input_mode": "gateway" if self.gateway else "direct"})
                    for jpeg, stamp in source.frames(cancelled):
                        if cancelled():
                            break
                        if jpeg is None:
                            if missing_since is None:
                                missing_since = time.monotonic()
                            session.input_state = 'reconnecting'
                            if time.monotonic() - missing_since >= self.recovery_seconds:
                                raise InputError('tello_reconnect_timeout')
                            continue
                        if stamp <= last_stamp:
                            continue
                        last_stamp = stamp
                        with self.lock:
                            self.preview = (session.id, jpeg, time.monotonic())
                        if missing_since is not None:
                            session.reconnects = getattr(session, 'reconnects', 0) + 1
                        missing_since = None
                        session.input_state, session.input_error = 'receiving', None
                        if not session.ready:
                            continue
                        if base is None:
                            base = stamp
                            if getattr(session, 'recording', None):
                                session.recording.set_mapping_origin(base)
                        session.offer(jpeg, stamp - base, sequence)
                        sequence += 1
                    if not cancelled():
                        raise InputError('tello_gateway_stream_ended')
                except Exception as error:
                    if cancelled():
                        break
                    if not retryable(error):
                        raise
                    if missing_since is None:
                        missing_since = time.monotonic()
                    session.input_state, session.input_error = 'reconnecting', str(error)
                    if time.monotonic() - missing_since >= self.recovery_seconds:
                        raise InputError('tello_reconnect_timeout') from error
                finally:
                    source.close()
                # Cancelable backoff; never create a second receiver while the old one owns ports.
                until = time.monotonic() + .25
                while not cancelled() and time.monotonic() < until:
                    time.sleep(.025)
                source = self.source_factory()
        except Exception as error:
            if not cancelled():
                session.input_state = 'failed'
                session.error = str(error) if isinstance(error, InputError) else "tello_input_unavailable: " + type(error).__name__
                session.stop("tello_input_disconnected")
        finally:
            try:
                source.close()
            except Exception:
                pass
            if not cancelled():
                session.stop("tello_input_ended")
            if getattr(session, 'recording', None):
                session.recording.finish_input()

    def latest(self, session_id):
        with self.lock:
            value = self.preview
            if not value or value[0] != session_id or time.monotonic() - value[2] > 2:
                return None
            return value[1]

    def shutdown(self):
        if self.session and not self.session.done:
            self.session.stop('server_shutdown')
        if self.worker:
            self.worker.join(timeout=10)
