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
from modules.mapping.data.tof_depth import FRONT_TOF_MIN_M, FRONT_TOF_MAX_M
from modules.autonomy.tello_link.sensor_pose import SensorPose

DEFAULT_CAMERA = Path(__file__).resolve().parents[3] / 'data/mapping/tello/20261028/v1/camera.json'
FRONT_TOF_MAX_FRAME_GAP_S = .2


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


def mjpeg_frames(response, cancelled, *, with_pose=False, with_metadata=False):
    """Read bounded JPEGs from both existing gateway MJPEG variants (no length header)."""
    buffer = bytearray()
    pose = None
    metadata = None
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
                if with_pose or with_metadata:
                    for header in buffer[:start].split(b'\r\n'):
                        if header.lower().startswith(b'x-tello-pose:') and len(header) <= 512:
                            try:
                                pose = SensorPose.from_values(json.loads(header.split(b':', 1)[1]))
                            except (ValueError, RecursionError):
                                pose = None
                        elif header.lower().startswith(b'x-tello-frame:') and len(header) <= 1024:
                            try:
                                value = json.loads(header.split(b':', 1)[1])
                                metadata = value if isinstance(value, dict) else None
                            except (ValueError, RecursionError):
                                metadata = None
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
            yield (jpeg, pose, metadata) if with_metadata else (jpeg, pose) if with_pose else jpeg
            pose = None
            metadata = None


class DirectCamera:
    time_basis = "pc_decode_receive_monotonic"
    source_label = "tello_sdk_camera"
    sensor_capable = True

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
        self.front_tof = None
        self.sensor_pose = None
        self.frame_metadata = None
        self.tof_stop = threading.Event()
        self.tof_thread = None

    def set_recording(self, recording):
        self.recording = recording
        if recording:
            recording.require_sensors()

    def open(self, cancelled):
        self.link = self.link_factory(allow_flight=False)
        self.link.start()
        self.run_id = self.link.run_id
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
        if not cancelled():
            self.tof_stop.clear()
            self.tof_thread = threading.Thread(target=self._poll_front_tof, daemon=True, name='mapping-front-tof')
            self.tof_thread.start()

    def _poll_front_tof(self):
        from modules.autonomy.tello_link.tello_link import parse_ext_tof

        while not self.tof_stop.is_set():
            started = time.monotonic()
            try:
                response = self.link.send('EXT tof?', timeout=.25, source='mapping').response
            except OSError:
                response = None
            finished = time.monotonic()
            reading = parse_ext_tof(response)
            self.front_tof = ((reading['front_tof_m'], (started + finished) / 2)
                if reading['valid'] and finished - started <= FRONT_TOF_MAX_FRAME_GAP_S else None)
            interval = .2 if reading['valid'] or reading['reason'] == 'out_of_range' else 2.
            self.tof_stop.wait(max(0., interval - (finished - started)))

    def frames(self, cancelled):
        last, last_new = None, time.monotonic()
        while not cancelled():
            self._record_sensors()
            snapshot = self.video.jpeg_snapshot()
            if snapshot and snapshot[1] != last and time.monotonic() - snapshot[1] < 1:
                jpeg, last, _ = snapshot
                last_new = time.monotonic()
                self.sensor_pose = self.link.sensor_pose_at(last)
                self.frame_metadata = {'run_id': self.run_id, 'timestamp_s': last, 'wall': snapshot[2],
                    'recording_part': self.recording_sink.part if self.recording_sink else None,
                    'video_match': 'h264_receive_time_approximation'}
                yield jpeg, last
            elif time.monotonic() - last_new > 5:
                raise InputError("tello_video_timeout_check_wifi_and_udp")
            elif time.monotonic() - last_new > .75:
                yield None, time.monotonic()
                time.sleep(.05)
            else:
                time.sleep(.005)

    def close(self):
        self.tof_stop.set()
        if self.tof_thread:
            self.tof_thread.join()
            self.tof_thread = None
        self.front_tof = None
        self.sensor_pose = None
        try:
            if self.link and self.stream_requested:
                self.link.send("streamoff", timeout=1, source="mapping")
        finally:
            try:
                if self.video:
                    self.video.stop()
            finally:
                self._record_sensors()
                if self.recording_sink:
                    self.recording_sink.close(self.video.stats() if self.video else None)
                    self.recording_sink = None
                if self.link:
                    self.link.close()
                self.link = self.video = None
                self.stream_requested = False

    def _record_sensors(self):
        if self.recording and self.link and hasattr(self.link, 'sensor_samples'):
            cursor = self.recording.sensor_cursors.get(self.link.run_id, 0)
            self.recording.sensor_batch(self.link.sensor_samples(cursor))


class GatewayCamera:
    time_basis = "mapping_bridge_receive_monotonic"
    sensor_capable = True

    def __init__(self, address):
        self.address = gateway_address(address)
        self.opener = build_opener(ProxyHandler({}), NoRedirect())
        self.response = None
        self.state_path = "/api/drone/state"
        self.checked = -math.inf
        self.run_id = None
        self.source_label = "gateway_unspecified"
        self.recording = self.recording_sink = None
        self.front_tof = None
        self.sensor_pose = None
        self.allow_visual_origin = True
        self.frame_metadata = None
        self.recording_frames = 0

    def set_recording(self, recording):
        self.recording = recording
        if recording:
            recording.require_sensors()

    def check(self):
        started = time.perf_counter()
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
        if self.run_id is not None and run is not None and run != self.run_id:
            raise InputError("tello_gateway_session_changed")
        if isinstance(run, str) and run:
            self.run_id = run
        self.checked = time.monotonic()
        mode = state.get("source", state.get("mode"))
        self.source_label = "gateway_reported_fake" if mode == "fake" else "gateway_reported_tello" if mode == "tello" else "gateway_unspecified"
        tof = state.get('front_tof')
        self.front_tof = None
        if isinstance(tof, dict) and tof.get('valid') is True:
            age, distance = tof.get('age_s'), tof.get('m')
            if (type(age) in (int, float) and type(distance) in (int, float)
                    and 0 <= age <= FRONT_TOF_MAX_FRAME_GAP_S - (time.perf_counter() - started)
                    and FRONT_TOF_MIN_M <= distance < FRONT_TOF_MAX_M):
                # Both clocks are local. Subtract age from request start to
                # conservatively account for the HTTP round trip.
                self.front_tof = (distance, started - age)

    def open(self, cancelled):
        self.check()
        if cancelled():
            return
        self.response = self.opener.open(Request(self.address + "/video.mjpg?mapping=1"), timeout=2)
        if "multipart/x-mixed-replace" not in self.response.headers.get("Content-Type", ""):
            raise InputError("tello_gateway_invalid_content_type")
        self.allow_visual_origin = self.response.headers.get("X-Tello-Sensor-Pose") != "1"
        if self.recording:
            self.recording_sink = self.recording.open_part('received_jpeg', self.time_basis)
            self.recording_frames = 0
            self._record_sensors()

    def frames(self, cancelled):
        for jpeg, pose, metadata in mjpeg_frames(self.response, cancelled, with_metadata=True):
            # Windows monotonic() can have a coarse tick. Several JPEGs may arrive
            # in one socket read, so use the high-resolution monotonic counter.
            stamp = time.perf_counter()
            if self.recording_sink:
                recorded = self.recording_sink.offer(jpeg, stamp)
            else:
                recorded = False
            self.frame_metadata = {**(metadata or {}), 'recording_part': self.recording_sink.part if self.recording_sink else None,
                'recording_frame_index': self.recording_frames, 'video_recorded': recorded,
                'video_match': 'received_jpeg_index', 'receive_timestamp_s': stamp}
            self.recording_frames += 1
            if time.monotonic() - self.checked >= .25:
                self.check()
                self._record_sensors()
            self.sensor_pose = pose
            yield jpeg, stamp

    def close(self):
        self.front_tof = None
        self.sensor_pose = None
        if self.response:
            self.response.close()
            self.response = None
        if self.recording_sink:
            self._record_sensors()
            self.recording_sink.close()
            self.recording_sink = None

    def _record_sensors(self):
        if not self.recording:
            return
        cursor = self.recording.sensor_cursors.get(self.run_id, 0)
        try:
            with self.opener.open(self.address + '/api/drone/sensors?after=' + str(cursor), timeout=.5) as response:
                data = response.read(2097153)
            if len(data) > 2097152:
                raise ValueError('sensor_batch_too_large')
            batch = json.loads(data)
            run_id = batch.get('run_id')
            if not isinstance(run_id, str) or not run_id:
                raise ValueError('sensor_run_missing')
            if self.run_id is not None and run_id != self.run_id:
                raise ValueError('sensor_run_changed')
            self.run_id = run_id
            self.recording.sensor_batch(batch)
        except (OSError, ValueError, KeyError, TypeError, RecursionError):
            self.recording.log_problem('recording_gateway_sensors_unavailable')


class TelloMapInput:
    def __init__(self, live, gateway=None, source_factory=None, *, camera=None, recovery_seconds=30):
        self.live, self.gateway = live, gateway_address(gateway) if gateway else None
        self.source_factory = source_factory or (lambda: GatewayCamera(self.gateway) if self.gateway else DirectCamera())
        self.lock = threading.Lock()
        self.worker = self.session = None
        self.active_source = None
        self.preview = None
        self.lease_until = 0
        self.camera = json.loads(DEFAULT_CAMERA.read_text(encoding='utf-8')) if camera is None else camera
        self.recovery_seconds = recovery_seconds

    @property
    def busy(self):
        return bool(self.worker and self.worker.is_alive())

    def start(self, config, *, image_id=None):
        # The current Tello receiver has one fixed full-frame 960x720 camera mode.
        config = {**config, "camera": self.camera if config.get('camera') is None else config['camera'],
                  "width": 960, "height": 720, "fps": 24}
        live_camera(config)  # reject bad calibration before any SDK or Docker action
        with self.lock:
            if self.worker and self.worker.is_alive():
                raise InputError("tello_input_cleanup_pending")
            source = self.source_factory()
            state = self.live.start(config, input_kind="tello_camera", timestamp_basis=source.time_basis,
                                    image_id=image_id)
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
        connection = 0
        session.input_state = 'connecting'
        try:
            while not cancelled():
                try:
                    if hasattr(source, 'set_recording'):
                        source.set_recording(getattr(session, 'recording', None))
                    source.open(cancelled)
                    self.active_source = source
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
                        if hasattr(session, 'note_input_frame'):
                            session.note_input_frame()
                        if missing_since is not None:
                            session.reconnects = getattr(session, 'reconnects', 0) + 1
                        missing_since = None
                        session.input_state, session.input_error = 'receiving', None
                        frame_sequence = sequence
                        sequence += 1
                        pose = getattr(source, 'sensor_pose', None)
                        sensor_pose = ((getattr(source, 'run_id', None) or connection,
                                        pose if isinstance(pose, SensorPose) else None,
                                        getattr(source, 'allow_visual_origin', False))
                                       if isinstance(pose, SensorPose) or getattr(source, 'sensor_capable', False) else None)
                        admission = ('model_wait' if not session.ready else 'rate_limited'
                                     if not getattr(session, 'can_accept_frame', True) else 'submitted')
                        if base is None and admission == 'submitted':
                            base = stamp
                            if getattr(session, 'recording', None):
                                session.recording.set_mapping_origin(base)
                        if getattr(session, 'recording', None):
                            session.recording.frame_input(frame_sequence, stamp - base if base is not None else None,
                                admission=admission, sensor_pose=sensor_pose,
                                front_tof=getattr(source, 'front_tof', None),
                                metadata={**(getattr(source, 'frame_metadata', None) or {}),
                                          'connection': connection, 'receive_timestamp_s': stamp})
                        if not session.ready:
                            if hasattr(session, 'note_skipped_frame'):
                                session.note_skipped_frame()
                            continue
                        if not getattr(session, 'can_accept_frame', True):
                            session.note_skipped_frame()
                            continue
                        sample = getattr(source, 'front_tof', None)
                        sensors = {}
                        if (sample is not None and abs(sample[1] - stamp) <= FRONT_TOF_MAX_FRAME_GAP_S
                                and FRONT_TOF_MIN_M <= sample[0] < FRONT_TOF_MAX_M):
                            sensors['front_tof'] = (sample[0], sample[1] - base)
                        if sensor_pose is not None:
                            sensors['sensor_pose'] = sensor_pose
                        session.offer(jpeg, stamp - base, frame_sequence, **sensors)
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
                    self.active_source = None
                    source.close()
                # Cancelable backoff; never create a second receiver while the old one owns ports.
                until = time.monotonic() + .25
                while not cancelled() and time.monotonic() < until:
                    time.sleep(.025)
                source = self.source_factory()
                connection += 1
        except Exception as error:
            if not cancelled():
                session.input_state = 'failed'
                session.error = str(error) if isinstance(error, InputError) else "tello_input_unavailable: " + type(error).__name__
                session.stop("tello_input_disconnected")
        finally:
            self.active_source = None
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
