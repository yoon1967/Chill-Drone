"""Bounded browser JPEG -> Docker ORB-SLAM3 transport; all artifacts stay local."""
from __future__ import annotations

import math
import io
import json
from pathlib import Path
import subprocess
import threading
import time
import uuid

from PIL import Image, UnidentifiedImageError

from .orbslam3 import IMAGE, SessionPublisher, atomic_json, read_json, write_settings, render_preview, camera_matrix, replace_file
from modules.mapping.data.live_recording import SessionRecording


def require_slam_image():
    """Bounded preflight; reconcile a failed tag lookup with the engine's image list.

    Retry once when tag lookup and image listing disagree.
    Never pull/build implicitly: a genuine missing image remains an actionable error.
    """
    inspect = ["docker", "image", "inspect", "--format", "{{.Id}}", IMAGE]
    try:
        try:
            return subprocess.check_output(inspect, stderr=subprocess.DEVNULL, text=True, timeout=4).strip()
        except subprocess.CalledProcessError:
            listed = subprocess.check_output(["docker", "image", "ls", "--no-trunc", "--quiet", IMAGE],
                                             stderr=subprocess.DEVNULL, text=True, timeout=4).strip()
            if not listed:
                raise RuntimeError("slam_image_missing")
            try:
                return subprocess.check_output(inspect, stderr=subprocess.DEVNULL, text=True, timeout=4).strip()
            except subprocess.CalledProcessError:
                raise RuntimeError("slam_image_missing") from None
    except (OSError, subprocess.TimeoutExpired, subprocess.CalledProcessError):
        raise RuntimeError("slam_docker_unavailable") from None


def live_camera(config):
    def number(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("camera_values_must_be_finite_numbers")
        return value

    width, height = number(config["width"]), number(config["height"])
    fps = number(config.get("fps", 30))
    if width != int(width) or height != int(height):
        raise ValueError("camera_dimensions_must_be_integers")
    width, height = int(width), int(height)
    if not (320 <= width <= 960 and 180 <= height <= 960 and 1 <= fps <= 60):
        raise ValueError("camera_dimensions_or_fps_out_of_range")
    supplied = config.get("camera")
    if supplied is not None:
        if not isinstance(supplied, dict):
            raise ValueError("invalid_camera_intrinsics")
        camera = dict(supplied)
        for key in ("width", "height", "fx", "fy", "cx", "cy"):
            camera[key] = number(camera[key])
        if min(camera[k] for k in ("width", "height", "fx", "fy")) <= 0:
            raise ValueError("invalid_camera_intrinsics")
        if not (0 <= camera["cx"] <= camera["width"] and 0 <= camera["cy"] <= camera["height"]):
            raise ValueError("camera_principal_point_out_of_bounds")
        if abs(camera["width"] / camera["height"] - width / height) > .01:
            raise ValueError("camera_crop_or_orientation_mismatch")
        for key in ("fx", "cx"):
            camera[key] *= width / supplied["width"]
        for key in ("fy", "cy"):
            camera[key] *= height / supplied["height"]
        camera.setdefault("calibration_status", "user_supplied; accuracy_not_independently_verified")
    else:
        fov = number(config.get("fov", 60))
        if not 25 <= fov <= 120:
            raise ValueError("horizontal_fov_out_of_range")
        focal = width / (2 * math.tan(math.radians(fov) / 2))
        camera = dict(fx=focal, fy=focal, cx=width / 2, cy=height / 2,
                      calibration_status="fov_estimated; not_calibrated", distortion=[0.] * 4)
    d = camera.get("distortion", [0.] * 4)
    if not isinstance(d, list) or len(d) != 4:
        raise ValueError("invalid_camera_distortion")
    d = [number(value) for value in d]
    if not isinstance(camera.get("calibration_status"), str):
        raise ValueError("invalid_calibration_status")
    camera.update(width=width, height=height, fps=fps, distortion=d)
    return camera


class LiveSession:
    def __init__(self, root, camera, image_id, image=IMAGE, *, input_kind="browser_camera", timestamp_basis="browser_media_time"):
        self.id = "live_" + uuid.uuid4().hex[:16]
        self.path = root / self.id
        self.camera, self.image = camera, image
        self.input_kind, self.timestamp_basis = input_kind, timestamp_basis
        self.condition = threading.Condition()
        self.pending = None
        self.closing = False
        self.done = False
        self.ready = False
        self.reason = None
        self.client_event_count = 0
        self.error = None
        self.last_sequence, self.last_pts, self.ack = -1, -1., -1.
        self.last_input = time.monotonic()
        self.created = self.last_input
        self.received = self.dropped = self.submitted = 0
        self.inflight = None
        self.path.mkdir(parents=True)
        (self.path / "revisions").mkdir()
        atomic_json(self.path / "camera.json", camera)
        atomic_json(self.path / "settings.json", {"backend": "orb_slam3_monocular", "input": input_kind,
            "input_timestamp_basis": timestamp_basis,
            "image": image, "image_id": image_id, "camera": camera, "scale": "arbitrary",
            "frame_policy": "latest_one_pending; one_inflight_until_tracker_ACK", "dense_surface": False})
        write_settings(self.path / "camera.yaml", camera, 1800)
        self.publisher = SessionPublisher(self.path, camera)
        self.publisher.preview_enabled = False
        self.previews = {}
        self.preview_progress = {}
        self.publisher.state["calibration_status"] = camera["calibration_status"]
        self.recording = SessionRecording(self.path, camera, input_kind)
        self.jpeg_recording = self.recording.open_part('received_jpeg', timestamp_basis) if input_kind == 'browser_camera' else None
        self.worker = threading.Thread(target=self._run, daemon=True, name=self.id)
        self.worker.start()

    def stats(self):
        with self.condition:
            lifecycle = ("failed" if self.error else "complete") if self.done else \
                "stopping" if self.closing else "receiving" if self.ready else "starting"
            return {"source": "live", "backend": "orb_slam3_monocular",
                    "session_id": self.id, "session_status": lifecycle,
                    "input": getattr(self, "input_kind", "browser_camera"),
                    "input_origin": getattr(self, "input_origin", "local_camera" if getattr(self, "input_kind", "browser_camera") == "browser_camera" else "not_connected"),
                    "input_state": getattr(self, "input_state", "receiving" if self.ready else "connecting"),
                    "input_error": getattr(self, "input_error", None), "reconnects": getattr(self, "reconnects", 0),
                    "input_age_s": max(0, time.monotonic() - self.last_input) if hasattr(self, 'last_input') else None,
                    "processed_input_timestamp_s": getattr(self, 'processed_pts', None),
                    "receive_to_track_ms": getattr(self, 'receive_to_track_ms', None),
                    "tracker_age_s": max(0, time.monotonic() - self.last_ack_mono) if hasattr(self, 'last_ack_mono') else None,
                    "numerical_rejections": getattr(self, 'numerical_rejections', 0),
                    "last_error": self.error,
                    "preview_error": getattr(self, 'preview_error', None),
                    "input_timestamp_basis": getattr(self, "timestamp_basis", "browser_media_time"), "transport_ready": self.ready,
                    "transport_active": not self.done, "transport_stopping": self.closing,
                    "transport_received": self.received, "transport_submitted": self.submitted,
                    "frame_drops": self.dropped, "queue_depth": int(self.pending is not None),
                    "queue_capacity": 1, "stop_reason": self.reason if self.closing else None,
                    "last_input_timestamp_s": self.last_pts if self.last_pts >= 0 else None,
                    "last_input_sequence": self.last_sequence if self.last_sequence >= 0 else None,
                    "recording": self.recording.stats() if getattr(self, 'recording', None) else None}

    def offer(self, jpeg, pts, sequence):
        if not math.isfinite(pts) or not 0 <= pts <= 86400 or sequence < 0:
            raise ValueError("invalid_frame_timestamp_or_sequence")
        if len(jpeg) > 2097152 or not jpeg.startswith(b"\xff\xd8"):
            raise ValueError("invalid_or_oversized_jpeg")
        # Inspect JPEG headers without decoding pixels a second time on the host.
        try:
            with Image.open(io.BytesIO(jpeg)) as image:
                valid = image.format == 'JPEG' and image.size == (self.camera['width'], self.camera['height'])
        except (UnidentifiedImageError, OSError, ValueError):
            valid = False
        if not valid or not jpeg.endswith(b'\xff\xd9'):
            raise ValueError("frame_dimensions_do_not_match_calibration")
        with self.condition:
            if self.closing or self.done or not self.ready:
                raise RuntimeError("camera_session_not_receiving")
            if pts <= self.last_pts or sequence <= self.last_sequence:
                raise ValueError("frame_timestamp_or_sequence_not_increasing")
            self.dropped += max(0, sequence - self.last_sequence - 1) + int(self.pending is not None)
            self.last_pts, self.last_sequence = pts, sequence
            self.received += 1
            self.last_input = time.monotonic()
            if getattr(self, 'jpeg_recording', None):
                self.jpeg_recording.offer(jpeg, pts)
            self.pending = (jpeg, pts, self.last_input)
            self.condition.notify_all()
        return self.stats()

    def record_event(self, kind, message=""):
        if kind not in {'client_error', 'status_retry', 'status_recovered', 'page_hidden', 'stop_requested'}:
            raise ValueError('invalid_client_event')
        if not isinstance(message, str) or len(message) > 1024:
            raise ValueError('invalid_client_event_message')
        with self.condition:
            if self.client_event_count >= 512:
                return
            event = {'event': kind, 'message': message, 'elapsed_s': time.monotonic() - self.created,
                     'input_timestamp_s': self.last_pts, 'processed_timestamp_s': self.ack}
            with (self.path / 'events.jsonl').open('a', encoding='utf-8') as output:
                output.write(json.dumps(event, ensure_ascii=False) + '\n')
            self.client_event_count += 1

    def stop(self, reason="user_stop"):
        with self.condition:
            # Cleanup and duplicate requests must not overwrite the first cause.
            if self.closing or self.done:
                return
            self.record_event('stop_requested', reason)
            self.closing, self.reason = True, reason
            self.pending = None
            self.condition.notify_all()

    def _write(self, process):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.pending is not None or self.closing or process.poll() is not None, timeout=1)
                    if process.poll() is not None or self.closing:
                        break
                    if self.pending is None:
                        if self.closing:
                            break
                        continue
                    jpeg, pts, accepted = self.pending
                    self.pending = None
                    self.inflight = (pts, accepted)
                packet = memoryview(f"{len(jpeg)} {pts:.17g}\n".encode("ascii") + jpeg)
                while packet:
                    written = process.stdin.write(packet)
                    if not written:
                        raise OSError("tracker_stdin_closed")
                    packet = packet[written:]
                process.stdin.flush()
                with self.condition:
                    self.submitted += 1
                    acknowledged = self.condition.wait_for(lambda: self.ack >= pts or process.poll() is not None or self.closing or self.error, timeout=15)
                    if not acknowledged:
                        raise RuntimeError("tracker_ack_timeout")
        except (OSError, RuntimeError) as error:
            self.error = self.error or str(error)
            self.stop(self.reason if self.closing else "transport_error")
        finally:
            try:
                process.stdin.close()
            except OSError:
                pass

    def _read(self, process, log):
        # The reader owns this file until EOF, including exceptional worker stops.
        # Closing it in the worker's exception unwind races with a final ACK/log.
        try:
            for line in iter(process.stdout.readline, b""):
                log.write(line)
                if line.startswith(b"ORB_ACK "):
                    with self.condition:
                        try:
                            self.ack = float(line.split()[1])
                        except (ValueError, IndexError):
                            continue
                        self.last_ack_mono = time.monotonic()
                        self.processed_pts = self.ack
                        if getattr(self, 'inflight', None) and self.inflight[0] == self.ack:
                            self.receive_to_track_ms = (self.last_ack_mono - self.inflight[1]) * 1000
                        self.condition.notify_all()
                elif b'SSAFY_SIM3_REJECT' in line:
                    self.numerical_rejections = getattr(self, 'numerical_rejections', 0) + 1
                elif b'SO3::exp failed' in line or b'Sophus ensure failed' in line or b'tracker_nonfinite_pose' in line:
                    self.error = 'tracker_numerical_failure'
                    self.stop('tracker_numerical_failure')
                elif b'ORB_FATAL_SIGNAL' in line:
                    self.error = self.error or 'tracker_process_crashed'
                    self.stop(self.error)
        finally:
            log.close()

    def _preview_loop(self):
        # Preview work must never wait for geometric confirmation/map conversion.
        import cv2
        import numpy as np
        remap = cv2.initUndistortRectifyMap(camera_matrix(self.camera), np.array(self.camera['distortion']),
            None, camera_matrix(self.camera), (self.camera['width'], self.camera['height']), cv2.CV_32FC1)
        last = None
        while not self.done:
            progress = read_json(self.path / 'tracking.json')
            if progress and progress.get('frame_id') and progress['frame_id'] != last:
                try:
                    data = render_preview(self.path, progress, self.camera, remap)
                except (OSError, ValueError, cv2.error) as error:
                    self.preview_error = type(error).__name__
                    time.sleep(.1)
                    continue
                if data is not None:
                    last = progress['frame_id']
                    with self.condition:
                        self.previews[last] = data
                        while len(self.previews) > 8:
                            del self.previews[next(iter(self.previews))]
                        self.preview_progress = {k: v for k, v in progress.items() if k not in {'features', 'T_cw'}}
                        self.preview_error = None
                    # Keep the last valid rendered image for reopening a stopped session.
                    temporary = self.path / 'preview.tmp.jpg'
                    try:
                        temporary.write_bytes(data)
                        replace_file(temporary, self.path / 'preview.jpg')
                    except OSError as error:
                        self.preview_error = 'preview_save_' + type(error).__name__
            time.sleep(.05)

    def _run(self):
        name = "ssafy-orb-" + uuid.uuid4().hex[:12]
        process = reader = writer = log = preview_worker = None
        try:
            command = ["docker", "run", "--rm", "-i", "--name", name, "--network", "none", "--cap-drop", "ALL",
                "--security-opt", "no-new-privileges", "--cpus", "6", "--memory", "8g",
                "--mount", f"type=bind,source={self.path},target=/output", self.image,
                "-", "/output/camera.yaml", "/output", "1", "0"]
            log = (self.path / "orb.log").open("wb", buffering=0)
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0)
            reader = threading.Thread(target=self._read, args=(process, log), daemon=True)
            reader.start()
            start = time.monotonic()
            while not (self.path / "ready.json").is_file():
                if process.poll() is not None:
                    raise RuntimeError("docker_or_tracker_start_failed; see local orb.log")
                if self.closing or time.monotonic() - start > 60:
                    raise RuntimeError(self.error or "tracker_start_cancelled_or_timeout")
                time.sleep(.1)
            self.ready = True
            self.last_input = time.monotonic()
            if hasattr(self, 'previews'):
                preview_worker = threading.Thread(target=self._preview_loop, daemon=True, name=self.id + '-preview')
                preview_worker.start()
            writer = threading.Thread(target=self._write, args=(process,), daemon=True)
            writer.start()
            while process.poll() is None:
                limit = 40 if getattr(self, 'input_kind', '') == 'tello_camera' else 10
                if not self.closing and time.monotonic() - self.last_input > limit:
                    self.stop("input_timeout")
                self.publisher.update(self.stats())
                if self.error:
                    raise RuntimeError(self.error)
                time.sleep(.15)
            with self.condition:
                self.condition.notify_all()
            writer.join(timeout=2)
            reader.join(timeout=2)
            self.publisher.update(self.stats())
            # Exit 3 means the stream ended without an initialized map. A requested
            # stop is a completed empty recording, not a camera/transport failure.
            empty_stop = process.returncode == 3 and self.closing and self.publisher.revision == 0
            if process.returncode and not empty_stop:
                raise RuntimeError(self.error or "tracker_failed_or_no_initialized_map; see local orb.log")
        except Exception as error:
            self.error = self.error or str(error)
            if process and process.poll() is None:
                subprocess.run(["docker", "stop", "-t", "3", name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
                process.wait(timeout=15)
        finally:
            if writer:
                writer.join(timeout=3)
            if reader and reader.ident:
                reader.join(timeout=3)
            elif log:
                log.close()
            self.done, self.ready = True, False
            if preview_worker:
                preview_worker.join(timeout=3)
            self.publisher.state.update(self.stats(), status="failed" if self.error else "complete",
                                        last_error=self.error, input=getattr(self, "input_kind", "browser_camera"))
            try:
                atomic_json(self.path / 'diagnostics.json', {'error': self.error,
                    'stop_reason': self.reason or 'tracker_eof', 'last_input_pts': self.last_pts,
                    'last_submitted_pts': self.inflight[0] if getattr(self, 'inflight', None) else None,
                    'last_ack_pts': getattr(self, 'ack', -1), 'last_valid_revision': self.publisher.revision,
                    'backend_log': 'orb.log', 'client_events': 'events.jsonl',
                    'tracking_failures': 'tracking_failures.jsonl', 'maps_preserved': True})
                atomic_json(self.path / "state.json", self.publisher.state)
            except OSError as error:
                self.error = self.error or 'session_metadata_write_' + type(error).__name__
                self.publisher.state.update(self.stats(), status='failed', last_error=self.error)
            finally:
                if getattr(self, 'recording', None):
                    self.recording.finish_mapping(self.publisher.state)


class LiveManager:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.lock = threading.Lock()
        self.current = None

    def start(self, config, *, input_kind="browser_camera", timestamp_basis="browser_media_time"):
        camera = live_camera(config)
        with self.lock:
            if self.current and not self.current.done:
                raise RuntimeError("another_camera_session_is_active")
            # Fail before changing the viewer's selected result when Docker isn't available.
            image_id = require_slam_image()
            self.current = LiveSession(self.root, camera, image_id, input_kind=input_kind, timestamp_basis=timestamp_basis)
            return self.current.stats()

    def require(self, session_id):
        with self.lock:
            current = self.current
            if not current or current.id != session_id:
                raise RuntimeError("camera_session_mismatch")
            return current

    def shutdown(self):
        if self.current and not self.current.done:
            self.current.stop("server_shutdown")
            self.current.worker.join(timeout=20)
        if self.current and getattr(self.current, 'recording', None):
            self.current.recording.wait(timeout=30)
