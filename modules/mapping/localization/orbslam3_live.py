"""Bounded browser JPEG -> Docker ORB-SLAM3 transport; all artifacts stay local."""
from __future__ import annotations

import math
from pathlib import Path
import subprocess
import threading
import time
import uuid

import cv2
import numpy as np

from .orbslam3 import IMAGE, SessionPublisher, atomic_json, read_json, write_settings


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
    def __init__(self, root, camera, image_id, image=IMAGE):
        self.id = "live_" + uuid.uuid4().hex[:16]
        self.path = root / self.id
        self.camera, self.image = camera, image
        self.condition = threading.Condition()
        self.pending = None
        self.closing = False
        self.done = False
        self.ready = False
        self.reason = "user_stop"
        self.error = None
        self.last_sequence, self.last_pts, self.ack = -1, -1., -1.
        self.last_input = time.monotonic()
        self.received = self.dropped = self.submitted = 0
        self.path.mkdir(parents=True)
        (self.path / "revisions").mkdir()
        atomic_json(self.path / "camera.json", camera)
        atomic_json(self.path / "settings.json", {"backend": "orb_slam3_monocular", "input": "browser_camera",
            "image": image, "image_id": image_id, "camera": camera, "scale": "arbitrary",
            "frame_policy": "latest_one_pending; one_inflight_until_tracker_ACK", "dense_surface": False})
        write_settings(self.path / "camera.yaml", camera, 1800)
        self.publisher = SessionPublisher(self.path, camera)
        self.publisher.state["calibration_status"] = camera["calibration_status"]
        self.worker = threading.Thread(target=self._run, daemon=True, name=self.id)
        self.worker.start()

    def stats(self):
        with self.condition:
            lifecycle = ("failed" if self.error else "complete") if self.done else \
                "stopping" if self.closing else "receiving" if self.ready else "starting"
            return {"source": "live", "backend": "orb_slam3_monocular",
                    "session_id": self.id, "session_status": lifecycle,
                    "input": "browser_camera", "transport_ready": self.ready,
                    "transport_active": not self.done, "transport_stopping": self.closing,
                    "transport_received": self.received, "transport_submitted": self.submitted,
                    "frame_drops": self.dropped, "queue_depth": int(self.pending is not None),
                    "queue_capacity": 1, "stop_reason": self.reason if self.closing else None,
                    "last_input_timestamp_s": self.last_pts if self.last_pts >= 0 else None,
                    "last_input_sequence": self.last_sequence if self.last_sequence >= 0 else None}

    def offer(self, jpeg, pts, sequence):
        if not math.isfinite(pts) or not 0 <= pts <= 86400 or sequence < 0:
            raise ValueError("invalid_frame_timestamp_or_sequence")
        if len(jpeg) > 2097152 or not jpeg.startswith(b"\xff\xd8"):
            raise ValueError("invalid_or_oversized_jpeg")
        frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None or frame.shape[:2] != (self.camera["height"], self.camera["width"]):
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
            self.pending = (jpeg, pts)
            self.condition.notify_all()
        return self.stats()

    def stop(self, reason="user_stop"):
        with self.condition:
            self.closing, self.reason = True, reason
            self.condition.notify_all()

    def _write(self, process):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.pending is not None or self.closing or process.poll() is not None, timeout=1)
                    if process.poll() is not None:
                        break
                    if self.pending is None:
                        if self.closing:
                            break
                        continue
                    jpeg, pts = self.pending
                    self.pending = None
                packet = memoryview(f"{len(jpeg)} {pts:.17g}\n".encode("ascii") + jpeg)
                while packet:
                    written = process.stdin.write(packet)
                    if not written:
                        raise OSError("tracker_stdin_closed")
                    packet = packet[written:]
                process.stdin.flush()
                with self.condition:
                    self.submitted += 1
                    acknowledged = self.condition.wait_for(lambda: self.ack >= pts or process.poll() is not None, timeout=15)
                    if not acknowledged:
                        raise RuntimeError("tracker_ack_timeout")
        except (OSError, RuntimeError) as error:
            self.error = str(error)
            self.stop("transport_error")
        finally:
            try:
                process.stdin.close()
            except OSError:
                pass

    def _read(self, process, log):
        for line in iter(process.stdout.readline, b""):
            log.write(line)
            if line.startswith(b"ORB_ACK "):
                with self.condition:
                    self.ack = float(line.split()[1])
                    self.condition.notify_all()

    def _run(self):
        name = "ssafy-orb-" + uuid.uuid4().hex[:12]
        process = None
        try:
            command = ["docker", "run", "--rm", "-i", "--name", name, "--network", "none", "--cap-drop", "ALL",
                "--security-opt", "no-new-privileges", "--cpus", "6", "--memory", "8g",
                "--mount", f"type=bind,source={self.path},target=/output", self.image,
                "-", "/output/camera.yaml", "/output", "1", "0"]
            with (self.path / "orb.log").open("wb", buffering=0) as log:
                process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0)
                reader = threading.Thread(target=self._read, args=(process, log), daemon=True)
                reader.start()
                start = time.monotonic()
                while not (self.path / "ready.json").is_file():
                    if process.poll() is not None:
                        raise RuntimeError("docker_or_tracker_start_failed; see local orb.log")
                    if self.closing or time.monotonic() - start > 60:
                        raise RuntimeError("tracker_start_cancelled_or_timeout")
                    time.sleep(.1)
                self.ready = True
                self.last_input = time.monotonic()
                writer = threading.Thread(target=self._write, args=(process,), daemon=True)
                writer.start()
                while process.poll() is None:
                    if not self.closing and time.monotonic() - self.last_input > 10:
                        self.stop("input_timeout")
                    self.publisher.update(self.stats())
                    if self.error:
                        raise RuntimeError(self.error)
                    time.sleep(.15)
                writer.join(timeout=2)
                reader.join(timeout=2)
                self.publisher.update(self.stats())
                # Exit 3 means the stream ended without an initialized map. A requested
                # stop is a completed empty recording, not a camera/transport failure.
                empty_stop = process.returncode == 3 and self.closing and self.publisher.revision == 0
                if process.returncode and not empty_stop:
                    raise RuntimeError("tracker_failed_or_no_initialized_map; see local orb.log")
        except Exception as error:
            self.error = str(error)
            if process and process.poll() is None:
                subprocess.run(["docker", "stop", "-t", "3", name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
                process.wait(timeout=15)
        finally:
            self.done, self.ready = True, False
            self.publisher.state.update(self.stats(), status="failed" if self.error else "complete",
                                        last_error=self.error, input="browser_camera")
            atomic_json(self.path / "state.json", self.publisher.state)


class LiveManager:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.lock = threading.Lock()
        self.current = None

    def start(self, config):
        camera = live_camera(config)
        with self.lock:
            if self.current and not self.current.done:
                raise RuntimeError("another_camera_session_is_active")
            # Fail before changing the viewer's selected result when Docker isn't available.
            image_id = subprocess.check_output(["docker", "image", "inspect", "--format", "{{.Id}}", IMAGE],
                                               stderr=subprocess.DEVNULL, text=True, timeout=10).strip()
            self.current = LiveSession(self.root, camera, image_id)
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
