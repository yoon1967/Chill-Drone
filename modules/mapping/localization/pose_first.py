"""Reuse the existing native ORB-SLAM3 camera tracker before depth inference.

One JPEG in flight. Each response pins pose, map ID and sparse depth anchors to
the exact input timestamp. No cached trajectory or RGB-D pose optimization.
"""
from __future__ import annotations

import json
from pathlib import Path
import queue
import subprocess
import threading
import time
import uuid

import cv2
import numpy as np

from .orbslam3 import IMAGE, write_settings
from .orbslam3_live import require_slam_image


def fixed_camera_pose(result, stamp):
    """Only a successful, exact-frame native tracking result can admit depth."""
    if result.get('timestamp_s') != stamp or result.get('tracking_state') != 2:
        return None
    value = np.asarray(result.get('T_cw', []), dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        return None
    if not np.allclose(value[3], [0, 0, 0, 1], atol=1e-5):
        return None
    rotation = value[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=.005) or np.linalg.det(rotation) < .99:
        return None
    return np.linalg.inv(value)


def depth_in_pose_units(depth, anchors, tracking_size):
    """Match depth scale to the fixed monocular pose; never change its translation."""
    anchors = np.asarray(anchors, dtype=float)
    if anchors.ndim != 2 or anchors.shape[1] != 3 or len(anchors) < 8:
        return None
    h, w = depth.shape
    xy = anchors[:, :2] * np.array([w / tracking_size[0], h / tracking_size[1]])
    uv = np.round(xy).astype(int)
    valid = np.isfinite(anchors).all(axis=1) & (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h) & (anchors[:, 2] > 0)
    uv, z = uv[valid], anchors[valid, 2]
    if len(z) < 8:
        return None
    estimated = depth[uv[:, 1], uv[:, 0]]
    valid = np.isfinite(estimated) & (estimated > 0)
    if valid.sum() < 8:
        return None
    scale = float(np.median(z[valid] / estimated[valid]))
    return (depth * scale).astype(np.float32), scale


class OrbPoseTracker:
    def __init__(self, output, size, original_size, camera=None, fps=15):
        require_slam_image()
        self.root = Path(output) / 'pose_tracker'
        self.root.mkdir()
        w, h = size
        if camera:
            if (camera['width'], camera['height']) != original_size:
                raise ValueError('frame_dimensions_do_not_match_calibration')
            calibration = {**camera, 'fx': camera['fx'] * w / camera['width'],
                'fy': camera['fy'] * h / camera['height'], 'cx': camera['cx'] * w / camera['width'],
                'cy': camera['cy'] * h / camera['height']}
        else:
            calibration = dict(fx=w * .8660254, fy=w * .8660254, cx=w / 2, cy=h / 2, distortion=[0.] * 4)
        calibration.update(width=w, height=h, fps=fps)
        self.camera = calibration
        write_settings(self.root / 'camera.yaml', calibration, 1800)
        with (self.root / 'camera.yaml').open('a', encoding='utf-8') as stream:
            stream.write('\nMapping.poseFirst: 1\n')
        self.name = 'ssafy-pose-' + uuid.uuid4().hex[:12]
        (self.root / 'container.json').write_text(json.dumps({'name': self.name}), encoding='utf-8')
        self.responses = queue.Queue(maxsize=2)
        self.log = (self.root / 'orb.log').open('wb', buffering=0)
        command = ['docker', 'run', '--rm', '-i', '--name', self.name, '--network', 'none',
            '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges', '--cpus', '6', '--memory', '8g',
            '--mount', f'type=bind,source={self.root.resolve()},target=/output', IMAGE,
            '-', '/output/camera.yaml', '/output', '1', '0']
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, bufsize=65536)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        started = time.monotonic()
        try:
            while not (self.root / 'ready.json').is_file():
                if self.process.poll() is not None:
                    raise RuntimeError('pose_tracker_start_failed')
                if time.monotonic() - started > 60:
                    raise RuntimeError('pose_tracker_start_timeout')
                time.sleep(.05)
            if not json.loads((self.root / 'ready.json').read_text(encoding='utf-8')).get('pose_first'):
                raise RuntimeError('pose_tracker_image_outdated; rebuild ssafy-orbslam3:local')
        except BaseException:
            self.close()
            raise

    def _read(self):
        try:
            for line in iter(self.process.stdout.readline, b''):
                if line.startswith(b'ORB_POSE '):
                    self.responses.put(json.loads(line[9:]))
                else:
                    self.log.write(line)
        finally:
            self.log.close()

    def track(self, rgb, stamp):
        valid, encoded = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not valid:
            raise ValueError('pose_frame_encode_failed')
        jpeg = encoded.tobytes()
        packet = memoryview(f'{len(jpeg)} {stamp:.17g}\n'.encode('ascii') + jpeg)
        while packet:
            written = self.process.stdin.write(packet)
            if not written:
                raise RuntimeError('pose_tracker_stdin_closed')
            packet = packet[written:]
        self.process.stdin.flush()
        started = time.monotonic()
        while time.monotonic() - started < 15:
            try:
                result = self.responses.get(timeout=.1)
                if result.get('timestamp_s') != stamp:
                    raise RuntimeError('pose_frame_timestamp_mismatch')
                return result
            except queue.Empty:
                if self.process.poll() is not None:
                    raise RuntimeError('pose_tracker_process_failed')
        raise RuntimeError('pose_tracker_frame_timeout; rebuild ssafy-orbslam3:local with current bridge')

    def close(self):
        try:
            self.process.stdin.close()
        except OSError:
            pass
        try:
            self.process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            subprocess.run(['docker', 'stop', '-t', '2', self.name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
            self.process.wait(timeout=5)
        self.reader.join(timeout=2)
