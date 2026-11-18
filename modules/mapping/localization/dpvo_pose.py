"""Optional DPVO transport and a short, causal bridge into ORB's map units."""

from collections import deque
import json
import math
import queue
import subprocess
import threading
import time
import uuid

import numpy as np

from .pose_first import fixed_camera_pose, send_pose_frame


IMAGE = 'ssafy-dpvo:local'
DPVO_POSE_SOURCE = 'dpvo_orb_aligned'


class DpvoPoseTracker:
    @classmethod
    def start(cls, size, intrinsics, distortion, cancelled):
        present = subprocess.run(['docker', 'image', 'inspect', IMAGE],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=4)
        return cls(size, intrinsics, distortion, cancelled) if present.returncode == 0 else None

    def __init__(self, size, intrinsics, distortion, cancelled):
        self.cancelled = cancelled
        self.name = 'ssafy-dpvo-' + uuid.uuid4().hex[:12]
        self.replies = queue.SimpleQueue()
        calibration = json.dumps(dict(size=size, intrinsics=intrinsics, distortion=distortion))
        self.process = subprocess.Popen(['docker', 'run', '--rm', '--pull=never', '-i', '--name', self.name,
            '--network', 'none', '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges',
            '--gpus', 'all', IMAGE, calibration], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, bufsize=65536)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        try:
            if self._reply(timeout=45).get('ready') is not True:
                raise RuntimeError('dpvo_start_failed')
        except BaseException:
            self.close()
            raise

    def _read(self):
        with self.process.stdout as output:
            for line in output:
                self.replies.put(line)

    def _reply(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.cancelled():
                raise InterruptedError('cancelled')
            try:
                result = json.loads(self.replies.get(timeout=.1))
            except queue.Empty:
                if self.process.poll() is not None:
                    raise RuntimeError('dpvo_worker_stopped')
                continue
            if not isinstance(result, dict):
                raise ValueError('invalid_dpvo_reply')
            if 'error' in result:
                raise RuntimeError(result['error'])
            return result
        raise RuntimeError('dpvo_reply_timeout')

    def track(self, rgb, stamp):
        send_pose_frame(self.process.stdin, rgb, stamp)
        result = self._reply(timeout=15)
        if result.get('timestamp_s') != stamp:
            raise RuntimeError('dpvo_frame_timestamp_mismatch')
        return fixed_camera_pose(result, stamp)

    def close(self):
        try:
            self.process.stdin.close()
        except OSError:
            pass
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            try:
                subprocess.run(['docker', 'stop', '-t', '1', self.name], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                pass
            if self.process.poll() is None:
                try:
                    self.process.kill()
                    self.process.wait(timeout=2)
                except (OSError, subprocess.TimeoutExpired):
                    pass
        self.reader.join(timeout=2)


class DpvoAlignment:
    """Fit a similarity from shared camera poses, then use it for a bounded gap."""

    def __init__(self):
        self.pairs = deque(maxlen=12)
        self.map_id = None
        self.fit = None

    def observe(self, stamp, map_id, orb_pose, dpvo_pose):
        if map_id != self.map_id or (self.pairs and stamp - self.pairs[-1][0] > .4):
            self.pairs.clear()
        self.map_id = map_id
        self.fit = None
        self.pairs.append((stamp, orb_pose, dpvo_pose))
        while stamp - self.pairs[0][0] > .6:
            self.pairs.popleft()

        rotation = orb_pose[:3, :3] @ dpvo_pose[:3, :3].T
        orb_position, dpvo_position = orb_pose[:3, 3], dpvo_pose[:3, 3]
        motions = []
        for previous_stamp, orb, dpvo in list(self.pairs)[:-1]:
            if self._turn(orb[:3, :3], rotation @ dpvo[:3, :3]) > math.radians(8):
                return
            orb_delta = orb_position - orb[:3, 3]
            dpvo_delta = rotation @ (dpvo_position - dpvo[:3, 3])
            orb_distance, dpvo_distance = np.linalg.norm(orb_delta), np.linalg.norm(dpvo_delta)
            if (orb_distance > 1e-4) != (dpvo_distance > 1e-4):
                return
            if min(orb_distance, dpvo_distance) > 1e-4:
                motions.append((orb_delta, dpvo_delta, orb_distance, dpvo_distance, stamp - previous_stamp))
        if len(motions) < 2:
            return

        scale = float(np.median([orb_distance / dpvo_distance for _, _, orb_distance, dpvo_distance, _ in motions]))
        if any(np.linalg.norm(orb_delta - scale * dpvo_delta) > .25 * distance
               for orb_delta, dpvo_delta, distance, _, _ in motions):
            return
        speed = max(distance / elapsed for _, _, distance, _, elapsed in motions)
        self.fit = (scale, rotation, speed)

    def project(self, stamp, dpvo_pose):
        if self.fit is None:
            return None
        last_stamp, orb, dpvo = self.pairs[-1]
        elapsed = stamp - last_stamp
        if not 0 < elapsed <= 1.:
            return None

        scale, rotation, speed = self.fit
        offset = scale * (rotation @ (dpvo_pose[:3, 3] - dpvo[:3, 3]))
        camera_rotation = rotation @ dpvo_pose[:3, :3]
        if (np.linalg.norm(offset) > 3 * speed * elapsed or
                self._turn(orb[:3, :3], camera_rotation) > math.radians(12 + 180 * elapsed)):
            return None
        pose = np.eye(4)
        pose[:3, :3], pose[:3, 3] = camera_rotation, orb[:3, 3] + offset
        return pose, self.map_id

    @staticmethod
    def _turn(a, b):
        return math.acos(float(np.clip((np.trace(a.T @ b) - 1.) / 2., -1., 1.)))
