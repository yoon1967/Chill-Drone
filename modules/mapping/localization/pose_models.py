"""Run a selected learned tracker locally using the ORB frame transport."""

import json
import queue
import subprocess
import threading
import time
import uuid

from .orbslam3 import IMAGE as ORB_IMAGE
from .pose_first import fixed_camera_pose, send_pose_frame

ORB = 'orb-slam3'
DPVO = 'dpvo'
HYBRID = 'orb-slam3-dpvo'
DPV = 'dpv-slam'
DPV_PLUS_PLUS = 'dpv-slam-plus-plus'
VISTA = 'vista-slam'

DPVO_IMAGE = 'ssafy-reconstruction-dpvo:local'
CLASSIC_IMAGE = 'ssafy-reconstruction-dpv-slam-plus-plus:local'
VISTA_IMAGE = 'ssafy-reconstruction-vista:local'
VISTA_CHECKPOINT = 'frontend_sta_weights.pth'
VISTA_VOCABULARY = 'ORBvoc.txt'
GPU_IMAGES = {DPVO: DPVO_IMAGE, HYBRID: DPVO_IMAGE, DPV: DPVO_IMAGE,
              DPV_PLUS_PLUS: CLASSIC_IMAGE, VISTA: VISTA_IMAGE}
POSE_IMAGES = {model: (image,) for model, image in GPU_IMAGES.items()}
POSE_IMAGES.update({ORB: (ORB_IMAGE,), HYBRID: (ORB_IMAGE, DPVO_IMAGE)})
BUILD_TARGETS = {DPVO_IMAGE: 'dpvo', CLASSIC_IMAGE: 'dpv_slam_plus_plus', VISTA_IMAGE: 'vista'}
BATCH_POSE_MODELS = frozenset(GPU_IMAGES) - {HYBRID}


class LearnedPoseTracker:
    """One frame in flight, with cancellation during startup, tracking and finalization."""

    def __init__(self, model, size, intrinsics, distortion, cancelled, assets=None):
        self.cancelled = cancelled
        self.name = 'ssafy-reconstruction-pose-' + uuid.uuid4().hex[:12]
        self.replies = queue.SimpleQueue()
        config = json.dumps(dict(model=model, size=size, intrinsics=intrinsics, distortion=distortion))
        command = ['docker', 'run', '--rm', '--pull=never', '-i', '--name', self.name,
                   '--network', 'none', '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges',
                   '--gpus', 'all']
        if assets is not None:
            # The Hugging Face snapshot may contain links to a separate blobs folder.
            for filename in (VISTA_CHECKPOINT, VISTA_VOCABULARY):
                source = (assets / filename).resolve(strict=True)
                command += ['--mount', f'type=bind,source={source},target=/pose-model/{filename},readonly']
        self.process = subprocess.Popen(command + [GPU_IMAGES[model], config],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=65536,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        try:
            if self._reply(240).get('ready') is not True:
                raise RuntimeError('pose_worker_start_failed')
        except BaseException:
            self.close()
            raise

    def _read(self):
        with self.process.stdout as output:
            for line in output:
                if line.startswith(b'POSE_RESULT '):
                    self.replies.put(line[12:])
        self.replies.put(None)

    def _reply(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.cancelled():
                raise InterruptedError('cancelled')
            try:
                line = self.replies.get(timeout=.1)
            except queue.Empty:
                continue
            if line is None:
                raise RuntimeError('pose_worker_stopped')
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError('invalid_pose_worker_reply')
            if 'error' in value:
                raise RuntimeError(value['error'])
            return value
        raise RuntimeError('pose_worker_timeout')

    def track(self, rgb, stamp):
        if self.cancelled():
            raise InterruptedError('cancelled')
        send_pose_frame(self.process.stdin, rgb, stamp)
        result = self._reply(180)
        if result.get('timestamp_s') != stamp:
            raise RuntimeError('pose_frame_timestamp_mismatch')
        return result

    def finish(self, stamps):
        self.process.stdin.close()
        trajectory = {}
        for stamp in stamps:
            result = self._reply(240)
            if result.get('finished') is True and not trajectory:
                return {}
            if result.get('timestamp_s') != stamp:
                raise RuntimeError('pose_frame_timestamp_mismatch')
            if fixed_camera_pose(result, stamp) is not None:
                trajectory[stamp] = result
            else:
                trajectory[stamp] = {'timestamp_s': stamp, 'tracking_state': 1, 'map_id': 0}
        if self._reply(240).get('finished') is not True:
            raise RuntimeError('invalid_pose_worker_reply')
        return trajectory

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
                               stderr=subprocess.DEVNULL, timeout=6)
            except (OSError, subprocess.TimeoutExpired):
                pass
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait(timeout=5)
        self.reader.join(timeout=2)
