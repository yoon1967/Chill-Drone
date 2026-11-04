"""Live-camera adapter for the same sequential pose/depth/history pipeline as saved video."""
import io
import json
import math
from pathlib import Path
import threading
import time
import uuid

from PIL import Image, UnidentifiedImageError

from modules.mapping.data.live_recording import SessionRecording
from modules.mapping.data.session_import import ImportedMap
from .orbslam3 import atomic_json


class LiveDepthSession:
    def __init__(self, root, camera, image_id, imports, *, input_kind='browser_camera', timestamp_basis='browser_media_time'):
        self.id = 'live_' + uuid.uuid4().hex[:16]
        self.path = Path(root) / self.id
        self.path.mkdir(parents=True)
        self.camera, self.imports = camera, imports
        self.input_kind, self.timestamp_basis = input_kind, timestamp_basis
        self.condition = threading.Condition()
        self.closing = self.done = self.ready = self.waiting = False
        self.pending = self.preview = self.reason = self.error = None
        self.last_sequence = self.completed_sequence = -1
        self.last_pts = self.ack = -1.
        self.created = self.last_input = time.monotonic()
        self.last_admitted = 0.
        self.client_event_count = self.received = self.submitted = self.dropped = 0
        self.sampled_out = self.raw_received = 0
        atomic_json(self.path / 'camera.json', camera)
        atomic_json(self.path / 'settings.json', {'backend': 'orb_slam3_depth', 'input': input_kind,
            'input_timestamp_basis': timestamp_basis, 'image_id': image_id, 'camera': camera,
            'frame_policy': 'sequential_pose_depth_record_present; fresh_live_admission', 'dense_surface': True})
        self.recording = SessionRecording(self.path, camera, input_kind)
        self.jpeg_recording = self.recording.open_part('received_jpeg', timestamp_basis) if input_kind == 'browser_camera' else None
        self.depth_job_id = imports.create('실시간 깊이 지도')['id']
        self.job = imports.job(self.depth_job_id)
        self.job.update(status='running', stage='model', output=self.path / 'reconstruction',
            options={'mode': 'depth_rgbd', 'frame_fps': camera['fps'], 'sample_fps': 5,
                     'camera': camera, 'run_id': self.id, 'render_window_s': 10.},
            live_session=self, presentation_sync=True)
        self._write_state()
        self.worker = threading.Thread(target=self._run, daemon=True, name=self.id)
        self.job['thread'] = self.worker
        self.worker.start()

    @property
    def can_accept_frame(self):
        return self.waiting and not self.closing and not self.done and time.monotonic() - self.last_admitted >= 1 / 15

    def stats(self):
        job = self.imports.status(self.depth_job_id)
        progress = job.get('progress', {})
        lifecycle = ('failed' if self.error else 'complete') if self.done else 'stopping' if self.closing else 'receiving' if self.ready else 'starting'
        return {'source': 'live', 'backend': 'orb_slam3_depth', 'session_id': self.id, 'session_status': lifecycle,
            'status': lifecycle, 'input': self.input_kind, 'camera': self.camera,
            'input_origin': getattr(self, 'input_origin', 'local_camera' if self.input_kind == 'browser_camera' else 'not_connected'),
            'input_state': getattr(self, 'input_state', 'receiving' if self.ready else 'connecting'),
            'input_error': getattr(self, 'input_error', None), 'reconnects': getattr(self, 'reconnects', 0),
            'input_age_s': max(0, time.monotonic() - self.last_input),
            'transport_ready': self.ready and not self.closing, 'transport_active': not self.done,
            'transport_stopping': self.closing, 'can_accept_frame': self.can_accept_frame,
            'transport_received': self.received, 'transport_submitted': self.submitted,
            'frame_drops': self.dropped, 'queue_depth': int(self.pending is not None), 'queue_capacity': 1,
            'live_sampled_out': self.sampled_out, 'raw_received': self.raw_received,
            'processed_input_timestamp_s': self.ack if self.ack >= 0 else None,
            'last_input_timestamp_s': self.last_pts if self.last_pts >= 0 else None,
            'last_input_sequence': self.last_sequence, 'last_error': self.error,
            'stop_reason': self.reason, 'revision': progress.get('revision', 0),
            'map_available': bool(self.done and (self.path / 'reconstruction/scene.json').is_file()),
            'revision_path': 'reconstruction' if (self.path / 'reconstruction/scene.json').is_file() else '',
            'depth_job_id': self.depth_job_id, 'depth_job': job, 'recording': self.recording.stats()}

    def _write_state(self):
        state = self.stats()
        state.pop('depth_job', None)
        atomic_json(self.path / 'state.json', state)

    def record_event(self, kind, message=''):
        if kind not in {'client_error', 'status_retry', 'status_recovered', 'page_hidden', 'stop_requested'} or not isinstance(message, str) or len(message) > 1024:
            raise ValueError('invalid_client_event')
        with self.condition:
            if self.client_event_count < 512:
                with (self.path / 'events.jsonl').open('a', encoding='utf-8') as output:
                    output.write(json.dumps({'event': kind, 'message': message,
                        'elapsed_s': time.monotonic() - self.created, 'input_timestamp_s': self.last_pts}) + '\n')
                self.client_event_count += 1

    def note_skipped_frame(self):
        self.sampled_out += 1

    def note_input_frame(self):
        self.last_input = time.monotonic()
        self.raw_received += 1

    def offer(self, jpeg, pts, sequence):
        if not math.isfinite(pts) or not 0 <= pts <= 86400 or sequence < 0:
            raise ValueError('invalid_frame_timestamp_or_sequence')
        if len(jpeg) > 2097152 or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
            raise ValueError('invalid_or_oversized_jpeg')
        try:
            with Image.open(io.BytesIO(jpeg)) as image:
                valid = image.format == 'JPEG' and image.size == (self.camera['width'], self.camera['height'])
        except (UnidentifiedImageError, OSError, ValueError):
            valid = False
        if not valid:
            raise ValueError('frame_dimensions_do_not_match_calibration')
        with self.condition:
            if self.input_kind == 'browser_camera':
                deadline = time.monotonic() + 30
                while not self.can_accept_frame and not self.closing and not self.done:
                    if time.monotonic() >= deadline:
                        raise RuntimeError('live_frame_admission_timeout')
                    self.condition.wait(.05)
            if self.closing or self.done:
                raise RuntimeError('camera_session_not_receiving')
            if not self.can_accept_frame:
                self.sampled_out += 1
                return self.stats()
            if pts <= self.last_pts or sequence <= self.last_sequence:
                raise ValueError('frame_timestamp_or_sequence_not_increasing')
            self.last_pts, self.last_sequence = pts, sequence
            self.last_input = self.last_admitted = time.monotonic()
            self.received += 1
            if self.jpeg_recording:
                self.jpeg_recording.offer(jpeg, pts)
            self.pending = (jpeg, pts, sequence)
            self.waiting = False
            self.condition.notify_all()
            if self.input_kind == 'browser_camera':
                self.condition.wait_for(lambda: self.completed_sequence >= sequence or self.closing or self.done, timeout=30)
        return self.stats()

    def _next_frame(self):
        with self.condition:
            self.completed_sequence = self.last_sequence
            if not self.ready:
                self.last_input = time.monotonic()
            self.ready = self.waiting = True
            waiting_since = time.monotonic()
            self.condition.notify_all()
            while self.pending is None and not self.closing and not self.job['cancel'].is_set():
                self.condition.wait(.2)
                if time.monotonic() - max(self.last_input, waiting_since) > (40 if self.input_kind == 'tello_camera' else 15):
                    self.stop('input_timeout')
            if self.closing or self.job['cancel'].is_set():
                return None
            packet, self.pending = self.pending, None
            self.waiting = False
            self.submitted += 1
            return packet

    def _published(self, snapshot):
        with self.condition:
            self.preview = snapshot
            self.ack = snapshot['timestamp_s']
            self.condition.notify_all()
        with self.imports.preview_changed:
            self.job['preview'] = snapshot
            self.imports.preview_changed.notify_all()
            self.imports.preview_changed.wait_for(lambda: self.closing or self.job['cancel'].is_set() or
                self.job.get('presented_revision', 0) >= snapshot['revision'])

    def stop(self, reason='user_stop'):
        with self.condition:
            if self.closing or self.done:
                return
            self.record_event('stop_requested', reason)
            self.closing, self.reason, self.waiting = True, reason, False
            self.pending = None
            self.condition.notify_all()
        with self.imports.preview_changed:
            self.imports.preview_changed.notify_all()
        if self.jpeg_recording:
            self.jpeg_recording.close()
            self.recording.finish_input()

    def _run(self):
        try:
            self.imports.depth_worker.run('live', self.job['output'], self.job['options'],
                stop_event=self.job['cancel'], on_preview=self._published, next_frame=self._next_frame)
            self.job.update(stage='saving')
        except Exception as error:
            empty_stop = self.closing and str(error) == 'no_valid_depth_map'
            if not empty_stop:
                self.error = str(error)
            self.job.update(status='cancelled' if empty_stop else 'failed', stage='finished', error=str(error))
        finally:
            self.done, self.ready, self.waiting = True, False, False
            with self.condition:
                self.condition.notify_all()
            try:
                self._write_state()
            except OSError as error:
                self.error = self.error or 'session_metadata_write_' + type(error).__name__
            finally:
                self.recording.finish_mapping(self.stats())
                if self.input_kind == 'browser_camera':
                    if self.jpeg_recording:
                        self.jpeg_recording.close()
                    self.recording.finish_input()
                # Read the completed manifest so reopened results include playback and hashes.
                # state.json must stay unchanged after the recorder has hashed it.
                self.recording.wait(timeout=30)
                try:
                    if (self.job['output'] / 'scene.json').is_file():
                        record = ImportedMap(self.path)
                        self.job.update(record=record, summary=record.summary(), status='complete', stage='finished')
                    self.imports._save(self.job)
                except (OSError, ValueError) as error:
                    self.error = self.error or str(error)
                    self.job.update(status='failed', stage='finished', error=self.error)
                with self.imports.preview_changed:
                    self.imports.preview_changed.notify_all()
