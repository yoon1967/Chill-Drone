"""Append-only reconstruction observations and bounded-display replay.

The preview may forget old voxels; observations retain full processed depth,
color, intrinsics, fixed pose and map identity for later integration. Replay
uses the exact displayed point sample, without running inference again.
"""
from __future__ import annotations

import io
import json
import math
from pathlib import Path
import queue
import threading
import zipfile

import numpy as np
from .mapping_limits import MAX_FRAME_PREVIEW_POINTS, MAX_PREVIEW_POINTS

SCHEMA = 'pose-depth-history/1.0'


def _array_bytes(array):
    output = io.BytesIO()
    np.save(output, np.asarray(array), allow_pickle=False)
    return output.getvalue()


class HistoryWriter:
    def __init__(self, output, write_json, render_window_s=10., synchronous=False):
        self.root = Path(output) / 'history'
        (self.root / 'frames').mkdir(parents=True)
        self.write_json = write_json
        self.manifest = {'schema_version': SCHEMA, 'status': 'recording', 'frames': 0,
            'coordinate_system': 'per-map ORB or Tello sensor frame; camera x-right/y-down/z-forward',
            'scale': 'meters for Tello sensor odometry; fixed monocular pose units otherwise',
            'render_window_s': render_window_s, 'preview_points_limit': MAX_PREVIEW_POINTS,
            'observation_fields': ['depth', 'rgb', 'world_from_camera', 'intrinsics', 'map_id', 'depth_scale'],
            'integration': 'unproject depth with intrinsics, transform by world_from_camera; integrate only within the same map_id until maps are explicitly aligned'}
        self.write_json(self.root / 'manifest.json', self.manifest)
        self.pending = queue.Queue(maxsize=8)
        self.error = None
        self.closed = False
        self.synchronous = synchronous
        self.index = None
        self.thread = None
        if synchronous:
            self.index = (self.root / 'index.jsonl').open('a', encoding='utf-8')
        else:
            self.thread = threading.Thread(target=self._write, daemon=True)
            self.thread.start()

    def append(self, snapshot, observation=None):
        if self.closed:
            raise RuntimeError('history_writer_closed')
        if self.synchronous:
            self._write_frame(self.index, snapshot, observation)
            return
        while True:
            if self.error:
                raise RuntimeError('history_recording_failed: ' + str(self.error)) from self.error
            try:
                self.pending.put((snapshot, observation), timeout=.1)
                return
            except queue.Full:
                continue  # Never drop reconstructed observations to protect the preview.

    def _write(self):
        try:
            with (self.root / 'index.jsonl').open('a', encoding='utf-8') as index:
                while True:
                    item = self.pending.get()
                    if item is None:
                        break
                    self._write_frame(index, *item)
        except Exception as error:
            self.error = error

    def _write_frame(self, index, snapshot, observation):
        revision = snapshot['revision']
        path = self.root / 'frames' / f'{revision:08d}.npz'
        temporary = path.with_suffix('.npz.tmp')
        arrays = {'points': np.asarray(snapshot['points'], dtype=np.float32).reshape(-1, 6),
                  'camera_path': np.asarray(snapshot['cameras'], dtype=np.float32).reshape(-1, 3)}
        if observation:
            arrays.update(observation)
        frame_preview = snapshot.get('frame_preview')
        if frame_preview:
            arrays['frame_preview_points'] = np.asarray(frame_preview['points'], dtype=np.float32).reshape(-1, 6)
        if snapshot.get('rescue') is not None:
            arrays['rescue_json'] = np.frombuffer(json.dumps(snapshot['rescue'], allow_nan=False,
                separators=(',', ':')).encode('utf-8'), dtype=np.uint8)
        # Sequential reconstruction trades larger chunks for no compression stall.
        compression = zipfile.ZIP_STORED if self.synchronous else zipfile.ZIP_DEFLATED
        with zipfile.ZipFile(temporary, 'w', compression=compression,
                             compresslevel=None if self.synchronous else 1) as archive:
            for name, array in arrays.items():
                archive.writestr(name + '.npy', _array_bytes(array))
        from modules.mapping.localization.orbslam3 import replace_file
        replace_file(temporary, path)
        entry = {key: value for key, value in snapshot.items() if key not in {'points', 'cameras', 'rescue', 'frame_preview'}}
        if frame_preview:
            entry['frame_preview_space'] = frame_preview['space']
            entry['frame_preview_tof'] = frame_preview.get('tof_verified', False)
        if snapshot.get('rescue') is not None:
            entry['semantic_mapping'] = True
        entry.update(observation=bool(observation), visible_points=len(snapshot['points']))
        index.write(json.dumps(entry, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n')
        index.flush()
        self.manifest.update(frames=self.manifest['frames'] + 1, last_timestamp_s=entry['timestamp_s'])
        self.write_json(self.root / 'manifest.json', self.manifest)

    def close(self, status='complete'):
        if self.closed:
            return
        self.closed = True
        if self.synchronous:
            self.index.close()
            self.manifest['status'] = status
            self.write_json(self.root / 'manifest.json', self.manifest)
            return
        while self.thread.is_alive():
            try:
                self.pending.put(None, timeout=.1)
                break
            except queue.Full:
                if self.error:
                    break
        self.thread.join()
        if self.error:
            raise RuntimeError('history_recording_failed: ' + str(self.error)) from self.error
        self.manifest['status'] = status
        self.write_json(self.root / 'manifest.json', self.manifest)


class HistoryReader:
    def __init__(self, output):
        self.root = Path(output) / 'history'
        self.offset = 0
        self.entries = []
        self.lock = threading.RLock()

    def refresh(self):
        with self.lock:
            manifest = json.loads((self.root / 'manifest.json').read_text(encoding='utf-8'))
            if manifest.get('schema_version') != SCHEMA:
                raise ValueError('unsupported_reconstruction_history')
            path = self.root / 'index.jsonl'
            if not path.is_file():
                return []
            if path.stat().st_size < self.offset:
                self.offset, self.entries = 0, []
            with path.open('rb') as stream:
                stream.seek(self.offset)
                for line in stream:
                    if not line.endswith(b'\n'):
                        break
                    value = json.loads(line)
                    revision, stamp = value.get('revision'), value.get('timestamp_s')
                    if type(revision) is not int or revision != len(self.entries) + 1 or type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp < 0:
                        raise ValueError('invalid_reconstruction_history_index')
                    if self.entries and stamp < self.entries[-1]['timestamp_s']:
                        raise ValueError('history_timestamps_not_ordered')
                    self.entries.append(value)
                    self.offset = stream.tell()
            return self.entries

    def timeline(self):
        entries = self.refresh()
        return {'schema_version': SCHEMA, 'entries': [{key: e.get(key) for key in (
            'revision', 'timestamp_s', 'frame_number', 'map_id', 'tracking', 'observation', 'map_points')} for e in entries],
            'available': bool(entries), 'count': len(entries), 'last_timestamp_s': entries[-1]['timestamp_s'] if entries else 0.}

    def snapshot(self, revision):
        with self.lock:
            entries = self.refresh()
            if type(revision) is not int or not 1 <= revision <= len(entries):
                raise ValueError('unknown_history_revision')
            path = self.root / 'frames' / f'{revision:08d}.npz'
            if path.is_symlink() or not path.resolve().is_relative_to(self.root.resolve()):
                raise ValueError('invalid_history_chunk_path')
            result = dict(entries[revision - 1])
            with zipfile.ZipFile(path) as archive:
                for name, limit in [('points.npy', MAX_PREVIEW_POINTS * 6 * 4 + 4096), ('camera_path.npy', 300 * 3 * 4 + 4096)]:
                    if archive.getinfo(name).file_size > limit:
                        raise ValueError('history_preview_too_large')
                points = np.load(io.BytesIO(archive.read('points.npy')), allow_pickle=False)
                cameras = np.load(io.BytesIO(archive.read('camera_path.npy')), allow_pickle=False)
                space = result.pop('frame_preview_space', None)
                if space is not None:
                    name = 'frame_preview_points.npy'
                    if (not isinstance(space, str) or space not in {'camera', 'map'} or name not in archive.namelist()
                            or archive.getinfo(name).file_size > MAX_FRAME_PREVIEW_POINTS * 6 * 4 + 4096):
                        raise ValueError('invalid_frame_depth_preview')
                    local = np.load(io.BytesIO(archive.read(name)), allow_pickle=False)
                    if local.dtype != np.float32 or local.ndim != 2 or local.shape[1] != 6 or len(local) > MAX_FRAME_PREVIEW_POINTS or not np.isfinite(local).all():
                        raise ValueError('invalid_frame_depth_preview')
                    result['frame_preview'] = {'space': space, 'points': local.tolist(),
                        'tof_verified': result.pop('frame_preview_tof', False) is True}
                rescue = None
                if 'rescue_json.npy' in archive.namelist():
                    if archive.getinfo('rescue_json.npy').file_size > 12 * 1024 * 1024:
                        raise ValueError('history_semantic_preview_too_large')
                    data = np.load(io.BytesIO(archive.read('rescue_json.npy')), allow_pickle=False)
                    if data.dtype != np.uint8 or data.ndim != 1:
                        raise ValueError('invalid_history_semantic_preview')
                    rescue = json.loads(data.tobytes())
                    from modules.mapping.data.rescue_mapping import validate_payload
                    validate_payload(rescue)
            if points.ndim != 2 or points.shape[1] != 6 or cameras.ndim != 2 or cameras.shape[1] != 3 or not np.isfinite(points).all() or not np.isfinite(cameras).all():
                raise ValueError('invalid_history_preview')
            result.update(points=points.tolist(), cameras=cameras.tolist(), recorded=True)
            if rescue is not None:
                result['rescue'] = rescue
            return result
