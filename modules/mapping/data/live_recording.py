"""Bounded, asynchronous capture storage. Never performs disk I/O on a video receiver."""
from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
from fractions import Fraction
import hashlib
import json
import os
from pathlib import Path
import threading
import time


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def atomic_json(path, value):
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    for attempt in range(40):
        try:
            temporary.replace(path)
            return
        except PermissionError:
            if attempt == 39:
                raise
            time.sleep(.005)


def file_hash(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def archive_files(path):
    """Do not follow user-created links/junctions out of a session export."""
    root = Path(path).resolve()
    for directory, directories, names in os.walk(root, followlinks=False):
        directories[:] = sorted(name for name in directories if not (
            (Path(directory) / name).is_symlink() or
            getattr(Path(directory) / name, 'is_junction', lambda: False)()))
        for name in sorted(names):
            item = Path(directory) / name
            if name.endswith('.tmp') or item.is_symlink() or not item.is_file():
                continue
            try:
                item.resolve().relative_to(root)
            except ValueError:
                continue
            yield item, item.relative_to(root).as_posix()


def remux_video(raw, index, destination, mode, fps):
    """Copy encoded packets; reception timestamps are an approximation for raw H.264."""
    import av
    time_base = Fraction(1, 1000000)
    temporary = destination.with_suffix(destination.suffix + '.tmp')
    count, skipped, first, previous = 0, 0, None, -1
    fallback = False
    width = height = 0
    with index.open(encoding='utf-8') as times, \
            av.open(str(raw), format='h264' if mode == 'h264' else 'mjpeg') as source:
        rows = (json.loads(line) for line in times)
        row, following = next(rows, None), next(rows, None)
        stream = source.streams.video[0]
        width, height = stream.codec_context.width, stream.codec_context.height
        with av.open(str(temporary), 'w', format='mp4' if mode == 'h264' else 'matroska') as output:
            if hasattr(output, 'add_stream_from_template'):
                target = output.add_stream_from_template(stream)
            else:
                target = output.add_stream(template=stream)  # PyAV 12 compatibility
            for packet in source.demux(stream):
                if not packet.size:
                    continue
                # No source PTS exists on the Tello UDP elementary stream. Do not
                # invent decode/display ordering for streams with B frames.
                if mode == 'h264' and stream.codec_context.has_b_frames:
                    raise ValueError('b_frame_timestamps_unavailable; raw_video_preserved')
                if mode == 'h264' and count == 0 and not packet.is_keyframe:
                    skipped += 1
                    continue
                if packet.pos is not None and packet.pos >= 0 and row:
                    while following and following['offset'] <= packet.pos:
                        row, following = following, next(rows, None)
                    stamp = row['timestamp_s']
                else:
                    stamp = (first or 0) + count / fps
                    fallback = True
                if first is None:
                    first = stamp
                pts = max(previous + 1, round((stamp - first) * 1000000))
                packet.pts = packet.dts = pts
                packet.duration = max(1, round(1000000 / fps))
                packet.time_base = time_base
                packet.stream = target
                output.mux(packet)
                previous = pts
                count += 1
    if not count:
        raise ValueError('no_playable_packets; raw_video_preserved')
    temporary.replace(destination)
    return {'path': destination.name, 'sha256': file_hash(destination), 'frames': count,
            'width': width, 'height': height, 'first_timestamp_s': first,
            'duration_s': (previous + max(1, round(1000000 / fps))) / 1000000,
            'leading_packets_skipped': skipped, 'nominal_fps_fallback': fallback,
            'time_basis': 'pc_packet_receive_approximation' if mode == 'h264' else 'received_frame_timestamp',
            'reencoded': False}


class RecordingSink:
    def __init__(self, owner, part):
        self.owner, self.part = owner, part
        self.closed = False

    def offer(self, data, timestamp):
        return not self.closed and self.owner.offer(self.part, data, timestamp)

    def close(self, receiver_stats=None):
        if not self.closed:
            self.closed = True
            self.owner.end_part(self.part, receiver_stats)


class SessionRecording:
    """One writer per session. Overflow preserves a prefix and reports partial capture."""
    def __init__(self, path, camera, input_kind, *, queue_bytes=8 << 20):
        self.path, self.camera, self.input_kind = Path(path), dict(camera), input_kind
        self.directory = self.path / 'recordings'
        self.directory.mkdir()
        self.condition = threading.Condition()
        self.jobs = deque()
        self.limit, self.queued_bytes = queue_bytes, 0
        self.parts = []
        self.bytes_received = self.bytes_written = 0
        self.error = None
        self.disabled = self.finishing = self.finalized = False
        self.input_closed = input_kind != 'tello_camera'
        self.mapping = None
        self.mapping_origin = None
        self.asset_hashes = {}
        self.started_at, self.ended_at = utc_now(), None
        self.finished = threading.Event()
        self._manifest()
        self.worker = threading.Thread(target=self._run, daemon=True, name=self.path.name + '-recording')
        self.worker.start()

    def stats(self):
        with self.condition:
            status = 'partial' if self.error else 'complete' if self.finalized else \
                'finalizing' if self.finishing else 'recording' if self.bytes_received else 'waiting'
            return {'status': status, 'finalized': self.finalized, 'error': self.error,
                    'bytes_received': self.bytes_received, 'bytes_written': self.bytes_written,
                    'queued_bytes': self.queued_bytes, 'queue_limit_bytes': self.limit,
                    'parts': len(self.parts), 'original_h264': any(p['mode'] == 'h264' for p in self.parts),
                    'playback_errors': sum('playback_error' in p for p in self.parts),
                    'session_path': str(self.path), 'manifest': 'session.json'}

    def open_part(self, mode, time_basis):
        if mode not in {'h264', 'received_jpeg'}:
            raise ValueError('invalid_recording_mode')
        with self.condition:
            if self.disabled or self.finishing:
                return None
            if len(self.parts) >= 64:
                self.error, self.disabled = 'recording_part_limit', True
                return None
            part = len(self.parts)
            extension = 'h264' if mode == 'h264' else 'mjpeg'
            self.parts.append({'id': part, 'mode': mode, 'time_basis': time_basis,
                'raw_path': f'recordings/part_{part:04d}.{extension}',
                'index_path': f'recordings/part_{part:04d}.jsonl', 'bytes': 0,
                'status': 'recording', 'original_drone_bitstream': mode == 'h264'})
            self.jobs.append(('open', part))
            self.condition.notify()
            return RecordingSink(self, part)

    def offer(self, part, data, timestamp):
        with self.condition:
            if self.disabled or self.finishing:
                return False
            self.bytes_received += len(data)
            if self.queued_bytes + len(data) > self.limit or len(self.jobs) >= 8192:
                self.error, self.disabled = 'recording_queue_overflow; prefix_preserved', True
                self.condition.notify()
                return False
            self.queued_bytes += len(data)
            self.jobs.append(('data', part, data, timestamp))
            self.condition.notify()
            return True

    def end_part(self, part, receiver_stats=None):
        with self.condition:
            if not self.finishing:
                if isinstance(receiver_stats, dict):
                    self.parts[part]['receiver_bytes'] = receiver_stats.get('bytes')
                    self.parts[part]['receiver_packets'] = receiver_stats.get('packets')
                    if receiver_stats.get('packet_sink_errors', 0):
                        self.error, self.disabled = 'recording_packet_observer_failed', True
                self.jobs.append(('close', part))
                self.condition.notify()

    def set_mapping_origin(self, stamp):
        with self.condition:
            if self.mapping_origin is None:
                self.mapping_origin = stamp

    def finish_input(self):
        with self.condition:
            self.input_closed = True
            self._finish_if_ready()

    def finish_mapping(self, state):
        with self.condition:
            if self.mapping is None:
                self.mapping = {key: state.get(key) for key in ('status', 'session_status', 'stop_reason',
                    'last_error', 'revision', 'revision_path', 'map_available', 'segments')}
            self._finish_if_ready()
            if self.finalized:
                self._manifest()

    def _finish_if_ready(self):
        if self.input_closed and self.mapping is not None and not self.finishing:
            self.finishing = True
            self.jobs.append(('finish',))
            self.condition.notify()

    def wait(self, timeout=30):
        return self.finished.wait(timeout)

    def snapshot(self):
        with self.condition:
            value = {'schema_version': 'mapping-session/1.0', 'session_id': self.path.name,
                'started_at': self.started_at, 'ended_at': self.ended_at, 'input': self.input_kind,
                'mapping_timestamp_zero_monotonic': self.mapping_origin,
                'recording': self.stats(), 'recordings': [dict(part) for part in self.parts],
                'mapping': self.mapping,
                'assets': {'camera': 'camera.json', 'settings': 'settings.json', 'state': 'state.json',
                    'native': 'native.json', 'camera_settings': 'camera.yaml',
                    'timeline': 'timeline.json', 'segments': 'segments.json', 'revisions': 'revisions/',
                    'keyframes': 'frames/', 'tracking': 'tracking.jsonl', 'diagnostics': 'diagnostics.json',
                    'tracking_state': 'tracking.json', 'preview': 'preview.jpg',
                    'events': 'events.jsonl', 'backend_log': 'orb.log'},
                'asset_sha256': dict(self.asset_hashes),
                'scale': 'arbitrary', 'privacy': 'local_only'}
            # The portable manifest never depends on the computer's absolute paths.
            value['recording'].pop('session_path', None)
            return value

    def _manifest(self):
        try:
            atomic_json(self.path / 'session.json', self.snapshot())
        except OSError as error:
            self.error = 'session_manifest_' + type(error).__name__

    def _run(self):
        opened, digests, offsets = {}, {}, {}
        storage_failed = False
        next_flush = time.monotonic() + 1

        def close(part):
            streams = opened.pop(part, None)
            if streams:
                for stream in streams:
                    try:
                        stream.close()
                    except OSError as error:
                        self.error = 'recording_close_' + type(error).__name__
                item = self.parts[part]
                raw = self.path / item['raw_path']
                actual_size = raw.stat().st_size
                digest = file_hash(raw) if storage_failed or actual_size != item['bytes'] else digests[part].hexdigest()
                with self.condition:
                    if item.get('receiver_bytes') is not None:
                        item['capture_complete'] = actual_size == item['receiver_bytes']
                        if not item['capture_complete']:
                            self.error = self.error or 'recording_receiver_bytes_mismatch'
                            self.disabled = True
                    self.bytes_written += actual_size - item['bytes']
                    item.update(bytes=actual_size, sha256=digest,
                                status='partial' if self.error else 'captured')

        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: bool(self.jobs), timeout=.25)
                    job = self.jobs.popleft() if self.jobs else None
                try:
                    if job and job[0] == 'open':
                        part = job[1]
                        item = self.parts[part]
                        opened[part] = ((self.path / item['raw_path']).open('xb'),
                                        (self.path / item['index_path']).open('x', encoding='utf-8'))
                        digests[part], offsets[part] = hashlib.sha256(), 0
                    elif job and job[0] == 'data':
                        if storage_failed:
                            continue
                        _, part, data, stamp = job
                        raw, index = opened[part]
                        raw.write(data)
                        index.write(json.dumps({'offset': offsets[part], 'size': len(data),
                            'timestamp_s': stamp}, allow_nan=False) + '\n')
                        digests[part].update(data)
                        offsets[part] += len(data)
                        with self.condition:
                            self.bytes_written += len(data)
                            self.parts[part]['bytes'] += len(data)
                            self.parts[part].setdefault('first_timestamp_s', stamp)
                            self.parts[part]['last_timestamp_s'] = stamp
                    elif job and job[0] == 'close':
                        close(job[1])
                    elif job and job[0] == 'finish':
                        break
                    if time.monotonic() >= next_flush:
                        for streams in opened.values():
                            for stream in streams:
                                stream.flush()
                        self._manifest()
                        next_flush = time.monotonic() + 1
                except (OSError, ValueError, KeyError) as error:
                    storage_failed = True
                    with self.condition:
                        self.error, self.disabled = 'recording_write_' + type(error).__name__, True
                finally:
                    if job and job[0] == 'data':
                        with self.condition:
                            self.queued_bytes -= len(job[2])
            for part in list(opened):
                try:
                    close(part)
                except OSError as error:
                    self.error = 'recording_close_' + type(error).__name__
            for item in self.parts:
                if not item['bytes']:
                    item['status'] = 'empty'
                    continue
                output = self.directory / f"part_{item['id']:04d}.{'mp4' if item['mode'] == 'h264' else 'mkv'}"
                try:
                    playback = remux_video(self.path / item['raw_path'], self.path / item['index_path'],
                                           output, item['mode'], self.camera.get('fps', 24))
                    playback['path'] = 'recordings/' + playback['path']
                    item['playback'] = playback
                except Exception as error:
                    item['playback_error'] = type(error).__name__ + ': ' + str(error)[:200]
        except Exception as error:
            self.error, self.disabled = 'recording_worker_' + type(error).__name__, True
        finally:
            for streams in opened.values():
                for stream in streams:
                    try:
                        stream.close()
                    except OSError:
                        pass
            for name in ('camera.json', 'settings.json', 'state.json'):
                try:
                    if (self.path / name).is_file():
                        self.asset_hashes[name] = file_hash(self.path / name)
                except OSError as error:
                    self.error = self.error or 'recording_metadata_' + type(error).__name__
            with self.condition:
                self.finalized, self.ended_at = True, utc_now()
            self._manifest()
            self.finished.set()
