"""Bounded, asynchronous capture storage. Never performs disk I/O on a video receiver."""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
from fractions import Fraction
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import uuid


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


def remux_video(raw, index, destination, mode, fps, *, clock_samples=None, source_run_id=None):
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
                if clock_samples is not None:
                    clock = (row or {}).get('sensor_clock')
                    if mode == 'h264' and not fallback and source_run_id:
                        clock = {'run_id': source_run_id, 'timestamp_s': stamp}
                    clock_samples.append(clock if not fallback else None)
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

    def offer(self, data, timestamp, *, sensor_clock=None):
        return not self.closed and self.owner.offer(self.part, data, timestamp, sensor_clock=sensor_clock)

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
        self.log_error = None
        self.log_failed = False
        self.log_counts = {'sensor': 0, 'frames': 0}
        self.log_dropped = {'sensor': 0, 'frames': 0}
        self.sensor_cursors = {}
        self.sensor_gaps = 0
        self.sensor_required = False
        self.sensor_samples_written = 0
        self.sync_info = {'path': 'video_sensor_sync.json', 'status': 'pending'}
        self.started_at, self.ended_at = utc_now(), None
        self.finished = threading.Event()
        self.checkpoints = []
        self._manifest()
        self.worker = threading.Thread(target=self._run, daemon=True, name=self.path.name + '-recording')
        self.worker.start()

    def stats(self):
        with self.condition:
            status = 'partial' if self.error or self.log_error else 'complete' if self.finalized else \
                'finalizing' if self.finishing else 'recording' if self.bytes_received else 'waiting'
            return {'status': status, 'finalized': self.finalized, 'error': self.error or self.log_error,
                    'log_error': self.log_error, 'log_records': dict(self.log_counts), 'sensor_gaps': self.sensor_gaps,
                    'log_dropped': dict(self.log_dropped),
                    'sensor_required': self.sensor_required, 'sensor_samples': self.sensor_samples_written,
                    'video_sensor_sync': dict(self.sync_info),
                    'bytes_received': self.bytes_received, 'bytes_written': self.bytes_written,
                    'queued_bytes': self.queued_bytes, 'queue_limit_bytes': self.limit,
                    'parts': len(self.parts), 'original_h264': any(p['mode'] == 'h264' for p in self.parts),
                    'playback_errors': sum('playback_error' in p for p in self.parts),
                    'session_path': str(self.path), 'manifest': 'session.json'}

    def open_part(self, mode, time_basis, *, source_run_id=None):
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
                'status': 'recording', 'original_drone_bitstream': mode == 'h264',
                'source_run_id': source_run_id})
            self.jobs.append(('open', part))
            self.condition.notify()
            return RecordingSink(self, part)

    def offer(self, part, data, timestamp, *, sensor_clock=None):
        with self.condition:
            if self.disabled or self.finishing:
                return False
            self.bytes_received += len(data)
            if self.queued_bytes + len(data) > self.limit or len(self.jobs) >= 8192:
                self.error, self.disabled = 'recording_queue_overflow; prefix_preserved', True
                self.condition.notify()
                return False
            self.queued_bytes += len(data)
            clock = None
            if isinstance(sensor_clock, dict):
                # Persist only source identity/time, never mapping poses or decisions.
                from .video_sensor_sync import finite
                if isinstance(sensor_clock.get('run_id'), str) and finite(sensor_clock.get('timestamp_s')):
                    clock = {'run_id': sensor_clock['run_id'], 'timestamp_s': sensor_clock['timestamp_s']}
            self.jobs.append(('data', part, data, timestamp, clock))
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

    def log_problem(self, reason):
        with self.condition:
            self.log_error = self.log_error or reason

    def require_sensors(self):
        with self.condition:
            self.sensor_required = True

    def log(self, stream, record):
        """Enqueue JSON only; logging failure must never interrupt flight or mapping."""
        try:
            line = json.dumps({'schema_version': 'mapping-' + stream + '/1.0', **record},
                              ensure_ascii=True, allow_nan=False, separators=(',', ':')) + '\n'
        except (TypeError, ValueError):
            self.log_problem('recording_log_invalid_' + stream)
            if stream in self.log_dropped:
                self.log_dropped[stream] += 1
            return False
        with self.condition:
            if self.finishing or self.finalized or self.log_failed:
                if stream in self.log_dropped:
                    self.log_dropped[stream] += 1
                return False
            if stream not in self.log_counts or len(line) > 65536 or self.queued_bytes + len(line) > self.limit or len(self.jobs) >= 8192:
                self.log_error = self.log_error or 'recording_log_queue_overflow'
                if stream in self.log_dropped:
                    self.log_dropped[stream] += 1
                return False
            self.queued_bytes += len(line)
            self.jobs.append(('json', stream, line, record.get('event')))
            self.condition.notify()
            return True

    def sensor_batch(self, batch):
        run_id = batch['run_id']
        after = self.sensor_cursors.get(run_id, 0)
        samples = batch['samples']
        if samples and after and samples[0]['sequence'] > after + 1:
            lost = samples[0]['sequence'] - after - 1
            self.sensor_gaps += lost
            self.log_problem('recording_sensor_gap')
            self.log('sensor', {'event': 'gap', 'source_run_id': run_id, 'after_sequence': after, 'lost': lost})
        for sample in samples:
            if sample['sequence'] > after:
                self.log('sensor', {'event': 'sample', 'source_run_id': run_id, **sample})
                after = sample['sequence']
        self.sensor_cursors[run_id] = max(after, batch['next_sequence'])

    def frame_input(self, sequence, stamp, *, admission, metadata=None, sensor_pose=None, front_tof=None):
        epoch, pose, visual = sensor_pose if sensor_pose is not None else (None, None, False)
        return self.log('frames', {'event': 'input', 'frame_sequence': sequence, 'timestamp_s': stamp,
            'admission': admission, 'mapping_applied': False, 'source': metadata,
            'sensor_epoch': epoch, 'sensor_pose': list(pose) if pose is not None else None,
            'allow_visual_origin': visual, 'front_tof': front_tof})

    def frame_result(self, snapshot):
        position, rotation = snapshot.get('pose_position'), snapshot.get('pose_rotation')
        transform = ([list(row) + [position[i]] for i, row in enumerate(rotation)] + [[0., 0., 0., 1.]]
                     if position is not None and rotation is not None else None)
        self.log('frames', {'event': 'mapping', 'frame_sequence': snapshot['frame_number'] - 1,
            **{key: snapshot.get(key) for key in ('frame_id', 'timestamp_s', 'revision', 'map_id', 'pose_map_id',
                'tracking', 'pose_source', 'map_unit', 'depth_scale', 'mapping_applied')},
            'pose_source': snapshot.get('pose_source') or ('orb_slam3_monocular' if transform is not None else None),
            'world_from_camera': transform})

    def finish_input(self):
        with self.condition:
            self.input_closed = True
            self._finish_if_ready()

    def finish_mapping(self, state):
        with self.condition:
            if self.mapping is None:
                self.mapping = {key: state.get(key) for key in ('status', 'session_status', 'stop_reason',
                    'last_error', 'revision', 'revision_path', 'map_available', 'segments')}
                if state.get('map_unit') == 'm':
                    self.mapping['map_unit'] = 'm'
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

    def checkpoint(self, map_pin):
        """Queue a prefix barrier. Receivers keep appending after this marker."""
        with self.condition:
            if self.finishing or self.finalized:
                raise ValueError('checkpoint_session_finished')
            if any(job['status'] == 'saving' for job in self.checkpoints):
                raise ValueError('checkpoint_busy')
            if self.disabled or not self.bytes_received:
                raise ValueError('checkpoint_video_not_ready')
            job = {'id': 'saved_' + datetime.now().strftime('%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:8],
                   'status': 'saving', 'cutoff_at': utc_now(), 'cutoff_monotonic': time.monotonic(),
                   'source_session_id': self.path.name, 'map': map_pin, 'error': None}
            self.checkpoints.append(job)
            self.jobs.append(('checkpoint', job))
            self.condition.notify()
            return self.checkpoint_status(job['id'])

    def checkpoint_status(self, key=None):
        with self.condition:
            jobs = self.checkpoints if key is None else [job for job in self.checkpoints if job['id'] == key]
            if not jobs:
                return None
            job = jobs[-1]
            return {name: value for name, value in job.items() if name != 'map'}

    def snapshot(self):
        with self.condition:
            value = {'schema_version': 'mapping-session/1.0', 'session_id': self.path.name,
                'started_at': self.started_at, 'ended_at': self.ended_at, 'input': self.input_kind,
                'mapping_timestamp_zero_monotonic': self.mapping_origin,
                'capture_metadata': {'schema_version': 'mapping-capture/1.0',
                    'sensor_time_basis': 'source_pc_receive_monotonic_seconds',
                    'frame_time_basis': ('browser_media_time' if self.input_kind == 'browser_camera'
                                         else 'mapping_input_receive_monotonic_minus_mapping_origin'),
                    'sensor_pose_fields': ['segment', 'x_m', 'y_m', 'z_m', 'yaw_rad', 'pitch_rad', 'roll_rad'],
                    'sensor_axes': 'power_on_forward_left_up', 'camera_axes': 'x_right_y_down_z_forward',
                    'raw_units': 'Tello state packet units; preserved without conversion',
                    'frame_results': 'fold frames.jsonl by frame_sequence; input then mapping result',
                    'unprocessed_frames': 'mapping_applied=false unless a mapping event overrides it'},
                'recording': self.stats(), 'recordings': [dict(part) for part in self.parts],
                'mapping': self.mapping,
                'assets': {'camera': 'camera.json', 'settings': 'settings.json', 'state': 'state.json',
                    'sensor': 'sensor.jsonl', 'frame_poses': 'frames.jsonl',
                    'video_sensor_sync': 'video_sensor_sync.json',
                    'native': 'native.json', 'camera_settings': 'camera.yaml',
                    'timeline': 'timeline.json', 'segments': 'segments.json', 'revisions': 'revisions/',
                    'keyframes': 'frames/', 'tracking': 'tracking.jsonl', 'diagnostics': 'diagnostics.json',
                    'tracking_state': 'tracking.json', 'preview': 'preview.jpg',
                    'events': 'events.jsonl', 'backend_log': 'orb.log'},
                'asset_sha256': dict(self.asset_hashes),
                'scale': (self.mapping or {}).get('map_unit', 'arbitrary'), 'privacy': 'local_only'}
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
        logs = {}
        clocks_by_part = {}
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
            for stream in self.log_counts:
                try:
                    logs[stream] = (self.path / (stream + '.jsonl')).open('x', encoding='utf-8')
                except OSError:
                    self.log_failed = True
                    self.log_problem('recording_log_write_failed')
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: bool(self.jobs), timeout=.25)
                    job = self.jobs.popleft() if self.jobs else None
                try:
                    if job and job[0] == 'json':
                        try:
                            if job[1] not in logs or self.log_failed:
                                self.log_dropped[job[1]] += 1
                                continue
                            logs[job[1]].write(job[2])
                            self.log_counts[job[1]] += 1
                            if job[1] == 'sensor' and job[3] == 'sample':
                                self.sensor_samples_written += 1
                        except OSError:
                            self.log_dropped[job[1]] += 1
                            self.log_failed = True
                            self.log_problem('recording_log_write_failed')
                    elif job and job[0] == 'open':
                        part = job[1]
                        item = self.parts[part]
                        opened[part] = ((self.path / item['raw_path']).open('xb'),
                                        (self.path / item['index_path']).open('x', encoding='utf-8'))
                        digests[part], offsets[part] = hashlib.sha256(), 0
                    elif job and job[0] == 'data':
                        if storage_failed:
                            continue
                        _, part, data, stamp, clock = job
                        raw, index = opened[part]
                        raw.write(data)
                        index.write(json.dumps({'offset': offsets[part], 'size': len(data),
                            'timestamp_s': stamp, **({'sensor_clock': clock} if clock else {})}, allow_nan=False) + '\n')
                        digests[part].update(data)
                        offsets[part] += len(data)
                        with self.condition:
                            self.bytes_written += len(data)
                            self.parts[part]['bytes'] += len(data)
                            self.parts[part].setdefault('first_timestamp_s', stamp)
                            self.parts[part]['last_timestamp_s'] = stamp
                    elif job and job[0] == 'close':
                        close(job[1])
                    elif job and job[0] == 'checkpoint':
                        checkpoint = job[1]
                        try:
                            from .live_checkpoint import save_checkpoint
                            for streams in opened.values():
                                for stream in streams:
                                    stream.flush()
                            for stream in logs.values():
                                stream.flush()
                            # Sizes are fixed here, before any later queued packet/log is written.
                            with self.condition:
                                manifest = deepcopy(self.snapshot())
                                parts = [deepcopy(part) for part in self.parts if part['bytes']]
                            names = ['sensor.jsonl', 'frames.jsonl']
                            names += [part[field] for part in parts for field in ('raw_path', 'index_path')]
                            sizes = {name: (self.path / name).stat().st_size for name in names if (self.path / name).is_file()}
                            prefix = {'manifest': manifest, 'parts': parts, 'sizes': sizes}
                            threading.Thread(target=save_checkpoint, args=(self, checkpoint, prefix),
                                daemon=True, name=checkpoint['id']).start()
                        except Exception as error:
                            with self.condition:
                                checkpoint.update(status='failed', error=str(error)[:200])
                    elif job and job[0] == 'finish':
                        break
                    if time.monotonic() >= next_flush:
                        for streams in opened.values():
                            for stream in streams:
                                stream.flush()
                        for stream in logs.values():
                            try:
                                stream.flush()
                            except OSError:
                                self.log_failed = True
                                self.log_problem('recording_log_write_failed')
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
                    elif job and job[0] == 'json':
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
                    clocks_by_part[item['id']] = []
                    playback = remux_video(self.path / item['raw_path'], self.path / item['index_path'],
                                           output, item['mode'], self.camera.get('fps', 24),
                                           clock_samples=clocks_by_part[item['id']], source_run_id=item.get('source_run_id'))
                    playback['path'] = 'recordings/' + playback['path']
                    item['playback'] = playback
                except Exception as error:
                    item['playback_error'] = type(error).__name__ + ': ' + str(error)[:200]
        except Exception as error:
            self.error, self.disabled = 'recording_worker_' + type(error).__name__, True
        finally:
            if self.sensor_required and self.bytes_received and not self.sensor_samples_written:
                self.log_problem('recording_sensor_samples_missing')
            for stream in logs.values():
                try:
                    stream.close()
                except OSError:
                    self.log_problem('recording_log_write_failed')
            for streams in opened.values():
                for stream in streams:
                    try:
                        stream.close()
                    except OSError:
                        pass
            try:
                from .video_sensor_sync import write_sync
                sync = write_sync(self.path, self.parts, clocks_by_part, self.sensor_required)
                self.sync_info['status'] = sync['status']
                if self.sensor_required and self.bytes_received and sync['status'] in {'unavailable', 'partial'}:
                    self.log_problem('recording_video_sensor_sync_incomplete')
            except Exception as error:
                self.sync_info.update(status='unavailable', error=type(error).__name__)
                self.log_problem('recording_video_sensor_sync_' + type(error).__name__)
            for name in ('camera.json', 'settings.json', 'state.json', 'sensor.jsonl', 'frames.jsonl', 'video_sensor_sync.json'):
                try:
                    if (self.path / name).is_file():
                        self.asset_hashes[name] = file_hash(self.path / name)
                except OSError as error:
                    self.error = self.error or 'recording_metadata_' + type(error).__name__
            with self.condition:
                self.finalized, self.ended_at = True, utc_now()
            self._manifest()
            self.finished.set()
