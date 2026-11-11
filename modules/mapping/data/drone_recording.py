"""Replay a finalized Tello recording through the live metric pose pipeline."""

from collections import defaultdict
from bisect import bisect_left
import json
import math
from pathlib import Path
import time

from modules.autonomy.tello_link.sensor_pose import TelloOdometry, SensorPose
from .camera_profiles import validate_intrinsics
from .session_import import inside, read_json


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def rows(path):
    with path.open(encoding='utf-8') as file:
        for line in file:
            if line.strip():
                yield json.loads(line)


class DroneRecording:
    def __init__(self, root):
        self.root = Path(root)
        manifest = read_json(inside(self.root, 'session.json'))
        if (manifest.get('input') != 'tello_camera' or manifest.get('schema_version') != 'mapping-session/1.0'
                or manifest.get('recording', {}).get('finalized') is not True):
            raise ValueError('drone_recording_not_finalized')
        assets = manifest.get('assets', {})
        self.camera = validate_intrinsics(read_json(inside(self.root, assets.get('camera', 'camera.json'))))
        self.frames_path = inside(self.root, assets.get('frame_poses', 'frames.jsonl'))
        self.sensor_path = inside(self.root, assets.get('sensor', 'sensor.jsonl'))
        if not self.frames_path.is_file() or not self.sensor_path.is_file():
            raise ValueError('drone_recording_missing_logs')
        self.parts = []
        self.unavailable_parts = 0
        for part in manifest.get('recordings', []):
            playback = part.get('playback') or {}
            mode = part.get('mode')
            if mode not in {'received_jpeg', 'h264'} or not playback.get('path'):
                self.unavailable_parts += 1
                continue
            video = inside(self.root, playback['path'])
            if not video.is_file() or playback.get('nominal_fps_fallback'):
                self.unavailable_parts += 1
                continue
            first, duration, count = (playback.get(key) for key in ('first_timestamp_s', 'duration_s', 'frames'))
            if not (type(part.get('id')) is int and number(first) and number(duration) and duration > 0
                    and type(count) is int and count > 0):
                raise ValueError('drone_recording_invalid_timing')
            if (playback.get('width'), playback.get('height')) != (self.camera['width'], self.camera['height']):
                raise ValueError('frame_dimensions_do_not_match_calibration')
            self.parts.append((part['id'], mode, video, first, duration, count))
        if not self.parts:
            raise ValueError('drone_recording_missing_video')
        self.parts.sort(key=lambda part: part[3])
        self.started_at = manifest.get('started_at')

    def summary(self):
        return dict(kind='drone_recording', name=self.root.name, started_at=self.started_at,
                    width=self.camera['width'], height=self.camera['height'],
                    unavailable_parts=self.unavailable_parts,
                    frames=sum(part[5] for part in self.parts),
                    duration_s=max(part[3] + part[4] for part in self.parts) - self.parts[0][3])

    def load_telemetry(self):
        frames = defaultdict(dict)
        for row in rows(self.frames_path):
            if row.get('event') != 'input':
                continue
            source = row.get('source') or {}
            part = source.get('recording_part')
            if type(part) is not int:
                continue
            index = source.get('recording_frame_index')
            if type(index) is int and source.get('video_recorded') is True:
                if index in frames[part]:
                    raise ValueError('drone_recording_invalid_timing')
                frames[part][index] = row
            elif source.get('video_match') == 'h264_receive_time_approximation' and number(source.get('timestamp_s')):
                frames[part][source['timestamp_s']] = row
        samples = defaultdict(list)
        for row in rows(self.sensor_path):
            stamp = row.get('received_mono')
            if (row.get('event') == 'sample' and number(stamp) and isinstance(row.get('raw'), dict)
                    and isinstance(row.get('source_run_id'), str)):
                samples[row['source_run_id']].append((stamp, row['raw']))
        return frames, {run: sorted(values, key=lambda item: item[0]) for run, values in samples.items()}

    def packets(self, rate, long_side, cancelled):
        import av
        import numpy as np
        from modules.mapping.localization.tello_input import FRONT_TOF_MAX_FRAME_GAP_S
        from .tof_depth import FRONT_TOF_MAX_M, FRONT_TOF_MIN_M

        frame_logs, samples = self.load_telemetry()
        origin = self.parts[0][3]
        next_stamp, previous, offset = 0., -1., 0
        size = self.camera['width'], self.camera['height']
        for part, mode, video, first, _, count in self.parts:
            logs = frame_logs[part]
            # Stored poses retain the live integration origin. Raw replay starts
            # at the first available sample, so keep its origin separate.
            recorded_epochs = {
                self.epoch(row) for row in logs.values() if SensorPose.from_values(row.get('sensor_pose')) is not None
            } if mode == 'received_jpeg' else set()
            direct_runs = {row['source'].get('run_id') for row in logs.values()}
            direct_run = next(iter(direct_runs)) if len(direct_runs) == 1 else None
            direct_stamps = sorted(logs) if mode == 'h264' else []
            raw_poses = {}
            with av.open(str(video)) as container:
                stream = container.streams.video[0]
                decoded = 0
                for index, frame in enumerate(container.decode(stream)):
                    if cancelled():
                        return
                    decoded += 1
                    if frame.pts is None or frame.time_base is None or index >= count:
                        raise ValueError('drone_recording_invalid_timing')
                    capture = first + float(frame.pts * frame.time_base)
                    stamp = max(capture - origin, previous + 1e-6)
                    previous = stamp
                    row = logs.get(index) if mode == 'received_jpeg' else None
                    if mode == 'h264' and direct_stamps:
                        position = bisect_left(direct_stamps, capture)
                        candidates = direct_stamps[max(0, position - 1):position + 1]
                        nearest = min(candidates, key=lambda value: abs(value - capture))
                        if abs(nearest - capture) <= FRONT_TOF_MAX_FRAME_GAP_S:
                            row = logs[nearest]
                    if row is not None and mode == 'received_jpeg':
                        received = row['source'].get('receive_timestamp_s')
                        # Matroska rounds PTS to milliseconds. An index mismatch
                        # must never apply another frame's recorded sensor pose.
                        if not number(received) or abs(received - capture) > .002:
                            raise ValueError('drone_recording_invalid_timing')
                    if stamp + 1e-6 < next_stamp:
                        continue
                    next_stamp = stamp + 1 / rate
                    if (frame.width, frame.height) != size:
                        raise ValueError('frame_dimensions_do_not_match_calibration')
                    source = row['source'] if row else {}
                    epoch = self.epoch(row) if row and mode == 'received_jpeg' else (source.get('run_id', direct_run),)
                    if epoch in recorded_epochs:
                        pose = SensorPose.from_values(row.get('sensor_pose'))
                    else:
                        run = source.get('run_id', direct_run)
                        if run not in raw_poses:
                            raw_poses[run] = RawPoses(samples.get(run, ()))
                        source_stamp = capture if mode == 'h264' else source.get('timestamp_s')
                        pose = raw_poses[run].at(source_stamp) if row or mode == 'h264' else None
                        epoch = ('raw', epoch)
                    tof = None
                    reading = row.get('front_tof') if row else None
                    if (isinstance(reading, list) and len(reading) == 2 and all(map(number, reading))
                            and abs(reading[1] - capture) <= FRONT_TOF_MAX_FRAME_GAP_S
                            and FRONT_TOF_MIN_M <= reading[0] < FRONT_TOF_MAX_M):
                        tof = (reading[0], stamp + reading[1] - capture)
                    scale = min(1., long_side / max(size))
                    rgb = np.asarray(frame.reformat(width=max(32, round(size[0] * scale)),
                        height=max(32, round(size[1] * scale)), format='rgb24').to_image())
                    yield offset + index + 1, stamp, rgb, time.monotonic(), size, {
                        'tello_telemetry': (epoch, pose, False), 'map_id': 0,
                    }, tof
                if decoded != count:
                    raise ValueError('drone_recording_invalid_timing')
            offset += count

    @staticmethod
    def epoch(row):
        source = row['source']
        return source.get('run_id'), source.get('connection'), row.get('sensor_epoch')


class RawPoses:
    def __init__(self, samples):
        self.samples = iter(samples)
        self.next_sample = next(self.samples, None)
        self.odometry = TelloOdometry()

    def at(self, stamp):
        if not number(stamp):
            return None
        while self.next_sample and self.next_sample[0] <= stamp:
            sample_stamp, raw = self.next_sample
            self.odometry.update(raw, sample_stamp)
            self.next_sample = next(self.samples, None)
        return self.odometry.at(stamp)


class DroneRecordingFrames:
    dropped = 0

    def __init__(self, recording, rate, cancelled):
        self.frames = recording.packets(rate, 640, cancelled)

    def take(self):
        return next(self.frames, None)

    def close(self):
        self.frames.close()
