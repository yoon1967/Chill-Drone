"""Progressive saved-video reconstruction: native camera pose first, depth second.

No SIFT database, all-pairs matching, or offline report/render pipeline is used.
The map is an AI depth estimate, not a measured survey or a navigation map.
"""
from __future__ import annotations

import argparse
from collections import OrderedDict
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

import cv2
import numpy as np
from PIL import Image, ImageOps

MODEL = 'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf'


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')), encoding='utf-8')
    # Windows readers briefly prevent replacement; retry without exposing half JSON.
    for attempt in range(7):
        try:
            os.replace(temporary, path)
            break
        except PermissionError:
            if attempt == 6:
                raise
            time.sleep(.005 * (attempt + 1))


class DepthEstimator:
    def __init__(self):
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        self.torch = torch
        torch.set_num_threads(2)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        try:
            self.processor = AutoImageProcessor.from_pretrained(MODEL, use_fast=False, local_files_only=True)
            model = AutoModelForDepthEstimation.from_pretrained(MODEL, local_files_only=True)
        except OSError:
            print('Downloading the depth model for first use. Keep internet available until setup finishes.', flush=True)
            self.processor = AutoImageProcessor.from_pretrained(MODEL, use_fast=False)
            model = AutoModelForDepthEstimation.from_pretrained(MODEL)
        self.model = model.to(self.device).eval()
        if self.device == 'cuda':
            self.model.half()

    def prepare(self):
        if not getattr(self, 'prepared', False):
            self.estimate(np.zeros((144, 256, 3), dtype=np.uint8))
            self.prepared = True

    def estimate(self, rgb):
        torch = self.torch
        # A fixed, small network input reduces inference cost, not only output size.
        h, w = rgb.shape[:2]
        nw = max(14, round(w / 14) * 14)
        nh = max(14, round(h / 14) * 14)
        values = self.processor(images=rgb, return_tensors='pt', size={'height': nh, 'width': nw},
                                keep_aspect_ratio=False)['pixel_values'].to(self.device)
        if self.device == 'cuda':
            values = values.half()
        with torch.inference_mode():
            depth = self.model(pixel_values=values).predicted_depth
            depth = torch.nn.functional.interpolate(depth[:, None].float(), size=(h, w),
                                                    mode='bilinear', align_corners=False)[0, 0]
        return depth.cpu().numpy().astype(np.float32)


def frames(source, fps, rate=None, long_side=256):
    """Yield display-oriented RGB images with source timestamps; never pre-extract."""
    source = Path(source)
    next_stamp = 0.
    if source.is_dir():
        from modules.mapping.data.frame_input import frame_paths
        for index, path in enumerate(frame_paths(source)):
            stamp = index / fps
            if rate and stamp + 1e-6 < next_stamp:
                continue
            next_stamp = stamp + 1 / rate if rate else stamp
            with Image.open(path) as image:
                image = ImageOps.exif_transpose(image).convert('RGB')
                original_size = image.size
                image.thumbnail((long_side, long_side))
                yield index + 1, stamp, np.asarray(image), original_size
    else:
        import av
        from modules.mapping.data.video_input import display_rotation
        with av.open(str(source)) as container:
            stream = container.streams.video[0]
            stream.thread_type = 'AUTO'
            stream.codec_context.thread_count = 2
            first = None
            previous = -1.
            for index, frame in enumerate(container.decode(stream)):
                if frame.pts is None or frame.time_base is None:
                    raise ValueError('video_frame_timestamp_missing')
                stamp = float(frame.pts * frame.time_base)
                first = stamp if first is None else first
                stamp -= first
                if stamp <= previous:
                    continue
                previous = stamp
                if rate and stamp + 1e-6 < next_stamp:
                    continue
                next_stamp = stamp + 1 / rate if rate else stamp
                # Decode timestamps for all frames, but convert/resize RGB only for candidates.
                scale = min(1., long_side / max(frame.width, frame.height))
                image = frame.reformat(width=max(32, round(frame.width * scale)),
                    height=max(32, round(frame.height * scale)), format='rgb24').to_image()
                rotation = display_rotation(frame, stream.metadata)
                original_size = (frame.width, frame.height)
                if rotation:
                    image = image.transpose({90: Image.Transpose.ROTATE_90,
                        180: Image.Transpose.ROTATE_180, 270: Image.Transpose.ROTATE_270}[rotation])
                    if rotation in {90, 270}:
                        original_size = original_size[::-1]
                yield index + 1, stamp, np.asarray(image.convert('RGB')), original_size


class PoseFrames:
    """Pull one fixed pose at a time; callers finish its depth/map before pulling again."""
    def __init__(self, source, fps, rate, output, camera=None, tracker_factory=None):
        from modules.mapping.localization.pose_first import OrbPoseTracker
        self.source, self.fps, self.rate = source, fps, rate
        self.output, self.camera = output, camera
        self.tracker_factory = tracker_factory or OrbPoseTracker
        self.native = None
        self.iterator = None
        self.next_output = 0.
        self.dropped = 0
        self.stop = threading.Event()
        self.cancel_event = None

    def cancelled(self):
        return self.stop.is_set() or (self.cancel_event is not None and self.cancel_event.is_set())

    def take(self):
        if self.iterator is None:
            self.iterator = iter(frames(self.source, self.fps, 15, 640))
        while not self.cancelled():
            item = next(self.iterator, None)
            if item is None:
                return None
            number, stamp, rgb, original_size = item
            if self.native is None:
                self.native = self.tracker_factory(self.output, (rgb.shape[1], rgb.shape[0]), original_size, self.camera, 15)
            if self.cancelled():
                return None
            received = time.monotonic()
            # Keep all tracking candidates, even if inference or storage was slow.
            result = self.native.track(rgb, stamp)
            if stamp + 1e-6 < self.next_output:
                continue
            self.next_output = stamp + 1 / self.rate
            return number, stamp, rgb, received, original_size, result
        return None

    def close(self):
        self.stop.set()
        try:
            if self.native:
                self.native.close()
                self.native = None
        finally:
            if self.iterator and hasattr(self.iterator, 'close'):
                self.iterator.close()


class LivePoseFrames(PoseFrames):
    """Request fresh live input only after the preceding output was presented."""
    def __init__(self, receive, fps, rate, output, camera, tracker_factory=None):
        super().__init__('live', fps, rate, output, camera, tracker_factory)
        self.receive = receive
        width, height = camera['width'], camera['height']
        factor = min(1., 640 / max(width, height))
        self.size = (round(width * factor), round(height * factor))
        self.original_size = (width, height)
        # Known live geometry lets Docker finish loading before admitting the first frame.
        self.native = self.tracker_factory(output, self.size, self.original_size, camera, 15)

    def take(self):
        while not self.cancelled():
            packet = self.receive()
            if packet is None:
                return None
            jpeg, stamp, sequence = packet
            image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None or (image.shape[1], image.shape[0]) != self.original_size:
                raise ValueError('frame_dimensions_do_not_match_calibration')
            rgb = cv2.cvtColor(cv2.resize(image, self.size), cv2.COLOR_BGR2RGB)
            received = time.monotonic()
            result = self.native.track(rgb, stamp)
            if stamp + 1e-6 < self.next_output:
                continue
            self.next_output = stamp + 1 / self.rate
            return sequence + 1, stamp, rgb, received, self.original_size, result
        return None


def camera_intrinsic(shape, original_size, camera):
    h, w = shape[:2]
    if camera:
        if (camera['width'], camera['height']) != original_size:
            raise ValueError('frame_dimensions_do_not_match_calibration')
        sx, sy = w / camera['width'], h / camera['height']
        return camera['fx'] * sx, camera['fy'] * sy, camera['cx'] * sx, camera['cy'] * sy
    return w * .8660254, w * .8660254, w / 2, h / 2


def add_points(cells, rgb, depth, pose, params, max_points=40000, voxel_size=.06, seen=None, stamp=0.):
    fx, fy, cx, cy = params
    v, u = np.mgrid[0:depth.shape[0]:4, 0:depth.shape[1]:4]
    z = depth[v, u]
    good = np.isfinite(z) & (z > 0)
    xyz = np.column_stack(((u[good] - cx) * z[good] / fx, (v[good] - cy) * z[good] / fy, z[good]))
    xyz = xyz @ pose[:3, :3].T + pose[:3, 3]
    colors = rgb[v[good], u[good]]
    keys = np.floor(xyz / voxel_size).astype(np.int64).tolist()
    for values, point, color in zip(keys, np.round(xyz, 4).tolist(), colors.tolist()):
        key = tuple(values)
        if seen is not None:
            cells.pop(key, None)
            seen[key] = stamp
        cells[key] = [*point, *color]
    removed = 0
    while len(cells) > max_points:
        if isinstance(cells, OrderedDict):
            key, _ = cells.popitem(last=False)
        else:
            key = next(iter(cells))
            del cells[key]
        if seen is not None:
            seen.pop(key, None)
        removed += 1
    return removed


def expire_render_points(cells, seen, stamp, window_s):
    if window_s is None:
        return 0
    removed = 0
    while cells:
        key = next(iter(cells))
        if seen[key] >= stamp - window_s:
            break
        del cells[key]
        del seen[key]
        removed += 1
    return removed


def run(source, output, options, estimator_factory=DepthEstimator, tracker_factory=None,
        stop_event=None, on_preview=None, live_receive=None):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / 'frames').mkdir()
    started = time.monotonic()
    run_id = options.get('run_id', output.parent.name + '_' + output.name)
    status = {'status': 'running', 'stage': 'model', 'backend': 'depth_rgbd', 'processed': 0,
              'registered': 0, 'dropped': 0, 'revision': 0, 'points': 0, 'timestamp_s': 0.,
              'tracking': 'initializing', 'processing_policy': 'sequential'}
    producer = history = None
    cameras, cells = [], OrderedDict()
    seen = {}
    last_nonempty_points = []
    render_window_s = options.get('render_window_s')
    def check_cancelled():
        if stop_event is not None and stop_event.is_set():
            raise InterruptedError('cancelled')
    def publish(**fields):
        if fields.get('status') not in {'cancelled', 'failed'}:
            check_cancelled()
        status.update(fields, elapsed_seconds=round(time.monotonic() - started, 3))
        write_json(output / 'progress.json', status)
    try:
        from modules.mapping.data.mapping_history import HistoryWriter
        history = HistoryWriter(output, write_json, render_window_s, synchronous=True)
        publish()
        model_started = time.perf_counter()
        estimator = estimator_factory()
        # Warm up before starting playback so model loading never consumes the input.
        if hasattr(estimator, 'prepare'):
            estimator.prepare()
        else:
            estimator.estimate(np.zeros((144, 256, 3), dtype=np.uint8))
        check_cancelled()
        publish(stage='tracking', device=getattr(estimator, 'device', 'test'),
                model_preparation_ms=round((time.perf_counter() - model_started) * 1000, 2))
        producer = (LivePoseFrames(live_receive, options['frame_fps'], min(options['sample_fps'], 10.),
                    output, options['camera'], tracker_factory) if live_receive is not None else
                    PoseFrames(source, options['frame_fps'], min(options['sample_fps'], 10.), output, options.get('camera'), tracker_factory))
        producer.cancel_event = stop_event
        from modules.mapping.localization.pose_first import fixed_camera_pose, depth_in_pose_units
        active_map = None
        submaps = []
        voxel_size = None
        while True:
            check_cancelled()
            item = producer.take()
            check_cancelled()
            if item is None:
                break
            number, stamp, display_rgb, received, original_size, native_pose = item
            tick = time.monotonic()
            # Admit pose before invoking depth. The returned matrix is immutable here.
            publish(stage='tracking', frame_number=number, timestamp_s=stamp, dropped=producer.dropped)
            pose = fixed_camera_pose(native_pose, stamp)
            h, w = display_rgb.shape[:2]
            rgb = cv2.resize(display_rgb, (round(w * 256 / max(w, h)), round(h * 256 / max(w, h))))
            params = camera_intrinsic(rgb.shape, original_size, options.get('camera'))
            status['processed'] += 1
            name = f'frame_{number:07d}.jpg'
            Image.fromarray(display_rgb).save(output / 'frames' / name, quality=80)
            mapped = False
            observation = None
            evicted = 0
            state = 'initializing' if native_pose.get('tracking_state', 0) <= 1 else 'lost'
            depth_scale = None
            if pose is not None:
                state = 'scale_wait'
                publish(stage='depth', pose_confirmed=True)
                if options.get('camera') and any(options['camera'].get('distortion', [])):
                    fx, fy, cx, cy = params
                    matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]])
                    rgb = cv2.undistort(rgb, matrix, np.asarray(options['camera']['distortion']))
                # Estimated meters are converted into the already fixed SLAM coordinate scale.
                depth = estimator.estimate(rgb)
                depth[~np.isfinite(depth) | (depth < .2) | (depth > 8.)] = 0
                aligned = depth_in_pose_units(depth, native_pose.get('depth_anchors', []), (w, h))
                if aligned is not None and np.count_nonzero(depth) > depth.size * .05:
                    depth, depth_scale = aligned
                    map_id = native_pose['map_id']
                    if active_map is not None and active_map != map_id:
                        (output / 'submaps').mkdir(exist_ok=True)
                        archive = f'submaps/{len(submaps):04d}-{active_map}.json'
                        write_json(output / archive, {'map_id': active_map,
                            'points': list(cells.values()), 'cameras': cameras})
                        submaps.append({'map_id': active_map, 'file': archive})
                        cells, cameras, seen = OrderedDict(), [], {}
                        voxel_size = None
                    active_map = map_id
                    voxel_size = voxel_size or max(.0001, .06 * depth_scale)
                    publish(stage='mapping')
                    evicted = add_points(cells, rgb, depth, pose, params,
                        max_points=float('inf') if render_window_s is None else 40000,
                        voxel_size=voxel_size, seen=seen, stamp=stamp)
                    # Full processed observations survive temporal/capacity render eviction.
                    observation = {'depth': depth, 'rgb': rgb, 'world_from_camera': pose,
                        'intrinsics': np.asarray(params), 'map_id': np.asarray(map_id),
                        'depth_scale': np.asarray(depth_scale), 'timestamp_s': np.asarray(stamp)}
                    cameras.append({'image_id': number, 'frame_id': name, 'source_pts_s': stamp,
                        'timestamp_s': stamp, 'position': pose[:3, 3].tolist(),
                        'world_from_camera_rotation': pose[:3, :3].tolist(), 'model': 'PINHOLE',
                        'width': rgb.shape[1], 'height': rgb.shape[0], 'params': list(params)})
                    status['registered'] += 1
                    mapped, state = True, 'tracking'
            status['revision'] += 1
            # Tracking loss must freeze the last valid map, including when an
            # older caller explicitly requests a temporal preview window.
            if mapped:
                evicted += expire_render_points(cells, seen, stamp, render_window_s)
            # One immutable snapshot binds this image and its map to the same revision.
            points = list(cells.values())
            if points:
                last_nonempty_points = points
            preview_points = points[::max(1, math.ceil(len(points) / 12000))]
            map_stamp = cameras[-1]['timestamp_s'] if cameras else stamp
            visible_cameras = [c['position'] for c in cameras
                if render_window_s is None or c['timestamp_s'] >= map_stamp - render_window_s]
            # Sample the entire route instead of dropping its beginning.
            visible_cameras = visible_cameras[::max(1, math.ceil(len(visible_cameras) / 300))]
            snapshot = {'revision': status['revision'], 'frame_id': name,
                'frame_number': number, 'timestamp_s': stamp, 'tracking': state, 'pose_confirmed': pose is not None, 'map_id': active_map, 'pose_map_id': native_pose.get('map_id'),
                'pose_position': pose[:3, 3].tolist() if pose is not None else None,
                'pose_rotation': pose[:3, :3].tolist() if pose is not None else None,
                'depth_scale': depth_scale, 'submaps': len(submaps) + (active_map is not None),
                'points': preview_points, 'map_points': len(points), 'cameras': visible_cameras,
                'render_window_s': render_window_s, 'render_evicted_points': evicted,
                'processed': status['processed'], 'registered': status['registered'],
                'dropped': producer.dropped, 'processing_ms': round((time.monotonic() - tick) * 1000),
                'latency_ms': round((time.monotonic() - received) * 1000)}
            history.append(snapshot, observation)
            check_cancelled()
            if on_preview is None:
                write_json(output / 'preview.json', snapshot)
            else:
                publish(stage='presentation', tracking=state, points=len(points), frame_id=name)
                on_preview(snapshot)
            publish(stage='mapping' if mapped else 'pose_wait', tracking=state, pose_confirmed=pose is not None,
                    points=len(points), frame_id=name, latency_ms=round((time.monotonic() - received) * 1000))
        if not last_nonempty_points or not cameras:
            raise ValueError('no_valid_depth_map')
        producer.close()
        producer = None
        history.close('complete')
        publish(stage='integration', integrated_frames=0, integration_total=status['registered'])
        finalize_history(output, status, run_id, started, publish)
        publish(status='complete', stage='finished')
        return status
    except BaseException as error:
        publish(status='cancelled' if isinstance(error, (KeyboardInterrupt, InterruptedError)) else 'failed', error=str(error)[:400])
        raise
    finally:
        try:
            if producer:
                producer.close()
        finally:
            if history:
                history.close(status['status'])


def finalize_history(output, status, run_id, started, publish=lambda **fields: None):
    from modules.mapping.data.history_fusion import integrate_history
    output = Path(output)
    groups, entries = integrate_history(output, lambda **fields: publish(stage='integration', **fields))
    primary = max(groups.values(), key=lambda group: len(group['cameras']))
    all_registered = sum(len(group['cameras']) for group in groups.values())
    submaps = []
    display_rotation = np.diag([1., -1., -1.])
    for group in groups.values():
        group['rows'] = [[i, p[0], -p[1], -p[2], *p[3:], 0., 1] for i, p in enumerate(group['cells'].values())]
        del group['cells']
        for camera in group['cameras']:
            camera['position'] = (display_rotation @ camera['position']).tolist()
            camera['world_from_camera_rotation'] = (display_rotation @ camera['world_from_camera_rotation']).tolist()
        if len(groups) > 1:
            name = f"submaps/full-{group['map_id']}.json"
            (output / 'submaps').mkdir(exist_ok=True)
            write_json(output / name, {'map_id': group['map_id'], 'points': group['rows'], 'cameras': group['cameras'], 'scope': 'all_recorded_observations'})
            submaps.append({'map_id': group['map_id'], 'file': name, 'points': len(group['rows']), 'frames': len(group['cameras'])})
    rows, cameras = primary['rows'], primary['cameras']
    xyz = np.array([p[1:4] for p in rows])
    partial = len(groups) > 1 or len(cameras) < 2
    quality = {'input_frames': len(entries), 'submitted_frames': len(entries),
        'registered_frames': len(cameras), 'registration_ratio': len(cameras) / len(entries),
        'points3D': len(rows), 'excluded_frames': [e['frame_id'] for e in entries if not e['observation']],
        'mean_reprojection_error_px': None, 'processing_seconds': time.monotonic() - started,
        'result_status': 'partial' if partial else 'complete',
        'partial_reasons': (['unaligned_map_segments'] if len(groups) > 1 else []) + (['insufficient_camera_baseline'] if len(cameras) < 2 else []),
        'bounds': {'min': xyz.min(axis=0).tolist(), 'max': xyz.max(axis=0).tolist()},
        'coverage': {'recorded_frames': len(entries), 'integrated_frames': all_registered,
            'initialization_frames': sum(e['tracking'] == 'initializing' for e in entries),
            'unmapped_frames': sum(not e['observation'] for e in entries), 'map_count': len(groups),
            'spatial_completeness_verified': False}}
    scene = {'schema_version': 'phone-sfm-result/1.2', 'input_run_id': run_id,
        'created_utc': datetime.now(timezone.utc).isoformat(), 'pose_source': 'orb_slam3_depth',
        'points': rows, 'point_fields': ['point_id', 'x', 'y', 'z', 'r', 'g', 'b', 'error_px', 'track_length'],
        'cameras': cameras, 'quality': quality, 'coordinate_system': {'name': 'first_camera_display', 'y': 'view-derived up, not measured gravity'},
        'segment_id': primary['map_id'], 'scale': {'unit': 'arbitrary', 'status': 'monocular_pose_scale', 'factor': 1.},
        'depth_mapping': {'model': MODEL, 'pose_first': True, 'tracker': 'orb_slam3_monocular',
            'submaps': submaps, 'dropped': status.get('dropped', 0), 'metric_accuracy_verified': False,
            'history': 'history/manifest.json', 'integration_scope': 'all_recorded_observations',
            'integrated_frames': all_registered, 'integrated_maps': len(groups),
            'primary_map_selection': 'most_observed_frames', 'preview_window_applied': False,
            'preview_point_limit_applied': False}}
    publish(stage='saving', points=len(rows))
    write_json(output / 'scene.json', scene)
    write_json(output / 'poses.json', {k: v for k, v in scene.items() if k not in ('points', 'point_fields')})
    write_json(output / 'local_assets.json', {'images_root': str(output / 'frames'), 'frames': [c['frame_id'] for c in cameras]})
    with (output / 'points.ply').open('w', encoding='ascii') as stream:
        stream.write(f'ply\nformat ascii 1.0\nelement vertex {len(rows)}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
        stream.writelines(' '.join(map(str, p[1:7])) + '\n' for p in rows)
    with (output / 'trajectory.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['frame_id', 'timestamp_s', 'x', 'y', 'z', 'pose_source', 'unit'])
        writer.writerows([c['frame_id'], c['timestamp_s'], *c['position'], 'orb_slam3_depth', 'arbitrary'] for c in cameras)
    write_json(output / 'status.json', {'status': quality['result_status']})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--options', type=Path, required=True)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    options = json.loads(args.options.read_text(encoding='utf-8'))
    source = args.source
    if source.is_dir():
        videos = [p for p in source.iterdir() if p.suffix.lower() in {'.mp4', '.mov', '.avi', '.mkv'}]
        if len(videos) == 1 and not any(p.suffix.lower() in {'.jpg', '.jpeg', '.png'} for p in source.iterdir()):
            source = videos[0]
    run(source, args.output, options)


if __name__ == '__main__':
    main()
