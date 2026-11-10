"""Progressive saved-video reconstruction: native camera pose first, depth second.

No SIFT database, all-pairs matching, or offline report/render pipeline is used.
The map is an AI depth estimate, not a measured survey or a navigation map.
"""
from __future__ import annotations

import argparse
from collections import OrderedDict
from contextlib import closing
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import cv2
import numpy as np
from PIL import Image, ImageOps

from .model_assets import DEPTH_MODEL as MODEL, cached_snapshot, ModelAssetsMissing
from .mapping_limits import MAX_FRAME_PREVIEW_POINTS
from .point_fusion import BLUE_DEPTH_MIN_M, FAR_DEPTH, STANDARD_DEPTH, TOF_DEPTH
from .tof_depth import TofDepth, match_front_surface

LEARNED_TRACKING_LONG_SIDE = 512


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
    def __init__(self, *, allow_download=False, location=None):
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        self.torch = torch
        torch.set_num_threads(2)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        if location is None:
            try:
                location = cached_snapshot(MODEL)
            except ModelAssetsMissing:
                if not allow_download:
                    raise
                location = MODEL
        location = str(location)
        local_only = not allow_download or location != MODEL
        self.processor = AutoImageProcessor.from_pretrained(location, use_fast=False, local_files_only=local_only)
        model = AutoModelForDepthEstimation.from_pretrained(location, local_files_only=local_only)
        self.model = model.to(self.device).eval()
        if self.device == 'cuda':
            self.model.half()

    def prepare(self, shape=(144, 256, 3)):
        # CUDA kernels/allocations depend on the network input shape. Preparing
        # 256px alone leaves the first 384/518px map frame cold.
        shapes = getattr(self, 'prepared_shapes', set())
        key = tuple(max(14, round(n / 14) * 14) for n in shape[:2])
        if key not in shapes:
            self.estimate(np.zeros(shape, dtype=np.uint8))
            shapes.add(key)
            self.prepared_shapes = shapes

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
        result = depth.cpu().numpy().astype(np.float32)
        self.last_inference = time.monotonic()
        return result

    def keep_warm(self, shape):
        # After long pose loss, WDDM/CUDA idle wake-up can cost ~700ms on the
        # measured laptop. A bounded synthetic inference while waiting keeps
        # the next real frame responsive; it never enters the map/history.
        if self.device == 'cuda' and time.monotonic() - self.last_inference >= .75:
            self.estimate(np.zeros(shape, np.uint8))


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
                    if rate:
                        raise ValueError('video_frame_timestamp_missing')
                    stamp = max(0., previous + 1 / fps)
                else:
                    stamp = float(frame.pts * frame.time_base)
                    first = stamp if first is None else first
                    stamp -= first
                if stamp <= previous:
                    if rate:
                        continue
                    stamp = previous + 1e-6
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


def scaled_size(width, height, long_side):
    scale = long_side / max(width, height)
    return max(1, round(width * scale)), max(1, round(height * scale))


class PoseFrames:
    """Pull one fixed pose at a time; callers finish its depth/map before pulling again."""
    def __init__(self, source, fps, rate, output, camera=None, tracker_factory=None, *, predict_pose=True):
        from modules.mapping.localization.pose_first import ConstantVelocityPose, OrbPoseTracker
        self.source, self.fps, self.rate = source, fps, rate
        self.output, self.camera = output, camera
        self.tracker_factory = tracker_factory or OrbPoseTracker
        self.motion = ConstantVelocityPose() if predict_pose else None
        self.on_tracker_ready = None
        self.native = None
        self.iterator = None
        self.next_output = 0.
        self.track_all_frames = rate is None
        self.tracking_rate = None
        self.tracking_size = 640
        self.trajectory = None
        self.tracking_epoch = 0
        self.had_native_pose = False
        self.legacy_dpvo_factory = self.dpvo = self.dpvo_alignment = None
        self.dpvo_status = None
        self.dropped = 0
        self.stop = threading.Event()
        self.cancel_event = None

    def cancelled(self):
        return self.stop.is_set() or (self.cancel_event is not None and self.cancel_event.is_set())

    def track(self, rgb, stamp):
        if self.trajectory is not None:
            return self.trajectory.pop(stamp)
        result = self.native.track(rgb, stamp)
        if self.motion is not None:
            prediction, first = self.motion.update(result, stamp)
            if prediction is not None:
                return {**result, 'predicted_pose': prediction, 'prediction_start': first}
        return result

    def _start_dpvo(self, rgb, original_size):
        from modules.mapping.localization.pose_models import HYBRID
        try:
            if self.legacy_dpvo_factory is None:
                self.dpvo = start_learned_tracker(HYBRID, rgb, original_size, self.camera, self.cancelled)
            else:
                self.dpvo = self.legacy_dpvo_factory(rgb.shape[1::-1],
                    camera_intrinsic(rgb.shape, original_size, self.camera),
                    (self.camera or {}).get('distortion', []), self.cancelled)
        except InterruptedError:
            raise
        except (OSError, RuntimeError, ValueError, subprocess.SubprocessError):
            if self.legacy_dpvo_factory is None:
                raise
            self.dpvo_status = 'failed'
        else:
            self.dpvo_status = 'active' if self.dpvo is not None else 'unavailable'

    def _track_dpvo(self, rgb, stamp, result):
        from modules.mapping.localization.pose_first import fixed_camera_pose
        try:
            pose = self.dpvo.track(rgb, stamp)
        except InterruptedError:
            raise
        except (OSError, RuntimeError, ValueError, subprocess.SubprocessError):
            if self.legacy_dpvo_factory is None:
                raise
            dpvo, self.dpvo = self.dpvo, None
            self.dpvo_status = 'failed'
            dpvo.close()
            return result
        if self.legacy_dpvo_factory is None:
            pose = fixed_camera_pose(pose, stamp)
        if pose is None:
            return result
        native_pose = fixed_camera_pose(result, stamp)
        if native_pose is not None:
            self.dpvo_alignment.observe(stamp, (result.get('map_id'), self.tracking_epoch), native_pose, pose)
            return result
        return {**result, 'dpvo_fallback': self.dpvo_alignment.project(stamp, pose)}

    def take(self):
        if self.iterator is None:
            rate = self.tracking_rate if self.track_all_frames else 15
            self.iterator = iter(frames(self.source, self.fps, rate, self.tracking_size))
        while not self.cancelled():
            item = next(self.iterator, None)
            if item is None:
                return None
            number, stamp, rgb, original_size = item
            if self.native is None and self.trajectory is None:
                self.native = self.tracker_factory(self.output, rgb.shape[1::-1], original_size, self.camera,
                                                   self.tracking_rate or (self.fps if self.track_all_frames else 15))
                if self.dpvo_alignment is not None:
                    self._start_dpvo(rgb, original_size)
                if self.on_tracker_ready:
                    self.on_tracker_ready()
            if self.cancelled():
                return None
            received = time.monotonic()
            # Keep all tracking candidates, even if inference or storage was slow.
            result = self.track(rgb, stamp)
            if self.track_all_frames:
                tracking_state = result.get('tracking_state', 0)
                if tracking_state == 2:
                    self.had_native_pose = True
                elif tracking_state <= 1 and self.had_native_pose and self.trajectory is None:
                    # ORB can reset its active map without changing the numeric map ID.
                    self.tracking_epoch += 1
                    self.had_native_pose = False
                result = {**result, 'tracking_epoch': self.tracking_epoch}
                if self.dpvo is not None:
                    result = self._track_dpvo(rgb, stamp, result)
            if self.rate is None:
                return number, stamp, rgb, received, original_size, result
            if stamp + 1e-6 < self.next_output and not result.get('prediction_start'):
                continue
            if self.track_all_frames:
                # Use absolute source times so timing differences between frames do not accumulate.
                self.next_output = (math.floor((stamp + 1e-6) * self.rate) + 1) / self.rate
            else:
                self.next_output = stamp + 1 / self.rate
            return number, stamp, rgb, received, original_size, result
        return None

    def close(self):
        self.stop.set()
        try:
            try:
                if self.dpvo is not None:
                    dpvo, self.dpvo = self.dpvo, None
                    dpvo.close()
            finally:
                if self.native:
                    self.native.close()
                self.native = None
        finally:
            if self.iterator and hasattr(self.iterator, 'close'):
                self.iterator.close()


class LivePoseFrames(PoseFrames):
    """Request fresh live input only after the preceding output was presented."""
    def __init__(self, receive, fps, rate, output, camera, tracker_factory=None, *, continuous=False,
                 predict_pose=False, prefer_sensor_pose=False):
        super().__init__('live', fps, rate, output, camera, tracker_factory, predict_pose=predict_pose)
        self.receive = receive
        width, height = camera['width'], camera['height']
        self.size = scaled_size(width, height, min(640, max(width, height)))
        self.original_size = (width, height)
        self.continuous = continuous
        self.prefer_sensor_pose = prefer_sensor_pose
        self.tello_mode = None
        if not prefer_sensor_pose:
            self._start_visual_tracker()
        self.condition = threading.Condition()
        self.pending = None
        self.pending_prediction = None
        self.ended = False
        self.failure = None
        self.tracked = self.sampled_out = 0
        self.tracking_thread = None
        if continuous:
            try:
                receive.tracking_stop = self.stop
            except (AttributeError, TypeError):
                # Bound methods and callable objects can disallow attributes.
                pass
            self.tracking_thread = threading.Thread(target=self._track_live, daemon=True)
            self.tracking_thread.start()

    def _start_visual_tracker(self):
        self.native = self.tracker_factory(self.output, self.size, self.original_size, self.camera,
                                           self.fps if self.continuous else 15)

    def _track_pose(self, rgb, stamp, telemetry):
        if self.tello_mode is None:
            self.tello_mode = self.prefer_sensor_pose and telemetry is not None
        if self.tello_mode:
            # The fusion consumer owns the metric frame and its depth references.
            return {'tello_telemetry': telemetry or (None, None, False), 'map_id': 0}
        if self.native is None:
            self._start_visual_tracker()
        return self.track(rgb, stamp)

    def _track_live(self):
        try:
            while not self.cancelled():
                packet = self._tracked_packet()
                if packet is None:
                    break
                prediction_start = packet[5].get('prediction_start')
                if packet[1] + 1e-6 < self.next_output and not prediction_start:
                    continue
                self.next_output = packet[1] + 1 / self.rate
                with self.condition:
                    if self.pending is not None:
                        self.sampled_out += 1
                    if prediction_start:
                        # Keep the first lost frame while depth is busy, then
                        # resume with the newest available mapping candidate.
                        if self.pending_prediction is not None:
                            self.sampled_out += 1
                        self.pending_prediction, self.pending = packet, None
                    else:
                        self.pending = packet
                    self.condition.notify_all()
        except BaseException as error:
            self.failure = error
        finally:
            with self.condition:
                self.ended = True
                self.condition.notify_all()

    def _tracked_packet(self):
        packet = self.receive()
        if packet is None:
            return None
        jpeg, stamp, sequence, *sensor = packet
        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None or (image.shape[1], image.shape[0]) != self.original_size:
            raise ValueError('frame_dimensions_do_not_match_calibration')
        rgb = cv2.cvtColor(cv2.resize(image, self.size), cv2.COLOR_BGR2RGB)
        received = time.monotonic()
        result = self._track_pose(rgb, stamp, sensor[1] if len(sensor) > 1 else None)
        self.tracked += 1
        return sequence + 1, stamp, rgb, received, self.original_size, result, *sensor

    def take(self):
        if self.continuous:
            with self.condition:
                self.condition.wait_for(lambda: self.pending_prediction is not None or self.pending is not None
                    or self.ended or self.cancelled(), timeout=30)
                if self.failure:
                    raise self.failure
                if self.pending_prediction is not None:
                    result, self.pending_prediction = self.pending_prediction, None
                    return result
                result, self.pending = self.pending, None
                return result
        while not self.cancelled():
            result = self._tracked_packet()
            if result is None:
                return None
            if result[1] + 1e-6 < self.next_output and not result[5].get('prediction_start'):
                continue
            self.next_output = result[1] + 1 / self.rate
            return result
        return None

    def close(self):
        self.stop.set()
        if self.tracking_thread:
            self.tracking_thread.join(timeout=2)
        super().close()


def camera_intrinsic(shape, original_size, camera):
    h, w = shape[:2]
    if camera:
        if (camera['width'], camera['height']) != original_size:
            raise ValueError('frame_dimensions_do_not_match_calibration')
        sx, sy = w / camera['width'], h / camera['height']
        return camera['fx'] * sx, camera['fy'] * sy, camera['cx'] * sx, camera['cy'] * sy
    return w * .8660254, w * .8660254, w / 2, h / 2


def start_learned_tracker(model, rgb, original_size, camera, cancelled, assets=None):
    from modules.mapping.localization.pose_models import LearnedPoseTracker

    return LearnedPoseTracker(model, rgb.shape[1::-1], camera_intrinsic(rgb.shape, original_size, camera),
                              (camera or {}).get('distortion', []), cancelled, assets)


def scan_trajectory(model, source, fps, rate, options, cancelled, publish):
    """Finish corrections before any depth observation is fixed in map space."""
    from modules.mapping.localization.pose_models import VISTA

    assets = options.get('pose_model_location') if model == VISTA else None
    if model == VISTA and not assets:
        from .reconstruction_models import pose_location
        assets = pose_location(model)
        if assets is None:
            raise ValueError('reconstruction_models_not_ready')
    tracker = None
    stamps = []
    last_progress = 0.
    publish(stage='pose_scan', pose_processed=0)
    try:
        with closing(frames(source, fps, rate, LEARNED_TRACKING_LONG_SIDE)) as inputs:
            for _, stamp, rgb, original_size in inputs:
                if tracker is None:
                    tracker = start_learned_tracker(model, rgb, original_size, options.get('camera'),
                                                    cancelled, Path(assets) if assets else None)
                tracker.track(rgb, stamp)
                stamps.append(stamp)
                now = time.monotonic()
                if now - last_progress >= .25:
                    publish(stage='pose_scan', pose_processed=len(stamps), pose_timestamp_s=stamp)
                    last_progress = now
        if tracker is None:
            raise ValueError('no_valid_camera_trajectory')
        publish(stage='pose_optimization', pose_processed=len(stamps))
        trajectory = tracker.finish(stamps)
        if not any(result.get('tracking_state') == 2 for result in trajectory.values()):
            raise ValueError('no_valid_camera_trajectory')
        return trajectory
    finally:
        if tracker is not None:
            tracker.close()


def depth_frame(display_rgb, original_size, options):
    rgb = cv2.resize(display_rgb, scaled_size(*original_size, options['depth_resolution']))
    camera = options.get('camera')
    params = camera_intrinsic(rgb.shape, original_size, camera)
    if camera and any(camera.get('distortion', [])):
        fx, fy, cx, cy = params
        matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]])
        rgb = cv2.undistort(rgb, matrix, np.asarray(camera['distortion']))
    return rgb, params


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


def frame_depth_preview(rgb, depth, params, pose=None, scale=None, *, tof_verified=False):
    from modules.mapping.data.rescue_mapping import unproject

    map_space = pose is not None and scale is not None
    stride = math.ceil(math.sqrt(depth.size / MAX_FRAME_PREVIEW_POINTS))
    xyz, rows, cols = unproject(depth * scale if map_space else depth,
        pose if map_space else np.eye(4), params, stride=stride)
    if len(xyz) > MAX_FRAME_PREVIEW_POINTS:
        keep = np.linspace(0, len(xyz) - 1, MAX_FRAME_PREVIEW_POINTS, dtype=int)
        xyz, rows, cols = xyz[keep], rows[keep], cols[keep]
    if len(xyz):
        return {'space': 'map' if map_space else 'camera', 'tof_verified': tof_verified,
                'points': np.column_stack((xyz.round(4), rgb[rows, cols])).tolist()}
    return None


def run(source, output, options, estimator_factory=DepthEstimator, tracker_factory=None,
        stop_event=None, on_preview=None, live_receive=None, semantic_factory=None):
    from modules.mapping.data.rescue_mapping import mapping_options, mapping_sample_fps
    options = {**options, **mapping_options(options), 'sample_fps': mapping_sample_fps(options)}
    from modules.mapping.localization.dpvo_pose import DPVO_POSE_SOURCE, DpvoAlignment, DpvoPoseTracker
    from modules.mapping.localization.pose_models import BATCH_POSE_MODELS, HYBRID, POSE_IMAGES
    offline = live_receive is None
    pose_model = options.get('pose_model') if offline else None
    if pose_model is not None and (not isinstance(pose_model, str) or pose_model not in POSE_IMAGES):
        raise ValueError('invalid_reconstruction_model')
    full_video = offline and options.get('all_video_frames') is True
    causal_video = full_video or (offline and (pose_model is not None or options.get('causal_video') is True))
    learned_pose = pose_model in BATCH_POSE_MODELS or pose_model == HYBRID
    tracking_rate = options['sample_fps'] if learned_pose and not full_video else None
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / 'frames').mkdir()
    started = time.monotonic()
    run_id = options.get('run_id', output.parent.name + '_' + output.name)
    status = {'status': 'running', 'stage': 'model', 'backend': 'depth_rgbd', 'processed': 0,
              'registered': 0, 'dropped': 0, 'revision': 0, 'points': 0, 'timestamp_s': 0.,
              'tracking': 'initializing', 'all_video_frames': full_video, 'causal_video': causal_video, 'processing_policy':
                  'continuous' if live_receive is not None and options['live_policy'] == 'continuous' else 'sequential'}
    if pose_model is not None:
        status['pose_model'] = pose_model
    producer = history = None
    rescue = semantics_estimator = None
    cameras = []
    tof_calibrations = {} if options.get('front_tof_enabled') else None
    tello_pose = None
    last_nonempty_points = []
    render_window_s = None  # Spatial compaction retains all ages within the point budget.
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
        history = HistoryWriter(output, write_json, render_window_s, synchronous=True, compress=causal_video)
        publish()
        trajectory = None
        if pose_model in BATCH_POSE_MODELS:
            trajectory = scan_trajectory(pose_model, source, options['frame_fps'], tracking_rate, options,
                                         lambda: stop_event is not None and stop_event.is_set(), publish)
            publish(stage='model')
        model_started = time.perf_counter()
        estimator = estimator_factory()
        check_cancelled()
        camera = options.get('camera') or dict(width=4, height=3)
        width, height = options.get('input_size') or (camera['width'], camera['height'])
        model_width, model_height = scaled_size(width, height, options['depth_resolution'])
        preparation_shape = (model_height, model_width, 3)
        # Use the inspected, display-oriented dimensions without opening the
        # input during model preparation.
        if hasattr(estimator, 'prepare'):
            estimator.prepare(preparation_shape)
        else:
            estimator.estimate(np.zeros(preparation_shape, dtype=np.uint8))
        from modules.mapping.data.rescue_mapping import SemanticEstimator, RescueMap
        rescue = RescueMap(options)
        if options['semantic_mapping']:
            publish(stage='semantic_model')
            semantics_estimator = (semantic_factory or SemanticEstimator)()
            if hasattr(semantics_estimator, 'prepare'):
                semantics_estimator.prepare(preparation_shape)
            status['semantic_mapping'] = True
            status['depth_resolution'] = options['depth_resolution']
        check_cancelled()
        publish(stage='tracking_setup' if offline else 'tracking', device=getattr(estimator, 'device', 'test'),
                model_preparation_ms=round((time.perf_counter() - model_started) * 1000, 2))
        if live_receive is not None:
            producer = LivePoseFrames(live_receive, options['frame_fps'], options['sample_fps'],
                output, options['camera'], tracker_factory, continuous=options['live_policy'] == 'continuous',
                predict_pose=tof_calibrations is not None, prefer_sensor_pose=options.get('prefer_sensor_pose', False))
        else:
            producer = PoseFrames(source, options['frame_fps'], None if full_video else options['sample_fps'],
                output, options.get('camera'), tracker_factory)
            producer.on_tracker_ready = lambda: publish(stage='initial_pose')
            producer.tracking_rate = tracking_rate
            if learned_pose:
                producer.tracking_size = LEARNED_TRACKING_LONG_SIDE
            producer.trajectory = trajectory
        producer.cancel_event = stop_event
        from modules.mapping.localization.pose_first import ConstantVelocityPose, fixed_camera_pose, depth_in_pose_units
        from modules.mapping.localization.tello_pose import LINEAR_PREDICTION, TelloMapPose
        video_pose = None
        if causal_video:
            from modules.mapping.localization.depth_pose import VideoMapPose
            video_pose = VideoMapPose()
            producer.motion = None
            producer.track_all_frames = True
            if pose_model == HYBRID:
                producer.dpvo_alignment = DpvoAlignment()
            elif pose_model is None and tracker_factory is None:
                producer.legacy_dpvo_factory = DpvoPoseTracker.start
                producer.dpvo_alignment = DpvoAlignment()
        active_map = None
        active_visual_scale = active_tof_scale = None
        visual_scale_stamp = None
        submaps = []
        while True:
            check_cancelled()
            if offline and status['processed']:
                recovering = status['tracking'] in {'lost', 'pose_predicted', 'pose_held', 'pose_dpvo'}
                initial = active_map is None and not status.get('pose_confirmed')
                publish(stage='initial_pose' if initial else 'recovering_pose' if recovering else 'tracking')
            item = producer.take()
            check_cancelled()
            if item is None:
                break
            if causal_video and producer.dpvo_status is not None:
                status['dpvo_status'] = producer.dpvo_status
            number, stamp, display_rgb, received, original_size, native_pose, *sensor = item
            tof_sample = sensor[0] if sensor else None
            tick = time.monotonic()
            # Admit pose before invoking depth. The returned matrix is immutable here.
            status.update(frame_number=number, timestamp_s=stamp, dropped=producer.dropped)
            if not offline:
                publish(stage='tracking')
            status.update(tracked_frames=getattr(producer, 'tracked', number),
                          mapping_sampled_out=getattr(producer, 'sampled_out', 0))
            telemetry = native_pose.get('tello_telemetry')
            pose_source = None
            pose_predicted = False
            if telemetry is not None:
                if tello_pose is None:
                    tello_pose = TelloMapPose()
                rgb, params = depth_frame(display_rgb, original_size, options)
                pose, pose_source = tello_pose.resolve(telemetry, rgb, params, stamp)
                pose_predicted = pose_source == LINEAR_PREDICTION
            else:
                pose = fixed_camera_pose(native_pose, stamp)
            metric_pose = telemetry is not None and pose is not None
            sensor_wait = telemetry is not None and pose is None
            h, w = display_rgb.shape[:2]
            map_id = native_pose.get('map_id')
            native_confirmed = pose is not None and not metric_pose
            prediction = native_pose.get('predicted_pose')
            fresh_scale = not offline or (visual_scale_stamp is not None and stamp - visual_scale_stamp <= ConstantVelocityPose.max_prediction_s)
            if not causal_video and pose is None and active_visual_scale is not None and active_map == map_id and fresh_scale:
                pose = prediction
                pose_predicted = pose is not None
            status['processed'] += 1
            name = f'frame_{number:07d}.jpg'
            Image.fromarray(display_rgb).save(output / 'frames' / name, quality=80)
            mapped = False
            observation = None
            evicted = 0
            state = 'initializing' if native_pose.get('tracking_state', 0) <= 1 else 'lost'
            if sensor_wait:
                state = 'sensor_wait'
            depth_scale = None
            rescue_snapshot = None
            frame_preview = None
            estimated_alignment = False
            tof_verified = False
            tof = None if tof_calibrations is None else dict(distance_m=tof_sample[0] if tof_sample else None,
                meters_per_slam_unit=None, corrected=False, surface_supported=False, used=False)
            if native_confirmed or metric_pose:
                state = 'scale_wait'
            if causal_video:
                camera_pose = pose
                native_map = (map_id, native_pose.get('tracking_epoch', 0))
                rgb, params = depth_frame(display_rgb, original_size, options)
                dpvo_fallback = native_pose.get('dpvo_fallback')
                if camera_pose is None and video_pose.last_pose is not None and dpvo_fallback is None:
                    publish(stage='visual_registration')
                pose, state = video_pose.resolve(stamp, rgb, params, camera_pose, native_map, dpvo_fallback)
                map_id = video_pose.map_id
                native_confirmed = state == 'tracking'
                pose_predicted = state == 'pose_predicted'
            if pose is not None or (tof is not None and not sensor_wait):
                publish(stage='depth', pose_confirmed=native_confirmed, pose_estimated=pose_predicted or state == 'pose_dpvo',
                        **({'alignment': state} if causal_video else {}))
                if telemetry is None and not causal_video:
                    rgb, params = depth_frame(display_rgb, original_size, options)
                # Metric model estimates are converted only for the legacy SLAM path.
                depth = estimator.estimate(rgb)
                depth[~np.isfinite(depth) | (depth < .2) | (depth > 8.)] = 0
                depth_classes = None
                if metric_pose:
                    depth_classes = np.where(depth >= BLUE_DEPTH_MIN_M, FAR_DEPTH, STANDARD_DEPTH).astype(np.uint8)
                semantics = None
                if semantics_estimator and pose is not None:
                    publish(stage='semantics')
                    semantics = semantics_estimator.estimate(rgb, name, stamp)
                    from modules.mapping.data.rescue_mapping import validate_semantics, PERSON
                    labels, confidence = validate_semantics(semantics, depth.shape, name, stamp)
                aligned = None
                excluded = (labels == PERSON) if semantics is not None else None
                usable_depth = np.count_nonzero(depth) > depth.size * .05
                if causal_video:
                    if usable_depth:
                        native_alignment = None
                        if camera_pose is not None:
                            publish(stage='depth_alignment')
                            native_alignment = depth_in_pose_units(depth, native_pose.get('depth_anchors', []), (w, h), excluded=excluded)
                        scale, state = video_pose.accept_depth(stamp, rgb, depth, params, excluded, pose, state, camera_pose, native_map,
                            native_alignment[1] if native_alignment is not None else None)
                        if scale is not None:
                            estimated_alignment = state in {'pose_dpvo', 'pose_predicted', 'pose_reused', 'pose_held', 'scale_reused'}
                            aligned = (depth * scale, scale)
                    else:
                        state = 'invalid_depth'
                        pose = None
                        native_confirmed = False
                elif metric_pose:
                    aligned = (depth, 1.)  # The pose and the metric depth model both use meters.
                elif native_confirmed:
                    if offline:
                        publish(stage='depth_alignment')
                    aligned = depth_in_pose_units(depth, native_pose.get('depth_anchors', []), (w, h), excluded=excluded)
                elif pose_predicted:
                    aligned = (depth * active_visual_scale, active_visual_scale)
                if not usable_depth:
                    aligned = None
                visual_scale = aligned[1] if aligned is not None else None
                supported = None
                if tof is not None and tof_sample is not None:
                    supported = match_front_surface(depth, params, tof_sample[0], excluded)
                    tof['surface_supported'] = supported is not None
                if aligned is not None and supported is not None and metric_pose:
                    supported_depth, model_range = supported
                    measured = supported_depth > 0
                    depth_classes[measured] = TOF_DEPTH
                    aligned = (np.where(measured, supported_depth, depth), 1.)
                    tof['corrected'] = abs(tof_sample[0] / model_range - 1.) >= .01
                    tof_verified = True
                elif aligned is not None and supported is not None and native_confirmed:
                    supported_depth, model_range = supported
                    if map_id not in tof_calibrations:
                        tof_calibrations[map_id] = TofDepth()
                    calibration = tof_calibrations[map_id]
                    scale, tof['corrected'] = calibration.scale_to_map(visual_scale, model_range, tof_sample)
                    tof['meters_per_slam_unit'] = calibration.meters_per_unit
                    if scale is not None:
                        aligned = (supported_depth * scale, scale)
                        tof_verified = True
                if aligned is not None:
                    if metric_pose and not pose_predicted:
                        tello_pose.remember(stamp, rgb, aligned[0], params, pose, excluded)
                    depth, depth_scale = aligned
                    if active_map is not None and active_map != map_id:
                        # The append-only history already durably holds this
                        # segment. Avoid serializing its entire cloud again on
                        # the live transition; finalization writes full submaps.
                        submaps.append(active_map)
                        cameras = []
                        active_tof_scale = None
                    active_map = map_id
                    if native_confirmed:
                        active_visual_scale = visual_scale
                        visual_scale_stamp = stamp
                    if tof_verified:
                        active_tof_scale = depth_scale
                        tof['used'] = True
                    publish(stage='mapping', pose_estimated=pose_predicted or state == 'pose_dpvo',
                            alignment=state if causal_video else None)
                    mapping_semantics = semantics or dict(frame_id=name, timestamp_s=stamp,
                        labels=np.zeros(depth.shape, np.uint8), confidence=np.ones(depth.shape, np.float32),
                        people=[], inference_ms=0.)
                    rescue_snapshot = rescue.update(rgb, depth, pose, params, mapping_semantics,
                        depth_scale, map_id, name, stamp, tof_verified=tof_verified,
                        pose_predicted=pose_predicted or estimated_alignment,
                        depth_classes=depth_classes)
                    # Keep source observations for replay; spatial fusion bounds display size.
                    observation = {'depth': depth, 'rgb': rgb, 'world_from_camera': pose,
                        'intrinsics': np.asarray(params), 'map_id': np.asarray(map_id),
                        'depth_scale': np.asarray(depth_scale), 'timestamp_s': np.asarray(stamp)}
                    if tof_verified:
                        observation['tof_verified'] = np.asarray(True)
                    if depth_classes is not None:
                        observation['depth_classes'] = depth_classes
                    if semantics is not None:
                        observation.update(semantic_labels=labels, semantic_confidence=confidence,
                            person_instances=np.stack([p['mask'] for p in semantics['people']]).astype(np.uint8)
                                if semantics['people'] else np.empty((0, *depth.shape), np.uint8),
                            person_confidences=np.asarray([p['confidence'] for p in semantics['people']], np.float32))
                    cameras.append({'image_id': number, 'frame_id': name, 'source_pts_s': stamp,
                        'timestamp_s': stamp, 'position': pose[:3, 3].tolist(),
                        'world_from_camera_rotation': pose[:3, :3].tolist(), 'model': 'PINHOLE',
                        'width': rgb.shape[1], 'height': rgb.shape[0], 'params': list(params)})
                    status['registered'] += 1
                    if pose_predicted:
                        status['pose_predicted_frames'] = status.get('pose_predicted_frames', 0) + 1
                    mapped = True
                    if not causal_video:
                        state = 'pose_predicted' if pose_predicted else 'sensor_pose' if metric_pose else 'tracking'
                elif tof is not None:
                    # A temporary projection can use the last scale of this map.
                    # Keep it outside the accumulated cloud and observation history.
                    scale = active_tof_scale if supported is not None else active_visual_scale
                    if map_id != active_map:
                        scale = None
                    frame_preview = frame_depth_preview(rgb, supported[0] if supported is not None else depth,
                        params, pose, scale, tof_verified=supported is not None)
                    tof['used'] = bool(frame_preview and frame_preview['tof_verified'])
            elif hasattr(estimator, 'keep_warm'):
                estimator.keep_warm(preparation_shape)
            status['revision'] += 1
            # One immutable snapshot binds this image and its map to the same revision.
            points = []
            if rescue_snapshot:
                points = rescue_snapshot.pop('static_points')
            elif rescue and rescue.surface:
                points = rescue.surface.points()
                rescue_snapshot = dict(schema_version='rescue-semantic-map/1.0', map_id=rescue.map_id,
                    render_mode='points', point_revision=rescue.surface.revision,
                    point_fusion=rescue.surface.metadata(), point_labels=rescue.surface.labels(), chunk=0,
                    people=rescue.sightings.snapshot(stamp), person_color=options['person_color'],
                    person_outline=options['person_outline'], tracking_lost=True)
            if points:
                last_nonempty_points = points
            preview_points = points
            visible_cameras = [c['position'] for c in cameras[::max(1, math.ceil(len(cameras) / 300))]]
            # Sample the entire route instead of dropping its beginning.
            snapshot = {'revision': status['revision'], 'frame_id': name,
                'frame_number': number, 'timestamp_s': stamp, 'tracking': state, 'pose_confirmed': native_confirmed,
                'map_id': active_map, 'pose_map_id': map_id,
                'pose_position': pose[:3, 3].tolist() if pose is not None else None,
                'pose_rotation': pose[:3, :3].tolist() if pose is not None else None,
                'depth_scale': depth_scale, 'submaps': len(submaps) + (active_map is not None),
                'points': preview_points, 'map_points': len(points), 'cameras': visible_cameras,
                'render_window_s': render_window_s, 'render_evicted_points': evicted,
                'processed': status['processed'], 'registered': status['registered'],
                'dropped': producer.dropped, 'processing_ms': round((time.monotonic() - tick) * 1000),
                'latency_ms': round((time.monotonic() - received) * 1000)}
            if metric_pose:
                snapshot['pose_source'] = pose_source
            elif state == 'pose_dpvo':
                snapshot['pose_source'] = DPVO_POSE_SOURCE
            elif pose_model in BATCH_POSE_MODELS and state in {'tracking', 'scale_reused'}:
                snapshot['pose_source'] = pose_model
            if estimated_alignment:
                snapshot['estimated_alignment'] = True
            if telemetry is not None and active_map is not None:
                snapshot['map_unit'] = 'm'
            if frame_preview is not None:
                snapshot['frame_preview'] = frame_preview
            if tof is not None:
                snapshot['front_tof'] = status['front_tof'] = tof
            if rescue_snapshot:
                snapshot['rescue'] = rescue_snapshot
                from modules.mapping.data.rescue_mapping import person_points
                people_points = person_points(rescue_snapshot)
                snapshot['points'] = snapshot['points'] + people_points
                rescue_snapshot['point_labels'] = rescue_snapshot['point_labels'] + [1]*len(people_points)
                for key, mask in (('point_tof', rescue.surface.tof_verified), ('point_far', rescue.surface.far_depth)):
                    if mask.any():
                        rescue_snapshot[key] = mask.tolist() + [False]*len(people_points)
                if snapshot['points']:
                    last_nonempty_points = snapshot['points']
            history.append(snapshot, observation)
            check_cancelled()
            if offline:
                status.update(completed_frame_number=number, completed_timestamp_s=stamp)
            if on_preview is None:
                write_json(output / 'preview.json', snapshot)
            else:
                publish(stage='presentation', tracking=state, points=len(points), frame_id=name)
                on_preview(snapshot)
            stage = 'mapping' if mapped else 'pose_wait'
            if frame_preview is not None:
                stage = 'frame_preview'
            publish(stage=stage, tracking=state, pose_confirmed=native_confirmed,
                    points=len(points), frame_id=name, latency_ms=round((time.monotonic() - received) * 1000))
        if not last_nonempty_points or not cameras:
            raise ValueError('no_valid_depth_map')
        producer.close()
        producer = None
        history.close('complete')
        publish(stage='integration', integrated_frames=0, integration_total=status['registered'])
        scale = finalize_history(output, status, run_id, started, publish)
        publish(status='complete', stage='finished', map_unit=scale['unit'])
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
            if rescue:
                rescue.close()


def finalize_history(output, status, run_id, started, publish=lambda **fields: None):
    from modules.mapping.localization.dpvo_pose import DPVO_POSE_SOURCE
    from modules.mapping.data.history_fusion import integrate_history
    from modules.mapping.localization.tello_pose import METRIC_POSE_SOURCES
    output = Path(output)
    groups, entries = integrate_history(output, lambda **fields: publish(stage='integration', **fields))
    primary = max(groups.values(), key=lambda group: len(group['cameras']))
    all_registered = sum(len(group['cameras']) for group in groups.values())
    submaps = []
    display_rotation = np.diag([1., -1., -1.])
    for group in groups.values():
        if group.get('rescue'):
            from modules.mapping.data.rescue_mapping import display_coordinates
            group['rescue'] = display_coordinates(group['rescue'])
        group['rows'] = [[i, p[0], -p[1], -p[2], *p[3:], 0., 1] for i, p in enumerate(group['cells'].values())]
        if group.get('rescue'):
            from modules.mapping.data.rescue_mapping import person_points
            offset = len(group['rows'])
            group['rows'].extend([[offset+i, *p, 0., 1] for i, p in enumerate(person_points(group['rescue']))])
            group['rescue']['points_include_people'] = True
            if group['rescue'].get('render_mode') == 'points':
                group['rescue']['point_labels'] += [1]*(len(group['rows'])-offset)
            for key in ('point_tof', 'point_far'):
                if key in group['rescue']:
                    group['rescue'][key] += [False]*(len(group['rows'])-offset)
        del group['cells']
        for camera in group['cameras']:
            camera['position'] = (display_rotation @ camera['position']).tolist()
            camera['world_from_camera_rotation'] = (display_rotation @ camera['world_from_camera_rotation']).tolist()
        if len(groups) > 1:
            name = f"submaps/full-{group['map_id']}.json"
            (output / 'submaps').mkdir(exist_ok=True)
            write_json(output / name, {'map_id': group['map_id'], 'points': group['rows'], 'cameras': group['cameras'],
                'scope': 'all_recorded_observations', **({'rescue': group['rescue']} if group.get('rescue') else {})})
            submaps.append({'map_id': group['map_id'], 'file': name, 'points': len(group['rows']), 'frames': len(group['cameras'])})
    rows, cameras = primary['rows'], primary['cameras']
    xyz = np.array([p[1:4] for p in rows] or [c['position'] for c in cameras])
    unmapped = sum(not e['observation'] for e in entries)
    partial_reasons = (['unaligned_map_segments'] if len(groups) > 1 else []) + (['insufficient_camera_baseline'] if len(cameras) < 2 else [])
    if (status.get('causal_video') or status.get('all_video_frames')) and unmapped:
        partial_reasons.append('unregistered_video_frames')
    partial = bool(partial_reasons)
    quality = {'input_frames': len(entries), 'submitted_frames': len(entries),
        'registered_frames': len(cameras), 'registration_ratio': len(cameras) / len(entries),
        'points3D': len(rows), 'excluded_frames': [e['frame_id'] for e in entries if not e['observation']],
        'mean_reprojection_error_px': None, 'processing_seconds': time.monotonic() - started,
        'result_status': 'partial' if partial else 'complete',
        'partial_reasons': partial_reasons,
        'bounds': {'min': xyz.min(axis=0).tolist(), 'max': xyz.max(axis=0).tolist()},
        'coverage': {'recorded_frames': len(entries), 'integrated_frames': all_registered,
            'initialization_frames': sum(e['tracking'] == 'initializing' for e in entries),
            'unmapped_frames': unmapped, 'map_count': len(groups),
            'held_pose_frames': sum(e['tracking'] == 'pose_held' for e in entries),
            'spatial_completeness_verified': False}}
    sensor_origin = primary['pose_source'] == 'tello_sensor_odometry'
    tello_metric = primary['pose_source'] in METRIC_POSE_SOURCES
    scale = {'unit': 'm' if tello_metric else 'arbitrary',
             'status': 'tello_metric_estimate' if tello_metric else 'monocular_pose_scale', 'factor': 1.}
    coordinate_system = ({'name': 'tello_sensor_display', 'y': 'up from Tello attitude'} if sensor_origin else
                         {'name': 'first_camera_display', 'y': 'view-derived up, not measured gravity'})
    scene = {'schema_version': 'phone-sfm-result/1.2', 'input_run_id': run_id,
        'created_utc': datetime.now(timezone.utc).isoformat(), 'pose_source': primary['pose_source'],
        'points': rows, 'point_fields': ['point_id', 'x', 'y', 'z', 'r', 'g', 'b', 'error_px', 'track_length'],
        'cameras': cameras, 'quality': quality, 'coordinate_system': coordinate_system,
        'segment_id': primary['map_id'], 'scale': scale,
        'depth_mapping': {'model': MODEL, 'pose_first': True, 'tracker': 'tello_sensor_rgbd' if tello_metric else 'orb_slam3_monocular',
            'submaps': submaps, 'dropped': status.get('dropped', 0), 'metric_accuracy_verified': False,
            'history': 'history/manifest.json', 'integration_scope': 'all_recorded_observations',
            'integrated_frames': all_registered, 'integrated_maps': len(groups),
            'primary_map_selection': 'most_observed_frames', 'preview_window_applied': False,
            'preview_point_limit_applied': False,
            'spatial_point_budget': primary.get('rescue', {}).get('point_fusion', {}).get('point_budget'),
            'spatial_density_adaptive': primary.get('rescue', {}).get('render_mode') == 'points'}}
    if 'pose_model' in status:
        scene['depth_mapping']['pose_model'] = status['pose_model']
    if 'dpvo_status' in status:
        scene['depth_mapping']['dpvo'] = {'status': status['dpvo_status'],
            'integrated_frames': sum(bool(entry['observation']) and entry.get('pose_source') == DPVO_POSE_SOURCE
                                     for entry in entries)}
    if primary.get('rescue'):
        scene['rescue'] = primary['rescue']
    tof_by_map = {}
    for entry in entries:
        if not entry['observation'] or 'front_tof' not in entry:
            continue
        tof = entry['front_tof']
        summary = tof_by_map.setdefault(entry['map_id'], dict(paired_frames=0, corrected_frames=0,
            meters_per_slam_unit=None))
        summary['paired_frames'] += tof['distance_m'] is not None
        summary['corrected_frames'] += tof['corrected']
        if tof['meters_per_slam_unit'] is not None:
            summary['meters_per_slam_unit'] = tof['meters_per_slam_unit']
    if tof_by_map:
        scene['depth_mapping']['front_tof_by_map'] = tof_by_map
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
        writer.writerows([c['frame_id'], c['timestamp_s'], *c['position'],
            c.get('pose_source', primary['pose_source']), scale['unit']] for c in cameras)
    write_json(output / 'status.json', {'status': quality['result_status']})
    return scale


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
