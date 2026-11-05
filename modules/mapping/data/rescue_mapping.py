"""Frame-aligned semantic RGB-D fusion and visible rescue-person observations.

Geometry stays in the native monocular map's units. A red person is an observed
surface, never a guessed full body. Static point fusion and sightings are separate.
"""
from __future__ import annotations

import math
import re
import time

import cv2
import numpy as np
from .mapping_limits import DEFAULT_POINT_BUDGET, MAX_POINT_BUDGET, MAX_PREVIEW_POINTS

CLASSES = ('unknown', 'person', 'wall', 'floor', 'ceiling', 'table', 'chair', 'door', 'windowpane')
PERSON = 1
SCHEMA = 'rescue-semantic-map/1.0'
DEFAULT_COLOR = '#ff3030'


def mapping_sample_fps(value):
    """Selected depth-map frames per second; tracking has its own 15 Hz ceiling."""
    rate = value.get('sample_fps', 5)
    if type(rate) not in (int, float) or not math.isfinite(rate) or not .1 <= rate <= 15:
        raise ValueError('invalid_sample_fps')
    return rate


def mapping_options(value):
    """Shared validation for saved and live input; old API clients opt in explicitly."""
    enabled = value.get('semantic_mapping', False)
    if type(enabled) is not bool:
        raise ValueError('invalid_semantic_mapping')
    resolution = value.get('depth_resolution', 384 if enabled else 256)
    if resolution not in (256, 384, 518):
        raise ValueError('invalid_depth_resolution')
    color = value.get('person_color', DEFAULT_COLOR)
    if not isinstance(color, str) or not re.fullmatch(r'#[0-9a-fA-F]{6}', color):
        raise ValueError('invalid_person_color')
    outline = value.get('person_outline', True)
    if type(outline) is not bool:
        raise ValueError('invalid_person_outline')
    policy = value.get('live_policy', 'sequential')
    if policy not in ('sequential', 'continuous'):
        raise ValueError('invalid_live_policy')
    budget = value.get('point_budget', DEFAULT_POINT_BUDGET)
    if type(budget) is not int or not 1000 <= budget <= MAX_POINT_BUDGET:
        raise ValueError('invalid_point_budget')
    return dict(semantic_mapping=enabled, depth_resolution=resolution, person_color=color.lower(),
                person_outline=outline, live_policy=policy, point_budget=budget)


class SemanticEstimator:
    """Reuse the perception team's two models, returning source-frame masks only."""
    def __init__(self, *, allow_download=False):
        from modules.perception.inference.tello_dual_segmentation import DualSegmentationProcessor, parser
        import torch
        args = parser().parse_args([])
        from .model_assets import cached_snapshot, person_checkpoint, ModelAssetsMissing
        checkpoint = person_checkpoint()
        if not checkpoint.is_file():
            if not allow_download:
                raise ModelAssetsMissing(args.yolo_model)
            checkpoint.parent.mkdir(parents=True, exist_ok=True)
        try:
            args.semantic_model = str(cached_snapshot(args.semantic_model))
        except ModelAssetsMissing:
            if not allow_download:
                raise
        args.yolo_model = str(checkpoint)
        args.local_models_only = not allow_download
        args.yolo_size = 384
        args.fp16 = torch.cuda.is_available()
        self.processor = DualSegmentationProcessor(args)

    def prepare(self, shape):
        shapes = getattr(self, 'prepared_shapes', set())
        key = tuple(shape)
        if key not in shapes:
            self.estimate(np.zeros(shape, np.uint8), 'model_warmup', 0.)
            shapes.add(key)
            self.prepared_shapes = shapes

    def estimate(self, rgb, frame_id, stamp):
        result = self.processor.process(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), structured=True)
        labels = np.zeros(rgb.shape[:2], np.uint8)
        for original, label in result['id2label'].items():
            if label in CLASSES:
                labels[result['labels'] == original] = CLASSES.index(label)
        people = []
        detections = result['people']
        if detections.boxes is not None and detections.masks is not None:
            from modules.perception.inference.tello_person_stream import _mask_polygons
            for index, confidence in enumerate(detections.boxes.conf.cpu().tolist()):
                mask = np.zeros(rgb.shape[:2], np.uint8)
                polygons = _mask_polygons(detections.masks, index, rgb.shape[1], rgb.shape[0])
                cv2.fillPoly(mask, [np.rint(p).astype(np.int32) for p in polygons if len(p) >= 3], 1)
                labels[mask.astype(bool)] = PERSON
                people.append({'mask': mask.astype(bool), 'confidence': float(confidence)})
        return dict(frame_id=frame_id, timestamp_s=stamp, labels=labels,
                    confidence=result['confidence'], people=people, inference_ms=result['inference_ms'])


def validate_semantics(result, shape, frame_id, stamp):
    if result.get('frame_id') != frame_id or result.get('timestamp_s') != stamp:
        raise ValueError('semantic_frame_mismatch')
    labels, confidence = np.asarray(result['labels']), np.asarray(result['confidence'])
    if labels.shape != shape or confidence.shape != shape or not np.isfinite(confidence).all():
        raise ValueError('semantic_geometry_mismatch')
    if labels.dtype.kind not in 'ui' or np.any(labels >= len(CLASSES)) or np.any(confidence < 0) or np.any(confidence > 1):
        raise ValueError('invalid_semantic_labels')
    for person in result['people']:
        if np.asarray(person['mask']).shape != shape or not math.isfinite(person['confidence']) or not 0 <= person['confidence'] <= 1:
            raise ValueError('invalid_person_mask')
    return labels.astype(np.uint8), confidence.astype(np.float32)


def smooth_structural_depth(depth, labels):
    """Denoise observed structural pixels without crossing semantic/depth edges."""
    valid = np.isfinite(depth) & (depth > 0)
    samples = []
    padded_depth = np.pad(depth, 1, mode='constant')
    padded_labels = np.pad(labels, 1, mode='constant', constant_values=255)
    h, w = depth.shape
    for y in range(3):
        for x in range(3):
            neighbor = padded_depth[y:y+h, x:x+w]
            same = padded_labels[y:y+h, x:x+w] == labels
            close = np.abs(neighbor - depth) < np.maximum(depth * .035, 1e-6)
            samples.append(np.where(same & close & (neighbor > 0), neighbor, np.nan))
    samples = np.stack(samples)
    count = np.isfinite(samples).sum(axis=0)
    # Avoid nanmedian's all-NaN warning on unobserved pixels.
    samples[:, ~valid] = 0
    median = np.nanmedian(samples, axis=0)
    result = depth.copy()
    selected = valid & np.isin(labels, [2, 3, 4, 5]) & (count >= 5)
    result[selected] = .5 * depth[selected] + .5 * median[selected]
    return result


def unproject(depth, pose, params, mask=None, stride=2):
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[v, u]
    valid = np.isfinite(z) & (z > 0)
    if mask is not None:
        valid &= mask[v, u]
    u, v, z = u[valid], v[valid], z[valid]
    fx, fy, cx, cy = params
    xyz = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))
    return xyz @ pose[:3, :3].T + pose[:3, 3], v, u


class PersonSightings:
    """Short-lived voxel silhouettes and persistent, explicitly last-seen markers."""
    def __init__(self, voxel_size):
        self.size = voxel_size
        self.people = {}
        self.next_id = 1

    def update(self, detections, depth, pose, params, stamp, frame_id):
        matched = set()
        for detection in detections:
            mask = np.asarray(detection['mask'], np.uint8)
            core = cv2.erode(mask, np.ones((3, 3), np.uint8)).astype(bool)
            xyz, _, _ = unproject(depth, pose, params, core)
            if len(xyz) < 4:
                continue
            # Reject background depth bleeding at silhouettes and severe depth outliers.
            camera_xyz = (xyz - pose[:3, 3]) @ pose[:3, :3]
            z = camera_xyz[:, 2]
            median = np.median(z)
            xyz = xyz[np.abs(z - median) <= max(4 * self.size, .3 * median)]
            if len(xyz) < 4:
                continue
            keys = np.unique(np.floor(xyz / self.size).astype(np.int64), axis=0)
            voxels = (keys + .5) * self.size
            # Keep the outline bounds from all observations, then bound draw cost.
            lo, hi = voxels.min(axis=0) - self.size / 2, voxels.max(axis=0) + self.size / 2
            center = np.median(xyz, axis=0)
            candidates = [(np.linalg.norm(center - p['center']), key) for key, p in self.people.items()
                          if key not in matched and stamp - p['last_seen_s'] <= 2.]
            distance, key = min(candidates, default=(float('inf'), None))
            if distance > self.size * 18:
                key = self.next_id
                self.next_id += 1
            matched.add(key)
            voxels = voxels[::max(1, math.ceil(len(voxels) / 1600))]
            self.people[key] = dict(id=key, center=center.tolist(), bounds=[lo.tolist(), hi.tolist()],
                voxels=voxels.tolist(), voxel_size=self.size, confidence=float(detection['confidence']),
                last_seen_s=float(stamp), source_frame_id=frame_id)
        # Bounded rescue sightings, retaining the newest 32 distinct locations.
        for key in sorted(self.people, key=lambda k: self.people[k]['last_seen_s'])[:-32]:
            del self.people[key]
        return self.snapshot(stamp)

    def snapshot(self, stamp):
        return [{**p, 'age_s': max(0., stamp - p['last_seen_s']),
                 'observed': stamp == p['last_seen_s'],
                 'voxels': p['voxels'] if stamp - p['last_seen_s'] <= 2 else []}
                for p in self.people.values()]


from modules.mapping.data.point_fusion import SemanticSurface


class RescueMap:
    def __init__(self, options):
        self.options = options
        self.surface = self.sightings = None
        self.map_id = None
        self.chunk = 0
        self.chunk_started = 0.

    def update(self, rgb, depth, pose, params, semantics, scale, map_id, frame_id, stamp):
        labels, confidence = validate_semantics(semantics, depth.shape, frame_id, stamp)
        size = max(1e-5, .04 * scale)
        if map_id != self.map_id:
            if self.surface:
                self.surface.close()
            # Convert the fixed .005 depth-unit step once into this map's
            # coordinate scale. Later frames and the current grid cannot alter it.
            self.surface = SemanticSurface(size, max_cells=self.options['point_budget'],
                spacing_increment=max(1e-6, .005 * scale))
            self.sightings = PersonSightings(max(1e-5, .06 * scale))
            self.map_id = map_id
            self.chunk_started = stamp
            self.chunk = 0
        started = time.perf_counter()
        self.surface.integrate(rgb, depth, pose, params, labels, confidence)
        people = self.sightings.update(semantics['people'], depth, pose, params, stamp, frame_id)
        return dict(schema_version=SCHEMA, classes=list(CLASSES), map_id=map_id,
                    person_color=self.options['person_color'], person_outline=self.options['person_outline'],
                    people=people, render_mode='points', point_revision=self.surface.revision,
                    point_fusion=self.surface.metadata(), point_labels=self.surface.labels(), chunk=0,
                    static_points=self.surface.points(), static_voxels=len(self.surface.cells),
                    fusion_ms=round((time.perf_counter() - started) * 1000, 2),
                    inference_ms=round(semantics['inference_ms'], 2))

    def close(self):
        if self.surface:
            self.surface.close()


def display_coordinates(value):
    """The saved viewer uses y-up/z-back; replay retains the native camera frame."""
    from copy import deepcopy
    result = deepcopy(value)
    result['display_y_up'] = True
    flip = np.array([1., -1., -1.])
    for mesh in result.get('meshes', []):
        for key in ('vertices', 'normals'):
            mesh[key] = (np.asarray(mesh[key]).reshape(-1, 3) * flip).tolist()
    for person in result.get('people', []):
        person['center'] = (np.asarray(person['center']) * flip).tolist()
        person['voxels'] = (np.asarray(person['voxels']).reshape(-1, 3) * flip).tolist()
        bounds = np.asarray(person['bounds']) * flip
        person['bounds'] = [bounds.min(axis=0).tolist(), bounds.max(axis=0).tolist()]
    return result


def validate_payload(value):
    """Validate persisted semantic geometry before exposing an imported ZIP to WebGL."""
    if not isinstance(value, dict) or value.get('schema_version') != SCHEMA:
        raise ValueError('invalid_rescue_schema')
    mapping_options(value)
    if value.get('render_mode') not in (None, 'points'):
        raise ValueError('invalid_rescue_render_mode')
    labels = value.get('point_labels', [])
    if not isinstance(labels, list) or len(labels) > MAX_PREVIEW_POINTS or any(type(v) is not int or not 0 <= v < len(CLASSES) for v in labels):
        raise ValueError('invalid_rescue_point_labels')
    def vectors(rows, maximum, *, bounds=False):
        array = np.asarray(rows, dtype=float)
        if array.size == 0:
            return array.reshape(0, 3)
        if array.ndim != 2 or array.shape[1] != 3 or len(array) > maximum or not np.isfinite(array).all():
            raise ValueError('invalid_rescue_geometry')
        if bounds and (np.any(array < 0) or np.any(array > 1)):
            raise ValueError('invalid_rescue_colors')
        return array
    people = value.get('people', [])
    if not isinstance(people, list) or len(people) > 32:
        raise ValueError('invalid_rescue_people')
    for person in people:
        vectors(person['voxels'], 1600)
        center, box = vectors([person['center']], 1), vectors(person['bounds'], 2)
        if len(box) != 2 or len(center) != 1 or np.any(box[0] > box[1]):
            raise ValueError('invalid_rescue_bounds')
        size = person['voxel_size']
        if type(size) not in (int, float) or not math.isfinite(size) or size <= 0:
            raise ValueError('invalid_person_voxel_size')
    meshes = value.get('meshes', [value.get('mesh')])
    if not isinstance(meshes, list):
        raise ValueError('invalid_rescue_meshes')
    for mesh in meshes:
        if mesh is None:
            continue
        vertices = vectors(mesh['vertices'], 160000)
        faces = np.asarray(mesh['triangles'])
        if faces.size and (faces.ndim != 2 or faces.shape[1] != 3 or faces.dtype.kind not in 'ui' or
                           len(faces) > 160000 or np.any(faces < 0) or np.any(faces >= len(vertices))):
            raise ValueError('invalid_rescue_triangles')
        for key in ('normals', 'colors'):
            attribute = vectors(mesh.get(key, []), 160000, bounds=key == 'colors')
            if len(attribute) not in (0, len(vertices)):
                raise ValueError('invalid_rescue_mesh_attribute')
    return value


def person_points(value, limit=2000):
    color = value.get('person_color', DEFAULT_COLOR)
    rgb = [int(color[i:i+2], 16) for i in (1, 3, 5)]
    people = [p for p in value.get('people', []) if p['voxels']]
    per_person = max(1, limit // max(1, len(people)))
    return [[*voxel, *rgb] for person in people
            for voxel in person['voxels'][::max(1, math.ceil(len(person['voxels']) / per_person))]]
