"""Frame-time feature tracking and bounded, persistent planar RGB mapping."""
from __future__ import annotations

from copy import deepcopy
from collections import Counter
from dataclasses import replace
from pathlib import Path
import shutil
import time

import cv2
import numpy as np

from planar import build_planar_objects, detection_settings, image_quads


class FrameTracker:
    """Track every consumed frame; expensive boundaries are proposed once per keyframe."""
    def __init__(self, profile="balanced", plane_error_px=None):
        self.settings = detection_settings(profile, plane_error_px)
        self.gray, self.points = None, None
        self.frames = 0

    def observe(self, image):
        ratio = min(1., 480 / image.shape[1])
        gray = cv2.cvtColor(cv2.resize(image, None, fx=ratio, fy=ratio), cv2.COLOR_BGR2GRAY)
        tracked = np.empty((0, 1, 2), np.float32)
        if self.gray is not None and self.gray.shape == gray.shape and self.points is not None:
            forward, good, _ = cv2.calcOpticalFlowPyrLK(self.gray, gray, self.points, None,
                winSize=(21, 21), maxLevel=3)
            if forward is not None:
                backward, back_good, _ = cv2.calcOpticalFlowPyrLK(gray, self.gray, forward, None,
                    winSize=(21, 21), maxLevel=3)
                if backward is not None:
                    xy = forward.reshape(-1, 2)
                    valid = good.ravel().astype(bool) & back_good.ravel().astype(bool)
                    valid &= np.linalg.norm((backward - self.points).reshape(-1, 2), axis=1) < 1.
                    valid &= (xy[:, 0] >= 0) & (xy[:, 0] < gray.shape[1]) & (xy[:, 1] >= 0) & (xy[:, 1] < gray.shape[0])
                    tracked = forward[valid]
        # Replenish only uncovered image regions, rather than redetecting the same corners.
        mask = np.full(gray.shape, 255, np.uint8)
        for point in tracked.reshape(-1, 2):
            cv2.circle(mask, tuple(np.rint(point).astype(int)), 7, 0, -1)
        added = cv2.goodFeaturesToTrack(gray, maxCorners=max(1, 500 - len(tracked)),
            qualityLevel=.015, minDistance=7, mask=mask) if len(tracked) < 500 else None
        new_count = len(added) if added is not None else 0
        self.points = np.concatenate([tracked, added]) if added is not None else tracked
        if not len(self.points):
            self.points = None
        self.gray = gray
        self.frames += 1
        return {"frames_tracked": self.frames, "tracked_features": len(tracked),
                "new_features": new_count, "tracking_resolution": [gray.shape[1], gray.shape[0]]}

    def propose(self, image):
        started = time.perf_counter()
        ratio = min(1., 640 / image.shape[1])
        analysis = cv2.resize(image, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_AREA)
        quads, _ = image_quads(analysis, replace(self.settings, min_side_px=max(4., self.settings.min_side_px * ratio)))
        return [(q / ratio).tolist() for q in quads], time.perf_counter() - started


class IncrementalPlaneMap:
    """Commit new surfaces once, carrying old geometry, RGB and source views forward.

    A revision's native landmark IDs are never used as cross-window identity. Reuse
    requires finite 3D coverage, depth, normal and projected image-boundary agreement.
    Preparation is transactional: a failed publish leaves the previous map intact.
    """
    def __init__(self, profile="balanced", plane_error_px=None, max_objects=32):
        self.profile, self.plane_error_px, self.max_objects = profile, plane_error_px, max_objects
        self.collection, self.directory = None, None
        self.seen, self.pending, self.sources = set(), {}, {}

    def prepare(self, model, scene, frames, directory):
        directory = Path(directory)
        images, textures = directory / "images", directory / "object_textures"
        same_segment = self.collection is not None and self.collection["segment_id"] == scene["segment_id"]
        previous = deepcopy(self.collection) if same_segment else None
        seen, pending = (self.seen.copy(), self.pending.copy()) if same_segment else (set(), {})
        cameras = {c["frame_id"]: c for c in scene["cameras"]}
        proposals = {f["frame_id"]: f["plane_quads"] for f in frames if f.get("plane_quads") is not None}
        ready = set(cameras) & set(proposals)
        fresh = ready - seen
        # A bounded retry allows initially untriangulated candidates to gain parallax.
        waiting = sorted((name for name in pending if name in cameras and pending[name] < 3),
                         key=lambda name: (pending[name], cameras[name].get("timestamp_s", 0)))
        retry = set(waiting[:2])
        sources = fresh | retry
        # Empty means not ready: never fall back to a second full boundary detection.
        proposals = {name: proposals.get(name, []) for name in cameras}
        collection = build_planar_objects(model, scene, images, textures, previous=previous,
            max_frames=len(cameras), max_objects=self.max_objects, profile=self.profile,
            plane_error_px=self.plane_error_px, source_frame_ids=sources, proposals=proposals,
            incremental=True)
        old_ids = {o["object_id"] for o in (previous or {}).get("objects", [])}
        source_cameras = dict(self.sources) if same_segment else {}
        for obj in collection["objects"]:
            name = obj["source_frame_id"]
            if obj["object_id"] in old_ids:
                shutil.copy2(self.directory / "object_textures" / obj["texture_file"], textures / obj["texture_file"])
                if not (images / name).is_file():
                    shutil.copy2(self.directory / "images" / name, images / name)
            else:
                source_cameras[name] = deepcopy(cameras[name])
                obj["observation_revision"] = int(directory.name)
                obj["created_timestamp_s"] = cameras[name].get("timestamp_s")
            obj["mapping_status"] = "retained" if obj["object_id"] in old_ids else "new"
        # Old source images remain reachable even after their keyframes leave the SfM window.
        retained_names = {o["source_frame_id"] for o in collection["objects"]}
        source_cameras = {name: source_cameras[name] for name in retained_names}
        for name, camera in source_cameras.items():
            if name not in cameras:
                scene["cameras"].append({**camera, "retained_object_source": True})
        scene["cameras"].sort(key=lambda c: c.get("timestamp_s", 0))
        for name in sorted(sources, key=lambda name: cameras[name].get("timestamp_s", 0)):
            pending[name] = pending.get(name, 0) + 1
        pending = {name: attempts for name, attempts in pending.items() if name in cameras and attempts < 3
                   and proposals.get(name)}
        seen = (seen | fresh) & set(cameras)
        stats = collection["statistics"]
        stats["shapes"] = dict(Counter(o["shape"] for o in collection["objects"]))
        groups = []
        for obj in collection["objects"]:
            n, center = np.array(obj["normal"]), np.mean(obj["corners"], axis=0)
            group = next((i for i, (other, offset, tolerance) in enumerate(groups) if abs(n @ other) > .995
                and abs(center @ other + offset) < 2 * min(tolerance, obj["plane_threshold"])), None)
            if group is None:
                group = len(groups)
                groups.append((n, obj["offset"], obj["plane_threshold"]))
            obj["plane_id"] = f"plane-{group + 1:03d}"
        stats["plane_groups"] = len(groups)
        stats.update(fresh_frames=len(fresh), retried_frames=len(retry), pending_frames=len(pending),
                     update_mode="incremental_keyframe", textures_written=stats.get("new_objects", 0))
        token = (deepcopy(collection), directory, seen, pending, source_cameras)
        return collection, token

    def commit(self, token):
        self.collection, self.directory, self.seen, self.pending, self.sources = token
