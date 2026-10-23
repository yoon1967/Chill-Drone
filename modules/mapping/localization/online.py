"""Bounded video ingestion and asynchronous COLMAP keyframe SfM (not a flight SLAM)."""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shutil
from threading import Condition, Event, Lock, Thread
import time

import cv2
import numpy as np
import pycolmap

from planar import detection_settings, scene_binding
from streaming_planes import FrameTracker, IncrementalPlaneMap


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    for attempt in range(10):
        try:
            os.replace(temporary, path)
            return
        except PermissionError:
            if attempt == 9:
                raise
            time.sleep(.01)


class LatestSlot:
    """One pending item: overload replaces stale work instead of growing memory."""
    def __init__(self):
        self.condition, self.item, self.closed, self.dropped = Condition(), None, False, 0

    def put(self, item):
        with self.condition:
            if self.closed:
                return
            if self.item is not None:
                self.dropped += 1
            self.item = item
            self.condition.notify_all()

    def get(self):
        with self.condition:
            self.condition.wait_for(lambda: self.item is not None or self.closed)
            result, self.item = self.item, None
            return result

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()


class State:
    def __init__(self, root):
        self.root, self.lock = root, Lock()
        self.last_write = 0.
        self.data = {"schema_version": "online-map-state/1.0", "revision": 0, "map_available": False,
                     "status": "starting", "capture_state": "starting", "mapping_state": "initializing",
                     "frames_received": 0, "keyframes": 0, "frame_drops": 0, "replaced_jobs": 0,
                     "failed_updates": 0, "backend": "pycolmap_keyframe_sfm_cpu",
                     "real_time_slam": False, "session_id": root.name}

    def update(self, **values):
        with self.lock:
            self.data.update(values)
            self.data["updated_utc"] = datetime.now(timezone.utc).isoformat()
            if values.get("mapping_state") in {"updated", "degraded"}:
                events = self.data.setdefault("update_history", [])
                events.append({k: self.data.get(k) for k in ("updated_utc", "mapping_state", "revision", "last_error", "update_seconds", "publish_lag_seconds")})
                self.data["update_history"] = events[-32:]
            # Per-frame counters remain current in memory; avoid two disk replacements
            # per received frame competing with tracking and JPEG/pose workers.
            telemetry = set(values) <= {"frame_tracking", "frames_received", "frame_drops", "received_fps"}
            now = time.monotonic()
            if not telemetry or now - self.last_write >= .1:
                atomic_json(self.root / "state.json", self.data)
                self.last_write = now

    def get(self, key):
        with self.lock:
            return self.data.get(key)


@dataclass
class Frame:
    image: np.ndarray
    timestamp_s: float
    source_pts_s: float | None
    timestamp_source: str
    received_at: float


def capture(source, slot, stop, state, replay_rate=1.0, max_seconds=0):
    is_file = Path(source).is_file()
    is_camera = source.isdigit() and not is_file
    if is_camera:
        cap = cv2.VideoCapture(int(source))
    else:
        cap = cv2.VideoCapture(source, cv2.CAP_FFMPEG,
                              [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000, cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000])
    started, count, previous = time.monotonic(), 0, -1.0
    try:
        if not cap.isOpened():
            state.update(capture_state="failed", source_error="input_open_failed")
            return
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        state.update(capture_state="receiving", status="receiving", input_kind="file_replay" if is_file else "live")
        while not stop.is_set():
            ok, image = cap.read()
            if not ok:
                state.update(capture_state="eof" if is_file else "disconnected",
                             source_error=None if is_file else "decoder_read_failed_or_stream_disconnected")
                break
            received = time.monotonic()
            pts = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000 if is_file else None
            if pts is not None and np.isfinite(pts) and pts > previous:
                timestamp, clock = pts, "source_pts"
            elif is_file:
                pts = None
                timestamp, clock = count / max(cap.get(cv2.CAP_PROP_FPS), 1), "nominal_fps_fallback"
            else:
                timestamp, clock = received - started, "receiver_monotonic"
            if timestamp <= previous:
                state.update(source_error="non_monotonic_timestamp")
                continue
            previous, count = timestamp, count + 1
            if max_seconds and timestamp > max_seconds:
                state.update(capture_state="duration_limit")
                break
            if is_file and replay_rate > 0:
                stop.wait(max(0, started + timestamp / replay_rate - time.monotonic()))
            slot.put(Frame(image, timestamp, pts, clock, time.monotonic()))
            state.update(frames_received=count, frame_drops=slot.dropped,
                         received_fps=count / max(time.monotonic() - started, .001))
    except Exception:
        state.update(capture_state="failed", source_error="capture_exception")
    finally:
        cap.release()
        slot.close()


def propose_boundaries(slot, tracker, frame_lock, state):
    """Keep line detection off the frame tracking path; coalesce overload explicitly."""
    while (item := slot.get()) is not None:
        meta, image = item
        try:
            quads, seconds = tracker.propose(image)
            with frame_lock:
                meta["plane_quads"] = quads
            state.update(frame_proposals={"frame_id": meta["frame_id"], "quads": len(quads),
                "processing_seconds": seconds, "replaced_requests": slot.dropped}, proposal_error=None)
        except (ValueError, cv2.error):
            state.update(proposal_error="boundary_detection_failed")


def isolated_pose_spikes(cameras, factor=5.0):
    """Scale-independent, isolated detours only; never a frame-name exception."""
    if len(cameras) < 7:
        return []
    positions = np.asarray([c["position"] for c in cameras])
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    baseline = np.median(steps[steps > 1e-8]) if np.any(steps > 1e-8) else 0
    result = []
    for i in range(1, len(cameras) - 1):
        smaller = min(steps[i - 1], steps[i])
        if baseline > 0 and smaller > factor * baseline and np.linalg.norm(positions[i + 1] - positions[i - 1]) < smaller * .65:
            result.append(cameras[i]["frame_id"])
    return result


def align_to_previous(cameras, previous):
    """Align gauges with common camera rotations/centers; reset when inconsistent."""
    by_name = {c["frame_id"]: c for c in previous or []}
    common = [(c, by_name[c["frame_id"]]) for c in cameras if c["frame_id"] in by_name]
    if len(common) >= 3:
        candidate = np.mean([np.array(b["world_from_camera_rotation"]) @ np.array(a["world_from_camera_rotation"]).T for a, b in common], axis=0)
        u, _, vt = np.linalg.svd(candidate)
        correction = np.eye(3)
        correction[2, 2] = np.linalg.det(u @ vt)
        rotation = u @ correction @ vt
        src = np.array([a["position"] for a, _ in common])
        dst = np.array([b["position"] for _, b in common])
        ratios = [np.linalg.norm(dst[j] - dst[i]) / np.linalg.norm(src[j] - src[i])
                  for i in range(len(src)) for j in range(i + 1, len(src)) if np.linalg.norm(src[j] - src[i]) > 1e-6]
        factor = float(np.median(ratios)) if ratios else 0
        translation = np.median(dst - factor * (src @ rotation.T), axis=0)
        error = np.linalg.norm(factor * (src @ rotation.T) + translation - dst, axis=1)
        if factor > 0 and np.median(error) < max(np.linalg.norm(np.ptp(dst, axis=0)) * .1, 1e-6):
            return factor, rotation, translation, True
    first = cameras[0]
    rotation = np.diag([1, -1, -1]) @ np.array(first["world_from_camera_rotation"]).T
    return 1.0, rotation, -rotation @ np.array(first["position"]), False


def fuse_history(previous, current, cell_size, max_points=50000):
    """Bounded spatial fusion. Repeated windows do not inflate independent track counts."""
    combined = np.asarray((previous or []) + current, dtype=float)
    cells, inverse = np.unique(np.floor(combined[:, 1:4] / cell_size).astype(np.int64), axis=0, return_inverse=True)
    weights = 1 / np.maximum(combined[:, 7] + .1, .1)
    values, totals = np.zeros((len(cells), 7)), np.zeros(len(cells))
    np.add.at(totals, inverse, weights)
    np.add.at(values, inverse, combined[:, 1:8] * weights[:, None])
    values /= totals[:, None]
    tracks = np.zeros(len(cells))
    np.maximum.at(tracks, inverse, combined[:, 8])
    # Prefer recently observed cells when the configured spatial budget is reached.
    latest = np.zeros(len(cells), dtype=int)
    np.maximum.at(latest, inverse, np.arange(len(combined)))
    keep = np.argsort(latest)[-max_points:]
    return [[int(i), *values[i, :3].tolist(), *np.rint(values[i, 3:6]).astype(int).tolist(),
             float(values[i, 6]), int(tracks[i])] for i in keep]


def scene_from_model(model, frames, previous, segment):
    meta = {f["frame_id"]: f for f in frames}
    cameras = []
    for im in model.images.values():
        if not im.has_pose:
            continue
        cam, pose = model.cameras[im.camera_id], im.cam_from_world()
        cameras.append({"frame_id": im.name, "image_id": im.image_id, **{k: meta[im.name][k] for k in
            ("source_pts_s", "timestamp_s", "timestamp_source")}, "position": im.projection_center().tolist(),
            "world_from_camera_rotation": pose.rotation.matrix().T.tolist(), "camera_id": im.camera_id,
            "width": cam.width, "height": cam.height, "params": cam.params.tolist(), "model": cam.model_name})
    cameras.sort(key=lambda c: c["timestamp_s"])
    if len(cameras) < 3 or model.num_points3D() < 20:
        raise ValueError("insufficient_registered_geometry")
    factor, rotation, translation, aligned = align_to_previous(cameras, previous)
    for c in cameras:
        c["position"] = (factor * rotation @ np.array(c["position"]) + translation).tolist()
        c["world_from_camera_rotation"] = (rotation @ np.array(c["world_from_camera_rotation"])).tolist()
    points = [[int(pid), *(factor * rotation @ p.xyz + translation).tolist(), *map(int, p.color),
               float(p.error), int(p.track.length())] for pid, p in model.points3D.items()]
    xyz = np.array([p[1:4] for p in points])
    registered = {c["frame_id"] for c in cameras}
    return {"schema_version": "phone-sfm-result/1.2", "input_run_id": "online", "pose_source": "sfm",
            "camera_parameters_source": "self_estimated; not measured calibration",
            "created_utc": datetime.now(timezone.utc).isoformat(), "segment_id": segment + int(bool(previous) and not aligned),
            "gauge_aligned": aligned, "scale": {"status": "arbitrary", "unit": "arbitrary", "factor": factor,
                "rotation": rotation.tolist(), "translation": translation.tolist(),
                "reason": "monocular_online_sfm_without_metric_reference",
                "independent_length_validation": "not_performed"},
            "cameras": cameras, "points": points,
            "point_fields": ["point_id", "x", "y", "z", "r", "g", "b", "error_px", "track_length"],
            "quality": {"input_frames": len(frames), "submitted_frames": len(frames), "registered_frames": len(cameras),
                "registration_ratio": len(cameras) / len(frames), "points3D": len(points), "excluded_frames": [],
                "unregistered_frames": [f["frame_id"] for f in frames if f["frame_id"] not in registered],
                "mean_reprojection_error_px": model.compute_mean_reprojection_error(),
                "error_statistics_scope": "current_sfm_window; historical fused points retain earlier estimates",
                "kind": "online_keyframe_sfm", "bounds": {"min": xyz.min(axis=0).tolist(), "max": xyz.max(axis=0).tolist()}},
            "limitations": ["온라인 키프레임 SfM이며 루프 폐쇄·재위치를 지원하는 실시간 SLAM은 아닙니다.",
                            "단안 임의 단위입니다. 미터 스케일·실제 드론 지연은 미검증입니다."]}


class Mapper(Thread):
    def __init__(self, args, state, jobs, frame_lock):
        super().__init__(daemon=True)
        self.args, self.state, self.jobs, self.frame_lock = args, state, jobs, frame_lock
        self.previous, self.segment, self.attempt = None, 0, 0
        self.history, self.cell_size = [], None
        self.plane_map = IncrementalPlaneMap(args.plane_profile, args.plane_error_px)

    def run(self):
        while (frames := self.jobs.get()) is not None:
            self.attempt += 1
            directory = self.args.session / "revisions" / f"{self.attempt:06d}"
            directory.mkdir(parents=True)
            images = directory / "images"
            images.mkdir()
            try:
                with self.frame_lock:
                    frames = [dict(f) for f in frames if Path(f["path"]).is_file()]
                    for f in frames:
                        shutil.copy2(f["path"], images / f["frame_id"])
                self.state.update(mapping_state="processing", processing_keyframes=len(frames), replaced_jobs=self.jobs.dropped)
                started = time.perf_counter()
                database = directory / "database.db"
                extraction = pycolmap.FeatureExtractionOptions()
                extraction.num_threads = self.args.threads
                extraction.max_image_size = self.args.width
                extraction.sift.max_num_features = 1200
                extraction.sift.max_num_orientations = 1
                extraction.sift.first_octave = 0
                extraction.use_gpu = False
                reader = pycolmap.ImageReaderOptions()
                reader.camera_model = "SIMPLE_RADIAL"
                pycolmap.extract_features(database, images, camera_mode=pycolmap.CameraMode.SINGLE,
                    reader_options=reader, extraction_options=extraction, device=pycolmap.Device.cpu)
                matching = pycolmap.FeatureMatchingOptions()
                matching.num_threads = self.args.threads
                matching.use_gpu = False
                matching.sift.max_ratio = .75
                matching.sift.cpu_brute_force_matcher = True
                pairing = pycolmap.SequentialPairingOptions()
                pairing.overlap, pairing.quadratic_overlap = 4, False
                verification = pycolmap.TwoViewGeometryOptions()
                verification.ransac.random_seed = 42
                verification.ransac.max_error = 1.5
                pycolmap.match_sequential(database, matching, pairing, verification, pycolmap.Device.cpu)
                options = pycolmap.IncrementalPipelineOptions()
                options.num_threads, options.random_seed, options.min_model_size = self.args.threads, 42, 4
                options.mapper.num_threads, options.mapper.random_seed = self.args.threads, 42
                models = pycolmap.incremental_mapping(database, images, directory / "sparse", options)
                if not models:
                    raise ValueError("no_connected_model")
                model = max(models.values(), key=lambda m: (m.num_reg_images(), m.num_points3D()))
                scene = scene_from_model(model, frames, self.previous, self.segment)
                rejected = isolated_pose_spikes(scene["cameras"])
                if rejected:
                    options.image_names = [f["frame_id"] for f in frames if f["frame_id"] not in rejected]
                    retry = pycolmap.incremental_mapping(database, images, directory / "refined", options)
                    if not retry:
                        raise ValueError("pose_spike_refinement_failed")
                    model = max(retry.values(), key=lambda m: (m.num_reg_images(), m.num_points3D()))
                    scene = scene_from_model(model, frames, self.previous, self.segment)
                    scene["quality"]["excluded_frames"] = rejected
                    scene["quality"]["submitted_frames"] = len(frames) - len(rejected)
                # Native landmark IDs/2D observations are still intact before voxel fusion.
                current_cameras = list(scene["cameras"])
                objects, object_token = self.plane_map.prepare(model, scene, frames, directory)
                history = self.history if scene["gauge_aligned"] else []
                cell_size = self.cell_size if scene["gauge_aligned"] else None
                if cell_size is None:
                    xyz = np.array([p[1:4] for p in scene["points"]])
                    cell_size = self.args.voxel_size or float(np.linalg.norm(np.ptp(xyz, axis=0))) / 200
                scene["quality"]["current_window_points"] = len(scene["points"])
                scene["points"] = fuse_history(history, scene["points"], cell_size / 2)
                scene["quality"]["points3D"] = len(scene["points"])
                xyz = np.array([p[1:4] for p in scene["points"]])
                scene["quality"]["bounds"] = {"min": xyz.min(axis=0).tolist(), "max": xyz.max(axis=0).tolist()}
                objects["scene_binding_sha256"] = scene_binding(scene)
                scene["planar_objects"] = objects
                scene["quality"]["processing_seconds"] = time.perf_counter() - started
                scene["revision"] = self.attempt
                atomic_json(directory / "scene.json", scene)
                atomic_json(directory / "local_assets.json", {"images_root": str(images.resolve()),
                    "frames": [c["frame_id"] for c in scene["cameras"]]})
                atomic_json(directory / "status.json", {"status": "complete"})
                self.state.update(status="mapping", mapping_state="updated", map_available=True,
                    revision=self.attempt, revision_path=f"revisions/{self.attempt:06d}", segment_id=scene["segment_id"],
                    registered_frames=len(current_cameras), points3D=len(scene["points"]),
                    update_seconds=scene["quality"]["processing_seconds"],
                    objects=objects.get("statistics", {}),
                    publish_lag_seconds=time.monotonic() - max(f["received_at"] for f in frames),
                    last_map_timestamp_s=max(f["timestamp_s"] for f in frames), last_error=None)
                self.plane_map.commit(object_token)
                self.previous, self.segment = current_cameras, scene["segment_id"]
                self.history, self.cell_size = scene["points"], cell_size
            except Exception as error:
                # Keep the previous valid map and expose degraded state, not fabricated geometry.
                atomic_json(directory / "status.json", {"status": "failed", "reason": type(error).__name__ + ": " + str(error)[:200]})
                self.state.update(mapping_state="degraded", failed_updates=self.state.get("failed_updates") + 1,
                                  last_error=str(error) if isinstance(error, ValueError) and re.fullmatch(r"[a-z_]+", str(error)) else type(error).__name__)
            self.prune()

    def prune(self):
        root = (self.args.session / "revisions").resolve()
        keep = {self.state.get("revision_path",)}
        directories = sorted(p for p in root.iterdir() if p.is_dir() and re.fullmatch(r"\d{6}", p.name))
        for candidate in directories[:-self.args.keep_revisions]:
            if candidate.resolve().parent != root or candidate.is_symlink() or f"revisions/{candidate.name}" in keep:
                continue
            shutil.rmtree(candidate)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, help="Local video, RTSP/UDP URL, or camera index")
    parser.add_argument("--session", type=Path, required=True, help="New private output directory")
    parser.add_argument("--sample-fps", type=float, default=2)
    parser.add_argument("--replay-rate", type=float, default=1)
    parser.add_argument("--max-seconds", type=float, default=0)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--min-sharpness", type=float, default=20)
    parser.add_argument("--max-keyframes", type=int, default=24)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--keep-revisions", type=int, default=3)
    parser.add_argument("--voxel-size", type=float, help="World units; arbitrary unless scale is externally calibrated")
    parser.add_argument("--plane-profile", choices=("balanced", "coverage", "strict"), default="balanced")
    parser.add_argument("--plane-error-px", type=float, help="Pixel-equivalent plane tolerance, 1..12")
    args = parser.parse_args()
    if (args.session.exists() or not all(np.isfinite(v) for v in (args.sample_fps, args.replay_rate, args.max_seconds, args.min_sharpness))
        or args.sample_fps <= 0 or args.replay_rate < 0 or args.max_seconds < 0 or args.min_sharpness < 0 or args.width < 320 or
        (args.voxel_size is not None and (not np.isfinite(args.voxel_size) or args.voxel_size <= 0)) or
        args.batch_size < 4 or args.max_keyframes < args.batch_size or args.keep_revisions < 2 or args.threads < 1):
        parser.error("Use a new session and positive, bounded input/processing settings")
    try:
        detection_settings(args.plane_profile, args.plane_error_px)
    except ValueError as error:
        parser.error(str(error))
    args.session = args.session.resolve()
    cv2.setNumThreads(1)
    args.session.mkdir(parents=True)
    atomic_json(args.session / "settings.json", {"pycolmap": pycolmap.__version__, "opencv": cv2.__version__,
        "max_keyframes": args.max_keyframes, "batch_size": args.batch_size, "sample_fps": args.sample_fps,
        "width": args.width, "threads": args.threads, "min_sharpness": args.min_sharpness,
        "max_fused_points": 50000, "replay_rate": args.replay_rate, "voxel_size": args.voxel_size,
        "plane_detection": {"profile": args.plane_profile, "plane_error_px": args.plane_error_px},
        "plane_update_mode": "incremental_keyframe; per_received_frame_LK_tracking",
        "feature_extraction": {"max_num_features": 1200, "max_num_orientations": 1, "first_octave": 0},
        "source": "local_file" if Path(args.source).is_file() else "live_uri_or_camera_index; credentials_not_stored"})
    (args.session / "frames").mkdir()
    (args.session / "revisions").mkdir()
    state, raw, jobs, stop, frame_lock = State(args.session), LatestSlot(), LatestSlot(), Event(), Lock()
    state.update()
    worker = Mapper(args, state, jobs, frame_lock)
    receiver = Thread(target=capture, args=(args.source, raw, stop, state, args.replay_rate, args.max_seconds), daemon=True)
    frames, last_sample, previous_gray, accepted = deque(), -1e9, None, 0
    tracker = FrameTracker(args.plane_profile, args.plane_error_px)
    proposals = LatestSlot()
    proposer = Thread(target=propose_boundaries, args=(proposals, tracker, frame_lock, state), daemon=True)
    proposer.start()
    worker.start()
    receiver.start()
    try:
        while (packet := raw.get()) is not None:
            tracking = tracker.observe(packet.image)
            state.update(frame_tracking=tracking)
            if packet.timestamp_s - last_sample < 1 / args.sample_fps:
                continue
            image = packet.image
            if image.shape[1] > args.width:
                image = cv2.resize(image, (args.width, round(image.shape[0] * args.width / image.shape[1])))
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            small = cv2.resize(gray, (160, 90))
            motion = float(np.mean(cv2.absdiff(small, previous_gray))) if previous_gray is not None else 255
            if sharpness < args.min_sharpness or motion < 2.0:
                state.update(last_quality_rejection="blur" if sharpness < args.min_sharpness else "redundant_view")
                continue
            accepted += 1
            name = f"kf_{accepted:07d}.jpg"
            path = args.session / "frames" / name
            with frame_lock:
                if not cv2.imwrite(str(path), image):
                    raise RuntimeError("Keyframe write failed")
                frames.append({"frame_id": name, "path": str(path), "timestamp_s": packet.timestamp_s,
                    "received_at": packet.received_at,
                    "source_pts_s": packet.source_pts_s, "timestamp_source": packet.timestamp_source,
                    "sharpness": sharpness, "plane_quads": None})
                proposal_meta = frames[-1]
                while len(frames) > args.max_keyframes:
                    old = Path(frames.popleft()["path"])
                    if old.resolve().parent == (args.session / "frames").resolve():
                        old.unlink(missing_ok=True)
            proposals.put((proposal_meta, image))
            previous_gray, last_sample = small, packet.timestamp_s
            state.update(keyframes=accepted, active_keyframes=len(frames), frame_drops=raw.dropped,
                         ingestion_lag_seconds=time.monotonic() - packet.received_at)
            if len(frames) >= args.batch_size:
                jobs.put(list(frames))
    except KeyboardInterrupt:
        state.update(capture_state="stopped")
    except Exception as error:
        state.update(capture_state="failed", source_error=type(error).__name__)
    finally:
        stop.set()
        raw.close()
        receiver.join(timeout=6)
        proposals.close()
        proposer.join()
        if len(frames) >= args.batch_size:
            jobs.put(list(frames))
        jobs.close()
        worker.join()
        capture_state = state.get("capture_state")
        final_status = ("complete" if state.get("map_available") else "failed") if capture_state in {"eof", "duration_limit"} else (
            "failed" if capture_state == "failed" else "stopped_or_disconnected")
        state.update(status=final_status,
                     mapping_state="idle", frame_drops=raw.dropped, replaced_jobs=jobs.dropped)
        print(json.dumps({"session": str(args.session), "map_available": state.get("map_available"),
                          "revision": state.get("revision"), "status": state.get("status")}))
        if not state.get("map_available") or state.get("capture_state") in {"failed", "disconnected"}:
            raise SystemExit(2)


if __name__ == "__main__":
    main()
