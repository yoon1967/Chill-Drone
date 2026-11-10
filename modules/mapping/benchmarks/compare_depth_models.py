"""Replay fixed ORB-SLAM3 poses against three metric depth models."""
from __future__ import annotations

import argparse
import hashlib
from io import StringIO
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import av
import cv2
import numpy as np
import torch
from huggingface_hub import hf_hub_download

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from modules.mapping.data.model_assets import DEPTH_MODEL, cached_snapshot
from modules.mapping.data.point_fusion import SemanticSurface
from modules.mapping.data.stream_mapping import DepthEstimator, PoseFrames, depth_frame, frames, write_json
from modules.mapping.localization.depth_pose import VideoMapPose
from modules.mapping.localization.pose_first import depth_in_pose_units, fixed_camera_pose

MODELS = {
    "depth-anything-v2": ("Depth Anything V2 Metric Indoor Small", "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf", "8078d68a9c75a972131914f6afd0c1723be0da7f", None),
    "metric-video-depth-anything": ("Metric Video Depth Anything Small", "depth-anything/Metric-Video-Depth-Anything-Small", "273d090f2ce17df50c2872d82c8322c45da5b4dd", "metric_video_depth_anything_vits.pth"),
    "moge-2": ("MoGe-2 Small", "Ruicheng/moge-2-vits-normal", "26b477f41595707c5db6770294c0d1721e8ed4ed", "model.pt"),
}
NEAR, FAR = .2, 8.
SAMPLE_FPS = 5
POINT_BUDGET = 32000
BASELINE = "depth-anything-v2"


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def read_npz(path):
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key] for key in data.files}


def ply_bytes(surface):
    points = np.column_stack((surface.xyz, np.rint(surface.rgb).clip(0, 255)))
    buffer = StringIO()
    buffer.write(
        f"ply\nformat ascii 1.0\ncomment coordinates are ORB-SLAM3 map units\n"
        f"element vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
    )
    np.savetxt(buffer, points, fmt="%.7g %.7g %.7g %d %d %d")
    return buffer.getvalue().encode("ascii")


def source_frames(movie, source, pose):
    with movie.open("rb") as stream:
        if hashlib.file_digest(stream, "sha256").hexdigest() != source["sha256"]:
            raise ValueError(f"source changed: {movie}")
    selection = iter(zip(pose["frame_number"], pose["timestamp_s"]))
    target = next(selection, None)
    options = dict(camera=source["camera_calibration"], depth_resolution=max(source["depth_size"]))
    for number, stamp, rgb, original in frames(movie, source["source_fps"], long_side=640):
        if target is None:
            break
        if number < target[0]:
            continue
        if number != target[0] or abs(stamp - target[1]) > 1e-7:
            raise ValueError(f"frame mismatch: {movie} frame {number}")
        rgb, params = depth_frame(rgb, original, options)
        if not np.allclose(params, pose["depth_intrinsics"], rtol=0, atol=1e-8):
            raise ValueError("intrinsics mismatch")
        yield stamp, rgb
        target = next(selection, None)
    if target is not None:
        raise ValueError(f"source ended early: {movie}")


def setup_model(key, vda_source, moge_source, mode):
    title, repo, revision, filename = MODELS[key]
    details = dict(title=title, repo=repo, hf_revision=revision)
    if key == BASELINE:
        details.update(repo=DEPTH_MODEL, hf_revision=cached_snapshot(DEPTH_MODEL).name)
        return DepthEstimator(), details
    path = vda_source if key == "metric-video-depth-anything" else moge_source
    if path is None:
        raise ValueError(f"source repository required for {key}")
    sys.path.insert(0, str(path.resolve()))
    details["code_revision"] = subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
    weight = hf_hub_download(repo, filename, revision=revision, local_files_only=True)
    if key == "metric-video-depth-anything":
        settings = dict(encoder="vits", features=64, out_channels=[48, 96, 192, 384])
        if mode == "batch":
            from video_depth_anything.video_depth import VideoDepthAnything
            model = VideoDepthAnything(**settings, metric=True)
        else:
            from video_depth_anything.video_depth_stream import VideoDepthAnything
            model = VideoDepthAnything(**settings)
        model.load_state_dict(torch.load(weight, map_location="cpu", weights_only=True), strict=True)
    else:
        from moge.model.v2 import MoGeModel
        model = MoGeModel.from_pretrained(weight)
    return model.to("cuda").eval(), details


def image_estimator(key, model, source, mode):
    width, height = source["depth_size"]
    if key == BASELINE:
        return model.estimate, dict(mode="independent one-frame", input_size=[round(width/14)*14, round(height/14)*14])
    if key == "metric-video-depth-anything":
        size = round(min(width, height) / 14) * 14
        if mode == "batch":
            def estimate_batch(rgb):
                return model.infer_video_depth(rgb, SAMPLE_FPS, input_size=size, device="cuda")[0]
            return estimate_batch, dict(mode="offline temporal windows", input_short_side=size)
        model.transform, model.id = None, -1
        model.frame_cache_list.clear()
        model.frame_id_list.clear()
        return (lambda rgb: model.infer_video_depth_one(rgb, input_size=size, device="cuda")), dict(
            mode="official causal one-frame, past feature cache", input_short_side=size)
    fov = None
    if source["camera_calibration"] is not None:
        fov = math.degrees(2 * math.atan(width / (2 * source["depth_intrinsics"][0])))

    def estimate(rgb):
        image = torch.as_tensor(rgb, device="cuda").permute(2, 0, 1).float() / 255
        return model.infer(image, resolution_level=0, fov_x=fov, use_fp16=True)["depth"].cpu().numpy()

    return estimate, dict(mode="independent one-frame", resolution_level=0, fov_x_degrees=fov)


class CommonPath:
    """Use V2 RGB-D only to bridge gaps in the shared native ORB-SLAM3 path."""

    def __init__(self):
        self.map = VideoMapPose()
        self.map.scale = 1.0
        self.bridges = []

    def step(self, stamp, rgb, depth, params, native, segment, native_scale):
        mapper = self.map
        if mapper.last_pose is None:
            pose = np.eye(4)
            mapper._bind(pose, pose, None, 1.0)
            mapper.accept_depth(stamp, rgb, depth, params, None, pose, "pose_reused", None, None, None)
            if native is not None and native_scale is not None:
                mapper._bind(pose, native, segment, mapper.scale / native_scale)
                return pose, "tracking", mapper.native_to_map[0], mapper.scale
            return pose, "origin_estimated", np.nan, mapper.scale

        pose, state = mapper.resolve(stamp, rgb, params, native, segment)
        source = state
        if native is not None and native_scale is not None and segment != mapper.native_map:
            # An RGB-D match is preferred. With no overlap, attach the new native
            # trajectory to the current fallback pose and retain that provenance.
            source = "bridge_visual" if state == "pose_reused" else "bridge_held"
            mapper._bind(pose, native, segment, mapper.scale / native_scale)
            self.bridges.append(dict(timestamp_s=round(stamp, 3), native_segment=list(segment), source=source))
            state = "tracking"
        scale, accepted = mapper.accept_depth(stamp, rgb, depth, params, None, pose, state, native, segment, native_scale)
        if source == state:
            source = accepted
        factor = mapper.native_to_map[0] if native is not None and segment == mapper.native_map else np.nan
        return pose, source, factor, scale


def milliseconds(s):
    return s * 1000


def stats(values):
    values = np.asarray(values, dtype=float)
    return {name: round(float(value), 3) for name, value in zip(
        ("p50", "p95", "p99", "max"), np.percentile(values, [50, 95, 99, 100]))}


def video_preview(path, rgb, depths):
    height, width = depths.shape[1:]
    with av.open(str(path), mode="w", options={"movflags": "+faststart"}) as output:
        video = output.add_stream("libx264", rate=SAMPLE_FPS)
        video.width, video.height, video.pix_fmt = width * 2, height, "yuv420p"
        video.options = {"preset": "veryfast", "crf": "20", "threads": "2"}
        for image, depth in zip(rgb, depths):
            codes = np.nan_to_num(np.clip((FAR - depth) / (FAR - NEAR), 0., 1.) * 255).astype(np.uint8)
            heat = cv2.applyColorMap(codes, cv2.COLORMAP_TURBO)
            heat[~np.isfinite(depth)] = 0
            frame = av.VideoFrame.from_ndarray(np.concatenate((cv2.cvtColor(image, cv2.COLOR_RGB2BGR), heat), axis=1), format="bgr24")
            for packet in video.encode(frame):
                output.mux(packet)
        for packet in video.encode():
            output.mux(packet)


def run_framewise(key, model, details, movie, source, pose, result, paced_seconds):
    common_path = result.parent / "common_trajectory.npz"
    estimate, settings = image_estimator(key, model, source, "framewise")
    reference = CommonPath() if key == BASELINE else None
    common = None if reference is not None else read_npz(common_path)
    if common is not None and (
        common["source_sha256"].item() != source["sha256"]
        or not np.array_equal(common["frame_number"], pose["frame_number"])
        or not np.array_equal(common["timestamp_s"], pose["timestamp_s"])
    ):
        raise ValueError("common trajectory differs from source frames")
    samples = len(pose["frame_number"])
    width, height = source["depth_size"]
    depths = np.empty((samples, height, width), np.float32)
    colors = np.empty((samples, height, width, 3), np.uint8)
    surface = SemanticSurface(.04, max_cells=POINT_BUDGET, spacing_increment=.01)
    labels = np.zeros((height, width), np.uint8)
    confidence = np.ones((height, width), np.float32)
    params, tracking_size = pose["depth_intrinsics"], pose["tracking_size"]
    offsets, anchors = pose["anchor_offsets"], pose["depth_anchors"]
    native_poses, native_segments = pose["world_from_camera"], pose["map_segment"]
    trace, pose_records = [], []
    began = None
    scale = 1.
    accepted_frames = 0
    print(f"START {movie.name}/{key} {samples} frames, one map", flush=True)
    torch.cuda.synchronize()
    torch.cuda.reset_peak_memory_stats()
    with torch.inference_mode():
        for i, (stamp, rgb) in enumerate(source_frames(movie, source, pose)):
            due = None
            if stamp - pose["timestamp_s"][0] < paced_seconds:
                began = time.perf_counter() if began is None else began
                due = began + stamp - pose["timestamp_s"][0]
                time.sleep(max(0., due - time.perf_counter()))
            start = time.perf_counter()
            raw = np.asarray(estimate(rgb), dtype=np.float32)
            torch.cuda.synchronize()
            inferred = time.perf_counter()
            if raw.shape != depths.shape[1:]:
                raise ValueError(f"wrong depth size: {raw.shape}")
            depths[i] = np.where(np.isfinite(raw) & (raw > 0.), raw, np.nan)
            colors[i] = rgb
            valid = np.isfinite(raw) & (raw >= NEAR) & (raw <= FAR)
            clipped = np.where(valid, raw, 0.)
            native = native_poses[i] if np.isfinite(native_poses[i]).all() else None
            segment = tuple(map(int, native_segments[i]))
            aligned = None if native is None else depth_in_pose_units(
                clipped, anchors[offsets[i]:offsets[i+1]], tracking_size)
            native_scale = None if aligned is None else aligned[1]
            if reference is not None:
                world, pose_source, factor, scale = reference.step(stamp, rgb, clipped, params, native, segment, native_scale)
                pose_records.append((world.copy(), pose_source, factor, scale))
            else:
                world, pose_source, factor = common["world_from_camera"][i], str(common["pose_source"][i]), common["native_to_world"][i]
                if native_scale is not None and np.isfinite(factor):
                    scale = float(factor * native_scale)
            fused_start = time.perf_counter()
            used = bool(valid.any())
            if used:
                surface.integrate(rgb, clipped * scale, world, params, labels, confidence,
                                  pose_predicted=pose_source in {"origin_estimated", "pose_predicted", "pose_held", "bridge_held"})
                accepted_frames += 1
            done = time.perf_counter()
            trace.append((milliseconds(inferred-start), milliseconds(fused_start-inferred),
                          milliseconds(done-fused_start), milliseconds(done-start),
                          np.nan if due is None else milliseconds(done-due), len(surface.keys), scale, used))
            if (i + 1) % 150 == 0:
                print(f"{movie.name}/{key} {i + 1}/{samples} points={len(surface.keys)}", flush=True)

    if len(trace) != samples or not accepted_frames or not len(surface.keys):
        raise RuntimeError(f"incomplete reconstruction {movie.name}/{key}")
    if reference is not None:
        world, sources, factors, scales = zip(*pose_records)
        np.savez_compressed(common_path, source_sha256=np.asarray(source["sha256"]),
                            frame_number=pose["frame_number"], timestamp_s=pose["timestamp_s"],
                            world_from_camera=np.asarray(world), pose_source=np.asarray(sources),
                            native_to_world=np.asarray(factors), reference_scale=np.asarray(scales),
                            native_segment=pose["map_segment"])
        write_json(result.parent / "pose_bridges.json", reference.bridges)
        common = read_npz(common_path)
    metrics = np.asarray(trace, dtype=np.float64)
    if not np.all(np.isfinite(depths).any(axis=(1, 2))):
        raise RuntimeError(f"at least one depth frame contains no positive finite estimate: {movie.name}/{key}")
    np.savez_compressed(result / "depth_m.npz", depth_m=depths, frame_number=pose["frame_number"],
                        timestamp_s=pose["timestamp_s"], depth_intrinsics=params)
    np.savez_compressed(result / "frame_progress.npz", frame_number=pose["frame_number"], timestamp_s=pose["timestamp_s"],
        pose_source=common["pose_source"], depth_ms=metrics[:, 0], align_ms=metrics[:, 1], voxel_ms=metrics[:, 2],
        combined_ms=metrics[:, 3], paced_due_to_map_ms=metrics[:, 4], points=metrics[:, 5].astype(np.int32),
        depth_to_map_scale=metrics[:, 6], integrated=metrics[:, 7].astype(bool))
    (result / "unified-cloud.ply").write_bytes(ply_bytes(surface))
    video_preview(result / "preview.mp4", colors, depths)
    delays = metrics[np.isfinite(metrics[:, 4]), 4]
    pose_types, counts = np.unique(common["pose_source"], return_counts=True)
    voxel = surface.metadata()
    report = dict(source=movie.name, model=details, model_settings={**settings, "fp16": True},
        resolution=source["depth_size"], sample_fps=SAMPLE_FPS, frames=samples, integrated_frames=accepted_frames,
        maps=1, points=len(surface.keys), depth_ms=stats(metrics[:, 0]), align_ms=stats(metrics[:, 1]),
        voxel_ms=stats(metrics[:, 2]), combined_ms=stats(metrics[:, 3]), first_depth_ms=round(metrics[0, 0], 3),
        estimated_depth_median_m=round(float(np.nanmedian(depths[:, ::8, ::8])), 4),
        depth_over_200ms=int(np.count_nonzero(metrics[:, 0] > 200)),
        combined_over_200ms=int(np.count_nonzero(metrics[:, 3] > 200)), paced_samples=len(delays),
        paced_due_to_map_ms=stats(delays) if len(delays) else None, paced_over_200ms=int(np.count_nonzero(delays > 200)),
        pose_source_counts=dict(zip(pose_types.tolist(), counts.tolist())), voxel=voxel,
        peak_cuda_allocated_mib=round(torch.cuda.max_memory_allocated()/2**20, 1), gpu=torch.cuda.get_device_name())
    write_json(result / "result.json", report)
    print(f"DONE {movie.name}/{key} {accepted_frames}/{samples} fused, {report['points']} points, "
          f"depth p95={report['depth_ms']['p95']} ms, combined p95={report['combined_ms']['p95']} ms", flush=True)



def prepare(movie, output, camera):
    """Track every decoded input frame and retain the exact 5 FPS benchmark samples."""
    with av.open(str(movie)) as container:
        stream = container.streams.video[0]
        fps = float(stream.average_rate)
        duration = float(stream.duration * stream.time_base) if stream.duration is not None else None
    output.mkdir(parents=True, exist_ok=False)
    producer = PoseFrames(movie, round(fps), None, output, camera, predict_pose=False)
    indices, timestamps, poses, segments, anchors = [], [], [], [], []
    offsets, next_stamp, total = [0], 0., 0
    try:
        while (packet := producer.take()) is not None:
            number, stamp, rgb, _, original, native = packet
            total += 1
            if stamp + 1e-6 < next_stamp:
                continue
            next_stamp = (math.floor((stamp + 1e-6) * SAMPLE_FPS) + 1) / SAMPLE_FPS
            image, intrinsics = depth_frame(rgb, original, dict(camera=camera, depth_resolution=384))
            tracking_size = rgb.shape[1::-1]
            pose = fixed_camera_pose(native, stamp)
            indices.append(number)
            timestamps.append(stamp)
            poses.append(pose if pose is not None else np.full((4, 4), np.nan))
            segments.append((native.get("map_id", -1), native.get("tracking_epoch", 0)))
            sparse = native.get("depth_anchors", []) if pose is not None else []
            anchors.append(np.asarray(sparse, np.float32).reshape(-1, 3))
            offsets.append(offsets[-1] + len(anchors[-1]))
    finally:
        producer.close()
    if not indices:
        raise ValueError(f"no decoded frames: {movie}")
    with movie.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    write_json(output / "source.json", dict(source=movie.name, sha256=digest, source_fps=fps,
        duration_s=duration, orb_frames=total, sample_fps=SAMPLE_FPS, sample_count=len(indices),
        depth_size=list(image.shape[1::-1]), depth_intrinsics=list(intrinsics), camera_calibration=camera))
    np.savez_compressed(output / "orbslam3_poses.npz", frame_number=np.asarray(indices, np.int32),
        timestamp_s=np.asarray(timestamps), world_from_camera=np.asarray(poses),
        map_segment=np.asarray(segments, np.int32), depth_anchors=np.concatenate(anchors),
        anchor_offsets=np.asarray(offsets, np.int64), tracking_size=np.asarray(tracking_size),
        depth_intrinsics=np.asarray(intrinsics))
    print(f"Prepared {len(indices)} samples from {total} ORB frames in {output}", flush=True)


def run_batch(key, model, details, movie, source, pose, result):
    """Reproduce the earlier offline comparison, with separate native maps."""
    estimate, settings = image_estimator(key, model, source, "batch")
    colors = np.stack([rgb for _, rgb in source_frames(movie, source, pose)])
    batched = key == "metric-video-depth-anything"
    with torch.inference_mode():
        estimate(colors[:1] if batched else colors[0])
        torch.cuda.synchronize()
        torch.cuda.reset_peak_memory_stats()
        started = time.perf_counter()
        depths = estimate(colors) if batched else np.stack([estimate(rgb) for rgb in colors])
        torch.cuda.synchronize()
        elapsed = time.perf_counter() - started
    if depths.shape != colors.shape[:3]:
        raise ValueError("model depth and input frame dimensions differ")
    depths = np.asarray(depths, np.float32)
    depths[~np.isfinite(depths) | (depths <= 0)] = np.nan
    if not np.isfinite(depths).any(axis=(1, 2)).all():
        raise ValueError("at least one frame contains no positive finite depth")
    np.savez_compressed(result / "depth_m.npz", depth_m=depths, frame_number=pose["frame_number"],
        timestamp_s=pose["timestamp_s"], depth_intrinsics=pose["depth_intrinsics"])
    video_preview(result / "preview.mp4", colors, depths)

    offsets, anchors = pose["anchor_offsets"], pose["depth_anchors"]
    segments, transforms = pose["map_segment"], pose["world_from_camera"]
    confirmed = np.isfinite(transforms).all(axis=(1, 2))
    params, tracking_size = pose["depth_intrinsics"], pose["tracking_size"]
    observations = {}
    for index in np.flatnonzero(confirmed):
        raw = depths[index]
        clipped = np.where((raw >= NEAR) & (raw <= FAR), raw, 0.)
        if np.count_nonzero(clipped) <= raw.size * .05:
            continue
        aligned = depth_in_pose_units(clipped, anchors[offsets[index]:offsets[index+1]], tracking_size)
        if aligned is not None:
            group = tuple(map(int, segments[index]))
            observations.setdefault(group, []).append((index, aligned[1]))
    labels = np.zeros(depths.shape[1:], np.uint8)
    confidence = np.ones(depths.shape[1:], np.float32)
    maps = []
    for group, rows in observations.items():
        shared = np.flatnonzero(confirmed & (segments == group).all(axis=1))
        z = np.concatenate([anchors[offsets[index]:offsets[index+1], 2] for index in shared])
        size = float(np.median(z[np.isfinite(z) & (z > 0)]) * .015)
        scale = float(np.median([value for _, value in rows]))
        surface = SemanticSurface(size, max_cells=POINT_BUDGET, spacing_increment=size / 4)
        for index, _ in rows:
            raw = depths[index]
            depth = np.where((raw >= NEAR) & (raw <= FAR), raw * scale, 0.)
            surface.integrate(colors[index], depth, transforms[index], params, labels, confidence)
        if len(surface.keys):
            file = f"cloud-map-{group[0]}-epoch-{group[1]}.ply"
            (result / file).write_bytes(ply_bytes(surface))
            maps.append(dict(file=file, map_id=group[0], epoch=group[1], frames=len(rows),
                             points=len(surface.keys), meters_to_orb_units=scale))
    write_json(result / "result.json", dict(model=details, model_settings=settings, sample_fps=SAMPLE_FPS,
        frames=len(colors), inference_s=round(elapsed, 3), effective_fps=round(len(colors) / elapsed, 2),
        peak_cuda_allocated_mib=round(torch.cuda.max_memory_allocated() / 2**20, 1), orb_maps=maps))
    print(f"DONE {movie.name}/{key} {len(colors)} frames, {len(maps)} native maps", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    preparation = commands.add_parser("prepare", help="track one source video once")
    preparation.add_argument("--video", type=Path, required=True)
    preparation.add_argument("--output", type=Path, required=True)
    preparation.add_argument("--camera", type=Path, help="matching camera calibration JSON")
    run = commands.add_parser("run", help="reuse a prepared pose archive for one model")
    run.add_argument("--video", type=Path, required=True)
    run.add_argument("--poses", type=Path, required=True)
    run.add_argument("--output", type=Path, required=True, help="directory shared by the three models for this mode")
    run.add_argument("--model", choices=MODELS, required=True)
    run.add_argument("--mode", choices=("framewise", "batch"), default="framewise")
    run.add_argument("--paced-seconds", type=float, default=0., help="feed the first N seconds on source timestamps")
    run.add_argument("--vda-source", type=Path)
    run.add_argument("--moge-source", type=Path)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.video, args.output, read_json(args.camera) if args.camera else None)
        return
    if not math.isfinite(args.paced_seconds) or args.paced_seconds < 0 or (args.mode == "batch" and args.paced_seconds):
        parser.error("--paced-seconds requires framewise mode and a nonnegative finite value")
    torch.set_num_threads(2)
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable")
    source = read_json(args.poses / "source.json")
    pose = read_npz(args.poses / "orbslam3_poses.npz")
    model, details = setup_model(args.model, args.vda_source, args.moge_source, args.mode)
    result = args.output / args.model
    if (result / "result.json").is_file():
        parser.error(f"completed result already exists: {result}")
    result.mkdir(parents=True, exist_ok=True)
    if args.mode == "batch":
        run_batch(args.model, model, details, args.video, source, pose, result)
    else:
        run_framewise(args.model, model, details, args.video, source, pose, result, args.paced_seconds)


if __name__ == "__main__":
    main()
