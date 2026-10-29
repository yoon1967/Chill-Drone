"""Consume verified phone inputs and export an offline SfM result (no raw uploads)."""
from __future__ import annotations

import argparse
import csv
from contextlib import closing
from datetime import datetime, timezone
import json
import platform
from pathlib import Path
import shutil
import sqlite3
import sys
import time

import numpy as np
import pycolmap
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "scripts/phone_mapping"))
from shared_inputs import DEFAULT_RUN, check_file, load_run, read_json, require, sha256, validate
from feature_input import load_feature_input
from path_options import writable_output
from validate_features import validate_features


def write_json(path, value):
    def encode(obj):
        if isinstance(obj, set):
            return sorted(obj)
        if isinstance(obj, Path):
            return str(obj)
        if hasattr(obj, "name"):
            return obj.name
        raise TypeError(f"Unsupported JSON value: {type(obj)}")
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False, default=encode) + "\n", encoding="utf-8")


def locate_assets(run, assets_root):
    """Accept the Drive capture/run layout or an extracted handoff, by content hash."""
    root = Path(assets_root).resolve()
    require(root.is_dir(), "Local asset root does not exist")
    databases = []
    for path in root.rglob("database.db"):
        if path.is_file() and sha256(path) == run["external"]["artifacts"]["database"]["sha256"]:
            databases.append(path)
    require(len(databases) == 1, "Expected exactly one matching source database below --assets-root")
    frames = read_json(run["paths"]["frames"])["frames"]
    directories = []
    for candidate in root.rglob(frames[0]["frame_id"]):
        directory = candidate.parent
        try:
            for frame in frames:
                check_file(directory / frame["frame_id"], {
                    "sha256": frame["sha256"], "bytes": run["external"]["frame_bytes"][frame["frame_id"]]})
        except ValueError:
            continue
        directories.append(directory)
    require(len(directories) == 1, "Expected exactly one complete, checksum-matching frame directory")
    return databases[0], directories[0], frames


def check_database(database, run, frames):
    """Compare all supplied keypoint coordinates and pair indices with the native DB."""
    with closing(sqlite3.connect(database.as_uri() + "?mode=ro&immutable=1", uri=True)) as db:
        require(db.execute("PRAGMA integrity_check").fetchone()[0] == "ok", "Invalid SQLite input")
        images = dict(db.execute("SELECT image_id,name FROM images"))
        require(images == {f["image_id"]: f["frame_id"] for f in frames}, "DB/frame inventory differs")
        keypoints = {image_id: np.frombuffer(data, dtype=np.float32).reshape(rows, cols)[:, :2]
                     for image_id, rows, cols, data in db.execute("SELECT image_id,rows,cols,data FROM keypoints")}
        count = 0
        with run["paths"]["keypoints"].open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                actual = keypoints[int(row["image_id"])][int(row["keypoint_id"])]
                require(np.allclose(actual, [float(row["x_px"]), float(row["y_px"])], atol=1e-5, rtol=0),
                        "DB/keypoint CSV coordinates differ")
                count += 1
        pairs = {pair: set(map(tuple, np.frombuffer(data, dtype=np.uint32).reshape(rows, cols).tolist()))
                 for pair, rows, cols, data in db.execute("SELECT pair_id,rows,cols,data FROM two_view_geometries")
                 if rows}
        name_to_id = {name: image_id for image_id, name in images.items()}
        inliers = 0
        with run["paths"]["matches"].open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                if row["geometric_inlier"] != "1":
                    continue
                a, b = name_to_id[row["frame_id_a"]], name_to_id[row["frame_id_b"]]
                ia, ib = int(row["keypoint_id_a"]), int(row["keypoint_id_b"])
                if a > b:
                    a, b, ia, ib = b, a, ib, ia
                require((ia, ib) in pairs.get(a * 2147483647 + b, set()), "DB/inlier CSV differs")
                inliers += 1
        require(inliers == sum(map(len, pairs.values())), "DB inlier count differs")
    return {"sqlite": "ok", "images": len(images), "keypoints_compared": count, "inliers_compared": inliers}


def triangulate_anchor(model, observations):
    """Undistort rays, solve their intersection, then optimize in original pixels."""
    images = {im.name: im for im in model.images.values() if im.has_pose}
    usable = [o for o in observations if o["frame_id"] in images and
              str(o["metric_validation_eligible"]).lower() == "true"]
    require(len(usable) >= 3, "Need at least three registered, eligible anchor observations")
    rays, centers, views = [], [], []
    for obs in usable:
        image = images[obs["frame_id"]]
        camera = model.cameras[image.camera_id]
        pose = image.cam_from_world()
        xy = np.array([float(obs["x_px"]), float(obs["y_px"])])
        uv = camera.cam_from_img(xy)
        require(uv is not None and np.isfinite(uv).all(), "Anchor undistortion failed")
        ray = pose.rotation.matrix().T @ np.r_[uv, 1.0]
        rays.append(ray / np.linalg.norm(ray))
        centers.append(image.projection_center())
        views.append((camera, pose, xy, obs))
    projection = [np.eye(3) - np.outer(ray, ray) for ray in rays]
    initial = np.linalg.lstsq(np.concatenate(projection),
                             np.concatenate([p @ c for p, c in zip(projection, centers)]), rcond=None)[0]

    def residual(point):
        values = []
        for camera, pose, xy, _ in views:
            projected = camera.img_from_cam(pose * point)
            values.extend((projected - xy) if projected is not None else [1e6, 1e6])
        return np.array(values)

    fit = least_squares(residual, initial, loss="soft_l1", f_scale=3.0, max_nfev=200)
    errors = np.linalg.norm(residual(fit.x).reshape(-1, 2), axis=1)
    angles = [np.degrees(np.arccos(np.clip(np.dot(a, b), -1, 1)))
              for i, a in enumerate(rays) for b in rays[i + 1:]]
    require(fit.success and np.isfinite(fit.x).all(), "Anchor fit failed")
    require(all((pose * fit.x)[2] > 0 for _, pose, _, _ in views), "Anchor is behind a camera")
    require(max(angles) >= 1.5, "Anchor parallax too small for scale estimation")
    require(float(errors.max()) <= 6.0, "Anchor reprojection error exceeds 6 pixels: " +
            str({v[3]["frame_id"]: round(float(e), 3) for v, e in zip(views, errors)}))
    return fit.x, {"views": len(usable), "max_ray_angle_deg": max(angles),
                   "reprojection_rmse_px": float(np.sqrt(np.mean(errors ** 2))),
                   "max_reprojection_error_px": float(errors.max()),
                   "observations": [{"frame_id": obs["frame_id"], "error_px": float(error)}
                                    for (_, _, _, obs), error in zip(views, errors)]}


def scale_transform(model, run):
    if run.get("input_mode") == "video_features":
        return 1.0, np.eye(3), np.zeros(3), {
            "status": "arbitrary", "unit": "arbitrary", "factor": 1.0,
            "reason": "no_metric_reference", "independent_length_validation": "not_performed_no_reference",
            "rotation": np.eye(3).tolist(), "translation": [0.0, 0.0, 0.0],
            "formula": "display_xyz = factor * rotation @ sfm_xyz + translation",
            "axes": "COLMAP arbitrary world axes"}
    anchors = read_json(run["paths"]["measurement_anchors"])
    with run["paths"]["measurement_observations"].open(newline="", encoding="utf-8") as stream:
        observations = list(csv.DictReader(stream))
    width = next(s for s in anchors["segments"] if s["segment_id"] == "width_44cm")
    require(width["correspondence_ready"] and width["role"] == "scale_reference", "Scale reference not ready")
    height = next(s for s in anchors["segments"] if s["segment_id"] == "height_136cm")
    require(height["correspondence_ready"] is False, "Unconfirmed height cannot be activated here")
    try:
        a, qa = triangulate_anchor(model, [o for o in observations if o["anchor_id"] == "A"])
        b, qb = triangulate_anchor(model, [o for o in observations if o["anchor_id"] == "B"])
        raw_length = float(np.linalg.norm(b - a))
        require(raw_length > 1e-8, "Degenerate scale segment")
        factor = width["length_m"] / raw_length
        x = (b - a) / raw_length
        up = np.mean([-im.cam_from_world().rotation.matrix().T[:, 1]
                      for im in model.images.values() if im.has_pose], axis=0)
        y = up - x * np.dot(x, up)
        require(np.linalg.norm(y) > 1e-6, "Cannot determine viewing up direction")
        y /= np.linalg.norm(y)
        rotation = np.array([x, y, np.cross(x, y)])
        translation = -factor * rotation @ a
        info = {"status": "reference_scaled", "unit": "m", "factor": factor,
                "reference_length_m": width["length_m"], "raw_reference_length": raw_length,
                "anchor_quality": {"A": qa, "B": qb},
                "anchors": {"A": [0, 0, 0], "B": [width["length_m"], 0, 0]},
                "independent_length_validation": "not_performed_unconfirmed_floor_C",
                "measurement_uncertainty_m": None}
    except ValueError as error:
        factor, rotation, translation = 1.0, np.eye(3), np.zeros(3)
        info = {"status": "arbitrary", "unit": "arbitrary", "factor": 1.0,
                "reason": str(error), "independent_length_validation": "not_performed_unconfirmed_floor_C"}
    info.update({"rotation": rotation.tolist(), "translation": translation.tolist(),
                 "formula": "display_xyz = factor * rotation @ sfm_xyz + translation",
                 "axes": "right-handed; x=A to B, y=view-derived up (not measured gravity), z=x cross y"
                         if info["unit"] == "m" else "COLMAP arbitrary world axes"})
    return factor, rotation, translation, info


def export_result(model, run, images_root, frames, output, elapsed, components, verification, excluded_frames):
    factor, rotation, translation, scale = scale_transform(model, run)
    transform = lambda xyz: factor * (rotation @ xyz) + translation
    points = sorted(model.points3D.items())
    require(len(points) > 0, "Empty reconstruction")
    rows = [[int(pid), *transform(p.xyz).tolist(), *map(int, p.color), float(p.error), int(p.track.length())]
            for pid, p in points]
    frame_map = {f["frame_id"]: f for f in frames}
    cameras = []
    for im in model.images.values():
        if not im.has_pose:
            continue
        pose = im.cam_from_world()
        camera = model.cameras[im.camera_id]
        cam_rotation = rotation @ pose.rotation.matrix().T
        center = transform(im.projection_center())
        cameras.append({"image_id": im.image_id, "frame_id": im.name,
                        "source_pts_s": frame_map[im.name]["source_pts_s"],
                        "position": center.tolist(), "world_from_camera_rotation": cam_rotation.tolist(),
                        "cam_from_world_rotation_raw": pose.rotation.matrix().tolist(),
                        "cam_from_world_translation_raw": pose.translation.tolist(),
                        "camera_id": im.camera_id, "model": camera.model_name,
                        "width": camera.width, "height": camera.height, "params": camera.params.tolist()})
    cameras.sort(key=lambda c: c["source_pts_s"])
    registered = {c["frame_id"] for c in cameras}
    errors = np.array([p.error for _, p in points])
    xyz = np.array([row[1:4] for row in rows])
    quality = {"input_frames": len(frames), "registered_frames": len(cameras),
               "registration_ratio": len(cameras) / len(frames), "points3D": len(rows),
               "excluded_frames": excluded_frames,
               "submitted_frames": len(frames) - len(excluded_frames),
               "unregistered_frames": [f["frame_id"] for f in frames if f["frame_id"] not in registered],
               "mean_point_reprojection_error_px": float(errors.mean()),
               "median_point_reprojection_error_px": float(np.median(errors)),
               "p95_point_reprojection_error_px": float(np.percentile(errors, 95)),
               "mean_reprojection_error_px": float(model.compute_mean_reprojection_error()),
               "processing_seconds": elapsed, "components": components,
               "kind": "sparse_sfm", "dense_reconstruction": "not_performed",
               "absolute_pose_accuracy": "unknown_no_ground_truth",
               "bounds": {"min": xyz.min(axis=0).tolist(), "max": xyz.max(axis=0).tolist()}}
    quality["submitted_registration_ratio"] = len(cameras) / quality["submitted_frames"]
    quality["unregistered_submitted_frames"] = [name for name in quality["unregistered_frames"]
                                                if name not in excluded_frames]
    quality["result_status"] = "partial" if quality["unregistered_submitted_frames"] else "complete"
    limitations = ["희소 SfM 복원: 보이지 않는 면과 무늬 없는 면은 비어 있습니다.",
                   "카메라 보정값은 영상에서 추정했으며 실측 보정이 아닙니다."]
    if run.get("input_mode") == "video_features":
        limitations.append("실측 기준 없는 임의 단위입니다. 좌표와 이동 거리를 m·cm로 해석할 수 없습니다.")
    else:
        limitations.extend(["44cm 기준 스케일 보정은 scale.status를 확인해야 하며 독립 치수 정확도는 미검증입니다.",
                            "바닥 C 동일점 미확인으로 136cm 높이 검증을 수행하지 않았습니다."])
    if quality["result_status"] == "partial":
        limitations.append("부분 복원입니다. 선택한 모델에 등록되지 않은 프레임이 있습니다. 다른 모델의 좌표는 합치지 않았습니다.")
    result = {"schema_version": "phone-sfm-result/1.0", "input_run_id": run["manifest"]["input_run_id"],
              "created_utc": datetime.now(timezone.utc).isoformat(), "pose_source": "sfm",
              "scale": scale, "quality": quality, "cameras": cameras,
              "point_fields": ["point_id", "x", "y", "z", "r", "g", "b", "error_px", "track_length"],
              "points": rows,
              "limitations": limitations}
    if __package__:
        from .observations import sfm_observations
    else:
        from observations import sfm_observations
    result["point_observations"] = sfm_observations(points, cameras)
    write_json(output / "scene.json", result)
    write_json(output / "poses.json", {k: v for k, v in result.items() if k not in ("points", "point_fields", "point_observations")})
    write_json(output / "quality.json", quality)
    write_json(output / "scale.json", scale)
    with (output / "points.ply").open("w", encoding="ascii", newline="\n") as stream:
        stream.write("ply\nformat ascii 1.0\ncomment phone-sfm-result/1.0\n")
        stream.write(f"element vertex {len(rows)}\nproperty double x\nproperty double y\nproperty double z\n")
        stream.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for row in rows:
            stream.write(" ".join(map(str, row[1:7])) + "\n")
    with (output / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["frame_id", "source_pts_s", "x", "y", "z", "unit", "pose_source"])
        writer.writerows([c["frame_id"], c["source_pts_s"], *c["position"], scale["unit"], "sfm"] for c in cameras)
    write_json(output / "local_assets.json", {"images_root": str(images_root), "frames": sorted(registered)})
    write_json(output / "provenance.json", {
        **(run["provenance"] if "provenance" in run else
           {"input_snapshot_sha256": sha256(run["root"] / "run_manifest.json")}),
        "database_sha256": run["external"]["artifacts"]["database"]["sha256"],
        "pycolmap": pycolmap.__version__, "numpy": np.__version__, "python": platform.python_version(),
        "platform": platform.platform(), "script_sha256": sha256(Path(__file__)),
        "input_verification": verification,
        "outputs": {name: sha256(output / name) for name in
                    ["scene.json", "poses.json", "points.ply", "trajectory.csv", "quality.json", "scale.json"]}})
    print(json.dumps({"output": str(output), "quality": quality, "scale_status": scale["status"]}, ensure_ascii=False))
    return result


def prepare_video_features(input_dir, features_dir, report_path):
    """Verify the complete producer handoff without modifying its DB or reports."""
    source = load_feature_input(input_dir)
    require(source["schema"] == "phone-video-input/1.0", "Use a stage-2 video input with --input-dir")
    features = Path(features_dir).resolve()
    require(read_json(features / "status.json")["status"] == "features_ready", "Features are not ready")
    metadata = read_json(features / "feature_metadata.json")
    require(metadata.get("schema_version") == "phone-feature-result/1.0", "Unsupported feature result schema")
    require(metadata.get("input_manifest_sha256") == source["manifest_sha256"], "Input manifest hash differs")
    database = features / "database.db"
    require(sha256(database) == metadata.get("database_sha256"), "Producer database checksum differs")
    verification = validate_features(input_dir, features, report_path=report_path)
    frames = read_json(features / "frames_manifest.json")["frames"]
    run = {"input_mode": "video_features", "root": source["root"], 'fixed_intrinsics': bool(source['manifest'].get('camera')),
           "manifest": {"input_run_id": source["run_id"]},
           "external": {"artifacts": {"database": {"sha256": metadata["database_sha256"]}}},
           "provenance": {"input_mode": "video_features", "input_manifest_sha256": source["manifest_sha256"],
                          "feature_run_id": metadata["feature_run_id"],
                          "feature_metadata_sha256": sha256(features / "feature_metadata.json"),
                          "coordinate_space": metadata["coordinate_space"],
                          "input_transforms": metadata["input_transforms"]}}
    return run, database, source["image_dir"], frames, verification


def reconstruct(output, *, input_dir=None, features_dir=None, run_dir=None, assets_root=None,
                threads=4, exclude_frames=()):
    """Return the selected model's scene; never overwrite outputs or source inputs."""
    require(pycolmap.__version__ == "4.2.1", "Use the pinned PyCOLMAP 4.2.1 environment")
    require(threads > 0, "Thread count must be positive")
    require((input_dir is None) == (features_dir is None), "Supply --input-dir and --features-dir together")
    require(input_dir is None or (run_dir is None and assets_root is None), "Do not mix video and snapshot inputs")
    output = writable_output(Path(output))
    require(not output.exists(), "Output exists; select a new run directory")
    source_roots = [Path(p).resolve() for p in (input_dir, features_dir) if p is not None]
    if input_dir is None:
        run_dir = Path(run_dir or DEFAULT_RUN).resolve()
        assets_root = Path(assets_root or ROOT / "data/real_data").resolve()
        source_roots = [run_dir, assets_root]
    require(all(not output.is_relative_to(root) and not root.is_relative_to(output) for root in source_roots),
            "Output must be separate from immutable source inputs")
    output.mkdir(parents=True)
    write_json(output / "status.json", {"status": "running", "stage": "input_validation"})
    started = time.perf_counter()
    timings = {"input_validation_seconds": None, "mapping_seconds": None, "export_seconds": None}
    try:
        if input_dir is not None:
            run, database, images_root, frames, verification = prepare_video_features(
                input_dir, features_dir, output / "input_validation.json")
        else:
            run = load_run(run_dir)
            snapshot_check = validate(run)
            database, images_root, frames = locate_assets(run, assets_root)
            verification = {"snapshot": snapshot_check, "native_database": check_database(database, run, frames)}
            run["provenance"] = {"input_mode": "snapshot",
                                 "input_snapshot_sha256": sha256(run["root"] / "run_manifest.json")}
        excluded = list(exclude_frames)
        require(set(excluded).issubset({f["frame_id"] for f in frames}), "Unknown excluded frame")
        require(len(excluded) == len(set(excluded)), "Duplicate excluded frame")
        names = [f["frame_id"] for f in frames if f["frame_id"] not in excluded]
        require(len(names) >= 3, "Submit at least three frames")
        timings["input_validation_seconds"] = time.perf_counter() - started
        write_json(output / "status.json", {"status": "running", "stage": "mapping"})
        mapping_started = time.perf_counter()
        shutil.copy2(database, output / "database.db")
        options = pycolmap.IncrementalPipelineOptions()
        options.num_threads = threads
        options.random_seed = 42
        options.mapper.num_threads = threads
        options.mapper.random_seed = 42
        options.triangulation.random_seed = 42
        options.ba_use_gpu = False
        if run.get('fixed_intrinsics'):
            options.ba_refine_focal_length = False
            options.ba_refine_principal_point = False
            options.ba_refine_extra_params = False
        options.image_names = names
        write_json(output / "settings.json", options.todict())
        models = pycolmap.incremental_mapping(output / "database.db", images_root, output / "sparse", options)
        require(bool(models), "SfM produced no connected model; inspect the captured log and image overlap")
        selected_id, model = max(models.items(), key=lambda item: (item[1].num_reg_images(), item[1].num_points3D()))
        timings["mapping_seconds"] = time.perf_counter() - mapping_started
        write_json(output / "status.json", {"status": "running", "stage": "export"})
        export_started = time.perf_counter()
        model.extract_colors_for_all_images(images_root)
        raw = output / "raw"
        raw.mkdir()
        model.write(raw)
        model.write_text(raw)
        model.export_PLY(raw / "points.ply")
        components = [{"model_id": mid, "registered_images": m.num_reg_images(),
                       "frames": sorted(im.name for im in m.images.values() if im.has_pose),
                       "native_path": f"sparse/{mid}",
                       "points3D": m.num_points3D(), "selected": mid == selected_id} for mid, m in models.items()]
        scene = export_result(model, run, images_root, frames, output,
                              time.perf_counter() - mapping_started, components, verification, excluded)
        require(sha256(database) == run["external"]["artifacts"]["database"]["sha256"], "Source DB changed")
        timings["export_seconds"] = time.perf_counter() - export_started
        write_json(output / "status.json", {"status": scene["quality"]["result_status"], "stage": "finished",
                   "registered_frames": len(scene["cameras"]), "submitted_frames": len(names)})
        return scene
    except BaseException as error:
        stage = read_json(output / "status.json").get("stage")
        write_json(output / "status.json", {"status": "failed", "stage": stage,
                   "reason": str(error) or type(error).__name__})
        raise
    finally:
        timings["total_seconds"] = time.perf_counter() - started
        timings["clock"] = "perf_counter"
        timings["scope"] = "input validation, database copy, SfM and result export; excludes environment installation and viewer"
        write_json(output / "timing.json", timings)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, help="Legacy Git snapshot (default: first experiment)")
    parser.add_argument("--assets-root", type=Path, help="Legacy immutable assets (default: data/real_data)")
    parser.add_argument("--input-dir", type=Path, help="Stage-2 video input directory")
    parser.add_argument("--features-dir", type=Path, help="Stage-3 feature result directory")
    parser.add_argument("--output", type=Path, required=True, help="New, private result directory (never overwritten)")
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--exclude-frame", action="append", default=[],
                        help="Explicitly omit a failed input frame; repeat as needed, original input is preserved")
    args = parser.parse_args()
    if (args.input_dir is None) != (args.features_dir is None):
        parser.error("Supply --input-dir and --features-dir together")
    if args.input_dir is not None and (args.run_dir is not None or args.assets_root is not None):
        parser.error("Do not mix video and snapshot inputs")
    reconstruct(args.output, input_dir=args.input_dir, features_dir=args.features_dir,
                run_dir=args.run_dir, assets_root=args.assets_root, threads=args.threads,
                exclude_frames=args.exclude_frame)


if __name__ == "__main__":
    main()
