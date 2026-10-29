"""Independently reload and compare saved SfM JSON, PLY and trajectory exports."""
from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import numpy as np

EXPORTS = ("scene.json", "poses.json", "points.ply", "trajectory.csv", "quality.json", "scale.json")
POINT_FIELDS = ["point_id", "x", "y", "z", "r", "g", "b", "error_px", "track_length"]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def validate_saved_result(result_dir):
    """Require a published, internally consistent result; source images are not needed."""
    root = Path(result_dir).resolve()
    status = read_json(root / "status.json")["status"]
    require(status in {"complete", "partial"}, "Result is not complete or partial")
    provenance = read_json(root / "provenance.json")
    hashes = {name: digest(root / name) for name in EXPORTS}
    require(all(provenance["outputs"][name] == sha for name, sha in hashes.items()), "Export checksum mismatch")
    scene = read_json(root / "scene.json")
    require(scene["schema_version"] == "phone-sfm-result/1.0", "Unsupported scene schema")
    require(scene["point_fields"] == POINT_FIELDS, "Unsupported point fields")
    points = np.asarray(scene["points"], dtype=float)
    cameras = scene["cameras"]
    require(points.ndim == 2 and points.shape[1] == 9 and len(points) > 0 and cameras, "Empty or malformed scene")
    require(np.isfinite(points).all(), "Non-finite point data")
    require(len(set(points[:, 0])) == len(points) and (points[:, 0] == points[:, 0].astype(np.int64)).all(), "Duplicate or invalid point IDs")
    require(((points[:, 4:7] >= 0) & (points[:, 4:7] <= 255)).all() and
            (points[:, 4:7] == points[:, 4:7].astype(int)).all(), "Invalid RGB values")
    require((points[:, 7] >= 0).all() and (points[:, 8] >= 2).all(), "Invalid point error/track length")
    quality, scale = scene["quality"], scene["scale"]
    require(read_json(root / "poses.json") == {k: v for k, v in scene.items() if k not in ("points", "point_fields", "point_observations")}, "Poses/scene differ")
    require(read_json(root / "quality.json") == quality and read_json(root / "scale.json") == scale, "Quality/scale exports differ")
    require(quality["points3D"] == len(points) and quality["registered_frames"] == len(cameras), "Scene counts differ")
    require(0 < len(cameras) <= quality["submitted_frames"] <= quality["input_frames"], "Invalid frame counts")
    require(np.isclose(quality["registration_ratio"], len(cameras) / quality["input_frames"]), "Registration ratio differs")
    if "result_status" in quality:
        expected = "partial" if quality["unregistered_submitted_frames"] else "complete"
        require(quality["result_status"] == status == expected, "Coverage/status differ")
    require(scale["unit"] in {"arbitrary", "m"}, "Unknown coordinate unit")
    factor = float(scale["factor"])
    rotation, translation = np.asarray(scale["rotation"]), np.asarray(scale["translation"])
    require(np.isfinite(factor) and factor > 0 and rotation.shape == (3, 3) and translation.shape == (3,), "Invalid scale transform")
    require(np.isfinite(rotation).all() and np.isfinite(translation).all() and
            np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-8) and np.isclose(np.linalg.det(rotation), 1), "Invalid scale rotation")
    require(len({c["frame_id"] for c in cameras}) == len(cameras) and
            len({c["image_id"] for c in cameras}) == len(cameras), "Duplicate camera IDs")
    times = [float(c["source_pts_s"]) for c in cameras]
    require(np.isfinite(times).all() and all(a < b for a, b in zip(times, times[1:])), "Camera PTS not ordered")
    for camera in cameras:
        r = np.asarray(camera["cam_from_world_rotation_raw"], dtype=float)
        t = np.asarray(camera["cam_from_world_translation_raw"], dtype=float)
        require(r.shape == (3, 3) and t.shape == (3,) and np.isfinite(r).all() and np.isfinite(t).all(), "Malformed camera pose")
        require(np.allclose(r @ r.T, np.eye(3), atol=1e-8) and np.isclose(np.linalg.det(r), 1), "Invalid camera rotation")
        require(np.allclose(camera["position"], factor * rotation @ (-r.T @ t) + translation, atol=1e-8), "Camera center/scale differ")
        require(np.allclose(camera["world_from_camera_rotation"], rotation @ r.T, atol=1e-8), "Camera orientation/scale differ")
    with (root / "points.ply").open(encoding="ascii") as stream:
        header = []
        for line in stream:
            header.append(line.strip())
            if line.strip() == "end_header":
                break
        require(header[:2] == ["ply", "format ascii 1.0"] and header[-1] == "end_header", "Unsupported PLY header")
        require(f"element vertex {len(points)}" in header, "PLY vertex count differs")
        ply = np.asarray([[float(value) for value in line.split()] for line in stream if line.strip()])
    require(ply.shape == (len(points), 6) and np.allclose(ply, points[:, 1:7], atol=1e-10, rtol=1e-10), "PLY/scene coordinates or colors differ")
    with (root / "trajectory.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    require(len(rows) == len(cameras), "Trajectory count differs")
    for row, camera in zip(rows, cameras):
        require(row["frame_id"] == camera["frame_id"] and row["unit"] == scale["unit"] and row["pose_source"] == "sfm", "Trajectory identity/unit differs")
        require(float(row["source_pts_s"]) == camera["source_pts_s"] and
                np.allclose([float(row[k]) for k in ("x", "y", "z")], camera["position"], atol=1e-10), "Trajectory coordinates/PTS differ")
    report = {"status": "passed", "input_run_id": scene["input_run_id"], "result_status": status,
              "unit": scale["unit"], "points_checked": len(points), "cameras_checked": len(cameras),
              "export_sha256": hashes,
              "checks": ["published status", "producer checksums", "JSON inventories", "PLY XYZ/RGB", "CSV camera positions/PTS",
                         "raw/display pose transform", "finite points and proper rotations"],
              "not_validated": ["absolute spatial accuracy", "correspondence ground truth", "unseen surfaces"]}
    return scene, report
