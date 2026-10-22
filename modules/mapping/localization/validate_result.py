"""Independent standard-library acceptance checks for exported PLY, poses and scale."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def mv(matrix, vector):
    return [sum(a * b for a, b in zip(row, vector)) for row in matrix]


def transpose(matrix):
    return list(map(list, zip(*matrix)))


def near(a, b, tolerance=1e-7):
    return len(a) == len(b) and all(abs(x - y) < tolerance for x, y in zip(a, b))


def rotation(matrix):
    require(len(matrix) == 3 and all(len(row) == 3 for row in matrix), "Invalid rotation dimensions")
    for i, row in enumerate(matrix):
        require(near([sum(a * b for a, b in zip(row, other)) for other in matrix],
                     [float(i == j) for j in range(3)]), "Rotation is not orthonormal")
    a, b, c = matrix
    determinant = a[0] * (b[1]*c[2] - b[2]*c[1]) - a[1] * (b[0]*c[2] - b[2]*c[0]) + a[2] * (b[0]*c[1] - b[1]*c[0])
    require(abs(determinant - 1) < 1e-7, "Rotation changes handedness")


def validate_result(result, snapshot):
    result, snapshot = Path(result), Path(snapshot)
    require(read(result / "status.json")["status"] == "complete", "Result is not complete")
    scene, poses, provenance = [read(result / name) for name in ("scene.json", "poses.json", "provenance.json")]
    for name, expected in provenance["outputs"].items():
        require(name in {"scene.json", "poses.json", "points.ply", "trajectory.csv", "quality.json", "scale.json"}, "Unknown output")
        require(hashlib.sha256((result / name).read_bytes()).hexdigest() == expected, "Output checksum differs")
    require(scene["schema_version"] == "phone-sfm-result/1.0", "Unsupported result schema")
    require(scene["cameras"] == poses["cameras"], "Pose exports differ")
    scale, quality = scene["scale"], scene["quality"]
    require(scale == read(result / "scale.json") and quality == read(result / "quality.json"), "Sidecar differs")
    require(scene["pose_source"] == "sfm", "Wrong pose source")
    require(scale["unit"] in {"m", "arbitrary"} and math.isfinite(scale["factor"]) and scale["factor"] > 0, "Invalid scale")
    require(scale["independent_length_validation"] == "not_performed_unconfirmed_floor_C", "Unconfirmed height reported as valid")
    rotation(scale["rotation"])
    if scale["unit"] == "m":
        require(scale["status"] == "reference_scaled" and scale["reference_length_m"] == 0.44, "Missing scale reference")
        require(abs(scale["factor"] * scale["raw_reference_length"] - 0.44) < 1e-9, "Scale factor mismatch")
        for info in scale["anchor_quality"].values():
            require(info["views"] >= 3 and info["max_reprojection_error_px"] <= 6 and info["max_ray_angle_deg"] >= 1.5,
                    "Unsafe anchor geometry")
    frames_doc = read(snapshot / "features_v1/frames_manifest.json")
    frames = {f["frame_id"]: f for f in frames_doc["frames"]}
    require(scene["input_run_id"] == frames_doc["input_run_id"], "Input run mismatch")
    camera_names = [c["frame_id"] for c in scene["cameras"]]
    require(len(camera_names) == len(set(camera_names)) > 0, "Duplicate/empty camera list")
    require(len(camera_names) == quality["registered_frames"], "Camera count differs")
    require(set(quality["unregistered_frames"]) == set(frames) - set(camera_names), "Unregistered frames mismatch")
    times = []
    for camera in scene["cameras"]:
        name = camera["frame_id"]
        require(name in frames and camera["source_pts_s"] == frames[name]["source_pts_s"], "Frame/PTS differs")
        times.append(camera["source_pts_s"])
        raw_rotation = camera["cam_from_world_rotation_raw"]
        rotation(raw_rotation)
        rotation(camera["world_from_camera_rotation"])
        raw_center = [-v for v in mv(transpose(raw_rotation), camera["cam_from_world_translation_raw"])]
        actual = [scale["factor"] * x + t for x, t in zip(mv(scale["rotation"], raw_center), scale["translation"])]
        require(near(actual, camera["position"]), "Camera center transform differs from point transform")
        expected_rotation = transpose([mv(scale["rotation"], col) for col in raw_rotation])
        require(all(near(a, b) for a, b in zip(expected_rotation, camera["world_from_camera_rotation"])), "Camera orientation mismatch")
    require(all(a < b for a, b in zip(times, times[1:])), "Unsorted/duplicate timestamps")
    require(len(scene["points"]) == quality["points3D"] > 0, "Point count differs")
    require(len({p[0] for p in scene["points"]}) == len(scene["points"]), "Duplicate point IDs")
    with (result / "points.ply").open(encoding="ascii") as stream:
        require(stream.readline().strip() == "ply", "Not PLY")
        count = None
        for line in stream:
            if line.startswith("element vertex "):
                count = int(line.split()[-1])
            if line.strip() == "end_header":
                break
        ply = [list(map(float, line.split())) for line in stream if line.strip()]
    require(count == len(ply) == len(scene["points"]), "PLY count mismatch")
    for exported, point in zip(ply, scene["points"]):
        require(len(point) == 9 and all(math.isfinite(v) for v in point), "Invalid point row")
        require(near(exported, point[1:7]), "PLY coordinates/colors differ from viewer")
        require(all(0 <= v <= 255 and int(v) == v for v in point[4:7]), "Invalid RGB")
    with (result / "trajectory.csv").open(newline="", encoding="utf-8") as stream:
        trajectory = list(csv.DictReader(stream))
    require(len(trajectory) == len(camera_names), "Trajectory count differs")
    for row, camera in zip(trajectory, scene["cameras"]):
        require(row["frame_id"] == camera["frame_id"] and float(row["source_pts_s"]) == camera["source_pts_s"], "Trajectory frame differs")
        require(row["unit"] == scale["unit"] and near([float(row[a]) for a in "xyz"], camera["position"]), "Trajectory transform differs")
    return {"result": "passed", "registered_frames": len(camera_names), "points3D": len(ply),
            "scale_status": scale["status"], "checks": ["hashes", "PLY/JSON", "pose/PTS/CSV", "similarity transform", "height exclusion"]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, default=Path(__file__).resolve().parents[3] / "data/mapping/phone_video/2f1_20261021")
    args = parser.parse_args()
    print(json.dumps(validate_result(args.result, args.snapshot), ensure_ascii=False))
