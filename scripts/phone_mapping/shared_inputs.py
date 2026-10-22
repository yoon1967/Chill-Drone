"""Read an immutable Git input snapshot; optionally attach an extracted Drive package.

The Git-only checks use Python's standard library. Producer JSON paths are
historical provenance: use load_run() and the manifests to resolve current paths.
"""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import shutil


DEFAULT_RUN = Path(__file__).resolve().parents[2] / "data/mapping/phone_video/2f1_20261021"
REQUIRED_FILES = {
    "keypoints", "matches", "frames", "feature_metadata", "feature_quality",
    "feature_validation", "manual_picks", "measurement_anchors",
    "measurement_observations", "annotation_quality", "external_artifacts",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def inside(root: Path, name: str) -> Path:
    root = root.resolve()
    relative = PurePosixPath(name)
    require(bool(name) and not relative.is_absolute() and ".." not in relative.parts
            and "\\" not in name and ":" not in name, f"Invalid relative path: {name}")
    result = root.joinpath(*relative.parts).resolve()
    require(result.is_relative_to(root), f"Path outside root: {name}")
    return result


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def check_file(path: Path, record: dict) -> None:
    require(path.is_file(), f"Missing file: {path}")
    require(path.stat().st_size == record["bytes"] and sha256(path) == record["sha256"],
            f"File checksum/size differs: {path}")


def load_run(run_dir: Path = DEFAULT_RUN, package_root: Path | None = None) -> dict:
    """Return verified current paths; importing this module does not need NumPy/COLMAP."""
    root = Path(run_dir).resolve()
    manifest = read_json(root / "run_manifest.json")
    require(manifest["schema_version"] == "phone-mapping-snapshot/1.0", "Unsupported snapshot version")
    require(set(manifest["files"]) == REQUIRED_FILES, "Snapshot inventory differs")
    paths = {}
    for key, record in manifest["files"].items():
        paths[key] = inside(root, record["path"])
        check_file(paths[key], record)
    external = read_json(paths["external_artifacts"])
    require(external["input_run_id"] == manifest["input_run_id"], "External run differs")
    result = {"root": root, "manifest": manifest, "paths": paths,
              "external": external, "package_paths": {}}
    if package_root is not None:
        package_root = Path(package_root).resolve()
        check_file(package_root / "handoff_manifest.json", external["package_manifest"])
        package_manifest = read_json(package_root / "handoff_manifest.json")
        require(package_manifest["package_id"] == external["package_id"], "Wrong external package")
        require(package_manifest["input_run_id"] == manifest["input_run_id"], "Wrong external input run")
        require(package_manifest["feature_run_id"] == manifest["feature_run_id"] and
                package_manifest["annotation_run_id"] == manifest["annotation_run_id"], "External versions differ")
        for key, record in external["artifacts"].items():
            path = inside(package_root, record["path"])
            if record["kind"] == "file":
                check_file(path, record)
            else:
                require(path.is_dir(), f"Missing directory: {path}")
            result["package_paths"][key] = path
        for record in manifest["files"].values():
            if "package_path" in record:
                check_file(inside(package_root, record["package_path"]), record)
        for frame in read_json(paths["frames"])["frames"]:
            check_file(inside(result["package_paths"]["images"], frame["frame_id"]),
                       {"sha256": frame["sha256"], "bytes": external["frame_bytes"][frame["frame_id"]]})
        result["package_root"] = package_root
        result["package_manifest"] = package_manifest
    return result


def csv_rows(path: Path):
    with path.open(newline="", encoding="utf-8") as stream:
        yield from csv.DictReader(stream)


def validate(run: dict) -> dict:
    paths, manifest = run["paths"], run["manifest"]
    require(manifest["height_correspondence_ready"] is False, "Unconfirmed floor height must stay disabled")
    metadata = read_json(paths["feature_metadata"])
    frame_data = read_json(paths["frames"])
    quality = read_json(paths["feature_quality"])
    frames = {f["frame_id"]: f for f in frame_data["frames"]}
    require(len(frames) == len(frame_data["frames"]) > 0, "Duplicate/empty frames")
    require(frame_data["input_run_id"] == metadata["input_run_id"] == manifest["input_run_id"], "Input run differs")
    require(metadata["feature_run_id"] == quality["feature_run_id"] == manifest["feature_run_id"], "Feature run differs")
    require(frame_data["frames"] == quality["frames"], "Frame/quality records differ")
    times = [f["source_pts_s"] for f in frame_data["frames"]]
    require(all(math.isfinite(t) and t >= 0 for t in times) and
            all(a < b for a, b in zip(times, times[1:])), "Invalid frame PTS")
    points, point_counts = {}, Counter()
    for row in csv_rows(paths["keypoints"]):
        name, index = row["frame_id"], int(row["keypoint_id"])
        require(name in frames and (name, index) not in points, "Unknown/duplicate keypoint")
        frame = frames[name]
        x, y = float(row["x_px"]), float(row["y_px"])
        require(index >= 0 and math.isfinite(x) and math.isfinite(y) and
                0 <= x < frame["width"] and 0 <= y < frame["height"], "Invalid keypoint coordinate/index")
        require(row["run_id"] == manifest["input_run_id"] and int(row["image_id"]) == frame["image_id"]
                and float(row["source_pts_s"]) == frame["source_pts_s"], "Keypoint frame metadata differs")
        points[name, index] = (x, y)
        point_counts[name] += 1
    require(len(points) == quality["summary"]["keypoints"], "Keypoint total differs")
    for name, frame in frames.items():
        require(point_counts[name] == frame["keypoints"] and
                all((name, i) in points for i in range(frame["keypoints"])), "Noncontiguous keypoint indices")
    pairs = {(p["frame_id_a"], p["frame_id_b"]): p for p in quality["pairs"]}
    seen, raw_counts, inlier_counts = set(), Counter(), Counter()
    for row in csv_rows(paths["matches"]):
        a, b = row["frame_id_a"], row["frame_id_b"]
        ia, ib = int(row["keypoint_id_a"]), int(row["keypoint_id_b"])
        key = a, ia, b, ib
        require(a != b and (a, b) in pairs and (a, ia) in points and (b, ib) in points, "Match references unknown point/pair")
        require(key not in seen and row["geometric_inlier"] in ("0", "1"), "Duplicate match/invalid inlier flag")
        seen.add(key)
        raw_counts[a, b] += 1
        inlier_counts[a, b] += int(row["geometric_inlier"])
    for pair, record in pairs.items():
        require(raw_counts[pair] == record["matches"] and
                inlier_counts[pair] == record["geometric_inliers"], "Pair counts differ")
    require(len(seen) == quality["summary"]["raw_matches"] and
            sum(inlier_counts.values()) == quality["summary"]["geometric_inlier_correspondences"], "Match totals differ")
    anchors, picks = read_json(paths["measurement_anchors"]), read_json(paths["manual_picks"])
    for source in (anchors, picks):
        require(source["input_run_id"] == manifest["input_run_id"] and
                source["feature_run_id"] == manifest["feature_run_id"] and
                source["annotation_run_id"] == manifest["annotation_run_id"], "Measurement version differs")
        height = next(s for s in source["segments"] if s["segment_id"] == "height_136cm")
        width = next(s for s in source["segments"] if s["segment_id"] == "width_44cm")
        require(height["length_m"] == 1.36 and width["length_m"] == 0.44, "Measured lengths differ")
        require(height["correspondence_ready"] is False and
                next(a for a in source["anchors"] if a["anchor_id"] == "C")["use_for_metric_validation"] is False,
                "Unconfirmed floor height must stay disabled")
    require(anchors["manual_picks_sha256"] == sha256(paths["manual_picks"]), "Manual input hash differs")
    observations = list(csv_rows(paths["measurement_observations"]))
    require(len(observations) == len(anchors["observations"]) == 15, "Manual observation count differs")
    require(anchors["anchors"] == picks["anchors"] and anchors["segments"] == picks["segments"], "Manual definitions differ")
    picked = {f["frame_id"]: f for f in picks["frames"]}
    for row, obs in zip(observations, anchors["observations"]):
        frame = frames[obs["frame_id"]]
        require(obs["anchor_id"] in {"A", "B", "C"} and obs["frame_id"] in picked, "Unknown manual observation")
        require(obs["image_id"] == frame["image_id"] and obs["source_pts_s"] == frame["source_pts_s"], "Manual frame differs")
        xy = obs["x_px"], obs["y_px"]
        require(all(math.isfinite(v) for v in xy) and 0 <= xy[0] < frame["width"] and 0 <= xy[1] < frame["height"], "Manual coordinate outside frame")
        integer_xy = picked[obs["frame_id"]]["points_opencv_px"][obs["anchor_id"]]
        require(list(xy) == [v + 0.5 for v in integer_xy], "Manual pixel convention differs")
        require(obs["estimated_annotation_radius_px"] > 0, "Invalid manual uncertainty")
        for key, value in obs.items():
            if key in row:
                require(row[key] == ("" if value is None else str(value)), f"Manual CSV/JSON differs: {key}")
        if obs["anchor_id"] == "C" or obs["status"] == "provisional_structure_alignment":
            require(obs["metric_validation_eligible"] is False, "Unconfirmed C must stay disabled")
    return {"status": "passed_with_declared_height_limitation", "input_run_id": manifest["input_run_id"],
            "frames": len(frames), "keypoints": len(points), "matches": len(seen),
            "geometric_inliers": sum(inlier_counts.values()), "manual_observations": len(observations),
            "external_package": "verified" if "package_root" in run else "not_requested",
            "not_verified": ["3D reconstruction", "same physical floor C across views", "recipient environment"]}


def prepare_workspace(run: dict, target: Path) -> dict:
    """Create a new writable copy, preserving the delivered package and Git snapshot."""
    require("package_root" in run, "--workspace requires --package-root")
    target = target.resolve()
    require(not target.exists(), "Workspace already exists; choose a new directory")
    root = run["package_root"]
    require(not target.is_relative_to(root) and not root.is_relative_to(target)
            and not target.is_relative_to(run["root"]) and not run["root"].is_relative_to(target),
            "Workspace must be separate from the package and Git input snapshot")
    # Validate every source before writing any files; do not run code from the package.
    inventory = run["package_manifest"]["files"]
    for name, record in inventory.items():
        check_file(inside(root, name), record)
    target.mkdir(parents=True)
    for name in inventory:
        destination = inside(target, name)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(inside(root, name), destination)
    shutil.copy2(root / "handoff_manifest.json", target / "handoff_manifest.json")
    for record in run["manifest"]["files"].values():
        if "package_path" in record:
            shutil.copy2(inside(run["root"], record["path"]), inside(target, record["package_path"]))
    return {key: str(inside(target, record["path"])) for key, record in run["external"]["artifacts"].items()}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--package-root", type=Path, help="Extracted ZIP root containing handoff_manifest.json")
    parser.add_argument("--native", action="store_true", help="Additionally check the package with PyCOLMAP 4.2.1")
    parser.add_argument("--workspace", type=Path, help="Create a new writable copy at this location")
    parser.add_argument("--report", type=Path, help="Write report outside the immutable snapshot/package")
    args = parser.parse_args()
    try:
        run = load_run(args.run_dir, args.package_root)
        if args.report:
            require(not args.report.resolve().is_relative_to(run["root"]) and
                    (args.package_root is None or not args.report.resolve().is_relative_to(args.package_root.resolve())),
                    "Report must be outside snapshot/package")
        result = validate(run)
        if args.native:
            require(args.package_root is not None, "--native requires --package-root")
            from check_handoff import check
            result["native_package_check"] = check(args.package_root.resolve(), True, "git-snapshot-local-check")
        if args.workspace:
            result["workspace_paths"] = prepare_workspace(run, args.workspace)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except (ValueError, KeyError, OSError, StopIteration, TypeError) as error:
        parser.exit(1, f"Input check failed: {error}\n")


if __name__ == "__main__":
    main()
