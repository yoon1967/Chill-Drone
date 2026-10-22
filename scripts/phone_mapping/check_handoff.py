"""Portable receiver check. Reads the package; optional native reads use a DB copy.

In the ZIP: python scripts/check_handoff.py --native --report ../receipt.json
In this repository: supply --root with the extracted package directory.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from contextlib import closing
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import sys
import tempfile

import numpy as np
import PIL
from PIL import Image


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def inside(root: Path, name: str) -> Path:
    path = (root / name).resolve()
    require(path.is_relative_to(root), f"Path outside package: {name}")
    return path


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def native_read(path: Path, names: set[str]) -> dict:
    import pycolmap

    require(pycolmap.__version__ == "4.2.1", f"Native reader must be pycolmap 4.2.1; got {pycolmap.__version__}")
    # Opening a native database can change its SQLite journal header. Test a copy.
    temp_base = Path(tempfile.gettempdir()).resolve()
    with tempfile.TemporaryDirectory(prefix="phone-mapping-native-", dir=temp_base) as folder:
        require(Path(folder).resolve().is_relative_to(temp_base), "Unexpected native-test cleanup path")
        working = Path(folder) / "database.db"
        shutil.copyfile(path, working)
        with pycolmap.Database.open(working) as db:
            require({image.name for image in db.read_all_images()} == names, "Native image names differ")
            options = pycolmap.DatabaseCacheOptions()
            options.min_num_matches = 15
            options.image_names = names
            cache = pycolmap.DatabaseCache.create(db, options)
            require(cache.num_images() == len(names), "SfM input cache did not load all images")
            require({image.name for image in cache.images.values()} == names, "SfM cache image names differ")
            result = {"status": "passed", "pycolmap_version": pycolmap.__version__,
                      "sfm_input_cache_images": cache.num_images(), "cameras": cache.num_cameras(),
                      "rigs": cache.num_rigs(), "frames": cache.num_frames(),
                      "method": "native Database read + DatabaseCache.create on temporary copy",
                      "reconstruction_executed": False}
            del cache
    return result


def check(root: Path, use_native: bool, receiver: str) -> dict:
    root = root.resolve()
    manifest = load(root / "handoff_manifest.json")
    require(manifest["schema_version"] == "1.0", "Unsupported handoff schema")
    inventory = manifest["files"]
    for name, record in inventory.items():
        path = inside(root, name)
        require(path.is_file(), f"Missing file: {name}")
        require(path.stat().st_size == record["bytes"] and sha256(path) == record["sha256"], f"Checksum/size mismatch: {name}")
    paths = {key: inside(root, value) for key, value in manifest["paths"].items()}
    source, quality, features = load(paths["source_manifest"]), load(paths["quality_report"]), load(paths["feature_metadata"])
    measurements = load(paths["measurement_anchors"])
    selected = {f["frameId"]: f for f in source["frames"] if f["selectedForFirstTest"]}
    require(len(selected) == 31 and source["runId"] == manifest["input_run_id"], "Input frame count/run_id differs")
    require(features["feature_run_id"] == measurements["feature_run_id"] == manifest["feature_run_id"], "Feature run_id differs")
    require(measurements["annotation_run_id"] == manifest["annotation_run_id"], "Annotation revision differs")
    require(features["pycolmap_version"] == manifest["producer_pycolmap_version"] == "4.2.1", "Producer version differs")
    for name, frame in selected.items():
        path = paths["images"] / name
        require(path.is_file() and sha256(path) == frame["sha256"], f"Image missing/changed: {name}")
        with Image.open(path) as image:
            require(image.size == (1920, 1080), f"Image dimensions differ: {name}")
            image.verify()
    require(sha256(paths["source_video"]) == source["sourceSha256"], "Source video checksum differs")
    require(sha256(paths["measurement_reference"]) == source["measurementReference"]["imageSha256"] == measurements["measurement_reference"]["sha256"], "Measurement reference mismatch")
    require(source["measurementReference"]["revision"] == 2, "Obsolete measurement reference")
    db_path = paths["database"]
    before = sha256(db_path)
    require(before == measurements["feature_database"]["sha256"], "Anchor/DB checksum mismatch")
    points, all_matches, all_inliers = {}, {}, {}
    frame_export = {f["frame_id"]: f for f in load(paths["frames_manifest"])["frames"]}
    with closing(sqlite3.connect(db_path.as_uri() + "?mode=ro", uri=True)) as db:
        require(db.execute("PRAGMA integrity_check").fetchone()[0] == "ok", "DB integrity failure")
        require(not db.execute("PRAGMA foreign_key_check").fetchall(), "DB foreign-key failure")
        images = {name: image_id for image_id, name in db.execute("SELECT image_id,name FROM images")}
        require(set(images) == set(selected) == set(frame_export), "DB/manifest names differ")
        for name, image_id in images.items():
            row, col, blob = db.execute("SELECT rows,cols,data FROM keypoints WHERE image_id=?", (image_id,)).fetchone()
            require(row > 0 and col >= 2 and len(blob) == row * col * 4, "Invalid DB keypoints")
            points[name] = np.frombuffer(blob, dtype=np.float32).reshape(row, col)[:, :2]
            dr, dc, data = db.execute("SELECT rows,cols,data FROM descriptors WHERE image_id=?", (image_id,)).fetchone()
            require(dr == row and dc == 128 and len(data) == dr * dc, "Invalid SIFT descriptors")
            record = frame_export[name]
            require(record["image_id"] == image_id and record["sha256"] == selected[name]["sha256"] and record["source_pts_s"] == selected[name]["sourcePtsSeconds"], "Frame ID/PTS/hash differs")
        for pair in quality["pairs"]:
            a, b = pair["frame_id_a"], pair["frame_id_b"]
            ia, ib = images[a], images[b]
            pair_id = min(ia, ib) * 2147483647 + max(ia, ib)
            for table, target in (("matches", all_matches), ("two_view_geometries", all_inliers)):
                # table is one of the fixed literals above, never user input.
                record = db.execute(f"SELECT rows,cols,data FROM {table} WHERE pair_id=?", (pair_id,)).fetchone()
                if record is None or record[0] == 0:
                    target[(a, b)] = set()
                else:
                    row, col, blob = record
                    require(col == 2 and len(blob) == row * 8, "Invalid match blob")
                    array = np.frombuffer(blob, dtype=np.uint32).reshape(row, 2)
                    if ia > ib:
                        array = array[:, ::-1]
                    target[(a, b)] = {tuple(map(int, m)) for m in array}
            require(len(all_matches[(a, b)]) == pair["matches"] and len(all_inliers[(a, b)]) == pair["geometric_inliers"], "DB/quality pair counts differ")

    csv_points = defaultdict(list)
    with paths["keypoints_csv"].open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            name = row["frame_id"]
            require(name in images and row["run_id"] == source["runId"], "Unknown CSV frame/run_id")
            require(int(row["image_id"]) == images[name] and float(row["source_pts_s"]) == selected[name]["sourcePtsSeconds"], "CSV image/PTS mismatch")
            require(int(row["keypoint_id"]) == len(csv_points[name]), "CSV duplicate or reordered feature index")
            csv_points[name].append((float(row["x_px"]), float(row["y_px"])))
    for name, xy in points.items():
        actual = np.asarray(csv_points[name], dtype=np.float64)
        require(actual.shape == xy.shape and np.isfinite(actual).all(), "CSV feature dimensions/values differ")
        require(np.allclose(actual, xy, rtol=0, atol=1e-6), "CSV coordinates differ from DB")
        require(((actual[:, 0] >= 0) & (actual[:, 0] < 1920) & (actual[:, 1] >= 0) & (actual[:, 1] < 1080)).all(), "CSV points outside image")
    raw_csv, inliers_csv = defaultdict(set), defaultdict(set)
    with paths["matches_csv"].open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            pair = row["frame_id_a"], row["frame_id_b"]
            indices = int(row["keypoint_id_a"]), int(row["keypoint_id_b"])
            require(pair in all_matches and indices not in raw_csv[pair], "Unknown/duplicate CSV match")
            require(all(0 <= k < len(points[n]) for n, k in zip(pair, indices)), "CSV match index outside DB")
            raw_csv[pair].add(indices)
            require(row["geometric_inlier"] in ("0", "1"), "Invalid CSV inlier flag")
            if row["geometric_inlier"] == "1":
                inliers_csv[pair].add(indices)
    for pair in all_matches:
        require(raw_csv[pair] == all_matches[pair] and inliers_csv[pair] == all_inliers[pair], "CSV/DB match sets differ")
    tracks = defaultdict(list)
    observations_seen = set()
    with paths["candidate_tracks_csv"].open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            name, index = row["frame_id"], int(row["keypoint_id"])
            require(name in images and 0 <= index < len(points[name]), "Track references unknown point")
            require((name, index) not in observations_seen, "Reused track observation")
            observations_seen.add((name, index))
            require(np.allclose([float(row["x_px"]), float(row["y_px"])], points[name][index], rtol=0, atol=1e-6), "Track coordinates differ")
            require(int(row["image_id"]) == images[name] and float(row["source_pts_s"]) == selected[name]["sourcePtsSeconds"], "Track ID/PTS differs")
            tracks[int(row["candidate_track_id"])].append((name, int(row["observation_count"])))
    for rows in tracks.values():
        require(len(rows) >= 2 and len({n for n, _ in rows}) == len(rows) and all(c == len(rows) for _, c in rows), "Conflicting candidate track")
    require(len(tracks) == quality["summary"]["candidate_tracks"], "Candidate track count differs")

    mdir = paths["measurement_anchors"].parent
    require((mdir / measurements["measurement_reference"]["file"]).resolve() == paths["measurement_reference"], "Measurement image relative path broken")
    require((mdir / measurements["image_root_relative_to_measurements"]).resolve() == paths["images"], "Measurement image root broken")
    require((mdir / measurements["feature_database"]["path"]).resolve() == db_path, "Measurement DB relative path broken")
    require((paths["feature_metadata"].parent / features["image_root_relative_to_full"]).resolve() == paths["images"], "Feature image root broken")
    height = next(s for s in measurements["segments"] if s["segment_id"] == "height_136cm")
    width = next(s for s in measurements["segments"] if s["segment_id"] == "width_44cm")
    require(width["length_m"] == 0.44 and height["length_m"] == 1.36, "Measured lengths differ")
    require(height["correspondence_ready"] is False, "Unconfirmed floor height must stay disabled")
    require(next(a for a in measurements["anchors"] if a["anchor_id"] == "C")["use_for_metric_validation"] is False, "Unconfirmed C must not be metric enabled")
    with paths["measurement_observations_csv"].open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    require(len(rows) == len(measurements["observations"]) == 15, "Manual observation count differs")
    for row, obs in zip(rows, measurements["observations"]):
        name = obs["frame_id"]
        require(name in images and obs["image_id"] == images[name] and obs["source_pts_s"] == selected[name]["sourcePtsSeconds"], "Manual image/PTS mismatch")
        require(0 <= obs["x_px"] < 1920 and 0 <= obs["y_px"] < 1080 and obs["estimated_annotation_radius_px"] > 0, "Invalid manual observation")
        require((row["anchor_id"], row["frame_id"], float(row["x_px"]), float(row["y_px"]), row["status"], row["metric_validation_eligible"]) == (obs["anchor_id"], name, obs["x_px"], obs["y_px"], obs["status"], str(obs["metric_validation_eligible"])), "Manual JSON/CSV mismatch")
        if obs["anchor_id"] == "C" or obs["status"] == "provisional_structure_alignment":
            require(obs["metric_validation_eligible"] is False, "Provisional C must be excluded from metric validation")
    native = native_read(db_path, set(images)) if use_native else {"status": "not_requested"}
    require(sha256(db_path) == before, "Receiver check changed package DB")
    return {"status": "passed_with_declared_height_limitation", "package_id": manifest["package_id"], "receiver_name": receiver,
            "python": sys.version.split()[0], "numpy": np.__version__, "pillow": PIL.__version__,
            "inventory_files_checked": len(inventory), "original_frames_checked": len(images),
            "keypoints_checked": sum(map(len, points.values())), "pairs_checked": len(all_matches),
            "raw_matches_checked": sum(map(len, all_matches.values())), "geometric_inliers_checked": sum(map(len, all_inliers.values())),
            "candidate_tracks_checked": len(tracks), "manual_observations_checked": len(rows),
            "manual_status_counts": dict(Counter(o["status"] for o in measurements["observations"])),
            "height_validation_ready": False, "native_sfm_input_read": native,
            "database_unchanged_sha256": before, "reconstruction_executed": False,
            "not_verified": ["same physical floor C across views", "3D reconstruction / metric length error", "visual correctness of every match"],
            "receipt_scope": "Checks only the environment in which this command was actually executed. A local producer run does not prove recipient-environment acceptance."}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--native", action="store_true", help="Use pycolmap 4.2.1 on a temporary DB copy")
    parser.add_argument("--receiver-name", default="not_provided")
    parser.add_argument("--report", type=Path, help="Write a JSON receipt outside the immutable package")
    args = parser.parse_args()
    if args.report:
        require(not args.report.resolve().is_relative_to(args.root.resolve()), "Save receipts outside the package root")
    try:
        result = check(args.root, args.native, args.receiver_name)
    except Exception as error:
        result = {"status": "failed", "error": f"{type(error).__name__}: {error}", "receiver_name": args.receiver_name}
        if args.report:
            args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False, indent=2))
        raise SystemExit(1)
    if args.report:
        args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
