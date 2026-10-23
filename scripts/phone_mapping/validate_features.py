"""Validate the full CSV exports against the original images and native DB."""

from __future__ import annotations

from collections import Counter, defaultdict
import argparse
from contextlib import closing
import csv
import json
from pathlib import Path
import sqlite3

import numpy as np
import pycolmap
from path_options import writable_output
from feature_input import digest, load_feature_input, native_database_copy


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "output/phone_mapping/2f1_20261021"
OUTPUT = ROOT / "output/phone_mapping/2f1_20261021_features_v1/full"


def check(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def validate_features(input_dir=INPUT, features_dir=OUTPUT, report_path=None):
    source = load_feature_input(input_dir)
    OUTPUT = Path(features_dir).resolve()
    validation_path = OUTPUT / "validation_report.json"
    if report_path:
        validation_path = writable_output(Path(report_path))
    else:
        writable_output(OUTPUT)
    validation_path.parent.mkdir(parents=True, exist_ok=True)
    validation_path.unlink(missing_ok=True)
    report = json.loads((OUTPUT / "quality_report.json").read_text(encoding="utf-8"))
    selected = {f["frameId"]: f for f in source["selected"]}
    metadata = json.loads((OUTPUT / "feature_metadata.json").read_text(encoding="utf-8"))
    frames_export = json.loads((OUTPUT / "frames_manifest.json").read_text(encoding="utf-8"))
    check(metadata["pycolmap_version"] == pycolmap.__version__ == "4.2.1", "Producer / reader version differs")
    check(metadata["input_run_id"] == frames_export["input_run_id"] == source["run_id"], "Input run IDs differ")
    if source["schema"] == "phone-video-input/1.0":
        check(metadata["input_manifest_sha256"] == source["manifest_sha256"], "Input manifest hash differs")
    check(frames_export["frames"] == report["frames"], "Frame exports disagree")
    check(set(selected) == {f["frame_id"] for f in report["frames"]}, "Selected/report frame mismatch")
    times = [f["source_pts_s"] for f in report["frames"]]
    check(all(a < b for a, b in zip(times, times[1:])), "PTS not strictly ordered")
    for name, frame in selected.items():
        actual = digest(source["image_dir"] / name)
        check(actual == frame["sha256"], f"Source changed: {name}")

    db_path = OUTPUT / "database.db"
    database_hash = digest(db_path)
    if "database_sha256" in metadata:
        check(database_hash == metadata["database_sha256"], "Producer database checksum differs")
    check(not Path(str(db_path) + "-wal").exists(), "Database still depends on WAL")
    with closing(sqlite3.connect(db_path.as_uri() + "?mode=ro&immutable=1", uri=True)) as connection:
        check(connection.execute("PRAGMA integrity_check").fetchone()[0] == "ok", "SQLite integrity failure")
        check(not connection.execute("PRAGMA foreign_key_check").fetchall(), "Foreign key failure")
    raw_counts, verified_counts = Counter(), Counter()
    inlier_edges = set()
    row_count = 0
    with native_database_copy(db_path, validation_path.parent) as copy, pycolmap.Database.open(copy) as db:
        images = {image.name: image for image in db.read_all_images()}
        check(set(images) == set(selected), "DB/source frame names differ")
        points = {name: db.read_keypoints(image.image_id) for name, image in images.items()}
        cameras = {camera.camera_id: camera for camera in db.read_all_cameras()}
        frame_reports = {f["frame_id"]: f for f in report["frames"]}
        for name, image in images.items():
            frame = selected[name]
            camera = cameras[image.camera_id]
            check((camera.width, camera.height) == (frame["width"], frame["height"]), "Camera/processed image dimensions differ")
            check(frame_reports[name]["keypoints"] == len(points[name]), "Per-image feature count differs")
            check((frame_reports[name]["width"], frame_reports[name]["height"]) == (frame["width"], frame["height"]), "Reported frame dimensions differ")
            descriptions = db.read_descriptors(image.image_id).data
            check(descriptions.shape == (len(points[name]), 128), f"Descriptor count/dimension mismatch: {name}")
            check(descriptions.dtype == np.uint8, "SIFT descriptor storage type mismatch")
        keys = set()
        with (OUTPUT / "keypoints.csv").open(newline="", encoding="utf-8") as file:
            for row in csv.DictReader(file):
                name, index = row["frame_id"], int(row["keypoint_id"])
                check(name in images and (name, index) not in keys, "Unknown/duplicate CSV keypoint")
                keys.add((name, index))
                check(0 <= index < len(points[name]), "CSV feature index outside DB")
                x, y = float(row["x_px"]), float(row["y_px"])
                check(np.isfinite([x, y]).all() and 0 <= x < selected[name]["width"] and 0 <= y < selected[name]["height"], "Invalid CSV coordinates")
                check(np.allclose([x, y], points[name][index, :2], atol=1e-6, rtol=0), "CSV/DB coordinates differ")
                check(int(row["image_id"]) == images[name].image_id, "CSV/DB image IDs differ")
                check(float(row["source_pts_s"]) == selected[name]["sourcePtsSeconds"], "CSV/source PTS differs")
                check(row["run_id"] == source["run_id"], "Input run_id differs")
        check(len(keys) == sum(map(len, points.values())) == report["summary"]["keypoints"], "Keypoint totals differ")
        raw_csv, verified_csv = defaultdict(set), defaultdict(set)
        with (OUTPUT / "matches.csv").open(newline="", encoding="utf-8") as file:
            for row in csv.DictReader(file):
                pair = row["frame_id_a"], row["frame_id_b"]
                ka, kb = int(row["keypoint_id_a"]), int(row["keypoint_id_b"])
                check((pair[0], ka) in keys and (pair[1], kb) in keys, "Match refers to nonexistent point")
                check((ka, kb) not in raw_csv[pair], "Duplicate CSV match")
                check(row["geometric_inlier"] in ("0", "1"), "Invalid inlier flag")
                raw_csv[pair].add((ka, kb))
                raw_counts[pair] += 1
                row_count += 1
                if row["geometric_inlier"] == "1":
                    verified_csv[pair].add((ka, kb))
                    verified_counts[pair] += 1
                    inlier_edges.add(((pair[0], ka), (pair[1], kb)))
        for pair in report["pairs"]:
            names = pair["frame_id_a"], pair["frame_id_b"]
            a, b = (images[n].image_id for n in names)
            raw = db.read_matches(a, b) if db.exists_matches(a, b) else np.empty((0, 2), dtype=np.uint32)
            geom = db.read_two_view_geometry(a, b) if db.exists_two_view_geometry(a, b) else None
            verified = geom.inlier_matches if geom else []
            check(raw_csv[names] == {tuple(map(int, m)) for m in raw}, "CSV/DB raw matches differ")
            check(verified_csv[names] == {tuple(map(int, m)) for m in verified}, "CSV/DB verified matches differ")
            check(raw_counts[names] == pair["matches"] and verified_counts[names] == pair["geometric_inliers"], "Pair report totals differ")
            if "preview" in pair:
                check((OUTPUT / pair["preview"]).is_file(), "Missing adjacent preview")
    check(row_count == report["summary"]["raw_matches"], "Global match total differs")
    check(sum(verified_counts.values()) == report["summary"]["geometric_inlier_correspondences"], "Inlier total differs")
    expected_pairs = {tuple(line.split()) for line in (OUTPUT / "pairs.txt").read_text().splitlines()}
    actual_pairs = {(p["frame_id_a"], p["frame_id_b"]) for p in report["pairs"]}
    names = source["names"]
    window = metadata["settings"]["pair_window_selected_frames"]
    check(type(window) is int and window > 0, "Invalid pairing window")
    planned_pairs = {(names[i], names[j]) for i in range(len(names))
                     for j in range(i + 1, min(i + window + 1, len(names)))}
    check(expected_pairs == actual_pairs == planned_pairs and len(report["pairs"]) == len(planned_pairs), "Attempted pairs not all reported")
    check(set(raw_csv).issubset(planned_pairs), "CSV has an unplanned match pair")

    track_rows = defaultdict(list)
    used_observations = set()
    with (OUTPUT / "candidate_tracks.csv").open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            node = row["frame_id"], int(row["keypoint_id"])
            check(node in keys and node not in used_observations, "Unknown/reused track observation")
            used_observations.add(node)
            check(np.allclose([float(row["x_px"]), float(row["y_px"])], points[node[0]][node[1], :2], atol=1e-6, rtol=0), "Track coordinate differs")
            check(float(row["source_pts_s"]) == selected[node[0]]["sourcePtsSeconds"], "Track PTS differs")
            check(int(row["image_id"]) == images[node[0]].image_id, "Track image ID differs")
            track_rows[int(row["candidate_track_id"])].append((node, int(row["observation_count"])))
    neighbors = defaultdict(set)
    for a, b in inlier_edges:
        neighbors[a].add(b)
        neighbors[b].add(a)
    for rows in track_rows.values():
        nodes = {node for node, _ in rows}
        check(len(nodes) >= 2 and len({node[0] for node in nodes}) == len(nodes), "Ambiguous candidate track")
        check(all(count == len(nodes) for _, count in rows), "Track observation count differs")
        seen, stack = {next(iter(nodes))}, [next(iter(nodes))]
        while stack:
            for other in neighbors[stack.pop()] & nodes:
                if other not in seen:
                    seen.add(other)
                    stack.append(other)
        check(seen == nodes, "Candidate track not connected through verified pairs")
    check(len(track_rows) == report["summary"]["candidate_tracks"], "Track totals differ")
    check(len(used_observations) == report["summary"]["candidate_track_observations"], "Track observation totals differ")
    check(digest(db_path) == database_hash, "Producer database changed during validation")
    check(not Path(str(db_path) + "-wal").exists(), "Database still depends on WAL")
    validation = {"status": "passed", "producer_reader": pycolmap.__version__, "frames_checked": len(selected),
                  "keypoints_checked": len(keys), "matches_checked": row_count,
                  "geometric_inliers_checked": sum(verified_counts.values()), "candidate_tracks_checked": len(track_rows),
                  "attempted_pairs_checked": len(planned_pairs), "database_sha256": database_hash,
                  "checks": ["source SHA256 / manifest / PTS", "SQLite integrity and native PyCOLMAP read on a temporary copy", "standalone producer DB hash preserved", "descriptor dimensions/type",
                             "every CSV coordinate / feature ID against DB", "every CSV match / inlier against DB",
                             "dynamic attempted pairs and adjacent previews", "track uniqueness / coordinates / connectivity"],
                  "not_validated": ["visual correctness of every correspondence", "3D reconstruction", "recipient environment"]}
    validation_path.write_text(json.dumps(validation, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(validation, ensure_ascii=False, indent=2))
    return validation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, default=INPUT)
    parser.add_argument("--features-dir", type=Path, default=OUTPUT)
    parser.add_argument("--report", type=Path, help="Write validation separately; producer DB is read through a copy")
    args = parser.parse_args()
    validate_features(args.input_dir, args.features_dir, args.report)


if __name__ == "__main__":
    main()
