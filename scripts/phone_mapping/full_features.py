"""Create a versioned COLMAP database and readable 2D correspondence exports.

Only feature extraction / matching run here. No 3D reconstruction is performed.
Requires pycolmap==4.2.1, numpy, and Pillow for previews.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sqlite3
import sys

import numpy as np
import pycolmap
import PIL
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "output/phone_mapping/2f1_20261021"
OUTPUT = ROOT / "output/phone_mapping/2f1_20261021_features_v1/full"
WINDOW = 4
GRAPH_MIN_INLIERS = 50  # A reporting threshold, not a reconstruction guarantee.
VALID_CONFIGS = {"CALIBRATED", "UNCALIBRATED", "PLANAR", "PANORAMIC", "PLANAR_OR_PANORAMIC"}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, default=str) + "\n", encoding="utf-8")


def components(names: list[str], edges: list[tuple[str, str]]) -> list[list[str]]:
    neighbors: dict[str, set[str]] = {name: set() for name in names}
    for a, b in edges:
        neighbors[a].add(b)
        neighbors[b].add(a)
    seen: set[str] = set()
    result = []
    for name in names:
        if name in seen:
            continue
        stack, group = [name], []
        seen.add(name)
        while stack:
            current = stack.pop()
            group.append(current)
            for other in sorted(neighbors[current]):
                if other not in seen:
                    seen.add(other)
                    stack.append(other)
        result.append(sorted(group))
    return result


class UnionFind:
    def __init__(self) -> None:
        self.parent: dict[tuple[int, int], tuple[int, int]] = {}
        self.size: dict[tuple[int, int], int] = {}

    def find(self, node: tuple[int, int]) -> tuple[int, int]:
        if node not in self.parent:
            self.parent[node], self.size[node] = node, 1
        root = node
        while self.parent[root] != root:
            root = self.parent[root]
        while self.parent[node] != node:
            previous = node
            node = self.parent[node]
            self.parent[previous] = root
        return root

    def join(self, a: tuple[int, int], b: tuple[int, int]) -> None:
        a, b = self.find(a), self.find(b)
        if a == b:
            return
        if self.size[a] < self.size[b]:
            a, b = b, a
        self.parent[b] = a
        self.size[a] += self.size[b]


def preview(images_dir: Path, name_a: str, name_b: str, xy_a: np.ndarray,
            xy_b: np.ndarray, pairs: np.ndarray, path: Path, title: str) -> None:
    with Image.open(images_dir / name_a) as original:
        a = original.convert("RGB")
    with Image.open(images_dir / name_b) as original:
        b = original.convert("RGB")
    # Sample across the matched feature row order instead of drawing all lines.
    chosen = np.linspace(0, len(pairs) - 1, min(45, len(pairs)), dtype=int) if len(pairs) else []
    canvas = Image.new("RGB", (a.width + b.width, a.height))
    canvas.paste(a, (0, 0))
    canvas.paste(b, (a.width, 0))
    draw = ImageDraw.Draw(canvas)
    for index in chosen:
        i, j = pairs[index]
        # COLMAP pixel centers are OpenCV positions +0.5.
        pa = np.rint(xy_a[i, :2] - 0.5).astype(int)
        pb = np.rint(xy_b[j, :2] - 0.5).astype(int) + (a.width, 0)
        draw.line([tuple(pa), tuple(pb)], fill=(0, 230, 0), width=2)
        for x, y in (pa, pb):
            draw.ellipse((x - 4, y - 4, x + 4, y + 4), outline=(0, 230, 0), width=2)
    canvas = canvas.resize((canvas.width // 2, canvas.height // 2), Image.Resampling.LANCZOS)
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((0, 0, canvas.width, 40), fill=(30, 30, 30))
    draw.text((12, 10), title, fill="white", font=ImageFont.load_default(size=18))
    canvas.save(path, quality=90)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--export-only", action="store_true", help="Read an existing DB; do not rerun extraction/matching")
    args = parser.parse_args()
    if pycolmap.__version__ != "4.2.1":
        raise RuntimeError("This run is pinned to pycolmap 4.2.1")
    manifest = json.loads((INPUT / "manifest.json").read_text(encoding="utf-8"))
    selected = sorted((f for f in manifest["frames"] if f["selectedForFirstTest"]), key=lambda f: f["sourcePtsSeconds"])
    names = [f["frameId"] for f in selected]
    assert len(names) == len(set(names)) == 31
    image_dir = INPUT / "frames_first_test"
    if {p.name for p in image_dir.glob("*.jpg")} != set(names):
        raise ValueError("Selected directory and manifest differ")
    for frame in selected:
        path = image_dir / frame["frameId"]
        with Image.open(path) as image:
            image_size = image.size
        if digest(path) != frame["sha256"] or image_size != (1920, 1080):
            raise ValueError(f"Input checksum or size mismatch: {path}")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / "match_previews").mkdir(exist_ok=True)
    db_path = OUTPUT / "database.db"
    pair_names = [(names[i], names[j]) for i in range(len(names)) for j in range(i + 1, min(i + WINDOW + 1, len(names)))]
    pair_file = OUTPUT / "pairs.txt"
    pair_file.write_text("".join(f"{a} {b}\n" for a, b in pair_names), encoding="utf-8")
    extraction = pycolmap.FeatureExtractionOptions()
    extraction.num_threads = 4
    extraction.max_image_size = 1920
    extraction.sift.max_num_features = 4000
    matching = pycolmap.FeatureMatchingOptions()
    matching.num_threads = 4
    matching.sift.max_ratio = 0.75
    matching.sift.cross_check = True
    matching.sift.cpu_brute_force_matcher = True
    verification = pycolmap.TwoViewGeometryOptions()
    verification.ransac.max_error = 1.5
    verification.ransac.random_seed = 42
    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = "SIMPLE_RADIAL"
    reader.default_focal_length_factor = 1.2
    settings = {"extraction": extraction.todict(), "matching": matching.todict(),
                "verification": verification.todict(), "reader": reader.todict(),
                "camera_mode": "SINGLE", "device": "cpu", "pair_window_selected_frames": WINDOW}
    if not args.export_only:
        if db_path.exists():
            raise FileExistsError("Database already exists; preserve it and use --export-only or a new run directory")
        pycolmap.set_random_seed(42)
        print("EXTRACT: 31 original-resolution frames", flush=True)
        pycolmap.extract_features(database_path=db_path, image_path=image_dir, image_names=names,
                                 camera_mode=pycolmap.CameraMode.SINGLE, reader_options=reader,
                                 extraction_options=extraction, device=pycolmap.Device.cpu)
        print(f"MATCH: {len(pair_names)} pairs", flush=True)
        pairing = pycolmap.ImportedPairingOptions()
        pairing.match_list_path = pair_file
        pycolmap.match_image_pairs(database_path=db_path, matching_options=matching,
                                  pairing_options=pairing, verification_options=verification,
                                  device=pycolmap.Device.cpu)
    elif not db_path.exists():
        raise FileNotFoundError(db_path)

    frame_meta = {f["frameId"]: f for f in selected}
    tracks = UnionFind()
    pair_reports, frame_reports = [], []
    xy_by_id, by_name = {}, {}
    with pycolmap.Database.open(db_path) as db:
        images = db.read_all_images()
        by_name = {image.name: image for image in images}
        if set(by_name) != set(names):
            raise ValueError("Database image names differ from selected input")
        with (OUTPUT / "keypoints.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(["run_id", "image_id", "frame_id", "source_pts_s", "keypoint_id", "x_px", "y_px"])
            for name in names:
                image = by_name[name]
                xy = db.read_keypoints(image.image_id)
                xy_by_id[image.image_id] = xy
                if len(xy) == 0 or not np.isfinite(xy).all() or not ((xy[:, 0] >= 0) & (xy[:, 0] < 1920) & (xy[:, 1] >= 0) & (xy[:, 1] < 1080)).all():
                    raise ValueError(f"Invalid keypoints in {name}")
                for k, point in enumerate(xy):
                    writer.writerow([manifest["runId"], image.image_id, name, frame_meta[name]["sourcePtsSeconds"], k,
                                     f"{point[0]:.8f}", f"{point[1]:.8f}"])
                frame_reports.append({"frame_id": name, "image_id": image.image_id, "source_pts_s": frame_meta[name]["sourcePtsSeconds"],
                                      "sha256": frame_meta[name]["sha256"], "width": 1920, "height": 1080,
                                      "keypoints": len(xy), "status": "extracted"})
        with (OUTPUT / "matches.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(["frame_id_a", "keypoint_id_a", "frame_id_b", "keypoint_id_b", "geometric_inlier"])
            for name_a, name_b in pair_names:
                a, b = by_name[name_a].image_id, by_name[name_b].image_id
                raw = db.read_matches(a, b) if db.exists_matches(a, b) else np.empty((0, 2), dtype=np.uint32)
                geo = db.read_two_view_geometry(a, b) if db.exists_two_view_geometry(a, b) else None
                verified = geo.inlier_matches if geo else np.empty((0, 2), dtype=np.uint32)
                inlier_set = {tuple(map(int, pair)) for pair in verified}
                raw_set = {tuple(map(int, pair)) for pair in raw}
                # Guided matching is off, so each verified match must come from the raw matches.
                if not inlier_set.issubset(raw_set):
                    raise ValueError("Verified matches outside raw set")
                config = pycolmap.TwoViewGeometryConfiguration(geo.config).name if geo else "UNDEFINED"
                for ka, kb in raw:
                    if ka >= len(xy_by_id[a]) or kb >= len(xy_by_id[b]):
                        raise ValueError("Feature index outside database keypoints")
                    writer.writerow([name_a, ka, name_b, kb, int((int(ka), int(kb)) in inlier_set)])
                if config in VALID_CONFIGS:
                    for ka, kb in verified:
                        tracks.join((a, int(ka)), (b, int(kb)))
                info = {"frame_id_a": name_a, "frame_id_b": name_b,
                        "time_gap_s": frame_meta[name_b]["sourcePtsSeconds"] - frame_meta[name_a]["sourcePtsSeconds"],
                        "matches": len(raw), "geometric_inliers": len(verified), "geometry_config": config,
                        "inlier_ratio": len(verified) / len(raw) if len(raw) else 0.0,
                        "graph_edge": config in VALID_CONFIGS and len(verified) >= GRAPH_MIN_INLIERS,
                        "adjacent_selected_frames": names.index(name_b) - names.index(name_a) == 1}
                if info["adjacent_selected_frames"]:
                    preview_name = f"{Path(name_a).stem}_{Path(name_b).stem}.jpg"
                    preview(image_dir, name_a, name_b, xy_by_id[a], xy_by_id[b], verified,
                            OUTPUT / "match_previews" / preview_name,
                            f"{name_a} <> {name_b} | inliers {len(verified)}/{len(raw)} | {config}")
                    info["preview"] = f"match_previews/{preview_name}"
                pair_reports.append(info)
        camera_records = [{"camera_id": c.camera_id, "model": c.model_name, "width": c.width, "height": c.height,
                           "params": c.params.tolist(), "has_prior_focal_length": c.has_prior_focal_length}
                          for c in db.read_all_cameras()]

    id_names = {image.image_id: image.name for image in images}
    groups: dict[tuple[int, int], list[tuple[int, int]]] = defaultdict(list)
    for node in tracks.parent:
        groups[tracks.find(node)].append(node)
    accepted, rejected = [], []
    for group in sorted(groups.values(), key=lambda g: sorted(g)[0]):
        group = sorted(group)
        counts = Counter(image_id for image_id, _ in group)
        if max(counts.values()) > 1:
            rejected.append({"observations": len(group), "frame_count": len(counts),
                             "conflicting_frames": [id_names[i] for i, c in counts.items() if c > 1]})
        else:
            accepted.append(group)
    with (OUTPUT / "candidate_tracks.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["candidate_track_id", "frame_id", "image_id", "keypoint_id", "source_pts_s", "x_px", "y_px", "observation_count"])
        for track_id, group in enumerate(accepted):
            for image_id, point_id in group:
                name = id_names[image_id]
                point = xy_by_id[image_id][point_id]
                writer.writerow([track_id, name, image_id, point_id, frame_meta[name]["sourcePtsSeconds"],
                                 f"{point[0]:.8f}", f"{point[1]:.8f}", len(group)])
    write_json(OUTPUT / "rejected_track_components.json", {"reason": "multiple feature indices from one image in a connected component",
                                                           "components": rejected})
    edges = [(p["frame_id_a"], p["frame_id_b"]) for p in pair_reports if p["graph_edge"]]
    graph_components = components(names, edges)
    adjacent = [p for p in pair_reports if p["adjacent_selected_frames"]]
    metadata = {"feature_run_id": "2f1_20261021_features_v1_full", "input_run_id": manifest["runId"],
                "producer": "pycolmap", "pycolmap_version": pycolmap.__version__, "numpy_version": np.__version__,
                "pillow_preview_version": PIL.__version__, "python_version": sys.version,
                "script_sha256": digest(Path(__file__)), "input_manifest_sha256": digest(INPUT / "manifest.json"),
                "capture_information": {"camera": manifest["camera"], "video": manifest["video"]},
                "generated_utc": datetime.now(timezone.utc).isoformat(), "settings": settings,
                "coordinate_convention": "COLMAP native; original 1920x1080 pixels; x right, y down; first pixel center (0.5,0.5); no manual offset",
                "keypoint_id": "zero-based row index into DB keypoints/descriptors for that image",
                "image_root_relative_to_full": "../../2f1_20261021/frames_first_test",
                "camera_initialization": "One shared SIMPLE_RADIAL; uncalibrated heuristic f=1.2*max(width,height), centered principal point, k=0. Not measured iPhone calibration; refine during reconstruction.",
                "camera_records": camera_records,
                "recipient_tool_version": "not_decided_user_confirmed", "recipient_read_test": "pending_stage_5",
                "pilot_relationship": "New native COLMAP extraction; feature IDs differ from OpenCV pilot. Do not mix pilot coordinates/matches with this database.",
                "candidate_tracks": "Transitive 2D inlier components with at most one feature per image. Conflicting components omitted from this convenience CSV only; DB/matches remain intact. Candidate IDs are not 3D point IDs.",
                "not_performed": ["3D reconstruction", "camera pose estimation", "metric scaling", "measured endpoint annotation", "recipient environment test"]}
    report = {"feature_run_id": metadata["feature_run_id"], "status": "full_2d_extracted_recipient_pending",
              "frames": frame_reports, "pairs": pair_reports,
              "summary": {"frames": len(names), "keypoints": sum(f["keypoints"] for f in frame_reports),
                          "attempted_pairs": len(pair_reports), "raw_matches": sum(p["matches"] for p in pair_reports),
                          "geometric_inlier_correspondences": sum(p["geometric_inliers"] for p in pair_reports),
                          "geometry_config_counts": dict(Counter(p["geometry_config"] for p in pair_reports)),
                          "graph_min_inliers": GRAPH_MIN_INLIERS, "graph_components": graph_components,
                          "adjacent_min_inliers": min(p["geometric_inliers"] for p in adjacent),
                          "weak_adjacent_pairs": [p for p in adjacent if not p["graph_edge"]],
                          "candidate_tracks": len(accepted), "candidate_tracks_3plus_views": sum(len(g) >= 3 for g in accepted),
                          "candidate_track_observations": sum(map(len, accepted)),
                          "rejected_conflicting_components": len(rejected),
                          "rejected_observations": sum(r["observations"] for r in rejected)},
              "limitations": ["Graph connectivity and many inliers do not establish 3D reconstruction success.",
                              "Locker repetition, glass reflections and nearly planar scene may retain incorrect correspondences.",
                              "Planar/panoramic pair classifications and camera initialization require attention during reconstruction.",
                              "Recipient version/read compatibility has not been validated."]}
    with sqlite3.connect(db_path) as conn:
        integrity = conn.execute("PRAGMA integrity_check").fetchone()[0]
        if integrity != "ok":
            raise ValueError(f"SQLite integrity check failed: {integrity}")
        conn.execute("PRAGMA wal_checkpoint(TRUNCATE)")
    write_json(OUTPUT / "feature_metadata.json", metadata)
    write_json(OUTPUT / "frames_manifest.json", {"input_run_id": manifest["runId"], "frames": frame_reports})
    write_json(OUTPUT / "quality_report.json", report)
    print(json.dumps(report["summary"], ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
