"""Export reviewed manual anchors, scientific review overlays and consistency QA.

Canonical manual inputs: measurements/manual_picks.json. All feature DB access
is SQLite read-only; original images, measurements reference and DB stay intact.
This script does not insert manual anchors into the automatic feature database.
"""

from __future__ import annotations

from collections import Counter
from contextlib import closing
import csv
import hashlib
import json
from pathlib import Path
import sqlite3

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "output/phone_mapping/2f1_20261021"
FEATURES = ROOT / "output/phone_mapping/2f1_20261021_features_v1/full"
OUTPUT = FEATURES.parent / "measurements"
COLORS = {"A": (255, 195, 0), "B": (255, 65, 65), "C": (0, 200, 255)}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, data: object) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def label(draw: ImageDraw.ImageDraw, xy: tuple[float, float], text: str,
          color: tuple[int, int, int], size: int = 24) -> None:
    font = ImageFont.load_default(size=size)
    bbox = draw.textbbox(xy, text, font=font)
    draw.rectangle((bbox[0] - 4, bbox[1] - 3, bbox[2] + 4, bbox[3] + 3), fill=(25, 25, 25))
    draw.text(xy, text, fill=color, font=font)


def make_overlay(frame: dict, pts: float) -> Path:
    with Image.open(INPUT / "frames_first_test" / frame["frame_id"]) as image:
        result = image.convert("RGB")
    draw = ImageDraw.Draw(result)
    a, b, c = (tuple(frame["points_opencv_px"][i]) for i in "ABC")
    provisional_c = frame.get("point_status", {}).get("C") == "provisional_structure_alignment"
    draw.line((a, b), fill=COLORS["B"], width=3)
    if provisional_c:
        length = float(np.linalg.norm(np.asarray(c) - np.asarray(a)))
        for start in np.arange(0, length, 20):
            p = np.asarray(a) + (np.asarray(c) - a) * (start / length)
            q = np.asarray(a) + (np.asarray(c) - a) * (min(start + 11, length) / length)
            draw.line((tuple(p), tuple(q)), fill=COLORS["C"], width=3)
    else:
        draw.line((a, c), fill=COLORS["C"], width=3)
    label(draw, ((a[0] + b[0]) / 2 - 28, min(a[1], b[1]) - 40), "44 cm", COLORS["B"])
    label(draw, (max(8, (a[0] + c[0]) / 2 - 155), (a[1] + c[1]) / 2), "136 cm target", COLORS["C"], size=20)
    for anchor, (x, y) in frame["points_opencv_px"].items():
        draw.ellipse((x - 8, y - 8, x + 8, y + 8), outline=COLORS[anchor], width=3)
        draw.line((x - 12, y, x + 12, y), fill=COLORS[anchor], width=2)
        draw.line((x, y - 12, x, y + 12), fill=COLORS[anchor], width=2)
        dx, dy = (-30, -34) if anchor == "A" else (17, 0)
        label(draw, (x + dx, y + dy), "C?" if anchor == "C" and provisional_c else anchor, COLORS[anchor])
    draw.rectangle((0, 0, result.width, 44), fill=(25, 25, 25))
    draw.text((12, 10), f'{frame["frame_id"]} | PTS {pts:.1f}s | C floor {"PROVISIONAL" if provisional_c else "user-marked reference"} | original 1920x1080',
              fill="white", font=ImageFont.load_default(size=22))
    path = OUTPUT / "review" / f'{Path(frame["frame_id"]).stem}_anchors.jpg'
    result.save(path, quality=94)
    return path


def make_endpoint_crops(frame: dict) -> None:
    result = Image.new("RGB", (960, 382), (25, 25, 25))
    draw = ImageDraw.Draw(result)
    font = ImageFont.load_default(size=18)
    with Image.open(INPUT / "frames_first_test" / frame["frame_id"]) as source:
        for i, anchor in enumerate("ABC"):
            x, y = frame["points_opencv_px"][anchor]
            crop = source.crop((x - 40, y - 40, x + 40, y + 40)).resize((320, 320), Image.Resampling.NEAREST)
            # Pixel-center mapping under the 4x enlargement.
            center = 40 * 4 + 1.5
            cd = ImageDraw.Draw(crop)
            cd.ellipse((center - 8, center - 8, center + 8, center + 8), outline=COLORS[anchor], width=2)
            cd.line((center - 16, center, center + 16, center), fill=COLORS[anchor], width=1)
            cd.line((center, center - 16, center, center + 16), fill=COLORS[anchor], width=1)
            result.paste(crop, (i * 320, 62))
            point_status = frame.get("point_status", {}).get(anchor, "visually_reviewed")
            radius = frame.get("point_radius_px", {}).get(anchor, frame["estimated_annotation_radius_px"])
            tag = " C? estimate" if point_status == "provisional_structure_alignment" else f" {anchor}"
            draw.text((i * 320 + 8, 5), f'{frame["frame_id"]}{tag}', fill=COLORS[anchor], font=font)
            draw.text((i * 320 + 8, 29), f'({x + 0.5:.1f}, {y + 0.5:.1f}) px | image radius ~{radius}', fill="white", font=font)
    result.save(OUTPUT / "review" / f'{Path(frame["frame_id"]).stem}_endpoint_crops.png')


def main() -> None:
    picks_path = OUTPUT / "manual_picks.json"
    picks = json.loads(picks_path.read_text(encoding="utf-8"))
    source = json.loads((INPUT / "manifest.json").read_text(encoding="utf-8"))
    feature_metadata = json.loads((FEATURES / "feature_metadata.json").read_text(encoding="utf-8"))
    frame_records = {f["frameId"]: f for f in source["frames"]}
    if picks["input_run_id"] != source["runId"] or picks["feature_run_id"] != feature_metadata["feature_run_id"]:
        raise ValueError("Input / feature run_id mismatch")
    if {a["anchor_id"] for a in picks["anchors"]} != set("ABC"):
        raise ValueError("Expected shared endpoint A and endpoints B/C")
    if len(picks["frames"]) < 3 or len({f["frame_id"] for f in picks["frames"]}) != len(picks["frames"]):
        raise ValueError("Need at least three distinct candidate views")
    ref_path = INPUT / picks["reference_image"]
    if sha256(ref_path) != source["measurementReference"]["imageSha256"]:
        raise ValueError("Measurement reference changed")
    if source["measurementReference"]["sourceFrameId"] != picks["reference_frame_id"]:
        raise ValueError("Reference frame differs")
    if picks["reference_frame_id"] not in {f["frame_id"] for f in picks["frames"]}:
        raise ValueError("Reference frame observation missing")
    for segment in picks["segments"]:
        original = next(m for m in source["measurements"] if m["meters"] == segment["length_m"])
        if set(segment["anchor_ids"]) - set("ABC") or segment["user_value"] != original["value"] or segment["user_unit"] != original["unit"]:
            raise ValueError("Measured length/source differs")

    db_path = FEATURES / "database.db"
    before_hash = sha256(db_path)
    observations, frames, epipolar = [], [], []
    (OUTPUT / "review").mkdir(exist_ok=True)
    overlays = []
    with closing(sqlite3.connect(db_path.resolve().as_uri() + "?mode=ro", uri=True)) as db:
        images = {name: (image_id, camera_id) for image_id, name, camera_id in db.execute("SELECT image_id,name,camera_id FROM images")}
        for frame in picks["frames"]:
            name = frame["frame_id"]
            record = frame_records[name]
            image_path = INPUT / "frames_first_test" / name
            if not record["selectedForFirstTest"] or sha256(image_path) != record["sha256"]:
                raise ValueError("Observation image not selected or changed")
            with Image.open(image_path) as image:
                if image.size != (1920, 1080):
                    raise ValueError("Observation image resized")
            if set(frame["points_opencv_px"]) != set("ABC") or frame["estimated_annotation_radius_px"] <= 0:
                raise ValueError("Missing endpoint or annotation uncertainty")
            for anchor, point in frame["points_opencv_px"].items():
                x, y = point
                if not isinstance(x, int) or not isinstance(y, int) or not (0 <= x < 1920 and 0 <= y < 1080):
                    raise ValueError("Invalid manual integer pixel coordinate")
                status = frame.get("point_status", {}).get(anchor, "visually_reviewed")
                radius = frame.get("point_radius_px", {}).get(anchor, frame["estimated_annotation_radius_px"])
                if status not in ("visually_reviewed", "user_marked_reference", "provisional_structure_alignment") or radius <= 0:
                    raise ValueError("Unknown observation status or invalid radius")
                metric_eligible = next(a for a in picks["anchors"] if a["anchor_id"] == anchor).get("use_for_metric_validation", True)
                if status == "provisional_structure_alignment" and metric_eligible:
                    raise ValueError("Provisional correspondence cannot be metric eligible")
                observations.append({"anchor_id": anchor, "frame_id": name, "image_id": images[name][0],
                                     "source_pts_s": record["sourcePtsSeconds"], "x_px": x + 0.5, "y_px": y + 0.5,
                                     "visibility": "floor_edge_visible_identity_unconfirmed" if status == "provisional_structure_alignment" else "visible",
                                     "estimated_annotation_radius_px": radius, "status": status,
                                     "method": "seam_alignment_floor_edge_estimate" if status == "provisional_structure_alignment" else "manual_pick_on_user_reference" if status == "user_marked_reference" else "manual_visual_pick_on_original",
                                     "metric_validation_eligible": metric_eligible, "automatic_keypoint_id": None,
                                     "annotation_note": frame["note"]})
            frames.append({"frame_id": name, "image_id": images[name][0], "camera_id": images[name][1],
                           "source_pts_s": record["sourcePtsSeconds"], "sha256": record["sha256"], "width": 1920, "height": 1080})
            overlays.append(make_overlay(frame, record["sourcePtsSeconds"]))
            make_endpoint_crops(frame)

        # Advisory consistency diagnostic using the existing automatically estimated F.
        # This is not camera-pose estimation or validation of physical length / depth.
        for i, left in enumerate(picks["frames"]):
            for right in picks["frames"][i + 1:]:
                a, b = left["frame_id"], right["frame_id"]
                ia, ib = images[a][0], images[b][0]
                pair_id = min(ia, ib) * 2147483647 + max(ia, ib)
                row = db.execute("SELECT F,config FROM two_view_geometries WHERE pair_id=?", (pair_id,)).fetchone()
                if row is None or row[0] is None:
                    continue
                fundamental = np.frombuffer(row[0], dtype=np.float64).reshape(3, 3)
                if ia > ib:
                    fundamental = fundamental.T
                if not np.isfinite(fundamental).all() or np.linalg.norm(fundamental) < 1e-12:
                    continue
                for anchor in "ABC":
                    p = np.r_[np.asarray(left["points_opencv_px"][anchor]) + 0.5, 1.0]
                    q = np.r_[np.asarray(right["points_opencv_px"][anchor]) + 0.5, 1.0]
                    fp, fq = fundamental @ p, fundamental.T @ q
                    denominator = float(fp[:2] @ fp[:2] + fq[:2] @ fq[:2])
                    if denominator <= 1e-20:
                        continue
                    residual = float(abs(q @ fp) / np.sqrt(denominator))
                    epipolar.append({"frame_id_a": a, "frame_id_b": b, "anchor_id": anchor,
                                     "includes_provisional_observation": any(f.get("point_status", {}).get(anchor) == "provisional_structure_alignment" for f in (left, right)),
                                     "sqrt_sampson_error_px": residual, "review_flag_over_3px": residual > 3.0})
    if before_hash != sha256(db_path):
        raise ValueError("Feature database changed during annotation")

    exported = {key: picks[key] for key in ("schema_version", "annotation_run_id", "input_run_id", "feature_run_id", "anchors", "segments", "annotation_method", "annotation_author", "review_status", "uncertainty_definition", "limitations", "inspection_candidates_not_used")}
    exported.update({key: picks[key] for key in ("supersedes", "correction") if key in picks})
    exported.update({"coordinate_convention": "COLMAP native original 1920x1080 pixels; x right, y down; first pixel center (0.5,0.5); manual integer pixel-center indices shifted +0.5",
                     "measurement_reference": {"file": "../../2f1_20261021/measurement_reference.png", "sha256": sha256(ref_path),
                                               "source_frame_id": picks["reference_frame_id"], "source_pts_s": frame_records[picks["reference_frame_id"]]["sourcePtsSeconds"]},
                     "image_root_relative_to_measurements": "../../2f1_20261021/frames_first_test",
                     "feature_database": {"path": "../full/database.db", "sha256": before_hash, "modified": False},
                     "manual_picks_sha256": sha256(picks_path), "input_manifest_sha256": sha256(INPUT / "manifest.json"),
                     "frames": frames, "observations": observations,
                     "reconstruction_status": "not_performed; C floor cross-view identity pending; height validation not ready",
                     "integration_note": "Read manual anchors separately from SIFT. Filter metric_validation_eligible: C is disabled until cross-view physical identity is confirmed. Provisional floor estimates and small epipolar residuals do not prove the same 3D point. After camera recovery, triangulate confirmed observations with the recovered distortion model."})
    write_json(OUTPUT / "measurement_anchors.json", exported)
    with (OUTPUT / "measurement_observations.csv").open("w", newline="", encoding="utf-8") as file:
        columns = ["anchor_id", "frame_id", "image_id", "source_pts_s", "x_px", "y_px", "visibility", "status", "metric_validation_eligible", "estimated_annotation_radius_px", "method", "automatic_keypoint_id"]
        writer = csv.DictWriter(file, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(observations)
    # Validate the written JSON/CSV, not only the in-memory data.
    written = json.loads((OUTPUT / "measurement_anchors.json").read_text(encoding="utf-8"))
    with (OUTPUT / "measurement_observations.csv").open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    if len(rows) != len(written["observations"]):
        raise ValueError("CSV/JSON observation count differs")
    for row, obs in zip(rows, written["observations"]):
        if (row["anchor_id"], row["frame_id"], float(row["x_px"]), float(row["y_px"]), int(row["image_id"]), float(row["source_pts_s"])) != (obs["anchor_id"], obs["frame_id"], obs["x_px"], obs["y_px"], obs["image_id"], obs["source_pts_s"]):
            raise ValueError("CSV/JSON observation mismatch")
        if row["status"] != obs["status"] or row["metric_validation_eligible"] != str(obs["metric_validation_eligible"]):
            raise ValueError("CSV/JSON eligibility differs")
    overview = Image.new("RGB", (1920, 1620), (30, 30, 30))
    for i, path in enumerate(overlays):
        with Image.open(path) as im:
            overview.paste(im.resize((960, 540), Image.Resampling.LANCZOS), ((i % 2) * 960, (i // 2) * 540))
    od = ImageDraw.Draw(overview)
    od.text((1000, 1130), "A-B = 0.44 m (scale)\nC corrected to floor contact\nC: 1 user reference + 4 estimates\nDashed line / C? = unconfirmed\n136 cm validation: NOT READY\n3D reconstruction: pending", fill="white", font=ImageFont.load_default(size=28), spacing=18)
    overview.save(OUTPUT / "review/overview.jpg", quality=92)
    compare = Image.new("RGB", (960, 884), (30, 30, 30))
    with Image.open(ref_path) as im:
        compare.paste(im.crop((540, 170, 1020, 1010)), (0, 44))
    with Image.open(OUTPUT / "review/frame_0035_anchors.jpg") as im:
        compare.paste(im.crop((540, 170, 1020, 1010)), (480, 44))
    cd = ImageDraw.Draw(compare)
    cd.text((12, 10), "User measurement marks", fill="white", font=ImageFont.load_default(size=22))
    cd.text((492, 10), "Corrected floor C reference", fill="white", font=ImageFont.load_default(size=22))
    compare.save(OUTPUT / "review/reference_comparison.jpg", quality=95)
    report = {"status": "local_file_checks_passed_floor_identity_pending", "frames": len(frames), "anchors": len(picks["anchors"]),
              "observations": len(observations), "observations_per_anchor": dict(Counter(o["anchor_id"] for o in observations)),
              "observation_status_counts": dict(Counter(o["status"] for o in observations)),
              "nonprovisional_observations_per_anchor": dict(Counter(o["anchor_id"] for o in observations if o["status"] != "provisional_structure_alignment")),
              "height_correspondence_ready": next(s for s in picks["segments"] if s["segment_id"] == "height_136cm")["correspondence_ready"],
              "checks": ["original frame / measurement reference SHA-256", "run_id / feature_run_id", "DB image IDs / PTS / original dimensions",
                         "in-frame coordinates and declared per-point status/uncertainty", "provisional floor observations disabled for metric validation",
                         "segment lengths / units against source manifest", "JSON / CSV consistency", "feature DB unchanged SHA-256"],
              "epipolar_diagnostic": {"definition": "sqrt Sampson error against existing two-view F, original pixels; advisory only",
                                      "tested_anchor_pair_observations": len(epipolar),
                                      "max_sqrt_sampson_error_px": max((e["sqrt_sampson_error_px"] for e in epipolar), default=None),
                                      "flags_over_3px": sum(e["review_flag_over_3px"] for e in epipolar), "results": epipolar},
              "not_validated": ["physical measurement uncertainty", "exact physical identity by survey", "3D camera / anchor reconstruction", "metric length error", "recipient environment"],
              "manual_review_scope": "A/B unchanged. C reference follows corrected user mark; four other C locations are estimates on a smooth floor-contact edge. Cross-view C identity is not established and 136cm validation is pending."}
    write_json(OUTPUT / "annotation_quality_report.json", report)
    print(json.dumps({k: v for k, v in report.items() if k not in ("epipolar_diagnostic",)}, ensure_ascii=False, indent=2))
    print(f'Epipolar diagnostic: {len(epipolar)} checks, max {report["epipolar_diagnostic"]["max_sqrt_sampson_error_px"]:.3f}px')


if __name__ == "__main__":
    main()
