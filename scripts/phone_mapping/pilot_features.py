"""Extract and inspect 2D SIFT correspondences from three selected video frames.

This is a small input pilot. It does not estimate 3D points or produce a COLMAP DB.
OpenCV coordinates are shifted by +0.5 in exported data to match the convention
documented for COLMAP: the center of the first pixel is (0.5, 0.5).
"""

from __future__ import annotations

import csv
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from path_options import writable_output


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "output" / "phone_mapping" / "2f1_20261021"
OUTPUT = ROOT / "output" / "phone_mapping" / "2f1_20261021_features_v1" / "pilot"
FRAME_IDS = ("frame_0034.jpg", "frame_0035.jpg", "frame_0036.jpg")
PAIRS = ((0, 1), (1, 2), (0, 2))
SIFT_MAX_FEATURES = 4000
LOWE_RATIO = 0.75
RANSAC_THRESHOLD_PX = 1.5


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ratio_matches(matcher: cv2.BFMatcher, left: np.ndarray, right: np.ndarray) -> list[cv2.DMatch]:
    result = []
    for near in matcher.knnMatch(left, right, k=2):
        if len(near) == 2 and near[0].distance < LOWE_RATIO * near[1].distance:
            result.append(near[0])
    return result


def main() -> None:
    global INPUT, OUTPUT
    parser = argparse.ArgumentParser(description="Historical OpenCV 3-frame pilot; IDs differ from native COLMAP")
    parser.add_argument("--input-dir", type=Path, default=INPUT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT)
    args = parser.parse_args()
    INPUT, OUTPUT = args.input_dir.resolve(), writable_output(args.output_dir)
    manifest = json.loads((INPUT / "manifest.json").read_text(encoding="utf-8"))
    recorded = {item["frameId"]: item for item in manifest["frames"]}
    OUTPUT.mkdir(parents=True, exist_ok=True)
    preview_dir = OUTPUT / "match_previews"
    preview_dir.mkdir(exist_ok=True)

    sift = cv2.SIFT_create(nfeatures=SIFT_MAX_FEATURES)
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    images: dict[str, np.ndarray] = {}
    keypoints: dict[str, list[cv2.KeyPoint]] = {}
    descriptors: dict[str, np.ndarray] = {}
    frame_report = []

    with (OUTPUT / "keypoints.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("run_id", "frame_id", "source_pts_s", "keypoint_id", "x_px", "y_px", "response", "size_px"))
        for name in FRAME_IDS:
            path = INPUT / "frames_first_test" / name
            expected = recorded[name]
            if not expected["selectedForFirstTest"] or sha256(path) != expected["sha256"]:
                raise ValueError(f"Input frame missing or checksum changed: {name}")
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None or image.shape[:2] != (1080, 1920):
                raise ValueError(f"Unexpected image dimensions: {name}")
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            points, description = sift.detectAndCompute(gray, None)
            if description is None or len(points) == 0:
                raise ValueError(f"No SIFT descriptors: {name}")
            images[name], keypoints[name], descriptors[name] = image, points, description
            for point_id, point in enumerate(points):
                x, y = point.pt
                writer.writerow((manifest["runId"], name, expected["sourcePtsSeconds"], point_id,
                                 f"{x + 0.5:.6f}", f"{y + 0.5:.6f}", f"{point.response:.8f}", f"{point.size:.5f}"))
            frame_report.append({"frame_id": name, "source_pts_s": expected["sourcePtsSeconds"],
                                 "sha256": expected["sha256"], "keypoints": len(points)})

    # These arrays keep descriptors and the original feature row order together.
    np.savez_compressed(
        OUTPUT / "sift_features.npz",
        **{f"descriptors_{name.removesuffix('.jpg')}": descriptors[name] for name in FRAME_IDS},
        **{f"xy_px_{name.removesuffix('.jpg')}":
           np.asarray([point.pt for point in keypoints[name]], dtype=np.float32) + 0.5 for name in FRAME_IDS},
    )

    pair_report = []
    with (OUTPUT / "matches.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("frame_id_a", "keypoint_id_a", "frame_id_b", "keypoint_id_b", "descriptor_distance", "geometric_inlier"))
        for index_a, index_b in PAIRS:
            name_a, name_b = FRAME_IDS[index_a], FRAME_IDS[index_b]
            forward = ratio_matches(matcher, descriptors[name_a], descriptors[name_b])
            backward = ratio_matches(matcher, descriptors[name_b], descriptors[name_a])
            reverse_pairs = {(match.trainIdx, match.queryIdx) for match in backward}
            candidates = sorted(
                (match for match in forward if (match.queryIdx, match.trainIdx) in reverse_pairs),
                key=lambda match: match.distance,
            )
            inlier_mask = np.zeros(len(candidates), dtype=bool)
            fundamental_found = False
            if len(candidates) >= 8:
                pts_a = np.float32([keypoints[name_a][m.queryIdx].pt for m in candidates])
                pts_b = np.float32([keypoints[name_b][m.trainIdx].pt for m in candidates])
                fundamental, mask = cv2.findFundamentalMat(
                    pts_a, pts_b, cv2.USAC_MAGSAC, RANSAC_THRESHOLD_PX, 0.999
                )
                if fundamental is not None and mask is not None:
                    fundamental_found = True
                    inlier_mask = mask.ravel().astype(bool)
            for match, inlier in zip(candidates, inlier_mask):
                writer.writerow((name_a, match.queryIdx, name_b, match.trainIdx,
                                 f"{match.distance:.5f}", int(inlier)))
            inliers = [match for match, keep in zip(candidates, inlier_mask) if keep]
            preview = cv2.drawMatches(
                images[name_a], keypoints[name_a], images[name_b], keypoints[name_b],
                inliers[:60], None, matchColor=(0, 255, 0),
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
            )
            preview = cv2.resize(preview, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
            cv2.putText(preview, f"{name_a} <-> {name_b}: {len(inliers)} verified / {len(candidates)} candidates",
                        (15, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (10, 10, 10), 3, cv2.LINE_AA)
            cv2.putText(preview, f"{name_a} <-> {name_b}: {len(inliers)} verified / {len(candidates)} candidates",
                        (15, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1, cv2.LINE_AA)
            preview_name = f"{name_a.removesuffix('.jpg')}_{name_b.removesuffix('.jpg')}.png"
            if not cv2.imwrite(str(preview_dir / preview_name), preview):
                raise OSError(f"Could not write preview: {preview_name}")
            pair_report.append({"frame_id_a": name_a, "frame_id_b": name_b,
                                "forward_ratio_matches": len(forward),
                                "mutual_ratio_matches": len(candidates),
                                "fundamental_estimated": fundamental_found,
                                "geometric_inliers": len(inliers), "preview": f"match_previews/{preview_name}"})

    report = {
        "feature_run_id": "2f1_20261021_features_v1_pilot",
        "status": "pilot_2d_only",
        "input_run_id": manifest["runId"],
        "extractor": "OpenCV SIFT",
        "opencv_version": cv2.__version__,
        "numpy_version": np.__version__,
        "sift_max_features": SIFT_MAX_FEATURES,
        "matching": "bidirectional Lowe ratio then fundamental matrix USAC_MAGSAC",
        "lowe_ratio": LOWE_RATIO,
        "ransac_threshold_px": RANSAC_THRESHOLD_PX,
        "coordinate_convention": "source 1920x1080; x right, y down; top-left pixel center at (0.5,0.5); OpenCV SIFT points shifted +0.5 in exports",
        "database_status": "not_created; compatibility with recipient COLMAP pending",
        "frames": frame_report,
        "pairs": pair_report,
    }
    (OUTPUT / "quality_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"frames": frame_report, "pairs": pair_report}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
