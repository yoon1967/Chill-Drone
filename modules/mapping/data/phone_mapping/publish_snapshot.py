"""Publish a NEW immutable Git snapshot from a previously delivered package.

Copies only the explicitly approved JSON/CSV inputs; never images, DBs or ZIPs.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil

from shared_inputs import check_file, inside, load_run, read_json, require, sha256, validate


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def publish(package: Path, destination: Path, archive: Path) -> dict:
    package, destination, archive = package.resolve(), destination.resolve(), archive.resolve()
    require(not destination.exists(), "Snapshot exists; publish into a NEW version directory")
    require(not destination.is_relative_to(package), "Keep the delivered package immutable")
    manifest = read_json(package / "handoff_manifest.json")
    # This publisher intentionally supports only the reviewed first-test format.
    require(manifest["package_id"] == "2f1_20261021_handoff_v1", "Unsupported source package")
    for name, record in manifest["files"].items():
        check_file(inside(package, name), record)
    paths = manifest["paths"]
    full = str(Path(paths["database"]).parent).replace("\\", "/")
    measured = str(Path(paths["measurement_anchors"]).parent).replace("\\", "/")
    selection = {
        "keypoints": ("features_v1/keypoints.csv", paths["keypoints_csv"]),
        "matches": ("features_v1/matches.csv", paths["matches_csv"]),
        "frames": ("features_v1/frames_manifest.json", full + "/frames_manifest.json"),
        "feature_metadata": ("features_v1/feature_metadata.json", paths["feature_metadata"]),
        "feature_quality": ("features_v1/quality_report.json", paths["quality_report"]),
        "feature_validation": ("features_v1/validation_report.json", full + "/validation_report.json"),
        "manual_picks": ("measurements_v2_floor/manual_picks.json", measured + "/manual_picks.json"),
        "measurement_anchors": ("measurements_v2_floor/measurement_anchors.json", paths["measurement_anchors"]),
        "measurement_observations": ("measurements_v2_floor/measurement_observations.csv", paths["measurement_observations_csv"]),
        "annotation_quality": ("measurements_v2_floor/annotation_quality_report.json", measured + "/annotation_quality_report.json"),
    }
    archive_record = {"name": archive.name, "bytes": archive.stat().st_size, "sha256": sha256(archive)}
    # Associate the supplied archive with this exact manifest, not merely a filename.
    from zipfile import ZipFile
    with ZipFile(archive) as zipped:
        require(zipped.read(manifest["package_id"] + "/handoff_manifest.json") ==
                (package / "handoff_manifest.json").read_bytes(), "Archive/package manifest differs")
    external = {"schema_version": "phone-mapping-external/1.0", "package_id": manifest["package_id"],
                "input_run_id": manifest["input_run_id"], "archive": archive_record,
                "distribution": {"provider": "team_google_drive", "folder_id": None,
                                 "status": "team_location_not_provided; identify supplied package by SHA256"},
                "package_manifest": {"bytes": (package / "handoff_manifest.json").stat().st_size,
                                     "sha256": sha256(package / "handoff_manifest.json")},
                "artifacts": {}, "frame_bytes": {}}
    for key in ("source_manifest", "source_video", "measurement_reference", "database", "images"):
        name = paths[key]
        external["artifacts"][key] = {"path": name, "kind": "directory" if key == "images" else "file"}
        if key != "images":
            external["artifacts"][key].update(manifest["files"][name])
    for key, name in {"input_directory": str(Path(paths["source_manifest"]).parent),
                      "feature_directory": full, "measurement_directory": measured}.items():
        external["artifacts"][key] = {"path": name.replace("\\", "/"), "kind": "directory"}
    for name, record in manifest["files"].items():
        if name.startswith(paths["images"] + "/"):
            external["frame_bytes"][Path(name).name] = record["bytes"]
    destination.mkdir(parents=True)
    inventory = {}
    for key, (relative, source_name) in selection.items():
        target = inside(destination, relative)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(inside(package, source_name), target)
        inventory[key] = {"path": relative, "package_path": source_name,
                          "bytes": target.stat().st_size, "sha256": sha256(target)}
    write_json(destination / "external_artifacts.json", external)
    inventory["external_artifacts"] = {"path": "external_artifacts.json",
                                       "bytes": (destination / "external_artifacts.json").stat().st_size,
                                       "sha256": sha256(destination / "external_artifacts.json")}
    write_json(destination / "run_manifest.json", {
        "schema_version": "phone-mapping-snapshot/1.0", "input_run_id": manifest["input_run_id"],
        "feature_run_id": manifest["feature_run_id"], "annotation_run_id": manifest["annotation_run_id"],
        "source_data_baseline_commit": manifest["data_baseline_commit"], "files": inventory,
        "producer_paths": "Historical provenance inside copied JSON. Resolve current paths with shared_inputs.load_run and external_artifacts.json; do not join legacy JSON paths to this snapshot directory.",
        "compatibility": "Legacy phone run IDs/CSV fields, not the PoC 0.1.0 interchange schema. No implicit UUID/frame/PTS/pixel conversion.",
        "reconstruction_status": "not_executed", "height_correspondence_ready": False})
    return validate(load_run(destination, package))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-root", type=Path, required=True)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--destination", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(publish(args.package_root, args.destination, args.archive), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
