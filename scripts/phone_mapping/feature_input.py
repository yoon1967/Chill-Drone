"""Normalize and verify legacy first-test frames or phone-video-input/1.0."""
from contextlib import contextmanager
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
import shutil
import tempfile

from PIL import Image


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def relative_file(root, name):
    relative = PurePosixPath(name)
    if not name or relative.is_absolute() or ".." in relative.parts or "\\" in name or ":" in name:
        raise ValueError("Invalid input relative path")
    path = root.joinpath(*relative.parts).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError("Input path escapes its root")
    return path


def load_feature_input(input_dir):
    root = Path(input_dir).resolve()
    manifest_path = root / "manifest.json"
    original = json.loads(manifest_path.read_text(encoding="utf-8"))
    schema = original.get("schema_version")
    if schema == "phone-video-input/1.0":
        if original.get("status") != "frames_ready":
            raise ValueError("Video input is not frames_ready")
        run_id = original["run_id"]
        image_dir = root / "frames"
        selected = []
        for frame in original["frames"]:
            if frame.get("selected") is not True or frame["path"] != "frames/" + frame["frame_id"]:
                raise ValueError("Selected frame/path disagrees with the video input contract")
            selected.append({**frame, "frameId": frame["frame_id"], "sourcePtsSeconds": frame["source_pts_s"]})
        if original.get("selected_frames") != len(selected):
            raise ValueError("Video selected frame count differs")
        audit = relative_file(root, original["frame_decisions"])
        if digest(audit) != original["frame_decisions_sha256"]:
            raise ValueError("Frame decisions checksum differs")
        decisions = [json.loads(line) for line in audit.read_text(encoding="utf-8").splitlines()]
        if len(decisions) != original["decoded_frames"]:
            raise ValueError("Decoded frame decision count differs")
        chosen = [row for row in decisions if row["selected"]]
        if len(chosen) != len(selected):
            raise ValueError("Selected frame decision count differs")
        for frame, decision in zip(selected, chosen):
            if any(frame.get(key) != decision.get(key) for key in (
                    "frame_id", "source_pts", "source_time_base", "source_pts_s", "timestamp_s",
                    "timestamp_source", "decoded_frame_index", "reason")):
                raise ValueError("Selected frame and source PTS decision disagree")
        capture = {"camera": {"intrinsics": "not_provided; estimate during reconstruction"}, "video": original["source"]}
        coordinate_space = "processed_image"
    elif schema is None and "runId" in original:
        schema = "legacy-phone-first-test"
        run_id = original["runId"]
        image_dir = root / "frames_first_test"
        selected = sorted((dict(frame, width=original["video"]["width"], height=original["video"]["height"])
                           for frame in original["frames"] if frame["selectedForFirstTest"]),
                          key=lambda frame: frame["sourcePtsSeconds"])
        capture = {"camera": original.get("camera", {}), "video": original["video"]}
        coordinate_space = "original_image"
    else:
        raise ValueError("Unsupported feature input schema")
    if not isinstance(run_id, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,95}", run_id):
        raise ValueError("Invalid input run ID")
    names = [frame["frameId"] for frame in selected]
    if len(names) < 3 or len(set(names)) != len(names):
        raise ValueError("Need at least three distinct selected frames")
    if any(not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*\.jpg", name) for name in names):
        raise ValueError("Frame IDs must be plain ASCII JPG names without spaces")
    times = [frame["sourcePtsSeconds"] for frame in selected]
    if any(not isinstance(t, (int, float)) or not math.isfinite(t) for t in times):
        raise ValueError("Invalid source PTS")
    if any(a >= b for a, b in zip(times, times[1:])):
        raise ValueError("Source PTS must increase strictly")
    if {path.name for path in image_dir.glob("*.jpg")} != set(names):
        raise ValueError("Selected image directory and manifest differ")
    dimensions = set()
    for frame in selected:
        path = relative_file(root, image_dir.name + "/" + frame["frameId"])
        if digest(path) != frame["sha256"] or ("bytes" in frame and path.stat().st_size != frame["bytes"]):
            raise ValueError("Input checksum/size differs: " + frame["frameId"])
        with Image.open(path) as image:
            size = image.size
            image.verify()
        if any(type(value) is not int or value <= 0 for value in (frame["width"], frame["height"])):
            raise ValueError("Invalid image dimensions")
        if size != (frame["width"], frame["height"]):
            raise ValueError("Image dimensions differ from manifest")
        dimensions.add(size)
    if len(dimensions) != 1:
        raise ValueError("One shared camera requires identical processed image dimensions")
    return {"root": root, "manifest": original, "manifest_sha256": digest(manifest_path),
            "schema": schema, "run_id": run_id, "selected": selected, "names": names,
            "image_dir": image_dir, "width": selected[0]["width"], "height": selected[0]["height"],
            "capture_information": capture, "coordinate_space": coordinate_space}


@contextmanager
def native_database_copy(database, temporary_parent):
    """Native DB opening may change journal headers; protect the producer DB."""
    with tempfile.TemporaryDirectory(prefix="native-read-", dir=temporary_parent) as directory:
        copy = Path(directory) / "database.db"
        shutil.copy2(database, copy)
        yield copy
