"""Process one local video through frames, 2D features and arbitrary-scale sparse SfM."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
SCHEMA = "phone-video-pipeline/1.0"
SCRIPTS = {
    "input": ROOT / "modules/mapping/data/video_input.py",
    "features": ROOT / "scripts/phone_mapping/full_features.py",
    "reconstruction": ROOT / "modules/mapping/localization/reconstruct.py",
    "preview": ROOT / "modules/mapping/data/render_preview.py",
}
EXIT_CODES = {"complete": 0, "partial": 3, "failed": 2, "interrupted": 130}


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path, value):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def sha256(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


class PipelineError(RuntimeError):
    pass


def run_command(command, log_path):
    """Capture Python and native stdout/stderr; a cancelled parent must stop its worker."""
    environment = os.environ.copy()
    environment.update(PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8")
    options = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {"start_new_session": True}
    with log_path.open("w", encoding="utf-8") as log:
        log.write(json.dumps({"command": command}, ensure_ascii=False) + "\n")
        log.flush()
        process = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=log,
                                   stderr=subprocess.STDOUT, **options)
        try:
            return process.wait()
        except BaseException:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            raise


def commands(video, output, run_id, settings):
    python = [sys.executable, "-B"]
    return {
        "input": python + [str(SCRIPTS["input"]), "--video", str(video), "--output", str(output / "input"),
                           "--run-id", run_id, "--sample-fps", str(settings["sample_fps"]),
                           "--max-width", str(settings["max_width"]), "--jpeg-quality", str(settings["jpeg_quality"])],
        "features": python + [str(SCRIPTS["features"]), "--input-dir", str(output / "input"),
                              "--output-dir", str(output / "features"), "--threads", str(settings["threads"]),
                              "--pair-window", str(settings["pair_window"]), "--max-features", str(settings["max_features"])],
        "reconstruction": python + [str(SCRIPTS["reconstruction"]), "--input-dir", str(output / "input"),
                                    "--features-dir", str(output / "features"), "--output", str(output / "reconstruction"),
                                    "--threads", str(settings["threads"])],
        "preview": python + [str(SCRIPTS["preview"]), "--result", str(output / "reconstruction"),
                             "--output", str(output / "preview")],
    }


def collect_result(output, run_id):
    """Check the final producer contract and checksum bindings before publishing a run."""
    result = output / "reconstruction"
    scene = read_json(result / "scene.json")
    quality = scene["quality"]
    if scene["input_run_id"] != run_id or scene["scale"]["unit"] != "arbitrary":
        raise PipelineError("Unexpected run ID or metric scale in unmeasured video result")
    if not scene["points"] or not scene["cameras"]:
        raise PipelineError("Reconstruction exported an empty scene")
    if quality["points3D"] != len(scene["points"]) or quality["registered_frames"] != len(scene["cameras"]):
        raise PipelineError("Scene inventory differs from quality report")
    expected = "partial" if quality["unregistered_submitted_frames"] else "complete"
    if quality["result_status"] != expected or read_json(result / "status.json")["status"] != expected:
        raise PipelineError("Reconstruction coverage/status mismatch")
    provenance = read_json(result / "provenance.json")
    paths = {name: result / name for name in
             ("scene.json", "poses.json", "points.ply", "trajectory.csv", "quality.json", "scale.json")}
    for name, path in paths.items():
        if provenance["outputs"][name] != sha256(path):
            raise PipelineError(f"Final export checksum differs: {name}")
    paths.update(input_manifest=output / "input/manifest.json",
                 feature_metadata=output / "features/feature_metadata.json",
                 reconstruction_provenance=result / "provenance.json")
    preview = read_json(output / "preview/manifest.json")
    if (preview["input_run_id"] != run_id or preview["scene_sha256"] != provenance["outputs"]["scene.json"]
            or preview["unit"] != scene["scale"]["unit"] or preview["result_status"] != expected):
        raise PipelineError("Preview is bound to another scene or status")
    for name in ("overview.png", "projections.png", "validation.json"):
        path = output / "preview" / name
        if preview["files"][name]["sha256"] != sha256(path):
            raise PipelineError(f"Preview checksum differs: {name}")
        paths["preview_" + name] = path
    paths["preview_manifest"] = output / "preview/manifest.json"
    artifacts = {name: {"path": path.relative_to(output).as_posix(), "bytes": path.stat().st_size,
                        "sha256": sha256(path)} for name, path in paths.items()}
    summary = {key: quality[key] for key in ("input_frames", "submitted_frames", "registered_frames",
               "registration_ratio", "points3D", "unregistered_submitted_frames", "mean_reprojection_error_px")}
    summary.update(unit=scene["scale"]["unit"], native_models=len(quality["components"]))
    return expected, summary, artifacts


def run_pipeline(video, output=None, *, run_id=None, sample_fps=2, max_width=0, jpeg_quality=95,
                 pair_window=4, threads=4, max_features=4000):
    """Return the final manifest. Runtime failures are recorded; unsafe/existing outputs raise."""
    started = time.perf_counter()
    created = datetime.now(timezone.utc).isoformat()
    video = Path(video).resolve()
    run_id = run_id or "video_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,95}", run_id):
        raise ValueError("Use 1..96 ASCII letters, digits, underscores or hyphens for run_id")
    if not math.isfinite(sample_fps) or sample_fps <= 0:
        raise ValueError("sample_fps must be finite and positive")
    if type(max_width) is not int or max_width < 0 or type(jpeg_quality) is not int or not 1 <= jpeg_quality <= 100:
        raise ValueError("max_width must be a nonnegative integer; jpeg_quality must be 1..100")
    if any(type(value) is not int or value <= 0 for value in (pair_window, threads, max_features)):
        raise ValueError("pair_window, threads and max_features must be positive integers")
    output = Path(output).resolve() if output is not None else ROOT / "output/phone_mapping" / run_id
    if output.is_relative_to(ROOT) and not output.is_relative_to(ROOT / "output"):
        raise ValueError("Generated repository files must be below output/")
    if video.is_relative_to(output):
        raise ValueError("Source video must be outside the new run directory")
    output.mkdir(parents=True, exist_ok=False)
    settings = dict(sample_fps=sample_fps, max_width=max_width, jpeg_quality=jpeg_quality,
                    pair_window=pair_window, threads=threads, max_features=max_features)
    manifest = {"schema_version": SCHEMA, "run_id": run_id, "created_utc": created, "status": "running",
                "stage": "preflight", "settings": settings, "source": {"filename": video.name},
                "paths": {name: name for name in SCRIPTS}, "artifacts": {}, "summary": None,
                "stages": [{"name": name, "status": "not_started", "elapsed_seconds": None,
                            "returncode": None, "log": f"logs/{name}.log"} for name in SCRIPTS],
                "not_performed": ["browser_visual_review", "metric_scaling", "dense_reconstruction"],
                "environment": {"python": platform.python_version(), "executable": sys.executable,
                                "platform": platform.platform()},
                "producer_sha256": {"pipeline": sha256(Path(__file__)),
                                    **{name: sha256(path) for name, path in SCRIPTS.items()},
                                    "result_validator": sha256(Path(__file__).with_name("result_io.py"))}}
    timings = {"clock": "perf_counter", "worker_timings": {}, "preflight_seconds": None, "finalization_seconds": None,
               "scope": "run_pipeline entry through final manifest/status/event saves; final timing.json write and console output excluded",
               "includes": ["input checks and source hashes", "worker startup/imports", "frame extraction",
                            "SIFT", "matching", "DB/CSV validation", "SfM", "export", "saved result reload", "PNG previews", "artifact verification", "worker logs"],
               "excludes": ["parent Python startup and CLI argument parsing", "environment installation",
                            "viewer/user waiting"]}
    current = None

    def event(kind, **fields):
        record = {"utc": datetime.now(timezone.utc).isoformat(), "event": kind,
                  "elapsed_seconds": time.perf_counter() - started, **fields}
        with (output / "execution.log").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")

    def save():
        write_json(output / "manifest.json", manifest)
        write_json(output / "status.json", {key: manifest[key] for key in
                   ("schema_version", "run_id", "status", "stage", "error") if key in manifest})

    stage_started = None
    finalization_started = None
    try:
        (output / "logs").mkdir()
        write_json(output / "local_source.json", {"video": str(video)})
        save()
        event("run_started", run_id=run_id)
        if not video.is_file():
            raise FileNotFoundError("Input must be an existing local video file")
        worker_commands = commands(video, output, run_id, settings)
        timings["preflight_seconds"] = time.perf_counter() - started
        for current in manifest["stages"]:
            name = current["name"]
            stage_started = time.perf_counter()
            current["status"] = "running"
            manifest["stage"] = name
            save()
            event("stage_started", stage=name)
            print(json.dumps({"run_id": run_id, "stage": name, "status": "running"}), flush=True)
            current["returncode"] = run_command(worker_commands[name], output / current["log"])
            child_status_path = output / name / "status.json"
            if child_status_path.is_file():
                current["worker_status"] = read_json(child_status_path)
            if current["returncode"] == 130:
                raise KeyboardInterrupt(f"{name} worker was interrupted")
            if current["returncode"] != 0:
                raise PipelineError(f"{name} exited with {current['returncode']}; inspect {current['log']}")
            accepted = {"input": {"frames_ready"}, "features": {"features_ready"},
                        "reconstruction": {"complete", "partial"}, "preview": {"preview_ready"}}[name]
            if current.get("worker_status", {}).get("status") not in accepted:
                raise PipelineError(f"{name} did not publish an accepted status; inspect {current['log']}")
            timings["worker_timings"][name] = read_json(output / name / "timing.json")
            if name == "input":
                source = read_json(output / "input/manifest.json")
                if source["run_id"] != run_id:
                    raise PipelineError("Frame input belongs to another run")
                manifest["source"] = source["source"]
            current.update(status="complete", elapsed_seconds=time.perf_counter() - stage_started)
            event("stage_finished", stage=name, worker_status=current["worker_status"]["status"])
            save()
            stage_started = None
        current = None
        manifest["stage"] = "finalization"
        save()
        finalization_started = time.perf_counter()
        status, summary, artifacts = collect_result(output, run_id)
        manifest.update(status=status, stage="finished", summary=summary, artifacts=artifacts)
        timings["finalization_seconds"] = time.perf_counter() - finalization_started
    except (Exception, KeyboardInterrupt) as error:
        interrupted = isinstance(error, KeyboardInterrupt)
        manifest["status"] = "interrupted" if interrupted else "failed"
        manifest["error"] = {"type": type(error).__name__, "reason": str(error) or "User interrupted execution",
                             "stage": manifest["stage"]}
        if current is not None and current["status"] == "running" and stage_started is not None:
            current.update(status=manifest["status"], elapsed_seconds=time.perf_counter() - stage_started)
            if "worker_status" in current:
                manifest["error"]["worker_status"] = current["worker_status"]
        elif manifest["stage"] == "preflight":
            timings["preflight_seconds"] = time.perf_counter() - started
        if finalization_started is not None:
            timings["finalization_seconds"] = time.perf_counter() - finalization_started
        with (output / "logs/pipeline_error.log").open("w", encoding="utf-8") as stream:
            traceback.print_exc(file=stream)
        event("run_error", **manifest["error"])
    finally:
        manifest["finished_utc"] = datetime.now(timezone.utc).isoformat()
        manifest["exit_code"] = EXIT_CODES[manifest["status"]]
        save()
        event("run_finished", status=manifest["status"], exit_code=manifest["exit_code"])
        timings["stages"] = [{key: stage[key] for key in ("name", "status", "elapsed_seconds", "returncode")}
                             for stage in manifest["stages"]]
        timings["total_seconds"] = time.perf_counter() - started
        write_json(output / "timing.json", timings)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--output", type=Path, help="New run root; default: output/phone_mapping/<auto run_id>")
    parser.add_argument("--run-id")
    parser.add_argument("--sample-fps", type=float, default=2)
    parser.add_argument("--max-width", type=int, default=0)
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--pair-window", type=int, default=4)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--max-features", type=int, default=4000)
    args = parser.parse_args()
    try:
        result = run_pipeline(**vars(args))
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(json.dumps({"status": "failed", "type": type(error).__name__, "reason": str(error)}, ensure_ascii=False), file=sys.stderr)
        return 2
    output = Path(args.output).resolve() if args.output else ROOT / "output/phone_mapping" / result["run_id"]
    print(json.dumps({"output": str(output), **{key: result[key] for key in
                     ("run_id", "status", "exit_code", "summary", "error") if key in result}}, ensure_ascii=False), flush=True)
    return result["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
