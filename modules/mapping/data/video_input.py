"""Decode a local video into a new, versioned input for the file mapping pipeline."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import sys
import time

import av
from PIL import Image
import PIL

ROOT = Path(__file__).resolve().parents[3]
SCHEMA = "phone-video-input/1.0"


class VideoInputError(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def write_json(path: Path, data: object) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def presentation_time(pts, time_base, previous: Fraction | None) -> Fraction:
    if pts is None or time_base is None:
        raise VideoInputError("missing_pts", "A decoded frame has no source PTS/time base; nominal FPS is not a timestamp substitute")
    value = pts * Fraction(time_base)
    if previous is not None and value <= previous:
        raise VideoInputError("non_monotonic_pts", "Decoded presentation timestamps must increase strictly")
    return value


def geometry(width: int, height: int, rotation: int, max_width: int):
    """Map coded pixel centers (0.5,0.5) to the saved, oriented image centers."""
    rotation %= 360
    if rotation == 0:
        matrix, display_size = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], (width, height)
    elif rotation == 90:
        matrix, display_size = [[0, 1, 0], [-1, 0, width], [0, 0, 1]], (height, width)
    elif rotation == 180:
        matrix, display_size = [[-1, 0, width], [0, -1, height], [0, 0, 1]], (width, height)
    elif rotation == 270:
        matrix, display_size = [[0, -1, height], [1, 0, 0], [0, 0, 1]], (height, width)
    else:
        raise VideoInputError("unsupported_rotation", "Only 0/90/180/270-degree display rotations are supported")
    dw, dh = display_size
    output_size = (max_width, max(1, round(dh * max_width / dw))) if max_width and dw > max_width else display_size
    scales = (output_size[0] / dw, output_size[1] / dh)
    matrix = [[float(value * scales[row]) if row < 2 else value for value in values]
              for row, values in enumerate(matrix)]
    return display_size, output_size, matrix


def display_rotation(frame, metadata: dict) -> int:
    # FFmpeg may express mirroring or translation in DISPLAYMATRIX as well as rotation.
    # Reject transforms we cannot apply faithfully rather than silently discarding them.
    has_matrix = False
    for kind in frame.side_data:
        if kind.name != "DISPLAYMATRIX":
            continue
        has_matrix = True
        data = bytes(frame.side_data[kind])
        if len(data) != 36:
            raise VideoInputError("unsupported_display_matrix", "Unexpected display matrix layout")
        a, b, u, c, d, v, x, y, w = struct.unpack("=9i", data)
        if (a * d - b * c <= 0 or any((u, v, x, y)) or w != 1 << 30
                or any(abs(value) not in (0, 65536) for value in (a, b, c, d))
                or a * a + c * c != 65536 ** 2 or b * b + d * d != 65536 ** 2
                or a * b + c * d != 0):
            raise VideoInputError("unsupported_display_matrix", "Mirrored, scaled or non-right-angle display matrices need an explicit adapter")
    if not has_matrix and metadata.get("rotate") not in (None, "0"):
        raise VideoInputError("unsupported_rotation_metadata", "Rotation tag is present without a decoded display matrix")
    return int(frame.rotation) % 360


def extract_video(video: Path, output: Path | None = None, *, run_id: str | None = None,
                  sample_fps: float = 2, max_width: int = 0, jpeg_quality: int = 95) -> dict:
    """Read every decoded frame; interval sampling never replaces pending work."""
    started = time.perf_counter()
    video = Path(video).resolve()
    if not video.is_file():
        raise VideoInputError("missing_video", "Input must be an existing local video file")
    if not math.isfinite(sample_fps) or sample_fps <= 0:
        raise VideoInputError("invalid_settings", "sample_fps must be finite and positive")
    if not isinstance(max_width, int) or max_width < 0 or not 1 <= jpeg_quality <= 100:
        raise VideoInputError("invalid_settings", "max_width must be a nonnegative integer; JPEG quality must be 1..100")
    run_id = run_id or "video_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,95}", run_id):
        raise VideoInputError("invalid_run_id", "Use 1..96 ASCII letters, digits, underscores or hyphens for run_id")
    output = Path(output).resolve() if output is not None else ROOT / "output/phone_mapping" / run_id / "input"
    if output.is_relative_to(ROOT) and not output.is_relative_to(ROOT / "output"):
        raise VideoInputError("unsafe_output", "Generated repository files must be below output/")
    if video.is_relative_to(output):
        raise VideoInputError("unsafe_output", "The source video must be outside the new output directory")
    # exist_ok=False is also the atomic guard against two writers sharing a run directory.
    output.mkdir(parents=True, exist_ok=False)
    frames_dir = output / "frames"
    frames_dir.mkdir()
    manifest = {"schema_version": SCHEMA, "run_id": run_id, "status": "extracting",
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "source": {"filename": video.name, "bytes": video.stat().st_size},
                "settings": {"sample_fps": sample_fps, "max_width": max_width, "jpeg_quality": jpeg_quality,
                             "sampling": "first decoded frame at/after each source-PTS grid boundary",
                             "timestamp_policy": "strict_source_pts", "rotation_policy": "apply_display_matrix_rotation"},
                "environment": {"pyav": av.__version__, "pillow": PIL.__version__,
                                "ffmpeg_libraries": {k: list(v) for k, v in av.library_versions.items()}},
                "coordinate_space": "processed_image",
                "pixel_centers": "COLMAP convention: first center (0.5,0.5), x right, y down",
                "metric_reference": None, "frames": [], "decoded_frames": 0,
                "frame_decisions": "frame_decisions.jsonl", "warnings": []}
    phase = "source_inspection"
    initial_hash = None
    extraction_started = None

    def finish(status: str, error: dict | None = None):
        manifest.update(status=status, selected_frames=len(manifest["frames"]))
        if error:
            manifest["error"] = error
        elapsed = time.perf_counter() - started
        write_json(output / "manifest.json", manifest)
        write_json(output / "timing.json", {"stage": "video_input", "elapsed_seconds": elapsed,
            "scope": "function entry through extraction/source verification; final report writes excluded",
            "decode_transform_write_seconds": None if extraction_started is None else extraction_seconds})
        write_json(output / "status.json", {"stage": "video_input", "status": status,
            "run_id": run_id, "decoded_frames": manifest["decoded_frames"],
            "selected_frames": len(manifest["frames"]), "error": error})
        with (output / "execution.log").open("a", encoding="utf-8") as log:
            log.write(json.dumps({"status": status, "phase": phase, "elapsed_seconds": elapsed, "error": error}, ensure_ascii=False) + "\n")

    extraction_seconds = None
    try:
        write_json(output / "status.json", {"stage": "video_input", "status": "extracting", "run_id": run_id})
        write_json(output / "local_source.json", {"video_path": str(video), "private_local_reference": True})
        initial_hash = sha256(video)
        manifest["source"]["sha256"] = initial_hash
        phase = "open_video"
        with av.open(str(video), mode="r") as container:
            if not container.streams.video:
                raise VideoInputError("no_video_stream", "Input contains no video stream")
            if len(container.streams.video) > 1:
                raise VideoInputError("ambiguous_video_stream", "Multiple video streams require explicit stream selection")
            stream = container.streams.video[0]
            sar = stream.sample_aspect_ratio
            if sar and sar != 1:
                raise VideoInputError("non_square_pixels", "Non-square pixels require an explicit resampling adapter")
            manifest["source"].update({"stream_index": stream.index, "codec": stream.codec_context.name,
                "coded_width": stream.width, "coded_height": stream.height,
                "stream_time_base": str(stream.time_base), "declared_frames": stream.frames or None,
                "nominal_fps": float(stream.average_rate) if stream.average_rate else None,
                "duration_seconds": float(stream.duration * stream.time_base) if stream.duration is not None and stream.time_base else None,
                "sample_aspect_ratio": str(sar) if sar else None,
                "raw_sample_aspect_ratio": str(sar) if sar is not None else None})
            # FFmpeg also uses 0/1 for an unspecified ratio (e.g. this iPhone file).
            if not sar:
                manifest["warnings"].append("sample_aspect_ratio_unknown; decoded pixels are treated as square")
            rate = Fraction(str(sample_fps))
            origin = previous = None
            expected_geometry = None
            next_sample = Fraction(0)
            extraction_started = time.perf_counter()
            phase = "decode_frames"
            with (output / "frame_decisions.jsonl").open("w", encoding="utf-8") as audit:
                for index, frame in enumerate(container.decode(stream)):
                    manifest["decoded_frames"] += 1
                    decision = {"decoded_frame_index": index, "source_pts": frame.pts,
                                "source_time_base": str(frame.time_base) if frame.time_base else None,
                                "source_pts_s": None, "selected": False}
                    try:
                        timestamp = presentation_time(frame.pts, frame.time_base, previous)
                        decision["source_pts_s"] = float(timestamp)
                        if frame.is_corrupt:
                            raise VideoInputError("corrupt_frame", "Decoder flagged a corrupt frame")
                        rotation = display_rotation(frame, stream.metadata)
                        current = (frame.width, frame.height, rotation)
                        if expected_geometry is None:
                            expected_geometry = current
                        elif current != expected_geometry:
                            raise VideoInputError("changing_geometry", "Coded image size or display rotation changed within one input")
                        if origin is None:
                            origin = timestamp
                            manifest["source"]["first_pts_s"] = float(origin)
                        relative_time = timestamp - origin
                        decision.update(timestamp_s=float(relative_time), timestamp_source="source_pts")
                        if relative_time >= next_sample:
                            display_size, image_size, matrix = geometry(frame.width, frame.height, rotation, max_width)
                            image = frame.to_image()
                            if rotation:
                                image = image.transpose({90: Image.Transpose.ROTATE_90, 180: Image.Transpose.ROTATE_180,
                                                         270: Image.Transpose.ROTATE_270}[rotation])
                            if image.size != image_size:
                                image = image.resize(image_size, Image.Resampling.LANCZOS)
                            name = f"frame_{index + 1:07d}.jpg"
                            path = frames_dir / name
                            image.save(path, format="JPEG", quality=jpeg_quality, subsampling=0)
                            record = {"frame_id": name, "path": "frames/" + name, **decision,
                                "width": image_size[0], "height": image_size[1], "bytes": path.stat().st_size,
                                "sha256": sha256(path), "selected": True, "reason": "sampling_grid",
                                "coded_width": frame.width, "coded_height": frame.height,
                                "display_width": display_size[0], "display_height": display_size[1],
                                "rotation_applied_ccw_degrees": rotation, "coded_to_processed_pixel_centers": matrix}
                            manifest["frames"].append(record)
                            decision.update(selected=True, reason="sampling_grid", frame_id=name)
                            next_sample = Fraction(int(relative_time * rate) + 1, 1) / rate
                        else:
                            decision["reason"] = "sampling_interval"
                        previous = timestamp
                    except VideoInputError as error:
                        decision.update(reason=error.code, error=str(error))
                        raise
                    finally:
                        audit.write(json.dumps(decision, ensure_ascii=False, allow_nan=False) + "\n")
            extraction_seconds = time.perf_counter() - extraction_started
            manifest["source"]["last_pts_s"] = float(previous) if previous is not None else None
            if not manifest["decoded_frames"]:
                raise VideoInputError("empty_video", "The video decoded no frames")
            if stream.frames and manifest["decoded_frames"] != stream.frames:
                raise VideoInputError("incomplete_decode", "Decoded frame count differs from the container's declared count")
            if len(manifest["frames"]) < 3:
                raise VideoInputError("insufficient_frames", "Fewer than three sampled frames; increase sample_fps or provide a longer video")
        phase = "verify_source"
        if video.stat().st_size != manifest["source"]["bytes"] or sha256(video) != initial_hash:
            raise VideoInputError("source_changed", "Source video changed during extraction")
        manifest["source"]["unchanged_after_decode"] = True
        manifest["frame_decisions_sha256"] = sha256(output / "frame_decisions.jsonl")
        phase = "ready"
        finish("frames_ready")
        return {"status": "frames_ready", "run_id": run_id, "output": str(output),
                "decoded_frames": manifest["decoded_frames"], "selected_frames": len(manifest["frames"])}
    except (Exception, KeyboardInterrupt) as error:
        if extraction_started is not None and extraction_seconds is None:
            extraction_seconds = time.perf_counter() - extraction_started
        code = error.code if isinstance(error, VideoInputError) else "interrupted" if isinstance(error, KeyboardInterrupt) else "video_input_failed"
        finish("failed", {"code": code, "phase": phase, "type": type(error).__name__, "message": str(error)})
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, required=True, help="Existing local video")
    parser.add_argument("--output", type=Path, help="New directory; default: output/phone_mapping/<run_id>/input")
    parser.add_argument("--run-id", help="Optional stable ID; generated automatically when omitted")
    parser.add_argument("--sample-fps", type=float, default=2, help="Sampling grid rate; default: 2")
    parser.add_argument("--max-width", type=int, default=0, help="Maximum oriented image width; 0 preserves size")
    parser.add_argument("--jpeg-quality", type=int, default=95)
    args = parser.parse_args()
    try:
        result = extract_video(args.video, args.output, run_id=args.run_id,
                               sample_fps=args.sample_fps, max_width=args.max_width, jpeg_quality=args.jpeg_quality)
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(json.dumps({"status": "failed", "code": getattr(error, "code", type(error).__name__), "message": str(error)}), file=sys.stderr)
        return 2
    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
