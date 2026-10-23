"""Reload saved sparse SfM exports and render data-derived PNG previews with Agg."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time

import numpy as np
from result_io import digest, validate_saved_result

ROOT = Path(__file__).resolve().parents[3]


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def render_preview(result_dir, output):
    started = time.perf_counter()
    source, output = Path(result_dir).resolve(), Path(output).resolve()
    if output.is_relative_to(source) or source.is_relative_to(output):
        raise ValueError("Preview must be separate from the saved result")
    if output.is_relative_to(ROOT) and not output.is_relative_to(ROOT / "output"):
        raise ValueError("Generated repository files must be below output/")
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "status.json", {"status": "running"})
    try:
        scene, validation = validate_saved_result(source)
        write_json(output / "validation.json", validation)
        # Keep font caches local to the generated run, including in restricted environments.
        os.environ["MPLCONFIGDIR"] = str(output / ".matplotlib")
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        points = np.asarray(scene["points"], dtype=float)
        xyz, rgb = points[:, 1:4], points[:, 4:7] / 255
        route = np.asarray([camera["position"] for camera in scene["cameras"]])
        unit, quality = scene["scale"]["unit"], scene["quality"]
        label = f"{scene['input_run_id']} | {validation['result_status'].upper()} | {len(route)}/{quality['input_frames']} frames | {len(xyz):,} points"
        with plt.rc_context({"figure.facecolor": "#f4f6f8", "axes.facecolor": "white", "font.size": 10}):
            fig = plt.figure(figsize=(12, 9), layout="constrained")
            ax = fig.add_subplot(111, projection="3d")
            ax.scatter(*xyz.T, c=rgb, s=3, depthshade=False, rasterized=True)
            ax.plot(*route.T, color="#c63d22", linewidth=1.7, label="Camera route (PTS order)")
            ax.scatter(*route[0], c="#157350", s=45, label="First registered camera")
            ax.scatter(*route[-1], c="#c63d22", marker="x", s=50, label="Last registered camera")
            combined = np.vstack((xyz, route))
            center = (combined.min(axis=0) + combined.max(axis=0)) / 2
            radius = max(float(np.ptp(combined, axis=0).max()) * 0.55, 1e-3)
            for setter, value in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), center):
                setter(value - radius, value + radius)
            ax.set_box_aspect((1, 1, 1))
            ax.set(xlabel=f"X ({unit})", ylabel=f"Y ({unit})", zlabel=f"Z ({unit})")
            ax.view_init(elev=22, azim=-60)
            ax.legend(loc="upper right")
            fig.suptitle(label + "\nSparse observed points; axes are not measured floor/gravity", fontsize=12)
            fig.savefig(output / "overview.png", dpi=140)
            plt.close(fig)
            fig, axes = plt.subplots(1, 3, figsize=(15, 5.5), layout="constrained")
            for ax, (a, b) in zip(axes, [(0, 1), (0, 2), (1, 2)]):
                ax.scatter(xyz[:, a], xyz[:, b], c=rgb, s=3, rasterized=True)
                ax.plot(route[:, a], route[:, b], color="#c63d22", linewidth=1.2)
                ax.scatter(route[0, a], route[0, b], c="#157350", s=25)
                ax.set(xlabel=f"{'XYZ'[a]} ({unit})", ylabel=f"{'XYZ'[b]} ({unit})", title=f"{'XYZ'[a]}{'XYZ'[b]} projection")
                ax.set_aspect("equal", adjustable="datalim")
                ax.grid(alpha=0.2)
            fig.suptitle(label + "\nAll points shown; projections are not measured floor plans", fontsize=12)
            fig.savefig(output / "projections.png", dpi=140)
            plt.close(fig)
        files = ("overview.png", "projections.png", "validation.json")
        write_json(output / "manifest.json", {"schema_version": "phone-preview/1.0", "input_run_id": scene["input_run_id"],
                   "scene_sha256": validation["export_sha256"]["scene.json"], "unit": unit,
                   "result_status": validation["result_status"], "matplotlib": matplotlib.__version__,
                   "renderer_sha256": digest(Path(__file__)), "validator_sha256": digest(Path(__file__).with_name("result_io.py")),
                   "files": {name: {"sha256": digest(output / name), "bytes": (output / name).stat().st_size} for name in files}})
        write_json(output / "status.json", {"status": "preview_ready", "input_run_id": scene["input_run_id"]})
    except (Exception, KeyboardInterrupt) as error:
        write_json(output / "status.json", {"status": "failed", "type": type(error).__name__, "reason": str(error)})
        raise
    finally:
        write_json(output / "timing.json", {"elapsed_seconds": time.perf_counter() - started,
                   "scope": "saved export reload/validation and PNG/report saves; final timing write excluded"})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    render_preview(args.result, args.output)


if __name__ == "__main__":
    main()
