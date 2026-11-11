#!/usr/bin/env python3
"""Check one fresh image from the existing web server; sends GET requests only."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np

from capture_http import fetch
from onnx_depth import MODELS, DepthModel


def check(url, output, device, kind):
    status = json.loads(fetch(url, "/api/status"))
    video_age = (status.get("video") or {}).get("frame_age_s")
    if video_age is None or not 0 <= video_age <= 0.3:
        raise RuntimeError("fresh video unavailable; connect SDK and start video first")
    image = cv2.imdecode(np.frombuffer(fetch(url, "/snapshot.jpg"), np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("snapshot is not a JPEG")
    model = DepthModel(device=device, kind=kind)
    depth, seconds = model.infer(image)
    output.mkdir(parents=True, exist_ok=False)
    name = "metric_depth" if model.spec.output == "metric" else "relative_depth"
    np.save(output / f"{name}.npy", depth, allow_pickle=False)
    cv2.imwrite(str(output / "rgb.jpg"), image)
    low, high = np.percentile(depth, [2, 98])
    preview = (np.clip((depth - low) / max(high - low, 1e-6), 0, 1) * 255).astype(np.uint8)
    cv2.imwrite(str(output / f"{name}.png"), cv2.applyColorMap(preview, cv2.COLORMAP_TURBO))
    state = status.get("state") or {}
    result = dict(ok=True, model=kind, depth_kind=model.spec.output, model_sha256=model.sha256, provider=model.provider,
                  inference_seconds=seconds, shape=list(depth.shape),
                  depth_percentiles=np.percentile(depth, [2, 50, 98]).tolist(),
                  state_age_s=state.get("age_s"), bottom_tof_cm=(state.get("raw") or {}).get("tof"),
                  source_mode=status.get("mode"), captured_utc=datetime.now(timezone.utc).isoformat(),
                  metric_calibration_validated=False, flight_commands_sent=False,
                  limitation="single depth frame; not navigation or collision-avoidance validation")
    (output / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(dict(output=str(output), **result), indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://host.docker.internal:8010")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", choices=("cuda", "cpu"), default="cuda")
    parser.add_argument("--model", choices=sorted(MODELS), default="da_v2_metric_indoor_small")
    args = parser.parse_args()
    check(args.url, args.output, args.device, args.model)
