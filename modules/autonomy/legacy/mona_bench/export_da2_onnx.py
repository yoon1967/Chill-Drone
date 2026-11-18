"""Export Depth Anything V2 Metric Indoor Small to the fixed-shape ONNX used by onnx_depth.py.

Run on the host (needs torch, transformers and onnx; not part of the ROS image):
    python modules/autonomy/legacy/mona_bench/export_da2_onnx.py data/outputs/mona-bench/models
    python modules/autonomy/legacy/mona_bench/export_da2_onnx.py data/outputs/mona-bench/models --precision fp32

fp16 (default, needs a CUDA GPU to export) keeps float32 input/output and is about 1.7x faster on the
RTX 4050 Laptop; fp32 is for CPU inference. Each run writes the .onnx, a .source.json with the pinned
upstream revision and hashes, and a reference input/output pair for the container parity check.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

REPO = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"  # Apache-2.0 (Small only)
REVISION = "8078d68a9c75a972131914f6afd0c1723be0da7f"
WEIGHTS_SHA256 = "e990eb82fbf11b05b7813261196a2b841bdcf5a05f64396724a8987fa90504a3"
INPUT_HW = (518, 686)  # 720x960 Tello frame, short side 518, multiple of 14 (DPT processor rule)
NAMES = {"fp16": "da_v2_metric_indoor_small_518x686_fp16", "fp32": "da_v2_metric_indoor_small_518x686"}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--weights-dir", type=Path, help="Optional separate copy of the Hugging Face weights")
    parser.add_argument("--precision", choices=sorted(NAMES), default="fp16")
    args = parser.parse_args()
    import torch
    from huggingface_hub import snapshot_download
    from huggingface_hub.errors import LocalEntryNotFoundError
    from transformers import AutoModelForDepthEstimation

    options = dict(revision=REVISION, local_dir=args.weights_dir, allow_patterns=["*.json", "model.safetensors"])
    try:
        weights = Path(snapshot_download(REPO, **options, local_files_only=True))
    except LocalEntryNotFoundError:
        weights = None
    if weights is None or not all((weights / name).is_file() for name in ("config.json", "model.safetensors")):
        weights = Path(snapshot_download(REPO, **options))
    if sha256(weights / "model.safetensors") != WEIGHTS_SHA256:
        weights = Path(snapshot_download(REPO, **options, force_download=True))
    if sha256(weights / "model.safetensors") != WEIGHTS_SHA256:
        raise SystemExit("model.safetensors SHA-256 differs from the pinned revision")
    model = AutoModelForDepthEstimation.from_pretrained(weights).eval()
    half = args.precision == "fp16"
    if half:
        if not torch.cuda.is_available():
            raise SystemExit("fp16 export needs a CUDA GPU (use --precision fp32 otherwise)")
        model = model.cuda().half()

    class Wrapper(torch.nn.Module):
        def __init__(self, inner):
            super().__init__()
            self.inner = inner

        def forward(self, pixel_values):  # float32 in and out; fp16 inside when half
            if not half:  # unchanged fp32 graph (pinned hash)
                return self.inner(pixel_values=pixel_values).predicted_depth[:, None]  # metres, 1x1xHxW
            return self.inner(pixel_values=pixel_values.half()).predicted_depth[:, None].float()

    wrapper = Wrapper(model)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    name = NAMES[args.precision]
    onnx_path = args.output_dir / f"{name}.onnx"
    example = torch.from_numpy(np.random.default_rng(3).normal(0, 1, (1, 3, *INPUT_HW)).astype(np.float32))
    if half:
        example = example.cuda()
    torch.onnx.export(wrapper, (example,), str(onnx_path), input_names=["pixel_values"],
                      output_names=["depth"], opset_version=17, dynamo=False)
    with torch.no_grad():
        reference = wrapper(example).cpu().numpy()
    np.savez(args.output_dir / f"{name}.reference.npz", input=example.cpu().numpy(), depth=reference)
    record = {"repo": REPO, "revision": REVISION, "license": "Apache-2.0",
              "weights_sha256": WEIGHTS_SHA256, "onnx_sha256": sha256(onnx_path),
              "onnx_bytes": onnx_path.stat().st_size, "input": [1, 3, *INPUT_HW],
              "output": "metric depth in metres, 1x1xHxW", "opset": 17, "precision": args.precision,
              "torch": torch.__version__, "transformers": __import__("transformers").__version__}
    (args.output_dir / f"{name}.onnx.source.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
