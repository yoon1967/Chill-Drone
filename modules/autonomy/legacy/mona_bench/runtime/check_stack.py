#!/usr/bin/env python3
"""Validate installed components without connecting to or moving a drone."""
import importlib
import json
from pathlib import Path
import subprocess
import sys

import rospkg

PACKAGES = ["rmtt_driver", "sc_depth_pl_ros", "scale_recovery", "orb_slam2_ros",
            "plan_manage", "mav_controller", "mona_integration"]
result = dict(packages={}, imports={}, flight_commands_sent=False)
rospack = rospkg.RosPack()
for package in PACKAGES:
    try:
        result["packages"][package] = rospack.get_path(package)
    except rospkg.ResourceNotFound:
        result["packages"][package] = None
for package, subdir in (("sc_depth_pl_ros", "src"), ("rmtt_driver", "scripts")):
    if result["packages"].get(package):
        sys.path.insert(0, str(Path(result["packages"][package]) / subdir))
for module in ("rospy", "cv_bridge", "tf", "ros_numpy", "numpy", "torch", "torchvision",
               "pytorch_lightning", "robomaster", "rmtt_core18", "SC_DepthV2", "onnxruntime"):
    try:
        loaded = importlib.import_module(module)
        result["imports"][module] = getattr(loaded, "__version__", "ok")
    except Exception as error:
        result["imports"][module] = {"error": str(error)}
try:
    import torch
    result["cuda_available"] = torch.cuda.is_available()
except ImportError:
    result["cuda_available"] = False
graph = subprocess.run(["roslaunch", "--nodes", "mona_integration", "full.launch"], capture_output=True, text=True)
result["launch_graph"] = dict(ok=graph.returncode == 0, nodes=graph.stdout.splitlines(), error=graph.stderr)
result["source"] = json.loads(Path("/opt/mona_ws/source-manifest.json").read_text())
result["checkpoint_present"] = Path("/models/sc_depth_v2_nyu.ckpt").is_file()
result["validation_scope"] = "installation_only"
result["inference_validated"] = False
result["onnx_models_present"] = {"da_v2_metric_indoor_small": Path("/models/da_v2_metric_indoor_small_518x686_fp16.onnx").is_file(),
                                 "da_v2_metric_indoor_small_fp32": Path("/models/da_v2_metric_indoor_small_518x686.onnx").is_file(),
                                 "sc_depth_v2_nyu": Path("/models/sc_depth_v2_nyu_480x640.onnx").is_file()}
result["onnx_model_present"] = result["onnx_models_present"]["da_v2_metric_indoor_small"]
result["default_depth_model"] = "da_v2_metric_indoor_small"
result["default_backend"] = "onnx_cuda"
try:
    import onnxruntime
    result["onnx_providers"] = onnxruntime.get_available_providers()
except ImportError:
    result["onnx_providers"] = []
result["blockers"] = [] if result["onnx_model_present"] else ["DA-V2 metric ONNX model missing (run export_da2_onnx.py)"]
result["ok"] = (all(result["packages"].values()) and graph.returncode == 0
                and all(isinstance(value, str) for value in result["imports"].values()))
print(json.dumps(result, indent=2))
sys.exit(0 if result["ok"] else 1)
