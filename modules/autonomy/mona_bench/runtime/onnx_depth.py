#!/usr/bin/env python3
"""Monocular depth ONNX inference shared by ROS and the local image check.

Two verified models are supported; both publish on the SC-Depth topics so the rest of
MoNA-Bench (scale recovery, SLAM, planner) is unchanged:
  sc_depth_v2_nyu            relative depth (scale recovered from the bottom ToF)
  da_v2_metric_indoor_small  metric depth in metres, fp16 inside (CUDA); ToF scale recovery only corrects it
  da_v2_metric_indoor_small_fp32  the same model in fp32 (CPU inference)
"""
from collections import namedtuple
import hashlib
from pathlib import Path
import time

import cv2
import numpy as np
import onnxruntime as ort

if __package__:
    from .depth_artifacts import DA_V2_FP16_FILE, DA_V2_FP16_SHA256
else:
    from depth_artifacts import DA_V2_FP16_FILE, DA_V2_FP16_SHA256

Spec = namedtuple("Spec", "path sha256 input_hw mean std interpolation output")
MODELS = {
    "sc_depth_v2_nyu": Spec(
        "/models/sc_depth_v2_nyu_480x640.onnx",
        "69a555382b152ae71930bef2777d2a3945df666c48f73bc6748873ab5f9e4d84",
        (480, 640), (0.45, 0.45, 0.45), (0.225, 0.225, 0.225), cv2.INTER_LINEAR, "relative"),
    # Exported by export_da2_onnx.py from the pinned Hugging Face revision (Apache-2.0).
    # fp16 inside with float32 input/output: 37 ms vs 64 ms per frame on the RTX 4050 Laptop,
    # p99 relative difference 0.26% on real frames, so the depth node keeps up with 8-9 Hz video.
    "da_v2_metric_indoor_small": Spec(
        f"/models/{DA_V2_FP16_FILE}", DA_V2_FP16_SHA256,
        (518, 686), (0.485, 0.456, 0.406), (0.229, 0.224, 0.225), cv2.INTER_CUBIC, "metric"),
    "da_v2_metric_indoor_small_fp32": Spec(
        "/models/da_v2_metric_indoor_small_518x686.onnx",
        "429f8d354187818d1f39d4d3280d948cf29651b75c40c105a2b661142ededbd3",
        (518, 686), (0.485, 0.456, 0.406), (0.229, 0.224, 0.225), cv2.INTER_CUBIC, "metric"),
}
DEFAULT_KIND = "sc_depth_v2_nyu"
MODEL_SHA256 = MODELS[DEFAULT_KIND].sha256
DEFAULT_MODEL = MODELS[DEFAULT_KIND].path
OUTPUT_HW = (480, 640)  # matches the camera_info used by scale recovery, SLAM and the planner
PROVIDERS = {"cpu": "CPUExecutionProvider", "cuda": "CUDAExecutionProvider"}


def prepare_bgr(image, spec=MODELS[DEFAULT_KIND]):
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("expected uint8 BGR image")
    height, width = spec.input_hw
    rgb = cv2.cvtColor(cv2.resize(image, (width, height), interpolation=spec.interpolation), cv2.COLOR_BGR2RGB)
    tensor = (rgb.astype(np.float32) / 255.0 - np.float32(spec.mean)) / np.float32(spec.std)
    return np.ascontiguousarray(tensor.transpose(2, 0, 1)[None], dtype=np.float32)


class DepthModel:
    def __init__(self, path=None, device="cpu", kind=DEFAULT_KIND):
        if device not in PROVIDERS:
            raise ValueError("device must be cpu or cuda")
        if kind not in MODELS:
            raise ValueError(f"model must be one of {sorted(MODELS)}")
        self.spec = MODELS[kind]
        self.kind = kind
        path = Path(path or self.spec.path)
        self.sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        if self.sha256 != self.spec.sha256:
            raise ValueError(f"model SHA-256 differs from the verified {kind} ONNX artifact")
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        requested = PROVIDERS[device]
        if requested not in ort.get_available_providers():
            raise RuntimeError(f"{requested} is not available in this onnxruntime build")
        # Never fall back silently: a CPU session must not be reported as GPU inference.
        self.session = ort.InferenceSession(str(path), sess_options=options, providers=[requested])
        self.provider = self.session.get_providers()[0]
        if self.provider != requested:
            raise RuntimeError(f"requested {requested}, session uses {self.provider}")
        self.device = device
        self.input = self.session.get_inputs()[0]
        if self.input.shape != [1, 3, *self.spec.input_hw] or self.input.type != "tensor(float)":
            raise ValueError("unexpected model input contract")
        if device == "cuda":
            # First CUDA run allocates and selects kernels; keep it off the first real frame.
            self.session.run(None, {self.input.name: np.zeros((1, 3, *self.spec.input_hw), np.float32)})

    def infer(self, image):
        started = time.monotonic()
        depth = self.session.run(None, {self.input.name: prepare_bgr(image, self.spec)})[0]
        if depth.shape != (1, 1, *self.spec.input_hw) or not np.all(np.isfinite(depth) & (depth > 0)):
            raise ValueError("invalid depth output")
        depth = depth[0, 0].astype(np.float32)
        if depth.shape != OUTPUT_HW:
            depth = cv2.resize(depth, OUTPUT_HW[::-1], interpolation=cv2.INTER_LINEAR)
        return depth, time.monotonic() - started


def ros_main():
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    rospy.init_node("sc_depth_pl")
    bridge = CvBridge()
    kind = rospy.get_param("~model_kind", DEFAULT_KIND)
    model = DepthModel(rospy.get_param("~model", "") or None, rospy.get_param("~device", "cpu"), kind)
    rospy.loginfo("%s ONNX session uses %s; output is %s depth", kind, model.provider, model.spec.output)
    rgb_pub = rospy.Publisher("/sc_depth_pl/color/image_raw", Image, queue_size=1)
    depth_pub = rospy.Publisher("/sc_depth_pl/depth/image_raw", Image, queue_size=1)

    def callback(msg):
        try:
            rgb = bridge.imgmsg_to_cv2(msg, "bgr8")
            depth, elapsed = model.infer(rgb)
            depth_msg = bridge.cv2_to_imgmsg(depth, "32FC1")
            depth_msg.header = msg.header
            color_msg = bridge.cv2_to_imgmsg(rgb, "bgr8")
            color_msg.header = msg.header
            rgb_pub.publish(color_msg)
            depth_pub.publish(depth_msg)
            rospy.loginfo_throttle(5, "%s %s inference %.3fs", kind, model.device.upper(), elapsed)
        except Exception as error:
            rospy.logerr_throttle(2, "%s frame rejected: %s", kind, error)
    rospy.Subscriber("/tello/image_raw", Image, callback, queue_size=1, buff_size=8 * 1024 * 1024)
    rospy.spin()


if __name__ == "__main__":
    ros_main()
