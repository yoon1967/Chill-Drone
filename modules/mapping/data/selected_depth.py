"""Inference adapters for explicitly selected local reconstruction models."""
import math
import sys

import numpy as np

from .reconstruction_models import DEPTH_V2, MOGE, MOGE_WEIGHT, VIDEO_DEPTH, VIDEO_WEIGHT


class VideoDepthEstimator:
    def __init__(self, location, camera):
        import torch
        from video_depth_anything.video_depth_stream import VideoDepthAnything
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        model = VideoDepthAnything(encoder='vits', features=64, out_channels=[48, 96, 192, 384])
        model.load_state_dict(torch.load(location / VIDEO_WEIGHT, map_location='cpu', weights_only=True))
        self.model = model.to(self.device).eval()

    def prepare(self, shape):
        # Synthetic frames would alter the causal history for the first real frame.
        pass

    def estimate(self, rgb):
        size = max(14, round(min(rgb.shape[:2]) / 14) * 14)
        depth = self.model.infer_video_depth_one(rgb, input_size=size, device=self.device, fp32=self.device == 'cpu')
        return np.array(depth, dtype=np.float32, copy=True)


class MoGeDepthEstimator:
    def __init__(self, location, camera):
        import torch
        from moge.model.v2 import MoGeModel
        self.torch = torch
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = MoGeModel.from_pretrained(location / MOGE_WEIGHT).to(self.device).eval()
        self.fov = math.degrees(2 * math.atan(camera['width'] / (2 * camera['fx']))) if camera else None

    def estimate(self, rgb):
        image = self.torch.as_tensor(rgb, device=self.device).permute(2, 0, 1).float() / 255
        output = self.model.infer(image, resolution_level=0, fov_x=self.fov, use_fp16=self.device == 'cuda')
        return np.array(output['depth'].cpu().numpy(), dtype=np.float32, copy=True)


def open_depth(key, location, camera=None):
    if key == DEPTH_V2:
        from .stream_mapping import DepthEstimator
        return DepthEstimator(location=location)
    # Keep optional source packages isolated to the inference process.
    runtime = str(location / 'runtime')
    if runtime not in sys.path:
        sys.path.insert(0, runtime)
    return {VIDEO_DEPTH: VideoDepthEstimator, MOGE: MoGeDepthEstimator}[key](location, camera)
