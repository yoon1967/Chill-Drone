"""Use one forward range beam with the fixed monocular map scale."""
from collections import deque
import math

import numpy as np

BEAM_RADIUS = math.tan(math.radians(12.5))
FRONT_TOF_MIN_M, FRONT_TOF_MAX_M = .1, 1.2


def match_front_surface(depth, params, distance, excluded=None):
    """Return depth anchored to one front range, with every unsupported pixel set to zero."""
    if not FRONT_TOF_MIN_M <= distance < FRONT_TOF_MAX_M:
        return None
    fx, fy, cx, cy = params
    y, x = np.ogrid[:depth.shape[0], :depth.shape[1]]
    radius = np.hypot((x - cx) / fx, (y - cy) / fy)
    ranges = depth * np.sqrt(1 + radius ** 2)
    beam = radius <= BEAM_RADIUS
    valid = beam & np.isfinite(ranges) & (ranges > 0)
    if excluded is not None:
        valid &= ~excluded
    core = valid & (radius <= BEAM_RADIUS / 2)
    if valid.sum() < .7 * beam.sum() or core.sum() < 8:
        return None
    near = float(np.percentile(ranges[valid], 20))
    center = float(np.median(ranges[core]))
    # One beam reports a range, not a depth image. Use a coherent surface near
    # the optical axis and reject edges or gross disagreement with the model.
    if not .9 <= center / near <= 1.2 or not .6 <= distance / near <= 1.6:
        return None
    correction = distance / near
    radial_m = ranges * correction
    supported = valid & (np.abs(radial_m - distance) <= max(.04, .1 * distance)) & (radial_m < FRONT_TOF_MAX_M)
    if np.count_nonzero(supported & core) < 8:
        return None
    return np.where(supported, depth * correction, 0).astype(np.float32), near


class TofDepth:
    def __init__(self):
        self.samples = deque(maxlen=16)
        self.meters_per_unit = None

    def scale_to_map(self, pose_scale, model_range, sample):
        distance, sampled_at = sample
        candidate = distance / (model_range * pose_scale)
        if self.meters_per_unit is None:
            self._calibrate(sampled_at, candidate)
        if self.meters_per_unit is None:
            return 1 / candidate, False
        ratio = candidate / self.meters_per_unit
        if not .8 <= ratio <= 1.2:
            return None, False
        return 1 / self.meters_per_unit, abs(ratio - 1) >= .01

    def _calibrate(self, stamp, ratio):
        # One observation per half second and a median consensus, as in the
        # autonomy depth scale. Freeze the scale once it fits one SLAM segment.
        while self.samples and stamp - self.samples[0][0] > 15:
            self.samples.popleft()
        if self.samples and stamp - self.samples[-1][0] < .5:
            return
        self.samples.append((stamp, ratio))
        if len(self.samples) >= 5:
            values = np.asarray([value for _, value in self.samples])
            median = float(np.median(values))
            inliers = values[np.abs(values / median - 1) <= .15]
            if len(inliers) >= max(5, math.ceil(.75 * len(values))):
                self.meters_per_unit = float(np.median(inliers))
