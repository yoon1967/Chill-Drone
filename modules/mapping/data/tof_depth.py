"""Use one forward range beam with the fixed monocular map scale."""
from collections import deque
import math

import numpy as np

BEAM_RADIUS = math.tan(math.radians(12.5))


class TofDepth:
    def __init__(self):
        self.samples = deque(maxlen=16)
        self.meters_per_unit = None

    def correct(self, depth, pose_scale, params, sample, excluded=None):
        distance, sampled_at = sample
        fx, fy, cx, cy = params
        # Assume the extension beam is centered on the optical axis. Limit the
        # correction to the beam and convert camera Z into radial distance.
        y, x = np.ogrid[:depth.shape[0], :depth.shape[1]]
        ray_radius = np.hypot((x - cx) / fx, (y - cy) / fy)
        radius = ray_radius / BEAM_RADIUS
        ranges = depth * np.sqrt(1 + ray_radius ** 2)
        valid = (radius <= 1) & np.isfinite(ranges) & (ranges > 0)
        if excluded is not None:
            valid &= ~excluded
        core = valid & (radius <= .5)
        if valid.sum() < .7 * np.count_nonzero(radius <= 1) or core.sum() < 8:
            return depth, False
        near = float(np.percentile(ranges[valid], 20))
        center = float(np.median(ranges[core]))
        # Require agreement between the foreground in the beam and the optical
        # center. Reject likely edges, missing depth and incompatible returns.
        if not .9 <= center / near <= 1.2 or not .6 <= distance * pose_scale / near <= 1.6:
            return depth, False
        candidate = distance / near
        if self.meters_per_unit is None:
            self._calibrate(sampled_at, candidate)
        if self.meters_per_unit is None:
            return depth, False
        ratio = candidate / self.meters_per_unit
        if not .8 <= ratio <= 1.2 or abs(ratio - 1) < .01:
            return depth, False

        # Feather at the beam boundary and across depth changes. Other surfaces
        # and camera translations stay in the same SLAM coordinate frame.
        angular_weight = np.clip(2 * (1 - radius[valid]), 0, 1)
        surface_weight = np.clip((.2 - np.abs(ranges[valid] / near - 1)) / .15, 0, 1)
        corrected = depth.copy()
        corrected[valid] *= 1 + (ratio - 1) * angular_weight * surface_weight
        return corrected, True

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
