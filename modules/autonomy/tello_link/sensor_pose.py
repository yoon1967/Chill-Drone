"""Receive-time Tello odometry for camera frames, independent of image processing."""

from collections import deque
import math
from typing import NamedTuple


class SensorPose(NamedTuple):
    segment: int
    x: float
    y: float
    z: float
    yaw: float
    pitch: float
    roll: float

    @classmethod
    def from_values(cls, values):
        if (isinstance(values, list) and len(values) == 7 and type(values[0]) is int and values[0] >= 0
                and all(type(v) in (int, float) and abs(v) <= 1e6 for v in values[1:])):
            return cls(*values)
        return None


MAX_POSE_AGE_S = .2
MAX_STATE_GAP_S = .35


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class TelloOdometry:
    """Integrate each state packet once, in power-on forward/left/up axes."""

    def __init__(self):
        self.samples = deque(maxlen=16)
        self.segment = 0
        self.height_anchored = False

    def update(self, raw, stamp):
        fields = [raw.get(k) for k in ('vgx', 'vgy', 'vgz', 'yaw', 'pitch', 'roll')]
        if not all(type(v) in (int, float) and abs(v) <= 360 for v in fields):
            return
        # From the tt-sim real-flight convention: vgx/vgy are power-on axes,
        # not body axes. A yaw-dependent rotation would turn the velocity twice.
        gx, gy, gz, yaw, pitch, roll = fields
        vx, vy, vz = -gx / 10., gy / 10., gz / 10.
        yaw, pitch, roll = map(math.radians, (-yaw, -pitch, roll))
        down = raw.get('tof')
        # Project the downward beam onto gravity before treating it as height.
        tilt = math.cos(pitch) * math.cos(roll)
        height = down / 100. * tilt if type(down) in (int, float) and 10 < down < 600 and tilt > .85 else None
        x = y = z = 0.
        if self.samples:
            before, last, velocity = self.samples[-1]
            dt = stamp - before
            if dt <= 0:
                return
            if dt > MAX_STATE_GAP_S:
                self.segment += 1
                self.samples.clear()
                self.height_anchored = False
            else:
                x = last.x + (velocity[0] + vx) * dt / 2
                y = last.y + (velocity[1] + vy) * dt / 2
                z = last.z + (velocity[2] + vz) * dt / 2
                # A sudden shorter floor range can be furniture passing below.
                if self.height_anchored and height is not None and abs(height - z) > .25:
                    height = None
        if height is not None:
            self.height_anchored = True
            z = height
        self.samples.append((stamp, SensorPose(self.segment, x, y, z, yaw, pitch, roll), (vx, vy, vz)))

    def at(self, stamp):
        """Interpolate available samples or extrapolate briefly, without waiting."""
        if not self.samples or stamp < self.samples[0][0]:
            return None
        for later in range(1, len(self.samples)):
            end, right, _ = self.samples[later]
            if stamp <= end:
                start, left, _ = self.samples[later - 1]
                fraction = (stamp - start) / (end - start)
                return SensorPose(self.segment, *(a + fraction * (b - a) for a, b in zip(left[1:4], right[1:4])),
                                  *(wrap(a + fraction * wrap(b - a)) for a, b in zip(left[4:], right[4:])))
        end, pose, velocity = self.samples[-1]
        age = stamp - end
        if age > MAX_POSE_AGE_S:
            return None
        angles = pose[4:]
        if len(self.samples) > 1:
            start, previous, _ = self.samples[-2]
            if end - start >= .03:
                turn = min(age, .1) * wrap(pose.yaw - previous.yaw) / (end - start)
                angles = (wrap(pose.yaw + max(-.25, min(.25, turn))), pose.pitch, pose.roll)
        return SensorPose(self.segment, *(p + v * age for p, v in zip(pose[1:4], velocity)), *angles)
