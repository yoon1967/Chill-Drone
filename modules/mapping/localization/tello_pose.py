"""Convert Tello body odometry to the mapping pipeline's optical axes."""

import math
from collections import deque

import cv2
import numpy as np

from .depth_pose import VisualPoseReferences
from .pose_first import extrapolate_pose


# Camera right/down/forward -> body forward/left/up.
OPTICAL_TO_BODY = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
CAMERA_FORWARD_M = .05  # tt-sim front mount; nominal for the real camera.
LINEAR_PREDICTION = 'tello_linear_prediction'
METRIC_POSE_SOURCES = {'tello_sensor_odometry', 'tello_visual_recovery', LINEAR_PREDICTION}


def yaw_rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def camera_transform(sample):
    """Return camera rotation and center in the power-on body reference."""
    cp, sp = math.cos(sample.pitch), math.sin(sample.pitch)
    cr, sr = math.cos(sample.roll), math.sin(sample.roll)
    tilt = np.array([[cp, sp * sr, sp * cr], [0., cr, -sr], [-sp, cp * sr, cp * cr]])
    body = yaw_rotation(sample.yaw) @ tilt
    return body @ OPTICAL_TO_BODY, np.asarray(sample[1:4]) + CAMERA_FORWARD_M * body[:, 0]


class TelloMapPose(VisualPoseReferences):
    """Maintain one metric map and re-anchor odometry only against that map."""

    def __init__(self, *, max_prediction_age=None):
        super().__init__()
        self.epoch = None
        self.world_from_odometry = None
        self.observed = deque(maxlen=2)
        self.predicting = False
        self.max_prediction_age = max_prediction_age

    def resolve(self, telemetry, rgb, params, stamp):
        pose, source = self._observed_pose(telemetry, rgb, params)
        if pose is not None:
            if self.predicting:
                # A relocalization correction is not a velocity sample.
                self.observed.clear()
            self.observed.append((stamp, pose))
            self.predicting = False
            return pose, source
        if not self.observed:
            return None, source
        self.predicting = True
        latest_stamp, latest = self.observed[-1]
        if self.max_prediction_age is not None and stamp - latest_stamp > self.max_prediction_age:
            return None, source
        previous_stamp, previous = self.observed[0]
        interval = latest_stamp - previous_stamp
        if interval < 1 / 120:
            return latest.copy(), LINEAR_PREDICTION
        steps = (stamp - latest_stamp) / interval
        return extrapolate_pose(previous, latest, steps, steps), LINEAR_PREDICTION

    def _observed_pose(self, telemetry, rgb, params):
        epoch, sample, allow_visual_origin = telemetry
        if sample is None:
            if allow_visual_origin and self.epoch is None and not self.references:
                return np.eye(4), 'tello_visual_recovery'
            pose = self.relocalize(rgb, params)
            return pose, 'tello_visual_recovery'
        epoch = (epoch, sample.segment)
        rotation, position = camera_transform(sample)
        odometry_pose = np.eye(4)
        odometry_pose[:3, :3], odometry_pose[:3, 3] = rotation, position
        if epoch != self.epoch:
            if self.epoch is not None or self.references:
                world_pose = self.relocalize(rgb, params)
                if world_pose is None:
                    return None, 'tello_sensor_odometry'
                self.world_from_odometry = world_pose @ np.linalg.inv(odometry_pose)
            else:
                axes = OPTICAL_TO_BODY.T @ yaw_rotation(-sample.yaw)
                self.world_from_odometry = np.eye(4)
                self.world_from_odometry[:3, :3] = axes
                self.world_from_odometry[:3, 3] = -axes @ position
            self.epoch = epoch
        return self.world_from_odometry @ odometry_pose, 'tello_sensor_odometry'
