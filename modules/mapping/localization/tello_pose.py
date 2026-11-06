"""Convert Tello body odometry to the mapping pipeline's optical axes."""

import math
from collections import deque

import cv2
import numpy as np

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


class TelloMapPose:
    """Maintain one metric map and re-anchor odometry only against that map."""

    def __init__(self):
        self.epoch = None
        self.world_from_odometry = None
        self.references = deque(maxlen=4)
        self.observed = deque(maxlen=2)
        self.predicting = False
        self.features = cv2.ORB_create(nfeatures=650, fastThreshold=12)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

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
            pose = self._relocalize(rgb, params)
            return pose, 'tello_visual_recovery'
        epoch = (epoch, sample.segment)
        rotation, position = camera_transform(sample)
        odometry_pose = np.eye(4)
        odometry_pose[:3, :3], odometry_pose[:3, 3] = rotation, position
        if epoch != self.epoch:
            if self.epoch is not None or self.references:
                world_pose = self._relocalize(rgb, params)
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

    def remember(self, stamp, rgb, depth, params, pose, excluded):
        if self.references and stamp - self.references[-1]['stamp'] < .4:
            return
        if excluded is not None:
            depth = np.where(excluded, 0., depth)
        self.references.append(dict(stamp=stamp, rgb=rgb, depth=depth, params=params, pose=pose))

    def _reference_features(self, ref):
        if 'descriptors' not in ref:
            gray = cv2.cvtColor(ref.pop('rgb'), cv2.COLOR_RGB2GRAY)
            mask = cv2.compare(ref['depth'], 0., cv2.CMP_GT)
            keypoints, descriptors = self.features.detectAndCompute(gray, mask)
            xy = np.asarray([kp.pt for kp in keypoints], dtype=np.float64).reshape(-1, 2)
            if len(xy):
                pixels = np.rint(xy).astype(int)
                z = ref['depth'][pixels[:, 1], pixels[:, 0]]
                keep = z > 0
                fx, fy, cx, cy = ref['params']
                xyz = np.column_stack(((xy[:, 0] - cx) * z / fx, (xy[:, 1] - cy) * z / fy, z))
                ref.update(descriptors=descriptors[keep], xyz=xyz[keep])
            else:
                ref.update(descriptors=None, xyz=None)
        return ref['descriptors'], ref['xyz']

    def _relocalize(self, rgb, params):
        if not self.references:
            return None
        keypoints, descriptors = self.features.detectAndCompute(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), None)
        if descriptors is None or len(descriptors) < 12:
            return None
        fx, fy, cx, cy = params
        camera = np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]])
        for ref in reversed(self.references):
            known, xyz = self._reference_features(ref)
            if known is None or len(known) < 12:
                continue
            pairs = self.matcher.knnMatch(known, descriptors, k=2)
            # Keep one depth point for each matched image location.
            matches = {}
            for first, second in pairs:
                if first.distance < .75 * second.distance:
                    best = matches.get(first.trainIdx)
                    if best is None or first.distance < best.distance:
                        matches[first.trainIdx] = first
            if len(matches) < 12:
                continue
            obj = xyz[[m.queryIdx for m in matches.values()]]
            uv = np.asarray([keypoints[i].pt for i in matches], dtype=np.float64)
            ok, rvec, translation, inliers = cv2.solvePnPRansac(obj, uv, camera, None,
                iterationsCount=80, reprojectionError=2.5, flags=cv2.SOLVEPNP_AP3P)
            if not ok or inliers is None or len(inliers) < max(12, .5 * len(uv)):
                continue
            if np.any(np.ptp(uv[inliers[:, 0]], axis=0) < .15 * np.asarray(rgb.shape[1::-1])):
                continue
            rotation = cv2.Rodrigues(rvec)[0]
            if not np.isfinite(translation).all() or np.any((obj[inliers[:, 0]] @ rotation.T + translation.T)[:, 2] <= 0):
                continue
            ref_from_camera = np.eye(4)
            ref_from_camera[:3, :3] = rotation.T
            ref_from_camera[:3, 3] = (-rotation.T @ translation).ravel()
            return ref['pose'] @ ref_from_camera
        return None
