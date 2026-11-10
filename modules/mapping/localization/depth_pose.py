"""Causal registration of camera poses using previously aligned RGB-D frames."""

from collections import deque

import cv2
import numpy as np

from .pose_first import ConstantVelocityPose


class VisualPoseReferences:
    """Recover a camera pose from previously aligned RGB-D observations."""

    def __init__(self, interval=.4):
        self.interval = interval
        self.references = deque(maxlen=4)
        self.features = cv2.ORB_create(nfeatures=650, fastThreshold=12)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

    def remember(self, stamp, rgb, depth, params, pose, excluded):
        if self.references and stamp - self.references[-1]['stamp'] < self.interval:
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

    def relocalize(self, rgb, params):
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
            for pair in pairs:
                if len(pair) < 2:
                    continue
                first, second = pair
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


class VideoMapPose:
    """Fix video poses from the native tracker or prior observations before inferring depth."""

    map_id = 0

    def __init__(self):
        self.references = VisualPoseReferences(interval=.15)
        self.motion = ConstantVelocityPose()
        self.last_rgb = self.last_pose = None
        self.native_map = self.native_to_map = None
        self.scale = None

    def _bind(self, pose, native_pose, native_map, factor):
        rotation = pose[:3, :3] @ native_pose[:3, :3].T
        offset = pose[:3, 3] - factor * (rotation @ native_pose[:3, 3])
        self.native_to_map = (factor, rotation, offset)
        self.native_map = native_map

    def _to_map(self, pose):
        factor, rotation, offset = self.native_to_map
        mapped = np.eye(4)
        mapped[:3, :3] = rotation @ pose[:3, :3]
        mapped[:3, 3] = offset + factor * (rotation @ pose[:3, 3])
        return mapped

    def resolve(self, stamp, rgb, params, native_pose, native_map, dpvo_fallback=None):
        needs_origin = self.native_to_map is None or (self.last_pose is None and native_map != self.native_map)
        if native_pose is not None and needs_origin:
            # Define the world axes at the first confirmed camera pose.
            self._bind(np.eye(4), native_pose, native_map, 1.)
        if native_pose is not None and self.native_to_map is not None and native_map == self.native_map:
            return self._to_map(native_pose), 'tracking'
        if self.last_pose is None:
            return None, 'initializing'
        if dpvo_fallback is not None and self.scale is not None:
            pose, aligned_map = dpvo_fallback
            if aligned_map == self.native_map:
                return self._to_map(pose), 'pose_dpvo'

        pose = self.last_pose.copy() if np.array_equal(self.last_rgb, rgb) else self.references.relocalize(rgb, params)
        if pose is not None:
            return pose, 'pose_reused'
        if native_map == self.native_map or native_pose is None:
            prediction, _ = self.motion.update(dict(timestamp_s=stamp, tracking_state=3, map_id=self.map_id), stamp)
            if prediction is not None:
                return prediction, 'pose_predicted'
        return self.last_pose.copy(), 'pose_held'

    def accept_depth(self, stamp, rgb, depth, params, excluded, pose, state, native_pose, native_map, native_scale):
        if state == 'tracking':
            if native_scale is not None:
                self.scale = self.native_to_map[0] * native_scale
            else:
                state = 'scale_reused'
        elif state == 'pose_reused' and native_pose is not None and native_scale is not None and native_map != self.native_map:
            self._bind(pose, native_pose, native_map, self.scale / native_scale)
        if self.scale is None:
            return None, 'scale_wait'

        if state in {'tracking', 'pose_reused'}:
            self.references.remember(stamp, rgb, depth * self.scale, params, pose, excluded)
        if state in {'tracking', 'pose_reused', 'scale_reused'}:
            self.motion.update(dict(timestamp_s=stamp, tracking_state=2, map_id=self.map_id, T_cw=np.linalg.inv(pose)), stamp)
            self.last_rgb, self.last_pose = rgb, pose
        return self.scale, state
