"""Calibrated multi-view scenes exercise the production map publication gate."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from modules.mapping.localization.landmarks import confirm_landmarks, UnconfirmedGeometry
from modules.mapping.localization.orbslam3 import convert, SessionPublisher


CAMERA = dict(width=640, height=480, fx=500., fy=500., cx=320., cy=240.,
              distortion=[.08, -.02, .001, -.001], calibration_status="synthetic")


def snapshot(centers=(0., .3, .6), xyz=None, scale=1.):
    xyz = xyz or [[.1, .2, 5.], [.16, .2, 5.]]
    points = [{"id": 50 + i, "xyz": (np.array(p) * scale).tolist(), "observations": 999} for i, p in enumerate(xyz)]
    K = np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]])
    frames = []
    for i, center in enumerate(centers):
        # Rotation and distortion both matter for prediction in another camera.
        rvec = np.array([0., .03 * i, 0.])
        R = cv2.Rodrigues(rvec)[0]
        t = -R @ np.array([center * scale, 0., 0.])
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, t
        xy = cv2.projectPoints(np.array([p["xyz"] for p in points]), rvec, t, K, np.array(CAMERA["distortion"]))[0].reshape(-1, 2)
        frames.append({"id": i, "frame_id": f"orb_{i:08d}.jpg", "timestamp_s": float(i),
                       "timestamp_source": "source_pts", "T_cw": T.tolist(),
                       "observations": [{"point_id": p["id"], "xy": v.tolist(), "sigma2": 1.}
                                        for p, v in zip(points, xy)]})
    return {"schema_version": "orb-slam3/1.0", "revision": 1, "map_id": 7,
            "map_change_index": 0, "big_change_index": 0, "final": False,
            "points": points, "keyframes": frames}


class LandmarkTests(unittest.TestCase):
    def test_view_changes_keep_native_ids_and_distinct_nearby_points(self):
        native = snapshot()
        original = deepcopy(native)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "frames").mkdir()
            for frame in native["keyframes"]:
                cv2.imwrite(str(root / "frames" / frame["frame_id"]), np.full((480, 640, 3), 128, np.uint8))
            result = convert(native, CAMERA, {"frames_received": 3}, root)
        self.assertEqual([p[0] for p in result["points"]], [50, 51])
        self.assertEqual([p[8] for p in result["points"]], [3, 3])
        self.assertEqual(result["point_observations"]["frame_index_by_point"], {"50": [0, 1, 2], "51": [0, 1, 2]})
        self.assertLess(result["quality"]["mean_reprojection_error_px"], 1e-9)
        self.assertEqual(native, original)

    def test_wrong_observation_is_excluded_from_support(self):
        native = snapshot(centers=(0., .3, .6, .9))
        native["keyframes"][0]["observations"][0]["xy"][0] += 40
        filtered, _, quality = confirm_landmarks(native, CAMERA)
        self.assertEqual([p["observations"] for p in filtered["points"]], [3, 4])
        self.assertEqual(quality["rejected_observations"], 1)
        self.assertEqual([o["point_id"] for o in filtered["keyframes"][0]["observations"]], [51])

    def test_two_views_cannot_be_inflated_by_native_count_or_duplicates(self):
        native = snapshot(centers=(0., .3))
        for frame in native["keyframes"]:
            frame["observations"] *= 3
        filtered, _, quality = confirm_landmarks(native, CAMERA)
        self.assertEqual(filtered["points"], [])
        self.assertEqual(quality["insufficient_views"], 2)

    def test_repeated_camera_centers_and_small_baseline_do_not_confirm_depth(self):
        for centers in ((0., 0., 0.), (0., .0001, .0002)):
            with self.subTest(centers=centers):
                filtered, _, quality = confirm_landmarks(snapshot(centers), CAMERA)
                self.assertEqual(filtered["points"], [])
                self.assertEqual(quality["insufficient_parallax"], 2)

    def test_scale_invariance(self):
        for scale in (.001, 1., 1000.):
            with self.subTest(scale=scale):
                filtered, errors, _ = confirm_landmarks(snapshot(scale=scale), CAMERA)
                self.assertEqual([p["id"] for p in filtered["points"]], [50, 51])
                self.assertLess(max(errors.values()), 1e-9)

    def test_pair_angle_is_checked_when_first_view_sits_between_other_views(self):
        # Each endpoint is <1 degree from the first view, but endpoints differ >1.
        filtered, _, _ = confirm_landmarks(snapshot(centers=(0., -.06, .06)), CAMERA)
        self.assertEqual([p["id"] for p in filtered["points"]], [50, 51])

    def test_duplicate_keyframe_ids_and_nonrigid_poses_are_invalid(self):
        native = snapshot()
        native["keyframes"][1]["id"] = native["keyframes"][0]["id"]
        with self.assertRaisesRegex(ValueError, "duplicate_native_keyframe"):
            confirm_landmarks(native, CAMERA)
        native = snapshot()
        native["keyframes"][0]["T_cw"][0][0] = 2.
        with self.assertRaisesRegex(ValueError, "invalid_native_camera_pose"):
            confirm_landmarks(native, CAMERA)

    def test_negative_depth_nonfinite_and_wrong_geometry_are_rejected(self):
        native = snapshot(xyz=[[.1, .2, -5.], [.16, .2, 5.]])
        native["points"][1]["xyz"][0] = float("nan")
        filtered, _, quality = confirm_landmarks(native, CAMERA)
        self.assertEqual(filtered["points"], [])
        self.assertEqual(quality["invalid_points"], 1)
        native = snapshot()
        for frame in native["keyframes"]:
            frame["observations"][0]["xy"][0] += 10
        filtered, _, _ = confirm_landmarks(native, CAMERA)
        self.assertEqual([p["id"] for p in filtered["points"]], [51])

    def test_pyramid_noise_is_accounted_for(self):
        native = snapshot()
        for frame in native["keyframes"]:
            frame["observations"][0].update(sigma2=4.)
            frame["observations"][0]["xy"][0] += 3
        filtered, _, _ = confirm_landmarks(native, CAMERA)
        self.assertEqual([p["id"] for p in filtered["points"]], [50, 51])
        for frame in native["keyframes"]:
            frame["observations"][0]["sigma2"] = 1.
        filtered, _, _ = confirm_landmarks(native, CAMERA)
        self.assertEqual([p["id"] for p in filtered["points"]], [51])

    def test_publisher_waits_then_publishes_and_preserves_last_valid_map(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "revisions").mkdir()
            (root / "frames").mkdir()
            publisher = SessionPublisher(root, CAMERA)
            for revision, centers, expected in ((1, (0., .3), 0), (2, (0., .3, .6), 2), (3, (0.,), 2)):
                native = snapshot(centers)
                native["revision"] = revision
                for frame in native["keyframes"]:
                    cv2.imwrite(str(root / "frames" / frame["frame_id"]), np.zeros((480, 640, 3), np.uint8))
                (root / "native.json").write_text(json.dumps(native), encoding="utf-8")
                (root / "tracking.json").write_text('{"tracking_state":2}', encoding="utf-8")
                publisher.update()
                self.assertEqual(publisher.revision, expected)
                self.assertEqual(publisher.consumed_revision, revision)
                publisher.update()
            self.assertTrue(publisher.state["map_available"])
            self.assertEqual(publisher.state["revision_path"], "revisions/000002")
            self.assertEqual(publisher.state["mapping_state"], "confirming_geometry")
            self.assertEqual(len(publisher.entries), 1)


if __name__ == "__main__":
    unittest.main()
