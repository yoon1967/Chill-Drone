import io
import unittest
from unittest.mock import MagicMock

import cv2
import numpy as np

from modules.mapping.localization.dpvo_pose import DpvoAlignment, DpvoPoseTracker


def pose(position, yaw):
    value = np.eye(4)
    value[:3, 3] = position
    value[:3, :3] = cv2.Rodrigues(np.array([0., yaw, 0.]))[0]
    return value


class DpvoAlignmentTests(unittest.TestCase):
    def test_recovers_rotation_translation_and_scale_until_the_overlap_expires(self):
        alignment = DpvoAlignment()
        rotation = pose([0, 0, 0], .4)[:3, :3]
        offset = np.array([4., 1., -2.])
        for stamp in (0., .1, .2):
            origin = np.array([stamp, 0., stamp / 2])
            alignment.observe(stamp, (0, 0), pose(3 * (rotation @ origin) + offset, stamp + .4),
                              pose(origin, stamp))
        projected, map_id = alignment.project(.3, pose([.3, 0., .15], .3))
        np.testing.assert_allclose(projected, pose(3 * (rotation @ [.3, 0., .15]) + offset, .7), atol=1e-10)
        self.assertEqual(map_id, (0, 0))
        self.assertIsNone(alignment.project(.3, pose([10., 0., 0.], .3)))
        self.assertIsNone(alignment.project(.3, pose([.3, 0., .15], 2.)))
        self.assertIsNone(alignment.project(1.21, pose([.3, 0., .15], .3)))
        alignment.observe(.31, (0, 1), pose([.3, 0., .15], .3), pose([.3, 0., .15], .3))
        self.assertIsNone(alignment.project(.4, pose([.4, 0., .2], .4)))

    def test_rotation_only_and_disagreeing_motion_cannot_set_scale(self):
        for direction in (0, -1):
            with self.subTest(direction=direction):
                alignment = DpvoAlignment()
                for stamp in (0., .1, .2):
                    alignment.observe(stamp, (0, 0), pose([stamp, 0, 0], stamp),
                                      pose([direction * stamp, 0, 0], stamp))
                self.assertIsNone(alignment.project(.3, pose([.3, 0, 0], .3)))

    def test_pose_transport_rejects_stale_pose_and_converts_world_to_camera(self):
        tracker = DpvoPoseTracker.__new__(DpvoPoseTracker)
        tracker.process = MagicMock(stdin=io.BytesIO())
        tracker._reply = MagicMock(return_value=dict(timestamp_s=.1, tracking_state=2, T_cw=pose([-2, 0, 0], 0)))
        image = np.zeros((32, 32, 3), np.uint8)
        with self.assertRaisesRegex(RuntimeError, 'timestamp_mismatch'):
            tracker.track(image, .2)
        np.testing.assert_allclose(tracker.track(image, .1), pose([2, 0, 0], 0))
