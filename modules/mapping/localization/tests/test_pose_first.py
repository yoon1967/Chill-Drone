"""A pose must precede depth, and depth may never change the native pose."""
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np
from modules.mapping.localization.pose_first import fixed_camera_pose, depth_in_pose_units
from modules.mapping.data.stream_mapping import run


class PoseFirstTests(unittest.TestCase):
    def test_only_successful_exact_frame_rigid_pose_is_admitted(self):
        native = {'timestamp_s': 1., 'tracking_state': 2, 'T_cw': np.eye(4).tolist()}
        np.testing.assert_equal(fixed_camera_pose(native, 1.), np.eye(4))
        self.assertIsNone(fixed_camera_pose(native, 2.))
        native['tracking_state'] = 1
        self.assertIsNone(fixed_camera_pose(native, 1.))
        native['tracking_state'] = 2
        native['T_cw'][0][0] = 2.
        self.assertIsNone(fixed_camera_pose(native, 1.))

    def test_only_depth_scale_changes_to_match_pose_coordinates(self):
        depth = np.full((32, 32), 2., dtype=np.float32)
        anchors = [[x, y, 3.] for x in (8, 16, 24, 28) for y in (8, 16)]
        aligned, scale = depth_in_pose_units(depth, anchors, (32, 32))
        self.assertEqual(scale, 1.5)
        np.testing.assert_equal(aligned, np.full((32, 32), 3.))
        np.testing.assert_equal(depth, np.full((32, 32), 2.))
        self.assertIsNone(depth_in_pose_units(depth, anchors[:3], (32, 32)))

    def test_unconfirmed_frames_do_not_run_depth_or_enter_map(self):
        calls = []
        class Estimator:
            def estimate(self, rgb):
                calls.append(rgb.shape)
                return np.full(rgb.shape[:2], 2., dtype=np.float32)
        matrix = np.eye(4); matrix[0, 3] = -1
        rgb = np.full((32, 32, 3), 120, dtype=np.uint8)
        anchors = [[x, y, 3.] for x in (8, 16, 24, 28) for y in (8, 16)]
        class Frames:
            dropped = 0
            stop = threading.Event()
            thread = type('Thread', (), {'start': lambda self: None, 'join': lambda self, timeout: None})()
            def __init__(self, *args):
                self.items = iter([(1, 0., rgb, 0., (32, 32), {'timestamp_s': 0., 'tracking_state': 1}),
                    (2, .2, rgb, 0., (32, 32), {'timestamp_s': .2, 'tracking_state': 2, 'map_id': 0,
                                             'T_cw': matrix.tolist(), 'depth_anchors': anchors})])
            def take(self):
                return next(self.items, None)
        with tempfile.TemporaryDirectory() as folder, patch('modules.mapping.data.stream_mapping.PoseFrames', Frames):
            output = Path(folder) / 'result'
            run('unused', output, {'frame_fps': 5, 'sample_fps': 5}, estimator_factory=Estimator)
            scene = json.loads((output / 'scene.json').read_text())
            # Warmup + the confirmed second frame, never the initializing first frame.
            self.assertEqual(len(calls), 2)
            self.assertEqual(len(scene['cameras']), 1)
            self.assertEqual(scene['cameras'][0]['frame_id'], 'frame_0000002.jpg')
            np.testing.assert_equal(scene['cameras'][0]['position'], [1., 0., 0.])
            self.assertEqual(scene['pose_source'], 'orb_slam3_depth')
            self.assertEqual(scene['scale']['unit'], 'arbitrary')
            np.testing.assert_equal(matrix[:3, 3], [-1., 0., 0.])
