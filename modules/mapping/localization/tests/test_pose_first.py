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
            def __init__(self, *args):
                self.items = iter([(1, 0., rgb, 0., (32, 32), {'timestamp_s': 0., 'tracking_state': 1}),
                    (2, .2, rgb, 0., (32, 32), {'timestamp_s': .2, 'tracking_state': 2, 'map_id': 0,
                                             'T_cw': matrix.tolist(), 'depth_anchors': anchors})])
            def take(self):
                return next(self.items, None)
            def close(self):
                pass
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

    def test_inconsistent_sparse_depths_do_not_scatter_the_accumulated_map(self):
        depth = np.full((32, 32), 2., np.float32)
        xy = [(x, y) for x in (4, 12, 20, 28) for y in (4, 12, 20, 28)]
        anchors = [[x, y, 3. if i < 12 else 300.] for i, (x, y) in enumerate(xy)]
        aligned, scale = depth_in_pose_units(depth, anchors, (32, 32))
        self.assertEqual(scale, 1.5)
        np.testing.assert_allclose(aligned, 3.)
        unstable = [[x, y, 1. if i < 8 else 20.] for i, (x, y) in enumerate(xy)]
        self.assertIsNone(depth_in_pose_units(depth, unstable, (32, 32)))

    def test_live_preview_keeps_old_points_and_path_after_motion_and_tracking_loss(self):
        rgb = np.full((256, 256, 3), 120, np.uint8)
        anchors = [[x, y, 3.] for x in (40, 80, 120, 160) for y in (40, 80)]
        class Estimator:
            def estimate(self, image):
                return np.full(image.shape[:2], 2., np.float32)
        class Frames:
            dropped = 0
            def __init__(self, *args):
                self.items = []
                # Exceed both the old 10-second and 40,000-voxel limits.
                for i in range(32):
                    stamp = i * 20.
                    matrix = np.eye(4); matrix[0, 3] = -i * 1000.
                    self.items.append((i + 1, stamp, rgb, 0., (256, 256),
                        {'timestamp_s': stamp, 'tracking_state': 2, 'map_id': 0,
                         'T_cw': matrix.tolist(), 'depth_anchors': anchors}))
                self.items.append((33, 700., rgb, 0., (256, 256),
                    {'timestamp_s': 700., 'tracking_state': 3, 'map_id': 0}))
                self.items = iter(self.items)
            def take(self):
                return next(self.items, None)
            def close(self):
                pass
        snapshots = []
        with tempfile.TemporaryDirectory() as folder, patch('modules.mapping.data.stream_mapping.PoseFrames', Frames):
            output = Path(folder) / 'result'
            run('unused', output, {'frame_fps': 5, 'sample_fps': 5},
                estimator_factory=Estimator, on_preview=snapshots.append)
            last = snapshots[-1]
            self.assertEqual(last['tracking'], 'lost')
            self.assertIsNone(last['render_window_s'])
            self.assertEqual(last['render_evicted_points'], 0)
            self.assertLessEqual(last['map_points'], 8000)
            self.assertGreater(last['rescue']['point_fusion']['coarsenings'], 0)
            self.assertLessEqual(len(last['points']), 12000)
            self.assertEqual(last['cameras'][0], [0., 0., 0.])
            self.assertEqual(last['cameras'][-1], [31000., 0., 0.])
            self.assertLess(min(p[0] for p in last['points']), 10.)
            self.assertGreater(max(p[0] for p in last['points']), 30990.)
            self.assertEqual(last['points'], snapshots[-2]['points'])
            scene = json.loads((output / 'scene.json').read_text())
            self.assertEqual(len(scene['points']), last['map_points'])

    def test_timed_preview_freezes_last_valid_map_until_tracking_recovers(self):
        rgb = np.full((32, 32, 3), 120, np.uint8)
        anchors = [[x, y, 3.] for x in (8, 16, 24, 28) for y in (8, 16)]
        class Estimator:
            def estimate(self, image):
                return np.full(image.shape[:2], 2., np.float32)
        class Frames:
            dropped = 0
            def __init__(self, *args):
                self.items = iter((i + 1, stamp, rgb, 0., (32, 32),
                    {'timestamp_s': stamp, 'tracking_state': state, 'map_id': 0,
                     'T_cw': np.eye(4).tolist(), 'depth_anchors': anchors})
                    for i, (stamp, state) in enumerate([(14.578, 2), (20., 3), (24.688, 3), (40., 3), (41., 2)]))
            def take(self):
                return next(self.items, None)
            def close(self):
                pass
        snapshots = []
        with tempfile.TemporaryDirectory() as folder, patch('modules.mapping.data.stream_mapping.PoseFrames', Frames):
            run('unused', Path(folder) / 'result', {'frame_fps': 5, 'sample_fps': 5, 'render_window_s': 10},
                estimator_factory=Estimator, on_preview=snapshots.append)
        self.assertTrue(snapshots[0]['points'])
        for lost in snapshots[1:4]:
            self.assertEqual(lost['tracking'], 'lost')
            self.assertEqual(lost['points'], snapshots[0]['points'])
            self.assertEqual(lost['cameras'], snapshots[0]['cameras'])
            self.assertEqual(lost['render_evicted_points'], 0)
        self.assertEqual(snapshots[-1]['tracking'], 'tracking')
