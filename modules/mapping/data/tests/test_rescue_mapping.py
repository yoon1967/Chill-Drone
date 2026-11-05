import json
from pathlib import Path
import tempfile
import queue
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from modules.mapping.data.rescue_mapping import (mapping_options, validate_semantics,
    smooth_structural_depth, SemanticSurface, PersonSightings, display_coordinates)
from modules.mapping.data.mapping_history import HistoryReader
from modules.mapping.data.mapping_limits import MAX_POINT_BUDGET, MAX_PREVIEW_POINTS
from modules.mapping.data.stream_mapping import run
from modules.mapping.localization.pose_first import depth_in_pose_units


class RescueMappingTests(unittest.TestCase):
    def test_depth_sampling_rate_is_validated_without_restricting_legacy_sfm(self):
        from modules.mapping.data.rescue_mapping import mapping_sample_fps
        from modules.mapping.data.frame_input import validate_options
        self.assertEqual(mapping_sample_fps({}), 5)
        for rate in (.1, .25, 2.5, 12, 15):
            self.assertEqual(mapping_sample_fps({'sample_fps': rate}), rate)
            self.assertEqual(validate_options({'confirmed': True, 'mode': 'depth_rgbd', 'sample_fps': rate}, {})['sample_fps'], rate)
        for rate in (True, '5', 0, 15.1, float('nan'), float('inf')):
            with self.subTest(rate=rate), self.assertRaisesRegex(ValueError, 'invalid_sample_fps'):
                mapping_sample_fps({'sample_fps': rate})
        self.assertEqual(validate_options({'confirmed': True, 'mode': 'sfm', 'sample_fps': 30}, {})['sample_fps'], 30)

    def test_settings_validate_and_keep_legacy_clients_compatible(self):
        self.assertFalse(mapping_options({})['semantic_mapping'])
        self.assertEqual(mapping_options({'semantic_mapping': True})['depth_resolution'], 384)
        self.assertEqual(mapping_options({})['person_color'], '#ff3030')
        for value in ({'person_color': 'red'}, {'person_outline': 1}, {'depth_resolution': 0},
                      {'semantic_mapping': 'true'}, {'live_policy': 'guess'}, {'point_budget': MAX_POINT_BUDGET + 1},
                      {'point_budget': True}, {'point_budget': 0}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                mapping_options(value)

    def test_rejects_mask_from_another_frame_or_geometry(self):
        value = dict(frame_id='a', timestamp_s=0., labels=np.zeros((8, 8), np.uint8),
                     confidence=np.ones((8, 8), np.float32), people=[])
        with self.assertRaisesRegex(ValueError, 'frame_mismatch'):
            validate_semantics(value, (8, 8), 'b', 0.)
        with self.assertRaisesRegex(ValueError, 'geometry_mismatch'):
            validate_semantics(value, (16, 8), 'a', 0.)

    def test_people_cannot_set_the_static_map_scale(self):
        depth = np.ones((20, 30), np.float32)
        excluded = np.zeros_like(depth, bool); excluded[:, 15:] = True
        anchors = [[x, y, 3.] for x in (2, 5, 8, 11) for y in (3, 6)]
        anchors += [[x, y, 30.] for x in (17, 19, 21, 23, 25) for y in (3, 6)]
        _, scale = depth_in_pose_units(depth, anchors, (30, 20), excluded)
        self.assertEqual(scale, 3.)

    def test_structural_smoothing_never_fills_holes_or_crosses_person_edges(self):
        depth = np.full((12, 16), 2., np.float32); depth[:, 8:] = 1.
        labels = np.full(depth.shape, 2, np.uint8); labels[:, 8:] = 1
        depth[3, 3] = 0; depth[6, 6] = 2.04
        result = smooth_structural_depth(depth, labels)
        self.assertEqual(result[3, 3], 0)
        np.testing.assert_array_equal(result[:, 8:], depth[:, 8:])
        self.assertLess(result[6, 6], depth[6, 6])

    def fixture(self):
        shape = (64, 80)
        rgb = np.full((*shape, 3), [120, 140, 160], np.uint8)
        depth = np.full(shape, 2., np.float32)
        mask = np.zeros(shape, bool); mask[12:54, 30:50] = True
        depth[mask] = 1.4
        labels = np.full(shape, 2, np.uint8); labels[mask] = 1
        return rgb, depth, mask, labels, (70, 70, 40, 32)

    def test_point_fusion_reuses_static_cells_and_excludes_people(self):
        rgb, depth, mask, labels, params = self.fixture()
        volume = SemanticSurface(.04)
        try:
            for _ in range(3):
                volume.integrate(rgb, depth, np.eye(4), params, labels, np.ones_like(depth))
            self.assertGreater(len(volume.points()), 100)
            self.assertTrue(all(p[2] > 1.8 for p in volume.points()))
            self.assertNotIn(1, volume.labels())
            before = volume.points()
            self.assertIs(before, volume.points())
            volume.integrate(rgb, depth, np.eye(4), params, labels, np.ones_like(depth))
            self.assertEqual(len(before), len(volume.points()))
            self.assertFalse(hasattr(volume, 'mesh'))
        finally:
            volume.close()

    def test_person_voxels_become_explicit_last_seen_markers(self):
        _, depth, mask, _, params = self.fixture()
        people = PersonSightings(.06)
        first = people.update([dict(mask=mask, confidence=.9)], depth, np.eye(4), params, 0., 'a')
        self.assertTrue(first[0]['voxels']); self.assertTrue(first[0]['observed'])
        second = people.update([dict(mask=mask, confidence=.9)], depth, np.eye(4), params, .2, 'b')
        self.assertEqual(first[0]['id'], second[0]['id'])
        stale = people.snapshot(3.)[0]
        self.assertFalse(stale['observed']); self.assertEqual(stale['voxels'], [])
        self.assertEqual(stale['source_frame_id'], 'b')

    def test_rescue_pipeline_history_and_saved_coordinates(self):
        rgb, _, mask, labels, _ = self.fixture()
        class Depth:
            def estimate(self, image):
                return np.full(image.shape[:2], 2., np.float32)
        class Semantics:
            def estimate(self, image, frame_id, stamp):
                import cv2
                m = cv2.resize(mask.astype(np.uint8), (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST).astype(bool)
                l = np.full(image.shape[:2], 2, np.uint8); l[m] = 1
                return dict(frame_id=frame_id, timestamp_s=stamp, labels=l, confidence=np.ones(l.shape, np.float32),
                            people=[dict(mask=m, confidence=.95)], inference_ms=1.)
        class Tracker:
            def __init__(self, *args): pass
            def track(self, image, stamp):
                h, w = image.shape[:2]
                return dict(timestamp_s=stamp, tracking_state=2, map_id=0, T_cw=np.eye(4).tolist(),
                    depth_anchors=[[w*x, h*y, 3.] for x in (.1,.2,.7,.8) for y in (.3,.6)])
            def close(self): pass
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / 'result'
            frames = [(i+1, i*.2, rgb, (80, 64)) for i in range(3)]
            with patch('modules.mapping.data.stream_mapping.frames', return_value=iter(frames)):
                run('unused', output, dict(frame_fps=5, sample_fps=5, semantic_mapping=True, depth_resolution=256),
                    estimator_factory=Depth, tracker_factory=Tracker, semantic_factory=Semantics)
            replay = HistoryReader(output).snapshot(3)
            self.assertTrue(replay['rescue']['people'][0]['voxels'])
            self.assertEqual(replay['rescue']['render_mode'], 'points')
            self.assertEqual(len(replay['rescue']['point_labels']), len(replay['points']))
            self.assertNotIn('mesh', replay['rescue'])
            saved = json.loads((output / 'scene.json').read_text())
            self.assertNotIn('meshes', saved['rescue'])
            self.assertLessEqual(len(saved['points']), MAX_PREVIEW_POINTS)
            self.assertEqual(len(saved['rescue']['point_labels']), len(saved['points']))
            self.assertEqual(saved['rescue']['person_color'], '#ff3030')
            self.assertTrue(any(point[4:7] == [255, 48, 48] for point in saved['points']))
            self.assertLess(saved['rescue']['people'][0]['center'][2], 0)
            self.assertGreater(replay['rescue']['people'][0]['center'][2], 0)
            with np.load(output / 'history/frames/00000003.npz') as frame:
                self.assertIn('semantic_labels', frame)
                self.assertEqual(len(frame['person_instances']), 1)

    def test_long_trajectory_stays_bounded_without_discarding_older_regions(self):
        rgb, depth, _, labels, params = self.fixture()
        volume = SemanticSurface(.04, max_cells=1000)
        pose = np.eye(4)
        for i in range(400):
            pose[0, 3] = (i-200)*.1
            volume.integrate(rgb, depth, pose, params, labels, np.ones_like(depth))
            self.assertLessEqual(len(volume.cells), 1000)
        self.assertGreater(volume.coarsenings, 0)
        points = np.asarray(volume.points())
        self.assertLess(points[:,0].min(), -18)
        self.assertGreater(points[:,0].max(), 18)
        self.assertEqual(volume.frames, 400)
        self.assertEqual(len(points), len(volume.labels()))
        self.assertEqual(len(np.unique(np.floor(points[:,:3]/volume.size).astype(int),axis=0)), len(points))

    def test_live_pose_tracking_continues_while_mapping_holds_its_selected_frame(self):
        import cv2
        from modules.mapping.data.stream_mapping import LivePoseFrames
        received, tracked = queue.Queue(), []
        jpeg = cv2.imencode('.jpg', np.full((48, 64, 3), 120, np.uint8))[1].tobytes()
        def receive(): return received.get(timeout=2)
        class Tracker:
            def __init__(self, *args): pass
            def track(self, rgb, stamp):
                tracked.append(stamp)
                return dict(timestamp_s=stamp, T_cw=np.eye(4).tolist())
            def close(self): pass
        producer = LivePoseFrames(receive, 15, 15, Path('unused'),
            dict(width=64, height=48), Tracker, continuous=True)
        try:
            received.put((jpeg, 0., 0))
            selected = producer.take()
            for i in range(1, 6): received.put((jpeg, i*.1, i))
            received.put(None)
            deadline = time.monotonic()+2
            while len(tracked)<6 and time.monotonic()<deadline: time.sleep(.01)
            self.assertEqual(len(tracked), 6)
            self.assertEqual(selected[1], 0., 'a selected frame must never be replaced')
            self.assertEqual(producer.take()[1], .5)
            self.assertGreater(producer.sampled_out, 0)
            self.assertIsNone(producer.take())
        finally: producer.close()


if __name__ == '__main__': unittest.main()
