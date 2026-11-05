import unittest
import tempfile
from pathlib import Path

import numpy as np

from modules.mapping.data.point_fusion import SemanticSurface
from modules.mapping.data.mapping_limits import DEFAULT_POINT_BUDGET
from modules.mapping.data.mapping_history import HistoryWriter, HistoryReader
from modules.mapping.data.stream_mapping import write_json


class PointFusionTests(unittest.TestCase):
    def surface(self, points, *, size=.04, step=.005, budget=16):
        surface = SemanticSurface(size, spacing_increment=step, max_cells=budget)
        xyz = np.asarray(points, np.float64)
        weights = np.ones(len(xyz))
        votes = np.zeros((len(xyz), 9)); votes[:, 2] = 1
        surface._merge(np.floor(xyz / size).astype(np.int64), xyz, np.full_like(xyz, 120), weights, votes,
                       np.ones(len(xyz), np.uint8))
        return surface

    def test_compaction_uses_constant_steps_and_regrids_negative_centroids(self):
        points = [[x*.041, y*.041, 1.] for x in range(-8, 9) for y in range(-3, 4)]
        surface = self.surface(points, budget=80)
        surface.compact()
        self.assertGreater(surface.coarsenings, 0)
        self.assertAlmostEqual(surface.size, .04 + .005 * surface.coarsenings)
        self.assertLessEqual(len(surface.cells), 80)
        np.testing.assert_array_equal(surface.keys, np.floor(surface.xyz / surface.size).astype(np.int64))
        first = surface.coarsenings
        surface.compact(40)
        self.assertGreater(surface.coarsenings, first)
        self.assertAlmostEqual(surface.size, .04 + .005 * surface.coarsenings)
        self.assertEqual(surface.metadata()['spacing_increment'], .005)

    def test_points_do_not_silently_restore_the_old_12000_point_limit(self):
        surface = self.surface([[i*.05, 0, 1] for i in range(15000)], budget=16000)
        self.assertEqual(len(surface.points()), 15000)
        self.assertEqual(surface.coarsenings, 0)

    def test_increment_must_be_positive_and_finite(self):
        for value in (0, -1, float('nan'), float('inf')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                SemanticSurface(.04, spacing_increment=value)

    def test_spacing_is_calibrated_once_from_the_first_static_observation(self):
        rgb = np.zeros((60, 60, 3), np.uint8)
        labels = np.zeros((60, 60), np.uint8)
        confidence = np.ones((60, 60), np.float32)
        depth = np.full((60, 60), 2., np.float32)
        surface = SemanticSurface(.08, max_cells=500)
        params = (180, 180, 30, 30)
        surface.integrate(rgb, np.zeros_like(depth), np.eye(4), params, labels, confidence)
        self.assertIsNone(surface.spacing_increment)
        surface.integrate(rgb, depth, np.eye(4), params, labels, confidence)
        expected = (3 * 2 / 180) * np.sqrt(10 * 400 / 500) / 4
        self.assertAlmostEqual(surface.spacing_increment, expected)
        for shift in range(1, 12):
            pose = np.eye(4)
            pose[0, 3] = shift * .5
            surface.integrate(rgb, depth * 2, pose, params, labels, confidence)
            self.assertLessEqual(len(surface.cells), 500)
        self.assertGreater(surface.coarsenings, 0)
        self.assertAlmostEqual(surface.spacing_increment, expected)
        self.assertAlmostEqual(surface.size, .08 + surface.coarsenings * expected)

    def test_preview_keeps_rounded_coordinates_and_integer_clipped_colors(self):
        surface = self.surface([[-.123456, .234567, 1.345678]])
        surface.rgb[:] = [-3., 128.6, 260.]
        self.assertEqual(surface.points(), [[-.12346, .23457, 1.34568, 0, 129, 255]])
        self.assertTrue(all(type(c) is int for c in surface.points()[0][3:]))
        self.assertIs(surface.points(), surface.points(), 'unchanged previews reuse the cached rows')

    def test_tof_refines_only_nearby_visual_voxels_and_keeps_evidence_after_merging(self):
        def ray(u, v, z):
            return [(u - 24) * z / 40, (v - 24) * z / 40, z]

        near, far, outside = ray(9, 15, 2.09), ray(15, 15, 2.4), ray(6, 15, 2.09)
        surface = self.surface([near, far, outside], size=.05, budget=500)
        depth = np.zeros((48, 48), np.float32)
        depth[9:27, 9:27] = 2.
        rgb = np.zeros((48, 48, 3), np.uint8)
        labels, confidence = np.zeros_like(depth, np.uint8), np.ones_like(depth)
        params = (40, 40, 24, 24)
        surface.integrate(rgb, depth, np.eye(4), params, labels, confidence, tof_verified=True)
        visual = surface.xyz[~surface.tof_verified]
        self.assertTrue(any(np.allclose(point, far) for point in visual))
        self.assertTrue(any(np.allclose(point, outside) for point in visual))
        moved = min(visual, key=lambda point: np.linalg.norm(point - near))
        self.assertLess(moved[2], near[2])
        self.assertGreater(moved[2], 2.)
        self.assertGreater(surface.tof_verified.sum(), 0)
        np.testing.assert_allclose(surface.xyz[surface.tof_verified, 2], 2.)
        surface.integrate(rgb, depth * 1.01, np.eye(4), params, labels, confidence)
        np.testing.assert_allclose(surface.xyz[surface.tof_verified, 2], 2.)
        np.testing.assert_array_equal(surface.keys, np.floor(surface.xyz / surface.size).astype(np.int64))

    def test_predicted_observation_adds_coverage_and_native_vision_wins_overlap(self):
        surface = self.surface([[0., 0., 2.01]], size=.1, budget=32)
        depth = np.zeros((12, 12), np.float32)
        rgb = np.zeros((12, 12, 3), np.uint8)
        labels, confidence = np.zeros_like(depth, np.uint8), np.ones_like(depth)
        params = (40, 40, 3, 3)
        depth[3, 3] = 2.08
        surface.integrate(rgb, depth, np.eye(4), params, labels, confidence, pose_predicted=True)
        np.testing.assert_allclose(surface.xyz[:, 2], [2.01])
        depth[3, 3] = 2.42
        surface.integrate(rgb, depth, np.eye(4), params, labels, confidence, pose_predicted=True)
        np.testing.assert_allclose(surface.xyz[:, 2], [2.01, 2.42])
        depth[3, 3] = 2.43
        surface.integrate(rgb, depth, np.eye(4), params, labels, confidence)
        np.testing.assert_allclose(surface.xyz[:, 2], [2.01, 2.43])

    def test_larger_point_map_survives_history_replay_without_truncation(self):
        count = DEFAULT_POINT_BUDGET
        snapshot = dict(revision=1, timestamp_s=0., points=[[i*.01,0,1,120,140,160] for i in range(count)],
            cameras=[], rescue=dict(schema_version='rescue-semantic-map/1.0', render_mode='points',
                                   point_labels=[2]*count, people=[]))
        with tempfile.TemporaryDirectory() as temporary:
            writer = HistoryWriter(Path(temporary), write_json, synchronous=True)
            writer.append(snapshot); writer.close()
            replay = HistoryReader(temporary).snapshot(1)
        self.assertEqual(len(replay['points']), count)
        self.assertEqual(len(replay['rescue']['point_labels']), count)
