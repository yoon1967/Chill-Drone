"""Check arbitrary-scale exports, partial coverage, input isolation and failures."""
import contextlib
import io
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from functools import partial
from http.server import ThreadingHTTPServer
import threading
import urllib.request
import urllib.error
from unittest.mock import patch

import numpy as np
import pycolmap
from PIL import Image

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'modules/mapping/localization'))
import reconstruct as sfm
spec = importlib.util.spec_from_file_location('phone_viewer', ROOT / 'apps/console/serve.py')
viewer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(viewer)


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


class ReconstructionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='sfm-contract-')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.images = self.root / 'input' / 'frames'
        self.images.mkdir(parents=True)
        options = pycolmap.SyntheticDatasetOptions()
        options.num_rigs = 1
        options.num_frames_per_rig = 5
        options.num_points3D = 40
        self.model = pycolmap.synthesize_dataset(options)
        self.frames = []
        for i, im in enumerate(sorted(self.model.images.values(), key=lambda im: im.name)):
            Image.new('RGB', (1024, 768), (40, 60, 80)).save(self.images / im.name)
            self.frames.append({'frame_id': im.name, 'image_id': im.image_id, 'source_pts_s': i * 0.5})
        self.database = self.root / 'features' / 'database.db'
        self.database.parent.mkdir()
        self.database.write_bytes(b'producer DB protected by checksum; mapper mocked')
        self.run = {'input_mode': 'video_features', 'root': self.images.parent,
                    'manifest': {'input_run_id': 'synthetic_new_video'},
                    'external': {'artifacts': {'database': {'sha256': sfm.sha256(self.database)}}},
                    'provenance': {'input_mode': 'video_features'}}

    def run_result(self, name='result', frames=None, models=None, **kwargs):
        with patch.object(sfm, 'prepare_video_features', return_value=(
                self.run, self.database, self.images, frames or self.frames, {'status': 'passed'})), \
             patch.object(sfm.pycolmap, 'incremental_mapping', return_value=(
                 {0: self.model} if models is None else models)), contextlib.redirect_stdout(io.StringIO()):
            return sfm.reconstruct(self.root / name, input_dir=self.images.parent,
                                   features_dir=self.database.parent, **kwargs)

    def test_supplied_intrinsics_disable_bundle_adjustment_refinement(self):
        self.run['fixed_intrinsics'] = True
        with patch.object(sfm, 'prepare_video_features', return_value=(self.run, self.database, self.images, self.frames, {'status': 'passed'})), \
             patch.object(sfm.pycolmap, 'incremental_mapping', return_value={0: self.model}) as mapping, contextlib.redirect_stdout(io.StringIO()):
            sfm.reconstruct(self.root / 'fixed', input_dir=self.images.parent, features_dir=self.database.parent)
        options = mapping.call_args.args[3]
        self.assertFalse(options.ba_refine_focal_length)
        self.assertFalse(options.ba_refine_principal_point)
        self.assertFalse(options.ba_refine_extra_params)

    def test_arbitrary_scale_geometry_pose_and_exports(self):
        scene = self.run_result()
        self.assertEqual(scene['scale']['unit'], 'arbitrary')
        self.assertEqual(scene['scale']['reason'], 'no_metric_reference')
        self.assertEqual(read(self.root / 'result/status.json')['status'], 'complete')
        self.assertEqual(len(scene['points']), self.model.num_points3D())
        for point in scene['points']:
            np.testing.assert_allclose(point[1:4], self.model.points3D[point[0]].xyz)
        for camera in scene['cameras']:
            im = self.model.images[camera['image_id']]
            np.testing.assert_allclose(camera['position'], im.projection_center())
            np.testing.assert_allclose(camera['world_from_camera_rotation'], im.cam_from_world().rotation.matrix().T)
        self.assertEqual([c['source_pts_s'] for c in scene['cameras']], sorted(c['source_pts_s'] for c in scene['cameras']))
        self.assertEqual(read(self.root / 'result/poses.json')['scale'], scene['scale'])
        self.assertIn('element vertex 40', (self.root / 'result/points.ply').read_text())
        self.assertNotIn('44cm', json.dumps(scene, ensure_ascii=False))
        for name, digest in read(self.root / 'result/provenance.json')['outputs'].items():
            self.assertEqual(sfm.sha256(self.root / 'result' / name), digest)
        self.assertIsNotNone(read(self.root / 'result/timing.json')['mapping_seconds'])
        self.assertEqual(sfm.sha256(self.database), self.run['external']['artifacts']['database']['sha256'])

    def test_partial_model_retains_unregistered_inventory(self):
        frames = self.frames + [{'frame_id': 'unconnected.jpg', 'image_id': 99, 'source_pts_s': 9.0}]
        scene = self.run_result(frames=frames)
        self.assertEqual(scene['quality']['result_status'], 'partial')
        self.assertEqual(scene['quality']['unregistered_submitted_frames'], ['unconnected.jpg'])
        self.assertEqual(read(self.root / 'result/status.json')['status'], 'partial')

    def test_explicit_exclusion_is_distinct_from_unregistered_submission(self):
        frames = self.frames + [{'frame_id': 'excluded.jpg', 'image_id': 99, 'source_pts_s': 9.0}]
        scene = self.run_result(frames=frames, exclude_frames=['excluded.jpg'])
        self.assertEqual(scene['quality']['result_status'], 'complete')
        self.assertEqual(scene['quality']['submitted_frames'], 5)
        self.assertEqual(scene['quality']['unregistered_frames'], ['excluded.jpg'])
        self.assertEqual(scene['quality']['registration_ratio'], 5 / 6)

    def test_no_model_is_failed_with_timing_and_no_scene(self):
        with self.assertRaisesRegex(ValueError, 'no connected model'):
            self.run_result(models={})
        self.assertEqual(read(self.root / 'result/status.json')['status'], 'failed')
        self.assertTrue((self.root / 'result/timing.json').is_file())
        self.assertFalse((self.root / 'result/scene.json').exists())

    def test_no_overwrite_mixed_inputs_or_nested_output(self):
        for output, kwargs in [(self.images / 'new', {'input_dir': self.images.parent, 'features_dir': self.database.parent}),
                               (self.root / 'unused', {'input_dir': self.images.parent}),
                               (self.root / 'unused', {'input_dir': self.images.parent, 'features_dir': self.database.parent,
                                                      'run_dir': ROOT / 'data'}),
                               (self.root, {})]:
            with self.subTest(output=output, kwargs=kwargs), self.assertRaises(ValueError):
                sfm.reconstruct(output, **kwargs)
        self.assertEqual(list(self.images.glob('new')), [])


    def test_adapter_rejects_not_ready_and_mismatched_producer_inputs(self):
        source = {"schema": "phone-video-input/1.0", "manifest_sha256": "original"}
        status = self.database.parent / 'status.json'
        metadata_path = self.database.parent / 'feature_metadata.json'
        metadata = {"schema_version": "phone-feature-result/1.0", "input_manifest_sha256": "other",
                    "database_sha256": sfm.sha256(self.database)}
        sfm.write_json(status, {"status": "failed"})
        sfm.write_json(metadata_path, metadata)
        with patch.object(sfm, 'load_feature_input', return_value=source):
            with self.assertRaisesRegex(ValueError, 'not ready'):
                sfm.prepare_video_features(self.images.parent, self.database.parent, self.root / 'check.json')
            sfm.write_json(status, {"status": "features_ready"})
            with self.assertRaisesRegex(ValueError, 'manifest hash'):
                sfm.prepare_video_features(self.images.parent, self.database.parent, self.root / 'check.json')
            metadata['input_manifest_sha256'] = 'original'
            metadata['database_sha256'] = 'wrong'
            sfm.write_json(metadata_path, metadata)
            with self.assertRaisesRegex(ValueError, 'checksum'):
                sfm.prepare_video_features(self.images.parent, self.database.parent, self.root / 'check.json')

    def test_viewer_serves_partial_scene_but_rejects_failed_result(self):
        frames = self.frames + [{'frame_id': 'unconnected.jpg', 'image_id': 99, 'source_pts_s': 9.0}]
        self.run_result(frames=frames)
        handler = partial(viewer.ViewerHandler, result=self.root / 'result')
        with ThreadingHTTPServer(('127.0.0.1', 0), handler) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                url = f'http://127.0.0.1:{server.server_port}'
                with urllib.request.urlopen(url + '/api/run') as response:
                    scene = json.load(response)
                    self.assertEqual(scene['quality']['result_status'], 'partial')
                    self.assertEqual(scene['scale']['unit'], 'arbitrary')
                with urllib.request.urlopen(url + '/api/points.ply') as response:
                    self.assertTrue(response.read().startswith(b'ply'))
                sfm.write_json(self.root / 'result/status.json', {'status': 'failed'})
                with self.assertRaises(urllib.error.HTTPError) as error:
                    urllib.request.urlopen(url + '/api/run')
                self.assertEqual(error.exception.code, 503)
            finally:
                server.shutdown()
                thread.join()

    def test_legacy_scale_path_still_uses_anchor_observations(self):
        anchors = self.root / 'anchors.json'
        observations = self.root / 'observations.csv'
        sfm.write_json(anchors, {'segments': [
            {'segment_id': 'width_44cm', 'length_m': 0.44, 'role': 'scale_reference', 'correspondence_ready': True},
            {'segment_id': 'height_136cm', 'correspondence_ready': False}]})
        observations.write_text('anchor_id,frame_id\nA,a.jpg\nB,b.jpg\n')
        run = {'paths': {'measurement_anchors': anchors, 'measurement_observations': observations}}
        with patch.object(sfm, 'triangulate_anchor', side_effect=[(np.zeros(3), {}), (np.array([2., 0., 0.]), {})]):
            factor, _, _, scale = sfm.scale_transform(self.model, run)
        self.assertAlmostEqual(factor, 0.22)
        self.assertEqual(scale['unit'], 'm')
        with patch.object(sfm, 'triangulate_anchor', side_effect=ValueError('bad parallax')):
            self.assertEqual(sfm.scale_transform(self.model, run)[3]['unit'], 'arbitrary')


if __name__ == '__main__':
    unittest.main()
