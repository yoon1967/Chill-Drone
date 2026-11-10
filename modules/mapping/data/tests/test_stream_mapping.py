"""Real queue bounds and RGB-D projection; no model download in unit tests."""
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
from modules.mapping.data.stream_mapping import DepthEstimator, PoseFrames, add_points, camera_intrinsic, run, write_json


class StreamMappingTests(unittest.TestCase):
    def test_warmup_matches_network_shape_and_only_repeats_for_new_resolution(self):
        estimator = DepthEstimator.__new__(DepthEstimator)
        estimator.estimate = MagicMock()
        estimator.prepare((288, 384, 3))
        estimator.prepare((289, 383, 3))  # Same 14px network dimensions.
        estimator.prepare((144, 256, 3))
        self.assertEqual([c.args[0].shape for c in estimator.estimate.call_args_list],
                         [(288, 384, 3), (144, 256, 3)])

    def test_cuda_idle_maintenance_is_rate_limited_and_cpu_has_no_extra_work(self):
        estimator = DepthEstimator.__new__(DepthEstimator)
        estimator.device = 'cuda'
        estimator.last_inference = 10.
        estimator.estimate = MagicMock()
        with patch('modules.mapping.data.stream_mapping.time.monotonic', return_value=10.5):
            estimator.keep_warm((288, 384, 3))
        estimator.estimate.assert_not_called()
        with patch('modules.mapping.data.stream_mapping.time.monotonic', return_value=10.8):
            estimator.keep_warm((288, 384, 3))
        self.assertEqual(estimator.estimate.call_args.args[0].shape, (288, 384, 3))
        estimator.estimate.reset_mock(); estimator.device = 'cpu'
        with patch('modules.mapping.data.stream_mapping.time.monotonic', return_value=20.):
            estimator.keep_warm((288, 384, 3))
        estimator.estimate.assert_not_called()

    def test_depth_model_reuses_cached_files_without_online_lookup(self):
        torch, transformers = MagicMock(), MagicMock()
        torch.cuda.is_available.return_value = False
        with patch.dict('sys.modules', {'torch': torch, 'transformers': transformers}), \
                patch('modules.mapping.data.stream_mapping.cached_snapshot', return_value=Path('local-depth-model')):
            estimator = DepthEstimator()
        self.assertEqual(estimator.device, 'cpu')
        transformers.AutoImageProcessor.from_pretrained.assert_called_once_with(
            'local-depth-model', use_fast=False, local_files_only=True)
        transformers.AutoModelForDepthEstimation.from_pretrained.assert_called_once_with(
            'local-depth-model', local_files_only=True)

    def test_only_explicit_preparation_can_download_missing_depth_weights(self):
        from modules.mapping.data.model_assets import ModelAssetsMissing
        torch, transformers = MagicMock(), MagicMock()
        torch.cuda.is_available.return_value = False
        model = MagicMock()
        transformers.AutoModelForDepthEstimation.from_pretrained.return_value = model
        with patch.dict('sys.modules', {'torch': torch, 'transformers': transformers}), \
                patch('modules.mapping.data.stream_mapping.cached_snapshot', side_effect=ModelAssetsMissing('depth')):
            with self.assertRaises(ModelAssetsMissing):
                DepthEstimator()
            transformers.AutoModelForDepthEstimation.from_pretrained.assert_not_called()
            DepthEstimator(allow_download=True)
        self.assertFalse(transformers.AutoModelForDepthEstimation.from_pretrained.call_args.kwargs['local_files_only'])
        model.to.assert_called_once_with('cpu')

    def test_snapshot_write_recovers_from_a_windows_reader_lock(self):
        import json
        import os
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'progress.json'
            write_json(path, {'revision': 1})
            replace = os.replace
            attempts = []
            def locked(source, target):
                attempts.append(1)
                if len(attempts) < 3:
                    raise PermissionError('reader busy')
                replace(source, target)
            with patch('modules.mapping.data.stream_mapping.os.replace', side_effect=locked):
                write_json(path, {'revision': 2})
            self.assertEqual(json.loads(path.read_text())['revision'], 2)
            self.assertFalse(path.with_suffix('.json.tmp').exists())

    def test_camera_resize_preserves_intrinsics(self):
        camera = dict(width=1920, height=1080, fx=1000, fy=1100, cx=960, cy=540)
        np.testing.assert_allclose(camera_intrinsic((180, 320, 3), (1920, 1080), camera),
                                   (1000 / 6, 1100 / 6, 160, 90))

    def test_cloud_rejects_invalid_depth_and_limits_voxel_count(self):
        rgb = np.full((16, 16, 3), 200, dtype=np.uint8)
        depth = np.full((16, 16), np.nan, dtype=np.float32)
        cells = {}
        add_points(cells, rgb, depth, np.eye(4), (20, 20, 8, 8), max_points=4)
        self.assertFalse(cells)
        depth[:] = 2
        add_points(cells, rgb, depth, np.eye(4), (20, 20, 8, 8), max_points=4)
        self.assertEqual(len(cells), 4)
        self.assertTrue(all(p[-3:] == [200, 200, 200] for p in cells.values()))

    def test_slow_consumer_keeps_every_selected_frame_in_order(self):
        rgb = np.zeros((32, 32, 3), dtype=np.uint8)
        with patch('modules.mapping.data.stream_mapping.frames', return_value=iter(
                [(1, 0., rgb, (32, 32)), (2, .01, rgb, (32, 32)), (3, .02, rgb, (32, 32))])):
            tracker = MagicMock()
            producer = PoseFrames('unused', 30, 100, Path('unused'), tracker_factory=lambda *args: tracker)
            try:
                self.assertEqual(producer.take()[0], 1)
                time.sleep(.25)
                self.assertEqual(producer.take()[0], 2)
                self.assertEqual(producer.take()[0], 3)
                self.assertEqual(producer.dropped, 0)
                self.assertIsNone(producer.take())
                self.assertEqual(tracker.track.call_count, 3)
            finally:
                producer.close()

    def test_decode_error_is_propagated(self):
        with patch('modules.mapping.data.stream_mapping.frames', side_effect=ValueError('bad_video')):
            producer = PoseFrames('unused', 30, 5, Path('unused'))
            with self.assertRaisesRegex(ValueError, 'bad_video'):
                producer.take()

    def test_recorded_video_predicts_a_short_gap_in_the_same_map_then_recovers(self):
        import json

        rgb = np.full((32, 32, 3), 120, np.uint8)
        stamps = (0., .083, .166, .25, .333, 1.5, 1.584)
        anchors = [[x, y, 3.] for x in (8, 16, 24, 28) for y in (8, 16)]

        def track(_rgb, stamp):
            if stamp in (.333, 1.5):
                return dict(timestamp_s=stamp, tracking_state=3, map_id=0)
            pose = np.eye(4)
            pose[0, 3] = -stamp
            return dict(timestamp_s=stamp, tracking_state=2, map_id=0,
                        T_cw=pose.tolist(), depth_anchors=anchors)

        tracker = MagicMock()
        tracker.track.side_effect = track
        class Estimator:
            def estimate(self, image):
                return np.full(image.shape[:2], 2., np.float32)
        images = [(index, stamp, rgb, (32, 32)) for index, stamp in enumerate(stamps, 1)]
        previews = []
        with tempfile.TemporaryDirectory() as folder, patch('modules.mapping.data.stream_mapping.frames', return_value=iter(images)):
            output = Path(folder) / 'result'
            progress = run('unused', output, {'frame_fps': 12, 'sample_fps': 12, 'semantic_mapping': False},
                           estimator_factory=Estimator, tracker_factory=lambda *args: tracker, on_preview=previews.append)
            self.assertEqual(progress['pose_predicted_frames'], 1)
            self.assertEqual(progress['completed_frame_number'], len(stamps))
            by_frame = {snapshot['frame_number']: snapshot for snapshot in previews}
            self.assertEqual(by_frame[5]['tracking'], 'pose_predicted')
            self.assertAlmostEqual(by_frame[5]['pose_position'][0], .333)
            self.assertEqual(by_frame[6]['tracking'], 'lost')
            self.assertEqual(by_frame[6]['points'], by_frame[5]['points'])
            self.assertEqual(by_frame[7]['tracking'], 'tracking')
            scene = json.loads((output / 'scene.json').read_text(encoding='utf-8'))
            self.assertEqual([c['frame_id'] for c in scene['cameras']],
                             [f'frame_{n:07d}.jpg' for n in (1, 3, 4, 5, 7)])

    def test_video_sampling_preserves_source_numbers_and_geometry(self):
        import av
        from modules.mapping.data.stream_mapping import frames
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'source.mp4'
            with av.open(str(path), 'w') as container:
                stream = container.add_stream('mpeg4', rate=12)
                stream.width, stream.height, stream.pix_fmt = 320, 160, 'yuv420p'
                for _ in range(12):
                    for packet in stream.encode(av.VideoFrame.from_ndarray(np.zeros((160, 320, 3), dtype=np.uint8), format='rgb24')):
                        container.mux(packet)
                for packet in stream.encode():
                    container.mux(packet)
            selected = list(frames(path, 12, 2))
            self.assertEqual([item[0] for item in selected], [1, 7])
            self.assertEqual([item[1] for item in selected], [0., .5])
            self.assertEqual(selected[0][2].shape, (128, 256, 3))
            self.assertEqual(selected[0][3], (320, 160))

    def test_causal_video_uses_dpvo_during_an_orb_gap_and_records_its_source(self):
        import json
        from modules.mapping.data.mapping_history import HistoryReader

        image = np.full((32, 32, 3), 120, np.uint8)
        stamps = [n / 10 for n in range(7)]
        anchors = [[x, y, 3.] for x in (8, 16, 24, 28) for y in (8, 16)]

        def orb_track(_image, stamp):
            if stamp in (.3, .4):
                return dict(timestamp_s=stamp, tracking_state=3, map_id=0)
            pose = np.eye(4)
            pose[0, 3] = -stamp
            return dict(timestamp_s=stamp, tracking_state=2, map_id=0, T_cw=pose, depth_anchors=anchors)

        def dpvo_track(_image, stamp):
            pose = np.eye(4)
            pose[0, 3] = 5. + stamp / 2.
            if stamp in (.3, .4):
                pose[:3, :3] = [[.98, 0, .199], [0, 1, 0], [-.199, 0, .98]]
            return pose

        class Estimator:
            def estimate(self, rgb):
                return np.full(rgb.shape[:2], 2., np.float32)

        orb, dpvo = MagicMock(), MagicMock()
        orb.track.side_effect = orb_track
        options = dict(frame_fps=10, sample_fps=5, causal_video=True)
        for mode in ('available', 'failed', 'unavailable'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                images = [(n, stamp, image, (32, 32)) for n, stamp in enumerate(stamps, 1)]
                dpvo.reset_mock()

                def track(rgb, stamp):
                    if mode == 'failed' and stamp == .3:
                        raise RuntimeError('gpu_failed')
                    return dpvo_track(rgb, stamp)

                dpvo.track.side_effect = track
                output = Path(directory) / 'result'
                with patch('modules.mapping.data.stream_mapping.frames', return_value=iter(images)), \
                        patch('modules.mapping.localization.pose_first.OrbPoseTracker', return_value=orb), \
                        patch('modules.mapping.localization.dpvo_pose.DpvoPoseTracker.start',
                              return_value=None if mode == 'unavailable' else dpvo):
                    run('unused', output, options, estimator_factory=Estimator)
                scene = json.loads((output / 'scene.json').read_text(encoding='utf-8'))
                expected_frames = int(mode == 'available')
                self.assertEqual(scene['depth_mapping']['dpvo'],
                                 dict(status='active' if expected_frames else mode, integrated_frames=expected_frames))
                history = HistoryReader(output)
                entries = history.refresh()
                self.assertEqual([entry['frame_number'] for entry in entries], [1, 3, 5, 7])
                self.assertEqual(entries[-1]['tracking'], 'tracking')
                if mode == 'available':
                    self.assertEqual(entries[2]['tracking'], 'pose_dpvo')
                    self.assertTrue(entries[2]['estimated_alignment'])
                    self.assertAlmostEqual(entries[2]['pose_position'][0], .4)
                    self.assertAlmostEqual(entries[2]['pose_rotation'][0][2], .199)
                    self.assertEqual(scene['cameras'][2]['pose_source'], 'dpvo_orb_aligned')
                    self.assertEqual(dpvo.track.call_count, len(stamps))
                else:
                    self.assertEqual(entries[2]['tracking'], 'pose_reused')
                self.assertEqual(dpvo.close.call_count, int(mode != 'unavailable'))

    def test_full_rate_video_processes_each_frame_before_reading_the_next(self):
        import json

        image = np.full((32, 32, 3), 120, dtype=np.uint8)
        later = np.full_like(image, 220)
        timestamps = [0., 1/30, 2/30, 3/30, 2.]
        actions = []

        def source(_path, fps, rate, _side):
            self.assertEqual(fps, 30)
            self.assertIsNone(rate)
            for number, stamp in enumerate(timestamps, 1):
                self.assertEqual(len(actions), max(0, number - 2))
                yield number, stamp, image if number < 5 else later, (32, 32)

        class Estimator:
            def prepare(self, _shape):
                pass

            def estimate(self, rgb):
                actions.append(int(rgb[0, 0, 0]))
                return np.full(rgb.shape[:2], 3. if actions[-1] == 220 else 2., np.float32)

        tracker = MagicMock()
        def track(_rgb, stamp):
            if stamp == timestamps[1]:
                return dict(timestamp_s=stamp, tracking_state=2, map_id=0, T_cw=np.eye(4),
                            depth_anchors=[[x, y, 2.] for x in (8, 12, 16, 24) for y in (8, 16)])
            return dict(timestamp_s=stamp, tracking_state=1 if stamp == 0. else 3, map_id=0)

        tracker.track.side_effect = track
        previews = []
        with tempfile.TemporaryDirectory() as folder, patch('modules.mapping.data.stream_mapping.frames', side_effect=source):
            output = Path(folder) / 'result'
            run('unused', output, {'frame_fps': 30, 'sample_fps': 2, 'all_video_frames': True},
                estimator_factory=Estimator, tracker_factory=lambda *args: tracker, on_preview=previews.append)
            self.assertEqual(actions, [120, 120, 120, 220])
            self.assertEqual([frame['frame_number'] for frame in previews], [1, 2, 3, 4, 5])
            self.assertEqual([frame['tracking'] for frame in previews], ['initializing', 'tracking', 'pose_reused', 'pose_reused', 'pose_held'])
            self.assertIsNone(previews[0]['pose_position'])
            self.assertFalse(previews[0]['points'])
            self.assertTrue(previews[-1]['estimated_alignment'])
            self.assertEqual(previews[-1]['pose_position'], previews[-2]['pose_position'])
            scene = json.loads((output / 'scene.json').read_text(encoding='utf-8'))
            self.assertEqual(scene['quality']['coverage']['integrated_frames'], 4)
            self.assertEqual(scene['quality']['coverage']['held_pose_frames'], 1)
            self.assertEqual(scene['cameras'][-1]['pose_source'], 'last_observed_pose')
            self.assertTrue(any(point[3] == -3 for point in scene['points']))

    def test_video_mapping_rate_samples_the_source_clock_without_skipping_pose_input(self):
        image = np.zeros((32, 32, 3), dtype=np.uint8)
        for rate, selected in [(3, [1, 11, 21, 31]), (4, [1, 9, 16, 24, 31])]:
            with self.subTest(rate=rate), patch('modules.mapping.data.stream_mapping.frames') as source:
                source.return_value = iter((index + 1, index / 30, image, (32, 32)) for index in range(33))
                tracker = MagicMock()
                tracker.track.side_effect = lambda _rgb, stamp: dict(timestamp_s=stamp,
                    tracking_state=1 if .2 <= stamp < .25 else 2, map_id=0)
                producer = PoseFrames('unused', 30, rate, Path('unused'), tracker_factory=lambda *args: tracker)
                producer.track_all_frames = True
                try:
                    items = list(iter(producer.take, None))
                    self.assertEqual([item[0] for item in items], selected)
                    self.assertEqual([item[5]['tracking_epoch'] for item in items], [0] + [1] * (len(items) - 1))
                    self.assertEqual(tracker.track.call_count, 33)
                    source.assert_called_once_with('unused', 30, None, 640)
                finally:
                    producer.close()
