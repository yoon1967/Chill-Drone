"""Real queue bounds and RGB-D projection; no model download in unit tests."""
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
from modules.mapping.data.stream_mapping import DepthEstimator, LatestFrames, add_points, camera_intrinsic, write_json


class StreamMappingTests(unittest.TestCase):
    def test_depth_model_reuses_cached_files_without_online_lookup(self):
        torch, transformers = MagicMock(), MagicMock()
        torch.cuda.is_available.return_value = False
        with patch.dict('sys.modules', {'torch': torch, 'transformers': transformers}):
            estimator = DepthEstimator()
        self.assertEqual(estimator.device, 'cpu')
        transformers.AutoImageProcessor.from_pretrained.assert_called_once_with(
            'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf', use_fast=False, local_files_only=True)
        transformers.AutoModelForDepthEstimation.from_pretrained.assert_called_once_with(
            'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf', local_files_only=True)

    def test_depth_model_downloads_when_the_cache_is_missing(self):
        torch, transformers = MagicMock(), MagicMock()
        torch.cuda.is_available.return_value = False
        model = MagicMock()
        transformers.AutoModelForDepthEstimation.from_pretrained.side_effect = [OSError('not cached'), model]
        with patch.dict('sys.modules', {'torch': torch, 'transformers': transformers}):
            DepthEstimator()
        self.assertEqual(transformers.AutoModelForDepthEstimation.from_pretrained.call_count, 2)
        self.assertNotIn('local_files_only', transformers.AutoModelForDepthEstimation.from_pretrained.call_args.kwargs)
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

    def test_pending_frames_are_replaced_and_final_frame_is_consumed(self):
        rgb = np.zeros((32, 32, 3), dtype=np.uint8)
        with patch('modules.mapping.data.stream_mapping.frames', return_value=iter(
                [(1, 0., rgb, (32, 32)), (2, .01, rgb, (32, 32)), (3, .02, rgb, (32, 32))])):
            producer = LatestFrames('unused', 30, 100)
            producer.thread.start()
            producer.thread.join(1)
            self.assertTrue(producer.done)
            self.assertEqual(producer.take()[0], 3)
            self.assertEqual(producer.dropped, 2)
            self.assertIsNone(producer.take())

    def test_decode_error_is_propagated(self):
        with patch('modules.mapping.data.stream_mapping.frames', side_effect=ValueError('bad_video')):
            producer = LatestFrames('unused', 30, 5)
            producer.thread.start()
            producer.thread.join(1)
            with self.assertRaisesRegex(ValueError, 'bad_video'):
                producer.take()

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
