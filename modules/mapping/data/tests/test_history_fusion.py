import json
from pathlib import Path
import tempfile
import time
import unittest

import numpy as np

from modules.mapping.data.mapping_history import HistoryWriter
from modules.mapping.data.stream_mapping import finalize_history, write_json


class HistoryFusionTests(unittest.TestCase):
    def make_history(self, output, map_ids):
        writer = HistoryWriter(output, write_json)
        writer.append(dict(revision=1, timestamp_s=0., frame_id='frame_0000001.jpg', frame_number=1,
                           tracking='initializing', observation=False, points=[], cameras=[]))
        for i, map_id in enumerate(map_ids):
            pose = np.eye(4); pose[0, 3] = i * 1000
            writer.append(dict(revision=i + 2, timestamp_s=(i + 1) * 20., frame_id=f'frame_{i+2:07d}.jpg',
                frame_number=i + 2, tracking='tracking', points=[], cameras=[]),
                dict(depth=np.full((256, 256), 2., np.float32), rgb=np.full((256, 256, 3), 150, np.uint8),
                     world_from_camera=pose, intrinsics=np.array([1., 1., 0., 0.]), map_id=np.array(map_id), depth_scale=np.array(1.)))
        writer.close()

    def test_final_map_contains_entire_route_without_preview_time_or_count_caps(self):
        with tempfile.TemporaryDirectory() as directory:
            self.make_history(directory, [0] * 11)
            stages = []
            finalize_history(directory, {}, 'test', time.monotonic(), lambda **kw: stages.append(kw))
            scene = json.loads((Path(directory) / 'scene.json').read_text())
            self.assertEqual(len(scene['points']), 11 * 4096)
            self.assertEqual(len(scene['cameras']), 11)
            self.assertEqual(scene['cameras'][0]['timestamp_s'], 20.)
            self.assertEqual(scene['cameras'][-1]['timestamp_s'], 220.)
            self.assertEqual(scene['quality']['result_status'], 'complete', 'initialization alone must not mark the whole map partial')
            self.assertEqual(scene['quality']['coverage']['unmapped_frames'], 1)
            self.assertFalse(scene['depth_mapping']['preview_window_applied'])
            self.assertEqual(stages[-2]['integrated_frames'], 11)
            self.assertEqual(json.loads((Path(directory) / 'status.json').read_text())['status'], 'complete')

    def test_disconnected_maps_are_fully_preserved_without_fabricated_alignment(self):
        with tempfile.TemporaryDirectory() as directory:
            self.make_history(directory, [0, 0, 1])
            finalize_history(directory, {}, 'test', time.monotonic())
            scene = json.loads((Path(directory) / 'scene.json').read_text())
            self.assertEqual(scene['segment_id'], 0, 'the last fragment must not replace the whole route')
            self.assertEqual(scene['quality']['partial_reasons'], ['unaligned_map_segments'])
            self.assertEqual(scene['depth_mapping']['integrated_frames'], 3)
            for submap in scene['depth_mapping']['submaps']:
                group = json.loads((Path(directory) / submap['file']).read_text())
                self.assertEqual(len(group['points']), submap['frames'] * 4096)
