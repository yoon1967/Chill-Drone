import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from modules.mapping.data.mapping_history import HistoryWriter, HistoryReader
from modules.mapping.data.stream_mapping import write_json, expire_render_points


class MappingHistoryTests(unittest.TestCase):
    def test_evicted_geometry_keeps_full_observation_and_exact_replay(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = HistoryWriter(directory, write_json, 10)
            observation = {'depth': np.full((4, 4), 2, np.float32),
                           'rgb': np.ones((4, 4, 3), np.uint8),
                           'world_from_camera': np.eye(4), 'intrinsics': np.array([2, 2, 2, 2]),
                           'map_id': np.array(0), 'depth_scale': np.array(.3)}
            point = [0, 0, 2, 255, 0, 0]
            writer.append(dict(revision=1, timestamp_s=0., frame_id='frame_0000001.jpg', points=[point], cameras=[[0, 0, 0]]), observation)
            cells, seen = {'a': point}, {'a': 0.}
            self.assertEqual(expire_render_points(cells, seen, 11., 10.), 1)
            writer.append(dict(revision=2, timestamp_s=11., frame_id='frame_0000002.jpg', points=list(cells.values()), cameras=[]))
            writer.close()
            reader = HistoryReader(directory)
            self.assertEqual(reader.timeline()['count'], 2)
            self.assertEqual(reader.snapshot(1)['points'], [point])
            self.assertEqual(reader.snapshot(2)['points'], [])
            with np.load(Path(directory) / 'history/frames/00000001.npz', allow_pickle=False) as chunk:
                np.testing.assert_array_equal(chunk['depth'], observation['depth'])
                np.testing.assert_array_equal(chunk['world_from_camera'], np.eye(4))
            self.assertEqual(json.loads((Path(directory) / 'history/manifest.json').read_text())['frames'], 2)
            # An interrupted append is never visible until its full line arrives.
            with (Path(directory) / 'history/index.jsonl').open('ab') as index:
                index.write(b'{"revision":3')
            self.assertEqual(reader.timeline()['count'], 2)
            with self.assertRaises(ValueError):
                reader.snapshot(3)

    def test_bounded_writer_drains_without_losing_processed_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = HistoryWriter(directory, write_json)
            for revision in range(1, 33):
                writer.append(dict(revision=revision, timestamp_s=revision / 5, points=[], cameras=[]))
            writer.close()
            self.assertEqual(HistoryReader(directory).timeline()['count'], 32)
