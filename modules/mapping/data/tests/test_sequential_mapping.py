"""The source cannot outrun preparation, recording or presentation."""
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np

from modules.mapping.data.stream_mapping import run


class SequentialMappingTests(unittest.TestCase):
    def pipeline(self, output, events, estimator_gate=None, presentation_gate=None, stop=None):
        rgb = np.full((32, 32, 3), 120, dtype=np.uint8)
        def source(*args):
            for number in range(1, 4):
                events.append(('read', number))
                yield number, (number - 1) * .2, rgb, (32, 32)
        class Estimator:
            def __init__(self):
                events.append(('model', 0))
                if estimator_gate:
                    estimator_gate[0].set()
                    if not estimator_gate[1].wait(3):
                        raise RuntimeError('test model gate timed out')
            def estimate(self, image):
                events.append(('depth' if image.any() else 'warmup', 0))
                return np.full(image.shape[:2], 2., dtype=np.float32)
        class Tracker:
            def __init__(self, *args):
                events.append(('tracker_start', 0))
            def track(self, image, stamp):
                number = round(stamp * 5) + 1
                events.append(('track', number))
                return {'timestamp_s': stamp, 'tracking_state': 2, 'map_id': 0,
                    'T_cw': np.eye(4).tolist(),
                    'depth_anchors': [[x, y, 3.] for x in (8, 16, 24, 28) for y in (8, 16)]}
            def close(self):
                events.append(('tracker_closed', 0))
        def published(snapshot):
            revision = snapshot['revision']
            entries = (output / 'history/index.jsonl').read_text(encoding='utf-8').splitlines()
            self.assertEqual(len(entries), revision)
            self.assertTrue((output / f'history/frames/{revision:08d}.npz').is_file())
            self.assertNotIn(('read', revision + 1), events)
            events.append(('shown', revision))
            if presentation_gate and revision == 1:
                presentation_gate[0].set()
                if not presentation_gate[1].wait(3):
                    raise RuntimeError('test presentation gate timed out')
        with patch('modules.mapping.data.stream_mapping.frames', side_effect=source):
            return run('unused', output, {'frame_fps': 5, 'sample_fps': 5},
                estimator_factory=Estimator, tracker_factory=Tracker, stop_event=stop, on_preview=published)

    def test_each_frame_is_recorded_and_presented_before_reading_the_next(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'result'
            events, errors = [], []
            entered, release = threading.Event(), threading.Event()
            def work():
                try:
                    self.pipeline(output, events, presentation_gate=(entered, release))
                except BaseException as error:
                    errors.append(error)
            thread = threading.Thread(target=work)
            thread.start()
            try:
                self.assertTrue(entered.wait(2))
                self.assertEqual([e for e in events if e[0] == 'read'], [('read', 1)])
                self.assertTrue(thread.is_alive())
            finally:
                release.set()
                thread.join(5)
            self.assertFalse(thread.is_alive())
            self.assertEqual(errors, [])
            self.assertEqual(events[:3], [('model', 0), ('warmup', 0), ('read', 1)])
            self.assertEqual([e for e in events if e[0] in {'track', 'shown'}],
                [('track', 1), ('shown', 1), ('track', 2), ('shown', 2), ('track', 3), ('shown', 3)])
            progress = json.loads((output / 'progress.json').read_text())
            self.assertEqual(progress['processed'], 3)
            self.assertEqual(progress['dropped'], 0)
            self.assertEqual(progress['status'], 'complete')

    def test_cancel_during_model_loading_never_opens_the_source(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'result'
            events, errors = [], []
            entered, release, stop = threading.Event(), threading.Event(), threading.Event()
            def work():
                try:
                    self.pipeline(output, events, estimator_gate=(entered, release), stop=stop)
                except BaseException as error:
                    errors.append(error)
            thread = threading.Thread(target=work)
            thread.start()
            try:
                self.assertTrue(entered.wait(2))
                self.assertFalse(any(e[0] == 'read' for e in events))
                progress = json.loads((output / 'progress.json').read_text())
                self.assertEqual((progress['stage'], progress['processed'], progress['timestamp_s']), ('model', 0, 0))
                stop.set()
            finally:
                release.set()
                thread.join(5)
            self.assertEqual(len(errors), 1)
            self.assertIsInstance(errors[0], InterruptedError)
            self.assertFalse(any(e[0] in {'read', 'track'} for e in events))
            self.assertEqual(json.loads((output / 'history/manifest.json').read_text())['status'], 'cancelled')
