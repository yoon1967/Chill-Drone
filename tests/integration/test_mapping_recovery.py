"""Failure boundaries and continuity with synthetic frames; no real drone."""
from copy import deepcopy
from io import BytesIO
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from modules.mapping.localization.orbslam3 import SessionPublisher, atomic_json
from modules.mapping.localization.orbslam3_live import LiveManager, LiveSession
from modules.mapping.localization.tello_input import TelloMapInput, InputError, DEFAULT_CAMERA
from modules.mapping.localization.tests.test_landmarks import snapshot, CAMERA
from test_map_sources import SyntheticSession
from test_tello_input import Frames, JPEG, wait_for


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.live = LiveManager(self.root)
        for target, value in [('LiveSession', SyntheticSession), ('require_slam_image', lambda: 'synthetic')]:
            mocked = patch('modules.mapping.localization.orbslam3_live.' + target, value)
            mocked.start(); self.addCleanup(mocked.stop)

    def adapter(self, factory, seconds=.8):
        adapter = TelloMapInput(self.live, source_factory=factory, recovery_seconds=seconds)
        self.addCleanup(adapter.shutdown)
        return adapter

    def test_measured_camera_is_default_and_explicit_override_wins(self):
        expected = json.loads(DEFAULT_CAMERA.read_text(encoding='utf-8'))
        adapter = self.adapter(Frames)
        adapter.start({'fov': 60})
        self.assertEqual(self.live.current.camera['fx'], expected['fx'])
        self.assertEqual(self.live.current.camera['distortion'], expected['distortion'])
        self.assertIn('checkerboard_measured', self.live.current.camera['calibration_status'])
        self.live.current.stop(); wait_for(lambda: not adapter.busy)
        override = {**expected, 'fx': 900.}
        adapter.start({'camera': override})
        self.assertEqual(self.live.current.camera['fx'], 900.)

    def test_short_pause_and_socket_reopen_preserve_session_and_timestamp(self):
        gate = threading.Event()
        class Paused(Frames):
            def frames(self, cancelled):
                yield JPEG, time.perf_counter()
                while not cancelled() and not gate.is_set():
                    yield None, time.perf_counter(); time.sleep(.01)
                raise InputError('tello_gateway_stream_ended')
        first = Paused()
        second = Frames()
        factory = Mock(side_effect=[first, second])
        adapter = self.adapter(factory)
        session_id = adapter.start({})['session_id']
        wait_for(lambda: self.live.current.received == 1)
        original = self.live.current
        original.publish()
        first_pts = original.last_pts
        wait_for(lambda: original.input_state == 'reconnecting')
        self.assertFalse(original.done)
        self.assertEqual(original.stats()['session_status'], 'receiving')
        gate.set()
        wait_for(lambda: original.received > 1)
        self.assertIs(original, self.live.current)
        self.assertEqual(original.id, session_id)
        self.assertGreater(original.last_pts, first_pts)
        self.assertGreater(original.last_sequence, 0)
        self.assertTrue((original.path / 'revisions/000001/scene.json').is_file())
        self.assertEqual(original.reconnects, 1)
        self.assertTrue(first.closed)

    def test_outage_limit_and_user_cancel_stop_retrying(self):
        source = Frames()
        source.open = Mock(side_effect=InputError('tello_gateway_video_off_or_stale'))
        factory = Mock(return_value=source)
        adapter = self.adapter(factory, seconds=.04)
        adapter.start({})
        wait_for(lambda: not adapter.busy)
        self.assertEqual(self.live.current.error, 'tello_reconnect_timeout')
        count = factory.call_count
        time.sleep(.05)
        self.assertEqual(factory.call_count, count)
        adapter.recovery_seconds = 30
        adapter.start({})
        wait_for(lambda: self.live.current.input_state == 'reconnecting')
        self.live.current.stop()
        wait_for(lambda: not adapter.busy)
        self.assertEqual(self.live.current.reason, 'user_stop')

    def test_numerical_failure_is_not_overwritten_by_ack_timeout(self):
        session = object.__new__(LiveSession)
        session.condition = threading.Condition()
        session.pending = None; session.error = None
        session._read(Mock(stdout=BytesIO(b'Sophus ensure failed\nSO3::exp failed! omega: -nan\n')), BytesIO())
        self.assertEqual(session.error, 'tracker_numerical_failure')
        self.assertTrue(session.closing)
        self.assertEqual(session.reason, 'tracker_numerical_failure')

    def test_fast_jpeg_cache_reuses_only_matching_frame_and_quality(self):
        from modules.autonomy.tello_link.tello_video import TelloVideo
        video = TelloVideo()
        video._frame = Mock()
        video._frame.to_ndarray.return_value = np.full((60, 80, 3), 128, np.uint8)
        video._frame_mono, video._frame_wall = 1., 2.
        first = video.jpeg_snapshot(75)
        self.assertEqual(video.jpeg_snapshot(75), first)
        self.assertEqual(cv2.imdecode(np.frombuffer(first[0], np.uint8), cv2.IMREAD_COLOR).shape, (60, 80, 3))
        video.jpeg_snapshot(90)
        self.assertEqual(video._frame.to_ndarray.call_count, 2)

    def test_invalid_snapshot_and_new_coordinate_system_preserve_old_revisions(self):
        (self.root / 'revisions').mkdir()
        publisher = SessionPublisher(self.root, CAMERA)
        publisher.preview_enabled = False
        atomic_json(self.root / 'tracking.json', {'tracking_state': 2})
        native = snapshot()
        atomic_json(self.root / 'native.json', native); publisher.update()
        first = (self.root / 'revisions/000001/scene.json').read_bytes()
        bad = deepcopy(native); bad['revision'] = 2
        bad['keyframes'][0]['T_cw'][0][0] = 2.
        atomic_json(self.root / 'native.json', bad); publisher.update()
        self.assertEqual(publisher.revision, 1)
        self.assertEqual(publisher.state['map_warning'], 'invalid_snapshot_preserved_previous')
        native.update(revision=3, map_id=8)
        atomic_json(self.root / 'native.json', native); publisher.update()
        self.assertEqual(publisher.revision, 3)
        self.assertEqual((self.root / 'revisions/000001/scene.json').read_bytes(), first)
        saved = json.loads((self.root / 'segments.json').read_text(encoding='utf-8'))
        self.assertEqual([x['map_id'] for x in saved['segments']], [7, 8])
        self.assertIn('never_concatenate', saved['alignment'])


if __name__ == '__main__':
    unittest.main()
