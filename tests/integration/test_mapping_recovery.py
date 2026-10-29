"""Failure boundaries and continuity with synthetic frames; no real drone."""
from copy import deepcopy
from io import BytesIO
from functools import partial
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch
from urllib.request import Request, urlopen
from urllib.error import HTTPError

import cv2
import numpy as np

from modules.mapping.localization.orbslam3 import SessionPublisher, atomic_json
from modules.mapping.localization.orbslam3_live import LiveManager, LiveSession
from modules.mapping.localization.tello_input import TelloMapInput, InputError, DEFAULT_CAMERA
from modules.mapping.localization.tests.test_landmarks import snapshot, CAMERA
from test_map_sources import SyntheticSession
from test_tello_input import Frames, JPEG, wait_for
from apps.console.serve import ViewerHandler, ViewerServer


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
        session.closing = session.done = False
        session.path = self.root
        session.created = time.monotonic()
        session.client_event_count = 0
        session.last_pts = session.ack = -1
        session._read(Mock(stdout=BytesIO(b'Sophus ensure failed\nSO3::exp failed! omega: -nan\n')), BytesIO())
        self.assertEqual(session.error, 'tracker_numerical_failure')
        self.assertTrue(session.closing)
        self.assertEqual(session.reason, 'tracker_numerical_failure')

    def test_first_stop_cause_survives_cleanup_and_events_are_bounded(self):
        from modules.mapping.localization.orbslam3_live import live_camera
        with patch('threading.Thread.start'):
            session = LiveSession(self.root, live_camera({'width': 960, 'height': 720}), 'synthetic')
        session.record_event('client_error', 'render failure')
        session.stop('tello_client_timeout')
        session.stop('user_stop')
        session.stop('server_shutdown')
        self.assertEqual(session.stats()['stop_reason'], 'tello_client_timeout')
        events = [json.loads(x) for x in (session.path / 'events.jsonl').read_text().splitlines()]
        self.assertEqual([x['event'] for x in events], ['client_error', 'stop_requested'])
        self.assertEqual(events[-1]['message'], 'tello_client_timeout')
        with self.assertRaises(ValueError):
            session.record_event('unknown')
        with self.assertRaises(ValueError):
            session.record_event('client_error', 'x' * 1025)
        for _ in range(600):
            session.record_event('status_retry', 'synthetic')
        self.assertEqual(len((session.path / 'events.jsonl').read_text().splitlines()), 512)

    def test_http_stop_and_event_validate_session_and_preserve_error(self):
        state = self.live.start({'width': 960, 'height': 720})
        session = self.live.current
        server = ViewerServer(('127.0.0.1', 0), partial(ViewerHandler, live=self.live))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        def post(path, **body):
            request = Request(f'http://127.0.0.1:{server.server_port}/api/live/{path}',
                data=json.dumps({'session_id': state['session_id'], **body}).encode(),
                headers={'X-Mapping-Client': '1'})
            return urlopen(request)
        with post('event', event='status_retry', message='synthetic timeout') as response:
            self.assertEqual(response.status, 202)
        for body, status in [({'session_id': 'other', 'event': 'client_error'}, 409),
                             ({'event': 'invalid'}, 400),
                             ({'event': 'client_error', 'message': 'x' * 1025}, 400)]:
            with self.assertRaises(HTTPError) as error:
                post('event', **body)
            self.assertEqual(error.exception.code, status)
        with self.assertRaises(HTTPError) as error:
            post('stop', reason='server_shutdown')
        self.assertEqual(error.exception.code, 400)
        with post('stop', reason='client_error', error='synthetic renderer exception') as response:
            self.assertEqual(response.status, 202)
        with post('stop', reason='user_stop'):
            pass
        self.assertEqual(session.reason, 'client_error')
        events = (session.path / 'events.jsonl').read_text()
        self.assertIn('synthetic renderer exception', events)

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

    def test_unchanged_large_snapshot_is_not_reparsed_for_status_polls(self):
        import modules.mapping.localization.orbslam3 as orb
        publisher = SessionPublisher(self.root, CAMERA)
        publisher.preview_enabled = False
        atomic_json(self.root / 'tracking.json', {'tracking_state': 2})
        (self.root / 'revisions').mkdir()
        native = snapshot()
        atomic_json(self.root / 'native.json', native)
        with patch.object(orb, 'read_json', wraps=orb.read_json) as reader:
            for _ in range(10):
                publisher.update()
            native_reads = [call for call in reader.call_args_list if call.args[0].name == 'native.json']
            self.assertEqual(len(native_reads), 1)
            native['revision'] = 2
            atomic_json(self.root / 'native.json', native)
            publisher.update()
            self.assertEqual(publisher.revision, 2)


if __name__ == '__main__':
    unittest.main()
