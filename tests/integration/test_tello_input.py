"""Camera-only SDK ownership and gateway/SLAM boundaries. Never contacts a drone."""
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import json
import socket
from pathlib import Path
import tempfile
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from unittest.mock import Mock, patch

import cv2
import numpy as np

from apps.console.serve import ObservationCache, Timeline, ViewerHandler, ViewerServer
from modules.mapping.localization.orbslam3_live import LiveManager
from modules.mapping.localization.tello_input import DirectCamera, GatewayCamera, InputError, TelloMapInput, gateway_address, mjpeg_frames
from test_map_sources import SyntheticSession


JPEG = cv2.imencode('.jpg', np.zeros((720, 960, 3), dtype=np.uint8))[1].tobytes()


def wait_for(predicate):
    deadline = time.monotonic() + 3
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError('timed out')
        time.sleep(.01)


class Frames:
    time_basis = 'synthetic_receive'

    def __init__(self):
        self.closed = False

    def open(self, cancelled):
        pass

    def frames(self, cancelled):
        while not cancelled():
            # 100fps synthetic input must not reuse Windows' coarse monotonic tick.
            yield JPEG, time.perf_counter()
            time.sleep(.01)

    def close(self):
        self.closed = True


class TelloInputTests(unittest.TestCase):
    def test_direct_sdk_and_h264_with_local_fake_tello(self):
        from modules.autonomy.tello_link.fake_tello import FakeTello
        from modules.autonomy.tello_link.tello_link import TelloLink
        from modules.autonomy.tello_link.tello_video import TelloVideo
        try:
            import av
        except ImportError:
            self.skipTest('PyAV is needed for the synthetic H.264 sender')
        def port():
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind(('127.0.0.1', 0))
                return probe.getsockname()[1]
        state_port, video_port = port(), port()
        fake = FakeTello(cmd_port=0, state_port=state_port, video_port=video_port).start()
        self.addCleanup(fake.close)
        source = DirectCamera(
            lambda **kw: TelloLink(host='127.0.0.1', cmd_port=fake._sock.getsockname()[1],
                local_cmd_port=0, state_port=state_port, bind_ip='127.0.0.1', **kw),
            lambda: TelloVideo(port=video_port, bind_ip='127.0.0.1'))
        try:
            source.open(lambda: False)
            jpeg, stamp = next(source.frames(lambda: False))
            self.assertTrue(jpeg.startswith(b'\xff\xd8'))
            self.assertGreater(stamp, 0)
        finally:
            source.close()
        self.assertEqual(fake.received, ['command', 'streamon', 'streamoff'])

    def test_direct_camera_uses_existing_receiver_and_sends_only_session_commands(self):
        link, video = Mock(), Mock()
        link.send.return_value.ok = True
        now = time.monotonic()
        video.jpeg_snapshot.side_effect = [(JPEG, now, 1), (JPEG, now, 1), (JPEG, now + .01, 2)]
        factory = Mock(return_value=link)
        source = DirectCamera(factory, Mock(return_value=video))
        source.open(lambda: False)
        iterator = source.frames(lambda: False)
        self.assertEqual(next(iterator)[1], now)
        self.assertEqual(next(iterator)[1], now + .01)
        iterator.close()
        source.close()
        factory.assert_called_once_with(allow_flight=False)
        self.assertEqual([c.args[0] for c in link.send.call_args_list], ['command', 'streamon', 'streamoff'])
        video.stop.assert_called_once()
        link.close.assert_called_once()

    def test_failed_or_cancelled_open_can_be_cleaned_without_streaming_or_flight(self):
        link, video = Mock(), Mock()
        source = DirectCamera(Mock(return_value=link), Mock(return_value=video))
        source.open(lambda: True)
        source.close()
        link.send.assert_not_called()
        video.start.assert_not_called()
        link.close.assert_called_once()

    def test_gateway_origin_rejects_remote_hosts_credentials_paths_and_redirects(self):
        for value in ['http://192.168.10.1:8010', 'https://localhost:8010', 'http://localhost',
                      'http://a@localhost:8010', 'http://localhost:8010/path', 'http://localhost:8010?x=1']:
            with self.subTest(value=value), self.assertRaises(ValueError):
                gateway_address(value)
        self.assertEqual(gateway_address('http://localhost:8010/'), 'http://127.0.0.1:8010')

    def test_mjpeg_handles_chunked_boundary_without_lengths_and_rejects_oversize(self):
        data = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + JPEG + b'\r\n--frame\r\n' + JPEG
        frames = mjpeg_frames(BytesIO(data), lambda: False)
        self.assertEqual(next(frames), JPEG)
        self.assertEqual(next(frames), JPEG)
        with self.assertRaises(InputError):
            next(frames)
        with self.assertRaisesRegex(InputError, 'too_large'):
            next(mjpeg_frames(BytesIO(b'\xff\xd8' + b'x' * 2097153), lambda: False))

    def gateway(self, *, old=False, age=.01, on=True, redirect=False):
        requests = []
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                requests.append(self.path)
                if redirect:
                    self.send_response(302)
                    self.send_header('Location', 'http://192.168.10.1/private')
                    self.end_headers()
                    return
                if self.path == '/api/drone/state' and old:
                    self.send_error(404)
                    return
                self.send_response(200)
                if self.path == '/video.mjpg':
                    body = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + JPEG + b'\r\n'
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                else:
                    body = json.dumps({'run_id': 'fake', 'stream_on': on,
                        'video': {'frame_age_s': age, **({} if old else {'on': on, 'available': on})}}).encode()
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return f'http://127.0.0.1:{server.server_port}', requests

    def test_real_gateway_reader_supports_both_status_contracts_with_gets_only(self):
        for old in (False, True):
            address, requests = self.gateway(old=old)
            source = GatewayCamera(address)
            source.open(lambda: False)
            self.addCleanup(source.close)
            iterator = source.frames(lambda: False)
            self.assertEqual(next(iterator)[0], JPEG)
            with self.assertRaisesRegex(InputError, 'stream_ended'):
                next(iterator)
            self.assertIn('/video.mjpg', requests)
            self.assertTrue(all(p in {'/api/drone/state', '/api/status', '/video.mjpg'} for p in requests))

    def test_stale_off_and_redirected_gateways_never_supply_frames(self):
        for values in [dict(age=4), dict(age=float('nan')), dict(on=False), dict(redirect=True)]:
            address, requests = self.gateway(**values)
            source = GatewayCamera(address)
            with self.subTest(values=values), self.assertRaises((InputError, HTTPError)):
                source.open(lambda: False)
            self.assertNotIn('/video.mjpg', requests)

    def test_gateway_burst_frames_remain_strictly_timed_on_coarse_windows_clock(self):
        source = GatewayCamera('http://127.0.0.1:8010')
        source.response = BytesIO(b'--frame\r\n' + JPEG + b'\r\n--frame\r\n' + JPEG)
        source.checked = 1
        with patch('modules.mapping.localization.tello_input.time.monotonic', return_value=1):
            frames = source.frames(lambda: False)
            first, second = next(frames), next(frames)
        self.assertGreater(second[1], first[1])

    def test_failed_source_and_cancelled_open_release_ownership_before_restart(self):
        with tempfile.TemporaryDirectory() as directory, \
                patch('modules.mapping.localization.orbslam3_live.LiveSession', SyntheticSession), \
                patch('modules.mapping.localization.orbslam3_live.subprocess.check_output', return_value='synthetic'):
            live = LiveManager(Path(directory))
            source = Frames()
            source.open = Mock(side_effect=InputError('tello_gateway_video_off_or_stale'))
            adapter = TelloMapInput(live, source_factory=lambda: source)
            adapter.start({'fov':60})
            wait_for(lambda: not adapter.busy)
            self.assertTrue(source.closed)
            self.assertEqual(live.current.error, 'tello_gateway_video_off_or_stale')
            gate, entered = threading.Event(), threading.Event()
            source = Frames()
            def opening(cancelled):
                entered.set(); gate.wait(2)
            source.open = opening
            adapter.source_factory = lambda: source
            adapter.start({'fov':60}); self.assertTrue(entered.wait(1))
            live.current.stop()
            with self.assertRaisesRegex(InputError, 'cleanup_pending'):
                adapter.start({'fov':60})
            gate.set(); wait_for(lambda: not adapter.busy)
            self.assertTrue(source.closed)
            self.assertEqual(live.current.received, 0)

    def test_session_cancellation_restart_lease_and_http_ownership(self):
        with tempfile.TemporaryDirectory() as directory, \
                patch('modules.mapping.localization.orbslam3_live.LiveSession', SyntheticSession), \
                patch('modules.mapping.localization.orbslam3_live.subprocess.check_output', return_value='synthetic'):
            live = LiveManager(Path(directory))
            sources = []
            def source_factory():
                source = Frames()
                sources.append(source)
                return source
            adapter = TelloMapInput(live, source_factory=source_factory)
            self.addCleanup(adapter.shutdown)
            first = adapter.start({'fov': 60})['session_id']
            wait_for(lambda: live.current.received > 1)
            self.assertEqual(live.current.stats()['input'], 'tello_camera')
            self.assertEqual(live.current.camera['height'], 720)
            self.assertEqual(adapter.latest(first), JPEG)
            live.current.stop()
            wait_for(lambda: not adapter.busy)
            self.assertTrue(sources[0].closed)
            second = adapter.start({'fov': 60})['session_id']
            self.assertNotEqual(first, second)
            with self.assertRaises(InputError):
                adapter.heartbeat(first)
            self.assertIsNone(adapter.latest(first))
            wait_for(lambda: live.current.received > 0)
            server = ViewerServer(('127.0.0.1', 0), partial(ViewerHandler, live=live, tello_map=adapter,
                timeline=Timeline(), observations=ObservationCache()))
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            self.addCleanup(server.server_close)
            self.addCleanup(server.shutdown)
            address = f'http://127.0.0.1:{server.server_port}'
            request = Request(address + '/api/live/frame?session=' + second, data=JPEG,
                headers={'X-Mapping-Client': '1', 'X-Frame-Sequence': '99', 'X-Frame-Timestamp': '9'})
            with self.assertRaises(HTTPError) as error:
                urlopen(request)
            self.assertEqual(error.exception.code, 409)
            with urlopen(address + '/api/live/tello-preview.jpg?session=' + second) as response:
                self.assertEqual(response.read(), JPEG)
            with self.assertRaises(HTTPError) as error:
                urlopen(address + '/api/live/tello-preview.jpg?session=' + first)
            self.assertEqual(error.exception.code, 409)
            adapter.lease_until = time.monotonic() - 1
            wait_for(lambda: not adapter.busy)
            self.assertEqual(live.current.reason, 'tello_client_timeout')
            self.assertTrue(sources[1].closed)

    def test_bad_calibration_never_opens_a_source_or_starts_slam(self):
        live, factory = Mock(), Mock()
        adapter = TelloMapInput(live, source_factory=factory)
        with self.assertRaises(ValueError):
            adapter.start({'camera': {'width': 1920, 'height': 1080, 'fx': 1000, 'fy': 1000, 'cx': 960, 'cy': 540}})
        live.start.assert_not_called()
        factory.assert_not_called()


if __name__ == '__main__':
    unittest.main()
