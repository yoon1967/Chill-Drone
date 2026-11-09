from functools import partial
import errno
from http.client import HTTPConnection
import io
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import zipfile

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from apps.console import serve
from apps.console.serve import ViewerHandler, ViewerServer
from modules.autonomy.tello_link.discovery import DroneDiscovery, wifi_candidates
from modules.mapping.data.session_import import ImportedMap


class DiscoveryTests(unittest.TestCase):
    def setUp(self):
        scan = patch('modules.autonomy.tello_link.discovery.wifi_candidates', return_value=[
            {'id': 'TELLO-test', 'ssid': 'TELLO-test', 'kind': 'wifi'}])
        scan.start(); self.addCleanup(scan.stop)
        sleep = patch('modules.autonomy.tello_link.discovery.time.sleep')
        sleep.start(); self.addCleanup(sleep.stop)
        self.link = Mock()
        self.link.latest_state.return_value = {'age_s': .1, 'raw': {'bat': 79, 'h': 0}}
        self.link.send.side_effect = lambda cmd, **kw: SimpleNamespace(ok=True, response={
            'ext version?': 'unknown command', 'ext tof?': None}.get(cmd, 'ok' if cmd == 'command' else '79'))
        self.factory = Mock(return_value=self.link)
        self.manager = DroneDiscovery(link_factory=self.factory, runner=lambda args: '')

    def test_scan_parses_windows_and_linux_and_marks_drone_candidates(self):
        runner = Mock(return_value='SSID 1 : Home\nSSID 2 : TELLO-ABC\n    BSSID 1 : aa:bb\nSSID 3 : RMTT-TEST\n')
        with patch('modules.autonomy.tello_link.wifi.request_windows_scan'):
            values = wifi_candidates(runner, 'win32')
        self.assertEqual([i['ssid'] for i in values], ['Home', 'RMTT-TEST', 'TELLO-ABC'])
        self.assertFalse(values[0]['drone_candidate'])
        self.assertTrue(all(not i['verified'] for i in values))
        self.assertEqual(wifi_candidates(lambda args: ':TELLO-a:80\n:Home:20\n:TELLO-a:70\n', 'linux')[0]['signal_pct'], 80)

    def test_explicit_current_network_connection_needs_no_scan_or_wifi_switch(self):
        self.manager.runner = Mock(return_value='')
        self.assertTrue(self.manager.connect('current-network')['connected'])
        self.assertEqual(self.manager.candidates, {})
        self.assertFalse(any(call.args[0][:3] == ['netsh', 'wlan', 'connect']
                             for call in self.manager.runner.call_args_list))
        self.assertEqual(self.link.send.call_args.args[0], 'command')
        self.manager.release()

    def test_connect_queries_resources_and_distinguishes_unknown_from_no_response(self):
        self.manager.scan()
        self.assertTrue(self.manager.connect('TELLO-test')['connected'])
        self.factory.assert_called_once_with(allow_flight=False, allow_config=False)
        data = self.manager.explore()
        results = {v['command']: v for v in data['resources'] if 'command' in v}
        self.assertEqual(results['ext version?']['status'], 'unsupported')
        self.assertEqual(results['ext tof?']['status'], 'unconfirmed')
        self.assertEqual(results['battery?']['value'], '79')
        self.assertFalse(any(call.args[0] in {'takeoff', 'streamon', 'wifi'} for call in self.link.send.call_args_list))
        self.manager.release()
        self.link.close.assert_called_once()

    def test_failed_sdk_and_busy_input_never_leave_a_second_receiver(self):
        self.manager.scan()
        self.manager.busy = lambda: True
        with self.assertRaisesRegex(RuntimeError, 'active'):
            self.manager.connect('TELLO-test')
        self.factory.assert_not_called()
        self.manager.busy = lambda: False
        self.link.send.return_value = SimpleNamespace(ok=False)
        self.link.send.side_effect = None
        with self.assertRaisesRegex(RuntimeError, 'sdk_unavailable'):
            self.manager.connect('TELLO-test')
        self.assertEqual(self.link.close.call_count, 3)
        self.assertFalse(self.manager.state()['connected'])

    def test_wifi_route_transition_retries_with_fresh_sockets(self):
        first = Mock()
        first.send.side_effect = OSError(errno.ENETUNREACH, 'Wi-Fi route not ready')
        self.factory.side_effect = [first, self.link]
        self.assertTrue(self.manager.connect('current-network')['connected'])
        first.close.assert_called_once()
        self.link.close.assert_not_called()
        self.assertEqual(self.factory.call_count, 2)

    def test_real_bind_conflict_is_not_retried_or_misreported_as_wifi(self):
        self.link.start.side_effect = OSError(errno.EADDRINUSE, 'port in use')
        with self.assertRaisesRegex(RuntimeError, 'drone_ports_busy_or_unavailable'):
            self.manager.connect('current-network')
        self.factory.assert_called_once()
        self.link.close.assert_called_once()
        self.link.send.assert_not_called()

    def test_persistent_route_failure_is_not_reported_as_port_conflict(self):
        self.link.send.side_effect = OSError(errno.ENETUNREACH, 'no route')
        with self.assertRaisesRegex(RuntimeError, 'drone_network_unavailable_check_wifi'):
            self.manager.connect('current-network')
        self.assertEqual(self.link.close.call_count, 3)
        self.assertFalse(self.manager.state()['connected'])

    def test_video_handoff_stops_owned_stream_and_borrowed_link_is_never_queried(self):
        self.manager.scan()
        self.manager.connect('TELLO-test')
        video = Mock()
        self.manager.video_factory = lambda: video
        self.manager.start_preview()
        self.manager.release()
        video.stop.assert_called_once()
        self.assertIn('streamoff', [c.args[0] for c in self.link.send.call_args_list])
        self.link.reset_mock()
        self.manager.borrowed = lambda: self.link
        self.assertTrue(self.manager.explore()['connected'])
        self.link.send.assert_not_called()
        self.manager.release()
        self.link.close.assert_not_called()


class ConsoleAPI(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        for source, count in [('saved', 1), ('live/revisions/000001', 2), ('live/revisions/000002', 3)]:
            folder = self.root / source
            folder.mkdir(parents=True)
            (folder / 'status.json').write_text('{"status":"complete"}')
            (folder / 'scene.json').write_text(json.dumps({'schema_version': 'phone-sfm-result/1.2',
                'input_run_id': 'saved-map', 'points': [[1, count, 0, 0, 10, 20, 30, 0, 3]],
                **({'revision': count - 1, 'segment_id': count - 1} if source.startswith('live/') else {}),
                'cameras': [{'image_id': 1, 'frame_id': 'a.jpg', 'timestamp_s': 0, 'position': [0, 0, 0]}],
                'scale': {'unit': 'arbitrary', 'status': 'arbitrary'},
                'coordinate_system': {'native': 'synthetic test'}}))
            (folder / 'points.ply').write_text(f'points {count}')
            (folder / 'private.txt').write_text('never export this')
        (self.root / 'live/state.json').write_text(json.dumps({'revision': 2, 'revision_path': 'revisions/000002'}))
        self.manager = Mock()
        self.manager.state.return_value = {'connected': False}
        self.manager.manual_disconnected = False
        self.manager.disconnected_session = None
        self.manager.mapping_session = None
        self.live = SimpleNamespace(current=None)
        self.tello_map = SimpleNamespace(busy=False)
        self.imports = Mock()
        self.server = ViewerServer(('127.0.0.1', 0), partial(ViewerHandler,
            result=self.root / 'saved', session=self.root / 'live', drones=self.manager, imports=self.imports,
            live=self.live, tello_map=self.tello_map))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        self.temp.cleanup()

    def request(self, path, method='GET', headers=None, body=None):
        connection = HTTPConnection('127.0.0.1', self.server.server_port)
        connection.request(method, path, json.dumps(body or {}) if method == 'POST' else None, headers=headers or {})
        response = connection.getresponse()
        data = response.read()
        result = response.status, data, dict(response.getheaders())
        connection.close()
        return result

    def test_export_respects_source_and_preserves_all_history_and_reimports(self):
        for path, expected in [('/api/saved/map.zip', 1), ('/api/live/map.zip?session=live&revision=000001', 3)]:
            status, payload, headers = self.request(path)
            self.assertEqual(status, 200)
            self.assertIn('attachment', headers['Content-Disposition'])
            with zipfile.ZipFile(io.BytesIO(payload)) as archive:
                if expected == 1:
                    self.assertEqual(set(archive.namelist()), {'scene.json', 'status.json', 'points.ply'})
                else:
                    self.assertIn('revisions/000001/scene.json', archive.namelist())
                    self.assertIn('revisions/000002/scene.json', archive.namelist())
                    self.assertEqual([e['revision'] for e in json.loads(archive.read('timeline.json'))['entries']], [1, 2])
                    self.assertFalse(any('private.txt' in name for name in archive.namelist()))
                restored = self.root / f'restored-{expected}'
                archive.extractall(restored)
                imported = ImportedMap(restored)
                self.assertEqual(json.loads((imported.result() / 'scene.json').read_text())['points'][0][1], expected)
                if expected == 3:
                    self.assertEqual(list(imported.revisions), ['000001', '000002'])
                    self.assertEqual([e['map_id'] for e in imported.entries], [1, 2])
                    self.imports.resolve.return_value = imported
                    status, saved_again, _ = self.request('/api/saved/map.zip?asset=history&session=saved-map')
                    self.assertEqual(status, 200)
                    with zipfile.ZipFile(io.BytesIO(saved_again)) as second:
                        self.assertEqual(set(second.namelist()), set(archive.namelist()))
        self.assertEqual(self.request('/api/live/map.zip?session=obsolete')[0], 409)
        self.assertEqual(self.request('/api/live/map.zip?revision=../../secret')[0], 404)
        self.assertEqual(self.request('/api/live/map.zip?revision=999999')[0], 200)

    def test_drone_queries_do_not_mutate_and_starts_require_same_origin_header(self):
        self.assertEqual(self.request('/api/drones/state')[0], 200)
        self.manager.connect.assert_not_called()
        self.assertEqual(self.request('/api/drones/scan', 'POST')[0], 403)
        self.assertEqual(self.request('/api/drones/scan', 'POST', {
            'X-Mapping-Client': '1', 'Origin': 'https://other.example'})[0], 403)
        self.manager.scan.assert_not_called()
        self.manager.scan.return_value = {'items': []}
        self.assertEqual(self.request('/api/drones/scan', 'POST', {'X-Mapping-Client': '1'})[0], 200)
        self.manager.scan.assert_called_once()

    def test_manual_disconnect_waits_for_mapping_and_input_cleanup_before_save(self):
        session = SimpleNamespace(id='live', path=self.root / 'live', input_kind='tello_camera',
            done=False, stop=Mock(), stats=lambda: {})
        self.live.current = session
        path = session.path / 'state.json'
        state = json.loads(path.read_text()) | {'map_available': True}
        path.write_text(json.dumps(state))
        read = lambda: json.loads(self.request('/api/drones/state')[1])
        self.assertFalse(read()['can_save'])
        self.assertEqual(self.request('/api/live/map.zip?session=live')[0], 409)
        headers = {'X-Mapping-Client': '1'}
        self.assertEqual(self.request('/api/drones/disconnect', 'POST', headers, {'session_id': 'old'})[0], 409)
        session.stop.assert_not_called()
        status, payload, _ = self.request('/api/drones/disconnect', 'POST', headers, {'session_id': 'live'})
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(payload)['finalizing'])
        session.stop.assert_called_once_with('user_stop')
        self.assertFalse(read()['can_save'])
        session.done = True
        self.tello_map.busy = True
        self.assertFalse(read()['can_save'])
        self.tello_map.busy = False
        self.assertTrue(read()['can_save'])
        self.assertEqual(self.request('/api/live/map.zip?session=live')[0], 200)
        state['map_available'] = False
        path.write_text(json.dumps(state))
        self.assertFalse(read()['can_save'])
        state['map_available'] = True
        path.write_text(json.dumps(state))
        self.manager.manual_disconnected = False  # unexpected loss / new connection
        self.assertFalse(read()['can_save'])
        self.manager.disconnected_session = None
        self.assertIsNone(read()['mapping_session_id'])
        self.request('/api/drones/disconnect', 'POST', headers)
        self.assertFalse(read()['can_save'])  # old completed map is not this connection's output


class GatewayModeConsoleAPI(unittest.TestCase):
    """--tello-gateway: the control gateway owns the Tello link, so the drone panel reports that link and
    refuses direct SDK/Wi-Fi actions (they would fail on busy ports or switch the PC Wi-Fi)."""

    def setUp(self):
        self.gateway_state = {'type': 'drone_state', 'source': 'tello', 'connected': True, 'mode': 'grounded',
                              'battery_pct': 87, 'video': {'available': False}}
        test = self

        class Gateway(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                body = json.dumps(test.gateway_state).encode()
                self.send_response(200 if self.path == '/api/drone/state' else 404)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
        self.gateway = ThreadingHTTPServer(('127.0.0.1', 0), Gateway)
        threading.Thread(target=self.gateway.serve_forever, daemon=True).start()
        self.gateway_url = f'http://127.0.0.1:{self.gateway.server_port}'
        self.manager = Mock()
        self.manager.state.side_effect = lambda: {'connected': False, 'owner': 'discovery'}   # 실제처럼 매번 새 dict
        self.manager.stop_preview.return_value = None
        self.manager.manual_disconnected = False
        self.manager.disconnected_session = None
        self.manager.mapping_session = None
        self.manager.link = None
        ttl = serve.GATEWAY_DRONE.ttl_s
        serve.GATEWAY_DRONE.ttl_s = 0   # 시험마다 게이트웨이 상태를 바로 다시 읽는다
        self.addCleanup(setattr, serve.GATEWAY_DRONE, 'ttl_s', ttl)
        self.server = ViewerServer(('127.0.0.1', 0), partial(ViewerHandler, drones=self.manager,
            live=SimpleNamespace(current=None), tello_map=SimpleNamespace(busy=False, gateway=self.gateway_url)))
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        if self.gateway:
            self.gateway.shutdown()
            self.gateway.server_close()

    def request(self, path, method='GET', body=None):
        connection = HTTPConnection('127.0.0.1', self.server.server_port)
        connection.request(method, path, json.dumps(body or {}) if method == 'POST' else None,
                           headers={'X-Mapping-Client': '1'} if method == 'POST' else {})
        response = connection.getresponse()
        data = json.loads(response.read())
        connection.close()
        return response.status, data

    def test_drone_state_reports_the_control_gateway_link(self):
        status, state = self.request('/api/drones/state')
        self.assertEqual((status, state['owner'], state['connected']), (200, 'gateway', True))
        self.assertEqual(state['gateway'], {'url': self.gateway_url, 'reachable': True, 'connected': True,
            'source': 'tello', 'battery_pct': 87, 'mode': 'grounded', 'video_available': False})
        self.gateway_state.update(connected=False, battery_pct='bad', video={'available': True})
        state = self.request('/api/drones/state')[1]
        self.assertEqual((state['connected'], state['gateway']['battery_pct'], state['gateway']['video_available']),
                         (False, None, True))
        self.gateway.shutdown()
        self.gateway.server_close()
        self.gateway = None
        state = self.request('/api/drones/state')[1]
        self.assertEqual((state['owner'], state['connected'], state['gateway']['reachable']), ('gateway', False, False))

    def test_direct_drone_actions_are_refused_but_preview_stop_still_works(self):
        for path in ('scan', 'connect', 'explore', 'preview/start'):
            self.assertEqual(self.request(f'/api/drones/{path}', 'POST', {'id': 'current-network'}),
                             (409, {'error': 'gateway_mode_owns_drone'}), path)
        for name in ('scan', 'connect', 'explore', 'start_preview'):
            getattr(self.manager, name).assert_not_called()
        self.assertEqual(self.request('/api/drones/preview/stop', 'POST')[0], 200)
        self.manager.stop_preview.assert_called_once()


if __name__ == '__main__':
    unittest.main()
