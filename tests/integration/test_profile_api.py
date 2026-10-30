"""Profile HTTP binding with real session metadata; only Docker execution is replaced."""
from functools import partial
from http.client import HTTPConnection
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from apps.console.serve import ViewerHandler, ViewerServer, Timeline, ObservationCache
from modules.mapping.data.camera_profiles import CameraProfiles, BUILTIN_ID
from modules.mapping.data.frame_input import validate_options
from modules.mapping.localization.orbslam3_live import LiveManager, LiveSession


class ProfileAPITests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.profiles = CameraProfiles(self.root / 'profiles')
        (self.root / 'live').mkdir()
        self.live = LiveManager(self.root / 'live')
        self.imports = Mock()
        self.imports.job.return_value = {'summary': {'width': 1920, 'height': 1080}}
        self.imports.start.side_effect = lambda key, body: {'options': validate_options(body, {'width': 1920, 'height': 1080})}
        self.tello = Mock(camera=self.profiles.get(BUILTIN_ID), gateway=None)
        self.tello.start.return_value = {'session_id': 'test_tello'}
        self.server = ViewerServer(('127.0.0.1', 0), partial(ViewerHandler, profiles=self.profiles,
            live=self.live, imports=self.imports, tello_map=self.tello, timeline=Timeline(), observations=ObservationCache()))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.body = {'camera': {'width': 1920, 'height': 1080, 'fx': 1200., 'fy': 1190., 'cx': 960., 'cy': 540., 'distortion': [0, 0, 0, 0]},
            'name': 'Camera A', 'camera_identity': 'device A', 'image_geometry': 'full frame landscape',
            'input_modes': ['camera', 'file'], 'confirmed': True}

    def tearDown(self):
        if self.live.current:
            session = self.live.current
            session.recording.finish_input()
            session.recording.finish_mapping({'status': 'complete'})
            session.recording.wait(timeout=5)
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(2)
        self.temp.cleanup()

    def request(self, path, body=None, headers=None):
        connection = HTTPConnection('127.0.0.1', self.server.server_port)
        connection.request('GET' if body is None else 'POST', path,
            body=None if body is None else json.dumps(body),
            headers=headers if headers is not None else {'X-Mapping-Client': '1', 'Content-Type': 'application/json'})
        response = connection.getresponse()
        payload = response.read()
        value = json.loads(payload) if response.getheader('Content-Type', '').startswith('application/json') else payload.decode()
        response_headers = dict(response.getheaders())
        connection.close()
        return response.status, value, response_headers

    def register(self):
        status, camera, _ = self.request('/api/camera-profiles', self.body)
        self.assertEqual(status, 201)
        selection = {k: camera['profile'][k] for k in ('id', 'version', 'camera_sha256')} | {'confirmed': True}
        return camera, selection

    def test_register_export_default_and_restart_keep_the_profile(self):
        camera, selection = self.register()
        status, exported, headers = self.request('/api/camera-profiles/export?id=' + selection['id'])
        self.assertEqual(status, 200)
        self.assertEqual(exported, camera)
        self.assertIn('attachment;', headers['Content-Disposition'])
        status, data, _ = self.request('/api/camera-profiles/default', {'mode': 'camera', 'id': selection['id']})
        self.assertEqual(status, 200)
        self.assertEqual(data['defaults']['camera'], selection['id'])
        self.assertEqual(CameraProfiles(self.profiles.root).list()['defaults']['camera'], selection['id'])

    def test_live_session_records_selected_profile_and_scaled_actual_values(self):
        camera, selection = self.register()
        config = {'width': 960, 'height': 540, 'fps': 30, 'source_width': 1920, 'source_height': 1080,
                  'camera': {'fx': -99}, 'profile_selection': selection}
        with patch('modules.mapping.localization.orbslam3_live.require_slam_image', return_value='synthetic'), \
             patch.object(LiveSession, '_run', return_value=None):
            status, data, _ = self.request('/api/live/start', config)
        self.assertEqual(status, 201)
        session = self.live.current
        stored = json.loads((session.path / 'camera.json').read_text(encoding='utf-8'))
        settings = json.loads((session.path / 'settings.json').read_text(encoding='utf-8'))
        self.assertEqual(stored['fx'], 600)
        self.assertEqual(stored['profile']['id'], selection['id'])
        self.assertEqual(stored['profile']['source_width'], 1920)
        self.assertEqual(settings['camera'], stored)
        before = (session.path / 'camera.json').read_bytes()
        self.request('/api/camera-profiles/default', {'mode': 'camera', 'id': None})
        self.assertEqual((session.path / 'camera.json').read_bytes(), before)
        self.assertEqual(self.profiles.get(selection['id']), camera)

    def test_unconfirmed_or_mismatched_input_never_starts_a_session(self):
        _, selection = self.register()
        with patch('modules.mapping.localization.orbslam3_live.require_slam_image') as docker:
            for change in ({'profile_selection': {**selection, 'confirmed': False}}, {'source_width': 960}):
                config = {'width': 960, 'height': 540, 'fps': 30, 'source_width': 1920,
                    'source_height': 1080, 'profile_selection': selection} | change
                status, _, _ = self.request('/api/live/start', config)
                self.assertEqual(status, 400)
            docker.assert_not_called()
        self.assertIsNone(self.live.current)

    def test_file_reconstruction_uses_same_server_profile_and_rejects_other_modes(self):
        camera, selection = self.register()
        status, data, _ = self.request('/api/import/start?id=test', {'confirmed': True, 'profile_selection': selection})
        self.assertEqual(status, 202)
        self.assertEqual(data['options']['camera']['profile']['id'], selection['id'])
        status, _, _ = self.request('/api/live/tello/start', {'profile_selection': selection})
        self.assertEqual(status, 400)
        self.tello.start.assert_not_called()

    def test_mutations_keep_existing_origin_header_requirement(self):
        status, _, _ = self.request('/api/camera-profiles', self.body, headers={})
        self.assertEqual(status, 403)


if __name__ == '__main__':
    unittest.main()
