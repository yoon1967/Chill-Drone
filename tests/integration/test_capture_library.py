"""Capture selection uses actual allowlisted assets and never starts a worker."""
from functools import partial
from http.client import HTTPConnection
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from apps.console.serve import ObservationCache, Timeline, ViewerHandler, ViewerServer
from modules.mapping.data.capture_library import capture_library, select_capture, capture_file, reconstruction_library, open_capture_folder
from modules.mapping.data.session_import import ImportManager


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding='utf-8')


def make_capture(root, name='live_capture', *, finalized=True, sync='complete', day=10, count=1):
    folder = root / name
    parts = []
    for i in range(count):
        base = f'recordings/part_{i:04d}'
        write(folder / (base + '.jsonl'), {'timestamp_s': 123 + i})
        (folder / (base + '.mkv')).write_bytes(b'0123456789')
        (folder / (base + '.mjpeg')).write_bytes(b'not selected')
        parts.append({'id': i, 'index_path': base + '.jsonl', 'raw_path': base + '.mjpeg',
                      'playback': {'path': base + '.mkv', 'duration_s': 10}})
    write(folder / 'session.json', {'schema_version': 'mapping-session/1.0', 'input': 'tello_camera',
        'session_id': name, 'started_at': f'2026-11-{day:02d}T08:00:00+00:00',
        'recording': {'finalized': finalized}, 'recordings': parts, 'assets': {'frame_poses': 'frames.jsonl'}})
    write(folder / 'sensor.jsonl', {'event': 'sample', 'received_mono': 123})
    write(folder / 'video_sensor_sync.json', {'schema_version': 'video-sensor-sync/1.0', 'status': sync})
    write(folder / 'camera.json', {'width': 960})
    write(folder / 'settings.json', {'mode': 'test'})
    write(folder / 'frames.jsonl', {'not': 'selected'})
    write(folder / 'frames/private.jpg', {'not': 'selected'})
    write(folder / 'recordings/unrelated.jsonl', {'not': 'selected'})
    return folder


class CaptureLibraryTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def test_explorer_opens_actual_capture_root_ignoring_the_other_video_shortcut(self):
        write(self.root / '.video-folder.json', {'folder': 'C:/somewhere_else'})
        with patch('modules.mapping.data.capture_library.os.startfile', create=True) as startfile:
            result = open_capture_folder(self.root)
            startfile.assert_called_once_with(str(self.root.resolve()))
            self.assertEqual(result, {'folder': str(self.root.resolve()), 'opened': True})

    def test_checkpoint_captures_sort_by_save_time_but_keep_original_start_time(self):
        folder = make_capture(self.root, 'saved_snapshot', day=9)
        value = json.loads((folder / 'session.json').read_text())
        value['checkpoint'] = {'cutoff_at': '2026-11-12T08:00:00+00:00'}
        write(folder / 'session.json', value)
        make_capture(self.root, 'live_newer_start', day=11)
        result = capture_library(self.root)['items']
        self.assertEqual(result[0]['session_id'], 'saved_snapshot')
        self.assertEqual(result[0]['started_at'], '2026-11-09T08:00:00+00:00')

    def test_allowlist_has_all_video_parts_and_shared_metadata_without_frames_or_raw(self):
        make_capture(self.root, count=2)
        result = select_capture(self.root, 'live_capture')
        self.assertTrue(result['selectable'])
        self.assertFalse(result['reconstruction_supported'])
        self.assertEqual(len(result['parts']), 2)
        names = {file['path'] for file in result['files']}
        self.assertEqual(names, {'session.json', 'sensor.jsonl', 'video_sensor_sync.json', 'camera.json', 'settings.json',
                                'recordings/part_0000.mkv', 'recordings/part_0000.jsonl',
                                'recordings/part_0001.mkv', 'recordings/part_0001.jsonl'})
        self.assertEqual(capture_file(self.root, 'live_capture', 'sensor.jsonl').read_text(),
                         (self.root / 'live_capture/sensor.jsonl').read_text())
        for name in ('frames.jsonl', 'frames/private.jpg', 'recordings/part_0000.mjpeg',
                     'recordings/unrelated.jsonl', '../session.json', 'C:/private', 'recordings/../sensor.jsonl'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                capture_file(self.root, 'live_capture', name)

    def test_discovery_uses_capture_start_time_and_only_drone_sessions(self):
        make_capture(self.root, 'live_old', day=9)
        make_capture(self.root, 'live_new', day=11)
        write(self.root / 'phone/session.json', {'input': 'browser_camera'})
        write(self.root / 'broken/session.json', {})
        value = capture_library(self.root)
        self.assertEqual(value['root_path'], str(self.root.resolve()))
        self.assertEqual([row['session_id'] for row in value['items']], ['live_new', 'live_old'])
        make_capture(self.root, 'live_added', day=12)
        self.assertEqual(capture_library(self.root)['items'][0]['session_id'], 'live_added')

    def test_recording_missing_empty_and_unavailable_sync_cannot_be_selected(self):
        make_capture(self.root, 'live_active', finalized=False)
        folder = make_capture(self.root, 'live_empty')
        (folder / 'sensor.jsonl').write_bytes(b'')
        make_capture(self.root, 'live_bad_sync', sync='unavailable')
        folder = make_capture(self.root, 'live_no_index')
        (folder / 'recordings/part_0000.jsonl').unlink()
        for row in capture_library(self.root)['items']:
            with self.subTest(row=row['session_id']):
                self.assertFalse(row['selectable'])
                with self.assertRaises(ValueError):
                    select_capture(self.root, row['session_id'])
                with self.assertRaises(ValueError):
                    capture_file(self.root, row['session_id'], 'session.json')

    def test_empty_finished_capture_and_resultless_local_folders_do_not_clutter_lists(self):
        folder = make_capture(self.root)
        (folder / 'recordings/part_0000.mkv').unlink()
        self.assertEqual(capture_library(self.root)['items'], [])
        write(self.root / 'live_failed/state.json', {'input': 'tello_camera', 'map_available': False})
        write(self.root / 'saved_map/state.json', {'revision_path': 'revisions/000001'})
        write(self.root / 'saved_map/revisions/000001/scene.json', {'points': []})
        class Imports:
            root = self.root
            def library(this):
                return [{'local_id': name, 'status': 'local'} for name in ('live_failed', 'saved_map')]
        self.assertEqual([row['local_id'] for row in reconstruction_library(Imports())['items']], ['saved_map'])

    def test_partial_and_approximate_are_explicit_but_still_selectable(self):
        for status in ('partial', 'approximate'):
            make_capture(self.root, 'live_' + status, sync=status)
            self.assertEqual(select_capture(self.root, 'live_' + status)['sync_status'], status)

    def test_path_injection_and_invalid_session_identity_are_not_exposed(self):
        folder = make_capture(self.root)
        value = json.loads((folder / 'session.json').read_text())
        value['recordings'][0]['playback']['path'] = 'frames/private.jpg'
        write(folder / 'session.json', value)
        self.assertEqual(capture_library(self.root)['items'], [])
        with self.assertRaisesRegex(ValueError, 'capture_incomplete'):
            select_capture(self.root, 'live_capture')
        for name in ('../live_capture', 'C:/live_capture', '.', None):
            with self.subTest(name=name), self.assertRaises(ValueError):
                select_capture(self.root, name)

    def test_reconstruction_view_excludes_capture_and_its_map_alias_but_preserves_legacy_library(self):
        folder = make_capture(self.root)
        write(self.root / 'import_alias/import.json', {'status': 'ready', 'map_root': str(folder)})
        write(self.root / 'import_video/import.json', {'status': 'complete'})
        class Imports:
            root = self.root
            def library(this):
                return [{'local_id': name} for name in ('live_capture', 'import_alias', 'import_video')]
        manager = Imports()
        self.assertEqual(reconstruction_library(manager), {'items': [{'local_id': 'import_video'}]})
        self.assertEqual(len(manager.library()), 3)

    def test_production_http_serves_selection_and_files_without_creating_import_jobs(self):
        make_capture(self.root)
        manager = ImportManager(self.root)
        self.addCleanup(manager.shutdown)
        server = ViewerServer(('127.0.0.1', 0), partial(ViewerHandler, imports=manager,
            timeline=Timeline(), observations=ObservationCache()))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        def get(path, headers=None):
            connection = HTTPConnection('127.0.0.1', server.server_port)
            connection.request('GET', '/api/import/' + path, headers=headers or {})
            response = connection.getresponse()
            payload = response.read()
            connection.close()
            return response.status, payload
        try:
            with patch.object(manager, 'create', side_effect=AssertionError('selection must not create a job')):
                with patch('modules.mapping.data.capture_library.os.startfile', create=True) as startfile:
                    connection = HTTPConnection('127.0.0.1', server.server_port)
                    connection.request('POST', '/api/import/capture-folder', json.dumps({'path': 'C:/wrong'}),
                        headers={'X-Mapping-Client': '1', 'Content-Type': 'application/json'})
                    response = connection.getresponse()
                    self.assertEqual(response.status, 202)
                    self.assertEqual(json.loads(response.read())['folder'], str(self.root.resolve()))
                    connection.close()
                    startfile.assert_called_once_with(str(self.root.resolve()))
                self.assertEqual(len(json.loads(get('capture-library')[1])['items']), 1)
                self.assertEqual(get('drone-recordings')[0], 200, 'legacy sensor reconstruction API must coexist')
                status, payload = get('capture-bundle?session_id=live_capture')
                self.assertEqual(status, 200)
                self.assertEqual(json.loads(payload)['schema_version'], 'mapping-capture-selection/1.0')
                self.assertEqual(get('capture-file?session_id=live_capture&path=recordings/part_0000.mkv',
                                     {'Range': 'bytes=2-5'}), (206, b'2345'))
                status, payload = get('capture-file?session_id=live_capture&path=sensor.jsonl')
                self.assertEqual((status, json.loads(payload)['event']), (200, 'sample'))
                self.assertEqual(get('capture-file?session_id=live_capture&path=frames.jsonl')[0], 400)
                self.assertEqual(json.loads(get('reconstruction-library')[1])['items'], [])
                (self.root / 'live_capture/sensor.jsonl').unlink()
                self.assertEqual(get('capture-bundle?session_id=live_capture')[0], 400)
                self.assertEqual(manager.jobs, {})
        finally:
            server.shutdown(); server.server_close(); thread.join(2)
