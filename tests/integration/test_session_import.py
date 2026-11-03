"""Portable imported maps reuse saved/live routes without trusting input paths."""
from functools import partial
from http.client import HTTPConnection
from io import BytesIO
import json
import sqlite3
from pathlib import Path
import tempfile
import threading
import time
import unittest
import zipfile

from PIL import Image

from apps.console.serve import ObservationCache, Timeline, ViewerHandler, ViewerServer
from modules.mapping.data.session_import import ImportManager, ImportedMap, extract_zip, relative_path, feature_progress
from modules.mapping.data.frame_input import inspect_input, prepare_frames, validate_options


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding='utf-8')


def make_session(root):
    write(root / 'session.json', {'schema_version': 'mapping-session/1.0', 'session_id': 'live_original',
        'recording': {'finalized': True}, 'recordings': []})
    write(root / 'state.json', {'session_id': 'live_original', 'revision_path': 'revisions/000002'})
    (root / 'frames').mkdir()
    Image.new('RGB', (64, 48)).save(root / 'frames/key.jpg')
    for revision in (1, 2):
        result = root / f'revisions/{revision:06d}'
        write(result / 'scene.json', {'schema_version': 'phone-sfm-result/1.2', 'input_run_id': 'live_original',
            'revision': revision, 'segment_id': revision - 1, 'pose_source': 'orb_slam3',
            'scale': {'unit': 'arbitrary'}, 'coordinate_system': {}, 'points': [[1, revision, 0, 0, 100, 100, 100, 0, 3]],
            'cameras': [{'frame_id': 'key.jpg', 'timestamp_s': revision, 'position': [0, 0, 0]}], 'slam': {'timestamp_s': revision}})
        write(result / 'status.json', {'status': 'partial'})
        write(result / 'local_assets.json', {'images_root': 'C:/old-pc/private', 'frames': ['key.jpg']})


class SessionImportTests(unittest.TestCase):
    def test_feature_progress_reads_only_committed_complete_frames(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            write(root / 'input/manifest.json', {'selected_frames': 3})
            self.assertEqual(feature_progress(root), {'phase': 'extraction', 'completed': None, 'total': 3})
            self.assertFalse((root / 'features/database.db').exists())
            (root / 'features').mkdir()
            connection = sqlite3.connect(root / 'features/database.db')
            try:
                connection.executescript('CREATE TABLE keypoints (image_id INTEGER); CREATE TABLE descriptors (image_id INTEGER); CREATE TABLE two_view_geometries (pair_id INTEGER);')
                connection.executemany('INSERT INTO keypoints VALUES (?)', [(1,), (2,), (3,)])
                connection.execute('INSERT INTO descriptors VALUES (1)')
                connection.commit()
                self.assertEqual(feature_progress(root), {'phase': 'extraction', 'completed': 1, 'total': 3})
                connection.execute('INSERT INTO descriptors VALUES (2)')
                self.assertEqual(feature_progress(root)['completed'], 1)
                connection.execute('INSERT INTO descriptors VALUES (3)')
                connection.commit()
                self.assertEqual(feature_progress(root), {'phase': 'matching', 'completed': 3, 'total': 3})
                (root / 'features/pairs.txt').write_text('one two\ntwo three\n', encoding='utf-8')
                connection.execute('INSERT INTO two_view_geometries VALUES (1)')
                connection.commit()
                self.assertEqual(feature_progress(root)['pairs_completed'], 1)
                self.assertEqual(feature_progress(root)['pairs_total'], 2)
                write(root / 'features/status.json', {'phase': 'export'})
                self.assertEqual(feature_progress(root)['phase'], 'export')
            finally:
                connection.close()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.manager = ImportManager(self.root / 'library')

    def tearDown(self):
        self.manager.shutdown()
        for job in self.manager.jobs.values():
            if job.get('thread'):
                job['thread'].join(5)
        self.temp.cleanup()

    def wait(self, key):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            job = self.manager.status(key)
            if job['status'] not in {'inspecting', 'running', 'uploading', 'cancelling'}:
                return job
            time.sleep(.01)
        self.fail('import did not finish')

    def test_moved_session_keeps_identity_history_segments_and_relative_images(self):
        root = self.root / 'renamed_session'
        make_session(root)
        record = ImportedMap(root)
        self.assertEqual(record.state['session_id'], 'live_original')
        self.assertEqual(record.state['revision'], 2)
        self.assertEqual([e['map_id'] for e in record.entries], [0, 1])
        self.assertEqual(record.image_root(record.result()), root / 'frames')
        self.assertEqual(record.warnings, [])

    def test_library_thumbnail_uses_registered_relative_image_and_rejects_other_paths(self):
        source = self.manager.root / 'saved_session'
        make_session(source)
        payload = self.manager.thumbnail('saved_session')
        with Image.open(BytesIO(payload)) as image:
            self.assertEqual(image.format, 'JPEG')
            self.assertLessEqual(image.width, 480)
        self.assertGreater(self.manager.library()[0]['updated_at'], 0)
        for name in ('../saved_session', 'missing', str(source)):
            with self.assertRaises(ValueError):
                self.manager.thumbnail(name)
        (source / 'frames/key.jpg').unlink()
        with self.assertRaises(OSError):
            self.manager.thumbnail('saved_session')

    def test_recent_library_lists_one_card_for_reopened_local_map(self):
        source = self.manager.root / 'saved_session'
        make_session(source)
        first = self.manager.add_local(source)
        self.wait(first)
        second = self.manager.add_local(source)
        self.wait(second)
        self.assertEqual(len(self.manager.library()), 1)
        restarted = ImportManager(self.manager.root)
        self.assertEqual(len(restarted.library()), 1)

    def test_original_video_restart_is_confined_to_owned_import_folder(self):
        source = self.manager.root / 'saved_session'
        make_session(source)
        key = self.manager.add_local(source)
        self.wait(key)
        job = self.manager.job(key)
        video = job['path'] / 'bundle/original.mp4'
        video.write_bytes(b'original bytes')
        job['source'] = video
        self.manager._save(job)
        restarted = ImportManager(self.manager.root)
        self.assertTrue(restarted.public(key)['source_video'])
        self.assertEqual(restarted.source_video(key), video)
        outside = self.root / 'unregistered.mp4'
        outside.write_bytes(b'private')
        restarted.job(key)['source'] = str(outside)
        with self.assertRaises(ValueError):
            restarted.source_video(key)

    def test_corrupt_latest_preserves_previous_and_missing_images_only_warn(self):
        root = self.root / 'session'
        make_session(root)
        (root / 'revisions/000002/scene.json').write_text('{broken')
        (root / 'frames/key.jpg').unlink()
        record = ImportedMap(root)
        self.assertEqual(record.state['revision'], 1)
        self.assertEqual(len(record.warnings), 2)

    def test_zip_roundtrip_upload_and_restart_reopen(self):
        source = self.root / 'old_pc'
        make_session(source)
        buffer = BytesIO()
        with zipfile.ZipFile(buffer, 'w') as archive:
            for path in source.rglob('*'):
                if path.is_file():
                    archive.write(path, 'moved/' + path.relative_to(source).as_posix())
        key = self.manager.create()['id']
        payload = buffer.getvalue()
        self.manager.upload(key, 'saved.zip', BytesIO(payload), len(payload))
        self.manager.inspect(key)
        self.assertEqual(self.wait(key)['status'], 'ready')
        self.assertEqual(self.manager.resolve(key).state['revision'], 2)
        restarted = ImportManager(self.manager.root)
        self.assertEqual(restarted.resolve(key).state['session_id'], 'live_original')

    def test_zip_and_upload_paths_cannot_escape_or_alias_on_windows(self):
        for name in ('../escape', '/abs', 'C:/escape', 'a\\b', 'a/../b', 'CON.txt', 'folder./a', 'a:stream'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                relative_path(name)
        archive = self.root / 'evil.zip'
        with zipfile.ZipFile(archive, 'w') as out:
            out.writestr('../outside', b'x')
        with self.assertRaises(ValueError):
            extract_zip(archive, self.root / 'extracted')
        self.assertFalse((self.root / 'outside').exists())

    def test_casefold_duplicates_and_zip_links_are_rejected(self):
        archive = self.root / 'evil.zip'
        with zipfile.ZipFile(archive, 'w') as out:
            out.writestr('A.txt', b'1')
            out.writestr('a.txt', b'2')
        with self.assertRaises(ValueError):
            extract_zip(archive, self.root / 'duplicate')
        with zipfile.ZipFile(archive, 'w') as out:
            info = zipfile.ZipInfo('link')
            info.external_attr = 0o120777 << 16
            out.writestr(info, 'outside')
        with self.assertRaises(ValueError):
            extract_zip(archive, self.root / 'linked')

    def test_cancellation_does_not_overwrite_source_or_accept_more_uploads(self):
        key = self.manager.create()['id']
        self.manager.upload(key, 'test.bin', BytesIO(b'first'), 5)
        self.assertEqual(self.manager.cancel(key)['status'], 'cancelled')
        with self.assertRaises(ValueError):
            self.manager.upload(key, 'test.bin', BytesIO(b'second'), 6)
        self.assertEqual((self.manager.root / key / 'bundle/test.bin').read_bytes(), b'first')

    def test_nonfinite_geometry_and_unknown_session_schema_are_rejected(self):
        root = self.root / 'session'
        make_session(root)
        write(root / 'session.json', {'schema_version': 'future/99'})
        with self.assertRaisesRegex(ValueError, 'unsupported_session_schema'):
            ImportedMap(root)

    def test_frame_adapter_orders_numerically_and_passes_existing_feature_validator(self):
        from modules.mapping.data.phone_mapping.feature_input import load_feature_input
        source = self.root / 'photos'
        source.mkdir()
        for name in ('img10.png', 'img2.png', 'img1.png'):
            Image.new('RGB', (64, 48)).save(source / name)
        info = inspect_input(source)
        self.assertEqual(info['order'], ['img1.png', 'img2.png', 'img10.png'])
        output = self.root / 'input'
        prepare_frames(source, output, 'frames_test', 2)
        value = load_feature_input(output)
        self.assertEqual([f['sourcePtsSeconds'] for f in value['selected']], [0, .5, 1])
        self.assertEqual(value['width'], 64)

    def test_reconstruction_requires_explicit_confirmation_and_valid_fps(self):
        with self.assertRaises(ValueError):
            validate_options({}, {})
        for value in (float('nan'), 0, 500):
            with self.assertRaises(ValueError):
                validate_options({'confirmed': True, 'frame_fps': value}, {})

    def test_calibration_accepts_full_resolution_and_rejects_other_crop(self):
        camera = {'width': 1920, 'height': 1080, 'fx': 1500, 'fy': 1500, 'cx': 960, 'cy': 540, 'distortion': [0, 0, 0, 0]}
        options = validate_options({'confirmed': True, 'camera': camera}, {'width': 1920, 'height': 1080})
        self.assertEqual(options['camera']['fx'], 1500)
        with self.assertRaisesRegex(ValueError, 'dimensions_must_match'):
            validate_options({'confirmed': True, 'camera': camera}, {'width': 960, 'height': 540})

    def test_legacy_cmd_revision_drop_keeps_images_and_selected_version(self):
        root = self.root / 'session'
        make_session(root)
        key = self.manager.add_local(root / 'revisions/000001')
        self.assertEqual(self.wait(key)['status'], 'ready')
        record = self.manager.resolve(key)
        self.assertEqual(record.state['revision'], 1)
        self.assertEqual(record.image_root(record.result()), root / 'frames')

    def test_exponential_nonfinite_json_is_rejected(self):
        from modules.mapping.data.session_import import read_json
        path = self.root / 'overflow.json'
        path.write_text('{"x": 1e9999}')
        with self.assertRaisesRegex(ValueError, 'non_finite_json'):
            read_json(path)

    def test_broken_root_metadata_can_recover_from_consistent_revisions(self):
        root = self.root / 'moved'
        make_session(root)
        (root / 'session.json').write_text('{broken')
        (root / 'state.json').write_text('{broken')
        record = ImportedMap(root)
        self.assertEqual(record.state['session_id'], 'live_original')
        self.assertEqual(record.state['revision'], 2)
        self.assertEqual(len(record.warnings), 2)

    def test_changed_recording_warns_and_does_not_hide_valid_map(self):
        import hashlib
        root = self.root / 'session'
        make_session(root)
        write(root / 'session.json', {'schema_version': 'mapping-session/1.0', 'session_id': 'live_original',
            'recording': {'finalized': True}, 'recordings': [{'id': 0, 'raw_path': 'recordings/raw.h264',
                'sha256': hashlib.sha256(b'expected').hexdigest()}]})
        (root / 'recordings').mkdir()
        (root / 'recordings/raw.h264').write_bytes(b'changed')
        record = ImportedMap(root)
        self.assertTrue(record.state['map_available'])
        self.assertTrue(any('무결성' in w for w in record.warnings))

    def test_saved_import_uses_existing_routes_without_replacing_live_source(self):
        root = self.root / 'session'
        make_session(root)
        key = self.manager.add_local(root)
        self.assertEqual(self.wait(key)['status'], 'ready')
        server = ViewerServer(('127.0.0.1', 0), partial(ViewerHandler, imports=self.manager,
            timeline=Timeline(), observations=ObservationCache()))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        def get(path, headers=None):
            connection = HTTPConnection('127.0.0.1', server.server_port)
            connection.request('GET', path, headers=headers or {})
            response = connection.getresponse()
            value = response.read()
            connection.close()
            return response.status, value
        try:
            status, payload = get('/api/saved/state?asset=' + key)
            state = json.loads(payload)
            self.assertEqual((status, state['source'], state['revision']), (200, 'saved', 2))
            self.assertTrue(state['map_available'])
            status, payload = get('/api/saved/run?asset=' + key + '&session=live_original&revision=000001')
            scene = json.loads(payload)
            self.assertEqual((status, scene['revision'], scene['source']), (200, 1, 'saved'))
            self.assertEqual(get('/api/saved/frame/key.jpg?asset=' + key)[0], 200)
            status, thumbnail = get('/api/import/thumbnail?local_id=' + key)
            self.assertEqual(status, 200)
            self.assertEqual(Image.open(BytesIO(thumbnail)).format, 'JPEG')
            self.assertEqual(get('/api/import/thumbnail?local_id=..%2Fsession')[0], 400)
            video = self.manager.job(key)['path'] / 'bundle/source.mp4'
            video.write_bytes(b'0123456789')
            self.manager.job(key)['source'] = video
            self.assertEqual(get('/api/import/source-video?id=' + key, {'Range': 'bytes=2-5'}), (206, b'2345'))
            self.assertEqual(get('/api/import/source-video?id=missing')[0], 400)
            self.assertEqual(json.loads(get('/api/live/state')[1])['status'], 'idle')
            self.assertEqual(get('/api/saved/state?asset=' + key + '&session=other')[0], 409)
            self.assertEqual(get('/api/saved/run?asset=' + key + '&revision=999999')[0], 503)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2)


if __name__ == '__main__':
    unittest.main()
