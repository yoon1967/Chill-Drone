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
from unittest.mock import patch
from types import SimpleNamespace
import zipfile

import numpy as np
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


def make_reconstruction(root):
    """A real portable history and fused result, without a model or SLAM process."""
    from modules.mapping.data.mapping_history import HistoryWriter
    from modules.mapping.data.stream_mapping import finalize_history, write_json
    (root / 'frames').mkdir(parents=True)
    writer = HistoryWriter(root, write_json)
    for revision in (1, 2):
        frame_id = f'frame_{revision:07d}.jpg'
        Image.new('RGB', (32, 24), (revision * 60, 100, 150)).save(root / 'frames' / frame_id)
        pose = np.eye(4); pose[0, 3] = revision / 10
        snapshot = dict(revision=revision, timestamp_s=float(revision), frame_id=frame_id,
            frame_number=revision, tracking='tracking', pose_confirmed=True, pose_map_id=0, map_id=0,
            pose_position=pose[:3, 3].tolist(), pose_rotation=pose[:3, :3].tolist(),
            points=[[revision / 10, 0, 2, 100, 100, 150]], cameras=[pose[:3, 3].tolist()],
            processed=revision, registered=revision, dropped=0, map_points=revision,
            latency_ms=20, processing_ms=10, render_window_s=10)
        writer.append(snapshot, dict(depth=np.full((24, 32), 2., np.float32),
            rgb=np.full((24, 32, 3), 100, np.uint8), world_from_camera=pose,
            intrinsics=np.array([20., 20., 16., 12.]), map_id=np.array(0), depth_scale=np.array(1.)))
    writer.close()
    write(root / 'preview.json', snapshot)
    finalize_history(root, {}, 'portable_history_test', time.monotonic())


class SessionImportTests(unittest.TestCase):
    def test_recent_library_reopens_failed_and_cancelled_attempts_after_restart(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            attempts = {'import_' + 'a' * 16: 'failed', 'import_' + 'b' * 16: 'cancelled'}
            for key, status in attempts.items():
                write(root / key / 'import.json', dict(name='capture.mp4', status=status, stage='depth', error='test'))
            manager = ImportManager(root)
            try:
                self.assertEqual({item['local_id']: item['status'] for item in manager.library()}, attempts)
                for key, status in attempts.items():
                    record = manager.open_local(key)
                    self.assertEqual((record['status'], record['name']), (status, 'capture.mp4'))
                    self.assertFalse(record['source_video'])
                    with self.assertRaisesRegex(ValueError, 'import_not_ready'):
                        manager.resolve(key)
            finally:
                manager.shutdown()

    def test_history_download_roundtrip_keeps_result_replay_and_relative_frames_after_restart(self):
        source = self.root / 'original_reconstruction'
        make_reconstruction(source)
        original_scene = json.loads((source / 'scene.json').read_text(encoding='utf-8'))
        source_key = self.manager.add_local(source)
        self.assertEqual(self.wait(source_key)['status'], 'ready')
        files, metadata = self.manager.history_archive(source_key)
        buffer = BytesIO()
        with zipfile.ZipFile(buffer, 'w') as archive:
            for path, name in files:
                archive.write(path, name)
            for name, value in metadata.items():
                archive.writestr(name, value)
        payload = buffer.getvalue()
        with zipfile.ZipFile(BytesIO(payload)) as archive:
            self.assertEqual(len(archive.namelist()), len(set(archive.namelist())))
            assets = json.loads(archive.read('local_assets.json'))
            self.assertEqual(assets['images_root'], 'frames')
            self.assertEqual(assets['frames'], ['frame_0000001.jpg', 'frame_0000002.jpg'])
        uploaded = self.manager.create('reconstruction-history.zip')['id']
        self.manager.upload(uploaded, 'reconstruction-history.zip', BytesIO(payload), len(payload))
        with patch.object(self.manager, 'start', side_effect=AssertionError('a saved archive must not run reconstruction')):
            self.manager.inspect(uploaded)
            result = self.wait(uploaded)
        self.assertEqual(result['status'], 'ready')
        self.assertTrue(result['history_available'])
        self.assertEqual(result['summary']['history_frames'], 2)
        record = self.manager.resolve(uploaded)
        self.assertEqual(json.loads((record.result() / 'scene.json').read_text()), original_scene)
        self.assertTrue(record.root.is_relative_to(self.manager.job(uploaded)['path']))
        restarted = ImportManager(self.manager.root)
        self.assertTrue(restarted.public(uploaded)['history_available'])
        self.assertEqual(restarted.history_timeline(uploaded)['count'], 2)
        self.assertEqual(restarted.history_snapshot(uploaded, 1)['frame_number'], 1)
        self.assertEqual(restarted.history_snapshot(uploaded, 2)['timestamp_s'], 2.)
        self.assertEqual(restarted.preview(uploaded)['revision'], 2)
        self.assertTrue(restarted.preview_frame(uploaded, 'frame_0000001.jpg').is_file())
        self.assertEqual(restarted.resolve(uploaded).warnings, [])

    def test_incomplete_reconstruction_history_is_rejected_before_offering_replay(self):
        for missing in ('history/frames/00000001.npz', 'frames/frame_0000002.jpg'):
            with self.subTest(missing=missing):
                source = self.root / ('incomplete_' + str(len(self.manager.jobs)))
                make_reconstruction(source)
                (source / missing).unlink()
                key = self.manager.add_local(source)
                result = self.wait(key)
                self.assertEqual(result['status'], 'failed')
                self.assertEqual(result['error'], 'reconstruction_history_incomplete')

    def test_history_archive_requires_a_finished_job_and_saved_result(self):
        source = self.root / 'finished_reconstruction'
        make_reconstruction(source)
        key = self.manager.add_local(source)
        self.assertEqual(self.wait(key)['status'], 'ready')
        job = self.manager.job(key)
        for state in ('running', 'cancelling', 'failed', 'cancelled'):
            job['status'] = state
            with self.subTest(status=state), self.assertRaisesRegex(ValueError, 'reconstruction_result_not_ready'):
                self.manager.history_archive(key)
        job['status'] = 'ready'
        (source / 'scene.json').unlink()
        with self.assertRaisesRegex(ValueError, 'reconstruction_result_not_ready'):
            self.manager.history_archive(key)

    def test_older_history_without_a_result_requests_a_completed_download(self):
        source = self.root / 'unfinished_history'
        make_reconstruction(source)
        (source / 'scene.json').unlink()
        (source / 'status.json').unlink()
        key = self.manager.add_local(source)
        result = self.wait(key)
        self.assertEqual(result['status'], 'failed')
        self.assertEqual(result['error'], 'reconstruction_result_not_ready')

    def test_delete_import_removes_record_but_preserves_external_source(self):
        source = self.root / 'original'
        make_session(source)
        key = self.manager.add_local(source)
        self.assertEqual(self.wait(key)['status'], 'ready')
        folder = self.manager.job(key)['path']
        alias = self.manager.add_local(source)
        self.assertEqual(self.wait(alias)['status'], 'ready')
        self.assertEqual(self.manager.delete_local(key)['status'], 'deleted')
        self.assertFalse(folder.exists())
        self.assertFalse((self.manager.root / alias).exists())
        self.assertTrue((source / 'session.json').exists())
        self.assertEqual(self.manager.library(), [])

    def test_delete_managed_session_and_aliases_without_touching_other_records(self):
        source = self.manager.root / 'managed_session'
        make_session(source)
        key = self.manager.add_local(source)
        self.assertEqual(self.wait(key)['status'], 'ready')
        other = self.manager.root / 'other'
        make_session(other)
        self.manager.delete_local(key)
        self.assertFalse(source.exists())
        self.assertFalse((self.manager.root / key).exists())
        self.assertTrue((other / 'session.json').exists())
        self.assertEqual([row['local_id'] for row in self.manager.library()], ['other'])

    def test_delete_rejects_traversal_and_busy_records(self):
        key = self.manager.create('busy')['id']
        with self.assertRaisesRegex(ValueError, 'record_busy'):
            self.manager.delete_local(key)
        with self.assertRaises(ValueError):
            self.manager.delete_local('../original')
        target = self.manager.root / 'selected'
        make_session(target)
        # Another in-flight import uses this record as its source.
        self.manager.job(key)['source'] = target
        with self.assertRaisesRegex(ValueError, 'record_busy'):
            self.manager.delete_local('selected')
        self.assertTrue(target.exists())
    def test_cancelling_pose_job_stops_transport_before_its_owned_container(self):
        key = self.manager.create('pose cancel')['id']
        job = self.manager.job(key)
        output = job['path'] / 'result'
        write(output / 'pose_tracker/container.json', {'name': 'ssafy-pose-123456abcdef'})
        job.update(status='running', options={'mode': 'depth_rgbd'}, output=output, process=object())
        events = []
        with patch.object(self.manager, '_terminate', side_effect=lambda process: events.append('transport')), \
             patch('modules.mapping.data.session_import.subprocess.run', side_effect=lambda command, **kwargs: (events.append(command[-1]) or SimpleNamespace(returncode=0))):
            self.manager.cancel(key)
        self.assertEqual(events, ['transport', 'ssafy-pose-123456abcdef'])

    def test_running_preview_is_readable_without_publishing_completed_map(self):
        value = self.manager.create('streaming')
        key = value['id']
        job = self.manager.job(key)
        output = job['path'] / 'stream'
        output.mkdir()
        job.update(output=output, status='running', options={'mode': 'depth_rgbd'})
        self.assertTrue(self.manager.preview(key)['waiting'])
        write(output / 'preview.json', {'revision': 2, 'points': [[0, 0, 1, 255, 0, 0]]})
        write(output / 'progress.json', {'status': 'running', 'stage': 'tracking', 'processed': 2})
        self.assertEqual(self.manager.status(key)['progress']['processed'], 2)
        self.assertEqual(self.manager.preview(key)['revision'], 2)
        self.assertTrue(self.manager.preview(key, 2)['unchanged'])
        with self.assertRaisesRegex(ValueError, 'import_not_ready'):
            self.manager.resolve(key)
        with self.assertRaises(ValueError):
            self.manager.preview_frame(key, '../source.mp4')
        job['status'] = 'complete'
        write(output / 'progress.json', {'status': 'complete', 'stage': 'finished', 'processed': 3})
        self.assertEqual(self.manager.status(key)['progress']['processed'], 3)

    def test_status_keeps_sfm_stage_counts_and_fast_reconstruction_progress_separate(self):
        key = self.manager.create('progress integration')['id']
        job = self.manager.job(key)
        output = job['path'] / 'result'
        job.update(output=output, status='running', options={'mode': 'sfm'})
        write(output / 'input/manifest.json', {'selected_frames': 3})
        write(output / 'manifest.json', {'stage': 'features'})
        status = self.manager.status(key)
        self.assertEqual(status['feature_progress']['total'], 3)
        write(output / 'manifest.json', {'stage': 'preview'})
        write(output / 'preview/status.json', {'phase': 'overview', 'completed': 0, 'total': 2})
        status = self.manager.status(key)
        self.assertNotIn('feature_progress', status)
        self.assertEqual(status['progress']['phase'], 'overview')
        job['options'] = {'mode': 'depth_rgbd'}
        write(output / 'progress.json', {'stage': 'integration', 'processed': 4, 'integrated_frames': 3})
        status = self.manager.status(key)
        self.assertEqual(status['stage'], 'integration')
        self.assertEqual(status['progress']['integrated_frames'], 3)
        self.assertNotIn('feature_progress', status)
        job['status'] = 'complete'
        self.assertEqual(self.manager.status(key)['progress']['processed'], 4)

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

    def test_depth_model_is_prepared_once_and_failed_preparation_is_retryable(self):
        from unittest.mock import MagicMock
        from modules.mapping.data.depth_worker import DepthModelCache
        cache = DepthModelCache()
        estimator = MagicMock()
        with patch('modules.mapping.data.stream_mapping.DepthEstimator', return_value=estimator) as factory:
            estimator.prepare.side_effect = [RuntimeError('model unavailable'), None]
            with self.assertRaisesRegex(RuntimeError, 'model unavailable'):
                cache.get()
            self.assertIs(cache.get(), estimator)
            self.assertIs(cache.get(), estimator)
            self.assertEqual(factory.call_count, 2)
            self.assertEqual(estimator.prepare.call_count, 2)

    def test_presentation_acknowledgement_is_bounded_and_cancellation_wakes_waiters(self):
        key = self.manager.create()['id']
        job = self.manager.job(key)
        output = job['path'] / 'output'
        output.mkdir()
        job.update(output=output, status='running', options={'mode': 'depth_rgbd'},
            presentation_sync=True, preview={'revision': 1, 'points': [], 'cameras': []})
        self.assertTrue(self.manager.preview(key)['needs_presentation'])
        for revision in (0, 2, True, '1'):
            with self.assertRaises(ValueError):
                self.manager.presented(key, revision)
        self.manager.presented(key, 1)
        self.assertFalse(self.manager.preview(key, 1)['needs_presentation'])
        done = threading.Event()
        def waiter():
            self.manager.preview(key, 1, wait=2)
            done.set()
        thread = threading.Thread(target=waiter)
        thread.start()
        self.manager.cancel(key)
        job['status'] = 'cancelled'
        with self.manager.preview_changed:
            self.manager.preview_changed.notify_all()
        thread.join(1)
        self.assertTrue(done.is_set())

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

    def test_default_depth_import_preserves_cumulative_preview_option(self):
        options = {'confirmed': True, 'mode': 'depth_rgbd'}
        self.assertIsNone(validate_options(options, {})['render_window_s'])
        self.assertIsNone(validate_options(options | {'render_window_s': None}, {})['render_window_s'])
        self.assertEqual(validate_options(options | {'render_window_s': 10}, {})['render_window_s'], 10)
        for value in (True, 0, -1, '10', float('nan'), float('inf')):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'invalid_render_window_s'):
                validate_options(options | {'render_window_s': value}, {})

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
            connection = HTTPConnection('127.0.0.1', server.server_port)
            connection.request('POST', '/api/import/delete', json.dumps({'local_id': key}),
                               {'Host': f'127.0.0.1:{server.server_port}', 'X-Mapping-Client': '1', 'Content-Type': 'application/json'})
            response = connection.getresponse()
            self.assertEqual(response.status, 202)
            self.assertEqual(json.loads(response.read())['status'], 'deleted')
            connection.close()
            self.assertTrue((root / 'session.json').exists())
            self.assertFalse((self.manager.root / key).exists())
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2)


if __name__ == '__main__':
    unittest.main()
