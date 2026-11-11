"""A save is a cumulative prefix, while video and sensor input keep running."""
from functools import partial
from http.client import HTTPConnection
from io import BytesIO
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import av
import numpy as np
from PIL import Image

from apps.console.serve import ViewerHandler, ViewerServer, Timeline, ObservationCache
from modules.mapping.data.live_checkpoint import capture_map, copy_prefix
from modules.mapping.data.live_recording import SessionRecording, atomic_json, file_hash
from modules.mapping.data.mapping_history import HistoryWriter
from modules.mapping.data.session_import import ImportedMap

CAMERA = dict(width=32, height=24, fps=10, fx=20, fy=20, cx=16, cy=12)


class LiveCheckpointTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / 'live_source'
        self.root.mkdir()
        atomic_json(self.root / 'camera.json', CAMERA)
        atomic_json(self.root / 'settings.json', {'test': True})
        self.state = {'status': 'receiving', 'session_status': 'receiving', 'input': 'tello_camera',
            'revision': 2, 'backend': 'orb_slam3_depth', 'map_available': False}
        atomic_json(self.root / 'state.json', self.state)
        self.recording = SessionRecording(self.root, CAMERA, 'tello_camera')
        self.recording.require_sensors()
        self.sink = self.recording.open_part('received_jpeg', 'gateway_receive')
        self.output = self.root / 'reconstruction'
        (self.output / 'frames').mkdir(parents=True)
        self.history = HistoryWriter(self.output, atomic_json, synchronous=True)
        self.session = SimpleNamespace(path=self.root, id=self.root.name, recording=self.recording,
            condition=threading.Condition(), stats=lambda: dict(self.state), preview={'revision': 2})
        for revision in (1, 2):
            self.append_map(revision)
            self.append_video(revision)
        self.addCleanup(self.finish)

    def finish(self):
        self.history.close()
        self.sink.close()
        self.recording.finish_input()
        self.recording.finish_mapping(self.state)
        self.assertTrue(self.recording.wait(10))
        for job in self.recording.checkpoints:
            self.wait_job(job['id'])

    def append_map(self, revision):
        frame_id = f'frame_{revision:07d}.jpg'
        Image.new('RGB', (32, 24)).save(self.output / 'frames' / frame_id)
        pose = np.eye(4); pose[0, 3] = revision / 10
        snapshot = dict(revision=revision, timestamp_s=float(revision), frame_id=frame_id,
            frame_number=revision, tracking='tracking', pose_confirmed=True, pose_source='tello_sensor_odometry',
            map_unit='m', pose_map_id=0, map_id=0, points=[[revision, 0, 2, 100, 100, 100]], cameras=[[revision, 0, 0]])
        self.history.append(snapshot, dict(depth=np.full((24, 32), 2., np.float32),
            rgb=np.full((24, 32, 3), 100, np.uint8), world_from_camera=pose, intrinsics=np.array([20., 20., 16., 12.]),
            map_id=np.array(0), depth_scale=np.array(1.)))
        self.session.preview = {'revision': revision}
        self.state['revision'] = revision

    def append_video(self, number):
        image = BytesIO()
        Image.new('RGB', (32, 24), (number * 20, 100, 150)).save(image, format='JPEG')
        self.assertTrue(self.sink.offer(image.getvalue(), float(number),
            sensor_clock={'run_id': 'sensor_run', 'timestamp_s': 1000. + number}))
        self.recording.log('sensor', {'event': 'sample', 'source_run_id': 'sensor_run',
                                     'sequence': number, 'received_mono': 1000. + number})

    def wait_job(self, key):
        end = time.monotonic() + 15
        while time.monotonic() < end:
            job = self.recording.checkpoint_status(key)
            if job['status'] != 'saving':
                return job
            time.sleep(.02)
        self.fail('checkpoint worker did not finish')

    def test_repeated_saves_include_start_to_each_click_without_stopping_receiver(self):
        entered, release = threading.Event(), threading.Event()
        def blocked(*args):
            entered.set()
            self.assertTrue(release.wait(5))
            return copy_prefix(*args)
        with patch('modules.mapping.data.live_checkpoint.copy_prefix', side_effect=blocked):
            first = self.recording.checkpoint(capture_map(self.session))
            self.assertTrue(entered.wait(5))
            with self.assertRaisesRegex(ValueError, 'checkpoint_busy'):
                self.recording.checkpoint(capture_map(self.session))
            self.append_map(3)
            self.append_video(3)
            self.append_video(4)
            end = time.monotonic() + 2
            while self.recording.sensor_samples_written < 4 and time.monotonic() < end:
                time.sleep(.01)
            self.assertEqual(self.recording.sensor_samples_written, 4, 'export must not block the writer')
            self.assertFalse(self.recording.finishing)
            self.assertFalse(self.recording.input_closed)
            release.set()
            first = self.wait_job(first['id'])
        self.assertEqual(first['status'], 'complete', first)
        first_root = Path(first['path'])
        hashes = {name: file_hash(first_root / name) for name in
                  ('sensor.jsonl', 'recordings/part_0000.mkv', 'reconstruction/scene.json')}
        self.append_map(4)
        self.append_video(5)
        second = self.wait_job(self.recording.checkpoint(capture_map(self.session))['id'])
        self.assertEqual(second['status'], 'complete', second)
        for saved, videos, maps in ((first, 2, 2), (second, 5, 4)):
            folder = Path(saved['path'])
            manifest = json.loads((folder / 'session.json').read_text())
            self.assertTrue(manifest['recording']['finalized'])
            self.assertEqual(manifest['recording']['status'], 'complete')
            self.assertEqual(manifest['recording']['sensor_samples'], videos)
            self.assertEqual(len((folder / 'sensor.jsonl').read_text().splitlines()), videos)
            self.assertEqual(manifest['checkpoint']['map_revision'], maps)
            with av.open(str(folder / 'recordings/part_0000.mkv')) as video:
                self.assertEqual(len(list(video.decode(video=0))), videos)
            sync = json.loads((folder / 'video_sensor_sync.json').read_text())
            self.assertEqual(len(sync['parts'][0]['samples']), videos)
            self.assertEqual(ImportedMap(folder).summary()['points'] > 0, True)
            scene = json.loads((folder / 'reconstruction/scene.json').read_text())
            self.assertEqual(scene['depth_mapping']['integrated_frames'], maps)
        self.assertEqual({name: file_hash(first_root / name) for name in hashes}, hashes)
        self.assertFalse(self.recording.finalized)

    def test_export_disk_failure_does_not_disable_original_and_next_save_can_succeed(self):
        with patch('modules.mapping.data.live_checkpoint.copy_prefix', side_effect=OSError('disk unavailable')):
            failed = self.wait_job(self.recording.checkpoint(capture_map(self.session))['id'])
        self.assertEqual(failed['status'], 'failed')
        self.assertFalse(self.recording.disabled)
        self.assertIsNone(self.recording.error)
        self.append_video(3)
        result = self.wait_job(self.recording.checkpoint(capture_map(self.session))['id'])
        self.assertEqual(result['status'], 'complete', result)

    def test_direct_h264_prefix_has_playback_and_explicit_approximate_sensor_time(self):
        from test_live_recording import encoded_h264
        packets = encoded_h264(4)
        sink = self.recording.open_part('h264', 'pc_receive_monotonic', source_run_id='sensor_run')
        for index, packet in enumerate(packets):
            self.assertTrue(sink.offer(packet, 1001. + index / 30))
        result = self.wait_job(self.recording.checkpoint(capture_map(self.session))['id'])
        self.assertEqual(result['status'], 'complete', result)
        folder = Path(result['path'])
        with av.open(str(folder / 'recordings/part_0001.mp4')) as video:
            self.assertEqual(len(list(video.decode(video=0))), 4)
        sync = json.loads((folder / 'video_sensor_sync.json').read_text())
        self.assertEqual(sync['status'], 'approximate')
        self.assertEqual(sync['parts'][1]['source_run_id'], 'sensor_run')
        sink.close()

    def test_native_scene_is_pinned_and_no_map_is_reported_before_first_commit(self):
        native = self.root / 'native_test'
        (native / 'revisions/000001').mkdir(parents=True)
        (native / 'frames').mkdir()
        Image.new('RGB', (32, 24)).save(native / 'frames/key.jpg')
        scene = {'schema_version': 'phone-sfm-result/1.2', 'input_run_id': 'native_test',
            'points': [[0, 0, 0, 1, 100, 100, 100]], 'scale': {'unit': 'arbitrary'},
            'cameras': [{'frame_id': 'key.jpg', 'position': [0, 0, 0]}], 'quality': {'result_status': 'partial'}}
        atomic_json(native / 'revisions/000001/scene.json', scene)
        from modules.mapping.localization.orbslam3_live import LiveSession
        session = LiveSession.__new__(LiveSession)
        session.__dict__.update(path=native, condition=threading.Condition(), id='native_test',
            ready=True, closing=False, done=False, error=None, camera={}, received=2, submitted=2,
            dropped=0, pending=None, reason=None, last_pts=1, last_sequence=1)
        atomic_json(native / 'state.json', {'revision': 1, 'revision_path': 'revisions/000001', 'map_available': True})
        self.assertNotIn('revision_path', session.stats())
        pin = capture_map(session)
        self.assertEqual(pin['revision'], 1)
        scene['points'] = []
        atomic_json(native / 'revisions/000001/scene.json', scene)
        result = self.wait_job(self.recording.checkpoint(pin)['id'])
        self.assertEqual(result['status'], 'complete', result)
        self.assertTrue(json.loads((Path(result['path']) / 'reconstruction/scene.json').read_text())['points'])
        atomic_json(native / 'state.json', {'revision': 0, 'map_available': False})
        with self.assertRaisesRegex(ValueError, 'checkpoint_map_not_ready'):
            capture_map(session)

    def test_http_checkpoint_accepts_only_configured_controller_origin_and_keeps_session_live(self):
        server = ViewerServer(('127.0.0.1', 0), partial(ViewerHandler,
            live=SimpleNamespace(current=self.session), tello_map=SimpleNamespace(gateway='http://127.0.0.1:8010'),
            timeline=Timeline(), observations=ObservationCache()))
        thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
        def request(method, path, origin, body=None):
            connection = HTTPConnection('127.0.0.1', server.server_port)
            connection.request(method, path, body=json.dumps(body) if body is not None else None,
                headers={'Origin': origin, 'X-Mapping-Client': '1', 'Content-Type': 'application/json'})
            response = connection.getresponse()
            result = (response.status, response.getheader('Access-Control-Allow-Origin'), response.read())
            connection.close()
            return result
        try:
            status, origin, _ = request('OPTIONS', '/api/live/checkpoint', 'http://127.0.0.1:8010')
            self.assertEqual((status, origin), (204, 'http://127.0.0.1:8010'))
            self.assertEqual(request('OPTIONS', '/api/live/stop', 'http://127.0.0.1:8010')[0], 403)
            self.assertEqual(request('POST', '/api/live/checkpoint', 'https://untrusted.example', {})[0], 403)
            self.assertEqual(request('POST', '/api/live/checkpoint', 'http://127.0.0.1:8010',
                                     {'session_id': 'another'})[0], 400)
            status, origin, data = request('POST', '/api/live/checkpoint', 'http://127.0.0.1:8010',
                                           {'session_id': self.session.id})
            self.assertEqual(status, 202, data)
            self.assertEqual(origin, 'http://127.0.0.1:8010')
            job = self.wait_job(json.loads(data)['id'])
            self.assertEqual(job['status'], 'complete', job)
            status, origin, data = request('GET', '/api/live/checkpoint-status', 'http://127.0.0.1:8010')
            self.assertEqual(status, 200)
            value = json.loads(data)
            self.assertEqual(value['latest']['id'], job['id'])
            self.assertTrue(value['available'])
            self.assertFalse(self.recording.finishing)
        finally:
            server.shutdown(); server.server_close(); thread.join(2)
