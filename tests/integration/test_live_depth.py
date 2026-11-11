"""Live pose/depth admission, display barriers and graceful final integration; no drone."""
import io
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

import cv2
import numpy as np
from PIL import Image

from modules.autonomy.tello_link.sensor_pose import SensorPose
from modules.mapping.data.mapping_history import HistoryReader
from modules.mapping.data.point_fusion import FAR_DEPTH, STANDARD_DEPTH, TOF_DEPTH
from modules.mapping.data.session_import import ImportManager
from modules.mapping.data.stream_mapping import LivePoseFrames, run
from modules.mapping.localization.live_depth import LiveDepthSession
from modules.mapping.localization.orbslam3_live import live_camera
from modules.mapping.localization.tello_input import TelloMapInput


def wait_for(predicate):
    deadline = time.monotonic() + 5
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError('live reconstruction did not reach the expected boundary')
        time.sleep(.01)


class FakeWorker:
    def __init__(self):
        self.loading, self.release = threading.Event(), threading.Event()
        self.tracked = []
        self.tracking_gate = None
        self.tracker_fps = None
        self.pose_overrides = {}
        self.depth_m = 2.
        self.far_strip = False
        self.far_depth_m = 3.

    def run(self, source, output, options, stop_event, on_preview, next_frame):
        owner = self
        class Estimator:
            device = 'test'
            def __init__(self):
                owner.loading.set()
                if not owner.release.wait(5):
                    raise RuntimeError('model gate timeout')
            def estimate(self, rgb):
                depth = np.full(rgb.shape[:2], owner.depth_m, np.float32)
                if owner.far_strip:
                    width = depth.shape[1]
                    depth[:, int(.54 * width):int(.59 * width)] = owner.far_depth_m
                return depth
        class Tracker:
            def __init__(self, output, size, original_size, camera, fps):
                owner.tracker_fps = fps
            def track(self, rgb, stamp):
                owner.tracked.append(stamp)
                if len(owner.tracked) == 1 and owner.tracking_gate:
                    if not owner.tracking_gate.wait(5):
                        raise RuntimeError('tracker gate timeout')
                h, w = rgb.shape[:2]
                pose = np.eye(4); pose[0, 3] = -stamp
                return {'timestamp_s': stamp, 'tracking_state': 2, 'map_id': 0, 'T_cw': pose.tolist(),
                    'depth_anchors': [[w * x, h * y, 3.] for x in (.2, .4, .6, .8) for y in (.3, .6)],
                    **owner.pose_overrides.get(stamp, {})}
            def close(self):
                pass
        return run(source, output, options, estimator_factory=Estimator, tracker_factory=Tracker,
            stop_event=stop_event, on_preview=on_preview, live_receive=next_frame)

    def close(self):
        pass


class LiveDepthTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.manager = ImportManager(self.root)
        self.runtime = self.manager.depth_worker = FakeWorker()
        self.session = LiveDepthSession(self.root, live_camera({'width': 320, 'height': 180, 'fov': 60}),
            'test-image', self.manager)
        output = io.BytesIO(); Image.new('RGB', (320, 180), (120, 100, 80)).save(output, format='JPEG')
        self.jpeg = output.getvalue()

    def tearDown(self):
        self.runtime.release.set()
        self.session.stop('test_cleanup')
        self.session.worker.join(5)
        self.session.recording.wait(5)
        self.manager.shutdown()
        self.temporary.cleanup()

    def test_model_then_each_frame_then_display_and_stop_integrates_all_observations(self):
        self.assertTrue(self.runtime.loading.wait(2))
        state = self.session.stats()
        self.assertFalse(state['transport_ready'])
        self.assertEqual(state['depth_job']['progress']['processed'], 0)
        self.assertEqual(self.runtime.tracked, [])
        self.runtime.release.set()
        wait_for(lambda: self.session.ready)
        results = []
        first = threading.Thread(target=lambda: results.append(self.session.offer(self.jpeg, 0., 0)))
        first.start()
        wait_for(lambda: self.session.job.get('preview', {}).get('revision') == 1)
        second = threading.Thread(target=lambda: results.append(self.session.offer(self.jpeg, .2, 1)))
        second.start()
        time.sleep(.1)
        self.assertEqual(self.runtime.tracked, [0.])
        self.assertTrue(first.is_alive())
        self.assertEqual(self.session.received, 1)
        self.manager.presented(self.session.depth_job_id, 1)
        wait_for(lambda: self.session.job.get('preview', {}).get('revision') == 2)
        self.session.stop('user_stop')
        first.join(3); second.join(3); self.session.worker.join(5)
        self.assertFalse(self.session.worker.is_alive())
        self.assertIsNone(self.session.error)
        self.assertEqual(self.runtime.tracked, [0., .2])
        self.assertEqual(self.session.job['status'], 'complete')
        scene = json.loads((self.session.path / 'reconstruction/scene.json').read_text())
        self.assertEqual(scene['input_run_id'], self.session.id)
        self.assertEqual(scene['depth_mapping']['integrated_frames'], 2)
        self.assertEqual(self.manager.history_timeline(self.session.depth_job_id)['count'], 2)
        self.assertTrue(self.session.stats()['map_available'])
        self.assertTrue(self.session.recording.stats()['finalized'])
        self.assertEqual(self.session.job['record'].warnings, [])
        self.assertEqual(len(self.session.job['record'].media), 1)

    def test_slow_display_does_not_count_as_missing_browser_input(self):
        self.runtime.release.set()
        wait_for(lambda: self.session.ready)
        sender = threading.Thread(target=lambda: self.session.offer(self.jpeg, 0., 0))
        sender.start()
        wait_for(lambda: bool(self.session.preview))
        self.session.last_input = time.monotonic() - 60
        self.manager.presented(self.session.depth_job_id, 1)
        sender.join(2)
        time.sleep(.3)
        self.assertFalse(self.session.closing)
        self.assertTrue(self.session.can_accept_frame)

    def test_tello_receiver_keeps_fresh_preview_but_waits_for_model_and_display(self):
        self.session.input_kind = 'tello_camera'
        session, jpeg = self.session, self.jpeg
        class Source:
            closed = False
            def open(self, cancelled):
                pass
            def frames(self, cancelled):
                while not cancelled():
                    yield jpeg, time.perf_counter()
                    time.sleep(.005)
            def close(self):
                self.closed = True
        source = Source()
        adapter = TelloMapInput(None)
        adapter.lease_until = time.monotonic() + 45
        receiver = threading.Thread(target=adapter._run, args=(session, source))
        receiver.start()
        try:
            wait_for(lambda: session.raw_received >= 5)
            self.assertEqual(session.received, 0)
            self.assertIsNotNone(adapter.latest(session.id))
            self.runtime.release.set()
            wait_for(lambda: bool(session.preview))
            before = session.raw_received
            time.sleep(.1)
            self.assertGreater(session.raw_received, before)
            self.assertEqual(session.received, 1)
            self.assertEqual(self.runtime.tracked, [0.])
            self.assertGreater(session.sampled_out, 5)
            self.assertEqual(session.dropped, 0)
        finally:
            session.stop('user_stop')
            receiver.join(3)
        self.assertFalse(receiver.is_alive())
        self.assertTrue(source.closed)

    def test_stop_during_model_preparation_does_not_admit_input(self):
        self.assertTrue(self.runtime.loading.wait(2))
        self.session.stop('user_stop')
        self.runtime.release.set()
        self.session.worker.join(5)
        self.assertTrue(self.session.done)
        self.assertFalse(self.session.ready)
        self.assertEqual(self.runtime.tracked, [])
        self.assertIsNone(self.session.error)
        self.assertFalse(self.session.stats()['map_available'])

    def test_front_tof_stays_with_its_frame_and_corrects_only_the_forward_surface(self):
        self.session.stop('test_reconfigure')
        self.runtime.release.set()
        self.session.worker.join(5)
        self.session.recording.wait(5)
        self.runtime.depth_m = 1.
        self.runtime.far_strip = True
        runtime = self.runtime
        self.session = LiveDepthSession(self.root, self.session.camera, 'test-image', self.manager,
            input_kind='tello_camera', mapping={'live_policy': 'continuous'})
        output = self.session.path / 'reconstruction'

        class Source:
            def open(self, cancelled):
                pass

            def frames(self, cancelled):
                for sequence, distance in enumerate([.8] * 5 + [.88, .88, .8]):
                    stamp = 100. + sequence * .6
                    # The penultimate sample is stale. The final image shows only far depth.
                    if sequence == 7:
                        runtime.depth_m = 3.
                    self.front_tof = (distance, stamp - .3 if sequence == 6 else stamp)
                    yield jpeg, stamp
                    wait_for(lambda: session.preview and session.preview['revision'] == sequence + 1)
                session.stop('user_stop')

            def close(self):
                pass

        session, jpeg = self.session, self.jpeg
        adapter = TelloMapInput(None)
        adapter.lease_until = time.monotonic() + 45
        wait_for(lambda: session.ready)
        adapter._run(session, Source())
        session.worker.join(5)
        self.assertIsNone(session.error)
        history = output / 'history'
        entries = [json.loads(line) for line in (history / 'index.jsonl').read_text().splitlines()]
        self.assertFalse(entries[3]['front_tof']['corrected'])
        self.assertTrue(entries[5]['front_tof']['corrected'])
        self.assertIsNone(entries[6]['front_tof']['distance_m'])
        self.assertEqual(entries[7]['front_tof']['distance_m'], .8)
        self.assertFalse(entries[7]['front_tof']['surface_supported'])
        self.assertTrue(entries[7]['pose_confirmed'])
        with np.load(history / 'frames/00000006.npz') as frame:
            self.assertTrue(bool(frame['tof_verified']))
            depth = frame['depth']
            self.assertAlmostEqual(float(depth[depth.shape[0] // 2, depth.shape[1] // 2]), 3.3, places=3)
            self.assertEqual(float(depth[0, 0]), 0.)
            self.assertEqual(float(depth[depth.shape[0] // 2, int(.56 * depth.shape[1])]), 0.)
            self.assertAlmostEqual(float(frame['world_from_camera'][0, 3]), 3.)
        with np.load(history / 'frames/00000007.npz') as frame:
            self.assertGreater(float(frame['depth'][0, 0]), 0.)
            self.assertNotIn('tof_verified', frame.files)
            self.assertNotIn('frame_preview_points', frame.files)
        with np.load(history / 'frames/00000008.npz') as frame:
            self.assertGreater(float(frame['depth'][0, 0]), 0.)
            self.assertNotIn('tof_verified', frame.files)
            self.assertNotIn('frame_preview_points', frame.files)
        scene = json.loads((output / 'scene.json').read_text())
        self.assertTrue(any(point[3] < -3.2 for point in scene['points']))
        self.assertEqual(len(scene['rescue']['point_tof']), len(scene['points']))
        self.assertEqual(set(scene['rescue']['point_tof']), {False, True})
        replay = self.manager.history_snapshot(session.depth_job_id, 8)
        self.assertEqual(len(replay['rescue']['point_tof']), len(replay['points']))
        self.assertEqual(replay['rescue']['point_tof'], scene['rescue']['point_tof'])
        np.testing.assert_allclose(np.asarray(replay['points'])[:, :3] * [1, -1, -1],
                                   np.asarray(scene['points'])[:, 1:4], atol=1e-6)
        summary = scene['depth_mapping']['front_tof_by_map']['0']
        self.assertEqual((summary['paired_frames'], summary['corrected_frames']), (7, 1))
        self.assertAlmostEqual(summary['meters_per_slam_unit'], .8 / 3, delta=.003)

    def test_capture_log_keeps_model_wait_frames_and_links_the_final_camera_pose(self):
        self.session.stop('test_reconfigure')
        self.runtime.release.set()
        self.session.worker.join(5)
        self.session.recording.wait(5)
        runtime = self.runtime = self.manager.depth_worker = FakeWorker()
        session = self.session = LiveDepthSession(self.root, self.session.camera, 'test-image', self.manager,
            input_kind='tello_camera', mapping={'live_policy': 'continuous'})
        self.assertTrue(runtime.loading.wait(2))
        jpeg = self.jpeg
        class Source:
            sensor_capable = True
            run_id = 'synthetic'
            sensor_pose = SensorPose(1, 0., 0., .8, 0., 0., 0.)
            def open(self, cancelled):
                pass
            def frames(self, cancelled):
                yield jpeg, 100.
                runtime.release.set()
                wait_for(lambda: session.ready)
                yield jpeg, 100.4
                wait_for(lambda: session.preview is not None)
                session.stop('user_stop')
            def close(self):
                pass
        adapter = TelloMapInput(None)
        adapter.lease_until = time.monotonic() + 45
        adapter._run(session, Source())
        session.worker.join(5)
        self.assertTrue(session.recording.wait(5))
        self.assertIsNone(session.error)
        logs = [json.loads(line) for line in (session.path / 'frames.jsonl').read_text().splitlines()]
        inputs = [row for row in logs if row['event'] == 'input']
        mapped = [row for row in logs if row['event'] == 'mapping']
        self.assertEqual([row['admission'] for row in inputs], ['model_wait', 'submitted'])
        self.assertEqual([row['frame_sequence'] for row in inputs], [0, 1])
        self.assertIsNone(inputs[0]['timestamp_s'])
        self.assertEqual(inputs[1]['timestamp_s'], 0.)
        self.assertEqual(mapped[0]['frame_sequence'], 1)
        self.assertTrue(mapped[0]['mapping_applied'])
        self.assertEqual(mapped[0]['pose_source'], 'tello_sensor_odometry')
        with np.load(session.path / 'reconstruction/history/frames/00000001.npz') as frame:
            np.testing.assert_allclose(mapped[0]['world_from_camera'], frame['world_from_camera'])

    def test_continuous_live_policy_keeps_processing_without_a_browser_ack(self):
        self.session.stop('test_reconfigure')
        self.runtime.release.set()
        self.session.worker.join(5)
        self.session.recording.wait(5)
        self.runtime = self.manager.depth_worker = FakeWorker()
        self.session = LiveDepthSession(self.root, self.session.camera, 'test-image', self.manager,
            mapping={'live_policy': 'continuous', 'sample_fps': 2.5})
        self.assertEqual(self.session.job['options']['sample_fps'], 2.5)
        self.assertEqual(json.loads((self.session.path / 'settings.json').read_text())['sample_fps'], 2.5)
        self.runtime.release.set()
        wait_for(lambda: self.session.ready)
        self.session.offer(self.jpeg, 0., 0)
        self.session.offer(self.jpeg, .2, 1)
        wait_for(lambda: len(self.runtime.tracked) == 2)
        wait_for(lambda: self.session.preview is not None)
        self.assertEqual(self.session.job.get('presented_revision', 0), 0)
        self.assertEqual(self.session.stats()['depth_job']['progress']['processing_policy'], 'continuous')
        self.assertFalse(self.manager.preview(self.session.depth_job_id)['needs_presentation'])

    def test_continuous_tello_admits_camera_cadence_and_replaces_only_pending_input(self):
        self.session.stop('test_reconfigure')
        self.runtime.release.set()
        self.session.worker.join(5)
        self.session.recording.wait(5)
        self.runtime = self.manager.depth_worker = FakeWorker()
        self.runtime.tracking_gate = threading.Event()
        camera = {**self.session.camera, 'fps': 24}
        self.session = LiveDepthSession(self.root, camera, 'test-image', self.manager,
            input_kind='tello_camera', mapping={'live_policy': 'continuous'})
        try:
            self.assertTrue(self.runtime.loading.wait(2))
            self.assertFalse(self.session.can_accept_frame)
            self.runtime.release.set()
            wait_for(lambda: self.session.can_accept_frame)
            self.session.offer(self.jpeg, 0., 0)
            wait_for(lambda: self.runtime.tracked == [0.])
            self.assertEqual(self.runtime.tracker_fps, 24)
            self.session.offer(self.jpeg, 1 / 24, 1)
            state = self.session.offer(self.jpeg, 2 / 24, 2)
            self.assertEqual(state['queue_depth'], 1)
            self.assertEqual(state['frame_drops'], 1)
            self.assertEqual(state['transport_received'], 3)
            self.runtime.tracking_gate.set()
            wait_for(lambda: self.runtime.tracked == [0., 2 / 24])
            self.assertEqual(self.session.submitted, 2)
        finally:
            self.runtime.tracking_gate.set()
            self.session.stop('test_cleanup')
            self.session.recording.finish_input()

    def test_existing_live_job_rejects_a_saved_reconstruction(self):
        key = self.manager.create()['id']
        self.manager.job(key).update(status='awaiting_options', summary={'width': 320, 'height': 180}, source=self.root)
        with self.assertRaisesRegex(ValueError, 'reconstruction_busy'):
            self.manager.start(key, {'mode': 'sfm', 'confirmed': True})

    def test_tello_frame_preview_continues_through_unaligned_depth_and_tracking_loss(self):
        self.session.stop('test_reconfigure')
        self.runtime.release.set()
        self.session.worker.join(5)
        self.session.recording.wait(5)
        self.runtime.depth_m = 1.
        self.runtime.pose_overrides = {0.: {'tracking_state': 1}, 1.: {'depth_anchors': []},
            2.25: {'tracking_state': 3}, 2.75: {'map_id': 1, 'depth_anchors': []}, 3.25: {'map_id': 1}}
        session = self.session = LiveDepthSession(self.root, self.session.camera, 'test-image', self.manager,
            input_kind='tello_camera', mapping={'live_policy': 'continuous'})
        wait_for(lambda: session.ready)
        frames = [(0., 'camera'), (.5, None), (1., 'map'), (2.25, 'camera'), (2.75, 'camera'), (3.25, None)]
        for sequence, (stamp, space) in enumerate(frames):
            sample = None if sequence == 3 else (.8, stamp)
            session.offer(self.jpeg, stamp, sequence, front_tof=sample)
            wait_for(lambda: session.preview and session.preview['revision'] == sequence + 1)
            frame = session.preview.get('frame_preview')
            self.assertEqual(frame['space'] if frame else None, space)
            if frame:
                self.assertLessEqual(len(frame['points']), 2048)
                self.assertEqual(frame['tof_verified'], sample is not None)
                expected_depth = 3. if space == 'map' else .8 if sample else 1.
                np.testing.assert_allclose(np.asarray(frame['points'])[:, 2], expected_depth, atol=.02)
                self.assertEqual(session.preview['registered'], int(sequence > 0))
                recorded = self.manager.history_snapshot(session.depth_job_id, sequence + 1)
                self.assertFalse(recorded['observation'])
                self.assertEqual(recorded['frame_preview']['space'], space)
                self.assertEqual(recorded['frame_preview']['tof_verified'], frame['tof_verified'])
                np.testing.assert_allclose(recorded['frame_preview']['points'], frame['points'], atol=1e-6)
        session.stop('user_stop')
        session.recording.finish_input()
        session.worker.join(5)
        self.assertIsNone(session.error)
        scene = json.loads((session.path / 'reconstruction/scene.json').read_text())
        self.assertEqual(scene['depth_mapping']['integrated_frames'], 2)
        self.assertEqual(scene['depth_mapping']['integrated_maps'], 2)

    def test_tello_immediately_accumulates_predicted_pose_and_reuses_last_scale(self):
        self.session.stop('test_reconfigure')
        self.runtime.release.set()
        self.session.worker.join(5)
        self.session.recording.wait(5)
        pose = np.eye(4)
        pose[0, 3] = .25
        pose[:3, :3] = cv2.Rodrigues(np.array([0., .1, 0.]))[0]
        self.runtime.pose_overrides = {.25: {'T_cw': np.linalg.inv(pose).tolist()},
            .27: {'tracking_state': 3}, .5: {'tracking_state': 3},
            1.26: {'tracking_state': 3}, 1.52: {'tracking_state': 3},
            1.76: {'tracking_state': 3, 'map_id': 1}}
        session = self.session = LiveDepthSession(self.root, self.session.camera, 'test-image', self.manager,
            input_kind='tello_camera', mapping={'live_policy': 'continuous'})
        wait_for(lambda: session.ready)
        frames = [(0., 'tracking'), (.25, 'tracking'), (.27, 'pose_predicted'), (.5, 'pose_predicted'),
                  (1.26, 'lost'), (1.5, 'tracking'), (1.52, 'pose_predicted'), (1.76, 'lost')]
        registered = 0
        for sequence, (stamp, state) in enumerate(frames):
            self.runtime.depth_m = 4. if state == 'pose_predicted' else 2.
            session.offer(self.jpeg, stamp, sequence)
            wait_for(lambda: session.preview and session.preview['revision'] == sequence + 1)
            self.assertEqual(session.preview['tracking'], state)
            if state == 'pose_predicted':
                self.assertFalse(session.preview['pose_confirmed'])
                # A return to native tracking clears the pre-loss velocity.
                self.assertAlmostEqual(session.preview['pose_position'][0], stamp if stamp < 1. else 1.5)
                expected_angle = stamp * .4 if stamp < 1. else 0.
                np.testing.assert_allclose(session.preview['pose_rotation'],
                    cv2.Rodrigues(np.array([0., expected_angle, 0.]))[0], atol=1e-6)
                self.assertAlmostEqual(session.preview['depth_scale'], 1.5)
            elif state == 'lost':
                self.assertEqual(session.preview['frame_preview']['space'], 'camera')
            registered += state != 'lost'
            self.assertEqual(session.preview['registered'], registered)
        session.stop('user_stop')
        session.recording.finish_input()
        session.worker.join(5)
        self.assertIsNone(session.error)
        scene = json.loads((session.path / 'reconstruction/scene.json').read_text())
        self.assertEqual(scene['depth_mapping']['integrated_frames'], 6)
        self.assertEqual([c['pose_source'] for c in scene['cameras'] if 'pose_source' in c], ['constant_velocity'] * 3)

    def test_tello_prediction_and_recovery_accumulate_in_one_metric_map(self):
        self.session.stop('test_reconfigure')
        self.runtime.release.set()
        self.session.worker.join(5)
        self.session.recording.wait(5)
        self.runtime.tracker_fps = None
        session = self.session = LiveDepthSession(self.root, self.session.camera, 'test-image', self.manager,
            input_kind='tello_camera', mapping={'live_policy': 'continuous'})
        wait_for(lambda: session.ready)
        texture = np.random.default_rng(71).integers(0, 256, (180, 320, 3), dtype=np.uint8)
        texture = cv2.GaussianBlur(texture, (5, 5), .9)
        fx = session.camera['fx']

        def jpeg_at(x):
            if x is None:
                image = np.zeros_like(texture)
            else:
                image = cv2.warpAffine(texture, np.array([[1., 0., -fx * x / 2], [0., 1., 0.]]), (320, 180))
            return cv2.imencode('.jpg', image)[1].tobytes()

        def odom(segment, right):
            return SensorPose(segment, 0., -right, .8, 0., 0., 0.)

        frames = [
            (0., None, None, None, None, None),
            (.4, 0., odom(0, 0.), None, 'tello_sensor_odometry', 0.),
            (.8, .10, odom(0, .10), None, 'tello_sensor_odometry', .10),
            (1.2, None, None, None, 'tello_linear_prediction', .20),
            (1.6, None, odom(1, 0.), None, 'tello_linear_prediction', .30),
            (2.4, None, odom(1, 0.), None, 'tello_linear_prediction', .50),
            (2.8, .30, odom(1, 0.), None, 'tello_sensor_odometry', .30),
            (3.2, None, None, None, 'tello_linear_prediction', .30),
            (3.6, .38, None, None, 'tello_visual_recovery', .38),
            (4., .50, odom(1, .20), (.8, 4.), 'tello_sensor_odometry', .50),
        ]
        for sequence, (stamp, x, sample, tof, expected, expected_x) in enumerate(frames):
            self.runtime.depth_m = 1. if tof is not None else 2.
            self.runtime.far_strip = tof is not None
            self.runtime.far_depth_m = 4.
            session.offer(jpeg_at(x), stamp, sequence, front_tof=tof, sensor_pose=(0, sample, False))
            wait_for(lambda: session.preview and session.preview['revision'] == sequence + 1)
            preview = session.preview
            self.assertEqual(preview.get('pose_source'), expected)
            if expected:
                self.assertEqual(preview['map_id'], 0)
                self.assertEqual(preview['submaps'], 1)
                self.assertEqual(preview['registered'], sequence)
                self.assertEqual(preview['tracking'] == 'pose_predicted', expected == 'tello_linear_prediction')
                np.testing.assert_allclose(preview['pose_position'], [expected_x, 0., 0.], atol=.04)
                self.assertAlmostEqual(preview['depth_scale'], 1.)
            else:
                self.assertEqual(preview['tracking'], 'sensor_wait')
                self.assertEqual(preview['registered'], 0)
        self.assertTrue(session.preview['front_tof']['used'])
        self.assertTrue(any(session.preview['rescue']['point_far']))
        self.assertTrue(any(session.preview['rescue']['point_tof']))
        session.stop('user_stop')
        session.recording.finish_input()
        session.worker.join(5)
        self.assertIsNone(session.error)
        self.assertEqual(self.runtime.tracked, [])
        self.assertIsNone(self.runtime.tracker_fps)
        output = session.path / 'reconstruction'
        scene = json.loads((output / 'scene.json').read_text())
        self.assertEqual(scene['scale']['unit'], 'm')
        self.assertEqual(scene['depth_mapping']['integrated_maps'], 1)
        self.assertEqual(scene['depth_mapping']['integrated_frames'], 9)
        self.assertEqual(scene['depth_mapping']['submaps'], [])
        self.assertEqual([c['pose_source'] for c in scene['cameras']], [row[4] for row in frames if row[4]])
        self.assertEqual(session.recording.snapshot()['scale'], 'm')
        logged = [json.loads(line) for line in (session.path / 'frames.jsonl').read_text().splitlines()]
        results = [row for row in logged if row['event'] == 'mapping']
        self.assertEqual(len(results), len(frames))
        self.assertFalse(results[0]['mapping_applied'])
        self.assertIsNone(results[0]['world_from_camera'])
        self.assertEqual([row['pose_source'] for row in results[1:]], [row[4] for row in frames[1:]])
        with np.load(output / 'history/frames/00000010.npz', allow_pickle=False) as last:
            np.testing.assert_allclose(results[-1]['world_from_camera'], last['world_from_camera'])
        with np.load(output / 'history/frames/00000010.npz', allow_pickle=False) as frame:
            depth, classes = frame['depth'], frame['depth_classes']
            self.assertEqual(set(np.unique(classes)), {FAR_DEPTH, STANDARD_DEPTH, TOF_DEPTH})
            np.testing.assert_allclose(depth[classes == FAR_DEPTH], 4.)
            np.testing.assert_allclose(depth[classes == STANDARD_DEPTH], 1.)
            self.assertLess(depth[classes == TOF_DEPTH].max(), .9)
        replay = HistoryReader(output).snapshot(10)
        for key in ('point_tof', 'point_far'):
            self.assertEqual(replay['rescue'][key], session.preview['rescue'][key])
            self.assertEqual(len(scene['rescue'][key]), len(scene['points']))
            self.assertTrue(any(scene['rescue'][key]))

    def test_continuous_mapping_preserves_first_prediction_while_depth_is_busy(self):
        frames = iter((self.jpeg, stamp, number) for number, stamp in enumerate((0., .1, .12, .4, .7)))

        class Tracker:
            def __init__(self, *args):
                pass

            def track(self, rgb, stamp):
                return dict(timestamp_s=stamp, tracking_state=2 if stamp <= .1 else 3,
                            map_id=0, T_cw=np.eye(4).tolist())

            def close(self):
                pass

        producer = LivePoseFrames(lambda: next(frames, None), 24., 4., self.root, self.session.camera,
                                  Tracker, continuous=True, predict_pose=True)
        try:
            # Let newer input arrive before the depth consumer asks for either frame.
            wait_for(lambda: producer.ended)
            first, latest = producer.take(), producer.take()
            self.assertEqual(first[1], .12)
            self.assertTrue(first[5]['prediction_start'])
            self.assertEqual(latest[1], .7)
            self.assertFalse(latest[5]['prediction_start'])
            self.assertIsNone(producer.take())
        finally:
            producer.close()
