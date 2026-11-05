"""Live pose/depth admission, display barriers and graceful final integration; no drone."""
import io
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

import numpy as np
from PIL import Image

from modules.mapping.data.session_import import ImportManager
from modules.mapping.data.stream_mapping import run
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

    def run(self, source, output, options, stop_event, on_preview, next_frame):
        owner = self
        class Estimator:
            device = 'test'
            def __init__(self):
                owner.loading.set()
                if not owner.release.wait(5):
                    raise RuntimeError('model gate timeout')
            def estimate(self, rgb):
                return np.full(rgb.shape[:2], 2., np.float32)
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
                    'depth_anchors': [[w * x, h * y, 3.] for x in (.2, .4, .6, .8) for y in (.3, .6)]}
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
