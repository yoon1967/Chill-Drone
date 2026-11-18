"""Recorded frame/sensor synchronization and metric reconstruction regressions."""
from fractions import Fraction
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import av
import numpy as np

from modules.mapping.data.drone_recording import DroneRecording, RawPoses
from modules.mapping.data.frame_input import validate_options
from modules.mapping.data.reconstruction_models import DEPTH_V2
from modules.mapping.data.session_import import ImportManager
from modules.mapping.data.stream_mapping import run
from modules.mapping.localization.tello_pose import TelloMapPose


def write(path, value):
    path.write_text(json.dumps(value), encoding='utf-8')


def recording(root, mode='received_jpeg'):
    root.mkdir()
    (root / 'recordings').mkdir()
    camera = dict(width=64, height=48, fx=50., fy=50., cx=32., cy=24.)
    write(root / 'camera.json', camera)
    write(root / 'session.json', dict(schema_version='mapping-session/1.0', input='tello_camera',
        recording={'finalized': True}, recordings=[dict(id=0, mode=mode, playback=dict(
            path='recordings/part_0000.mkv', width=64, height=48, frames=6, first_timestamp_s=100., duration_s=.6))]))
    with av.open(str(root / 'recordings/part_0000.mkv'), 'w') as output:
        stream = output.add_stream('mjpeg', rate=10)
        stream.width, stream.height, stream.pix_fmt = 64, 48, 'yuvj420p'
        for index in range(6):
            frame = av.VideoFrame.from_ndarray(np.full((48, 64, 3), 120, np.uint8), format='rgb24')
            frame.pts, frame.time_base = index * 100, Fraction(1, 1000)
            for packet in stream.encode(frame):
                output.mux(packet)
        for packet in stream.encode():
            output.mux(packet)
    frames, samples = [], []
    for index in range(6):
        stamp = 100 + index / 10
        frames.append(dict(event='input', frame_sequence=index, sensor_epoch=0,
            sensor_pose=[0, 5. + index / 10, 0., 1., 0., 0., 0.],
            source=dict(run_id='run', connection=0, recording_part=0, recording_frame_index=index,
                        video_recorded=True, timestamp_s=stamp, receive_timestamp_s=stamp)))
        samples.append(dict(event='sample', source_run_id='run', received_mono=stamp,
            raw=dict(vgx=-10., vgy=0., vgz=0., yaw=0., pitch=0., roll=0., tof=100.)))
    for name, values in [('frames', frames), ('sensor', samples)]:
        (root / f'{name}.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in values), encoding='utf-8')
    return frames, samples


class DroneRecordingTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / 'live_test'
        self.frames, self.samples = recording(self.source)

    def test_recording_index_uses_input_poses_even_for_unmapped_frames(self):
        with (self.source / 'frames.jsonl').open('a') as file:
            file.write(json.dumps(dict(event='mapping', frame_sequence=0, sensor_pose=None)) + '\n')
        packets = list(DroneRecording(self.source).packets(5, 64, lambda: False))
        self.assertEqual([packet[0] for packet in packets], [1, 3, 5])
        self.assertEqual([packet[5]['tello_telemetry'][1].x for packet in packets], [5., 5.2, 5.4])

    def test_raw_sensor_fallback_stays_in_a_separate_origin(self):
        for row in self.frames:
            row['sensor_pose'] = None
        (self.source / 'frames.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in self.frames))
        packets = list(DroneRecording(self.source).packets(5, 64, lambda: False))
        self.assertAlmostEqual(packets[-1][5]['tello_telemetry'][1].x, .4)
        self.assertNotEqual(packets[0][5]['tello_telemetry'][0], ('run', 0, 0))

    def test_sensor_gaps_are_not_filled_by_unbounded_pose_prediction(self):
        raw = RawPoses([(100., self.samples[0]['raw']), (101., self.samples[1]['raw'])])
        self.assertIsNotNone(raw.at(100.))
        self.assertIsNone(raw.at(100.5))
        self.assertEqual(raw.at(101.).segment, 1)
        resolver = TelloMapPose(max_prediction_age=.2)
        image = np.zeros((48, 64, 3), np.uint8)
        telemetry = list(DroneRecording(self.source).packets(5, 64, lambda: False))[0][5]['tello_telemetry']
        self.assertIsNotNone(resolver.resolve(telemetry, image, (50, 50, 32, 24), 0.)[0])
        self.assertIsNone(resolver.resolve((telemetry[0], None, False), image, (50, 50, 32, 24), 1.)[0])

    def test_h264_playback_uses_packet_receive_time_to_replay_raw_sensor_values(self):
        manifest = json.loads((self.source / 'session.json').read_text())
        manifest['recordings'][0]['mode'] = 'h264'
        write(self.source / 'session.json', manifest)
        for row in self.frames:
            del row['source']['recording_frame_index']
            row['source']['video_match'] = 'h264_receive_time_approximation'
            row['front_tof'] = [.8, row['source']['timestamp_s']]
        (self.source / 'frames.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in self.frames))
        packets = list(DroneRecording(self.source).packets(5, 64, lambda: False))
        self.assertAlmostEqual(packets[-1][5]['tello_telemetry'][1].x, .4)
        self.assertAlmostEqual(packets[-1][-1][0], .8)
        self.assertAlmostEqual(packets[-1][-1][1], .4)

    def test_a_changed_video_cannot_silently_shift_sensor_poses(self):
        self.frames[0]['source']['receive_timestamp_s'] += .1
        (self.source / 'frames.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in self.frames))
        with self.assertRaisesRegex(ValueError, 'drone_recording_invalid_timing'):
            list(DroneRecording(self.source).packets(5, 64, lambda: False))

    def test_a_missing_logged_pose_does_not_switch_to_an_unaligned_raw_origin(self):
        self.frames[2]['sensor_pose'] = None
        (self.source / 'frames.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in self.frames))
        packets = list(DroneRecording(self.source).packets(5, 64, lambda: False))
        self.assertIsNone(packets[1][5]['tello_telemetry'][1])
        self.assertEqual(packets[0][5]['tello_telemetry'][0], packets[1][5]['tello_telemetry'][0])

    def test_cancellation_closes_video_and_unavailable_parts_are_reported(self):
        manifest = json.loads((self.source / 'session.json').read_text())
        manifest['recordings'].append(dict(id=1, mode='h264', bytes=100))
        write(self.source / 'session.json', manifest)
        source = DroneRecording(self.source)
        self.assertEqual(source.summary()['unavailable_parts'], 1)
        self.assertEqual(list(source.packets(5, 64, lambda: True)), [])

    def test_selected_drone_session_uses_its_own_camera_and_only_checks_depth(self):
        manager = ImportManager(self.root / 'imports', self.root)
        self.addCleanup(manager.shutdown)
        job = manager.open_drone_recording('live_test')
        self.assertEqual(job['summary']['kind'], 'drone_recording')
        options = dict(confirmed=True, mode='depth_rgbd', depth_model=DEPTH_V2)
        with patch.object(manager.depth_worker, 'run', side_effect=InterruptedError('test finished')) as worker, \
                patch.object(manager.models, 'status', side_effect=AssertionError('pose model was requested')), \
                patch('modules.mapping.data.reconstruction_models.depth_location', return_value=self.root):
            manager.start(job['id'], options)
            manager.job(job['id'])['thread'].join(5)
        worker.assert_called_once()
        self.assertEqual(worker.call_args.args[0], self.source)
        self.assertEqual(worker.call_args.args[2]['camera']['fx'], 50.)
        validated = validate_options({**options, 'camera': DroneRecording(self.source).camera}, job['summary'])
        self.assertEqual(validated['pose_source'], 'drone_recording')
        self.assertNotIn('pose_model', validated)
        with self.assertRaisesRegex(ValueError, 'drone_recording_requires_sensor_pose'):
            validate_options({**options, 'camera': DroneRecording(self.source).camera, 'causal_video': True}, job['summary'])

    def test_sensor_reconstruction_produces_metric_history_and_previews_without_pose_models(self):
        estimator = MagicMock(device='test')
        estimator.estimate.side_effect = lambda rgb: np.full(rgb.shape[:2], 2., np.float32)
        previews = []
        tracker = MagicMock(side_effect=AssertionError('a pose model was started'))
        output = self.root / 'result'
        status = run(self.source, output, dict(pose_source='drone_recording', frame_fps=10., sample_fps=5.),
                     estimator_factory=lambda: estimator, tracker_factory=tracker, on_preview=previews.append)
        self.assertEqual((status['registered'], len(previews)), (3, 3))
        self.assertTrue(all(frame['mapping_applied'] for frame in previews))
        scene = json.loads((output / 'scene.json').read_text())
        self.assertEqual(scene['scale']['unit'], 'm')
        self.assertEqual(scene['pose_source'], 'tello_sensor_odometry')
        self.assertEqual(scene['depth_mapping']['input_kind'], 'drone_recording')
        self.assertTrue(scene['points'])
        tracker.assert_not_called()


if __name__ == '__main__':
    unittest.main()
