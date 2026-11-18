"""Verify exported time pairs against actual media PTS without mapping frames or poses."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from modules.mapping.data.live_recording import SessionRecording
from modules.mapping.data.video_sensor_sync import export_existing_sync, playback_times, sensor_time_at
from test_live_recording import CAMERA, FINAL, encoded_h264


class VideoSensorSyncTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.recording = SessionRecording(self.root, CAMERA, 'tello_camera')
        self.recording.require_sensors()
        self.jpeg = cv2.imencode('.jpg', np.full((240, 320, 3), 80, np.uint8))[1].tobytes()
        self.addCleanup(self.finish)

    def finish(self):
        self.recording.finish_input()
        self.recording.finish_mapping(FINAL)
        self.assertTrue(self.recording.wait(10))

    def capture(self, received, sensor_times, run='source-a', *, clock=True):
        sink = self.recording.open_part('received_jpeg', 'mapping_bridge_receive_monotonic')
        self.recording.sensor_batch({'run_id': run, 'next_sequence': len(sensor_times),
            'samples': [{'sequence': i + 1, 'received_mono': t, 'raw': {'yaw': i}}
                        for i, t in enumerate(sensor_times)]})
        for arrival, source in zip(received, sensor_times):
            sink.offer(self.jpeg, arrival, sensor_clock={'run_id': run, 'timestamp_s': source,
                       'pose': 'must not be exported'} if clock else None)
        sink.close()

    def sync(self):
        return json.loads((self.root / 'video_sensor_sync.json').read_text())

    def test_real_video_pts_and_variable_latency_are_preserved_without_mapping_logs(self):
        self.capture([1000., 1000.04, 1000.10, 1000.30], [55., 55.04, 55.08, 55.12])
        self.finish()
        sync = self.sync()
        times, _ = playback_times(self.root / 'recordings/part_0000.mkv')
        self.assertEqual(sync['status'], 'complete')
        self.assertEqual(sync['parts'][0]['samples'], [list(pair) for pair in zip(times, [55., 55.04, 55.08, 55.12])])
        self.assertAlmostEqual(sensor_time_at(sync, 0, .07)['received_mono'], 55.06)
        self.assertIsNone(sensor_time_at(sync, 0, -1))
        self.assertIsNone(sensor_time_at(sync, 0, 10))
        self.assertEqual((self.root / 'frames.jsonl').stat().st_size, 0)
        self.assertNotIn('pose', json.dumps(sync))
        self.assertNotIn('frame_sequence', json.dumps(sync))
        manifest = json.loads((self.root / 'session.json').read_text())
        self.assertEqual(manifest['assets']['video_sensor_sync'], 'video_sensor_sync.json')
        self.assertEqual(manifest['asset_sha256']['video_sensor_sync.json'],
                         hashlib.sha256((self.root / 'video_sensor_sync.json').read_bytes()).hexdigest())

    def test_reconnect_keeps_source_clocks_separate(self):
        self.capture([1000., 1000.1], [20., 20.1], 'source-a')
        self.capture([1100., 1100.1], [900., 900.1], 'source-b')
        self.finish()
        sync = self.sync()
        self.assertEqual(sync['status'], 'complete')
        self.assertEqual(sensor_time_at(sync, 0, .05), {'source_run_id': 'source-a', 'received_mono': 20.05})
        self.assertEqual(sensor_time_at(sync, 1, .05), {'source_run_id': 'source-b', 'received_mono': 900.05})

    def test_missing_clock_is_reported_instead_of_inventing_an_offset(self):
        self.capture([1000., 1000.1], [20., 20.1], clock=False)
        self.finish()
        self.assertEqual(self.sync()['status'], 'unavailable')
        self.assertIsNone(sensor_time_at(self.sync(), 0, .05))
        self.assertEqual(self.recording.stats()['status'], 'partial')

    def test_lookup_never_bridges_a_missing_clock_sample(self):
        self.recording.sensor_batch({'run_id': 'source-a', 'next_sequence': 1,
            'samples': [{'sequence': 1, 'received_mono': 20., 'raw': {'yaw': 0}}]})
        sink = self.recording.open_part('received_jpeg', 'mapping_bridge_receive_monotonic')
        for i in range(3):
            sink.offer(self.jpeg, 1000. + i * .1,
                       sensor_clock={'run_id': 'source-a', 'timestamp_s': 20. + i * .1} if i != 1 else None)
        sink.close()
        self.finish()
        sync = self.sync()
        self.assertEqual(sync['status'], 'partial')
        self.assertEqual(sync['parts'][0]['coverage']['missing_clock_samples'], 1)
        self.assertIsNone(sensor_time_at(sync, 0, .05))
        self.assertIsNone(sensor_time_at(sync, 0, .15))
        self.assertEqual(sensor_time_at(sync, 0, 0.)['received_mono'], 20.)

    def test_container_timestamp_duplicates_are_not_claimed_to_be_exact(self):
        self.capture([1000., 1000.0001, 1000.2], [20., 20.04, 20.08])
        self.finish()
        sync = self.sync()
        self.assertEqual(sync['status'], 'partial')
        self.assertGreater(sync['parts'][0]['coverage']['duplicate_video_times'], 0)
        self.assertIsNone(sensor_time_at(sync, 0, 0.))
        self.assertIsNone(sensor_time_at(sync, 0, .1))

    def test_direct_h264_is_marked_as_a_packet_receive_approximation(self):
        packets = encoded_h264()
        self.recording.sensor_batch({'run_id': 'direct', 'next_sequence': 1,
            'samples': [{'sequence': 1, 'received_mono': 50., 'raw': {'tof': 80}}]})
        sink = self.recording.open_part('h264', 'pc_udp_receive_monotonic', source_run_id='direct')
        for i, packet in enumerate(packets):
            sink.offer(packet, 50. + i / 30)
        sink.close()
        self.finish()
        sync = self.sync()
        self.assertEqual(sync['status'], 'approximate')
        self.assertEqual(sync['parts'][0]['accuracy'], 'packet_receive_approximation')
        self.assertEqual(sensor_time_at(sync, 0, 0.)['source_run_id'], 'direct')

    def test_existing_capture_exports_only_timing_and_preserves_originals(self):
        clocks = [55., 55.04, 55.08]
        self.capture([1000., 1000.1, 1000.2], clocks)
        self.finish()
        original = self.sync()
        (self.root / 'video_sensor_sync.json').unlink()
        index = self.root / 'recordings/part_0000.jsonl'
        rows = [json.loads(line) for line in index.read_text().splitlines()]
        for row in rows:
            row.pop('sensor_clock', None)
        index.write_text(''.join(json.dumps(row) + '\n' for row in rows))
        (self.root / 'frames.jsonl').write_text(''.join(json.dumps({'event': 'input', 'source': {
            'recording_part': 0, 'recording_frame_index': i, 'video_recorded': True,
            'run_id': 'source-a', 'timestamp_s': t}, 'sensor_pose': ['not exported']}) + '\n'
            for i, t in enumerate(clocks)))
        paths = [self.root / 'sensor.jsonl', self.root / 'frames.jsonl', self.root / 'recordings/part_0000.mkv']
        before = {str(p): p.read_bytes() for p in paths}
        result = export_existing_sync(self.root)
        self.assertEqual(result['parts'][0]['samples'], original['parts'][0]['samples'])
        self.assertNotIn('sensor_pose', json.dumps(result))
        for p in paths:
            self.assertEqual(p.read_bytes(), before[str(p)])
        with self.assertRaisesRegex(ValueError, 'already_exists'):
            export_existing_sync(self.root)

    def test_new_capture_can_regenerate_sync_without_mapping_logs(self):
        self.capture([1000., 1000.1], [55., 55.1])
        self.finish()
        (self.root / 'video_sensor_sync.json').unlink()
        self.assertEqual((self.root / 'frames.jsonl').stat().st_size, 0)
        sync = export_existing_sync(self.root)
        self.assertEqual(sync['status'], 'complete')
        self.assertEqual(sensor_time_at(sync, 0, .05)['received_mono'], 55.05)

    def test_existing_export_refuses_a_running_capture(self):
        with self.assertRaisesRegex(ValueError, 'finalized'):
            export_existing_sync(self.root)
        self.assertFalse((self.root / 'video_sensor_sync.json').exists())

    def test_old_timing_is_not_bound_to_a_changed_video(self):
        self.capture([1000., 1000.1], [55., 55.1])
        self.finish()
        (self.root / 'video_sensor_sync.json').unlink()
        path = self.root / 'session.json'
        manifest = json.loads(path.read_text())
        manifest['recordings'][0]['playback']['sha256'] = '0' * 64
        path.write_text(json.dumps(manifest))
        sync = export_existing_sync(self.root)
        self.assertEqual(sync['status'], 'unavailable')
        self.assertEqual(sync['parts'][0]['reason'], 'video_hash_changed')


if __name__ == '__main__':
    unittest.main()
