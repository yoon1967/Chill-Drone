"""Synthetic capture only: original bytes, bounded I/O and portable session export."""
from functools import partial
from fractions import Fraction
from io import BytesIO
import hashlib
import json
import socket
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import urlopen
import zipfile

import av
import cv2
import numpy as np

from apps.console.serve import ViewerHandler, ViewerServer
from modules.mapping.data.live_recording import SessionRecording, archive_files

CAMERA = dict(width=320, height=240, fps=30, fx=300., fy=300., cx=160., cy=120., distortion=[0.] * 4)
FINAL = dict(status='complete', session_status='complete', stop_reason='user_stop', map_available=False, revision=0)


def encoded_h264(count=12):
    codec = av.CodecContext.create('libx264', 'w')
    codec.width, codec.height = 320, 240
    codec.pix_fmt, codec.framerate, codec.time_base = 'yuv420p', Fraction(30), Fraction(1, 30)
    codec.options = {'tune': 'zerolatency', 'preset': 'ultrafast', 'g': '10'}
    packets = []
    for index in range(count):
        image = np.full((240, 320, 3), index * 10, np.uint8)
        image[40:100, 20 + index:70 + index] = (0, 200, 255)
        frame = av.VideoFrame.from_ndarray(image, format='bgr24')
        frame.pts = index
        packets.extend(bytes(packet) for packet in codec.encode(frame))
    packets.extend(bytes(packet) for packet in codec.encode(None))
    return packets


class RecordingTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name) / 'live_recording_test'
        self.root.mkdir()
        for name, value in [('camera.json', CAMERA), ('settings.json', {'input': 'synthetic'}), ('state.json', FINAL)]:
            (self.root / name).write_text(json.dumps(value), encoding='utf-8')
        self.recording = SessionRecording(self.root, CAMERA, 'tello_camera')
        self.addCleanup(self.finish)

    def finish(self):
        self.recording.finish_input()
        self.recording.finish_mapping(FINAL)
        self.assertTrue(self.recording.wait(10), 'recording thread failed to finish')

    def manifest(self):
        return json.loads((self.root / 'session.json').read_text(encoding='utf-8'))

    def test_original_bytes_remux_and_input_close_barrier(self):
        packets = encoded_h264()
        sink = self.recording.open_part('h264', 'pc_udp_receive_monotonic')
        for frame, packet in enumerate(packets):
            for offset in range(0, len(packet), 1460):
                self.assertTrue(sink.offer(packet[offset:offset + 1460], 1000 + frame / 30))
        self.recording.set_mapping_origin(1000.1)
        self.recording.finish_mapping(FINAL)
        self.assertFalse(self.recording.wait(.02))
        sink.close()
        self.recording.finish_input()
        self.assertTrue(self.recording.wait(10))
        manifest = self.manifest()
        part = manifest['recordings'][0]
        expected = b''.join(packets)
        self.assertEqual((self.root / part['raw_path']).read_bytes(), expected)
        self.assertEqual(part['sha256'], hashlib.sha256(expected).hexdigest())
        self.assertNotIn('playback_error', part, part)
        self.assertFalse(part['playback']['reencoded'])
        with av.open(str(self.root / part['playback']['path'])) as video:
            frames = list(video.decode(video=0))
        self.assertEqual(len(frames), 12)
        self.assertEqual((frames[0].width, frames[0].height), (320, 240))
        self.assertEqual(manifest['mapping_timestamp_zero_monotonic'], 1000.1)
        self.assertNotIn(str(self.root), json.dumps(manifest))
        self.assertEqual(manifest['recording']['status'], 'complete')

    def test_reconnect_parts_and_received_jpeg_label(self):
        jpeg = cv2.imencode('.jpg', np.full((240, 320, 3), 80, np.uint8))[1].tobytes()
        for start in (20., 32.):
            sink = self.recording.open_part('received_jpeg', 'mapping_bridge_receive_monotonic')
            for index in range(4):
                sink.offer(jpeg, start + index / 24)
            sink.close()
        self.finish()
        manifest = self.manifest()
        self.assertEqual(len(manifest['recordings']), 2)
        self.assertFalse(manifest['recording']['original_h264'])
        for item in manifest['recordings']:
            self.assertEqual((self.root / item['raw_path']).read_bytes(), jpeg * 4)
            self.assertNotIn('playback_error', item)
            with av.open(str(self.root / item['playback']['path'])) as source:
                self.assertEqual(len(list(source.decode(video=0))), 4)
        self.assertEqual(manifest['recordings'][1]['first_timestamp_s'], 32.)

    def test_direct_camera_records_real_receiver_before_jpeg_sampling(self):
        from modules.autonomy.tello_link.fake_tello import FakeTello
        from modules.autonomy.tello_link.tello_link import TelloLink
        from modules.autonomy.tello_link.tello_video import TelloVideo
        from modules.mapping.localization.tello_input import DirectCamera
        def port():
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind(('127.0.0.1', 0))
                return probe.getsockname()[1]
        state_port, video_port = port(), port()
        fake = FakeTello(cmd_port=0, state_port=state_port, video_port=video_port).start()
        self.addCleanup(fake.close)
        source = DirectCamera(
            lambda **kw: TelloLink(host='127.0.0.1', cmd_port=fake._sock.getsockname()[1],
                local_cmd_port=0, state_port=state_port, bind_ip='127.0.0.1', **kw),
            lambda: TelloVideo(port=video_port, bind_ip='127.0.0.1'))
        source.set_recording(self.recording)
        try:
            source.open(lambda: False)
            video = source.video
            frames = source.frames(lambda: False)
            jpeg, stamp = next(frames)
            frames.close()
            self.assertTrue(jpeg.startswith(b'\xff\xd8'))
            self.recording.set_mapping_origin(stamp)
            time.sleep(.15)
        finally:
            source.close()
        self.finish()
        manifest = self.manifest()
        part = manifest['recordings'][0]
        self.assertEqual(part['bytes'], video.bytes)
        self.assertEqual((self.root / part['raw_path']).stat().st_size, video.bytes)
        self.assertEqual(video.packet_sink_errors, 0)
        self.assertTrue(part['capture_complete'])
        self.assertNotIn('playback_error', part, part)
        self.assertEqual((part['playback']['width'], part['playback']['height']), (960, 720))
        self.assertEqual(fake.received, ['command', 'streamon', 'streamoff'])

    def test_queue_overflow_is_nonblocking_and_preserves_prefix(self):
        self.recording.limit = 1024
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        original = Path.open
        class SlowFile:
            def __init__(self, stream): self.stream = stream
            def write(self, data):
                entered.set()
                if not release.wait(3): raise TimeoutError('test gate')
                return self.stream.write(data)
            def __getattr__(self, name): return getattr(self.stream, name)
        def opening(path, *args, **kwargs):
            result = original(path, *args, **kwargs)
            return SlowFile(result) if path.suffix == '.h264' and args == ('xb',) else result
        with patch.object(Path, 'open', opening):
            sink = self.recording.open_part('h264', 'pc_udp_receive_monotonic')
            self.assertTrue(sink.offer(b'a' * 600, 1.))
            self.assertTrue(entered.wait(2))
            started = time.perf_counter()
            self.assertFalse(sink.offer(b'b' * 600, 2.))
            self.assertLess(time.perf_counter() - started, .1)
            self.assertLessEqual(self.recording.stats()['queued_bytes'], 1024)
            release.set()
            sink.close()
            self.finish()
        part = self.manifest()['recordings'][0]
        self.assertEqual((self.root / part['raw_path']).read_bytes(), b'a' * 600)
        self.assertEqual(self.recording.stats()['status'], 'partial')
        self.assertIn('queue_overflow', self.recording.stats()['error'])

    def test_disk_write_failure_does_not_escape_receiver(self):
        original = Path.open
        class BrokenFile:
            def __init__(self, stream): self.stream = stream
            def write(self, data): raise OSError('synthetic disk full')
            def __getattr__(self, name): return getattr(self.stream, name)
        def opening(path, *args, **kwargs):
            result = original(path, *args, **kwargs)
            return BrokenFile(result) if path.suffix == '.h264' and args == ('xb',) else result
        with patch.object(Path, 'open', opening):
            sink = self.recording.open_part('h264', 'pc_udp_receive_monotonic')
            sink.offer(b'preserve mapping', 1.)
            self.finish()
        self.assertTrue(self.recording.stats()['finalized'])
        self.assertIn('recording_write_OSError', self.recording.stats()['error'])
        self.assertEqual(self.manifest()['mapping']['status'], 'complete')

    def test_empty_session_manifest(self):
        self.finish()
        value = self.manifest()
        self.assertEqual(value['recordings'], [])
        self.assertFalse(value['mapping']['map_available'])
        self.assertTrue(value['recording']['finalized'])
        self.assertEqual(value['assets']['camera'], 'camera.json')

    def test_http_session_export_and_identity(self):
        server = ViewerServer(('127.0.0.1', 0), partial(ViewerHandler, session=self.root,
            live=SimpleNamespace(current=None)))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        base = f'http://127.0.0.1:{server.server_port}/api/live/'
        with self.assertRaises(HTTPError) as error:
            urlopen(base + 'session-download?session=wrong')
        self.assertEqual(error.exception.code, 409)
        with self.assertRaises(HTTPError) as error:
            urlopen(base + 'session-download?session=' + self.root.name)
        self.assertEqual(error.exception.code, 409)
        self.finish()
        (self.root / 'not-finished.mp4.tmp').write_bytes(b'not an export')
        with urlopen(base + 'session-download?session=' + self.root.name) as response:
            self.assertEqual(response.headers['Content-Type'], 'application/zip')
            data = response.read()
        with zipfile.ZipFile(BytesIO(data)) as archive:
            self.assertIn(self.root.name + '/camera.json', archive.namelist())
            self.assertIn(self.root.name + '/session.json', archive.namelist())
            self.assertNotIn(self.root.name + '/not-finished.mp4.tmp', archive.namelist())
            self.assertTrue(json.loads(archive.read(self.root.name + '/session.json'))['recording']['finalized'])
        with urlopen(base + 'session?session=' + self.root.name) as response:
            self.assertEqual(json.load(response)['schema_version'], 'mapping-session/1.0')
        (self.root / 'session.json').write_text('[]', encoding='utf-8')
        with self.assertRaises(HTTPError) as error:
            urlopen(base + 'session-download?session=' + self.root.name)
        self.assertEqual(error.exception.code, 400)

    def test_failed_manifest_write_can_export_live_diagnostics(self):
        with patch('modules.mapping.data.live_recording.atomic_json', side_effect=OSError('synthetic manifest failure')):
            self.finish()
        self.assertFalse(self.manifest()['recording']['finalized'])
        current = SimpleNamespace(path=self.root, id=self.root.name, done=True, recording=self.recording)
        server = ViewerServer(('127.0.0.1', 0), partial(ViewerHandler, live=SimpleNamespace(current=current)))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        with urlopen(f'http://127.0.0.1:{server.server_port}/api/live/session-download?session={self.root.name}') as response:
            data = response.read()
        with zipfile.ZipFile(BytesIO(data)) as archive:
            latest = json.loads(archive.read(self.root.name + '/session.json'))
            self.assertTrue(latest['recording']['finalized'])
            self.assertIn('session_manifest_OSError', latest['recording']['error'])
            self.assertEqual(archive.namelist().count(self.root.name + '/session.json'), 1)

    def test_h264_without_usable_display_timestamps_preserves_the_original(self):
        # Ordinary frame reordering needs source PTS, unavailable on raw Tello UDP.
        codec = av.CodecContext.create('libx264', 'w')
        codec.width, codec.height, codec.pix_fmt = 320, 240, 'yuv420p'
        codec.framerate, codec.time_base = Fraction(30), Fraction(1, 30)
        codec.options = {'preset': 'ultrafast', 'bf': '2', 'g': '10'}
        packets = []
        for index in range(15):
            frame = av.VideoFrame.from_ndarray(np.full((240, 320, 3), index * 10, np.uint8), format='bgr24')
            frame.pts = index
            packets.extend(bytes(packet) for packet in codec.encode(frame))
        packets.extend(bytes(packet) for packet in codec.encode(None))
        sink = self.recording.open_part('h264', 'pc_udp_receive_monotonic')
        for index, packet in enumerate(packets):
            sink.offer(packet, 10 + index / 30)
        sink.close()
        self.finish()
        part = self.manifest()['recordings'][0]
        self.assertEqual((self.root / part['raw_path']).read_bytes(), b''.join(packets))
        self.assertIn('b_frame_timestamps_unavailable', part.get('playback_error', ''))
        self.assertEqual(self.recording.stats()['playback_errors'], 1)

    def test_session_export_does_not_follow_links(self):
        outside = self.root.parent / 'outside.txt'
        outside.write_text('must stay outside the export')
        link = self.root / 'outside-link.txt'
        try:
            link.symlink_to(outside)
        except OSError:
            self.skipTest('Windows symlink privilege not available')
        self.assertNotIn('outside-link.txt', [relative for _, relative in archive_files(self.root)])


if __name__ == '__main__':
    unittest.main()
