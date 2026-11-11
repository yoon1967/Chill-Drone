"""Production gateway routes -> mapping receiver -> persisted sensor/video files."""
import json
from contextlib import ExitStack
from pathlib import Path
import socket
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.request import ProxyHandler, build_opener

from modules.autonomy.tello_link import gateway_server
from modules.autonomy.tello_link.fake_tello import FakeTello
from modules.autonomy.tello_link.gateway import Gateway
from modules.autonomy.tello_link.tello_link import TelloLink
from modules.autonomy.tello_link.tello_video import TelloVideo
from modules.mapping.data.live_recording import SessionRecording
from modules.mapping.localization.tello_input import DEFAULT_CAMERA, GatewayCamera


def free_port(kind=socket.SOCK_STREAM):
    with socket.socket(socket.AF_INET, kind) as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


def wait_for(predicate):
    until = time.monotonic() + 5
    while not predicate():
        if time.monotonic() > until:
            raise AssertionError('local production gateway did not become ready')
        time.sleep(.02)


class GatewaySensorRecordingTests(unittest.TestCase):
    def record(self, *, omit_status_run_id=False):
        with tempfile.TemporaryDirectory() as directory, ExitStack() as cleanup:
            root = Path(directory)
            state_port, video_port = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_DGRAM)
            fake = FakeTello(cmd_port=0, state_port=state_port, video_port=video_port).start()
            cleanup.callback(fake.close)
            fake.temp_override = 50
            link = TelloLink(host='127.0.0.1', cmd_port=fake._sock.getsockname()[1], local_cmd_port=0,
                state_port=state_port, bind_ip='127.0.0.1', allow_flight=True, log_root=str(root / 'gateway'))
            link.start()
            cleanup.callback(link.close)
            video = TelloVideo(port=video_port, bind_ip='127.0.0.1')
            cleanup.callback(video.stop)
            gateway = Gateway(link, source='fake', video=video)
            if omit_status_run_id:
                original = gateway.drone_state
                def old_status():
                    return {k: v for k, v in original().items() if k != 'run_id'}
                gateway.drone_state = old_status
            with patch.object(gateway_server, 'build_gateway', return_value=(gateway, fake)):
                app = gateway_server.build_app(SimpleNamespace())
            port = free_port()
            server = gateway_server.make_server(app, '127.0.0.1', port)
            thread = threading.Thread(target=server.run, daemon=True)
            recording = source = None
            thread.start()
            try:
                wait_for(lambda: server.started and gateway.arbiter.mode == 'grounded')
                self.assertTrue(gateway.set_video(True, 'synthetic_capture')[0])
                wait_for(lambda: video.jpeg_snapshot() is not None and link.state_count > 2)
                base = f'http://127.0.0.1:{port}'
                with build_opener(ProxyHandler({})).open(base + '/api/drone/state') as response:
                    status = json.load(response)
                if not omit_status_run_id:
                    self.assertEqual(status.get('run_id'), link.run_id)
                camera = json.loads(DEFAULT_CAMERA.read_text(encoding='utf-8'))
                camera.update(width=960, height=720, fps=24)
                path = root / 'capture'
                path.mkdir()
                recording = SessionRecording(path, camera, 'tello_camera')
                source = GatewayCamera(base)
                source.set_recording(recording)
                source.open(lambda: False)
                frames = source.frames(lambda: False)
                for _ in range(8):
                    jpeg, stamp = next(frames)
                    self.assertTrue(jpeg.startswith(b'\xff\xd8'))
                source.close()
                recording.finish_input()
                recording.finish_mapping({'status': 'complete'})
                self.assertTrue(recording.wait(10))
                sensors = [json.loads(row) for row in (path / 'sensor.jsonl').read_text().splitlines()]
                self.assertGreater(len(sensors), 2, 'actual gateway sensor packets must reach the saved session')
                for row in sensors:
                    self.assertEqual(row['source_run_id'], link.run_id)
                    self.assertIn('vgx', row['raw'])
                    self.assertIn('yaw', row['raw'])
                    self.assertIsInstance(row['received_mono'], (float, int))
                manifest = json.loads((path / 'session.json').read_text())
                self.assertEqual(manifest['recording']['status'], 'complete')
                self.assertEqual(manifest['recording']['log_records']['sensor'], len(sensors))
                self.assertGreater((path / 'recordings/part_0000.mjpeg').stat().st_size, 0)
                self.assertNotIn('takeoff', fake.received)
            finally:
                if source:
                    source.close()
                if recording:
                    recording.finish_input()
                    recording.finish_mapping({'status': 'failed'})
                    recording.wait(10)
                app.state.stopping.set()
                server.should_exit = True
                thread.join(10)
                self.assertFalse(thread.is_alive(), 'test gateway must shut down before temporary files are removed')

    def test_production_gateway_saves_real_sensor_packets_beside_video(self):
        self.record()

    def test_old_status_without_run_id_can_capture_from_sensor_endpoint(self):
        self.record(omit_status_run_id=True)


if __name__ == '__main__':
    unittest.main()
