"""Sensor cursor and mapping-frame transport contracts, without contacting a drone."""
import asyncio
import json
import os
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tello_link import TelloLink
from sensor_pose import SensorPose
try:
    from fastapi import HTTPException
    from starlette.requests import Request
    import gateway_server
except ImportError:
    gateway_server = None


class SensorCaptureTests(unittest.TestCase):
    def test_sensor_cursor_exposes_dropped_packets_and_retains_receive_clock(self):
        link = TelloLink(run_id='synthetic')
        link.state_count = 600
        link._sensor_history.extend({'sequence': i, 'received_mono': i * .1, 'raw': {'yaw': i % 360}}
                                    for i in range(1, 601))
        batch = link.sensor_samples(10)
        self.assertEqual(batch['lost'], 78)
        self.assertEqual(len(batch['samples']), 512)
        self.assertEqual(batch['next_sequence'], 600)
        self.assertEqual(batch['samples'][0]['sequence'], 89)
        self.assertAlmostEqual(batch['samples'][0]['received_mono'], 8.9)
        self.assertEqual(link.sensor_samples(600)['samples'], [])

    @unittest.skipIf(gateway_server is None, 'FastAPI is not installed')
    def test_sensor_endpoint_is_local_read_only_and_handles_old_replay(self):
        gw = SimpleNamespace(link=TelloLink(run_id='synthetic'), listeners=[])
        with patch.object(gateway_server, 'build_gateway', return_value=(gw, None)):
            app = gateway_server.build_app(SimpleNamespace())
        route = next(route for route in app.routes if route.path == '/api/drone/sensors')
        request = Request({'type': 'http', 'client': ('127.0.0.1', 1000)})
        self.assertEqual(route.endpoint(request, 0)['run_id'], 'synthetic')
        for client, after, status in [('192.168.1.5', 0, 403), ('127.0.0.1', -1, 400)]:
            with self.assertRaises(HTTPException) as error:
                route.endpoint(Request({'type': 'http', 'client': (client, 1000)}), after)
            self.assertEqual(error.exception.status_code, status)
        gw.link = SimpleNamespace()
        with self.assertRaises(HTTPException) as error:
            route.endpoint(request, 0)
        self.assertEqual(error.exception.status_code, 404)

    @unittest.skipIf(gateway_server is None, 'FastAPI is not installed')
    def test_gateway_mjpeg_carries_original_frame_clock(self):
        jpeg = b'\xff\xd8synthetic\xff\xd9'
        pose = SensorPose(1, 0., 0., .8, 0., 0., 0.)
        stopping = threading.Event()
        class Client:
            async def is_disconnected(self):
                return False
        async def read():
            frames = gateway_server.mjpeg_frames(Client(), lambda: (jpeg, 35.2, 1800000000.),
                lambda: True, stopping, lambda stamp: pose, 'synthetic')
            part = await anext(frames)
            await frames.aclose()
            return part
        part = asyncio.run(read())
        headers, image = part.split(b'\r\n\r\n', 1)
        values = dict(line.split(b': ', 1) for line in headers.split(b'\r\n')[1:])
        meta = json.loads(values[b'X-Tello-Frame'])
        self.assertEqual(image, jpeg + b'\r\n')
        self.assertEqual(json.loads(values[b'X-Tello-Pose']), list(pose))
        self.assertEqual(meta, {'run_id': 'synthetic', 'timestamp_s': 35.2, 'wall': 1800000000.})
        self.assertNotIn(b'X-Tello-Frame', gateway_server.mjpeg_part(jpeg))


if __name__ == '__main__':
    unittest.main()
