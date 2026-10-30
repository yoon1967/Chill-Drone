"""Actual checkerboard detection/optimization and bounded local HTTP lifecycle."""
from functools import partial
from http.client import HTTPConnection
import json
import threading
import time
import unittest
from unittest.mock import Mock

try:
    import cv2
    import numpy as np
    import PIL
except ImportError as error:
    raise unittest.SkipTest('calibration tests need the mapping Python environment') from error

from apps.console.serve import ViewerHandler, ViewerServer
from modules.mapping.data.calibration_tutorial import (
    CalibrationSession, CalibrationTutorial, board_options, board_svg, calibrate_views, diversity)

BOARD = {'columns': 9, 'rows': 6, 'square_mm': 25}
SIZE = (960, 720)
MATRIX = np.array([[850., 0, 480], [0, 855., 360], [0, 0, 1]])


def views(count=25, noise=.08):
    obj = np.zeros((54, 3), np.float32)
    obj[:, :2] = np.mgrid[:9, :6].T.reshape(-1, 2) * 25
    rng = np.random.default_rng(350)
    result = []
    for i in range(count):
        rotation = np.array([(-.4, .25, .4, -.25, .1)[i % 5], (-.35, .3, .1)[i % 3], .04*(i % 4)])
        z = (500, 620, 780)[i % 3]
        center = ((.32, .5, .68)[i % 3], (.32, .5, .68)[(i//3) % 3])
        translation = np.array([(center[0]*960-480)/850*z-100, (center[1]*720-360)/855*z-62.5, z])
        projected, _ = cv2.projectPoints(obj, rotation, translation, MATRIX, np.array([-.05, .01, .001, -.001, 0.]))
        result.append((projected + rng.normal(0, noise, projected.shape)).astype(np.float32))
    return result


def checkerboard_jpeg(index=None):
    image = np.full((420, 600), 255, np.uint8)
    for y in range(7):
        for x in range(10):
            if (x+y) % 2 == 0:
                image[y*60:(y+1)*60, x*60:(x+1)*60] = 0
    target = np.float32([[200, 160], [740, 180], [720, 560], [220, 540]])
    if index is not None:
        i = index
        rotation = np.array([(-.4, .25, .4, -.25, .1)[i % 5], (-.35, .3, .1)[i % 3], .04*(i % 4)])
        z = (480, 580, 690)[i % 3]
        center = ((.32, .5, .68)[i % 3], (.32, .5, .68)[(i//3) % 3])
        translation = np.array([(center[0]*960-480)/850*z-100, (center[1]*720-360)/855*z-62.5, z])
        target, _ = cv2.projectPoints(np.float32([[-25, -25, 0], [225, -25, 0], [225, 150, 0], [-25, 150, 0]]),
            rotation, translation, MATRIX, np.zeros(5))
        target = target.reshape(4, 2)
    transform = cv2.getPerspectiveTransform(np.float32([[0, 0], [599, 0], [599, 419], [0, 419]]), target)
    frame = cv2.warpPerspective(image, transform, SIZE, borderValue=255)
    return cv2.imencode('.jpg', frame)[1].tobytes()


class CalibrationMathTests(unittest.TestCase):
    def test_rendered_jpegs_pass_detection_collection_and_real_calibration(self):
        session = CalibrationSession({**BOARD, 'source': 'camera', 'confirmed': True})
        for index in range(36):
            session.feed(checkerboard_jpeg(index))
        collected = session.snapshot()
        self.assertTrue(collected['coverage']['ready'], collected)
        session.solve(); session.worker.join(10)
        self.assertEqual(session.status, 'complete', session.error)
        self.assertAlmostEqual(session.result['fx'], 850, delta=20)
        self.assertAlmostEqual(session.result['fy'], 855, delta=20)
        self.assertLess(session.result['calibration_summary']['validation_rms_px'], 1.)
        session.stop()

    def test_actual_optimizer_recovers_known_camera_with_held_out_views(self):
        points = views()
        self.assertTrue(diversity(points, SIZE, BOARD)['ready'])
        camera = calibrate_views(points, SIZE, BOARD)
        self.assertAlmostEqual(camera['fx'], 850, delta=7)
        self.assertAlmostEqual(camera['fy'], 855, delta=7)
        self.assertAlmostEqual(camera['cx'], 480, delta=7)
        self.assertAlmostEqual(camera['cy'], 360, delta=7)
        self.assertEqual(len(camera['distortion']), 4)
        self.assertEqual(camera['calibration_summary']['validation_views'], 5)
        self.assertLess(camera['calibration_summary']['validation_rms_px'], .2)
        json.dumps(camera, allow_nan=False)

    def test_identical_front_facing_views_cannot_pass_by_count(self):
        repeated = [views(1)[0]] * 25
        with self.assertRaisesRegex(ValueError, 'diverse'):
            calibrate_views(repeated, SIZE, BOARD)

    def test_corrupted_held_out_views_fail_validation(self):
        points = views()
        points[4][::2, 0, 0] += 15
        with self.assertRaisesRegex(ValueError, 'reprojection'):
            calibrate_views(points, SIZE, BOARD)

    def test_actual_jpeg_detection_rejects_duplicate_and_changed_resolution(self):
        session = CalibrationSession({**BOARD, 'source': 'camera', 'confirmed': True})
        jpeg = checkerboard_jpeg()
        first = session.feed(jpeg)
        self.assertEqual(first['accepted'], 1)
        self.assertEqual(first['reason'], 'accepted')
        self.assertEqual(session.feed(jpeg)['reason'], 'checkerboard_duplicate_pose')
        self.assertEqual(len(session.points), 1)
        altered = cv2.imencode('.jpg', cv2.resize(cv2.imdecode(np.frombuffer(jpeg, np.uint8), 1), (640, 480)))[1].tobytes()
        with self.assertRaisesRegex(ValueError, 'resolution_changed'):
            session.feed(altered)
        session.stop()
        with self.assertRaisesRegex(ValueError, 'not_collecting'):
            session.feed(jpeg)

    def test_pattern_has_correct_physical_size_and_rejects_invalid_settings(self):
        svg = board_svg(BOARD).decode()
        self.assertIn('width="270mm"', svg)
        self.assertIn('height="205mm"', svg)
        self.assertEqual(svg.count('<rect'), 36)
        for body in ({'columns': True}, {'rows': 100}, {'square_mm': float('nan')}):
            with self.assertRaises(ValueError):
                board_options(body)


class CalibrationHTTPTests(unittest.TestCase):
    def setUp(self):
        self.tutorial = CalibrationTutorial()
        self.live = Mock(current=None)
        self.server = ViewerServer(('127.0.0.1', 0), partial(ViewerHandler, calibrations=self.tutorial, live=self.live))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.tutorial.shutdown()
        self.server.shutdown(); self.server.server_close(); self.thread.join(2)

    def request(self, path, body=None, headers=None):
        connection = HTTPConnection('127.0.0.1', self.server.server_port, timeout=10)
        data = body if isinstance(body, bytes) else json.dumps(body) if body is not None else None
        connection.request('POST' if body is not None else 'GET', '/api/calibration' + path, body=data,
            headers=headers if headers is not None else {'X-Mapping-Client': '1', 'Content-Type': 'application/json'})
        response = connection.getresponse()
        payload, kind = response.read(), response.getheader('Content-Type', '')
        value = json.loads(payload) if kind.startswith('application/json') else payload
        code = response.status; connection.close()
        return code, value

    def start(self):
        code, state = self.request('/start', {**BOARD, 'source': 'camera', 'confirmed': True})
        self.assertEqual(code, 201)
        return state['id']

    def test_collection_solve_download_stop_and_old_session_isolation(self):
        key = self.start()
        self.assertEqual(self.request('/frame?id=' + key, checkerboard_jpeg())[1]['accepted'], 1)
        self.assertEqual(self.request('/undo', {'id': key})[1]['accepted'], 0)
        session = self.tutorial.require(key)
        with session.lock:
            session.points, session.size = views(), SIZE
        self.assertEqual(self.request('/solve', {'id': key})[0], 202)
        session.worker.join(5)
        code, camera = self.request('/camera.json?id=' + key)
        self.assertEqual(code, 200)
        self.assertAlmostEqual(camera['fx'], 850, delta=7)
        self.assertEqual(self.request('/stop', {'id': key})[0], 200)
        newer = self.start()
        self.assertNotEqual(key, newer)
        self.assertEqual(self.request('/stop', {'id': key})[0], 400)
        self.assertEqual(self.tutorial.current.id, newer)

    def test_origin_confirmation_and_active_mapping_are_enforced(self):
        body = {**BOARD, 'source': 'camera', 'confirmed': True}
        self.assertEqual(self.request('/start', body, headers={})[0], 403)
        self.assertEqual(self.request('/start', {**body, 'confirmed': False})[0], 400)
        self.live.current = Mock(done=False)
        self.assertEqual(self.request('/start', body)[0], 409)
        self.assertIsNone(self.tutorial.current)

    def test_active_calibration_blocks_mapping_start_without_changing_map(self):
        self.start()
        connection = HTTPConnection('127.0.0.1', self.server.server_port)
        connection.request('POST', '/api/live/start', '{}', {'X-Mapping-Client': '1'})
        response = connection.getresponse(); response.read(); connection.close()
        self.assertEqual(response.status, 409)
        self.live.start.assert_not_called()

    def test_oversize_payload_and_missing_result_are_rejected(self):
        key = self.start()
        self.assertEqual(self.request('/camera.json?id=' + key)[0], 400)
        self.assertEqual(self.request('/solve', {'id': key})[0], 400)
        self.assertEqual(self.request('/frame?id=' + key, b'bad jpeg')[0], 400)
        self.assertEqual(self.request('/board.svg?columns=99')[0], 400)


class DroneCleanupTests(unittest.TestCase):
    def test_drone_receiver_closes_on_stop_and_never_creates_a_map(self):
        closed = threading.Event()
        class Source:
            def open(self, cancelled):
                pass
            def frames(self, cancelled):
                while not cancelled():
                    time.sleep(.01)
                    yield None, time.monotonic()
            def close(self):
                closed.set()
        tutorial = CalibrationTutorial(Source)
        key = tutorial.start({**BOARD, 'source': 'tello', 'confirmed': True})['id']
        self.assertTrue(tutorial.busy)
        tutorial.require(key).stop()
        self.assertTrue(closed.wait(2))
        tutorial.shutdown()
        self.assertFalse(tutorial.busy)
