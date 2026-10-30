"""Bounded, local checkerboard collection and four-coefficient pinhole calibration."""
from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import threading
import time
import uuid


MIN_VIEWS, MAX_VIEWS = 18, 40


def board_options(body):
    if not isinstance(body, dict):
        raise ValueError('invalid_calibration_options')
    columns, rows, square = body.get('columns', 9), body.get('rows', 6), body.get('square_mm', 25)
    if (type(columns) is not int or type(rows) is not int or not 4 <= columns <= 12 or not 4 <= rows <= 10
            or type(square) not in (int, float) or not math.isfinite(square) or not 5 <= square <= 60):
        raise ValueError('invalid_checkerboard_dimensions')
    return {'columns': columns, 'rows': rows, 'square_mm': square}


def board_svg(options):
    b = board_options(options)
    c, r, size = b['columns'] + 1, b['rows'] + 1, b['square_mm']
    width, height = c * size + 20, r * size + 30
    squares = ''.join(f'<rect x="{10+x*size}" y="{10+y*size}" width="{size}" height="{size}"/>'
                      for y in range(r) for x in range(c) if (x + y) % 2 == 0)
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}mm" height="{height}mm" '
            f'viewBox="0 0 {width} {height}"><rect width="100%" height="100%" fill="white"/>'
            f'<g fill="black">{squares}</g><text x="10" y="{height-7}" font-size="3.5">'
            f'{b["columns"]} x {b["rows"]} inner corners; square {size} mm; print 100%</text></svg>').encode()


def diversity(points, size, board):
    import numpy as np
    width, height = size
    descriptors, cells, areas, tilts = [], set(), [], []
    for value in points:
        p = np.asarray(value).reshape(board['rows'], board['columns'], 2)
        flat = p.reshape(-1, 2) / [width, height]
        center = flat.mean(axis=0)
        cells.add((min(2, int(center[0]*3)), min(2, int(center[1]*3))))
        areas.append(float(np.prod(np.ptp(flat, axis=0))))
        u = (p[0, -1] - p[0, 0]) / (board['columns'] - 1)
        v = (p[-1, 0] - p[0, 0]) / (board['rows'] - 1)
        lu, lv = np.linalg.norm(u), np.linalg.norm(v)
        tilts.append(float(max(abs(np.log(max(lu, 1e-6)/max(lv, 1e-6))),
                               abs(np.dot(u, v)/max(lu*lv, 1e-6)))))
        descriptors.append(center)
    spread = np.ptp(descriptors, axis=0) if descriptors else [0., 0.]
    scale_ratio = max(areas)/max(min(areas), 1e-6) if areas else 1.
    tilted = sum(t > .10 for t in tilts)
    checks = {'views': len(points) >= MIN_VIEWS, 'positions': bool(len(cells) >= 4 and min(spread) >= .18),
              'distances': scale_ratio >= 1.5, 'tilts': tilted >= 3}
    return {'checks': checks, 'ready': all(checks.values()), 'position_cells': len(cells),
            'center_spread': [float(v) for v in spread], 'size_ratio': float(scale_ratio), 'tilted_views': tilted}


def calibrate_views(points, size, board):
    """Keep every fifth view out of intrinsics fitting; solve only its pose for validation."""
    import cv2
    import numpy as np
    from .camera_profiles import validate_intrinsics
    coverage = diversity(points, size, board)
    if not coverage['ready']:
        raise ValueError('calibration_views_not_diverse_enough')
    obj = np.zeros((board['columns'] * board['rows'], 3), np.float32)
    obj[:, :2] = np.mgrid[:board['columns'], :board['rows']].T.reshape(-1, 2) * board['square_mm']
    training = [np.asarray(p, np.float32).reshape(-1, 1, 2) for i, p in enumerate(points) if i % 5 != 4]
    validation = [np.asarray(p, np.float32).reshape(-1, 1, 2) for i, p in enumerate(points) if i % 5 == 4]
    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
        [obj] * len(training), training, tuple(size), None, None, flags=cv2.CALIB_FIX_K3)
    errors = []
    for observed in validation:
        found, rotation, translation = cv2.solvePnP(obj, observed, matrix, distortion)
        if not found:
            raise ValueError('calibration_validation_failed')
        projected, _ = cv2.projectPoints(obj, rotation, translation, matrix, distortion)
        errors.append(float(np.sqrt(np.mean(np.sum((projected-observed)**2, axis=2)))))
    validation_rms = math.sqrt(sum(v*v for v in errors) / len(errors))
    if not math.isfinite(rms) or not math.isfinite(validation_rms) or rms > 1.0 or max(errors) > 1.5 or validation_rms > 1.0:
        raise ValueError('calibration_reprojection_error_too_high')
    width, height = size
    if not (.2*width < matrix[0, 0] < 5*width and .2*height < matrix[1, 1] < 8*height):
        raise ValueError('calibration_unstable_focal_length')
    camera = validate_intrinsics({'width': width, 'height': height, 'fx': float(matrix[0, 0]),
        'fy': float(matrix[1, 1]), 'cx': float(matrix[0, 2]), 'cy': float(matrix[1, 2]),
        'distortion': distortion.ravel()[:4].tolist(), 'square_mm': board['square_mm'],
        'calibration_status': 'checkerboard_calibrated; held_out_reprojection_checked',
        'calibration_summary': {'views': len(points), 'training_views': len(training), 'validation_views': len(validation),
            'training_rms_px': float(rms), 'validation_rms_px': validation_rms,
            'validation_max_view_rms_px': max(errors), 'model': 'OPENCV k1 k2 p1 p2; k3 fixed zero',
            'board': board, 'coverage': coverage, 'created_at': datetime.now(timezone.utc).isoformat(),
            'validation_note': 'Held-out views, poses estimated per view; not independent physical accuracy or metric map scale.'}})
    return camera


class CalibrationSession:
    def __init__(self, options):
        self.id = 'calibration_' + uuid.uuid4().hex[:16]
        self.board = board_options(options)
        self.source = options.get('source')
        if self.source not in {'camera', 'tello'} or options.get('confirmed') is not True:
            raise ValueError('confirm_calibration_camera_and_board')
        from .camera_profiles import text
        self.identity = text(options.get('camera_identity', 'Tello 촬영 기체' if self.source == 'tello' else '연결된 카메라'), 'camera_identity')
        self.lock, self.frame_lock = threading.RLock(), threading.Lock()
        self.cancel = threading.Event()
        self.status, self.error, self.reason = 'collecting', None, 'show_checkerboard'
        self.points, self.size, self.result, self.preview = [], None, None, None
        self.last_seen, self.created = time.monotonic(), time.monotonic()
        self.worker = self.input_worker = None

    @property
    def busy(self):
        return (not self.cancel.is_set() and self.status in {'collecting', 'calculating'}
                or bool(self.worker and self.worker.is_alive()) or bool(self.input_worker and self.input_worker.is_alive()))

    def snapshot(self):
        with self.lock:
            return {'id': self.id, 'status': self.status, 'source': self.source, 'board': self.board,
                'size': self.size, 'accepted': len(self.points), 'minimum': MIN_VIEWS, 'maximum': MAX_VIEWS,
                'coverage': diversity(self.points, self.size, self.board) if self.size else
                    {'ready': False, 'checks': dict.fromkeys(['views', 'positions', 'distances', 'tilts'], False)},
                'reason': self.reason, 'error': self.error, 'camera': self.result,
                'closing': self.cancel.is_set() and self.busy}

    def feed(self, jpeg):
        import cv2
        import numpy as np
        from PIL import Image
        from io import BytesIO
        if not self.frame_lock.acquire(blocking=False):
            raise RuntimeError('calibration_frame_busy')
        try:
            with self.lock:
                if self.cancel.is_set() or self.status != 'collecting':
                    raise ValueError('calibration_not_collecting')
            if not jpeg or len(jpeg) > 2097152:
                raise ValueError('calibration_frame_too_large')
            with Image.open(BytesIO(jpeg)) as image:
                if image.format != 'JPEG' or not 320 <= image.width <= 2560 or not 180 <= image.height <= 1920:
                    raise ValueError('unsupported_calibration_frame_dimensions')
                size = image.size
            frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if frame is None or tuple(frame.shape[1::-1]) != size:
                raise ValueError('invalid_calibration_jpeg')
            with self.lock:
                if self.size and tuple(self.size) != size:
                    raise ValueError('calibration_resolution_changed_restart')
                self.size = size
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # A bounded working image for detection, then refine in native pixels.
            scale = min(1., 960 / max(size))
            small = cv2.resize(gray, None, fx=scale, fy=scale) if scale < 1 else gray
            found, corners = cv2.findChessboardCorners(small, (self.board['columns'], self.board['rows']),
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
            reason = 'checkerboard_not_found'
            if found:
                corners = cv2.cornerSubPix(gray, corners / scale, (5, 5), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .01))
                p = corners.reshape(-1, 2)
                normalized = p / size
                span = np.ptp(normalized, axis=0)
                left, top = np.maximum(p.min(axis=0).astype(int), 0)
                right, bottom = np.minimum(p.max(axis=0).astype(int)+1, size)
                sharpness = cv2.Laplacian(gray[top:bottom, left:right], cv2.CV_64F).var()
                if np.min(normalized) < .02 or np.max(normalized) > .98:
                    reason = 'checkerboard_too_close_to_edge'
                elif float(np.prod(span)) < .04:
                    reason = 'checkerboard_too_small'
                elif float(sharpness) < 40:
                    reason = 'checkerboard_blurry'
                else:
                    reason = 'accepted'
                    with self.lock:
                        if any(min(float(np.sqrt(np.mean(((p-q.reshape(-1, 2))/size)**2))),
                                   float(np.sqrt(np.mean(((p-q.reshape(-1, 2)[::-1])/size)**2)))) < .015 for q in self.points):
                            reason = 'checkerboard_duplicate_pose'
                        elif len(self.points) >= MAX_VIEWS:
                            reason = 'calibration_collection_full'
                        elif not self.cancel.is_set() and self.status == 'collecting':
                            self.points.append(corners.copy())
                cv2.drawChessboardCorners(frame, (self.board['columns'], self.board['rows']), corners, found)
            _, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with self.lock:
                if not self.cancel.is_set():
                    self.preview, self.reason = encoded.tobytes(), reason
            return self.snapshot()
        finally:
            self.frame_lock.release()

    def solve(self):
        with self.lock:
            if self.cancel.is_set() or self.status != 'collecting' or not self.snapshot()['coverage']['ready']:
                raise ValueError('calibration_views_not_diverse_enough')
            self.status, self.error = 'calculating', None
            points, size = list(self.points), self.size
            def run():
                try:
                    camera = calibrate_views(points, size, self.board)
                    camera['camera_identity'] = self.identity
                    with self.lock:
                        if not self.cancel.is_set():
                            self.result, self.status = camera, 'complete'
                except Exception as error:
                    with self.lock:
                        if not self.cancel.is_set():
                            self.error, self.status = str(error), 'collecting'
            self.worker = threading.Thread(target=run, daemon=True, name='checkerboard-solve')
            self.worker.start()
        return self.snapshot()

    def stop(self):
        self.cancel.set()
        with self.lock:
            if self.status not in {'complete', 'failed'}:
                self.status = 'stopped'


class CalibrationTutorial:
    def __init__(self, source_factory=None):
        self.lock = threading.RLock()
        self.current = None
        self.source_factory = source_factory

    @property
    def busy(self):
        return bool(self.current and self.current.busy)

    def require(self, key, touch=False):
        with self.lock:
            if not self.current or self.current.id != key:
                raise ValueError('calibration_session_mismatch')
            if touch:
                self.current.last_seen = time.monotonic()
            return self.current

    def start(self, options):
        with self.lock:
            if self.busy:
                raise RuntimeError('calibration_input_is_active')
            session = CalibrationSession(options)
            if session.source == 'tello' and self.source_factory is None:
                raise ValueError('calibration_tello_not_enabled')
            self.current = session
            def lease():
                while not session.cancel.wait(1):
                    if time.monotonic() - session.last_seen > 45 or time.monotonic() - session.created > 900:
                        session.stop()
                        break
            threading.Thread(target=lease, daemon=True, name='checkerboard-lease').start()
            if session.source == 'tello':
                def receive():
                    source = None
                    try:
                        source = self.source_factory()
                        source.open(session.cancel.is_set)
                        sampled = -math.inf
                        for jpeg, stamp in source.frames(session.cancel.is_set):
                            if session.cancel.is_set() or session.status == 'complete':
                                break
                            if jpeg and session.status == 'collecting' and stamp - sampled >= .75:
                                sampled = stamp
                                session.feed(jpeg)
                    except Exception as error:
                        with session.lock:
                            if not session.cancel.is_set():
                                session.error, session.status = str(error), 'failed'
                    finally:
                        if source:
                            source.close()
                session.input_worker = threading.Thread(target=receive, daemon=True, name='checkerboard-tello')
                session.input_worker.start()
            return session.snapshot()

    def shutdown(self):
        if self.current:
            self.current.stop()
            for worker in (self.current.input_worker, self.current.worker):
                if worker:
                    worker.join(timeout=5)
