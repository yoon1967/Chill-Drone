"""Immutable local camera profiles shared by live capture and file reconstruction."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
import threading
import uuid
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[3]
BUILTIN_CAMERA = ROOT / 'data/mapping/tello/20261028/v1/camera.json'
BUILTIN_ID = 'tello_measured_v1'
SCHEMA = 'mapping-camera-profile/1.0'
MODES = {'tello', 'camera', 'file'}


def text(value, field, maximum=160):
    if not isinstance(value, str) or not value.strip() or len(value) > maximum or any(ord(c) < 32 for c in value):
        raise ValueError('invalid_profile_' + field)
    return value.strip()


def validate_intrinsics(value):
    if not isinstance(value, dict):
        raise ValueError('invalid_camera_intrinsics')
    camera = {key: value.get(key) for key in ('width', 'height', 'fx', 'fy', 'cx', 'cy')}
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in camera.values()):
        raise ValueError('camera_values_must_be_finite_numbers')
    if any(not 1 <= camera[k] <= 16384 or int(camera[k]) != camera[k] for k in ('width', 'height')):
        raise ValueError('camera_dimensions_must_be_integers')
    if not (camera['fx'] > 0 and camera['fy'] > 0 and 0 <= camera['cx'] <= camera['width'] and 0 <= camera['cy'] <= camera['height']):
        raise ValueError('invalid_camera_intrinsics')
    distortion = value.get('distortion', [0., 0., 0., 0.])
    if not isinstance(distortion, list) or len(distortion) != 4 or any(type(v) not in (int, float) or not math.isfinite(v) for v in distortion):
        raise ValueError('invalid_camera_distortion')
    camera.update(width=int(camera['width']), height=int(camera['height']), distortion=distortion)
    if 'profile' in value and not isinstance(value['profile'], dict):
        raise ValueError('invalid_camera_profile_metadata')
    for key in ('calibration_status', 'camera_identity', 'square_mm', 'calibration_summary', 'profile'):
        if key in value:
            camera[key] = json.loads(json.dumps(value[key], allow_nan=False))
    camera.setdefault('calibration_status', 'user_supplied; accuracy_not_independently_verified')
    if not isinstance(camera['calibration_status'], str):
        raise ValueError('invalid_calibration_status')
    return camera


def fingerprint(camera):
    fields = {k: camera[k] for k in ('width', 'height', 'fx', 'fy', 'cx', 'cy', 'distortion')}
    return hashlib.sha256(json.dumps(fields, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


class CameraProfiles:
    def __init__(self, root, builtin_camera=BUILTIN_CAMERA):
        self.root = Path(root).resolve()
        self.builtin_camera = Path(builtin_camera)
        self.lock = threading.RLock()

    def _file(self, key):
        if not re.fullmatch(r'profile_[a-f0-9]{16}', str(key)):
            raise ValueError('unknown_camera_profile')
        path = self.root / key / 'camera.json'
        if path.is_symlink() or path.parent.is_symlink() or (hasattr(path.parent, 'is_junction') and path.parent.is_junction()):
            raise ValueError('linked_camera_profile')
        return path

    def get(self, key):
        with self.lock:
            if key == BUILTIN_ID:
                camera = validate_intrinsics(json.loads(self.builtin_camera.read_text(encoding='utf-8')))
                camera['profile'] = {'schema_version': SCHEMA, 'id': BUILTIN_ID, 'version': 1,
                    'source_width': camera['width'], 'source_height': camera['height'],
                    'name': 'Tello TELLO-59FA6D · 기존 실측', 'camera_identity': camera.get('camera_identity', 'TELLO-59FA6D'),
                    'input_modes': ['tello', 'file'], 'image_geometry': '960×720 원본 · 가로 방향 · 크롭 없음',
                    'source': 'bundled_measured', 'readonly': True, 'camera_sha256': fingerprint(camera)}
                return camera
            camera = validate_intrinsics(json.loads(self._file(key).read_text(encoding='utf-8')))
            profile = camera.get('profile', {})
            if (profile.get('schema_version') != SCHEMA or profile.get('id') != key or profile.get('version') != 1
                    or profile.get('camera_sha256') != fingerprint(camera)):
                raise ValueError('camera_profile_integrity_failed')
            modes = profile.get('input_modes')
            if not isinstance(modes, list) or not modes or any(not isinstance(m, str) or m not in MODES for m in modes):
                raise ValueError('invalid_profile_input_modes')
            for field in ('name', 'camera_identity', 'image_geometry'):
                text(profile.get(field), field, 80 if field == 'name' else 160)
            return camera

    def _write(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
        try:
            temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)

    def register(self, body):
        if not isinstance(body, dict):
            raise ValueError('invalid_profile_request')
        camera = validate_intrinsics(body.get('camera'))
        old_profile = camera.pop('profile', None)
        if old_profile and old_profile.get('schema_version') != SCHEMA:
            raise ValueError('unsupported_camera_profile_schema')
        name = text(body.get('name'), 'name', 80)
        identity = text(body.get('camera_identity'), 'camera_identity')
        geometry = text(body.get('image_geometry'), 'image_geometry')
        camera['camera_identity'] = identity
        modes = body.get('input_modes')
        if not isinstance(modes, list) or not modes or any(m not in MODES for m in modes):
            raise ValueError('invalid_profile_input_modes')
        if body.get('confirmed') is not True:
            raise ValueError('confirm_camera_profile_binding')
        modes = sorted(set(modes))
        signature = hashlib.sha256(json.dumps([name, identity, geometry, modes, camera], sort_keys=True).encode()).hexdigest()
        with self.lock:
            # A retried registration is idempotent; existing profiles are never overwritten.
            for row in self.list()['profiles']:
                if row.get('registration_signature') == signature:
                    return self.get(row['id'])
            key = 'profile_' + uuid.uuid4().hex[:16]
            profile = {'schema_version': SCHEMA, 'id': key, 'version': 1, 'name': name,
                'source_width': camera['width'], 'source_height': camera['height'],
                'camera_identity': identity, 'image_geometry': geometry, 'input_modes': modes,
                'created_at': datetime.now(timezone.utc).isoformat(), 'source': 'imported_profile_json' if old_profile else 'legacy_camera_json',
                'readonly': False, 'camera_sha256': fingerprint(camera), 'registration_signature': signature,
                'verification': 'file_values_checked; measurement_accuracy_not_independently_verified'}
            if old_profile:
                profile['imported_from'] = {k: old_profile.get(k) for k in ('id', 'version', 'name')}
            camera['profile'] = profile
            if len(json.dumps(camera, ensure_ascii=False, indent=2).encode('utf-8')) > 65536:
                raise ValueError('camera_profile_json_too_large')
            self._write(self._file(key), camera)
            return camera

    def list(self):
        rows, warnings = [], []
        keys = [BUILTIN_ID] + sorted(p.name for p in self.root.glob('profile_*') if p.is_dir())
        for key in keys:
            try:
                camera = self.get(key)
                rows.append({**camera['profile'], 'width': camera['width'], 'height': camera['height'],
                    'camera': camera, 'calibration_status': camera['calibration_status']})
            except (OSError, ValueError, KeyError, TypeError):
                warnings.append('보정 프로필을 읽지 못했습니다: ' + key)
        defaults = {'tello': BUILTIN_ID, 'camera': None, 'file': None}
        path = self.root / 'defaults.json'
        if path.is_file():
            try:
                saved = json.loads(path.read_text(encoding='utf-8'))
                for mode in MODES:
                    key = saved.get(mode)
                    if key is None or any(p['id'] == key and mode in p['input_modes'] for p in rows):
                        defaults[mode] = key
            except (OSError, ValueError, TypeError, AttributeError):
                warnings.append('기본 프로필 설정을 읽지 못해 기존 기본값을 사용합니다.')
        return {'profiles': rows, 'defaults': defaults, 'warnings': warnings}

    def set_default(self, mode, key):
        if mode not in MODES:
            raise ValueError('invalid_profile_input_modes')
        with self.lock:
            if key is not None and mode not in self.get(key)['profile']['input_modes']:
                raise ValueError('camera_profile_input_mode_mismatch')
            defaults = self.list()['defaults']
            defaults[mode] = key
            self._write(self.root / 'defaults.json', defaults)
            return self.list()

    def resolve(self, selection, mode, width, height):
        if not isinstance(selection, dict) or selection.get('confirmed') is not True:
            raise ValueError('confirm_camera_profile_binding')
        camera = self.get(selection.get('id'))
        profile = camera['profile']
        if mode not in profile['input_modes']:
            raise ValueError('camera_profile_input_mode_mismatch')
        if selection.get('version') != profile['version'] or selection.get('camera_sha256') != profile['camera_sha256']:
            raise ValueError('camera_profile_changed_refresh_selection')
        if type(width) not in (int, float) or type(height) not in (int, float) or (width, height) != (camera['width'], camera['height']):
            raise ValueError('calibration_dimensions_must_match_input')
        return camera
