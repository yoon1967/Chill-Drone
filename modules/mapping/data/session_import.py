"""Local import jobs and portable bindings for the existing console viewer.

Input files are never executed, rewritten, or used as absolute asset paths.
"""
from __future__ import annotations

from contextlib import closing
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import signal
import sqlite3
import subprocess
import sys
import threading
import time
import uuid
import zipfile

ROOT = Path(__file__).resolve().parents[3]
VIDEO = {'.mp4', '.mov', '.avi', '.mkv'}
IMAGES = {'.jpg', '.jpeg', '.png'}
MAX_BYTES = 64 * 1024**3
MAX_FILES = 50000


def read_json(path):
    def invalid(value):
        raise ValueError('non_finite_json')
    def finite_float(value):
        result = float(value)
        if not math.isfinite(result):
            raise ValueError('non_finite_json')
        return result
    value = json.loads(Path(path).read_text(encoding='utf-8'), parse_constant=invalid, parse_float=finite_float)
    if not isinstance(value, dict):
        raise ValueError('json_object_required')
    return value


def feature_progress(output):
    """Read committed extraction results without changing the native worker."""
    output = Path(output)
    try:
        total = read_json(output / 'input/manifest.json')['selected_frames']
        if type(total) is not int or total <= 0:
            return None
    except (OSError, ValueError, KeyError):
        return None
    completed = 0
    database = output / 'features/database.db'
    try:
        with closing(sqlite3.connect(database.resolve().as_uri() + '?mode=ro', uri=True, timeout=0.05)) as connection:
            completed = connection.execute(
                'SELECT COUNT(*) FROM keypoints JOIN descriptors USING (image_id)').fetchone()[0]
            pairs_completed = connection.execute('SELECT COUNT(*) FROM two_view_geometries').fetchone()[0] if completed >= total else 0
    except sqlite3.Error:
        # The worker may not have created its schema yet or may be committing.
        return {'phase': 'extraction', 'completed': None, 'total': total}
    result = {'phase': 'matching' if completed >= total else 'extraction',
              'completed': min(completed, total), 'total': total}
    if completed >= total:
        try:
            result.update(pairs_total=len((output / 'features/pairs.txt').read_text(encoding='utf-8').splitlines()),
                          pairs_completed=pairs_completed)
        except OSError:
            pass
    try:
        result['phase'] = read_json(output / 'features/status.json').get('phase', result['phase'])
    except (OSError, ValueError):
        pass
    return result


def stage_progress(output, stage):
    try:
        status = read_json(Path(output) / stage / 'status.json')
        return {key: status[key] for key in ('phase', 'stage', 'completed', 'total', 'selected') if key in status}
    except (OSError, ValueError):
        return None


def relative_path(name):
    """Reject Windows aliases too, even when tests run on Linux."""
    if not isinstance(name, str) or not name or '\\' in name or ':' in name or '\x00' in name:
        raise ValueError('invalid_relative_path')
    path = PurePosixPath(name)
    if path.is_absolute() or any(p in {'', '.', '..'} for p in name.split('/')):
        raise ValueError('invalid_relative_path')
    if any(p.endswith((' ', '.')) or re.match(r'^(CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])(?:\.|$)', p, re.I)
           for p in path.parts):
        raise ValueError('invalid_relative_path')
    return path


def inside(root, name):
    relative = relative_path(name)
    target = Path(root).joinpath(*relative.parts)
    if not target.resolve().is_relative_to(Path(root).resolve()):
        raise ValueError('asset_outside_bundle')
    for parent in [target, *target.parents]:
        if parent == Path(root).parent:
            break
        if parent.is_symlink() or (hasattr(parent, 'is_junction') and parent.is_junction()):
            raise ValueError('linked_asset')
    return target


def extract_zip(source, destination, cancelled=lambda: False):
    destination.mkdir()
    with zipfile.ZipFile(source) as archive:
        items = archive.infolist()
        if len(items) > MAX_FILES or sum(i.file_size for i in items) > MAX_BYTES:
            raise ValueError('archive_too_large')
        if sum(i.file_size for i in items) > shutil.disk_usage(destination).free:
            raise ValueError('insufficient_disk_space')
        names = set()
        for item in items:
            if cancelled():
                raise InterruptedError('cancelled')
            name = item.filename.rstrip('/')
            target = inside(destination, name)
            folded = name.casefold()
            if folded in names or (item.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError('duplicate_or_linked_zip_entry')
            names.add(folded)
            if item.is_dir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(item) as src, target.open('xb') as dst:
                while block := src.read(1024 * 1024):
                    if cancelled():
                        raise InterruptedError('cancelled')
                    dst.write(block)


def scene_at(path):
    if read_json(inside(path, 'status.json')).get('status') not in {'complete', 'partial'}:
        raise ValueError('incomplete_result')
    scene = read_json(inside(path, 'scene.json'))
    if scene.get('schema_version') not in {'phone-sfm-result/1.0', 'phone-sfm-result/1.1', 'phone-sfm-result/1.2'}:
        raise ValueError('unsupported_scene_schema')
    if not isinstance(scene.get('points'), list) or not isinstance(scene.get('cameras'), list):
        raise ValueError('invalid_scene')
    if scene.get('rescue') is not None:
        from modules.mapping.data.rescue_mapping import validate_payload
        validate_payload(scene['rescue'])
    if any(not isinstance(p, list) or len(p) != 9 or
           any(type(n) not in (float, int) or not math.isfinite(n) for n in p[1:4]) for p in scene['points']):
        raise ValueError('invalid_point_coordinates')
    for camera in scene['cameras']:
        if not isinstance(camera, dict):
            raise ValueError('invalid_camera')
        position = camera.get('position')
        if not isinstance(position, list) or len(position) != 3 or any(type(n) not in (float, int) or not math.isfinite(n) for n in position):
            raise ValueError('invalid_camera_position')
    return scene


class ImportedMap:
    def __init__(self, root, cancelled=lambda: False):
        self.root = Path(root).resolve()
        self.result_root = self.root / 'reconstruction' if (self.root / 'reconstruction/scene.json').is_file() else self.root
        self.warnings = []
        try:
            self.manifest = read_json(inside(self.root, 'session.json')) if (self.root / 'session.json').is_file() else None
        except (OSError, ValueError):
            self.manifest = None
            self.warnings.append('세션 정보 파일을 읽지 못했습니다. 지도 기록에서 복구합니다.')
        if self.manifest and self.manifest.get('schema_version') != 'mapping-session/1.0':
            raise ValueError('unsupported_session_schema')
        self.is_session = (self.root / 'revisions').is_dir()
        self.revisions = {}
        self.entries = []
        if self.is_session:
            try:
                state = read_json(inside(self.root, 'state.json')) if (self.root / 'state.json').is_file() else {}
            except (OSError, ValueError):
                state = {}
                self.warnings.append('마지막 상태 파일을 읽지 못했습니다. 남은 지도 버전에서 선택합니다.')
            inferred_id = self.manifest_id or state.get('session_id')
            if state.get('status') == 'failed' or (self.manifest or {}).get('mapping', {}).get('status') == 'failed':
                self.warnings.append('이전 실행이 오류로 종료됐습니다. 마지막 유효 지도와 남아 있는 기록을 표시합니다.')
            for folder in sorted(inside(self.root, 'revisions').iterdir()):
                if cancelled():
                    raise InterruptedError('cancelled')
                if not re.fullmatch(r'\d{6}', folder.name):
                    continue
                try:
                    folder = inside(self.root, 'revisions/' + folder.name)
                    scene = scene_at(folder)
                    if inferred_id is None:
                        inferred_id = scene.get('input_run_id')
                    if not isinstance(inferred_id, str) or scene.get('input_run_id') != inferred_id:
                        raise ValueError('revision_session_mismatch')
                    if scene.get('revision') != int(folder.name):
                        raise ValueError('revision_number_mismatch')
                    self.revisions[folder.name] = folder
                    slam = scene.get('slam', {})
                    stamp = slam.get('timestamp_s', max((c.get('timestamp_s', 0) for c in scene['cameras']), default=0))
                    if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp < 0:
                        raise ValueError('invalid_revision_time')
                    self.entries.append({'revision': int(folder.name), 'map_id': scene.get('segment_id', 0),
                        'timestamp_s': stamp, 'timestamp_basis': slam.get('timestamp_basis', 'latest_surviving_keyframe_estimate'),
                        'points3D': len(scene['points']), 'keyframes': len(scene['cameras']), 'final': slam.get('final', False)})
                except (OSError, ValueError, KeyError, TypeError):
                    self.revisions.pop(folder.name, None)
                    self.warnings.append('지도 버전 ' + folder.name + '을 읽지 못해 제외했습니다.')
            preferred = str(state.get('revision_path', '')).removeprefix('revisions/')
            self.latest = preferred if preferred in self.revisions else next(reversed(self.revisions), None)
            identity = inferred_id or self.root.name
            self.state = {**state, 'session_id': identity, 'status': 'complete', 'session_status': 'complete',
                'recorded': True, 'revision': int(self.latest or 0), 'map_available': bool(self.latest),
                'revision_path': 'revisions/' + self.latest if self.latest else '', 'backend': 'orb_slam3_monocular'}
            if self.manifest and not self.manifest.get('recording', {}).get('finalized'):
                self.warnings.append('저장이 완료되기 전에 중단된 세션입니다. 남아 있는 지도만 엽니다.')
            if not self.latest:
                self.warnings.append('생성된 유효 지도가 없습니다. 녹화 영상과 진단은 확인할 수 있습니다.')
        else:
            scene = scene_at(self.result_root)
            self.latest = None
            self.state = {'status': 'offline', 'recorded': True, 'map_available': True}
        self.media = []
        def verify(name, expected=None):
            path = inside(self.root, name)
            if not path.is_file():
                self.warnings.append('파일 누락: ' + name)
                return False
            if expected:
                digest = hashlib.sha256()
                with path.open('rb') as stream:
                    while block := stream.read(1024 * 1024):
                        if cancelled():
                            raise InterruptedError('cancelled')
                        digest.update(block)
                if digest.hexdigest() != expected:
                    self.warnings.append('저장 무결성 확인 실패: ' + name)
                    return False
            return True
        if self.manifest:
            for part in self.manifest.get('recordings', []):
                if part.get('raw_path'):
                    verify(part['raw_path'], part.get('sha256'))
                playback = part.get('playback', {})
                name = playback.get('path')
                if name:
                    target = inside(self.root, name)
                    if target.suffix.lower() in VIDEO and verify(name, playback.get('sha256')):
                        self.media.append({'index': len(self.media), 'path': name, 'label': f"녹화 구간 {part['id'] + 1}",
                            'browser_playable': target.suffix.lower() == '.mp4'})
            for name, expected in self.manifest.get('asset_sha256', {}).items():
                verify(name, expected)
        self._check_images()
        self.history_frames = 0
        history = self.result_root / 'history'
        if (history / 'manifest.json').exists() or (history / 'index.jsonl').exists():
            from modules.mapping.data.mapping_history import HistoryReader
            for name in ('history/manifest.json', 'history/index.jsonl'):
                if not inside(self.result_root, name).is_file():
                    raise ValueError('reconstruction_history_incomplete')
            entries = HistoryReader(self.result_root).refresh()
            for entry in entries:
                name = entry.get('frame_id', '')
                if not re.fullmatch(r'frame_\d{7,}\.jpg', name):
                    raise ValueError('invalid_history_frame_name')
                for asset in (f"history/frames/{entry['revision']:08d}.npz", 'frames/' + name):
                    if not inside(self.result_root, asset).is_file():
                        raise ValueError('reconstruction_history_incomplete')
            self.history_frames = len(entries)

    @property
    def manifest_id(self):
        return self.manifest.get('session_id') if self.manifest else None

    def result(self, revision=None):
        return self.revisions.get(revision or self.latest) if self.is_session else self.result_root

    def image_root(self, result):
        candidates = [self.root / 'frames', self.root / 'input/frames', self.root / 'images', result / 'images',
                      result / 'frames', self.root.parent / 'input/frames']
        for folder in candidates:
            if folder.is_relative_to(self.root) and folder.is_dir():
                return inside(self.root, folder.relative_to(self.root).as_posix())
        return None

    def _check_images(self):
        result = self.result()
        if result:
            scene = scene_at(result)
            self.points, self.cameras = len(scene['points']), len(scene['cameras'])
            if not self.is_session and read_json(result / 'status.json').get('status') == 'partial':
                self.warnings.append('미정렬 지도 구간 또는 부족한 관측이 있습니다. 구간별 전체 관측은 보존되지만 하나의 좌표계로 연결되지 않았을 수 있습니다.' if scene.get('depth_mapping', {}).get('integration_scope') == 'all_recorded_observations' else '부분 복원 결과입니다. 연결되지 않은 프레임과 다른 모델의 좌표는 합치지 않았습니다.')
            images = self.image_root(result)
            missing = sum(not images or not inside(images, c['frame_id']).is_file() for c in scene['cameras'])
            if missing:
                self.warnings.append(f'키프레임 이미지 {missing}개가 없습니다. 지도와 경로는 표시합니다.')
        else:
            self.points = self.cameras = 0

    def summary(self):
        return {'kind': 'session' if self.is_session else 'result', 'name': self.root.name,
            'session_id': self.state.get('session_id'), 'points': self.points, 'keyframes': self.cameras,
            'revisions': len(self.entries), 'history_frames': self.history_frames, 'warnings': self.warnings,
            'media': [{k: v for k, v in m.items() if k != 'path'} for m in self.media],
            'recording': self.manifest.get('recording') if self.manifest else None}


def detect_root(root):
    """A ZIP can have a single wrapper. Never guess between multiple sessions."""
    for _ in range(5):
        if any((root / name).is_file() for name in ('session.json', 'scene.json')) or (root / 'revisions').is_dir():
            return root
        if (root / 'reconstruction/scene.json').is_file():
            return root
        children = [p for p in root.iterdir() if p.name not in {'__MACOSX', '.DS_Store'}]
        if len(children) == 1 and children[0].is_dir():
            root = inside(root, children[0].name)
        else:
            return root
    return root


class ImportManager:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.jobs = {}
        self.lock = threading.RLock()
        self.history_readers = {}
        self.preview_changed = threading.Condition()
        from modules.mapping.data.depth_worker import DepthWorker
        self.depth_worker = DepthWorker()
        self.thumbnail_cache = {}

    def create(self, name='불러온 자료'):
        with self.lock:
            if sum(j['status'] in {'uploading', 'inspecting', 'running', 'cancelling'} for j in self.jobs.values()) >= 2:
                raise ValueError('import_busy')
            key = 'import_' + uuid.uuid4().hex[:16]
            path = self.root / key
            path.mkdir()
            (path / 'bundle').mkdir()
            self.jobs[key] = {'id': key, 'name': str(name)[:160], 'status': 'uploading', 'stage': 'upload',
                              'path': path, 'bytes': 0, 'files': set(), 'cancel': threading.Event()}
            return self.public(key)

    def job(self, key):
        with self.lock:
            if key not in self.jobs:
                if not re.fullmatch(r'import_[a-f0-9]{16}', str(key)):
                    raise ValueError('unknown_import')
                path = inside(self.root, key)
                info = read_json(inside(path, 'import.json'))
                if info.get('status') in {'failed', 'cancelled'}:
                    self.jobs[key] = {k: info[k] for k in ('name', 'status', 'stage', 'summary', 'error', 'options') if k in info}
                    self.jobs[key].update(id=key, path=path, cancel=threading.Event(), files=set())
                    return self.jobs[key]
                map_root = Path(info.get('map_root', '')).resolve()
                if info.get('status') not in {'ready', 'complete'} or not map_root.is_relative_to(self.root):
                    raise ValueError('external_import_requires_reselection')
                record = ImportedMap(map_root)
                self.jobs[key] = {**info, 'id': key, 'path': path, 'record': record, 'summary': record.summary(),
                    'output': map_root, 'cancel': threading.Event(), 'files': set()}
            return self.jobs[key]

    def public(self, key):
        with self.lock:
            job = self.job(key)
            value = {k: v for k, v in job.items() if k in {'id', 'name', 'status', 'stage', 'bytes', 'summary', 'error', 'options', 'feature_progress', 'progress'}}
            source = job.get('source')
            value['source_video'] = bool(source and Path(source).suffix.lower() in {'.mp4', '.mov', '.avi', '.mkv'})
            output = self.preview_root(job)
            record = job.get('record')
            value['history_available'] = bool(record.history_frames) if record else bool(
                output and (output / 'history/manifest.json').is_file() and (output / 'history/index.jsonl').is_file())
            return value

    def source_video(self, key):
        job = self.job(key)
        source = job.get('source')
        if not source or Path(source).suffix.lower() not in {'.mp4', '.mov', '.avi', '.mkv'}:
            raise ValueError('source_video_unavailable')
        # Path objects are explicitly registered in this running process by inspect().
        # Persisted strings must resolve inside this import's owned folder.
        target = Path(source).resolve()
        if isinstance(source, str):
            target = inside(job['path'], target.relative_to(job['path']).as_posix())
        if not target.is_file():
            raise ValueError('source_video_unavailable')
        return target

    def thumbnail(self, local_id):
        from io import BytesIO
        from PIL import Image
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,96}', str(local_id)):
            raise ValueError('unknown_local_record')
        folder = inside(self.root, local_id)
        job = self.jobs.get(local_id, {})
        record = job.get('record')
        if not record and (not folder.is_dir() or not any((folder / name).is_file() for name in
                ('import.json', 'status.json', 'state.json', 'reconstruction/status.json'))):
            raise ValueError('unknown_local_record')
        if record:
            root, result = record.root, record.result()
        else:
            root = folder
            if local_id.startswith('import_'):
                info = read_json(inside(folder, 'import.json'))
                if info.get('status') not in {'ready', 'complete'}:
                    raise ValueError('thumbnail_unavailable')
                selected = Path(info['map_root']).resolve()
                if not selected.is_relative_to(self.root):
                    raise ValueError('invalid_local_record')
                root = inside(self.root, selected.relative_to(self.root).as_posix())
            result = root / 'reconstruction' if (root / 'reconstruction/scene.json').is_file() else root
            if (root / 'revisions').is_dir():
                try:
                    state = read_json(inside(root, 'state.json'))
                except (OSError, ValueError):
                    state = {}
                preferred = state.get('revision_path', '')
                result = inside(root, preferred) if re.fullmatch(r'revisions/\d{6}', preferred) else None
                if result is None or not (result / 'status.json').is_file():
                    result = next((inside(root, 'revisions/' + p.name) for p in sorted((root / 'revisions').iterdir(), reverse=True)
                        if re.fullmatch(r'\d{6}', p.name) and (p / 'status.json').is_file()), None)
        if result is None or read_json(inside(result, 'status.json')).get('status') not in {'complete', 'partial'}:
            raise ValueError('thumbnail_unavailable')
        # Poses/assets contain frame references without loading all points or old revisions.
        assets = result / 'local_assets.json'
        poses = result / 'poses.json'
        metadata = assets if assets.is_file() else poses if poses.is_file() else result / 'scene.json'
        signature = (str(result), metadata.stat().st_mtime_ns)
        cached = self.thumbnail_cache.get(local_id)
        if cached and cached[0] == signature and cached[2].stat().st_mtime_ns == cached[3]:
            return cached[1]
        value = read_json(inside(result, metadata.name))
        names = value.get('frames') if metadata == assets else [c['frame_id'] for c in value.get('cameras', [])]
        images = next((p for p in [root / 'frames', root / 'input/frames', root / 'images',
            result / 'images', result / 'frames'] if p.is_relative_to(root) and p.is_dir()), None)
        if images is None or not names:
            raise ValueError('thumbnail_unavailable')
        target = inside(images, names[len(names) // 2])
        with Image.open(target) as image:
            image.thumbnail((480, 270))
            output = BytesIO()
            image.convert('RGB').save(output, format='JPEG', quality=80)
        payload = output.getvalue()
        if len(self.thumbnail_cache) >= 100:
            self.thumbnail_cache.pop(next(iter(self.thumbnail_cache)))
        self.thumbnail_cache[local_id] = (signature, payload, target, target.stat().st_mtime_ns)
        return payload

    def upload(self, key, name, stream, length):
        job = self.job(key)
        if type(length) is not int or not 0 <= length <= MAX_BYTES:
            raise ValueError('invalid_upload_size')
        with self.lock:
            if job['status'] != 'uploading' or name.casefold() in job['files']:
                raise ValueError('upload_not_available_or_duplicate')
            if len(job['files']) >= MAX_FILES or job['bytes'] + length > MAX_BYTES:
                raise ValueError('import_too_large')
            target = inside(job['path'] / 'bundle', name)
            if length > shutil.disk_usage(job['path']).free:
                raise ValueError('insufficient_disk_space')
            job['files'].add(name.casefold())
            job['bytes'] += length
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open('xb') as output:
            remaining = length
            while remaining:
                if job['cancel'].is_set():
                    raise InterruptedError('cancelled')
                block = stream.read(min(1024 * 1024, remaining))
                if not block:
                    raise ValueError('incomplete_upload')
                output.write(block)
                remaining -= len(block)

    def inspect(self, key, source=None, *, start_video=None):
        job = self.job(key)
        with self.lock:
            if job['status'] != 'uploading':
                raise ValueError('import_not_uploading')
            job.update(status='inspecting', stage='inspect')
        def work():
            try:
                root = Path(source).resolve() if source else job['path'] / 'bundle'
                files = list(root.iterdir()) if root.is_dir() else [root]
                if len(files) == 1 and files[0].suffix.lower() == '.zip':
                    destination = job['path'] / 'extracted'
                    extract_zip(files[0], destination, job['cancel'].is_set)
                    root = destination
                elif root.is_file():
                    root = files[0]
                root = detect_root(root) if root.is_dir() else root
                # Preserve the old CMD drop of a revision or reconstruction directory,
                # while binding images to its explicitly selected surrounding bundle.
                selected_revision = None
                if source and root.is_dir() and root.parent.name == 'revisions' and (root.parent.parent / 'state.json').is_file():
                    selected_revision = root.name
                    root = root.parent.parent
                elif source and root.name == 'reconstruction' and (root.parent / 'input/frames').is_dir():
                    root = root.parent
                if root.is_dir() and ((root / 'revisions').is_dir() or (root / 'scene.json').is_file() or (root / 'session.json').is_file()):
                    record = ImportedMap(root, job['cancel'].is_set)
                elif root.is_dir() and (root / 'reconstruction/scene.json').is_file():
                    record = ImportedMap(root, job['cancel'].is_set)
                else:
                    record = None
                if record:
                    if selected_revision and selected_revision in record.revisions and selected_revision != record.latest:
                        record.latest = selected_revision
                        record.state.update(revision=int(selected_revision), revision_path='revisions/' + selected_revision)
                        record._check_images()
                    job.update(record=record, summary=record.summary(), status='ready', stage='ready')
                else:
                    if root.is_dir() and (root / 'history/manifest.json').is_file():
                        raise ValueError('reconstruction_result_not_ready')
                    from modules.mapping.data.frame_input import inspect_input
                    info = inspect_input(root)
                    if start_video is not None:
                        if info['kind'] != 'video':
                            raise ValueError('video_required')
                        if job['cancel'].is_set():
                            raise InterruptedError('cancelled')
                        job.update(source=root, summary=info, stage='preflight')
                        start_video()
                        return
                    job.update(source=root, summary=info, status='awaiting_options', stage='confirm')
                if job['cancel'].is_set():
                    raise InterruptedError('cancelled')
                self._save(job)
            except Exception as error:
                job.update(status='cancelled' if job['cancel'].is_set() else 'failed', error=str(error)[:400])
                self._save(job)
        thread = threading.Thread(target=work, daemon=True)
        job['thread'] = thread
        thread.start()
        return self.public(key)

    def _save(self, job):
        value = self.public(job['id'])
        if job.get('record'):
            value['map_root'] = str(job['record'].root)
        if job.get('source'):
            value['source'] = str(job['source'])
        (job['path'] / 'import.json').write_text(json.dumps(value, ensure_ascii=False), encoding='utf-8')

    def resolve(self, key):
        job = self.job(key)
        if job['status'] not in {'ready', 'complete'} or not job.get('record'):
            raise ValueError('import_not_ready')
        return job['record']

    def add_local(self, source):
        info = self.create(Path(source).name)
        self.inspect(info['id'], source)
        return info['id']

    def library(self):
        rows = []
        seen_maps = set()
        for folder in sorted(self.root.iterdir(), key=lambda p: p.stat().st_mtime, reverse=True):
            if not folder.is_dir() or folder.is_symlink() or (hasattr(folder, 'is_junction') and folder.is_junction()):
                continue
            try:
                if folder.name.startswith('import_'):
                    if folder.name in self.jobs:
                        job = self.jobs[folder.name]
                        record = job.get('record')
                        identity = record.root if record else None
                        if identity and identity in seen_maps:
                            continue
                        if identity:
                            seen_maps.add(identity)
                        rows.append(self.public(folder.name))
                    elif (folder / 'import.json').is_file():
                        info = read_json(folder / 'import.json')
                        if info.get('status') in {'ready', 'complete'}:
                            identity = Path(info['map_root']).resolve() if info.get('map_root') else None
                            if identity and identity in seen_maps:
                                continue
                            if identity:
                                seen_maps.add(identity)
                        if info.get('status') in {'ready', 'complete', 'failed', 'cancelled'}:
                            fields = ('name', 'summary', 'status', 'stage', 'error')
                            rows.append({**{k: info[k] for k in fields if k in info}, 'local_id': folder.name})
                elif any((folder / n).is_file() for n in ('session.json', 'state.json', 'scene.json', 'reconstruction/scene.json')):
                    if folder in seen_maps:
                        continue
                    seen_maps.add(folder)
                    rows.append({'name': folder.name, 'local_id': folder.name, 'status': 'local'})
            except (OSError, ValueError, KeyError):
                continue
        for row in rows:
            folder = inside(self.root, row.get('local_id', row.get('id')))
            row['updated_at'] = folder.stat().st_mtime
        return rows[:100]

    def open_local(self, name):
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,96}', name):
            raise ValueError('invalid_local_id')
        folder = inside(self.root, name)
        if name.startswith('import_'):
            return self.public(name)
        if not folder.is_dir():
            raise ValueError('unknown_local_session')
        return self.public(self.add_local(folder))

    def delete_local(self, name, protected_paths=()):
        """Delete a selected managed record; never follow its external source path."""
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,96}', name):
            raise ValueError('invalid_local_id')
        with self.lock:
            if name in self.jobs and self.jobs[name]['status'] not in {'ready', 'complete', 'failed', 'cancelled'}:
                raise ValueError('record_busy')
            selected = inside(self.root, name)
            if not selected.is_dir():
                raise ValueError('unknown_local_session')
            info = read_json(selected / 'import.json') if (selected / 'import.json').is_file() else {}
            if not info and name not in self.jobs and not any((selected / n).is_file() for n in ('session.json', 'state.json', 'scene.json', 'reconstruction/scene.json')):
                raise ValueError('unknown_local_session')
            identity = Path(info['map_root']).resolve() if info.get('map_root') else selected.resolve()
            targets = {selected}
            # Imported aliases to a managed session must not bring a deleted card back.
            if identity.is_relative_to(self.root) and identity != self.root:
                targets.add(inside(self.root, identity.relative_to(self.root).parts[0]))
            for folder in self.root.iterdir():
                if not re.fullmatch(r'import_[a-f0-9]{16}', folder.name):
                    continue
                folder = inside(self.root, folder.name)
                if (folder / 'import.json').is_file():
                    alias = read_json(folder / 'import.json')
                    if alias.get('map_root') and Path(alias['map_root']).resolve() == identity:
                        targets.add(folder)
            busy = {'uploading', 'inspecting', 'awaiting_options', 'running', 'cancelling',
                    'starting', 'initializing', 'tracking', 'mapping', 'stopping'}
            for target in targets:
                # Verify the final absolute targets immediately before recursive removal.
                if target.resolve().parent != self.root or target.is_symlink() or (hasattr(target, 'is_junction') and target.is_junction()):
                    raise ValueError('invalid_delete_target')
                if any(Path(path).resolve().is_relative_to(target) for path in protected_paths):
                    raise ValueError('record_busy')
                for metadata in ('import.json', 'state.json'):
                    if (target / metadata).is_file():
                        state = read_json(target / metadata)
                        if state.get('status') in busy or state.get('session_status') in busy:
                            raise ValueError('record_busy')
                for job in self.jobs.values():
                    if job['status'] not in busy:
                        continue
                    paths = [job.get('path'), job.get('source'), job.get('output')]
                    if any(path and Path(path).resolve().is_relative_to(target) for path in paths):
                        raise ValueError('record_busy')
            for target in targets:
                shutil.rmtree(target)
            for key, job in list(self.jobs.items()):
                if any(Path(job['path']).resolve().is_relative_to(target) for target in targets):
                    self.jobs.pop(key)
            for root in list(self.history_readers):
                if any(Path(root).resolve().is_relative_to(target) for target in targets):
                    self.history_readers.pop(root)
            if getattr(self, 'initial', None) not in self.jobs:
                self.initial = None
            return {'status': 'deleted', 'local_id': name}

    def start(self, key, options, presentation_sync=False, *, inspected=False):
        from modules.mapping.data.frame_input import validate_options
        job = self.job(key)
        options = validate_options(options, job.get('summary', {}))
        if options['mode'] == 'depth_rgbd':
            import importlib.util
            if any(importlib.util.find_spec(name) is None for name in ('torch', 'transformers')):
                raise ValueError('depth_mapping_dependencies_missing')
            from modules.mapping.localization.orbslam3_live import require_slam_image
            try:
                require_slam_image()
            except RuntimeError as error:
                raise ValueError(str(error)) from error
        with self.lock:
            if job['cancel'].is_set() or job['status'] != ('inspecting' if inspected else 'awaiting_options'):
                raise ValueError('import_not_waiting_for_options')
            if any(j is not job and j['status'] in {'running', 'cancelling'} for j in self.jobs.values()):
                raise ValueError('reconstruction_busy')
            output = job['path'] / ('reconstruction_' + uuid.uuid4().hex[:8])
            config = job['path'] / 'options.json'
            config.write_text(json.dumps(options), encoding='utf-8')
            job.update(status='running', stage='input', options=options, output=output, presentation_sync=presentation_sync)
        def work():
            try:
                if options['mode'] == 'depth_rgbd':
                    source = Path(job['source'])
                    if source.is_dir():
                        videos = [p for p in source.iterdir() if p.suffix.lower() in VIDEO]
                        if len(videos) == 1 and not any(p.suffix.lower() in {'.jpg', '.jpeg', '.png'} for p in source.iterdir()):
                            source = videos[0]
                    def published(snapshot):
                        with self.preview_changed:
                            job['preview'] = snapshot
                            self.preview_changed.notify_all()
                            if presentation_sync:
                                self.preview_changed.wait_for(lambda: job['cancel'].is_set() or
                                    job.get('presented_revision', 0) >= snapshot['revision'])
                    summary = job.get('summary', {})
                    worker_options = dict(options)
                    if summary.get('width') and summary.get('height'):
                        worker_options['input_size'] = (summary['width'], summary['height'])
                    self.depth_worker.run(source, output, worker_options, stop_event=job['cancel'], on_preview=published)
                    code = 0
                else:
                    command = [sys.executable, '-B', '-m', 'modules.mapping.data.frame_input', '--source', str(job['source']),
                        '--output', str(output), '--options', str(config)]
                    with (job['path'] / 'worker.log').open('w', encoding='utf-8') as log:
                        proc = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                            env={**os.environ, 'OMP_NUM_THREADS': '2', 'PYTHONIOENCODING': 'utf-8'},
                            **({'creationflags': subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == 'nt' else {'start_new_session': True}))
                        job['process'] = proc
                        if job['cancel'].is_set():
                            self._terminate(proc)
                        code = proc.wait()
                if job['cancel'].is_set():
                    job.update(status='cancelled', stage='cancelled')
                elif code in {0, 3}:
                    record = ImportedMap(output)
                    if job['cancel'].is_set():
                        job.update(status='cancelled', stage='cancelled')
                    else:
                        job.update(record=record, summary=record.summary(), status='complete', stage='finished')
                else:
                    if options['mode'] == 'depth_rgbd' and (output / 'progress.json').is_file():
                        raise ValueError(read_json(output / 'progress.json').get('error', 'depth_mapping_failed'))
                    details = read_json(output / 'manifest.json').get('error', {}) if (output / 'manifest.json').is_file() else {}
                    raise ValueError(details.get('reason', f'reconstruction_failed_exit_{code}; worker.log'))
            except Exception as error:
                job.update(status='cancelled' if job['cancel'].is_set() else 'failed', error=str(error)[:400])
            finally:
                self._save(job)
                with self.preview_changed:
                    self.preview_changed.notify_all()
        job['thread'] = threading.Thread(target=work, daemon=True)
        job['thread'].start()
        return self.public(key)

    @staticmethod
    def _terminate(process):
        if process.poll() is not None:
            return
        if os.name == 'nt':
            subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'], capture_output=True, timeout=10)
        else:
            os.killpg(process.pid, signal.SIGTERM)

    def cancel(self, key):
        job = self.job(key)
        if job['status'] in {'ready', 'complete', 'failed', 'cancelled'}:
            return self.public(key)
        if job.get('live_session') is not None:
            job['live_session'].stop('user_stop')
            return self.public(key)
        with self.lock:
            if job['cancel'].is_set() or job['status'] in {'ready', 'complete', 'failed', 'cancelled'}:
                return self.public(key)
            job['status'] = 'cancelling'
            job['cancel'].set()
        with self.preview_changed:
            self.preview_changed.notify_all()
        if job.get('process'):
            self._terminate(job['process'])
        elif job['stage'] in {'upload', 'confirm'}:
            job.update(status='cancelled', stage='cancelled')
        # Stop transport first: it must not start a container after cancellation.
        container = job.get('output', Path()) / 'pose_tracker/container.json'
        if job.get('options', {}).get('mode') == 'depth_rgbd' and job.get('process') and container.is_file():
            name = read_json(container).get('name', '')
            if re.fullmatch(r'ssafy-pose-[a-f0-9]{12}', name):
                for attempt in range(6):
                    try:
                        stopped = subprocess.run(['docker', 'stop', '-t', '2', name], capture_output=True, timeout=10)
                        if stopped.returncode == 0:
                            break
                    except (OSError, subprocess.TimeoutExpired):
                        break
                    time.sleep(.1)  # A daemon-side create request may still be finishing.
        self._save(job)
        return self.public(key)

    def status(self, key):
        # Concurrent browser/status requests must not clear each other's progress.
        with self.lock:
            return self._status(key)

    def _status(self, key):
        job = self.job(key)
        job.pop('feature_progress', None)
        if job.get('output'):
            try:
                if job.get('options', {}).get('mode') == 'depth_rgbd':
                    job['progress'] = read_json(job['output'] / 'progress.json')
                    if job['status'] == 'running':
                        job['stage'] = job['progress']['stage']
                elif job['status'] == 'running':
                    job['stage'] = read_json(job['output'] / 'manifest.json')['stage']
            except (OSError, ValueError, KeyError):
                pass
            if job['status'] == 'running' and job.get('options', {}).get('mode') != 'depth_rgbd':
                if job['stage'] == 'features':
                    job['feature_progress'] = feature_progress(job['output'])
                else:
                    job['progress'] = stage_progress(job['output'], job['stage'])
        return self.public(key)

    def preview(self, key, revision=0, wait=0):
        job = self.job(key)
        output = self.preview_root(job)
        if not output:
            raise ValueError('progressive_preview_not_available')
        if wait and job.get('options', {}).get('mode') == 'depth_rgbd':
            with self.preview_changed:
                self.preview_changed.wait_for(lambda: job.get('preview', {}).get('revision', 0) > revision or
                    job['status'] not in {'running', 'cancelling'}, timeout=min(max(wait, 0), 2))
        path = inside(output, 'preview.json')
        value = job.get('preview')
        if value is None:
            if path.is_file():
                value = read_json(path)
            else:
                try:
                    reader = self.history_reader(key)
                    count = len(reader.refresh())
                    value = reader.snapshot(count) if count else None
                except (OSError, ValueError):
                    value = None
        done = job['status'] not in {'running', 'cancelling'}
        if value is None:
            return {'revision': 0, 'waiting': True, 'done': done}
        flags = {'done': done, 'needs_presentation': bool(job.get('presentation_sync') and
            job.get('presented_revision', 0) < value['revision'])}
        return {'revision': revision, 'unchanged': True, **flags} if value['revision'] == revision else {**value, **flags}

    def presented(self, key, revision):
        job = self.job(key)
        with self.preview_changed:
            if type(revision) is not int or not 1 <= revision <= job.get('preview', {}).get('revision', 0):
                raise ValueError('unknown_preview_revision')
            job['presented_revision'] = max(job.get('presented_revision', 0), revision)
            self.preview_changed.notify_all()
        return {'revision': revision}

    def preview_frame(self, key, name):
        job = self.job(key)
        output = self.preview_root(job)
        if not output or not re.fullmatch(r'frame_\d{7,}\.jpg', name):
            raise ValueError('unknown_preview_frame')
        return inside(output, 'frames/' + name)

    @staticmethod
    def preview_root(job):
        if job.get('output'):
            return Path(job['output'])
        record = job.get('record')
        return record.result_root if record else None

    def history_reader(self, key):
        from modules.mapping.data.mapping_history import HistoryReader
        output = self.preview_root(self.job(key))
        if not output or not (output / 'history/manifest.json').is_file():
            raise ValueError('reconstruction_history_not_available')
        inside(output, 'history/manifest.json')
        inside(output, 'history/index.jsonl')
        with self.lock:
            return self.history_readers.setdefault(str(output), HistoryReader(output))

    def history_timeline(self, key):
        return self.history_reader(key).timeline()

    def history_snapshot(self, key, revision):
        return self.history_reader(key).snapshot(revision)

    def history_archive(self, key):
        if self.job(key)['status'] not in {'ready', 'complete'}:
            raise ValueError('reconstruction_result_not_ready')
        reader = self.history_reader(key)
        output = reader.root.parent
        try:
            scene = scene_at(output)
        except (OSError, ValueError, KeyError) as error:
            raise ValueError('reconstruction_result_not_ready') from error
        # Keep the exported index and its chunks on the same revision.
        entries = [dict(entry) for entry in reader.refresh()]
        manifest = read_json(reader.root / 'manifest.json')
        manifest.update(frames=len(entries), exported_last_revision=entries[-1]['revision'] if entries else 0)
        metadata = {'history/manifest.json': json.dumps(manifest, ensure_ascii=False),
                    'history/index.jsonl': ''.join(json.dumps(e, ensure_ascii=False) + '\n' for e in entries)}
        if entries:
            metadata['preview.json'] = json.dumps(reader.snapshot(entries[-1]['revision']), ensure_ascii=False)
        files = []
        frames = {}
        for entry in entries:
            chunk = f"history/frames/{entry['revision']:08d}.npz"
            files.append((inside(output, chunk), chunk))
            name = entry.get('frame_id', '')
            if not re.fullmatch(r'frame_\d{7,}\.jpg', name):
                raise ValueError('invalid_history_frame_name')
            frames[name] = inside(output, 'frames/' + name)
        for camera in scene['cameras']:
            name = camera['frame_id']
            if not re.fullmatch(r'frame_\d{7,}\.jpg', name):
                raise ValueError('invalid_history_frame_name')
            frames[name] = inside(output, 'frames/' + name)
        files.extend((path, 'frames/' + name) for name, path in frames.items())
        metadata['local_assets.json'] = json.dumps({'images_root': 'frames', 'frames': list(frames)}, ensure_ascii=False)
        for name in ('scene.json', 'status.json', 'poses.json', 'trajectory.csv', 'points.ply', 'pose_tracker/camera.yaml', 'pose_tracker/native.json'):
            if (output / name).is_file():
                files.append((inside(output, name), name))
        for path in (output / 'submaps').glob('*.json'):
            name = path.relative_to(output).as_posix()
            files.append((inside(output, name), name))
        if any(not path.is_file() for path, _ in files):
            raise ValueError('reconstruction_history_incomplete')
        return files, metadata

    def shutdown(self):
        for key, job in list(self.jobs.items()):
            if job['status'] in {'running', 'cancelling', 'uploading', 'awaiting_options', 'inspecting'}:
                self.cancel(key)
        for job in list(self.jobs.values()):
            if job.get('thread'):
                job['thread'].join(timeout=30)
        self.depth_worker.close()
