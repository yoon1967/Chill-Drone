"""Local import jobs and portable bindings for the existing console viewer.

Input files are never executed, rewritten, or used as absolute asset paths.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import signal
import subprocess
import sys
import threading
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
                self.warnings.append('부분 복원 결과입니다. 연결되지 않은 프레임과 다른 모델의 좌표는 합치지 않았습니다.')
            images = self.image_root(result)
            missing = sum(not images or not inside(images, c['frame_id']).is_file() for c in scene['cameras'])
            if missing:
                self.warnings.append(f'키프레임 이미지 {missing}개가 없습니다. 지도와 경로는 표시합니다.')
        else:
            self.points = self.cameras = 0

    def summary(self):
        return {'kind': 'session' if self.is_session else 'result', 'name': self.root.name,
            'session_id': self.state.get('session_id'), 'points': self.points, 'keyframes': self.cameras,
            'revisions': len(self.entries), 'warnings': self.warnings,
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
                map_root = Path(info.get('map_root', '')).resolve()
                if info.get('status') not in {'ready', 'complete'} or not map_root.is_relative_to(self.root):
                    raise ValueError('external_import_requires_reselection')
                record = ImportedMap(map_root)
                self.jobs[key] = {**info, 'id': key, 'path': path, 'record': record, 'summary': record.summary(),
                    'cancel': threading.Event(), 'files': set()}
            return self.jobs[key]

    def public(self, key):
        with self.lock:
            job = self.job(key)
            return {k: v for k, v in job.items() if k in {'id', 'name', 'status', 'stage', 'bytes', 'summary', 'error', 'options'}}

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

    def inspect(self, key, source=None):
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
                    from modules.mapping.data.frame_input import inspect_input
                    info = inspect_input(root)
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
        for folder in sorted(self.root.iterdir(), key=lambda p: p.stat().st_mtime, reverse=True):
            if not folder.is_dir() or folder.is_symlink() or (hasattr(folder, 'is_junction') and folder.is_junction()):
                continue
            try:
                if folder.name.startswith('import_'):
                    if folder.name in self.jobs:
                        rows.append(self.public(folder.name))
                    elif (folder / 'import.json').is_file():
                        info = read_json(folder / 'import.json')
                        if info.get('status') in {'ready', 'complete'}:
                            rows.append({**{k: info[k] for k in ('name', 'summary', 'status')}, 'local_id': folder.name})
                elif any((folder / n).is_file() for n in ('session.json', 'state.json', 'scene.json', 'reconstruction/scene.json')):
                    rows.append({'name': folder.name, 'local_id': folder.name, 'status': 'local'})
            except (OSError, ValueError, KeyError):
                continue
        return rows[:100]

    def open_local(self, name):
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,96}', name):
            raise ValueError('invalid_local_id')
        folder = inside(self.root, name)
        if name.startswith('import_'):
            if name in self.jobs:
                return self.public(name)
            info = read_json(folder / 'import.json')
            # Only paths retained by this app, confined to its local data root, are reopened.
            root = Path(info.get('map_root', '')).resolve()
            if not root.is_relative_to(self.root):
                raise ValueError('external_import_requires_reselection')
            return self.public(self.add_local(root))
        if not folder.is_dir():
            raise ValueError('unknown_local_session')
        return self.public(self.add_local(folder))

    def start(self, key, options):
        from modules.mapping.data.frame_input import validate_options
        job = self.job(key)
        options = validate_options(options, job.get('summary', {}))
        with self.lock:
            if job['status'] != 'awaiting_options':
                raise ValueError('import_not_waiting_for_options')
            if any(j['status'] == 'running' for j in self.jobs.values()):
                raise ValueError('reconstruction_busy')
            job.update(status='running', stage='input', options=options)
        output = job['path'] / ('reconstruction_' + uuid.uuid4().hex[:8])
        job['output'] = output
        config = job['path'] / 'options.json'
        config.write_text(json.dumps(options), encoding='utf-8')
        def work():
            try:
                command = [sys.executable, '-B', '-m', 'modules.mapping.data.frame_input', '--source', str(job['source']),
                    '--output', str(output), '--options', str(config)]
                with (job['path'] / 'worker.log').open('w', encoding='utf-8') as log:
                    proc = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                        **({'creationflags': subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == 'nt' else {'start_new_session': True}))
                    job['process'] = proc
                    if job['cancel'].is_set():
                        self._terminate(proc)
                    code = proc.wait()
                if job['cancel'].is_set():
                    job.update(status='cancelled', stage='cancelled')
                elif code in {0, 3}:
                    record = ImportedMap(output)
                    job.update(record=record, summary=record.summary(), status='complete', stage='finished')
                else:
                    details = read_json(output / 'manifest.json').get('error', {}) if (output / 'manifest.json').is_file() else {}
                    raise ValueError(details.get('reason', f'reconstruction_failed_exit_{code}; worker.log'))
            except Exception as error:
                job.update(status='cancelled' if job['cancel'].is_set() else 'failed', error=str(error)[:400])
            finally:
                self._save(job)
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
        job['cancel'].set()
        job['status'] = 'cancelling'
        if job.get('process'):
            self._terminate(job['process'])
        elif job['stage'] in {'upload', 'confirm'}:
            job.update(status='cancelled', stage='cancelled')
        self._save(job)
        return self.public(key)

    def status(self, key):
        job = self.job(key)
        if job['status'] == 'running' and job.get('output'):
            try:
                job['stage'] = read_json(job['output'] / 'manifest.json')['stage']
            except (OSError, ValueError, KeyError):
                pass
        return self.public(key)

    def shutdown(self):
        for key, job in list(self.jobs.items()):
            if job['status'] in {'running', 'uploading', 'awaiting_options', 'inspecting'}:
                self.cancel(key)
        for job in list(self.jobs.values()):
            if job.get('thread'):
                job['thread'].join(timeout=5)
