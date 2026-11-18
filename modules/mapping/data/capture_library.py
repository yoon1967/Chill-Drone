"""Read-only selection of recorded video/sensor bundles, without reconstruction."""
from pathlib import Path
import os
import re
from urllib.parse import urlencode

from .session_import import inside, read_json

SCHEMA = 'mapping-capture-selection/1.0'
METADATA = {'session.json': 'session', 'sensor.jsonl': 'sensor',
            'video_sensor_sync.json': 'sync', 'camera.json': 'camera', 'settings.json': 'settings'}


def open_capture_folder(root):
    """Always open the real recording root; the Explorer shortcut preference is unrelated."""
    path = Path(root).resolve(strict=True)
    if not path.is_dir() or not hasattr(os, 'startfile'):
        raise ValueError('capture_folder_open_unavailable')
    os.startfile(str(path))
    return {'folder': str(path), 'opened': True}


def capture_bundle(root, session_id):
    root = Path(root).resolve()
    if not isinstance(session_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,96}', session_id):
        raise ValueError('invalid_capture_id')
    folder = inside(root, session_id)
    manifest = read_json(inside(folder, 'session.json'))
    if manifest.get('input') != 'tello_camera':
        raise ValueError('not_drone_capture')
    files, missing, parts = {}, [], []

    def add(name, role):
        path = inside(folder, name)
        if not path.is_file() or not path.stat().st_size:
            return False
        files[name] = {'path': name, 'role': role, 'bytes': path.stat().st_size,
                       'url': '/api/import/capture-file?' + urlencode({'session_id': session_id, 'path': name})}
        return True

    for name, role in METADATA.items():
        if not add(name, role) and role in {'session', 'sensor', 'sync'}:
            missing.append(name)
    recordings = manifest.get('recordings', [])
    if not isinstance(recordings, list):
        raise ValueError('invalid_capture_recordings')
    for part in recordings:
        playback = part.get('playback') or {}
        video, index = playback.get('path'), part.get('index_path')
        # Only declared playback media and its input timestamp index are selectable.
        if not isinstance(video, str) or not re.fullmatch(r'recordings/[A-Za-z0-9_-]+\.(mkv|mp4)', video):
            missing.append('recordings/video')
            continue
        if not add(video, 'video'):
            missing.append(video)
            continue
        if not isinstance(index, str) or not re.fullmatch(r'recordings/[A-Za-z0-9_-]+\.jsonl', index):
            missing.append('recordings/index.jsonl')
            index = None
        elif not add(index, 'video_index'):
            missing.append(index)
        parts.append({'part_id': part.get('id'), 'video_path': video, 'index_path': index,
                      'duration_s': playback.get('duration_s')})
    if not parts:
        missing.append('recordings/video')
    sync_status = 'unavailable'
    if 'video_sensor_sync.json' in files:
        try:
            sync = read_json(inside(folder, 'video_sensor_sync.json'))
            if sync.get('schema_version') == 'video-sensor-sync/1.0':
                sync_status = sync.get('status', 'unavailable')
        except (OSError, ValueError):
            pass
        if sync_status not in {'complete', 'partial', 'approximate'}:
            missing.append('video_sensor_sync.json (unavailable)')
    finalized = manifest.get('recording', {}).get('finalized') is True
    status = 'recording' if not finalized else 'incomplete' if missing else 'ready'
    return {'schema_version': SCHEMA, 'session_id': session_id, 'name': session_id,
            'saved_at': (manifest.get('checkpoint') or {}).get('cutoff_at'),
            'started_at': manifest.get('started_at'), 'ended_at': manifest.get('ended_at'),
            'status': status, 'selectable': status == 'ready', 'sync_status': sync_status,
            'missing': sorted(set(missing)), 'parts': parts, 'files': list(files.values()),
            'reconstruction_supported': False}


def capture_library(root):
    root = Path(root).resolve()
    items = []
    for folder in root.iterdir():
        if not folder.is_dir():
            continue
        try:
            bundle = capture_bundle(root, folder.name)
            if bundle['parts'] or bundle['status'] == 'recording':
                items.append(bundle)
        except (OSError, ValueError, KeyError, TypeError, AttributeError):
            continue
    items.sort(key=lambda row: (row.get('saved_at') or row.get('started_at') or '', row['session_id']), reverse=True)
    return {'root_path': str(root), 'items': items[:100]}


def select_capture(root, session_id):
    value = capture_bundle(root, session_id)
    if not value['selectable']:
        raise ValueError('capture_recording' if value['status'] == 'recording' else 'capture_incomplete')
    return value


def capture_file(root, session_id, name):
    # Rebuild the allowlist on access. Never follow arbitrary assets in session.json.
    value = select_capture(root, session_id)
    if name not in {file['path'] for file in value['files']}:
        raise ValueError('capture_file_not_selected')
    return inside(inside(Path(root).resolve(), session_id), name)


def has_saved_scene(folder):
    if any(inside(folder, name).is_file() for name in ('scene.json', 'reconstruction/scene.json')):
        return True
    state = inside(folder, 'state.json')
    if state.is_file():
        revision = read_json(state).get('revision_path')
        if isinstance(revision, str) and inside(folder, revision + '/scene.json').is_file():
            return True
    revisions = inside(folder, 'revisions')
    return revisions.is_dir() and any(
        re.fullmatch(r'\d{6}', entry.name) and inside(folder, f'revisions/{entry.name}/scene.json').is_file()
        for entry in revisions.iterdir())


def reconstruction_library(imports):
    """Keep the legacy library intact; hide live-session aliases in this view only."""
    rows = []
    for row in imports.library():
        try:
            folder = inside(imports.root, row.get('local_id', row.get('id')))
            if (folder / 'import.json').is_file():
                info = read_json(inside(folder, 'import.json'))
                if info.get('map_root'):
                    folder = Path(info['map_root']).resolve()
                    if not folder.is_relative_to(imports.root):
                        continue
            if (folder / 'session.json').is_file() and read_json(inside(folder, 'session.json')).get('input') == 'tello_camera':
                continue
            if row.get('status') == 'local' and not has_saved_scene(folder):
                continue
            rows.append(row)
        except (OSError, ValueError, TypeError):
            continue
    return {'items': rows}


def editable_maps(imports, active_path=None):
    """List saved scenes, including completed capture checkpoints and older local maps."""
    items, seen = [], set()
    for row in imports.library(limit=None):
        status = row.get('status')
        if status not in {'local', 'ready', 'complete'} or (row.get('summary') or {}).get('points') == 0:
            continue
        key = row.get('local_id') or row.get('id')
        try:
            folder = inside(imports.root, key)
            if status != 'local' and (folder / 'import.json').is_file():
                info = read_json(inside(folder, 'import.json'))
                if info.get('map_root'):
                    folder = Path(info['map_root']).resolve()
                    if not folder.is_relative_to(imports.root) and not row.get('id'):
                        continue
            if folder == active_path or folder in seen:
                continue
            if status == 'local':
                if not has_saved_scene(folder):
                    continue
                if (folder / 'checkpoint.json').is_file() and read_json(inside(folder, 'checkpoint.json')).get('status') != 'complete':
                    continue
            seen.add(folder)
            identity = folder.name if folder.parent == imports.root else key
            item = {'key': identity, 'name': row.get('name') or identity, 'updated_at': row.get('updated_at')}
            item['local_id' if status == 'local' else 'asset'] = key
            items.append(item)
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return items
