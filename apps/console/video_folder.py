"""Local source folder preferences and video selection for the mapping console."""

import json
import os
from pathlib import Path
from tempfile import NamedTemporaryFile
import time

from modules.mapping.data.session_import import VIDEO, inside


def local_directory(value):
    if not isinstance(value, str) or not value or len(value) > 4096:
        raise ValueError('invalid_video_folder')
    path = Path(value)
    if not path.is_absolute() or (os.name == 'nt' and len(path.drive) != 2):
        raise ValueError('invalid_video_folder')
    try:
        path = path.resolve(strict=True)
        if not path.is_dir() or (os.name == 'nt' and len(path.drive) != 2):
            raise ValueError('invalid_video_folder')
    except OSError as error:
        raise ValueError('invalid_video_folder') from error
    return path


def input_folder(root, kind, body=None):
    settings = root / f'.{kind}-folder.json'
    try:
        saved = json.loads(settings.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        saved = None
    configured = saved.get('folder') if isinstance(saved, dict) else None
    if not isinstance(configured, str):
        configured = None
    if body is not None:
        action = body['action']
        if action == 'open':
            folder = configured or str(root)
            if body.get('folder', folder) != folder:
                raise ValueError('input_folder_changed')
            if not hasattr(os, 'startfile'):
                raise ValueError('video_folder_open_unavailable')
            path = local_directory(folder)
            try:
                os.startfile(str(path))
            except OSError as error:
                raise ValueError('video_folder_open_failed') from error
        elif action in {'set', 'reset'}:
            configured = str(local_directory(body['folder'])) if action == 'set' else None
            with NamedTemporaryFile(dir=root, prefix=f'.{kind}-folder-', delete=False) as file:
                temporary = Path(file.name)
            try:
                temporary.write_text(json.dumps({'folder': configured}, ensure_ascii=False), encoding='utf-8')
                # Windows can briefly hold the destination open while another console reads it.
                for attempt in range(40):
                    try:
                        temporary.replace(settings)
                        break
                    except PermissionError:
                        if attempt == 39:
                            raise
                        time.sleep(.005)
            finally:
                temporary.unlink(missing_ok=True)
        else:
            raise ValueError('invalid_video_folder_action')
    return {'folder': configured or str(root), 'configured': bool(configured), 'can_open': hasattr(os, 'startfile')}


def folder_video(folder, name):
    if not isinstance(name, str) or '/' in name or Path(name).suffix.lower() not in VIDEO:
        raise ValueError('video_file_unavailable')
    path = inside(folder, name)
    if not path.is_file():
        raise ValueError('video_file_unavailable')
    return path


def folder_videos(folder):
    names = []
    for entry in folder.iterdir():
        if entry.suffix.lower() not in VIDEO:
            continue
        try:
            names.append(folder_video(folder, entry.name).name)
        except (OSError, ValueError):
            continue
    return sorted(names, key=str.casefold)
