"""Local Explorer preference for the mapping console."""

import json
import os
from pathlib import Path
from tempfile import NamedTemporaryFile
import time


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


def video_folder(root, body=None):
    settings = root / '.video-folder.json'
    try:
        saved = json.loads(settings.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        saved = None
    configured = saved.get('folder') if isinstance(saved, dict) else None
    if body is not None:
        action = body['action']
        if action == 'open':
            if not hasattr(os, 'startfile'):
                raise ValueError('video_folder_open_unavailable')
            path = local_directory(configured or str(root))
            try:
                os.startfile(str(path))
            except OSError as error:
                raise ValueError('video_folder_open_failed') from error
        elif action in {'set', 'reset'}:
            configured = str(local_directory(body['folder'])) if action == 'set' else None
            with NamedTemporaryFile(dir=root, prefix='.video-folder-', delete=False) as file:
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
    return {'folder': configured or str(root), 'can_open': hasattr(os, 'startfile')}
