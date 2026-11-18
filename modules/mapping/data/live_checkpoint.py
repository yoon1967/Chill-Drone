"""Cumulative, immutable exports of a live session without stopping its receiver."""
from copy import deepcopy
import json
from pathlib import Path
import shutil
import time

from .live_recording import atomic_json, file_hash, remux_video, utc_now
from .session_import import inside, read_json


def capture_map(session):
    """Pin only committed revisions; an in-flight depth inference belongs to a later save."""
    with session.condition:
        state = deepcopy(session.stats())
        preview = getattr(session, 'preview', None) or {}
        revision = preview.get('revision', state.get('revision', 0))
    root = Path(session.path)
    history = root / 'reconstruction/history'
    if history.is_dir():
        entries = []
        with (history / 'index.jsonl').open(encoding='utf-8') as stream:
            for line in stream:
                try:
                    row = json.loads(line)
                except ValueError:
                    break  # An incomplete append is not a committed observation.
                if row['revision'] <= revision:
                    entries.append(row)
        if not any(entry.get('observation') for entry in entries):
            raise ValueError('checkpoint_map_not_ready')
        return {'kind': 'history', 'root': root / 'reconstruction', 'entries': entries,
                'manifest': read_json(history / 'manifest.json'), 'state': state,
                'revision': entries[-1]['revision']}
    # LiveSession.stats() describes transport, not the native publisher's map.
    # state.json is published atomically only after its revision files exist.
    try:
        published = read_json(inside(root, 'state.json'))
    except FileNotFoundError:
        published = {}
    revision_path = published.get('revision_path')
    if not revision_path:
        raise ValueError('checkpoint_map_not_ready')
    state = {**published, **state, **{key: published[key] for key in
             ('revision', 'revision_path', 'map_available', 'segments', 'segment_id') if key in published}}
    revision = published.get('revision', 0)
    scene = read_json(inside(root, revision_path + '/scene.json'))
    if not scene.get('points'):
        raise ValueError('checkpoint_map_not_ready')
    return {'kind': 'scene', 'root': root, 'scene': scene, 'state': state, 'revision': revision}


def copy_prefix(source, destination, size):
    destination.parent.mkdir(parents=True, exist_ok=True)
    with source.open('rb') as reader, destination.open('xb') as writer:
        while size:
            chunk = reader.read(min(size, 1 << 20))
            if not chunk:
                raise ValueError('checkpoint_source_truncated')
            writer.write(chunk)
            size -= len(chunk)


def save_map(pin, destination, run_id):
    output = destination / 'reconstruction'
    output.mkdir()
    frames = set()
    if pin['kind'] == 'history':
        from .stream_mapping import finalize_history
        from .mapping_history import HistoryReader
        history = output / 'history'
        (history / 'frames').mkdir(parents=True)
        entries = pin['entries']
        for entry in entries:
            name = f"history/frames/{entry['revision']:08d}.npz"
            shutil.copyfile(inside(pin['root'], name), inside(output, name))
            frames.add(entry['frame_id'])
        (history / 'index.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in entries), encoding='utf-8')
        atomic_json(history / 'manifest.json', {**pin['manifest'], 'status': 'complete', 'frames': len(entries),
            'exported_last_revision': pin['revision']})
        finalize_history(output, {}, run_id, time.monotonic())
        atomic_json(output / 'preview.json', HistoryReader(output).snapshot(pin['revision']))
    else:
        scene = deepcopy(pin['scene'])
        scene['input_run_id'] = run_id
        atomic_json(output / 'scene.json', scene)
        atomic_json(output / 'status.json', {'status': scene.get('quality', {}).get('result_status', 'partial')})
        frames.update(camera['frame_id'] for camera in scene.get('cameras', []))
    (output / 'frames').mkdir()
    for frame in frames:
        shutil.copyfile(inside(pin['root'], 'frames/' + frame), inside(output, 'frames/' + frame))
    atomic_json(output / 'local_assets.json', {'images_root': 'frames', 'frames': sorted(frames)})
    return read_json(output / 'scene.json')


def save_checkpoint(recording, job, prefix):
    """Only this worker writes the new folder; the original session remains append-only."""
    from .video_sensor_sync import write_sync
    root = recording.path
    destination = root.parent / job['id']
    try:
        destination.mkdir()
        atomic_json(destination / 'checkpoint.json', {'status': 'saving', 'source_session_id': root.name,
            'cutoff_at': job['cutoff_at'], 'cutoff_monotonic': job['cutoff_monotonic']})
        for name in ('camera.json', 'settings.json'):
            shutil.copyfile(inside(root, name), destination / name)
        for name, size in prefix['sizes'].items():
            copy_prefix(inside(root, name), inside(destination, name), size)
        parts, clocks = prefix['parts'], {}
        errors = []
        for part in parts:
            raw = inside(destination, part['raw_path'])
            part.update(bytes=raw.stat().st_size, sha256=file_hash(raw), status='captured')
            part.pop('playback', None)
            part.pop('receiver_bytes', None)
            part.pop('receiver_packets', None)
            part.pop('capture_complete', None)
            target = raw.with_suffix('.mp4' if part['mode'] == 'h264' else '.mkv')
            clocks[part['id']] = []
            try:
                result = remux_video(raw, inside(destination, part['index_path']), target, part['mode'],
                    recording.camera.get('fps', 24), clock_samples=clocks[part['id']], source_run_id=part.get('source_run_id'))
                part['playback'] = {**result, 'path': target.relative_to(destination).as_posix()}
            except Exception as error:
                part['playback_error'] = str(error)[:200]
                errors.append('playback_unavailable')
        scene = save_map(job['map'], destination, destination.name)
        state = {**job['map']['state'], 'session_id': destination.name, 'status': 'complete', 'session_status': 'complete',
                 'map_available': True, 'revision_path': 'reconstruction', 'revision': job['map']['revision'],
                 'stop_reason': 'manual_checkpoint', 'transport_active': False}
        # Original import jobs and their absolute paths are not part of this saved session.
        state.pop('depth_job', None)
        state.pop('depth_job_id', None)
        state.pop('recording', None)
        atomic_json(destination / 'state.json', state)
        sync = write_sync(destination, parts, clocks, recording.sensor_required)
        if recording.sensor_required and sync['status'] in {'partial', 'unavailable'}:
            errors.append('video_sensor_sync_incomplete')
        metadata = {'source_session_id': root.name, 'cutoff_at': job['cutoff_at'],
                    'cutoff_monotonic': job['cutoff_monotonic'], 'scope': 'session_start_to_request',
                    'map_revision': job['map']['revision'], 'map_scope': 'committed_observations_at_request'}
        manifest = prefix['manifest']
        manifest.update(session_id=destination.name, ended_at=job['cutoff_at'], checkpoint=metadata,
            recordings=parts, mapping=state, scale=scene.get('scale', {}).get('unit', 'arbitrary'))
        manifest['recording'].update(finalized=True, status='partial' if errors or manifest['recording'].get('error') else 'complete', queued_bytes=0,
            error='; '.join(errors) or manifest['recording'].get('error'),
            bytes_received=sum(part['bytes'] for part in parts), bytes_written=sum(part['bytes'] for part in parts),
            parts=len(parts), video_sensor_sync={'path': 'video_sensor_sync.json', 'status': sync['status']})
        manifest['asset_sha256'] = {name: file_hash(destination / name) for name in
            ('camera.json', 'settings.json', 'state.json', 'sensor.jsonl', 'frames.jsonl', 'video_sensor_sync.json')
            if (destination / name).is_file()}
        atomic_json(destination / 'session.json', manifest)
        atomic_json(destination / 'checkpoint.json', {'status': 'complete', **metadata})
        with recording.condition:
            job.update(status='complete', path=str(destination), error=None, completed_at=utc_now(),
                       capture_status=manifest['recording']['status'], sync_status=sync['status'])
    except Exception as error:
        with recording.condition:
            job.update(status='failed', error=type(error).__name__ + ': ' + str(error)[:200])
        if destination.is_dir():
            atomic_json(destination / 'checkpoint.json', {'status': 'failed', 'error': job['error']})
    finally:
        with recording.condition:
            job.pop('map', None)
