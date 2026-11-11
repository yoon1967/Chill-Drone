"""Portable video-time to sensor-clock mapping, independent of mapping decisions."""
from bisect import bisect_left
import json
import math
from pathlib import Path

SCHEMA = 'video-sensor-sync/1.0'
FILENAME = 'video_sensor_sync.json'


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def playback_times(path):
    import av
    with av.open(str(path)) as video:
        stream = video.streams.video[0]
        resolution = float(stream.time_base)
        times = [float(packet.pts * packet.time_base) if packet.pts is not None else None
                 for packet in video.demux(stream) if packet.size]
    return times, resolution


def inside(root, relative):
    path = (root / relative).resolve()
    path.relative_to(root.resolve())
    return path


def describe_part(root, part, clocks, sensor_runs, required):
    from .live_recording import file_hash
    playback = part.get('playback') or {}
    result = {'part_id': part['id'], 'video_path': playback.get('path'),
              'status': 'unavailable', 'source_run_id': None, 'samples': []}
    if not required:
        result['status'] = 'not_applicable'
        return result
    if not playback.get('path'):
        result['reason'] = 'playback_unavailable'
        return result
    path = inside(root, playback['path'])
    result['video_sha256'] = file_hash(path)
    if playback.get('sha256') and playback['sha256'] != result['video_sha256']:
        result['reason'] = 'video_hash_changed'
        return result
    times, resolution = playback_times(path)
    result['video_time_resolution_s'] = resolution
    if any(not finite(stamp) for stamp in times):
        result['reason'] = 'video_pts_missing'
        return result
    if len(clocks) != len(times):
        result['reason'] = 'video_and_clock_sample_count_mismatch'
        return result
    ids = {clock['run_id'] for clock in clocks if isinstance(clock, dict)
           and isinstance(clock.get('run_id'), str) and finite(clock.get('timestamp_s'))}
    if len(ids) != 1 or next(iter(ids), None) not in sensor_runs:
        result['reason'] = 'sensor_clock_identity_missing_or_mixed'
        return result
    run_id = result['source_run_id'] = next(iter(ids))
    samples = []
    missing = 0
    for i, stamp in enumerate(times):
        clock = clocks[i] if i < len(clocks) else None
        sensor_stamp = (clock.get('timestamp_s') if isinstance(clock, dict)
                        and clock.get('run_id') == run_id else None)
        if not finite(sensor_stamp):
            missing += 1
            sensor_stamp = None
        samples.append([stamp, sensor_stamp])
    duplicate = sum(a[0] == b[0] for a, b in zip(samples, samples[1:]))
    backward = sum(b[0] < a[0] or (finite(a[1]) and finite(b[1]) and b[1] < a[1])
                   for a, b in zip(samples, samples[1:]))
    result.update(samples=samples, coverage={'video_samples': len(times), 'missing_clock_samples': missing,
        'duplicate_video_times': duplicate, 'backward_times': backward},
        sensor_available_range_s=sensor_runs[run_id],
        accuracy='packet_receive_approximation' if part['mode'] == 'h264' else 'gateway_decode_receive_alignment')
    usable = sum(finite(pair[1]) for pair in samples)
    result['status'] = ('unavailable' if not usable else 'partial' if missing or duplicate or backward
                        else 'approximate' if part['mode'] == 'h264' else 'complete')
    return result


def write_sync(root, parts, clocks_by_part, required, *, provenance='recording_timestamps'):
    from .live_recording import atomic_json, file_hash
    root = Path(root)
    sensor = root / 'sensor.jsonl'
    sensor_runs = {}
    if sensor.is_file():
        with sensor.open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                if row.get('event') != 'sample' or not finite(row.get('received_mono')):
                    continue
                key, stamp = row.get('source_run_id'), row['received_mono']
                if key not in sensor_runs:
                    sensor_runs[key] = [stamp, stamp]
                else:
                    sensor_runs[key] = [min(sensor_runs[key][0], stamp), max(sensor_runs[key][1], stamp)]
    descriptions = [describe_part(root, part, clocks_by_part.get(part['id'], []), sensor_runs, required)
                    for part in parts if part.get('bytes')]
    states = {part['status'] for part in descriptions}
    usable = states & {'complete', 'approximate', 'partial'}
    status = ('not_applicable' if not descriptions or not required else 'unavailable' if not usable else
              'partial' if states & {'partial', 'unavailable'} else 'approximate' if 'approximate' in states else 'complete')
    value = {'schema_version': SCHEMA, 'session_id': root.name, 'status': status,
             'sensor_path': 'sensor.jsonl', 'sensor_sha256': file_hash(sensor) if sensor.is_file() else None,
             'sample_fields': ['video_time_s', 'sensor_received_mono_s'], 'time_unit': 'seconds',
             'sensor_clock': 'source_pc_receive_monotonic', 'provenance': provenance,
             'interpolation': 'linear_between_adjacent_unique_times; no extrapolation or bridging null values',
             'accuracy_scope': 'receive timestamps, not measured camera exposure time', 'parts': descriptions}
    atomic_json(root / FILENAME, value)
    return value


def sensor_time_at(sync, part_id, video_time_s):
    """Return sensor run/time for any caller-selected video time, or None if ambiguous."""
    if not finite(video_time_s):
        return None
    part = next((p for p in sync['parts'] if p['part_id'] == part_id), None)
    if not part or not part['source_run_id'] or (part.get('coverage') or {}).get('backward_times'):
        return None
    pairs = part['samples']
    times = [pair[0] for pair in pairs]
    i = bisect_left(times, video_time_s)
    def unique(index):
        return (0 <= index < len(times) and finite(pairs[index][1])
                and (index == 0 or times[index] != times[index - 1])
                and (index + 1 == len(times) or times[index] != times[index + 1]))
    if i < len(times) and times[i] == video_time_s:
        stamp = pairs[i][1] if unique(i) else None
    elif not unique(i - 1) or not unique(i):
        stamp = None
    else:
        a, b = pairs[i - 1], pairs[i]
        stamp = a[1] + (video_time_s - a[0]) / (b[0] - a[0]) * (b[1] - a[1])
    return {'source_run_id': part['source_run_id'], 'received_mono': stamp} if stamp is not None else None


def export_existing_sync(session):
    """Extract only timing from old captures; never rewrite images, media or sensor data."""
    from .live_recording import atomic_json, file_hash
    root = Path(session).resolve()
    manifest_path = root / 'session.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if manifest.get('recording', {}).get('finalized') is not True:
        raise ValueError('sync_export_requires_finalized_session')
    if (root / FILENAME).exists():
        raise ValueError('sync_export_already_exists')
    legacy = {}
    source = root / 'frames.jsonl'
    if source.is_file():
        with source.open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                data = row.get('source') or {}
                if row.get('event') == 'input' and data.get('video_recorded') is True:
                    legacy[(data.get('recording_part'), data.get('recording_frame_index'))] = {
                        'run_id': data.get('run_id'), 'timestamp_s': data.get('timestamp_s')}
    clocks_by_part = {}
    for part in manifest['recordings']:
        playback = part.get('playback') or {}
        if not playback.get('path'):
            continue
        times, _ = playback_times(inside(root, playback['path']))
        stored = []
        if part['mode'] == 'received_jpeg':
            with inside(root, part['index_path']).open(encoding='utf-8') as stream:
                stored = [json.loads(line).get('sensor_clock') for line in stream]
        clocks_by_part[part['id']] = [
            (stored[i] if i < len(stored) else None) or legacy.get((part['id'], i))
            for i in range(len(times))]
    value = write_sync(root, manifest['recordings'], clocks_by_part,
        manifest.get('input') == 'tello_camera', provenance='existing_capture_timing')
    manifest['assets']['video_sensor_sync'] = FILENAME
    manifest['asset_sha256'][FILENAME] = file_hash(root / FILENAME)
    manifest['recording']['video_sensor_sync'] = {'path': FILENAME, 'status': value['status']}
    atomic_json(manifest_path, manifest)
    return value
