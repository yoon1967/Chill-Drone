"""Recover an empty sensor sidecar only from the exact gateway run and frame clock."""
import hashlib
import json
import math
from pathlib import Path
import re
from datetime import datetime, timezone

from .live_recording import atomic_json, file_hash


def recover_empty_sensor_log(session, gateway_root):
    session, gateway_root = Path(session).resolve(), Path(gateway_root).resolve()
    target, manifest_path = session / 'sensor.jsonl', session / 'session.json'
    if not target.is_file() or target.stat().st_size:
        raise ValueError('recovery_requires_existing_empty_sensor_log')
    original = manifest_path.read_bytes()
    manifest = json.loads(original)
    if manifest.get('recording', {}).get('finalized') is not True:
        raise ValueError('recovery_requires_finalized_session')
    sources = []
    with (session / 'frames.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row.get('event') == 'input':
                source = row.get('source') or {}
                stamp = source.get('timestamp_s')
                if not isinstance(source.get('run_id'), str) or type(stamp) not in (int, float) or not math.isfinite(stamp):
                    raise ValueError('recovery_requires_source_frame_identity_and_clock')
                sources.append(source)
    runs = {source['run_id'] for source in sources}
    if len(runs) != 1:
        raise ValueError('recovery_requires_one_gateway_run')
    run_id = next(iter(runs))
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', run_id):
        raise ValueError('invalid_gateway_run_id')
    run = (gateway_root / run_id).resolve()
    if run.parent != gateway_root:
        raise ValueError('invalid_gateway_run_path')
    if json.loads((run / 'manifest.json').read_text(encoding='utf-8')).get('run_id') != run_id:
        raise ValueError('gateway_manifest_identity_mismatch')
    source_path = run / 'sensor.jsonl'
    begin, end = min(s['timestamp_s'] for s in sources), max(s['timestamp_s'] for s in sources)
    samples = []
    with source_path.open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            stamp = row.get('received_mono')
            if type(stamp) in (int, float) and math.isfinite(stamp) and begin - .2 <= stamp <= end + .2:
                if type(row.get('sequence')) is not int or not isinstance(row.get('raw'), dict):
                    raise ValueError('gateway_sensor_identity_missing')
                samples.append(row)
    if not samples or samples[0]['received_mono'] > begin + .2 or samples[-1]['received_mono'] < end - .2:
        raise ValueError('gateway_sensors_do_not_cover_captured_frames')
    if any(b['sequence'] != a['sequence'] + 1 or b['received_mono'] < a['received_mono']
           for a, b in zip(samples, samples[1:])):
        raise ValueError('gateway_sensor_log_is_discontinuous')
    recovered = ''.join(json.dumps({'schema_version': 'mapping-sensor/1.0', 'event': 'sample',
        'source_run_id': run_id, **row}, ensure_ascii=True, allow_nan=False, separators=(',', ':')) + '\n'
        for row in samples).encode('utf-8')
    provenance = {'method': 'matching_gateway_raw_log', 'source_run_id': run_id,
        'source_sha256': file_hash(source_path), 'frame_timestamp_range_s': [begin, end],
        'sensor_timestamp_range_s': [samples[0]['received_mono'], samples[-1]['received_mono']],
        'samples': len(samples), 'recovered_at': datetime.now(timezone.utc).isoformat()}
    if target.stat().st_size or manifest_path.read_bytes() != original:
        raise ValueError('session_changed_during_recovery')
    temporary = target.with_suffix('.jsonl.tmp')
    try:
        temporary.write_bytes(recovered)
        temporary.replace(target)
    finally:
        temporary.unlink(missing_ok=True)
    manifest['asset_sha256']['sensor.jsonl'] = hashlib.sha256(recovered).hexdigest()
    manifest['recording'].update(sensor_required=True, sensor_samples=len(samples))
    manifest['recording']['log_records']['sensor'] = len(samples)
    manifest.setdefault('capture_metadata', {})['sensor_recovery'] = provenance
    atomic_json(manifest_path, manifest)
    return {'session_id': session.name, 'bytes': len(recovered), **provenance}
