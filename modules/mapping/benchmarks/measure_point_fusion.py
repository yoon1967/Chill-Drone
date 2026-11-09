"""Compare fusion on identical recorded observations, then stress spatial growth.

No drone commands or model inference. Only numeric results are written; source
RGB/depth/masks remain private. The baseline is read from Git into memory.
"""
import argparse
import json
import platform
from pathlib import Path
import subprocess
import sys
import time
import types

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from modules.mapping.data.rescue_mapping import RescueMap, mapping_options, SemanticSurface


def distribution(values):
    return dict(n=len(values), mean=float(np.mean(values)), p50=float(np.median(values)),
        p95=float(np.percentile(values, 95)), maximum=float(np.max(values)))


def observations(root):
    rows = []
    for path in sorted((root/'history/frames').glob('*.npz')):
        with np.load(path, allow_pickle=False) as frame:
            if 'semantic_labels' in frame:
                rows.append({key: frame[key].copy() for key in frame.files if key != 'rescue_json'})
    if not rows:
        raise ValueError('recorded_semantic_observations_required')
    return rows


def measure(factory, rows):
    instance = factory(mapping_options({'semantic_mapping': True}))
    elapsed, packing, sizes, counts = [], [], [], []
    # Warm imports equally; a new map's first allocation and mesh work are timed.
    row = rows[0]
    instance.update(*arguments(row, 0))
    instance.close()
    instance = factory(mapping_options({'semantic_mapping': True}))
    try:
        for index, row in enumerate(rows):
            start = time.perf_counter()
            result = instance.update(*arguments(row, index))
            elapsed.append((time.perf_counter()-start)*1000)
            start = time.perf_counter()
            encoded = json.dumps(result, separators=(',', ':'), allow_nan=False).encode()
            packing.append((time.perf_counter()-start)*1000)
            sizes.append(len(encoded))
            counts.append(len(result['static_points']))
    finally:
        instance.close()
    return dict(fusion_ms=distribution(elapsed), json_encode_ms=distribution(packing),
        payload_bytes=distribution(sizes), static_points=counts)


def arguments(row, index):
    stamp = float(row['timestamp_s'])
    semantics = dict(frame_id=str(index), timestamp_s=stamp, labels=row['semantic_labels'],
        confidence=row['semantic_confidence'], inference_ms=0., people=[dict(mask=mask,
        confidence=float(confidence)) for mask, confidence in zip(row['person_instances'], row['person_confidences'])])
    return (row['rgb'], row['depth'], row['world_from_camera'], row['intrinsics'], semantics,
        float(row['depth_scale']), int(row['map_id']), str(index), stamp)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--history', type=Path, required=True, help='reconstruction directory')
    parser.add_argument('--baseline-ref', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stress-frames', type=int, default=3600)
    args = parser.parse_args()
    if args.output.exists() or args.stress_frames < 100:
        parser.error('new output path and at least 100 stress frames required')
    commit = subprocess.check_output(['git', 'rev-parse', '--verify', args.baseline_ref+'^{commit}'], cwd=ROOT, text=True).strip()
    source = subprocess.check_output(['git', 'show', commit+':modules/mapping/data/rescue_mapping.py'], cwd=ROOT, text=True, encoding='utf-8')
    baseline = types.ModuleType('recorded_fusion_baseline')
    baseline.__package__ = 'modules.mapping.data'
    exec(compile(source, '<git-fusion-baseline>', 'exec'), baseline.__dict__)
    rows = observations(args.history)
    report = dict(schema_version='point-fusion-profile/1.0', baseline_commit=commit,
        scope='identical stored RGB/depth/semantic masks/fixed poses; excludes decode, SLAM, inference, browser and disk',
        environment=dict(python=platform.python_version(), platform=platform.platform(), numpy=np.__version__),
        input_shape=list(rows[0]['depth'].shape), point_budget=8000,
        recorded_observations=len(rows), baseline=measure(baseline.RescueMap, rows),
        points=measure(RescueMap, rows))
    row = rows[0]
    fusion = SemanticSurface(max(1e-5, .04*float(row['depth_scale'])), max_cells=8000)
    times, counts = [], []
    pose = row['world_from_camera'].copy()
    step = fusion.base_size*5
    for index in range(args.stress_frames):
        pose[0,3] = row['world_from_camera'][0,3]+index*step
        start = time.perf_counter()
        fusion.integrate(row['rgb'], row['depth'], pose, row['intrinsics'], row['semantic_labels'], row['semantic_confidence'])
        fusion.points()
        times.append((time.perf_counter()-start)*1000)
        counts.append(len(fusion.cells))
        if len(fusion.cells) > 8000:
            raise AssertionError('point_budget_exceeded')
    if np.any(fusion.xyz.min(axis=0)-fusion.bounds[0] > fusion.size*1.01) or np.any(fusion.bounds[1]-fusion.xyz.max(axis=0) > fusion.size*1.01):
        raise AssertionError('observation_coverage_lost')
    report['stress'] = dict(scope='recorded first RGB-D observation translated along synthetic fixed poses; no real long-flight claim',
        frames=args.stress_frames, equivalent_seconds_at_5hz=args.stress_frames/5,
        maximum_points=max(counts), final_points=counts[-1], first_100_ms=distribution(times[:100]),
        last_100_ms=distribution(times[-100:]), fusion=fusion.metadata(),
        representative_point_bounds=[fusion.xyz.min(axis=0).tolist(), fusion.xyz.max(axis=0).tolist()])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
