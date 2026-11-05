"""Build final maps from all observations; new point maps retain a spatial budget."""
import json
import numpy as np
from .mapping_limits import DEFAULT_POINT_BUDGET, MAX_POINT_BUDGET

from modules.mapping.data.mapping_history import HistoryReader


def integrate_history(output, progress=lambda **fields: None):
    from modules.mapping.data.stream_mapping import add_points
    from modules.mapping.data.rescue_mapping import SemanticSurface, PersonSightings, SCHEMA
    reader = HistoryReader(output)
    entries = list(reader.refresh())
    groups = {}
    total = sum(bool(entry['observation']) for entry in entries)
    done = 0
    for entry in entries:
        if not entry['observation']:
            continue
        path = reader.root / 'frames' / f"{entry['revision']:08d}.npz"
        with np.load(path, allow_pickle=False) as frame:
            map_id = int(frame['map_id'])
            group = groups.setdefault(map_id, {'map_id': map_id, 'cells': {}, 'cameras': [],
                'voxel_size': max(.0001, .06 * float(frame['depth_scale']))})
            depth = frame['depth']
            recorded = json.loads(frame['rescue_json'].tobytes()) if 'rescue_json' in frame else {}
            if 'semantic_labels' in frame or recorded.get('render_mode') == 'points':
                fusion = recorded.get('point_fusion', {})
                budget = fusion.get('point_budget', DEFAULT_POINT_BUDGET)
                if type(budget) is not int or not 1000 <= budget <= MAX_POINT_BUDGET:
                    raise ValueError('invalid_point_budget')
                if 'surface' not in group:
                    size = max(1e-5, .04 * float(frame['depth_scale']))
                    increment = fusion.get('spacing_increment') if fusion.get('static_points') else None
                    if type(increment) not in (int, float) or not size / 10 <= increment <= size:
                        increment = None
                    group.update(surface=SemanticSurface(size, max_cells=budget, spacing_increment=increment),
                        sightings=PersonSightings(group['voxel_size']))
                labels = frame['semantic_labels'] if 'semantic_labels' in frame else np.zeros(depth.shape, np.uint8)
                confidence = frame['semantic_confidence'] if 'semantic_confidence' in frame else np.ones(depth.shape, np.float32)
                group['surface'].integrate(frame['rgb'], depth, frame['world_from_camera'],
                    frame['intrinsics'], labels, confidence)
                detections = [{'mask': mask, 'confidence': float(confidence)} for mask, confidence in
                    zip(frame['person_instances'], frame['person_confidences'])] if 'person_instances' in frame else []
                people = group['sightings'].update(detections, depth, frame['world_from_camera'],
                    frame['intrinsics'], entry['timestamp_s'], entry['frame_id'])
                group['rescue'] = dict(schema_version=SCHEMA, map_id=map_id, people=people,
                    render_mode='points', point_revision=group['surface'].revision,
                    point_labels=group['surface'].labels(),
                    point_fusion=group['surface'].metadata(),
                    person_color=recorded.get('person_color', '#ff3030'),
                    person_outline=recorded.get('person_outline', True))
            else:
                # Existing non-semantic archives keep their original reconstruction.
                add_points(group['cells'], frame['rgb'], depth, frame['world_from_camera'],
                    frame['intrinsics'], max_points=float('inf'), voxel_size=group['voxel_size'])
            pose = frame['world_from_camera']
            group['cameras'].append({'image_id': entry['frame_number'], 'frame_id': entry['frame_id'],
                'timestamp_s': entry['timestamp_s'], 'source_pts_s': entry['timestamp_s'],
                'position': pose[:3, 3].tolist(), 'world_from_camera_rotation': pose[:3, :3].tolist(),
                'model': 'PINHOLE', 'width': frame['rgb'].shape[1], 'height': frame['rgb'].shape[0],
                'params': frame['intrinsics'].tolist()})
        done += 1
        progress(integrated_frames=done, integration_total=total, integrated_maps=len(groups),
            integrated_points=sum(len(g['surface'].cells) if 'surface' in g else len(g['cells']) for g in groups.values()))
    if not groups:
        raise ValueError('no_recorded_depth_observations')
    for group in groups.values():
        if 'surface' in group:
            group['cells'] = dict(enumerate(group['surface'].points()))
            group['surface'].close()
            del group['surface'], group['sightings']
    return groups, entries
