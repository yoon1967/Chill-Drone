"""Build final maps from every recorded observation, independent of preview limits."""
import numpy as np

from modules.mapping.data.mapping_history import HistoryReader


def integrate_history(output, progress=lambda **fields: None):
    from modules.mapping.data.stream_mapping import add_points
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
            # No age, point-count or last-segment eviction in the final map.
            add_points(group['cells'], frame['rgb'], frame['depth'], frame['world_from_camera'],
                       frame['intrinsics'], max_points=float('inf'), voxel_size=group['voxel_size'])
            pose = frame['world_from_camera']
            group['cameras'].append({'image_id': entry['frame_number'], 'frame_id': entry['frame_id'],
                'timestamp_s': entry['timestamp_s'], 'source_pts_s': entry['timestamp_s'],
                'position': pose[:3, 3].tolist(), 'world_from_camera_rotation': pose[:3, :3].tolist(),
                'model': 'PINHOLE', 'width': frame['rgb'].shape[1], 'height': frame['rgb'].shape[0],
                'params': frame['intrinsics'].tolist()})
        done += 1
        progress(integrated_frames=done, integration_total=total, integrated_maps=len(groups),
                 integrated_points=sum(len(g['cells']) for g in groups.values()))
    if not groups:
        raise ValueError('no_recorded_depth_observations')
    return groups, entries
