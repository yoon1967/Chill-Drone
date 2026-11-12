"""Run the local Roomform boundary checkpoint on a saved RGB point cloud."""
from __future__ import annotations

import base64
from collections import defaultdict
from itertools import groupby
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from .roomform_errors import RoomformError


SURFACES = ('wall', 'floor', 'ceiling')


def _vector(value):
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        return None
    if any(not isinstance(axis, (int, float)) or not np.isfinite(axis) for axis in value):
        return None
    return np.asarray(value, dtype=np.float64)


def _cloud(scene):
    if scene.get('schema_version') not in {'phone-sfm-result/1.0', 'phone-sfm-result/1.1', 'phone-sfm-result/1.2'}:
        raise RoomformError('roomform_invalid_scene')
    points, colors = [], []
    for row in scene.get('points', []):
        if not isinstance(row, list) or (xyz := _vector(row[1:4])) is None:
            continue
        points.append(xyz)
        colors.append([min(255, max(0, round(value))) if isinstance(value, (int, float)) and np.isfinite(value)
                       else 128 for value in (row[4:7] + [128, 128, 128])[:3]])
    if len(points) < 16:
        raise RoomformError('roomform_too_few_points')
    return np.asarray(points), np.asarray(colors, dtype=np.uint8)


def _up_and_scale(points, scene):
    from scipy.ndimage import gaussian_filter1d
    from scipy.signal import find_peaks
    from scipy.spatial import cKDTree

    declared = _vector(scene.get('up'))
    if declared is not None and np.linalg.norm(declared) > 0:
        up = declared / np.linalg.norm(declared)
        explicit = True
    else:
        up = np.array([0., 1., 0.])
        explicit = False
    metric = (scene.get('scale') or {}).get('unit') == 'm'
    if metric and explicit:
        return up, 1.

    # Estimate a floor normal near the saved display up, then use distinct
    # horizontal levels bracketing the capture path to infer monocular scale.
    sample = points[np.linspace(0, len(points) - 1, min(len(points), 24000), dtype=int)]
    neighbors = cKDTree(points).query(sample, k=min(20, len(points)), workers=2)[1]
    local = points[neighbors]
    local -= local.mean(axis=1, keepdims=True)
    values, vectors = np.linalg.eigh(np.einsum('nki,nkj->nij', local, local))
    normals = vectors[:, :, 0]
    normals[normals @ up < 0] *= -1
    planar = (values[:, 0] < .13 * values[:, 1]) & (values[:, 1] > .09 * values[:, 2])
    near = planar & (normals @ up > .78)
    if not explicit and near.sum() >= 24:
        directions = normals[near]
        bins = np.rint(directions * 24).astype(int)
        modes, counts = np.unique(bins, axis=0, return_counts=True)
        cluster = directions[np.max(np.abs(bins - modes[counts.argmax()]), axis=1) <= 1]
        if len(cluster) >= 12:
            up = np.median(cluster, axis=0)
            up /= np.linalg.norm(up)
    if metric:
        return up, 1.

    heights = sample[planar & (normals @ up > .975)] @ up
    span = np.ptp(np.quantile(points @ up, [.1, .9]))
    if len(heights) >= 24:
        low, high = np.quantile(heights, [.01, .99])
        if high > low:
            histogram, edges = np.histogram(heights, range=(low, high), bins=160)
            smooth = gaussian_filter1d(np.pad(histogram.astype(float), 1), 1.)
            peaks = find_peaks(smooth, prominence=smooth.max() * .08, distance=6)[0] - 1
            levels = (edges[peaks] + edges[peaks + 1]) / 2
            cameras = [pos for camera in scene.get('cameras', []) if isinstance(camera, dict)
                       and (pos := _vector(camera.get('position'))) is not None]
            pairs = [(a, b) for a in range(len(peaks)) for b in range(a + 1, len(peaks))
                     if levels[b] - levels[a] > .15 * span]
            if cameras:
                camera_height = np.median(np.asarray(cameras) @ up)
                bracketed = [(a, b) for a, b in pairs if levels[a] < camera_height < levels[b]]
                pairs = bracketed or pairs
            if pairs:
                a, b = max(pairs, key=lambda pair: smooth[peaks[pair[0]] + 1] * smooth[peaks[pair[1]] + 1])
                span = levels[b] - levels[a]
    if span <= 0:
        raise RoomformError('roomform_no_height')
    return up, 2.6 / span


def _rectangles(cells):
    """Join runs of identical width on neighboring grid rows."""
    rows = defaultdict(list)
    for x, y in cells:
        rows[y].append(x)
    active = {}
    for y, xs in sorted(rows.items()):
        spans = set()
        for _, group in groupby(enumerate(sorted(xs)), lambda item: item[1] - item[0]):
            run = [x for _, x in group]
            spans.add((run[0], run[-1] + 1))
        for span, (start, end) in list(active.items()):
            if end != y or span not in spans:
                yield (*span, start, end)
                del active[span]
        for span in spans:
            active[span] = (active.get(span, (y, y))[0], y + 1)
    for span, (start, end) in active.items():
        yield (*span, start, end)


def _join_quads(quads, voxel):
    if len(quads) < 2:
        return quads
    origin = quads[0, 0]
    axes = (quads[0, [1, 3]] - origin) / voxel
    cells = np.rint((quads[:, 0] - origin) @ axes.T / voxel).astype(int)
    if not np.allclose(quads[:, 0], origin + voxel * cells @ axes, atol=1e-5, rtol=0):
        return quads
    corners = np.asarray([[(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
                          for x0, x1, y0, y1 in _rectangles(set(map(tuple, cells)))])
    return origin + voxel * corners @ axes


def _sheets(nodes, voxel, offsets, openings):
    from roomform import planes

    mask = nodes > .5
    openings = np.argwhere(openings.max(0) > .69) if openings is not None else None
    # Use the pinned exporter's individual sheets. Keeping them separate lets
    # us discard empty sheets and merge neighboring coplanar render quads.
    yield 'wall', planes._wall_sheets(mask[0], offsets, openings, voxel, planes.CLASS_COLORS['wall'])
    for index, kind in enumerate(SURFACES[1:], start=1):
        yield kind, planes._horizontal_sheets(mask[index], offsets, voxel, planes.CLASS_COLORS[kind])


def reconstruct_roomform(scene, checkpoint, progress):
    import torch
    from roomform.inference.local import build_input, load_checkpoint
    from roomform.pipe.evidence.build import build_evidence

    points, colors = _cloud(scene)
    up, scale = _up_and_scale(points, scene)
    origin = np.median(points, axis=0)
    x = np.eye(3)[0 if abs(up[0]) < .9 else 1]
    x -= np.dot(x, up) * up
    x /= np.linalg.norm(x)
    basis = np.column_stack((x, np.cross(up, x), up))
    xyz = ((points - origin) @ basis * scale).astype(np.float32)

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    step, context = (7.2 if device == 'cuda' else 4.8), .64
    base = (np.floor(np.quantile(xyz, .01, axis=0) / step) + .5) * step
    keys = np.floor((xyz - base) / step).astype(np.int64)
    cells, counts = np.unique(keys, axis=0, return_counts=True)
    tiles = cells[counts >= 16]
    if not len(tiles):
        raise RoomformError('roomform_too_few_points')
    progress(0, len(tiles))
    model, cfg = load_checkpoint(str(checkpoint), device)
    by_kind = defaultdict(list)
    with TemporaryDirectory(prefix='mapping-roomform-') as folder:
        folder = Path(folder)
        for index, cell in enumerate(tiles):
            low, high = base + cell * step, base + (cell + 1) * step
            keep = np.all((xyz >= low - context) & (xyz < high + context), axis=1)
            scan = folder / 'tile.npz'
            np.savez(scan, pts=xyz[keep], pts_color=colors[keep])
            evidence = build_evidence(str(scan), str(folder / 'evidence.npz'), cfg.vox_m)
            features = torch.from_numpy(build_input(evidence, cfg))[None].to(device)
            with torch.inference_mode(), torch.autocast(device, dtype=torch.float16, enabled=device == 'cuda'):
                outputs = model(features)
                nodes = outputs[0][0].sigmoid().float().cpu().numpy()
                offsets = outputs[2][0].float().cpu().numpy() if cfg.predict_offsets else None
                openings = outputs[3 if cfg.predict_offsets else 2][0].sigmoid().float().cpu().numpy() if cfg.predict_openings else None
            del features, outputs
            for kind, sheets in _sheets(nodes, cfg.vox_m, offsets, openings):
                for sheet in sheets:
                    if not len(sheet.faces):
                        continue
                    faces = np.asarray(sheet.faces).reshape(-1, 2, 3)
                    indices = np.column_stack((faces[:, 0], faces[:, 1, 2]))
                    quads = np.asarray(sheet.vertices)[indices] + evidence.origin
                    centers = (quads[:, 0] + quads[:, 2]) * .5
                    quads = quads[np.all((centers >= low) & (centers < high), axis=1)]
                    if len(quads):
                        by_kind[kind].append(_join_quads(quads, cfg.vox_m))
            progress(index + 1, len(tiles))
    if not by_kind:
        raise RoomformError('roomform_no_surfaces')
    surfaces = {}
    for kind, parts in by_kind.items():
        quads = np.concatenate(parts) @ basis.T / scale
        surfaces[kind] = {'quads': len(quads),
                          'positions': base64.b64encode(quads.astype('<f4').tobytes()).decode('ascii')}
    return {'schema_version': 'roomform-surface/1.0', 'origin': origin.tolist(), 'up': up.tolist(),
            'scale_estimated': (scene.get('scale') or {}).get('unit') != 'm',
            'device': device, 'surfaces': surfaces}
