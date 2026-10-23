"""Image-bounded planar objects supported by native SfM observations and multiple views."""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass, replace
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np


def scene_binding(scene):
    return hashlib.sha256(json.dumps({k: scene[k] for k in ("points", "cameras", "scale")},
        sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class DetectionSettings:
    plane_error_px: float = 4.
    min_support: int = 5
    min_border: int = 3
    min_coverage: float = .03
    min_area_ratio: float = .0015
    min_side_px: float = 10.
    edge_agreement: float = .68
    min_edge_agreement: float = .40
    min_views: int = 3
    max_normal_angle_deg: float = 15.
    appearance_correlation: float = .55
    max_image_aspect: float = 30.


def detection_settings(profile="balanced", plane_error_px=None):
    settings = {
        "balanced": DetectionSettings(),
        "coverage": DetectionSettings(6., 3, 2, .015, .0004, 5., .62, .35, 3, 20., .45, 40.),
        "strict": DetectionSettings(2.5, 8, 5, .12, .004, 20., .75, .50, 3, 10., .65, 16.),
    }[profile]
    if plane_error_px is not None:
        if not np.isfinite(plane_error_px) or not 1 <= plane_error_px <= 12:
            raise ValueError("plane_error_px_must_be_between_1_and_12")
        settings = replace(settings, plane_error_px=float(plane_error_px))
    return settings


def fit_planes(xyz, threshold, minimum=24, maximum=16, trials=280):
    """Deterministic RANSAC followed by SVD; reject line-like support."""
    rng = np.random.default_rng(42)
    remaining = np.arange(len(xyz))
    planes = []
    for _ in range(maximum):
        if len(remaining) < minimum:
            break
        p = xyz[remaining]
        best = np.empty(0, dtype=int)
        for _ in range(trials):
            a, b, c = p[rng.choice(len(p), 3, replace=False)]
            n = np.cross(b - a, c - a)
            length = np.linalg.norm(n)
            if length < 1e-10:
                continue
            n /= length
            selected = np.flatnonzero(np.abs((p - a) @ n) < threshold)
            if len(selected) > len(best):
                best = selected
        if len(best) < minimum:
            break
        for _ in range(3):
            if len(best) < minimum:
                break
            center = p[best].mean(axis=0)
            _, singular, vh = np.linalg.svd(p[best] - center, full_matrices=False)
            normal = vh[-1]
            best = np.flatnonzero(np.abs((p - center) @ normal) < threshold)
        if len(best) < minimum or singular[1] < singular[0] * .025:
            break
        planes.append((normal, -float(normal @ center)))
        remaining = np.delete(remaining, best)
    return planes


def ordered_quad(points):
    p = np.asarray(points, dtype=np.float32).reshape(4, 2)
    p = p[np.argsort(np.arctan2(p[:, 1] - p[:, 1].mean(), p[:, 0] - p[:, 0].mean()))]
    p = np.roll(p, -int(np.argmin(p.sum(axis=1))), axis=0)
    return p


def quad_distances(quad, points):
    """Signed distances to convex supporting edges, vectorized over native observations."""
    edges = np.roll(quad, -1, axis=0) - quad
    delta = np.asarray(points)[:, None] - quad
    sign = 1 if cv2.contourArea(quad, oriented=True) > 0 else -1
    return (sign * (edges[None, :, 0] * delta[:, :, 1] - edges[None, :, 1] * delta[:, :, 0]) /
            np.linalg.norm(edges, axis=1)).min(axis=1)


def edge_agreements(quad, distance, tolerance=3.):
    t = np.linspace(.06, .94, 16)[:, None]
    samples = np.stack([a * (1 - t) + b * t for a, b in zip(quad, np.roll(quad, -1, axis=0))])
    h, w = distance.shape
    if np.any(samples < 0) or np.any(samples[:, :, 0] >= w - 1) or np.any(samples[:, :, 1] >= h - 1):
        return np.zeros(4)
    pixels = np.rint(samples).astype(int)
    return (distance[pixels[:, :, 1], pixels[:, :, 0]] < tolerance).mean(axis=1)


def line_quads(gray, distance, settings):
    """Finite line arrangements cover doors, small compartments, and thin horizontal patches."""
    h, w = gray.shape
    detected = cv2.createLineSegmentDetector().detect(gray)[0]
    if detected is None:
        return []
    groups = [[], []]
    for segment in sorted(detected.reshape(-1, 2, 2), key=lambda s: -np.linalg.norm(s[1] - s[0]))[:320]:
        delta = segment[1] - segment[0]
        length = np.linalg.norm(delta)
        vertical = abs(delta[1]) > abs(delta[0]) * 2.5 and length > settings.min_side_px
        horizontal = abs(delta[0]) > abs(delta[1]) * 2 and length > settings.min_side_px
        if not (vertical or horizontal):
            continue
        group = groups[0 if vertical else 1]
        equation = np.cross([*segment[0], 1], [*segment[1], 1])
        equation /= np.linalg.norm(equation[:2])
        merged = False
        for old_eq, old in group:
            direction = (old[1] - old[0]) / np.linalg.norm(old[1] - old[0])
            projections = (segment - old[0]) @ direction
            if abs(equation[:2] @ old_eq[:2]) > .995 and \
                    np.max(np.abs(segment @ old_eq[:2] + old_eq[2])) < 2.5 and \
                    projections.min() < np.linalg.norm(old[1] - old[0]) + w * .08 and projections.max() > -w * .08:
                extent = [min(0., projections.min()), max(np.linalg.norm(old[1] - old[0]), projections.max())]
                old[:] = old[0] + np.asarray(extent)[:, None] * direction
                merged = True
                break
        if merged:
            continue
        group.append((equation, segment))
    verticals, horizontals = groups[0][:36], groups[1][:48]
    if not verticals or not horizontals:
        return []
    crossings3 = np.cross(np.array([v[0] for v in verticals])[:, None], np.array([v[0] for v in horizontals])[None])
    valid_crossings = np.abs(crossings3[:, :, 2]) > 1e-5
    intersections = crossings3[:, :, :2] / np.where(valid_crossings, crossings3[:, :, 2], 1)[:, :, None]
    proposals = []
    for i, (left, ls) in enumerate(verticals):
        for ri in range(i + 1, len(verticals)):
            right, rs = verticals[ri]
            middle_separation = abs((rs.mean(axis=0) @ left[:2]) + left[2])
            if not settings.min_side_px < middle_separation < w * .65:
                continue
            crossings = []
            for hi, (_, hs) in enumerate(horizontals):
                if not valid_crossings[i, hi] or not valid_crossings[ri, hi]:
                    continue
                a, b = intersections[i, hi], intersections[ri, hi]
                if min(a[0], b[0]) < hs[:, 0].min() - 6 or max(a[0], b[0]) > hs[:, 0].max() + 6:
                    continue
                if min(a[1], b[1]) < max(ls[:, 1].min(), rs[:, 1].min()) - h * .08 or \
                        max(a[1], b[1]) > min(ls[:, 1].max(), rs[:, 1].max()) + h * .08:
                    continue
                crossings.append((a, b))
            for ti, (a, b) in enumerate(crossings):
                for c, d in crossings[ti + 1:]:
                    q = ordered_quad([a, b, c, d])
                    if not cv2.isContourConvex(q):
                        continue
                    width = (np.linalg.norm(q[1] - q[0]) + np.linalg.norm(q[2] - q[3])) / 2
                    height = (np.linalg.norm(q[3] - q[0]) + np.linalg.norm(q[2] - q[1])) / 2
                    if min(width, height) < settings.min_side_px or max(width, height) / min(width, height) > settings.max_image_aspect or \
                            abs(cv2.contourArea(q)) < w * h * settings.min_area_ratio:
                        continue
                    if np.any(q < 2) or np.any(q[:, 0] > w - 3) or np.any(q[:, 1] > h - 3):
                        continue
                    agreement = edge_agreements(q, distance)
                    if agreement.mean() < settings.edge_agreement or agreement.min() < settings.min_edge_agreement:
                        continue
                    # Full-width/height seams split compartments; shelf edges inside a window do not span the outer frame.
                    x0, x1 = q[:, 0].min(), q[:, 0].max()
                    y0, y1 = q[:, 1].min(), q[:, 1].max()
                    if any(x0 + width * .12 < s[:, 0].mean() < x1 - width * .12 and
                           min(s[:, 1].max(), y1) - max(s[:, 1].min(), y0) > height * .90
                           for _, s in verticals):
                        continue
                    if not any(cv2.intersectConvexConvex(q, old)[0] / max(abs(cv2.contourArea(q)), 1) > .92 for old in proposals):
                        proposals.append(q)
    return proposals


def image_quads(image, settings):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    edges = cv2.Canny(gray, 35, 100)
    h, w = gray.shape
    quads = []
    for binary in (edges, cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))):
        contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            poly = cv2.approxPolyDP(contour, .018 * cv2.arcLength(contour, True), True)
            if len(poly) != 4 or not cv2.isContourConvex(poly):
                continue
            area = abs(cv2.contourArea(poly))
            if not w * h * settings.min_area_ratio < area < w * h * .6:
                continue
            q = ordered_quad(poly)
            lengths = np.linalg.norm(q - np.roll(q, -1, axis=0), axis=1)
            if lengths.min() < settings.min_side_px or lengths.max() / lengths.min() > settings.max_image_aspect:
                continue
            if min(q[:, 0].min(), q[:, 1].min(), w - 1 - q[:, 0].max(), h - 1 - q[:, 1].max()) < 2:
                continue
            if any(cv2.intersectConvexConvex(q, old)[0] / min(area, abs(cv2.contourArea(old))) > .95
                   and abs(area - abs(cv2.contourArea(old))) / area < .15 for old in quads):
                continue
            quads.append(q)
    distance = cv2.distanceTransform(255 - edges, cv2.DIST_L2, 3)
    quads += line_quads(gray, distance, settings)
    quads = [cell for q in quads for cell in split_at_seams(q, edges, settings)]
    # Separate budgets prevent large door proposals from evicting thin or compact surfaces.
    groups = [[], [], []]
    for q in sorted(quads, key=lambda q: -abs(cv2.contourArea(q))):
        sides = np.linalg.norm(q - np.roll(q, -1, axis=0), axis=1)
        aspect = (sides[1] + sides[3]) / max(sides[0] + sides[2], 1e-6)
        group = groups[0 if aspect > 1.6 else 1 if aspect < .625 else 2]
        area = abs(cv2.contourArea(q))
        if len(group) < 55 and not any(cv2.intersectConvexConvex(q, old)[0] /
                max(area + abs(cv2.contourArea(old)) - cv2.intersectConvexConvex(q, old)[0], 1) > .85 for old in group):
            group.append(q)
    return sum(groups, []), distance


def split_at_seams(quad, edges, settings):
    """Partition a visible rectangle only where an observed seam spans the rectified patch."""
    sides = np.linalg.norm(quad - np.roll(quad, -1, axis=0), axis=1)
    width = max(4, round((sides[0] + sides[2]) / 2))
    height = max(4, round((sides[1] + sides[3]) / 2))
    target = np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]], np.float32)
    transform = cv2.getPerspectiveTransform(quad, target)
    patch = cv2.warpPerspective(edges, transform, (width, height), flags=cv2.INTER_NEAREST)
    detected = cv2.createLineSegmentDetector().detect(patch)[0]
    spans = [[], []]
    if detected is not None:
        for segment in detected.reshape(-1, 2, 2):
            delta = np.abs(segment[1] - segment[0])
            if delta[1] > height * .65 and delta[0] < height * .06:
                spans[0].append(float(segment[:, 0].mean()))
            if delta[0] > width * .65 and delta[1] < width * .06:
                spans[1].append(float(segment[:, 1].mean()))
    patch = cv2.dilate(patch, np.ones((3, 3), np.uint8))
    cuts = []
    for axis, (length, strength) in enumerate(((width, (patch[2:-2] > 0).mean(axis=0)),
                             (height, (patch[:, 2:-2] > 0).mean(axis=1)))):
        indices = np.flatnonzero((strength > .78) & (np.arange(length) > max(4, length * .08)) &
                                (np.arange(length) < length - max(4, length * .08)))
        indices = np.unique(np.r_[indices, [round(p) for p in spans[axis] if max(4, length * .08) < p < length - max(4, length * .08)]].astype(int))
        groups = np.split(indices, np.flatnonzero(np.diff(indices) > 4) + 1)
        positions = [float(np.median(group)) for group in groups if len(group)]
        cuts.append([0., *positions[:8], float(length - 1)])
    if len(cuts[0]) == len(cuts[1]) == 2:
        return [quad]
    inverse = np.linalg.inv(transform)
    cells = []
    for x0, x1 in zip(cuts[0], cuts[0][1:]):
        for y0, y1 in zip(cuts[1], cuts[1][1:]):
            if min(x1 - x0, y1 - y0) < settings.min_side_px:
                continue
            q = cv2.perspectiveTransform(np.array([[[x0, y0], [x1, y0], [x1, y1], [x0, y1]]], np.float32), inverse)[0]
            if abs(cv2.contourArea(q)) >= edges.size * settings.min_area_ratio:
                cells.append(q)
    return cells


def prepare_view(model, camera, images_root, points_by_id, max_width=960, settings=None, proposal=None):
    im = model.images[camera["image_id"]]
    native_camera = model.cameras[im.camera_id]
    image = cv2.imread(str(Path(images_root) / camera["frame_id"]))
    if image is None:
        return None
    # Preserve the COLMAP pinhole projection in the undistorted analysis image.
    name, params = native_camera.model_name, native_camera.params
    distortion = np.zeros(5)
    if name in ("SIMPLE_RADIAL", "RADIAL"):
        distortion[:len(params) - 3] = params[3:]
    elif name == "OPENCV":
        distortion[:4] = params[4:8]
    elif name not in ("PINHOLE", "SIMPLE_PINHOLE"):
        return None
    k = native_camera.calibration_matrix()
    image = cv2.undistort(image, k, distortion)
    ratio = min(1., max_width / image.shape[1])
    image = cv2.resize(image, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_AREA)
    k = k.copy()
    k[:2] *= ratio
    observations = [p for p in im.points2D if p.has_point3D() and p.point3D_id in points_by_id]
    if len(observations) < 12:
        return None
    ids = np.array([p.point3D_id for p in observations], dtype=np.int64)
    xy = np.array([p.xy for p in observations])
    xy = native_camera.cam_from_img(xy)
    xy = xy * np.array([k[0, 0], k[1, 1]]) + k[:2, 2]
    if proposal is None:
        quads, distance = image_quads(image, settings or detection_settings())
    else:
        # Proposals were detected once on receipt, before camera calibration was available.
        # Transform their raw pixels into this revision's undistorted analysis image.
        quads = [cv2.undistortPoints(np.array(q, np.float32).reshape(-1, 1, 2),
            native_camera.calibration_matrix(), distortion, P=k).reshape(4, 2) for q in proposal]
        edges = cv2.Canny(cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (3, 3), 0), 35, 100)
        distance = cv2.distanceTransform(255 - edges, cv2.DIST_L2, 3)
    return {"camera": camera, "image": image, "gray": cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), "K": k, "R": np.array(camera["world_from_camera_rotation"]),
            "origin": np.array(camera["position"]), "ids": ids, "xy": xy,
            "xyz": np.array([points_by_id[int(pid)] for pid in ids]), "quads": quads, "distance": distance}


def project(points, view):
    local = (np.asarray(points) - view["origin"]) @ view["R"]
    uv = local[:, :2] / np.maximum(local[:, 2:], 1e-10)
    uv = uv * np.array([view["K"][0, 0], view["K"][1, 1]]) + view["K"][:2, 2]
    return uv, local[:, 2]


def intersect_plane(quad, normal, offset, view):
    rays = np.column_stack(((quad - view["K"][:2, 2]) /
        np.array([view["K"][0, 0], view["K"][1, 1]]), np.ones(4))) @ view["R"].T
    divisor = rays @ normal
    if np.min(np.abs(divisor)) < .08:
        return None
    depth = -(view["origin"] @ normal + offset) / divisor
    if np.any(depth <= 0):
        return None
    corners = view["origin"] + rays * depth[:, None]
    sides = np.roll(corners, -1, axis=0) - corners
    lengths = np.linalg.norm(sides, axis=1)
    if lengths.min() < 1e-5 or lengths.max() / lengths.min() > 12:
        return None
    cosine = np.abs(np.sum(sides * np.roll(sides, -1, axis=0), axis=1) / (lengths * np.roll(lengths, -1)))
    if cosine.max() > .38 or max(lengths[0], lengths[2]) / min(lengths[0], lengths[2]) > 1.4 or \
            max(lengths[1], lengths[3]) / min(lengths[1], lengths[3]) > 1.4:
        return None
    return corners, lengths


def quad_normal(quad, view):
    """A calibrated rectangle's two vanishing directions constrain its plane normal."""
    h = cv2.getPerspectiveTransform(np.array([[0, 0], [1, 0], [1, 1], [0, 1]], np.float32), quad)
    directions = np.linalg.solve(view["K"], h[:, :2])
    normal = np.cross(directions[:, 0], directions[:, 1])
    length = np.linalg.norm(normal)
    return view["R"] @ normal / length if length > 1e-10 else None


def consensus_quads(planes, xyz, views, threshold, settings):
    """Vote for finite image seams on an observed plane; parallax weakens off-plane edges."""
    chosen = []
    proposals = []
    for pi, (normal, offset) in enumerate(planes):
        if len(chosen) >= 3:
            break
        if any(abs(normal @ n) > .99 and abs(offset - np.sign(normal @ n) * d) < threshold * 6 for n, d in chosen):
            continue
        support = xyz[np.abs(xyz @ normal + offset) < threshold]
        if len(support) < 35:
            continue
        chosen.append((normal, offset))
        origin = support.mean(axis=0)
        origin -= normal * (origin @ normal + offset)
        v = np.median([view["R"][:, 1] for view in views], axis=0)
        v -= normal * (v @ normal)
        if np.linalg.norm(v) < .1:
            continue
        v /= np.linalg.norm(v)
        u = np.cross(v, normal)
        if u @ views[len(views) // 2]["R"][:, 0] < 0:
            u = -u
        coordinates = np.column_stack(((support - origin) @ u, (support - origin) @ v))
        low, high = np.quantile(coordinates, [.005, .995], axis=0)
        extent = high - low
        if extent.min() < threshold * 10:
            continue
        low -= extent * .12
        high += extent * .12
        step = max(float((high - low).max() / 1100), threshold * .55)
        width, height = np.ceil((high - low) / step).astype(int) + 1
        if min(width, height) < 32:
            continue
        target = np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]], np.float32)
        corners = origin + np.array([[low[0], low[1]], [high[0], low[1]], [high[0], high[1]], [low[0], high[1]]]) @ np.array([u, v])
        votes, visible = np.zeros((height, width), np.float32), np.zeros((height, width), np.float32)
        for view in views:
            q, depth = project(corners, view)
            if np.any(depth <= 0) or abs(normal @ ((origin - view["origin"]) / np.linalg.norm(origin - view["origin"]))) < .15:
                continue
            transform = cv2.getPerspectiveTransform(q.astype(np.float32), target)
            observed = cv2.dilate((view["distance"] < 1.5).astype(np.uint8), np.ones((3, 3), np.uint8))
            votes += cv2.warpPerspective(observed, transform, (width, height), flags=cv2.INTER_NEAREST)
            visible += cv2.warpPerspective(np.ones_like(observed), transform, (width, height), flags=cv2.INTER_NEAREST)
        edges = ((votes >= settings.min_views) & (votes / np.maximum(visible, 1) >= .45)).astype(np.uint8) * 255
        # The atlas supplies boundary proposals, never unobserved cells or point-connectivity meshes.
        quads, _ = image_quads(cv2.cvtColor(255 - edges, cv2.COLOR_GRAY2BGR), settings)
        for quad in sorted(quads, key=lambda q: -abs(cv2.contourArea(q)))[:24]:
            world = origin + (low + quad * ((high - low) / [width - 1, height - 1])) @ np.array([u, v])
            sources = []
            for view in views:
                q, depth = project(world, view)
                h, w = view["gray"].shape
                if np.any(depth <= 0) or q.min() < 2 or np.any(q[:, 0] > w - 3) or np.any(q[:, 1] > h - 3):
                    continue
                q = q.astype(np.float32)
                inside = quad_distances(q, view["xy"]) >= -2
                count = int((inside & (np.abs(view["xyz"] @ normal + offset) < threshold)).sum())
                sources.append((count, abs(cv2.contourArea(q)), view, q))
            for _, _, view, q in sorted(sources, key=lambda item: (-item[0], -item[1]))[:1]:
                proposals.append((view, q, pi))
    return proposals


def boundary_evidence(corners, views, settings=None, source=None):
    """A plane at the wrong depth must also explain boundaries in other views."""
    samples = np.concatenate([a[None] * (1 - np.linspace(.08, .92, 20)[:, None]) +
        b[None] * np.linspace(.08, .92, 20)[:, None] for a, b in zip(corners, np.roll(corners, -1, axis=0))])
    settings = settings or detection_settings()
    target = np.array([[0, 0], [95, 0], [95, 95], [0, 95]], np.float32)
    yy, xx = np.indices((96, 96))
    ring = (np.minimum.reduce([xx, yy, 95 - xx, 95 - yy]) < 16) & (np.minimum.reduce([xx, yy, 95 - xx, 95 - yy]) >= 2)
    def appearance(view):
        q, _ = project(corners, view)
        transform = cv2.getPerspectiveTransform(q.astype(np.float32), target)
        patch = cv2.warpPerspective(view["gray"], transform, (96, 96))
        return cv2.GaussianBlur(patch, (3, 3), 0)[ring].astype(float)
    reference = appearance(source) if source is not None else None
    if reference is not None:
        reference -= reference.mean()
    textured = reference is not None and reference.std() >= 5
    evidence = []
    for view in views:
        uv, depth = project(samples, view)
        h, w = view["distance"].shape
        valid = (depth > 0) & (uv[:, 0] >= 1) & (uv[:, 0] < w - 1) & (uv[:, 1] >= 1) & (uv[:, 1] < h - 1)
        if valid.mean() < .85:
            continue
        pixels = np.rint(uv[valid]).astype(int)
        distances = view["distance"][pixels[:, 1], pixels[:, 0]]
        agreement = float(np.mean(distances < 3.0))
        per_edge = edge_agreements(project(corners, view)[0], view["distance"])
        if agreement >= settings.edge_agreement and per_edge.min() >= settings.min_edge_agreement and float(np.median(distances)) < 2.:
            correlation = None
            if textured:
                patch = appearance(view)
                patch -= patch.mean()
                correlation = float(reference @ patch / max(np.linalg.norm(reference) * np.linalg.norm(patch), 1e-10))
                if correlation < settings.appearance_correlation:
                    continue
            evidence.append({"frame_id": view["camera"]["frame_id"], "edge_agreement": agreement,
                             "min_edge_agreement": float(per_edge.min()),
                             "border_appearance_correlation": correlation,
                             "median_edge_error_analysis_px": float(np.median(distances))})
    return evidence


def overlap(a, b):
    """Intersection on the first object's plane, normalized by the smaller patch."""
    n1, n2 = np.array(a["normal"]), np.array(b["normal"])
    if abs(n1 @ n2) < np.cos(np.deg2rad(15)):
        return 0.
    pa, pb = np.array(a["corners"], dtype=float), np.array(b["corners"], dtype=float)
    tolerance = max(max(a["width"], a["height"], b["width"], b["height"]) * .035,
                    4 * max(a.get("plane_threshold", 0), b.get("plane_threshold", 0)))
    if abs((pb.mean(axis=0) - pa.mean(axis=0)) @ n1) > tolerance:
        return 0.
    u = pa[1] - pa[0]
    u /= np.linalg.norm(u)
    v = np.cross(n1, u)
    qa, qb = [(np.column_stack(((p - pa[0]) @ u, (p - pa[0]) @ v))).astype(np.float32) for p in (pa, pb)]
    return float(cv2.intersectConvexConvex(cv2.convexHull(qa), cv2.convexHull(qb))[0] /
                 max(min(abs(cv2.contourArea(qa)), abs(cv2.contourArea(qb))), 1e-10))


def composite_boundary(candidate, candidates):
    """Prefer separate, disjoint observed cells to a rectangle covering several doors."""
    if candidate["height"] / candidate["width"] > 2:
        return False
    children = [c for c in candidates if .08 < c["area"] / candidate["area"] < .65 and overlap(c, candidate) > .85]
    disjoint = []
    for child in sorted(children, key=lambda c: -c["area"] * c["geometric_score"]):
        if not any(overlap(child, other) > .20 for other in disjoint):
            disjoint.append(child)
        if len(disjoint) >= 2 and sum(c["area"] for c in disjoint) > candidate["area"] * .60:
            return True
    return False


def covered_points(xyz, objects):
    """Reuse only observed finite patches at the same depth, never an infinite plane."""
    covered = np.zeros(len(xyz), dtype=bool)
    for obj in objects:
        corners, normal = np.asarray(obj["corners"], dtype=float), np.asarray(obj["normal"], dtype=float)
        u = corners[1] - corners[0]
        u /= np.linalg.norm(u)
        v = np.cross(normal, u)
        quad = np.column_stack(((corners - corners[0]) @ u, (corners - corners[0]) @ v)).astype(np.float32)
        xy = np.column_stack(((xyz - corners[0]) @ u, (xyz - corners[0]) @ v))
        tolerance = max(obj["plane_threshold"] * 1.5, 1e-8)
        covered |= (np.abs(xyz @ normal + obj["offset"]) <= tolerance) & (quad_distances(quad, xy) >= -tolerance)
    return covered


def covered_boundary(quad, view, objects):
    """Skip an old patch only with compatible orientation, depth support and image overlap."""
    normal = quad_normal(quad, view)
    if normal is None:
        return False
    inside = quad_distances(quad, view["xy"]) >= -2
    if inside.sum() < 3:
        return False
    area = abs(cv2.contourArea(quad))
    for obj in objects:
        if abs(normal @ np.array(obj["normal"])) < np.cos(np.deg2rad(8)):
            continue
        projected, depth = project(np.array(obj["corners"]), view)
        if np.any(depth <= 0):
            continue
        intersection = cv2.intersectConvexConvex(quad.astype(np.float32), projected.astype(np.float32))[0]
        if intersection / max(area, 1) < .90:
            continue
        if covered_points(view["xyz"][inside], [obj]).mean() >= .70:
            return True
    return False


def same_patch(candidate, old):
    if abs(np.array(candidate["normal"]) @ np.array(old["normal"])) < np.cos(np.deg2rad(8)):
        return False
    separation = abs((np.mean(candidate["corners"], axis=0) - np.mean(old["corners"], axis=0)) @ np.array(old["normal"]))
    if separation > 1.5 * min(candidate["plane_threshold"], old["plane_threshold"]):
        return False
    # A larger, newly observed extension must not disappear merely because it contains an old patch.
    return overlap(candidate, old) * min(candidate["area"], old["area"]) / candidate["area"] > .90


def build_planar_objects(model, scene, images_root, textures_dir, previous=None, max_frames=12, max_objects=48,
                         profile="balanced", plane_error_px=None, source_frame_ids=None, proposals=None,
                         incremental=False):
    started = time.perf_counter()
    settings = detection_settings(profile, plane_error_px)
    rows = [p for p in scene["points"] if p[7] <= 1.5 and p[8] >= 3]
    points = {int(p[0]): np.array(p[1:4]) for p in rows}
    xyz = np.array(list(points.values()))
    cameras = scene["cameras"]
    collection = {"schema_version": "planar-objects/1.0", "scene_binding_sha256": scene_binding(scene),
        "unit": scene["scale"]["unit"], "segment_id": scene.get("segment_id", 0), "objects": [],
        "method": "native_tracks+seam_cells+calibrated_rectangles+ransac_svd+multiview_edges",
        "classification": "geometric_rectangular_patch; semantic_object_category_unverified",
        "detection": {"profile": profile, **asdict(settings)}}
    old_objects = previous.get("objects", []) if previous and previous.get("segment_id") == collection["segment_id"] else []
    if len(xyz) < 24 or len(cameras) < 3:
        if incremental:
            Path(textures_dir).mkdir(parents=True, exist_ok=True)
            collection["objects"] = old_objects
            collection["id_counter"] = (previous or {}).get("id_counter", 0)
        collection["statistics"] = {"objects": len(collection["objects"]), "reason": "insufficient_geometry",
            "new_objects": 0, "retained_objects": len(old_objects) if incremental else 0}
        return collection
    depth = float(np.median(np.linalg.norm(xyz - np.median([c["position"] for c in cameras], axis=0), axis=1)))
    threshold = depth / np.median([c["params"][0] for c in cameras]) * max(settings.plane_error_px, 4 * np.median([p[7] for p in rows]))
    novel = ~covered_points(xyz, old_objects) if incremental else np.ones(len(xyz), dtype=bool)
    planes = fit_planes(xyz[novel], threshold)
    if incremental:
        for obj in old_objects:
            n, d = np.array(obj["normal"]), obj["offset"]
            if not any(abs(n @ other) > .995 and abs(d - np.sign(n @ other) * offset) < threshold for other, offset in planes):
                planes.append((n, d))
    indices = np.unique(np.linspace(0, len(cameras) - 1, min(max_frames, len(cameras))).astype(int))
    views = [prepare_view(model, cameras[i], images_root, points, settings=settings,
             proposal=(proposals or {}).get(cameras[i]["frame_id"])) for i in indices]
    views = [view for view in views if view is not None]
    consensus = consensus_quads(planes, xyz, views, threshold, settings) if not incremental and len(views) >= settings.min_views else []
    for view, quad, pi in consensus:
        view["quads"].append(quad)
        view.setdefault("plane_hints", {})[id(quad)] = pi
    candidates = []
    rejected = Counter()
    local_planes = 0
    for view in views:
        if source_frame_ids is not None and view["camera"]["frame_id"] not in source_frame_ids:
            continue
        for quad in view["quads"]:
            if incremental and covered_boundary(quad, view, old_objects):
                rejected["existing_surface"] += 1
                continue
            expected_normal = quad_normal(quad, view)
            if expected_normal is None:
                rejected["degenerate_boundary"] += 1
                continue
            distances = quad_distances(quad, view["xy"])
            inside = distances >= -2
            # Boundary support is given priority over features visible through glass.
            boundary = inside & (distances < max(5., np.sqrt(abs(cv2.contourArea(quad))) * .08))
            if inside.sum() < settings.min_support or boundary.sum() < settings.min_border:
                rejected["source_support"] += 1
                continue
            hint = view.get("plane_hints", {}).get(id(quad))
            hypotheses = [(hint, planes[hint])] if hint is not None else list(enumerate(planes))
            if hint is None and inside.sum() >= 8 and not any((inside & (np.abs(view["xyz"] @ n + d) < threshold)).sum() >= max(8, inside.sum() * .65)
                    for n, d in planes):
                for local in fit_planes(view["xyz"][inside], threshold, minimum=6, maximum=1, trials=96):
                    hypotheses.append((len(planes) + local_planes, local))
                    local_planes += 1
            compatible = [(pi, (n, d)) for pi, (n, d) in hypotheses if
                abs(n @ expected_normal) >= np.cos(np.deg2rad(settings.max_normal_angle_deg))]
            rejected["boundary_plane_orientation"] += len(hypotheses) - len(compatible)
            compatible.sort(key=lambda item: -float((boundary & (np.abs(view["xyz"] @ item[1][0] + item[1][1]) < threshold)).sum() +
                .15 * (inside & (np.abs(view["xyz"] @ item[1][0] + item[1][1]) < threshold)).sum()))
            for pi, (normal, offset) in compatible[:4]:
                residual = np.abs(view["xyz"] @ normal + offset)
                support = inside & (residual < threshold)
                border_support = boundary & support
                if support.sum() < settings.min_support or border_support.sum() < settings.min_border or border_support.sum() / boundary.sum() < .30:
                    rejected["plane_support"] += 1
                    continue
                inferred = intersect_plane(quad, normal, offset, view)
                if inferred is None:
                    rejected["shape_or_view_angle"] += 1
                    continue
                corners, lengths = inferred
                observations = view["xy"][support].astype(np.float32)
                coverage = abs(cv2.contourArea(cv2.convexHull(observations))) / abs(cv2.contourArea(quad))
                if coverage < settings.min_coverage:
                    rejected["support_coverage"] += 1
                    continue
                evidence = boundary_evidence(corners, views, settings, source=view)
                if len(evidence) < settings.min_views:
                    rejected["multiview_boundary"] += 1
                    continue
                # Require distinct camera centers, rather than repeated observations at one pose.
                centers = [v["origin"] for v in views if v["camera"]["frame_id"] in {e["frame_id"] for e in evidence}]
                baseline = max(np.linalg.norm(a - b) for a in centers for b in centers)
                if baseline < np.linalg.norm(corners.mean(axis=0) - view["origin"]) * .012:
                    rejected["camera_baseline"] += 1
                    continue
                if normal @ (view["origin"] - corners.mean(axis=0)) < 0:
                    normal, offset = -normal, -offset
                width, height = float((lengths[0] + lengths[2]) / 2), float((lengths[1] + lengths[3]) / 2)
                shape = "vertical" if height / width > 1.6 else "horizontal" if width / height > 1.6 else "compact"
                orientation = "nonparallel" if planes and abs(normal @ planes[0][0]) < np.cos(np.deg2rad(30)) else "reference"
                score = float(.4 * min(1., support.sum() / 35) + .3 * min(1., len(evidence) / 5) + .3 * min(1., coverage / .6))
                candidates.append({"corners": corners.tolist(), "normal": normal.tolist(), "offset": offset,
                    "width": width, "height": height, "area": width * height, "plane_index": pi,
                    "label": {"vertical": "문 형태 평면", "horizontal": "가로형 평면", "compact": "사각 평면"}[shape],
                    "shape": shape, "plane_orientation": orientation,
                    "boundary_source": "multiview_plane_consensus" if hint is not None else "image_lines_or_contours",
                    "geometric_score": score, "support_count": int(support.sum()),
                    "support_point_ids": view["ids"][support].astype(int).tolist()[:160],
                    "support_points": view["xyz"][support].tolist()[:160], "coverage": float(coverage),
                    "plane_rms": float(np.sqrt(np.mean(residual[support] ** 2))), "plane_threshold": float(threshold),
                    "source_frame_id": view["camera"]["frame_id"], "source_quad": quad.tolist(),
                    "source_analysis_size": [int(view["image"].shape[1]), int(view["image"].shape[0])],
                    "evidence": evidence, "_view": view, "_quad": quad})
                break  # One image boundary has one best supported depth, including its own local plane if needed.
    accepted = []
    original_count = len(candidates)
    candidates = [c for c in candidates if not composite_boundary(c, candidates)]
    rejected["composite_boundary"] = original_count - len(candidates)
    for candidate in sorted(candidates, key=lambda c: c["geometric_score"] * np.sqrt(c["area"]), reverse=True):
        if not any(overlap(candidate, old) > .70
                   for old in accepted):
            accepted.append(candidate)
        if len(accepted) >= max_objects:
            break
    if incremental:
        accepted = [obj for obj in accepted if not any(same_patch(obj, old) for old in old_objects)]
        accepted = accepted[:max(0, max_objects - len(old_objects))]
    counter = max([int(o["object_id"].split("-")[-1]) for o in old_objects] + [int(previous.get("id_counter", 0)) if previous else 0])
    used = set()
    plane_groups = []
    textures_dir = Path(textures_dir)
    textures_dir.mkdir(parents=True, exist_ok=True)
    for obj in accepted:
        matches = sorted(((overlap(obj, old), old) for old in ([] if incremental else old_objects) if old["object_id"] not in used and
            .5 < obj["area"] / max(old["area"], 1e-10) < 2), key=lambda x: x[0], reverse=True)
        if matches and matches[0][0] > .55:
            obj["object_id"] = matches[0][1]["object_id"]
        else:
            counter += 1
            obj["object_id"] = f"object-{counter:04d}"
        used.add(obj["object_id"])
        obj.pop("plane_index")
        normal = np.array(obj["normal"])
        center = np.mean(obj["corners"], axis=0)
        group = next((i for i, (n, d) in enumerate(plane_groups) if abs(n @ normal) > .995 and
                      abs(center @ n + d) < threshold * 2), None)
        if group is None:
            group = len(plane_groups)
            plane_groups.append((normal, obj["offset"]))
        obj["plane_id"] = f"plane-{group + 1:03d}"
        source = obj.pop("_view")["image"]
        quad = obj.pop("_quad")
        aspect = obj["width"] / obj["height"]
        w, h = (min(1024, max(64, round(768 * aspect))), 768) if aspect < 1 else (768, min(1024, max(64, round(768 / aspect))))
        transform = cv2.getPerspectiveTransform(quad.astype(np.float32), np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], np.float32))
        texture = cv2.warpPerspective(source, transform, (w, h))
        obj["texture_file"] = obj["object_id"] + ".jpg"
        if not cv2.imwrite(str(textures_dir / obj["texture_file"]), texture, [cv2.IMWRITE_JPEG_QUALITY, 90]):
            raise OSError("Cannot write planar object texture")
        obj["inference"] = "boundary-supported planar completion; appearance is observed RGB, not measured solid geometry"
    collection["objects"] = old_objects + accepted if incremental else accepted
    collection["id_counter"] = counter
    collection["statistics"] = {"objects": len(collection["objects"]), "planes": len(planes), "analysis_frames": len(views),
        "image_quads": sum(len(v["quads"]) for v in views), "supported_candidates": len(candidates),
        "local_planes": local_planes, "consensus_proposals": len(consensus), "rejected": dict(rejected),
        "plane_groups": len(plane_groups), "object_limit_reached": len(accepted) >= max_objects,
        "shapes": dict(Counter(o["shape"] for o in accepted)),
        "processing_seconds": time.perf_counter() - started}
    if incremental:
        collection["method"] = "incremental_frame_proposals+finite_patch_reuse+native_tracks+multiview_confirmation"
        collection["statistics"].update(new_objects=len(accepted), retained_objects=len(old_objects),
            reused_boundaries=rejected["existing_surface"], new_landmarks=int(novel.sum()),
            reused_landmarks=int((~novel).sum()), source_frames=len(source_frame_ids or []),
            object_limit_reached=len(collection["objects"]) >= max_objects)
    return collection


def main():
    import pycolmap
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-frames", type=int, default=12)
    parser.add_argument("--profile", choices=("balanced", "coverage", "strict"), default="balanced")
    parser.add_argument("--plane-error-px", type=float, help="Pixel-equivalent plane tolerance, 1..12; profile default if omitted")
    args = parser.parse_args()
    if not 3 <= args.max_frames <= 60:
        parser.error("max-frames must be between 3 and 60")
    if args.output.exists():
        parser.error("Use a new object output folder")
    try:
        detection_settings(args.profile, args.plane_error_px)
    except ValueError as error:
        parser.error(str(error))
    scene = json.loads((args.result / "scene.json").read_text(encoding="utf-8"))
    assets = json.loads((args.result / "local_assets.json").read_text(encoding="utf-8"))
    models = [pycolmap.Reconstruction(p) for p in sorted((args.result / "sparse").iterdir()) if (p / "images.bin").is_file()]
    if not models:
        parser.error("Native SfM model is required")
    names = {c["frame_id"] for c in scene["cameras"]}
    models = [m for m in models if {im.name for im in m.images.values() if im.has_pose} == names]
    if not models:
        parser.error("Native model cameras do not match the result")
    model = max(models, key=lambda m: (m.num_reg_images(), m.num_points3D()))
    objects = build_planar_objects(model, scene, assets["images_root"], args.output / "textures", max_frames=args.max_frames,
        profile=args.profile, plane_error_px=args.plane_error_px)
    (args.output / "objects.json").write_text(json.dumps(objects, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    print(json.dumps(objects["statistics"]))


if __name__ == "__main__":
    main()
