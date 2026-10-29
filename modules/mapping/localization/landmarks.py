"""Confirm landmarks against already-estimated, fixed snapshot camera poses."""
import cv2
import numpy as np


class UnconfirmedGeometry(ValueError):
    def __init__(self, quality):
        super().__init__("awaiting_confirmed_landmarks")
        self.quality = quality


def confirm_landmarks(native, camera, min_views=3, min_parallax_deg=1.):
    K = np.array([[camera["fx"], 0., camera["cx"]],
                  [0., camera["fy"], camera["cy"]], [0., 0., 1.]])
    distortion = np.asarray(camera["distortion"], dtype=float)
    points = {p["id"]: p for p in native["points"]}
    point_ids = list(points)
    point_index = {pid: i for i, pid in enumerate(point_ids)}
    all_xyz = np.asarray([points[pid]["xyz"] for pid in point_ids], dtype=float).reshape(-1, 3)
    finite_points = np.isfinite(all_xyz).all(axis=1)
    support_ids, support_errors, support_rays = [], [], []
    quality = {"policy": "multi_view_reprojection/1.0", "min_views": min_views,
               "min_parallax_deg": min_parallax_deg, "max_normalized_error_squared": 5.991,
               "candidate_points": len(points), "confirmed_points": 0,
               "insufficient_views": 0, "insufficient_parallax": 0,
               "invalid_points": 0, "rejected_observations": 0}
    frames, frame_ids = [], set()
    for frame in native["keyframes"]:
        if frame["id"] in frame_ids:
            raise ValueError("duplicate_native_keyframe")
        frame_ids.add(frame["id"])
        T = np.asarray(frame["T_cw"], dtype=float)
        if (T.shape != (4, 4) or not np.isfinite(T).all()
                or not np.allclose(T[3], [0, 0, 0, 1])
                or not np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-4)
                or not np.isclose(np.linalg.det(T[:3, :3]), 1., atol=1e-4)):
            raise ValueError("invalid_native_camera_pose")
        R, t = T[:3, :3], T[:3, 3]
        observations = [o for o in frame["observations"] if o["point_id"] in points]
        valid = []
        if observations:
            rows = np.asarray([point_index[o["point_id"]] for o in observations])
            xyz = all_xyz[rows]
            finite = np.isfinite(xyz).all(axis=1)
            projected = np.full((len(xyz), 2), np.nan)
            if finite.any():
                projected[finite] = cv2.projectPoints(xyz[finite], cv2.Rodrigues(R)[0], t, K, distortion)[0].reshape(-1, 2)
            depth = (xyz @ R.T + t)[:, 2]
            xy = np.asarray([obs["xy"] for obs in observations], dtype=float)
            sigma2 = np.asarray([obs.get("sigma2", 1.) for obs in observations], dtype=float)
            error2 = np.sum((projected - xy) ** 2, axis=1)
            mask = (np.isfinite(xy).all(axis=1) & np.isfinite(error2) & np.isfinite(sigma2)
                    & (sigma2 > 0) & np.isfinite(depth) & (depth > 0) & (error2 <= 5.991 * sigma2)
                    & (xy[:, 0] >= 0) & (xy[:, 0] < camera["width"])
                    & (xy[:, 1] >= 0) & (xy[:, 1] < camera["height"]))
            indices = np.flatnonzero(mask)
            ids = np.asarray([obs["point_id"] for obs in observations])
            _, first = np.unique(ids[indices], return_index=True)
            indices = indices[np.sort(first)]
            quality["rejected_observations"] += len(observations) - len(indices)
            center = -R.T @ t
            valid = [observations[index] for index in indices]
            rays = xyz[indices] - center
            lengths = np.linalg.norm(rays, axis=1)
            rays /= lengths[:, None]
            support_ids.append(rows[indices])
            support_errors.append(np.sqrt(error2[indices]))
            support_rays.append(rays)
        frames.append({**frame, "observations": valid})
    observed = np.concatenate(support_ids) if support_ids else np.empty(0, dtype=int)
    residuals = np.concatenate(support_errors) if support_errors else np.empty(0)
    rays = np.concatenate(support_rays) if support_rays else np.empty((0, 3))
    counts = np.bincount(observed, minlength=len(points))
    totals = np.bincount(observed, weights=residuals, minlength=len(points))
    eligible = finite_points & (counts >= min_views)
    parallax = np.zeros(len(points), dtype=bool)
    if len(observed):
        # Most landmarks clear 1 degree against their first view. Vectorize this
        # common case; only the narrow angular boundary needs exact pair checks.
        unique, first = np.unique(observed, return_index=True)
        references = np.zeros((len(points), 3))
        references[unique] = rays[first]
        cosines = np.ones(len(points))
        np.minimum.at(cosines, observed, np.sum(rays * references[observed], axis=1))
        threshold = np.cos(np.deg2rad(min_parallax_deg))
        parallax = cosines <= threshold
        uncertain = eligible & ~parallax & (cosines <= np.cos(np.deg2rad(min_parallax_deg / 2)))
        for index in np.flatnonzero(uncertain):
            group = rays[observed == index]
            parallax[index] = np.min(group @ group.T) <= threshold
    keep = np.flatnonzero(eligible & parallax)
    confirmed = {point_ids[i]: {**points[point_ids[i]], "observations": int(counts[i])} for i in keep}
    errors = {point_ids[i]: float(totals[i] / counts[i]) for i in keep}
    quality["invalid_points"] = int(np.sum(~finite_points))
    quality["insufficient_views"] = int(np.sum(finite_points & ~eligible))
    quality["insufficient_parallax"] = int(np.sum(eligible & ~parallax))
    quality["confirmed_points"] = len(confirmed)
    for frame in frames:
        frame["observations"] = [o for o in frame["observations"] if o["point_id"] in confirmed]
    return {**native, "points": list(confirmed.values()), "keyframes": frames}, errors, quality
