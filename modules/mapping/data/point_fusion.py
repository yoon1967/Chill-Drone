"""Bounded spatial point fusion; retain coverage by coarsening, never by age."""
from __future__ import annotations

import cv2
import numpy as np
from .mapping_limits import DEFAULT_POINT_BUDGET


class SemanticSurface:
    """Confidence-weighted voxel centroids with a fixed point budget.

    The world grid has a fixed origin. When full, adjacent cells are merged at
    a fixed additional spacing, including negative coordinates and old observations.
    Persistent and temporary work are bounded by the budget plus one image.
    No volume, triangles, normals, or per-frame Python voxel loop is created.
    """
    def __init__(self, voxel_size, *, max_cells=DEFAULT_POINT_BUDGET, spacing_increment=None):
        if (not np.isfinite(voxel_size) or voxel_size <= 0 or type(max_cells) is not int or max_cells < 16
                or (spacing_increment is not None
                    and (not np.isfinite(spacing_increment) or spacing_increment <= 0))):
            raise ValueError('invalid_point_fusion_budget')
        self.size, self.max_cells = float(voxel_size), max_cells
        self.base_size = self.size
        self.spacing_increment = float(spacing_increment) if spacing_increment is not None else None
        self.frames = self.revision = self.coarsenings = 0
        self.keys = np.empty((0, 3), np.int64)
        self.xyz = self.rgb = np.empty((0, 3), np.float64)
        self.weights = np.empty(0, np.float64)
        self.votes = np.empty((0, 9), np.float64)
        self.bounds = None
        self._cached_points = None
        self._cached_labels = None

    @property
    def cells(self):
        return self.keys

    def _calibrate_spacing(self, xyz, rows, columns, shape, stride):
        # Compare immediate neighbors on the sampled image grid in map units.
        # Mask holes never create artificial edges across disconnected surfaces.
        indices = np.full(((shape[0] + stride - 1) // stride,
                           (shape[1] + stride - 1) // stride), -1, dtype=np.intp)
        indices[rows // stride, columns // stride] = np.arange(len(xyz))
        distances = []
        for left, right in ((indices[:, :-1], indices[:, 1:]), (indices[:-1], indices[1:])):
            valid = (left >= 0) & (right >= 0)
            distances.append(np.linalg.norm(xyz[left[valid]] - xyz[right[valid]], axis=1))
        distances = np.concatenate(distances)
        distances = distances[np.isfinite(distances) & (distances > 0)]
        lower = self.base_size / 4
        if len(distances) < 8:
            self.spacing_increment = lower
            return
        # Surface sample count grows with area. Use a fraction of the predicted
        # pitch at 10x coverage so a budget crossing does not erase fine detail.
        growth = np.sqrt(max(1., 10 * len(xyz) / self.max_cells))
        target_pitch = np.median(distances) * growth
        self.spacing_increment = float(np.clip(target_pitch / 4, lower, self.base_size / 2))

    @staticmethod
    def _group(keys):
        # Sort numeric columns directly; np.unique(axis=0) uses structured-row
        # comparisons that dominate fusion time once the point budget grows.
        order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
        ordered = keys[order]
        starts = np.r_[True, np.any(ordered[1:] != ordered[:-1], axis=1)]
        return order, ordered, starts

    def _merge(self, keys, xyz, rgb, weights, votes, *, budget=None):
        order, ordered, starts = self._group(keys)
        if budget is not None and np.count_nonzero(starts) > budget:
            if budget < 8 and len(np.unique(xyz < 0, axis=0)) > budget:
                raise ValueError('invalid_point_fusion_budget')
            increment = self.spacing_increment or self.base_size / 4
            # Probe occupancy with grid keys. Aggregate colors, confidence and
            # centroids once at the accepted spacing, directly from the inputs.
            offset = 1
            while True:
                steps = self.coarsenings + offset
                size = self.base_size + steps * increment
                order, ordered, starts = self._group(np.floor(xyz / size).astype(np.int64))
                if np.count_nonzero(starts) <= budget:
                    self.spacing_increment = increment
                    self.size, self.coarsenings = size, steps
                    break
                offset *= 2
        inverse = np.empty(len(keys), np.intp)
        inverse[order] = np.cumsum(starts) - 1
        unique = ordered[starts]
        total = np.bincount(inverse, weights=weights, minlength=len(unique))
        self.xyz = np.column_stack([np.bincount(inverse, weights=xyz[:, i]*weights, minlength=len(unique))/total for i in range(3)])
        self.rgb = np.column_stack([np.bincount(inverse, weights=rgb[:, i]*weights, minlength=len(unique))/total for i in range(3)])
        combined_votes = np.column_stack([np.bincount(inverse, weights=votes[:, i], minlength=len(unique)) for i in range(9)])
        self.weights = np.minimum(total, 30.)
        self.votes = combined_votes * (self.weights/total)[:, None]
        self.keys = unique

    def compact(self, budget=None):
        budget = self.max_cells if budget is None else budget
        if type(budget) is not int or budget < 1:
            raise ValueError('invalid_point_fusion_budget')
        if len(self.keys) > budget:
            self._merge(self.keys, self.xyz, self.rgb, self.weights, self.votes, budget=budget)
        self._cached_points = None
        self._cached_labels = None

    def integrate(self, rgb, depth, pose, params, labels, confidence):
        from modules.mapping.data.rescue_mapping import unproject, PERSON
        person = cv2.dilate((labels == PERSON).astype(np.uint8), np.ones((3,3), np.uint8)) > 0
        static = np.where(person | ~np.isfinite(depth), 0, depth)
        stride = 3
        xyz, v, u = unproject(static, pose, params, stride=stride)
        if len(xyz):
            if self.spacing_increment is None:
                self._calibrate_spacing(xyz, v, u, depth.shape, stride)
            low, high = xyz.min(axis=0), xyz.max(axis=0)
            self.bounds = [low, high] if self.bounds is None else [np.minimum(self.bounds[0], low), np.maximum(self.bounds[1], high)]
            keys = np.floor(xyz/self.size).astype(np.int64)
            weights = np.maximum(.1, confidence[v, u]).astype(np.float64)
            votes = np.zeros((len(xyz), 9), np.float64)
            votes[np.arange(len(xyz)), labels[v, u]] = weights
            self._merge(np.concatenate((self.keys, keys)), np.concatenate((self.xyz, xyz)),
                np.concatenate((self.rgb, rgb[v, u])), np.concatenate((self.weights, weights)),
                np.concatenate((self.votes, votes)), budget=self.max_cells)
            self._cached_points = self._cached_labels = None
            self.revision += 1
        self.frames += 1

    def points(self, limit=None):
        if limit is not None and len(self.keys) > limit:
            self.compact(limit)
        if self._cached_points is None:
            # Box the numeric columns once, then allocate one row list per point.
            # Separate position/color lists plus concatenation triple the number
            # of GC-tracked rows and trigger long scans of loaded model objects.
            rows = np.empty((len(self.xyz), 6), dtype=object)
            rows[:, :3] = np.round(self.xyz, 5)
            rows[:, 3:] = np.rint(self.rgb).clip(0,255).astype(np.uint8)
            self._cached_points = rows.tolist()
        return self._cached_points

    def metadata(self):
        return dict(point_budget=self.max_cells, static_points=len(self.keys), voxel_size=self.size,
            base_voxel_size=self.base_size, spacing_increment=self.spacing_increment or self.base_size / 4,
            spacing_policy='fixed_increment', coarsenings=self.coarsenings, observations=self.frames,
            coverage_bounds=[v.tolist() for v in self.bounds] if self.bounds is not None else [],
            policy='spatial_voxel_merge_no_age_eviction')

    def labels(self):
        if self._cached_labels is None:
            self._cached_labels = np.argmax(self.votes, axis=1).astype(np.uint8).tolist()
        return self._cached_labels

    def close(self):
        self.keys = np.empty((0,3), np.int64)
        self.xyz = self.rgb = np.empty((0,3), np.float64)
        self.weights = np.empty(0, np.float64)
        self.votes = np.empty((0,9), np.float64)
        self._cached_points = []
        self._cached_labels = []
        self.bounds = None
