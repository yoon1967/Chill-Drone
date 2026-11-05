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
    def __init__(self, voxel_size, *, max_cells=DEFAULT_POINT_BUDGET, spacing_increment=.005):
        if (not np.isfinite(voxel_size) or voxel_size <= 0 or type(max_cells) is not int or max_cells < 16
                or not np.isfinite(spacing_increment) or spacing_increment <= 0):
            raise ValueError('invalid_point_fusion_budget')
        self.size, self.max_cells = float(voxel_size), max_cells
        self.base_size = self.size
        self.spacing_increment = float(spacing_increment)
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

    def _merge(self, keys, xyz, rgb, weights, votes):
        # Sort numeric columns directly; np.unique(axis=0) uses structured-row
        # comparisons that dominate fusion time once the point budget grows.
        order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
        ordered = keys[order]
        starts = np.r_[True, np.any(ordered[1:] != ordered[:-1], axis=1)]
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
        while len(self.keys) > budget:
            self.coarsenings += 1
            self.size = self.base_size + self.coarsenings * self.spacing_increment
            # A non-nested grid needs the actual retained centroids, not integer
            # division of the preceding grid indices (especially around zero).
            self._merge(np.floor(self.xyz / self.size).astype(np.int64), self.xyz, self.rgb, self.weights, self.votes)
        self._cached_points = None
        self._cached_labels = None

    def integrate(self, rgb, depth, pose, params, labels, confidence):
        from modules.mapping.data.rescue_mapping import unproject, PERSON
        person = cv2.dilate((labels == PERSON).astype(np.uint8), np.ones((3,3), np.uint8)) > 0
        static = np.where(person | ~np.isfinite(depth), 0, depth)
        xyz, v, u = unproject(static, pose, params, stride=3)
        if len(xyz):
            low, high = xyz.min(axis=0), xyz.max(axis=0)
            self.bounds = [low, high] if self.bounds is None else [np.minimum(self.bounds[0], low), np.maximum(self.bounds[1], high)]
            keys = np.floor(xyz/self.size).astype(np.int64)
            weights = np.maximum(.1, confidence[v, u]).astype(np.float64)
            votes = np.zeros((len(xyz), 9), np.float64)
            votes[np.arange(len(xyz)), labels[v, u]] = weights
            self._merge(np.concatenate((self.keys, keys)), np.concatenate((self.xyz, xyz)),
                np.concatenate((self.rgb, rgb[v, u])), np.concatenate((self.weights, weights)), np.concatenate((self.votes, votes)))
            self.compact()
            self.revision += 1
        self.frames += 1

    def points(self, limit=None):
        if limit is not None and len(self.keys) > limit:
            self.compact(limit)
        if self._cached_points is None:
            positions = np.round(self.xyz, 5).tolist()
            colors = np.rint(self.rgb).clip(0,255).astype(np.uint8).tolist()
            self._cached_points = [p+c for p,c in zip(positions, colors)]
        return self._cached_points

    def metadata(self):
        return dict(point_budget=self.max_cells, static_points=len(self.keys), voxel_size=self.size,
            base_voxel_size=self.base_size, spacing_increment=self.spacing_increment,
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
