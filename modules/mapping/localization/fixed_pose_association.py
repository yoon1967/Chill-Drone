"""Read-only association AFTER SLAM pose estimation; never feed changes to SLAM.

Geometry is checked with the supplied camera poses held fixed. A shared pixel or
color alone cannot join identities. Native descriptors and every cross-view
projection must agree; simultaneous observations of two IDs remain distinct.
"""
from collections import defaultdict

import cv2
import numpy as np


class FixedPoseAssociation:
    def __init__(self):
        self.map_id = None
        self.aliases = {}
        self.cursor = 0

    def apply(self, native, camera):
        if self.map_id != native["map_id"]:
            self.map_id, self.aliases, self.cursor = native["map_id"], {}, 0
        points = {p["id"]: p for p in native["points"]}
        previous_aliases = self.aliases
        # Recheck each snapshot: a prior merge cannot override contradictory views.
        self.aliases = {pid: pid for pid in points}
        # Old native snapshots have no descriptors and retain their native IDs.
        catalog = [p for p in points.values() if len(p.get("descriptor", [])) == 32
                   and np.isfinite(p["xyz"]).all()]
        # Bound brute-force work. Unexamined points keep their existing identities.
        catalog.sort(key=lambda p: p["id"], reverse=True)
        catalog = catalog[:12000]
        quality = {"basis": "fixed_camera_poses; orb_descriptor_and_cross_reprojection",
                   "camera_poses_modified": False, "descriptor_points": len(catalog),
                   "query_points": min(len(catalog), 512), "merged_pairs": 0}
        evidence = defaultdict(list)
        poses = []
        for index, frame in enumerate(native["keyframes"]):
            poses.append(np.asarray(frame["T_cw"], dtype=float))
            for obs in frame["observations"]:
                evidence[obs["point_id"]].append((index, obs["xy"], obs.get("sigma2", 1.)))
        rotations = np.asarray([T[:3, :3] for T in poses])
        translations = np.asarray([T[:3, 3] for T in poses])
        frame_sets = {pid: {e[0] for e in entries} for pid, entries in evidence.items()}
        # Pin-hole OPENCV distortion, vectorized over independently fixed views.
        def agrees(xyz, entries):
            if len(entries) < 2:
                return False
            indices = np.asarray([e[0] for e in entries])
            xc = np.einsum('nij,j->ni', rotations[indices], xyz) + translations[indices]
            if not np.isfinite(xc).all() or np.any(xc[:, 2] <= 0):
                return False
            x, y = (xc[:, :2] / xc[:, 2, None]).T
            r2 = x*x + y*y
            k1, k2, p1, p2 = camera["distortion"]
            radial = 1 + k1*r2 + k2*r2*r2
            predicted = np.column_stack((camera["fx"] * (x*radial + 2*p1*x*y + p2*(r2 + 2*x*x)) + camera["cx"],
                                         camera["fy"] * (y*radial + p1*(r2 + 2*y*y) + 2*p2*x*y) + camera["cy"]))
            xy = np.asarray([e[1] for e in entries])
            variance = np.asarray([e[2] for e in entries])
            return bool(np.all(np.sum((predicted - xy)**2, axis=1) <= 5.991 * variance))
        if len(catalog) >= 2:
            descriptors = np.asarray([p["descriptor"] for p in catalog], dtype=np.uint8)
            # Rotate bounded queries through the catalog to revisit later pose refinements.
            indices = (np.arange(min(len(catalog), 512)) + self.cursor) % len(catalog)
            self.cursor = (self.cursor + len(indices)) % len(catalog)
            matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
            cache = {}
            def choices(query_indices):
                if not len(query_indices):
                    return
                matches = matcher.knnMatch(descriptors[query_indices], descriptors, k=min(4, len(catalog)))
                for index, neighbors in zip(query_indices, matches):
                    a = catalog[index]
                    valid = []
                    for match in neighbors:
                        b = catalog[match.trainIdx]
                        if (index == match.trainIdx or match.distance > 40
                                or frame_sets[a["id"]] & frame_sets[b["id"]]):
                            continue
                        if agrees(a["xyz"], evidence[b["id"]]) and agrees(b["xyz"], evidence[a["id"]]):
                            valid.append((match.distance, match.trainIdx))
                    valid.sort()
                    cache[int(index)] = valid[0][1] if valid and (len(valid) == 1 or valid[0][0] < .75 * valid[1][0]) else None
            choices(indices)
            reverse = np.asarray(sorted({j for j in cache.values() if j is not None and j not in cache}), dtype=int)
            choices(reverse)
            quality["query_points"] += len(reverse)
            for i in indices:
                j = cache[int(i)]
                if j is None or cache.get(j) != i:
                    continue
                a, b = catalog[i]["id"], catalog[j]["id"]
                left, right = self.aliases[a], self.aliases[b]
                if left == right:
                    continue
                canonical, replaced = min(left, right), max(left, right)
                # Avoid transitive co-observation conflicts when a component grows.
                left_frames, right_frames = set(), set()
                for pid in points:
                    if self.aliases[pid] == canonical: left_frames.update(frame_sets[pid])
                    if self.aliases[pid] == replaced: right_frames.update(frame_sets[pid])
                if left_frames & right_frames:
                    continue
                for pid in self.aliases:
                    if self.aliases[pid] == replaced: self.aliases[pid] = canonical
                quality["merged_pairs"] += 1
        groups = defaultdict(list)
        for pid, point in points.items():
            groups[self.aliases[pid]].append(point)
        merged = []
        used = set()
        for canonical, members in groups.items():
            member_ids = {p["id"] for p in members}
            historical = min(previous_aliases.get(pid, pid) for pid in member_ids)
            if historical not in used and (historical not in points or historical in member_ids):
                canonical = historical
            used.add(canonical)
            for pid in member_ids:
                self.aliases[pid] = canonical
            # Keep the strongest existing 3D estimate; no camera/point optimization.
            representative = max(members, key=lambda p: p["observations"])
            merged.append({**representative, "id": canonical})
        frames = [{**f, "observations": [{**o, "point_id": self.aliases[o["point_id"]]} for o in f["observations"]]}
                  for f in native["keyframes"]]
        # Only keep live aliases and their stable representatives; bounded by the map.
        self.aliases = {pid: self.aliases[pid] for pid in points}
        quality["renamed_points"] = sum(pid != alias for pid, alias in self.aliases.items())
        return {**native, "points": merged, "keyframes": frames}, quality
