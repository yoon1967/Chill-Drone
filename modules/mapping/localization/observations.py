"""Optional distinct-frame evidence for one coherent sparse-map snapshot (stdlib only)."""


def native_observations(scene, native):
    """Never borrow a newer map's observations for an earlier replay revision."""
    if (native.get("schema_version") != "orb-slam3/1.0"
            or scene.get("revision") != native.get("revision")
            or scene.get("segment_id") != native.get("map_id")):
        return None
    cameras = scene.get("cameras", [])
    keyframes = native.get("keyframes", [])
    catalog = {(c["image_id"], c["frame_id"]): i for i, c in enumerate(cameras)}
    if (len(catalog) != len(cameras) or len(keyframes) != len(cameras)
            or {(k["id"], k["frame_id"]) for k in keyframes} != set(catalog)):
        return None
    frames = {str(p[0]): set() for p in scene["points"]}
    for keyframe in keyframes:
        index = catalog[(keyframe["id"], keyframe["frame_id"])]
        for observation in keyframe.get("observations", []):
            point = str(observation["point_id"])
            if point in frames:
                frames[point].add(index)
    return {"schema_version": "point-observations/1.0", "basis": "keyframes",
            "frame_index_by_point": {point: sorted(indices) for point, indices in frames.items()}}


def sfm_observations(points, cameras):
    catalog = {c["image_id"]: i for i, c in enumerate(cameras)}
    return {"schema_version": "point-observations/1.0", "basis": "registered_frames",
            "frame_index_by_point": {str(pid): sorted({catalog[e.image_id] for e in point.track.elements
                                                       if e.image_id in catalog}) for pid, point in points}}
