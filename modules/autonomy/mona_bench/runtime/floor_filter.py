#!/usr/bin/env python3
"""Remove the floor from the planner's depth image.

The map frame comes from the first (hand-held, pitched) camera pose and a fixed 10 degree tilt
assumption, so the real floor lands in the map as a slope that rises into flight height a few
metres ahead and blocks every path. Each frame the floor plane is fitted in the camera frame and
pixels near or below it are set to 0, which Fast-Planner treats as free space up to its maximum
ray length (it also clears floor cells that were marked before).
"""
import numpy as np

UP = np.array([0.0, -1.0, 0.0])  # optical frame: y points down


def backproject(depth, rows, cols, fx, fy, cx, cy):
    z = depth[rows, cols]
    return np.stack([(cols - cx) / fx * z, (rows - cy) / fy * z, z], -1)


def fit_floor(points, rng, iterations=150, threshold=0.04, max_tilt_deg=35.0, height_range=(0.3, 3.0)):
    """RANSAC plane whose upward normal is within max_tilt of the camera's up axis and lies below it."""
    if len(points) < 50:
        return None
    min_up = np.cos(np.radians(max_tilt_deg))
    sample = points[rng.integers(0, len(points), min(len(points), 2000))]
    triples = rng.integers(0, len(sample), (iterations, 3))
    best = None
    for i, j, k in triples:
        a, b, c = sample[i], sample[j], sample[k]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        if normal @ UP < 0:
            normal = -normal
        height = -(normal @ a)  # signed distance from the camera (origin) down to the plane
        if normal @ UP < min_up or not height_range[0] <= height <= height_range[1]:
            continue
        count = int((np.abs(sample @ normal + height) < threshold).sum())
        if best is None or count > best[0]:
            best = (count, normal, height)
    if best is None:
        return None
    inliers = np.abs(points @ best[1] + best[2]) < threshold
    floor = points[inliers]
    centroid = floor.mean(0)
    normal = np.linalg.svd(floor - centroid, full_matrices=False)[2][-1]
    if normal @ UP < 0:
        normal = -normal
    height = -(normal @ centroid)
    if normal @ UP < min_up or not height_range[0] <= height <= height_range[1]:
        return None
    return normal, height, float(inliers.mean())


def remove_floor(depth, fx, fy, cx, cy, rng, band=0.15, min_inlier_ratio=0.15, stride=4, search_from=0.55):
    """Return (planning depth, stats). Pixels within `band` metres above the floor or below it become 0."""
    depth = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(depth) & (depth > 0)
    height_px, width_px = depth.shape
    rows, cols = np.mgrid[int(height_px * search_from):height_px:stride, 0:width_px:stride]
    keep = valid[rows, cols]
    candidates = backproject(depth, rows[keep], cols[keep], fx, fy, cx, cy)
    fit = fit_floor(candidates, rng)
    out = np.where(valid, depth, 0).astype(np.float32)
    if fit is None or fit[2] < min_inlier_ratio:
        return out, dict(floor=False)
    normal, height, ratio = fit
    # height above the floor = depth * (normal . ray(u, v)) + height; the dot product is separable in u and v
    ray_dot = ((normal[0] * (np.arange(width_px) - cx) / fx)[None, :]
               + (normal[1] * (np.arange(height_px) - cy) / fy)[:, None] + normal[2])
    above = out * ray_dot + height
    floor = valid & (above < band)
    out[floor] = 0
    tilt = float(np.degrees(np.arccos(np.clip(normal @ UP, -1, 1))))
    return out, dict(floor=True, height_m=float(height), tilt_deg=tilt, inlier_ratio=float(ratio),
                     removed_ratio=float(floor.sum() / max(valid.sum(), 1)))


def ros_main():
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    rospy.init_node("floor_filter")
    bridge = CvBridge()
    fx, fy = rospy.get_param("~fx"), rospy.get_param("~fy")
    cx, cy = rospy.get_param("~cx"), rospy.get_param("~cy")
    band = rospy.get_param("~band_m", 0.15)
    rng = np.random.default_rng(0)
    publisher = rospy.Publisher("~depth_out", Image, queue_size=2)

    def callback(msg):
        depth, stats = remove_floor(bridge.imgmsg_to_cv2(msg, "32FC1"), fx, fy, cx, cy, rng, band=band)
        out = bridge.cv2_to_imgmsg(depth, "32FC1")
        out.header = msg.header
        publisher.publish(out)
        if stats["floor"]:
            rospy.loginfo_throttle(5, "floor %.2fm below, tilt %.1f deg, removed %.0f%% of pixels",
                                   stats["height_m"], stats["tilt_deg"], 100 * stats["removed_ratio"])
        else:
            rospy.logwarn_throttle(5, "no floor plane found; depth passed through unchanged")
    rospy.Subscriber("~depth_in", Image, callback, queue_size=2, buff_size=4 * 1024 * 1024)
    rospy.spin()


if __name__ == "__main__":
    ros_main()
