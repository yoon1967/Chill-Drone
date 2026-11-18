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


class HoldBuffer:
    """Keeps planner depth back for delay_s so frames taken while SLAM jumped can still be dropped.

    The bridge sees a jump only when SLAM's pose for that frame arrives, after the depth went on; the planner map would
    already hold the frame at the wrong pose (sim 11/09: a 0.55 m jump while looking around left no free start cell,
    "open set empty" until no_progress). While a hold is set (/mona/slam_hold > 0: ROS stamp from which SLAM is not
    trusted) frames stamped at or after it are dropped, buffered or new; the map simply does not update until the hold
    ends. A hold set later than delay_s after the bad frame (drift) only stops what follows."""

    def __init__(self, delay_s=0.3):
        self.delay_s = delay_s
        self.items = []          # (receive time, stamp s, item)
        self.hold_since = None
        self.dropped = 0

    def set_hold(self, since_stamp):
        self.hold_since = since_stamp if since_stamp and since_stamp > 0 else None
        if self.hold_since is not None:
            keep = [i for i in self.items if i[1] < self.hold_since]
            self.dropped += len(self.items) - len(keep)
            self.items = keep

    def push(self, t, stamp, item):
        if self.hold_since is not None and stamp >= self.hold_since:
            self.dropped += 1
            return
        self.items.append((t, stamp, item))

    def ready(self, t):
        out = [i[2] for i in self.items if t - i[0] >= self.delay_s]
        self.items = [i for i in self.items if t - i[0] < self.delay_s]
        return out


def ros_main():
    import time

    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float64
    rospy.init_node("floor_filter")
    bridge = CvBridge()
    fx, fy = rospy.get_param("~fx"), rospy.get_param("~fy")
    cx, cy = rospy.get_param("~cx"), rospy.get_param("~cy")
    band = rospy.get_param("~band_m", 0.15)
    rng = np.random.default_rng(0)
    publisher = rospy.Publisher("~depth_out", Image, queue_size=5)
    held = HoldBuffer(float(rospy.get_param("~hold_delay_s", 0.3)))

    def on_hold(msg):
        held.set_hold(msg.data)
        rospy.loginfo("SLAM hold %s: planner depth %s (dropped so far %d)",
                      "from %.2f" % msg.data if msg.data > 0 else "off",
                      "paused" if msg.data > 0 else "resumed", held.dropped)

    def flush(_event):
        for out in held.ready(time.monotonic()):
            publisher.publish(out)

    def callback(msg):
        depth, stats = remove_floor(bridge.imgmsg_to_cv2(msg, "32FC1"), fx, fy, cx, cy, rng, band=band)
        out = bridge.cv2_to_imgmsg(depth, "32FC1")
        out.header = msg.header
        held.push(time.monotonic(), msg.header.stamp.to_sec(), out)
        if stats["floor"]:
            rospy.loginfo_throttle(5, "floor %.2fm below, tilt %.1f deg, removed %.0f%% of pixels",
                                   stats["height_m"], stats["tilt_deg"], 100 * stats["removed_ratio"])
        else:
            rospy.logwarn_throttle(5, "no floor plane found; depth passed through unchanged")
    rospy.Subscriber("~depth_in", Image, callback, queue_size=2, buff_size=4 * 1024 * 1024)
    rospy.Subscriber("/mona/slam_hold", Float64, on_hold, queue_size=5)
    rospy.Timer(rospy.Duration(0.02), flush)
    rospy.spin()


if __name__ == "__main__":
    ros_main()
