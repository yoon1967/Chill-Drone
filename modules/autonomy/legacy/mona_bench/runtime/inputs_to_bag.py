#!/usr/bin/env python3
"""Turn a live run's saved inputs into a replay bag: `inputs_to_bag.py <live run dir> [out.bag]`.

gateway_bridge.py (`~inputs_dir`) keeps the gateway JPEG bytes as `frames/<stamp ns>.jpg` and the bottom ToF as
`tof.csv` (`<stamp ns>,<m>`), with the same ROS stamps it published. This writes them as `/tello/image_raw`
(bgr8) and `/tello/tof_btm` (Range, m), the replay input format, so `run.ps1 -Action Replay` can run the
real flight's images again (planner or depth changes checked offline). Nothing is sent to a drone.
"""
import sys
from pathlib import Path

RGB, TOF = "/tello/image_raw", "/tello/tof_btm"
EDGE_NS = 500_000_000   # keep frames up to 0.5 s around the airborne (valid bottom ToF) span


def airborne_frames(frames, tof):
    """Frames inside the bottom-ToF span. On the ground ToF is invalid (<= 10 cm) and video keeps running after
    landing; such frames have no height for the scale step, and the replay input check needs ToF next to frames."""
    if not tof:
        return []
    lo, hi = tof[0][0] - EDGE_NS, tof[-1][0] + EDGE_NS
    return [f for f in frames if lo <= f[0] <= hi]


def read_inputs(run_dir):
    """-> (sorted [(stamp_ns, jpeg path)], sorted [(stamp_ns, height_m)])."""
    run_dir = Path(run_dir)
    frames = sorted((int(p.stem), p) for p in (run_dir / "inputs" / "frames").glob("*.jpg") if p.stem.isdigit())
    tof = []
    tof_file = run_dir / "inputs" / "tof.csv"
    if tof_file.exists():
        for line in tof_file.read_text().splitlines():
            parts = line.split(",")
            if len(parts) == 2:
                try:
                    tof.append((int(parts[0]), float(parts[1])))
                except ValueError:
                    continue
    tof.sort()
    return frames, tof


def write_bag(run_dir, out_path):
    import cv2
    import numpy as np
    import rosbag
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Range

    all_frames, tof = read_inputs(run_dir)
    frames = airborne_frames(all_frames, tof)
    if not frames or not tof:
        raise SystemExit(f"no saved inputs in {run_dir}/inputs (frames {len(all_frames)}, in ToF span "
                         f"{len(frames)}, tof {len(tof)})")
    bridge = CvBridge()
    written = bad = 0
    with rosbag.Bag(str(out_path), "w", compression="lz4") as bag:
        for stamp, height in tof:
            msg = Range()
            msg.header.stamp = rospy.Time(nsecs=stamp)
            msg.header.frame_id = "base_link"
            msg.radiation_type = Range.INFRARED
            msg.min_range, msg.max_range, msg.range = 0.1, 6.0, height
            bag.write(TOF, msg, msg.header.stamp)
        for stamp, path in frames:
            image = cv2.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                bad += 1
                continue
            msg = bridge.cv2_to_imgmsg(image, "bgr8")
            msg.header.stamp = rospy.Time(nsecs=stamp)
            msg.header.frame_id = "camera_link"
            bag.write(RGB, msg, msg.header.stamp)
            written += 1
    span = (frames[-1][0] - frames[0][0]) / 1e9
    print(f"{out_path}: {written} frames ({bad} unreadable, {len(all_frames) - len(frames)} outside the ToF span "
          f"left out), {len(tof)} ToF, {span:.1f} s")
    return written, len(tof)


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        raise SystemExit(__doc__)
    run = Path(sys.argv[1])
    write_bag(run, Path(sys.argv[2]) if len(sys.argv) == 3 else run / "input.bag")
