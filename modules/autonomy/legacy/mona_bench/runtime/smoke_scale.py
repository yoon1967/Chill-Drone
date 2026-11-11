#!/usr/bin/env python3
"""Exercise compiled ground RANSAC and synchronized scale ROS nodes, using a synthetic floor."""
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Range, PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


def main():
    processes = []
    streams = []
    try:
        commands = [
            ["roscore"],
            ["rosrun", "scale_recovery", "ground_seg_planner", "_algorithm_name:=SC-Depth"],
            ["rosrun", "scale_recovery", "scale_recovery_planner.py"],
        ]
        for index, command in enumerate(commands):
            stream = open("/output/smoke-scale-%d.log" % index, "w")
            streams.append(stream)
            processes.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT))
            if index == 0:
                for _ in range(40):
                    if subprocess.run(["rosparam", "list"], stdout=subprocess.DEVNULL,
                                      stderr=subprocess.DEVNULL).returncode == 0:
                        break
                    time.sleep(0.25)
                else:
                    raise RuntimeError("ROS master did not become ready")
        rospy.init_node("mona_scale_smoke", disable_signals=True)
        bridge = CvBridge()
        received = []
        floor_stamps = []
        rospy.Subscriber("/sync/depth_metric", Image, received.append)
        rospy.Subscriber("/sync/pcl_ground", PointCloud2, lambda msg: floor_stamps.append(msg.header.stamp.to_nsec()))
        cloud_pub = rospy.Publisher("/sync/pcl_cam", PointCloud2, queue_size=2)
        tof_pub = rospy.Publisher("/tello/tof_btm", Range, queue_size=2)
        depth_pub = rospy.Publisher("/sync/depth", Image, queue_size=2)
        for _ in range(100):
            if all(p.get_num_connections() for p in (cloud_pub, tof_pub, depth_pub)):
                break
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("ROS component exited; see smoke-scale logs")
            time.sleep(0.1)
        else:
            raise RuntimeError("ROS input subscriptions were not established")
        points = [(float(x), 0.5, float(z)) for x in np.linspace(-1, 1, 64) for z in np.linspace(1, 2, 64)]
        sent_stamps = set()
        for index in range(40):
            stamp = rospy.Time.now()
            sent_stamps.add(stamp.to_nsec())
            header = Header(seq=index, stamp=stamp, frame_id="camera_link")
            cloud_pub.publish(create_cloud_xyz32(header, points))
            tof_pub.publish(Range(header=header, min_range=0.1, max_range=6.0, range=1.0))
            image = bridge.cv2_to_imgmsg(np.full((48, 64), 2.0, dtype=np.float32), "32FC1")
            image.header = header
            depth_pub.publish(image)
            time.sleep(0.15)
        time.sleep(0.5)
        values = [bridge.imgmsg_to_cv2(msg) for msg in received]
        result = dict(metric_frames=len(received), ground_frames=len(floor_stamps),
                      timestamps_preserved=bool(floor_stamps) and set(floor_stamps).issubset(sent_stamps),
                      finite_positive=bool(values) and all(np.all(np.isfinite(v) & (v > 0)) for v in values),
                      flight_commands_sent=False)
        # Upstream metric_depth_callback copies stamp, not seq. Test the late timestamp instead.
        late_stamp = sorted(sent_stamps)[30]
        result["output_after_frame_30"] = any(msg.header.stamp.to_nsec() >= late_stamp for msg in received)
        result["ok"] = (result["metric_frames"] >= 15 and result["timestamps_preserved"]
                        and result["output_after_frame_30"] and result["finite_positive"])
        Path("/output/smoke-scale-result.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
        return 0 if result["ok"] else 1
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        for stream in streams:
            stream.close()


if __name__ == "__main__":
    raise SystemExit(main())
