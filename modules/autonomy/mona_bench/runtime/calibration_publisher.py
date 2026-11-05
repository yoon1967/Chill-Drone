#!/usr/bin/env python3
"""Provide latched camera calibration before the first replay image or clock tick."""
import math

import rospy
from sensor_msgs.msg import CameraInfo
import yaml


def load_calibration(path, virtual_baseline_m=0.0):
    """Load the 640x480 camera; optionally add ORB-SLAM2's RGB-D virtual baseline.

    orb_slam2_ros reads CameraInfo.P[3] as bf (baseline x fx). A monocular P[3]
    of 0 makes every depth "far" (threshold 0) and drops depth from tracking.
    """
    with open(path) as stream:
        data = yaml.safe_load(stream)
    message = CameraInfo()
    message.width = int(data["image_width"])
    message.height = int(data["image_height"])
    if message.width != 640 or message.height != 480:
        raise ValueError("calibration must match the pipeline's 640x480 images")
    for field, key, count in (("K", "camera_matrix", 9), ("R", "rectification_matrix", 9),
                               ("P", "projection_matrix", 12)):
        values = data[key]["data"]
        if len(values) != count or not all(math.isfinite(float(v)) for v in values):
            raise ValueError("invalid " + key)
        setattr(message, field, values)
    message.D = data["distortion_coefficients"]["data"]
    if len(message.D) < 5 or not all(math.isfinite(float(v)) for v in message.D):
        raise ValueError("expected five finite distortion coefficients")
    if message.K[0] <= 0 or message.K[4] <= 0:
        raise ValueError("focal lengths must be positive")
    if not (math.isfinite(virtual_baseline_m) and 0 <= virtual_baseline_m <= 0.5):
        raise ValueError("virtual baseline must be within [0, 0.5] m")
    if virtual_baseline_m:
        message.P = list(message.P)
        message.P[3] = virtual_baseline_m * message.K[0]
    message.distortion_model = data["distortion_model"]
    message.header.frame_id = "camera_link"
    return message


if __name__ == "__main__":
    rospy.init_node("camera_calibration")
    baseline = float(rospy.get_param("~orb_virtual_baseline_m", 0.0))
    message = load_calibration(rospy.get_param("~camera_info"), baseline)
    # Stamp zero is intentional: this is static calibration for SLAM startup.
    # Keep it separate from timestamped CameraInfo used by depth_image_proc.
    publisher = rospy.Publisher("/mona/camera_info", CameraInfo, queue_size=1, latch=True)
    publisher.publish(message)
    rospy.loginfo("Latched 640x480 calibration published before replay frames; ORB bf=%.2f (baseline %.3f m)",
                  message.P[3], baseline)
    rospy.spin()
