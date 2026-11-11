#!/usr/bin/env python3
"""Offline replay only: give Fast-Planner one goal ahead of the first SLAM body pose.

The goal is computed once from the first /sync/tf_odom (map -> base_link) and
published on the planner's manual-target topic. Nothing here talks to a drone.
The goal keeps the start height (the patched planner no longer forces z=1.0 m).
The planner silently ignores goals below -0.1 m, so those are rejected here.
"""
import argparse
import json
import math
from pathlib import Path
import time

PLANNER_MIN_GOAL_Z_M = -0.1  # kino_replan_fsm drops goals below this height


def forward_goal(position, quaternion, distance):
    """Point `distance` m along the horizontal body x axis at the start height.

    The quaternion is x, y, z, w."""
    if not (math.isfinite(distance) and 0 < distance <= 5):
        raise ValueError("goal distance must be in (0, 5] m")
    values = list(position) + list(quaternion)
    if len(values) != 7 or not all(math.isfinite(v) for v in values):
        raise ValueError("pose must be finite")
    x, y, z, w = quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-6:
        raise ValueError("quaternion must be non-zero")
    x, y, z, w = (v / norm for v in (x, y, z, w))
    # First column of the rotation matrix: body x axis expressed in map.
    ax, ay = 1 - 2*(y*y + z*z), 2*(x*y + w*z)
    horizontal = math.hypot(ax, ay)
    if horizontal < 0.1:
        raise ValueError("body x axis is nearly vertical; forward goal is undefined")
    if position[2] < PLANNER_MIN_GOAL_Z_M:
        raise ValueError("start height is below the planner's accepted goal height")
    return (position[0] + distance*ax/horizontal, position[1] + distance*ay/horizontal,
            position[2]), math.atan2(ay, ax)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--forward-m", type=float, required=True)
    parser.add_argument("--record", required=True)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()

    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path as PathMsg

    rospy.init_node("mona_replay_goal", anonymous=True)
    record = dict(forward_m=args.forward_m, topic="/waypoint_generator/waypoints",
                  source_pose_topic="/sync/tf_odom", goal_height="start_height",
                  flight_commands_sent=False, sent=False)
    try:
        print("waiting for /sync/tf_odom", flush=True)
        odom = rospy.wait_for_message("/sync/tf_odom", Odometry, timeout=args.timeout)
        print(f"first odometry at {odom.header.stamp.to_sec():.3f}", flush=True)
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        goal, heading = forward_goal((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), args.forward_m)
        publisher = rospy.Publisher("/waypoint_generator/waypoints", PathMsg, queue_size=1, latch=True)
        message = PathMsg()
        message.header.stamp = odom.header.stamp
        message.header.frame_id = "map"  # same frame as /sync/tf_odom
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = goal
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
        # Wall-clock wait for the planner's subscription; rospy.sleep follows the bag clock and can stall.
        time.sleep(0.5)
        publisher.publish(message)
        record.update(sent=True, odom_stamp_s=odom.header.stamp.to_sec(),
                      start_m=[p.x, p.y, p.z], body_heading_deg=math.degrees(heading), goal_m=list(goal))
        Path(args.record).write_text(json.dumps(record, indent=2))
        print(json.dumps(record))
        rospy.spin()  # keep the latched publisher alive until replay cleanup
    except Exception as error:  # record why no goal was sent; replay summary decides success
        record["error"] = f"{type(error).__name__}: {error}"
        Path(args.record).write_text(json.dumps(record, indent=2))
        raise


if __name__ == "__main__":
    main()
