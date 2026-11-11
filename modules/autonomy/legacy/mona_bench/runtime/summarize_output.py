#!/usr/bin/env python3
"""Describe which navigation stages actually produced messages during replay."""
import json
from pathlib import Path
import sys

import rosbag

STAGES = {"metric_depth": "/sync/depth_metric", "slam_pose": "/orb_slam2_rgbd/pose",
          "trajectory": "/planning/bspline", "position_command": "/planning/pos_cmd",
          "candidate_velocity": "/mona/candidate_cmd_vel"}


LIGHT_SKIPPED = {"metric_depth"}  # not recorded in light mode; SLAM poses need it, so they stand in


def summarize(path, goal_record=None, record_mode="full"):
    result = dict(stages={}, flight_commands_sent=False, record=record_mode,
                  scope="offline_replay_outputs; message presence does not establish accuracy or collision avoidance")
    with rosbag.Bag(str(path), "r") as bag:
        info = bag.get_type_and_topic_info().topics
        for stage, topic in STAGES.items():
            if record_mode == "light" and stage in LIGHT_SKIPPED:
                result["stages"][stage] = dict(topic=topic, messages=None, recorded=False)
                continue
            entry = info.get(topic)
            result["stages"][stage] = dict(topic=topic, messages=entry.message_count if entry else 0)
    result["all_stages_observed"] = all(v["messages"] for v in result["stages"].values() if v.get("recorded", True))
    if goal_record is not None:
        # A missing or unsent goal is reported, never treated as observed planning input.
        goal = json.loads(goal_record.read_text()) if goal_record.exists() else dict(sent=False, error="goal record missing")
        result["goal"] = goal
    return result


if __name__ == "__main__":
    goal = Path(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[3] != "-" else None
    result = summarize(Path(sys.argv[1]), goal, sys.argv[4] if len(sys.argv) > 4 else "full")
    payload = json.dumps(result, indent=2)
    Path(sys.argv[2]).write_text(payload)
    print(payload)
