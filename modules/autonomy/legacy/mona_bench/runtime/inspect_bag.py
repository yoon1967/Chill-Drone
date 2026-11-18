#!/usr/bin/env python3
"""Validate the RGB and bottom-ToF inputs used by MoNA-Bench, without ROS master."""
import argparse
import bisect
import json
import math
from pathlib import Path

import rosbag

RGB = "/tello/image_raw"
TOF = "/tello/tof_btm"


def inspect(path):
    stamps = {RGB: [], TOF: []}
    invalid = {RGB: 0, TOF: 0}
    errors = []
    with rosbag.Bag(str(path), "r") as bag:
        for topic, msg, _ in bag.read_messages(topics=[RGB, TOF]):
            stamp = msg.header.stamp.to_sec() if hasattr(msg, "header") else 0
            valid = math.isfinite(stamp) and stamp > 0
            if topic == RGB:
                valid = (valid and getattr(msg, "_type", None) == "sensor_msgs/Image"
                         and msg.encoding in ("rgb8", "bgr8") and msg.width >= 32 and msg.height >= 32
                         and msg.step >= msg.width * 3 and len(msg.data) == msg.step * msg.height)
            else:
                valid = (valid and getattr(msg, "_type", None) == "sensor_msgs/Range"
                         and math.isfinite(msg.range) and 0.1 < msg.range < 6
                         and msg.min_range <= msg.range <= msg.max_range)
            if valid:
                stamps[topic].append(stamp)
            else:
                invalid[topic] += 1
    summaries = {}
    for topic, times in stamps.items():
        increasing = all(b > a for a, b in zip(times, times[1:]))
        duration = max(times) - min(times) if times else 0
        summaries[topic] = dict(valid_messages=len(times), invalid_messages=invalid[topic],
                                timestamps_strictly_increasing=increasing, duration_s=duration,
                                rate_hz=(len(times) - 1) / duration if duration else 0)
        if len(times) < 10:
            errors.append(topic + ": fewer than 10 valid messages")
        if invalid[topic]:
            errors.append(topic + ": invalid message content or timestamp")
        if not increasing:
            errors.append(topic + ": repeated or regressing timestamps")
    heights = sorted(stamps[TOF])
    matched = 0
    for stamp in stamps[RGB]:
        index = bisect.bisect_left(heights, stamp)
        neighbors = heights[max(0, index - 1):index + 1]
        matched += bool(neighbors) and min(abs(stamp - t) for t in neighbors) <= 0.1
    ratio = matched / len(stamps[RGB]) if stamps[RGB] else 0
    if ratio < 0.8:
        errors.append("fewer than 80% of RGB frames have bottom-ToF within 0.1 seconds")
    warnings = [topic + ": input rate below 5 Hz" for topic, values in summaries.items()
                if values["valid_messages"] and values["rate_hz"] < 5]
    return dict(ok=not errors, topics=summaries, rgb_with_bottom_tof_ratio=ratio,
                errors=errors, warnings=warnings, scope="input_quality_only", flight_commands_sent=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = inspect(args.bag)
    payload = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(payload)
    print(payload)
    raise SystemExit(0 if result["ok"] else 2)
