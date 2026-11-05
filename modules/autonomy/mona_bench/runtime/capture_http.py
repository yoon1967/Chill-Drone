#!/usr/bin/env python3
"""Record existing web video and bottom ToF using GET only; never operates the drone."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time
from urllib.request import urlopen

import cv2
from cv_bridge import CvBridge
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Range

from inspect_bag import inspect, RGB, TOF


def fetch(base, resource):
    if resource not in ("/api/status", "/snapshot.jpg"):
        raise ValueError("only read-only capture resources are allowed")
    with urlopen(base.rstrip("/") + resource, timeout=2) as response:
        return response.read()


def valid_sample(status):
    """Return independently timestamped image/state samples, or an explicit rejection."""
    state = status.get("state") or {}
    video = status.get("video") or {}
    if state.get("valid") is not True:
        raise ValueError("state_invalid")
    for label, age in (("state", state.get("age_s")), ("video", video.get("frame_age_s"))):
        if not isinstance(age, (int, float)) or not math.isfinite(age) or not 0 <= age <= 0.3:
            raise ValueError(label + "_stale_or_unknown")
    raw_height = (state.get("raw") or {}).get("tof")
    if not isinstance(raw_height, (int, float)) or not math.isfinite(raw_height) or not 10 < raw_height < 600:
        raise ValueError("bottom_tof_invalid")
    state_stamp = datetime.fromisoformat(state["received_at"]).timestamp()
    image_stamp = datetime.fromisoformat(status["now"]).timestamp() - video["frame_age_s"]
    if not all(math.isfinite(t) and t > 0 for t in (state_stamp, image_stamp)):
        raise ValueError("invalid_timestamp")
    return state_stamp, image_stamp, raw_height / 100.0


def capture(base, seconds, hz, output, stop=None):
    """Record until `seconds` elapse or `stop` (threading.Event) is set; always finalize."""
    stop = stop or threading.Event()
    output.mkdir(parents=True, exist_ok=False)
    rejected = Counter()
    bridge = CvBridge()
    frames = heights = 0
    last_digest = last_state_stamp = None
    last_image_stamp = 0
    source_modes = set()
    started_utc = datetime.now(timezone.utc).isoformat()
    started = time.monotonic()
    with rosbag.Bag(str(output / "input.bag"), "w") as bag:
        while not stop.is_set() and time.monotonic() - started < seconds:
            tick = time.monotonic()
            try:
                status = json.loads(fetch(base, "/api/status"))
                state_stamp, image_stamp, height = valid_sample(status)
                payload = fetch(base, "/snapshot.jpg")
                if time.monotonic() - tick > 0.2:
                    raise ValueError("http_roundtrip_over_0.2s")
                digest = hashlib.sha256(payload).hexdigest()
                image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError("invalid_jpeg")
                source_modes.add(status.get("mode", "unknown"))
                if state_stamp != last_state_stamp:
                    msg = Range()
                    msg.header.stamp = rospy.Time.from_sec(state_stamp)
                    msg.header.frame_id = "base_link"
                    msg.radiation_type = Range.INFRARED
                    msg.min_range, msg.max_range, msg.range = 0.1, 6.0, height
                    bag.write(TOF, msg, msg.header.stamp)
                    heights += 1
                    last_state_stamp = state_stamp
                if digest != last_digest and image_stamp > last_image_stamp:
                    msg = bridge.cv2_to_imgmsg(image, "bgr8")
                    msg.header.stamp = rospy.Time.from_sec(image_stamp)
                    msg.header.frame_id = "camera_link"
                    bag.write(RGB, msg, msg.header.stamp)
                    frames += 1
                    last_digest, last_image_stamp = digest, image_stamp
                else:
                    rejected["duplicate_or_nonincreasing_frame"] += 1
            except (OSError, ValueError, KeyError, TypeError) as error:
                rejected[str(error)] += 1
            stop.wait(max(0, 1 / hz - (time.monotonic() - tick)))
    result = inspect(output / "input.bag")
    manifest = dict(started_utc=started_utc, source_url=base,
                    duration_s=time.monotonic() - started, frames=frames, bottom_tof_samples=heights,
                    stop_reason="stop_requested" if stop.is_set() else "time_limit",
                    source_modes=sorted(source_modes), rejected=dict(rejected), read_only=True,
                    timestamp_source="web server receive time; RGB age estimated from separate status request",
                    limitations=["HTTP status and JPEG are not atomic; up to 0.2s request skew",
                                 "identical JPEGs are deduplicated; this may drop truly stationary frames",
                                 "grounded zero/invalid bottom ToF is excluded; no height is synthesized"],
                    input_quality=result)
    (output / "capture.json").write_text(json.dumps(manifest, indent=2))
    print(json.dumps(dict(output=str(output), **manifest), indent=2))
    return 0 if result["ok"] else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://host.docker.internal:8010")
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--hz", type=float, default=10)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stop-on-enter", action="store_true",
                        help="finish and save when Enter is pressed (needs docker run -i)")
    args = parser.parse_args()
    if not 0 < args.seconds <= 3600 or not 0 < args.hz <= 30:
        parser.error("seconds must be in (0,3600], hz in (0,30]")
    stop = threading.Event()
    # Ctrl+C / docker stop end the loop; the bag, quality check and capture.json are still written.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    if args.stop_on_enter:
        def wait_for_enter():
            if sys.stdin.readline():  # EOF ('') is not a stop request
                stop.set()
        threading.Thread(target=wait_for_enter, daemon=True).start()
        print(f"Recording up to {args.seconds:.0f}s. Press Enter to stop and save.", flush=True)
    raise SystemExit(capture(args.url, args.seconds, args.hz, args.output, stop))
