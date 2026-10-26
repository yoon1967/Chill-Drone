"""Opt-in real-time MP4 -> live HTTP -> Docker SLAM -> viewer verification.

Run from the repository root. Uses the same start/frame/stop endpoints as capture.js;
it does not assert browser rendering or physical camera performance.
"""
from __future__ import annotations

import argparse
import hashlib
from http.client import HTTPConnection
import json
from pathlib import Path
import sys
import time
from urllib.parse import urlencode, urlsplit

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from modules.mapping.localization.orbslam3 import atomic_json


def request(connection, path, body=None, headers=None):
    headers = {"X-Mapping-Client": "1", **(headers or {})}
    if isinstance(body, dict):
        body = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    connection.request("GET" if body is None else "POST", path, body=body, headers=headers)
    response = connection.getresponse()
    payload = response.read()
    if response.status >= 400:
        raise RuntimeError(f"HTTP {response.status}: {payload.decode('utf-8', errors='replace')}")
    return json.loads(payload)


def replay(args):
    report = {"schema_version": "live-slam-replay-check/1.0", "status": "running",
              "input": "recorded_video_over_live_http", "replay_rate": 1,
              "source_sha256": hashlib.sha256(args.source.read_bytes()).hexdigest(),
              "sent_frames": 0, "source_frames_skipped": 0, "observations": [], "map_updates": []}
    capture = cv2.VideoCapture(str(args.source))
    address = urlsplit(args.url)
    connection = HTTPConnection(address.hostname, address.port, timeout=15)
    session = None
    stage = "input"
    started = time.monotonic()
    last_revision = 0

    def observe():
        nonlocal last_revision, stage
        stage = "state_transport"
        state = request(connection, "/api/state")
        if state.get("session_id") != session:
            raise RuntimeError("viewer_session_changed")
        report["observations"].append({"wall_seconds": round(time.monotonic() - started, 3),
            **{key: state.get(key) for key in ("status", "transport_active", "transport_received",
                "frames_received", "tracked_frames", "mapping_state", "revision", "points3D", "frame_drops")}})
        revision = state.get("revision", 0)
        if revision and revision != last_revision:
            stage = "map_transport"
            scene = request(connection, "/api/run?" + urlencode({"revision": f"{revision:06d}", "session": session}))
            if scene.get("revision") != revision or not scene.get("points") or not scene.get("cameras"):
                raise RuntimeError("invalid_map_response")
            report["map_updates"].append({"revision": revision, "points": len(scene["points"]),
                "keyframes": len(scene["cameras"]), "input_pts_s": scene["slam"]["timestamp_s"],
                "during_stream": bool(state.get("transport_active") and not state.get("transport_stopping"))})
            last_revision = revision
        if state.get("status") == "failed":
            stage = "slam"
            raise RuntimeError(state.get("last_error") or "tracker_failed")
        return state

    try:
        if not capture.isOpened():
            raise ValueError("video_unreadable")
        info = request(connection, "/api/live")
        camera = info.get("demo_camera")
        if not info.get("enabled") or not info.get("demo_available") or not camera:
            raise ValueError("server_requires_enabled_demo_with_calibration")
        report["camera"] = camera
        stage = "slam_startup"
        state = request(connection, "/api/live/start", {"width": camera["width"], "height": camera["height"],
            "fps": camera["fps"], "camera": camera})
        session = report["session_id"] = state["session_id"]
        deadline = time.monotonic() + 65
        while not observe().get("transport_ready"):
            if time.monotonic() > deadline:
                stage = "slam_startup"
                raise RuntimeError("tracker_start_timeout")
            time.sleep(.25)
        clock, next_sample = time.monotonic(), 0.
        first_pts, last_pts = None, -1.
        sequence = 0
        while True:
            stage = "input"
            success, image = capture.read()
            if not success:
                break
            pts = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000.
            if first_pts is None:
                first_pts = pts
            pts -= first_pts
            if pts <= last_pts:
                raise ValueError("source_pts_not_increasing")
            last_pts = pts
            elapsed = time.monotonic() - clock
            if sequence and pts < elapsed - 1 / camera["fps"]:
                # Match live presentation: skip stale decoded frames instead of slowing the recording.
                report["source_frames_skipped"] += 1
                sequence += 1
                continue
            time.sleep(max(0., pts - elapsed))
            image = cv2.resize(image, (camera["width"], camera["height"]))
            encoded, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not encoded:
                raise ValueError("jpeg_encode_failed")
            stage = "frame_transport"
            request(connection, "/api/live/frame?" + urlencode({"session": session}), jpeg.tobytes(),
                {"Content-Type": "image/jpeg", "X-Frame-Timestamp": str(pts), "X-Frame-Sequence": str(sequence)})
            sequence += 1
            report["sent_frames"] += 1
            if pts >= next_sample:
                observe()
                next_sample = pts + .5
        report["source_duration_s"] = last_pts
        report["source_frames"] = sequence
        report["send_wall_seconds"] = time.monotonic() - clock
        stage = "stop_transport"
        request(connection, "/api/live/stop", {"session_id": session})
        deadline = time.monotonic() + 45
        while True:
            state = observe()
            if not state.get("transport_active"):
                break
            if time.monotonic() > deadline:
                stage = "slam_shutdown"
                raise RuntimeError("tracker_stop_timeout")
            time.sleep(.25)
        report["final_state"] = state
        live_updates = [item for item in report["map_updates"] if item["during_stream"]]
        if len(live_updates) < 2 or not state.get("tracked_frames"):
            stage = "slam"
            raise RuntimeError("need_two_live_map_updates_and_tracked_frames")
        report["status"] = "passed"
    except Exception as error:
        report.update(status="failed", failure_stage=stage, error=str(error))
        if session:
            try:
                connection.close()
                request(connection, "/api/live/stop", {"session_id": session})
            except Exception:
                pass
    finally:
        capture.release()
        connection.close()
        report["wall_seconds"] = time.monotonic() - started
        atomic_json(args.report, report)
    print(json.dumps({key: report.get(key) for key in
        ("status", "session_id", "sent_frames", "source_duration_s", "send_wall_seconds", "failure_stage", "error")}, ensure_ascii=False))
    print(f"observed_map_updates={len(report['map_updates'])}; report={args.report}")
    return 0 if report["status"] == "passed" else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8791")
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    url = urlsplit(args.url)
    if url.scheme != "http" or url.hostname not in {"127.0.0.1", "localhost"} or url.path or url.query or url.fragment or url.username:
        parser.error("--url must be a loopback HTTP origin")
    if not args.source.is_file() or args.report.exists():
        parser.error("Use an existing video and a new report file")
    args.report.parent.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(1)
    raise SystemExit(replay(args))


if __name__ == "__main__":
    main()
