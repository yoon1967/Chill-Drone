"""Run official ORB-SLAM3 in Docker and publish its native sparse map to the console.

COLMAP is not used for tracking, matching, triangulation or optimization here.
Only camera intrinsics from a prior result may be reused as a self-estimated prior.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import subprocess
import time
import uuid

import cv2
import numpy as np

if __package__:
    from .observations import native_observations
else:
    from observations import native_observations

IMAGE = "ssafy-orbslam3:local"
DISPLAY = np.diag([1., -1., -1.])


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    replace_file(temporary, path)


def replace_file(source, target):
    for attempt in range(40):
        try:
            source.replace(target)
            return
        except PermissionError:
            if attempt == 39:
                raise
            time.sleep(.005)


def read_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def calibration(args):
    capture = cv2.VideoCapture(str(args.source))
    success, image = capture.read()
    fps = capture.get(cv2.CAP_PROP_FPS)
    capture.release()
    if not success or not np.isfinite(fps) or fps <= 0:
        raise ValueError("video_unreadable_or_unknown_frame_rate")
    original_height, original_width = image.shape[:2]
    if args.camera:
        camera = json.loads(args.camera.read_text(encoding="utf-8"))
    else:
        scene = json.loads((args.calibration_result / "scene.json").read_text(encoding="utf-8"))
        source = scene["cameras"][0]
        p = source["params"]
        if source["model"] == "SIMPLE_RADIAL":
            fx, fy, cx, cy, distortion = p[0], p[0], p[1], p[2], [p[3], 0., 0., 0.]
        elif source["model"] == "PINHOLE":
            fx, fy, cx, cy, distortion = *p, [0., 0., 0., 0.]
        elif source["model"] == "OPENCV":
            fx, fy, cx, cy, distortion = *p[:4], p[4:8]
        else:
            raise ValueError("camera_requires_SIMPLE_RADIAL_PINHOLE_or_OPENCV")
        camera = dict(width=source["width"], height=source["height"], fx=fx, fy=fy,
                      cx=cx, cy=cy, distortion=distortion, calibration_status="prior_self_estimated")
    required = ("width", "height", "fx", "fy", "cx", "cy")
    if not all(np.isfinite(camera[k]) for k in required) or min(camera[k] for k in ("width", "height", "fx", "fy")) <= 0:
        raise ValueError("invalid_camera_intrinsics")
    if abs(camera["width"] / camera["height"] - original_width / original_height) > .01:
        raise ValueError("camera_orientation_or_crop_does_not_match_video")
    width = min(args.width, original_width)
    height = round(original_height * width / original_width)
    sx, sy = width / camera["width"], height / camera["height"]
    for name in ("fx", "cx"):
        camera[name] *= sx
    for name in ("fy", "cy"):
        camera[name] *= sy
    camera.update(width=width, height=height, fps=fps)
    d = camera.get("distortion", [0.] * 4)
    if len(d) != 4 or not np.isfinite(d).all():
        raise ValueError("camera_distortion_requires_k1_k2_p1_p2")
    camera["distortion"] = d
    camera.setdefault("calibration_status", "user_supplied; accuracy_not_independently_verified")
    return camera


def write_settings(path, camera, features):
    d = camera["distortion"]
    values = {"Camera.type": '"PinHole"', "Camera.fx": camera["fx"], "Camera.fy": camera["fy"],
        "Camera.cx": camera["cx"], "Camera.cy": camera["cy"], "Camera.k1": d[0], "Camera.k2": d[1],
        "Camera.p1": d[2], "Camera.p2": d[3], "Camera.width": camera["width"], "Camera.height": camera["height"],
        "Camera.fps": camera["fps"], "Camera.RGB": 0, "ORBextractor.nFeatures": features,
        "ORBextractor.scaleFactor": 1.2, "ORBextractor.nLevels": 8,
        "ORBextractor.iniThFAST": 20, "ORBextractor.minThFAST": 7,
        "Viewer.KeyFrameSize": .05, "Viewer.KeyFrameLineWidth": 1., "Viewer.GraphLineWidth": .9,
        "Viewer.PointSize": 2., "Viewer.CameraSize": .08, "Viewer.CameraLineWidth": 3.,
        "Viewer.ViewpointX": 0., "Viewer.ViewpointY": -.7, "Viewer.ViewpointZ": -1.8, "Viewer.ViewpointF": 500.}
    # ORB's OpenCV YAML reader requires real scalars, even for zero distortion.
    for key in ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "fps"):
        values["Camera." + key] = float(values["Camera." + key])
    path.write_text("%YAML:1.0\n" + "\n".join(f"{k}: {v}" for k, v in values.items()) + "\n", encoding="utf-8")


def camera_matrix(camera):
    return np.array([[camera["fx"], 0., camera["cx"]], [0., camera["fy"], camera["cy"]], [0., 0., 1.]])


def convert(native, camera, progress, session):
    """One atomic map revision: all poses and points share the same native map snapshot."""
    native_points = {p["id"]: p for p in native["points"]}
    colors, errors = {}, defaultdict(list)
    cameras = []
    K = camera_matrix(camera)
    distortion = np.array(camera["distortion"])
    for keyframe in native["keyframes"]:
        T = np.array(keyframe["T_cw"])
        if T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1]):
            raise ValueError("invalid_native_camera_pose")
        R, t = T[:3, :3], T[:3, 3]
        name = keyframe["frame_id"]
        if not re.fullmatch(r"orb_\d{8}\.jpg", name):
            raise ValueError("invalid_native_frame_name")
        cameras.append({"image_id": keyframe["id"], "frame_id": name, "timestamp_s": keyframe["timestamp_s"],
            "source_pts_s": keyframe["timestamp_s"] if keyframe["timestamp_source"] in {"source_pts", "browser_media_pts"} else None,
            "timestamp_source": keyframe["timestamp_source"], "position": (DISPLAY @ (-R.T @ t)).tolist(),
            "world_from_camera_rotation": (DISPLAY @ R.T).tolist(),
            "cam_from_world_rotation_raw": R.tolist(), "cam_from_world_translation_raw": t.tolist(),
            "camera_id": 1, "model": "OPENCV", "width": camera["width"], "height": camera["height"],
            "params": [camera[k] for k in ("fx", "fy", "cx", "cy")] + camera["distortion"]})
        observations = [o for o in keyframe["observations"] if o["point_id"] in native_points]
        if not observations:
            continue
        xyz = np.array([native_points[o["point_id"]]["xyz"] for o in observations])
        projected, _ = cv2.projectPoints(xyz, cv2.Rodrigues(R)[0], t, K, distortion)
        residual = np.linalg.norm(projected.reshape(-1, 2) - np.array([o["xy"] for o in observations]), axis=1)
        image = cv2.imread(str(session / "frames" / name))
        for obs, error in zip(observations, residual):
            errors[obs["point_id"]].append(float(error))
            x, y = np.rint(obs["xy"]).astype(int)
            if obs["point_id"] not in colors and image is not None and 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                colors[obs["point_id"]] = image[y, x, ::-1].tolist()
    points = []
    for p in native_points.values():
        if not errors[p["id"]] or not np.isfinite(p["xyz"]).all():
            continue
        points.append([p["id"], *(DISPLAY @ p["xyz"]).tolist(), *colors.get(p["id"], [170, 190, 190]),
                       float(np.mean(errors[p["id"]])), p["observations"]])
    if not points or not cameras:
        raise ValueError("empty_native_map")
    xyz = np.array([p[1:4] for p in points])
    count = progress.get("frames_received", 0)
    result = {"schema_version": "phone-sfm-result/1.2", "pose_source": "orb_slam3", "input_run_id": session.name,
        "created_utc": datetime.now(timezone.utc).isoformat(), "revision": native["revision"],
        "segment_id": native["map_id"], "gauge_aligned": True,
        "coordinate_system": {"native": "ORB-SLAM3 world; camera x-right y-down z-forward", "display_rotation": DISPLAY.tolist()},
        "scale": {"status": "arbitrary", "unit": "arbitrary", "reason": "monocular_without_metric_sensor"},
        "points": points, "cameras": cameras,
        "quality": {"registered_frames": len(cameras), "input_frames": count,
            "registration_ratio": len(cameras) / max(count, 1), "excluded_frames": [], "points3D": len(points),
            "mean_reprojection_error_px": float(np.mean([p[7] for p in points])), "result_status": "partial",
            "processing_seconds": progress.get("elapsed_seconds", 0),
            "bounds": {"min": xyz.min(axis=0).tolist(), "max": xyz.max(axis=0).tolist()}},
        "slam": {"map_id": native["map_id"], "map_change_index": native["map_change_index"],
            "timestamp_s": native.get("timestamp_s", max(c["timestamp_s"] for c in cameras)),
            "timestamp_basis": "snapshot_input_pts" if "timestamp_s" in native else "latest_surviving_keyframe_estimate",
            "big_change_index": native["big_change_index"], "final": native["final"],
            "calibration_status": camera["calibration_status"], "keyframes": len(cameras)},
        "limitations": ["단안 임의 단위 · 실제 치수 미검증", "ORB-SLAM3의 희소 지도 · 밀집 표면 복원은 별도 단계"]}
    result["point_observations"] = native_observations(result, native)
    return result


def preview(session, progress, camera):
    name = progress.get("frame_id", "")
    if not re.fullmatch(r"orb_\d{8}\.jpg", name):
        return
    image = cv2.imread(str(session / "previews" / name))
    if image is None:
        return
    image = cv2.undistort(image, camera_matrix(camera), np.array(camera["distortion"]))
    for i, (x, y, point_id, same) in enumerate(progress.get("features", [])):
        x, y = round(x), round(y)
        color = (70, 230, 255) if same else (120, 240, 110)
        cv2.circle(image, (x, y), 2, color, 1, cv2.LINE_AA)
        if i % 15 == 0:
            cv2.putText(image, str(point_id), (x + 3, y - 3), cv2.FONT_HERSHEY_SIMPLEX, .3, color, 1, cv2.LINE_AA)
    temporary = session / "preview.tmp.jpg"
    cv2.imwrite(str(temporary), image)
    replace_file(temporary, session / "preview.jpg")


def timeline_entry(scene):
    return {"revision": scene["revision"], "map_id": scene["segment_id"],
            "timestamp_s": scene["slam"]["timestamp_s"], "timestamp_basis": scene["slam"]["timestamp_basis"],
            "points3D": len(scene["points"]), "keyframes": len(scene["cameras"]), "final": scene["slam"]["final"]}


class SessionPublisher:
    """Publish the same coherent snapshots for file input and the live camera transport."""
    def __init__(self, session, camera):
        self.session, self.camera = session, camera
        self.revision, self.last_frame = 0, None
        self.entries = []
        self.state = {"status": "starting", "backend": "orb_slam3_monocular", "revision": 0,
                      "session_id": session.name, "map_available": False, "frame_drops": 0}
        atomic_json(session / "state.json", self.state)

    def update(self, extra=None):
        progress = read_json(self.session / "tracking.json")
        if progress:
            self.state.update({k: v for k, v in progress.items() if k not in {"features", "T_cw", "revision"}})
            self.state["mapping_state"] = {2: "tracking", 3: "recently_lost", 4: "lost"}.get(progress.get("tracking_state"), "initializing")
            if progress.get("frame_id") != self.last_frame:
                preview(self.session, progress, self.camera)
                self.last_frame = progress.get("frame_id")
            native = read_json(self.session / "native.json")
            if native and native["revision"] != self.revision:
                scene = convert(native, self.camera, progress, self.session)
                directory = self.session / "revisions" / f"{native['revision']:06d}"
                directory.mkdir(exist_ok=True)
                atomic_json(directory / "scene.json", scene)
                atomic_json(directory / "local_assets.json", {"images_root": str(self.session / "frames"),
                    "frames": [c["frame_id"] for c in scene["cameras"]]})
                atomic_json(directory / "status.json", {"status": "partial"})
                self.revision = native["revision"]
                self.entries.append(timeline_entry(scene))
                atomic_json(self.session / "timeline.json", {"schema_version": "orb-map-timeline/1.0",
                    "session_id": self.session.name, "entries": self.entries})
                self.state.update(revision=self.revision, map_available=True, revision_path=f"revisions/{self.revision:06d}",
                                  segment_id=native["map_id"], points3D=len(scene["points"]), keyframes=len(scene["cameras"]))
        self.state.update(extra or {})
        atomic_json(self.session / "state.json", self.state)


def run(args):
    if args.session.exists():
        raise ValueError("use_a_new_session_directory")
    camera = calibration(args)
    if args.build:
        subprocess.run(["docker", "build", "-t", args.image, str(Path(__file__).with_name("orbslam3"))], check=True)
    image_id = subprocess.check_output(["docker", "image", "inspect", "--format", "{{.Id}}", args.image], text=True).strip()
    args.session.mkdir(parents=True)
    (args.session / "revisions").mkdir()
    write_settings(args.session / "camera.yaml", camera, args.features)
    atomic_json(args.session / "camera.json", camera)
    atomic_json(args.session / "settings.json", {"backend": "orb_slam3_monocular", "image": args.image, "image_id": image_id,
        "upstream_revision": "4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4", "camera": camera,
        "replay_rate": args.replay_rate, "frame_policy": "sequential; keyframes_selected_by_ORB_SLAM3",
        "input": "local_file", "scale": "arbitrary", "dense_surface": False})
    name = "ssafy-orb-" + uuid.uuid4().hex[:12]
    command = ["docker", "run", "--rm", "--name", name, "--network", "none", "--cap-drop", "ALL",
        "--security-opt", "no-new-privileges", "--cpus", str(args.cpus), "--memory", "8g",
        "--mount", f"type=bind,source={args.source.parent},target=/input,readonly",
        "--mount", f"type=bind,source={args.session},target=/output", args.image,
        "/input/" + args.source.name, "/output/camera.yaml", "/output", str(args.replay_rate), str(args.max_seconds)]
    publisher = SessionPublisher(args.session, camera)
    state = publisher.state
    with (args.session / "orb.log").open("w", encoding="utf-8") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            while True:
                publisher.update()
                if process.poll() is not None:
                    # Consume the final atomic native snapshot in the next iteration.
                    latest = read_json(args.session / "native.json")
                    if latest and latest["revision"] != publisher.revision:
                        continue
                    break
                time.sleep(.15)
        except BaseException:
            subprocess.run(["docker", "stop", "-t", "3", name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            process.wait(timeout=15)
            state.update(status="stopped_or_disconnected", mapping_state="stopped")
            atomic_json(args.session / "state.json", state)
            raise
    if process.returncode:
        state.update(status="failed", last_error="ORB-SLAM3 exited; see local orb.log", map_available=publisher.revision > 0)
    else:
        state["status"] = "complete"
    atomic_json(args.session / "state.json", state)
    return process.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    prior = parser.add_mutually_exclusive_group(required=True)
    prior.add_argument("--camera", type=Path, help="JSON: width,height,fx,fy,cx,cy,distortion=[k1,k2,p1,p2]")
    prior.add_argument("--calibration-result", type=Path, help="Reuse only self-estimated intrinsics, not poses or points")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--features", type=int, default=1800)
    parser.add_argument("--replay-rate", type=float, default=1)
    parser.add_argument("--max-seconds", type=float, default=0)
    parser.add_argument("--cpus", type=int, default=6)
    parser.add_argument("--image", default=IMAGE)
    parser.add_argument("--build", action="store_true")
    args = parser.parse_args()
    if not args.source.is_file() or args.width < 320 or args.features < 500 or args.cpus < 1 or not np.isfinite([args.replay_rate, args.max_seconds]).all() or min(args.replay_rate, args.max_seconds) < 0:
        parser.error("local video and positive, finite processing settings required")
    args.source, args.session = args.source.resolve(), args.session.resolve()
    cv2.setNumThreads(1)
    raise SystemExit(run(args))


if __name__ == "__main__":
    main()
