#!/usr/bin/env python3
"""사람 인식 성능을 거리·방향별로 잰다 (시뮬레이션).

사람마다 거리 1–6 m, 사람을 중심으로 4방향(사람이 보는 방향 기준 0·90·180·270°)에 드론을
사람 쪽을 보게 순간이동시키고, 카메라 한 장에 YOLO를 돌려 그 사람이 잡혔는지 본다.
벽·가구에 가려 안 보이거나 드론이 들어갈 수 없는 자리는 뺀다.

결과 (world_dir 안)
  detection_eval.csv   자리마다 결과
  detection_eval.png   상자를 그린 사진 모음 (O = 잡음, X = 못 잡음)

Gazebo가 떠 있어야 한다. 보통은 scripts/eval_detection.sh로 실행한다.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from autonomy import VoxelMap  # noqa: E402
from perception import PERSON, YoloDetector, draw, project  # noqa: E402
from worldgen.images import write_png  # noqa: E402

WIDTH, HEIGHT, HFOV = 960, 720, 1.4416   # models/tt_drone/model.sdf와 같아야 한다
CAM_Z = 1.0
ANGLES = (0, 90, 180, 270)


def image_to_rgb(msg: Image) -> np.ndarray:
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    return buf[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()


def teleport(x: float, y: float, z: float, yaw: float) -> None:
    req = (f'name: "tt_drone", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, '
           f'orientation: {{z: {math.sin(yaw / 2):.5f}, w: {math.cos(yaw / 2):.5f}}}')
    subprocess.run(["gz", "service", "-s", "/world/tt_world/set_pose", "--reqtype", "gz.msgs.Pose",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                   check=True, capture_output=True)


def thumbnail(rgb: np.ndarray, text: str, ok: bool) -> np.ndarray:
    import cv2

    small = cv2.resize(rgb, (240, 180), interpolation=cv2.INTER_AREA)
    color = (30, 150, 60) if ok else (200, 40, 40)
    cv2.rectangle(small, (0, 0), (239, 20), color, -1)
    cv2.putText(small, text, (4, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return small


def main() -> int:
    ap = argparse.ArgumentParser(description="사람 인식 거리·방향별 평가")
    ap.add_argument("world_dir", type=Path)
    ap.add_argument("--distances", default="1,2,3,4,5,6")
    ap.add_argument("--model", default=str(ROOT / "weights" / "yolo11n.pt"))
    ap.add_argument("--conf", type=float, default=0.4)
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])
    distances = [float(d) for d in args.distances.split(",")]

    truth = json.loads((args.world_dir / "ground_truth.json").read_text(encoding="utf-8"))
    if not truth["humans"]:
        print("이 맵에는 사람이 없습니다. 3·4단계 맵을 쓰세요.")
        return 1
    p = truth["params"]
    solid = VoxelMap.from_known_map({"bounds": {"xmin": 0, "ymin": 0, "xmax": p["width"], "ymax": p["depth"],
                                                "zmax": p["wall_height"]},
                                     "walls": truth["walls"], "lintels": truth["lintels"],
                                     "obstacles": truth["obstacles"]}, 0.05)
    roomy = solid.inflate(0.15)

    Path(args.model).parent.mkdir(parents=True, exist_ok=True)
    yolo = YoloDetector(args.model, conf=args.conf)
    yolo.detect(np.zeros((HEIGHT, WIDTH, 3), np.uint8))   # 첫 추론은 GPU 준비로 느려서 미리 한 번 돌린다
    rclpy.init()
    node = rclpy.create_node("tt_detection_eval")
    latest: dict = {}
    node.create_subscription(Image, "/tt/camera/image_raw", lambda m: latest.update(msg=m), qos_profile_sensor_data)
    node.create_subscription(Odometry, "/tt/odometry_gt", lambda m: latest.update(odom=m), 10)

    def capture(x: float, y: float) -> Image | None:
        """드론이 (x, y)에 와 있는 것을 확인한 뒤 찍힌 프레임을 돌려준다."""
        deadline = time.time() + 10
        t_ok = None
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            odom, msg = latest.get("odom"), latest.get("msg")
            now = node.get_clock().now().nanoseconds * 1e-9
            if t_ok is None and odom is not None:
                pos = odom.pose.pose.position
                if math.hypot(pos.x - x, pos.y - y) < 0.05 and abs(pos.z - CAM_Z) < 0.05:
                    t_ok = now
            if t_ok is not None and msg is not None and msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 > t_ok + 0.1:
                return msg
        return None

    rows, thumbs = [], []
    for h in truth["humans"]:
        target = (h["x"], h["y"], CAM_Z)
        for dist in distances:
            for ang in ANGLES:
                theta = h["yaw"] + math.radians(ang)
                x, y = h["x"] + dist * math.cos(theta), h["y"] + dist * math.sin(theta)
                yaw = theta + math.pi
                cam = (x + 0.05 * math.cos(yaw), y + 0.05 * math.sin(yaw), CAM_Z)
                if roomy.occupied_at((x, y, CAM_Z)):
                    continue        # 드론이 들어갈 수 없는 자리
                if not (solid.segment_free(cam, (h["x"], h["y"], 1.25)) and solid.segment_free(cam, target)):
                    continue        # 벽·가구에 가려 사람이 안 보이는 자리
                teleport(x, y, CAM_Z, yaw)
                msg = capture(x, y)
                if msg is None:
                    print(f"{h['name']} {dist} m {ang}°: 영상이 오지 않아 건너뜀")
                    continue
                rgb = image_to_rgb(msg)
                t0 = time.perf_counter()
                dets = yolo.detect(rgb)
                infer_ms = (time.perf_counter() - t0) * 1000
                u, v = project(cam, yaw, target, WIDTH, HEIGHT, HFOV)
                people = [d for d in dets if d.cls == PERSON]
                hits = [d for d in people if d.contains(u, v, margin=15)]
                conf = max((d.conf for d in hits), default=0.0)
                rows.append({"human": h["name"], "distance_m": dist, "angle_deg": ang, "x": round(x, 2),
                             "y": round(y, 2), "hit": int(bool(hits)), "conf": round(conf, 3),
                             "people_in_view": len(people), "infer_ms": round(infer_ms, 1)})
                mark = "O" if hits else "X"
                thumbs.append(thumbnail(draw(rgb, dets), f"{h['name']} {dist:g}m {ang}deg {mark} {conf:.2f}", bool(hits)))
                print(f"{h['name']} {dist:g} m {ang:3d}°: {mark} 신뢰도 {conf:.2f} (사람 상자 {len(people)}개, {infer_ms:.0f} ms)")
    node.destroy_node()
    rclpy.try_shutdown()
    if not rows:
        print("평가할 수 있는 자리가 없습니다.")
        return 1

    with open(args.world_dir / "detection_eval.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    cols = 6
    blank = np.full((180, 240, 3), 255, np.uint8)
    thumbs += [blank] * (-len(thumbs) % cols)
    sheet = np.vstack([np.hstack(thumbs[i:i + cols]) for i in range(0, len(thumbs), cols)])
    write_png(args.world_dir / "detection_eval.png", sheet.shape[1], sheet.shape[0],
              [bytearray(r.tobytes()) for r in sheet])

    print("\n거리별 결과 (가려지지 않은 자리만)")
    print("거리   자리  잡음  인식률  평균 신뢰도")
    for dist in distances:
        sel = [r for r in rows if r["distance_m"] == dist]
        if sel:
            hit = [r for r in sel if r["hit"]]
            mean_conf = np.mean([r["conf"] for r in hit]) if hit else 0.0
            print(f"{dist:4g} m  {len(sel):3d}  {len(hit):3d}   {100 * len(hit) / len(sel):5.0f}%   {mean_conf:.2f}")
    total = sum(r["hit"] for r in rows)
    print(f"전체   {len(rows):3d}  {total:3d}   {100 * total / len(rows):5.0f}%")
    print(f"추론 시간 평균 {np.mean([r['infer_ms'] for r in rows]):.0f} ms")
    return 0


if __name__ == "__main__":
    sys.exit(main())
