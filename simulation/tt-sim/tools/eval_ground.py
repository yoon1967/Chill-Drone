#!/usr/bin/env python3
"""YOLO 상자로 추정한 사람 위치(바닥 접점)가 실제와 얼마나 어긋나는지 잰다 (시뮬레이션).

사람마다 거리 1.5–4 m, 사람 기준 4방향에 드론을 사람 쪽을 보게 순간이동시키고, YOLO 상자 아래 변으로
바닥 접점을 구해(autonomy.obstacles.ground_point) 사람 위치와 비교한다.
  앞뒤 오차: 추정 거리 - (사람 중심까지 거리 - 사람 반지름 0.2 m)   (+면 실제보다 멀게 봄)
  옆 오차:   시선과 수직 방향으로 어긋난 거리
Gazebo가 떠 있어야 한다. 보통은 scripts/eval_detection.sh와 같은 방법으로 띄운 뒤 실행한다.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
from autonomy import VoxelMap  # noqa: E402
from autonomy.obstacles import ground_point  # noqa: E402
from eval_detection import CAM_Z, HEIGHT, HFOV, WIDTH, image_to_rgb, teleport  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from perception import PERSON, YoloDetector, project  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

PERSON_RADIUS = 0.2


def main() -> int:
    ap = argparse.ArgumentParser(description="YOLO 바닥 접점 위치 오차 측정")
    ap.add_argument("world_dir", type=Path)
    ap.add_argument("--distances", default="1.5,2,3,4")
    ap.add_argument("--model", default=str(ROOT / "weights" / "yolo11n.pt"))
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])
    distances = [float(d) for d in args.distances.split(",")]

    truth = json.loads((args.world_dir / "ground_truth.json").read_text(encoding="utf-8"))
    people = truth["humans"] + [{"name": o.get("name", "person"), "x": o["x"], "y": o["y"], "yaw": o["yaw"]}
                                for o in truth["obstacles"] if o["kind"] == "person"]   # --blockers로 세운 사람
    if not people:
        print("사람이 없는 맵입니다.")
        return 1
    p = truth["params"]
    solid = VoxelMap.from_known_map({"bounds": {"xmin": 0, "ymin": 0, "xmax": p["width"], "ymax": p["depth"],
                                                "zmax": p["wall_height"]},
                                     "walls": truth["walls"], "lintels": truth["lintels"],
                                     "obstacles": [o for o in truth["obstacles"] if o["kind"] != "person"]}, 0.05)
    roomy = solid.inflate(0.15)

    yolo = YoloDetector(args.model, conf=0.4)
    yolo.detect(np.zeros((HEIGHT, WIDTH, 3), np.uint8))
    rclpy.init()
    node = rclpy.create_node("tt_ground_eval")
    latest: dict = {}
    node.create_subscription(Image, "/tt/camera/image_raw", lambda m: latest.update(msg=m), qos_profile_sensor_data)
    node.create_subscription(Odometry, "/tt/odometry_gt", lambda m: latest.update(odom=m), 10)

    def capture(x, y):
        t_ok = None
        for _ in range(200):
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

    rows = []
    for h in people:
        for dist in distances:
            for ang in (0, 90, 180, 270):
                theta = h["yaw"] + math.radians(ang)
                x, y = h["x"] + dist * math.cos(theta), h["y"] + dist * math.sin(theta)
                yaw = theta + math.pi
                cam = (x + 0.05 * math.cos(yaw), y + 0.05 * math.sin(yaw), CAM_Z)
                if roomy.occupied_at((x, y, CAM_Z)) or not solid.segment_free(cam, (h["x"], h["y"], 0.1)):
                    continue   # 들어갈 수 없거나 발이 가려 보이지 않는 자리
                teleport(x, y, CAM_Z, yaw)
                msg = capture(x, y)
                if msg is None:
                    continue
                dets = [d for d in yolo.detect(image_to_rgb(msg)) if d.cls == PERSON]
                u, v = project(cam, yaw, (h["x"], h["y"], 1.0), WIDTH, HEIGHT, HFOV)
                hit = [d for d in dets if d.contains(u, v, margin=15)]
                if not hit:
                    continue
                d = max(hit, key=lambda k: k.conf)
                g = ground_point((d.x1, d.y1, d.x2, d.y2), cam, yaw, WIDTH, HEIGHT, HFOV)
                if g is None:
                    rows.append((h.get("name", "?"), dist, ang, None, None, d.y2))
                    continue
                gx, gy, est = g
                true_front = math.hypot(h["x"] - cam[0], h["y"] - cam[1]) - PERSON_RADIUS
                ex, ey = gx - cam[0], gy - cam[1]
                along = math.cos(yaw) * ex + math.sin(yaw) * ey
                side = -math.sin(yaw) * (gx - h["x"]) + math.cos(yaw) * (gy - h["y"])
                rows.append((h.get("name", "?"), dist, ang, along - true_front, side, d.y2))
                print(f"{h.get('name', '?')} {dist:g} m {ang:3d}°: 앞뒤 오차 {along - true_front:+.2f} m, "
                      f"옆 오차 {side:+.2f} m (상자 아래 변 {d.y2:.0f}px)")
    node.destroy_node()
    rclpy.try_shutdown()

    print("\n거리별 (발이 보이는 자리만)")
    print("거리   자리  아래잘림  앞뒤 오차 평균(최소~최대)   옆 오차 평균 절댓값")
    for dist in distances:
        sel = [r for r in rows if r[1] == dist]
        ok = [r for r in sel if r[3] is not None]
        if ok:
            a = [r[3] for r in ok]
            s = [abs(r[4]) for r in ok]
            print(f"{dist:4g} m  {len(sel):3d}  {len(sel) - len(ok):5d}     {np.mean(a):+.2f} ({min(a):+.2f}~{max(a):+.2f})"
                  f"        {np.mean(s):.2f}")
        elif sel:
            print(f"{dist:4g} m  {len(sel):3d}  {len(sel):5d}     -")
    return 0


if __name__ == "__main__":
    sys.exit(main())
