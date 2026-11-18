#!/usr/bin/env python3
"""카메라 영상으로 추정한 거리(perception/depth.py)가 실제와 얼마나 맞는지 잰다 (시뮬레이션).

맵 안의 빈 자리 여러 곳에 드론을 비행 높이(1.0 m)로 순간이동시켜, 앞 카메라 영상으로 깊이를 추정하고
평가 전용 정답 깊이 카메라(/tt/camera/depth_gt)와 비교한다.
  AbsRel: |추정 - 정답| / 정답 의 평균,  δ1: 추정/정답 비가 1.25배 안인 화소 비율,  배율: 정답/추정 의 중앙값
  ToF 맞춤: 앞 ToF(1.2 m 안)에 닿는 것이 있으면 그 거리로 배율을 맞춘 뒤의 같은 값 (실기체에서 쓸 수 있는 보정)
--doors: 무작위 자리 대신 문 앞 1.5–3.5 m에서 문을 보는 자리로 간다 (문틀 경계의 깊이 오차 보기).
--save-raw: 장마다 영상·정답·추정 깊이·자세를 depth_eval/raw_NN.npz로 저장한다 (시뮬레이터 없이 거르기 시험용).
Gazebo가 떠 있어야 한다: bash scripts/eval_depth.sh worlds/s7_stage1
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
from autonomy import VoxelMap  # noqa: E402
from eval_detection import image_to_rgb, teleport  # noqa: E402
from perception.depth import DepthEstimator, scale_to_anchor  # noqa: E402

HFOV = 1.4416
TOF_HALF = math.radians(12.5)
BINS = ((0.3, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 8.0))


def depth_image(msg: Image) -> np.ndarray:
    return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()


def tof_region(h: int, w: int) -> tuple[slice, slice]:
    """ToF 원뿔(±12.5°)이 닿는 영상 가운데 부분."""
    f = (w / 2) / math.tan(HFOV / 2)
    r = int(f * math.tan(TOF_HALF))
    return slice(h // 2 - r, h // 2 + r), slice(w // 2 - r, w // 2 + r)


def colorize(depth: np.ndarray, vmax: float = 6.0) -> np.ndarray:
    """가까우면 노랑, 멀면 보라. 정답이 없는 곳(inf)은 검정."""
    import cv2

    d = np.clip(np.nan_to_num(depth, nan=0.0, posinf=0.0) / vmax, 0, 1)
    img = cv2.applyColorMap((255 * (1 - d)).astype(np.uint8), cv2.COLORMAP_PLASMA)[:, :, ::-1]
    img[~np.isfinite(depth) | (depth <= 0)] = 0
    return img


def metrics(pred: np.ndarray, gt: np.ndarray) -> dict:
    ok = np.isfinite(gt) & (gt > 0.3) & (gt < 8.0) & np.isfinite(pred) & (pred > 0)
    p, g = pred[ok], gt[ok]
    if g.size == 0:
        return {}
    ratio = np.maximum(p / g, g / p)
    out = {"n": int(g.size), "absrel": float(np.mean(np.abs(p - g) / g)), "rmse": float(np.sqrt(np.mean((p - g) ** 2))),
           "d1": float(np.mean(ratio < 1.25)), "scale": float(np.median(g / p))}
    for lo, hi in BINS:
        m = (g >= lo) & (g < hi)
        out[f"absrel_{lo:g}-{hi:g}"] = float(np.mean(np.abs(p[m] - g[m]) / g[m])) if m.any() else math.nan
    return out


def door_poses(truth: dict, solid: VoxelMap, k: int, rng: random.Random, n: int) -> list[tuple[float, float, float]]:
    """문마다 양쪽에서 1.5–3.5 m 떨어져 문 가운데(±15°)를 보는 빈 자리 (x, y, yaw)를 n개까지."""
    poses = []
    for _ in range(50 * n):
        if len(poses) >= n or not truth["doors"]:
            break
        d = truth["doors"][rng.randrange(len(truth["doors"]))]
        normal = (1.0, 0.0) if d["wall"] == "x" else (0.0, 1.0)
        side = rng.choice((-1.0, 1.0))
        dist, lateral = rng.uniform(1.5, 3.5), rng.uniform(-0.6, 0.6)
        x = d["x"] + side * dist * normal[0] + lateral * normal[1]
        y = d["y"] + side * dist * normal[1] + lateral * normal[0]
        idx = solid.index((x, y, 0))
        if not (0 <= idx[0] < solid.occ.shape[0] and 0 <= idx[1] < solid.occ.shape[1]) or solid.occ[idx[0], idx[1], k]:
            continue
        yaw = math.atan2(d["y"] - y, d["x"] - x) + rng.uniform(-0.26, 0.26)
        poses.append((x, y, yaw))
    return poses


def main() -> int:
    ap = argparse.ArgumentParser(description="카메라 깊이 추정 정확도 측정")
    ap.add_argument("world_dir", type=Path)
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--height", type=float, default=1.0)
    ap.add_argument("--doors", action="store_true", help="문 앞에서 문을 보는 자리만")
    ap.add_argument("--save-raw", action="store_true", help="장마다 영상·정답·추정 깊이를 npz로 저장")
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    truth = json.loads((args.world_dir / "ground_truth.json").read_text(encoding="utf-8"))
    p = truth["params"]
    humans = [{"x": h["x"], "y": h["y"], "z": 0.875, "sx": 0.5, "sy": 0.5, "sz": 1.75} for h in truth.get("humans", [])]
    solid = VoxelMap.from_known_map({"bounds": {"xmin": 0, "ymin": 0, "xmax": p["width"], "ymax": p["depth"],
                                                "zmax": p["wall_height"]},
                                     "walls": truth["walls"], "lintels": truth["lintels"],
                                     "obstacles": truth["obstacles"] + humans}, 0.1).inflate(0.4)
    k = solid.index((0, 0, args.height))[2]
    free = np.argwhere(~solid.occ[:, :, k])
    rng = random.Random(args.seed)
    if args.doors:
        poses = door_poses(truth, solid, k, rng, args.frames)
    else:
        poses = []
        for _ in range(args.frames):
            ci, cj = free[rng.randrange(len(free))][:2]
            x, y, _ = solid.center((ci, cj, k))
            poses.append((float(x), float(y), rng.uniform(-math.pi, math.pi)))

    est = DepthEstimator(hfov=HFOV)
    est.estimate(np.zeros((720, 960, 3), np.uint8))   # 첫 실행(모델 준비) 시간은 빼고 잰다

    rclpy.init()
    node = rclpy.create_node("tt_depth_eval")
    latest = {}
    node.create_subscription(Image, "/tt/camera/image_raw", lambda m: latest.__setitem__("rgb", m), qos_profile_sensor_data)
    node.create_subscription(Image, "/tt/camera/depth_gt", lambda m: latest.__setitem__("depth", m), qos_profile_sensor_data)

    def stamp(m) -> float:
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def wait_fresh(after: float, timeout: float = 15.0) -> bool:
        t_end = time.time() + timeout
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if all(key in latest and stamp(latest[key]) > after for key in ("rgb", "depth")):
                return True
        return False

    out_dir = args.world_dir / "depth_eval"
    out_dir.mkdir(exist_ok=True)
    rows, anchored, times = [], [], []
    for i, (x, y, yaw) in enumerate(poses):
        teleport(x, y, args.height, yaw)
        wait_fresh(-1)
        t_sim = max(stamp(latest["rgb"]), stamp(latest["depth"]))
        if not wait_fresh(t_sim + 0.3):
            print(f"  {i}: 영상이 안 와서 건너뜀")
            continue
        rgb = image_to_rgb(latest["rgb"])
        gt = depth_image(latest["depth"])
        t0 = time.time()
        pred_full = est.estimate(rgb)
        times.append(time.time() - t0)
        import cv2

        pred = cv2.resize(pred_full, (gt.shape[1], gt.shape[0]), interpolation=cv2.INTER_AREA)
        m = metrics(pred, gt)
        if not m:
            continue
        rows.append(m)
        if args.save_raw:
            np.savez_compressed(out_dir / f"raw_{i:02d}.npz", rgb=cv2.resize(rgb, (gt.shape[1], gt.shape[0])), gt=gt,
                                pred=pred, pose=np.array([x, y, args.height, yaw]), hfov=HFOV)
        # ToF 맞춤: 원뿔 안 가장 가까운 정답 거리가 1.2 m 안이면 그 값을 ToF 측정으로 보고 배율을 맞춘다
        region = tof_region(*gt.shape)
        g_patch = gt[region]
        g_patch = g_patch[np.isfinite(g_patch) & (g_patch > 0)]
        tof = float(g_patch.min()) if g_patch.size else math.inf
        if tof <= 1.2:
            s = scale_to_anchor(pred, tof, region)
            anchored.append({**metrics(pred * s, gt), "tof_scale": s})
        if i < 6:
            panel = np.concatenate([cv2.resize(rgb, (gt.shape[1], gt.shape[0])), colorize(gt), colorize(pred)], axis=1)
            cv2.imwrite(str(out_dir / f"frame_{i:02d}.png"), panel[:, :, ::-1])
        print(f"  {i:2d}: ({x:.1f}, {y:.1f}) yaw {math.degrees(yaw):4.0f}°  AbsRel {m['absrel']:.3f}  δ1 {m['d1']:.2f}  "
              f"배율 {m['scale']:.2f}" + (f"  ToF {tof:.2f} m" if tof <= 1.2 else ""))

    def summary(name, rs):
        if not rs:
            print(f"{name}: 없음")
            return
        keys = ["absrel", "rmse", "d1", "scale"] + [f"absrel_{lo:g}-{hi:g}" for lo, hi in BINS]
        avg = {key: float(np.nanmean([r.get(key, math.nan) for r in rs])) for key in keys}
        sc = [r["scale"] for r in rs]
        print(f"{name} ({len(rs)}장): AbsRel {avg['absrel']:.3f}, RMSE {avg['rmse']:.2f} m, δ1 {avg['d1']:.2f}, "
              f"배율(정답/추정) 중앙 {np.median(sc):.2f} (범위 {min(sc):.2f}–{max(sc):.2f})")
        print("   거리별 AbsRel: " + ", ".join(f"{lo:g}–{hi:g} m {avg[f'absrel_{lo:g}-{hi:g}']:.3f}" for lo, hi in BINS))

    print()
    summary("그대로", rows)
    summary("ToF로 배율 맞춤", anchored)
    if times:
        print(f"추정 시간 평균 {1000 * np.mean(times):.0f} ms (960×720 한 장)")
    (out_dir / "summary.json").write_text(json.dumps({"raw": rows, "anchored": anchored,
                                                      "ms": 1000 * float(np.mean(times)) if times else None},
                                                     ensure_ascii=False, indent=1), encoding="utf-8")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
