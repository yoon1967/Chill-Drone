#!/usr/bin/env python3
"""Gazebo 없이 위치 추정기를 다시 돌려 본다: 기록된 실제 궤적(truth_log.csv)에서 Tello 상태값과 전방 ToF를
sim_tello_state.py·Gazebo와 같은 규칙으로 다시 만들어 PoseEstimator에 넣고, 추정 오차를 잰다.

추정기 값을 바꿔 가며 여러 비행을 몇 초 만에 비교할 때 쓴다 (비행 경로는 기록된 그대로라 되먹임은 없다).

  python3 tools/replay_pose.py worlds/s3_stage2 --truth runs/est2_s3_stage2_truth_log.csv
  python3 tools/replay_pose.py worlds/s3_stage2 --truth ... --set tof_max_yaw_rate=100 --set tof_outlier=1.1
  python3 tools/replay_pose.py worlds/s3_stage2 --truth ... --inputs worlds/s3_stage2/pose_inputs.csv

--inputs를 주면 상태값·ToF를 만들지 않고 pose_node.py가 실제로 받은 것(pose_inputs.csv)을 그대로 넣는다.
Gazebo ToF가 가구의 실제 모양을 본 값까지 같으므로 실비행의 추정을 그대로 재현한다.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import random
import sys
from dataclasses import fields
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from autonomy import tello_state  # noqa: E402
from autonomy.state_estimator import EstimatorParams, PoseEstimator, occupancy_slice  # noqa: E402
from autonomy.tello_state import wrap  # noqa: E402

TOF_HALF = math.radians(12.5)
FAN = np.linspace(-TOF_HALF, TOF_HALF, 7)
SENSOR_X = 0.05


class Truth:
    """truth_log.csv를 시각으로 보간한다."""

    def __init__(self, path: Path):
        rows = list(csv.DictReader(open(path, encoding="utf-8")))
        self.t = [float(r["t"]) for r in rows]
        self.p = np.array([[float(r[k]) for k in ("x", "y", "z")] for r in rows])
        self.yaw = np.unwrap([float(r["yaw"]) for r in rows])

    def at(self, t: float):
        i = min(max(bisect.bisect_right(self.t, t) - 1, 0), len(self.t) - 2)
        a = min(max((t - self.t[i]) / max(self.t[i + 1] - self.t[i], 1e-9), 0.0), 1.0)
        p = self.p[i] + a * (self.p[i + 1] - self.p[i])
        return p[0], p[1], p[2], wrap(self.yaw[i] + a * (self.yaw[i + 1] - self.yaw[i]))


def true_occupancy(gt: dict, height: float, porous: float = 0.0, seed: int = 0):
    """정답 지도(숨긴 물건·사람 포함)의 비행 높이 단면.

    porous > 0이면 받은 지도에 있는 가구 상자 속 10 cm 칸을 그 비율만큼 비운다. Gazebo의 ToF는 가구의 실제
    모양(선반 칸, 다리 사이)을 보므로 상자보다 멀게 재는 일이 많은데, 그것을 대충 흉내 낸다.
    """
    humans = [{"x": h["x"], "y": h["y"], "z": 0.875, "sx": 2 * h["radius"], "sy": 2 * h["radius"], "sz": 1.75}
              for h in gt.get("humans", [])]
    data = {"bounds": _bounds(gt), "walls": gt["walls"], "lintels": gt["lintels"],
            "obstacles": gt["obstacles"] + humans}
    occ, origin, res = occupancy_slice(data, height, 0.02)
    if porous > 0:
        known = [o for o in gt["obstacles"] if o["in_known_map"]]
        furn = occupancy_slice({**data, "walls": [], "lintels": [], "obstacles": known}, height, 0.02)[0]
        rng = np.random.default_rng(seed)
        holes = rng.random(((occ.shape[0] + 4) // 5, (occ.shape[1] + 4) // 5)) < porous
        holes = np.repeat(np.repeat(holes, 5, axis=0), 5, axis=1)[:occ.shape[0], :occ.shape[1]]
        solid = occupancy_slice({**data, "obstacles": [o for o in data["obstacles"] if o not in known]}, height, 0.02)[0]
        occ = solid | (furn & ~holes)
    return occ, origin, res


def _bounds(gt: dict) -> dict:
    p = gt["params"]
    return {"xmin": 0, "ymin": 0, "xmax": p["width"], "ymax": p["depth"], "zmax": p["wall_height"]}


def tof_range(occ, origin, res, x, y, yaw, max_range=1.2) -> float:
    ox, oy = x + SENSOR_X * math.cos(yaw), y + SENSOR_X * math.sin(yaw)
    ts = np.arange(0.02, max_range + 1e-9, 0.01)
    ang = yaw + FAN
    px = ox + np.cos(ang)[:, None] * ts[None, :]
    py = oy + np.sin(ang)[:, None] * ts[None, :]
    i = np.floor((px - origin[0]) / res).astype(int)
    j = np.floor((py - origin[1]) / res).astype(int)
    ok = (i >= 0) & (j >= 0) & (i < occ.shape[0]) & (j < occ.shape[1])
    hit = np.ones_like(ok)
    hit[ok] = occ[i[ok], j[ok]]
    first = np.where(hit.any(axis=1), ts[np.argmax(hit, axis=1)], np.inf)
    return float(first.min())


def replay(world_dir: Path, truth_path: Path, params: EstimatorParams, scale: float | None = None,
           seed: int = 0, height: float = 1.0, verbose: bool = False, porous: float = 0.0,
           use_solid: bool = True, inputs: Path | None = None) -> dict:
    km = json.loads((world_dir / "known_map.json").read_text(encoding="utf-8"))
    gt = json.loads((world_dir / "ground_truth.json").read_text(encoding="utf-8"))
    truth = Truth(truth_path)
    occ_t, org_t, res_t = true_occupancy(gt, height, porous, seed)
    occ, origin, res = occupancy_slice(km, height, 0.05)
    solid = occupancy_slice({**km, "obstacles": []}, height, 0.05)[0] if use_solid else None
    spawn = km["spawn"]
    est = PoseEstimator((spawn["x"], spawn["y"]), spawn.get("yaw", 0.0), spawn.get("z", 0.0),
                        occ2d=occ, origin=origin, res=res, params=params, seed=seed, solid2d=solid)

    # sim_tello_state.py와 같은 오차 (가정값)
    rng = random.Random(seed)
    scale = scale if scale is not None else 1.0 + rng.gauss(0, 0.03)
    yaw_zero = rng.uniform(-math.pi, math.pi)
    vel_noise, yaw_noise, drift, h_noise, latency = 0.03, math.radians(0.3), math.radians(1.0) / 60, 0.02, 0.03

    t0, t_end = truth.t[0], truth.t[-1]
    z0 = truth.at(t0)[2]
    events = []   # (시각, 종류, 값)
    prev = None
    k = 0
    while t0 + k * 0.1 <= t_end:
        ts = t0 + k * 0.1
        x, y, z, yaw = truth.at(ts)
        vx_w = vy_w = vz = 0.0
        if prev is not None:
            vx_w, vy_w, vz = (x - prev[0]) / 0.1, (y - prev[1]) / 0.1, (z - prev[2]) / 0.1
        prev = (x, y, z)
        c, s = math.cos(yaw), math.sin(yaw)
        vx = (c * vx_w + s * vy_w) * scale + rng.gauss(0, vel_noise)
        vy = (-s * vx_w + c * vy_w) * scale + rng.gauss(0, vel_noise)
        yaw_reading = wrap(yaw - yaw_zero + drift * (ts - t0) + rng.gauss(0, yaw_noise))
        h = max(0.0, z - z0 + rng.gauss(0, h_noise))
        raw = tello_state.encode(tello_state.from_si(yaw_reading, vx, vy, vz, h, z, 90))
        events.append((ts + latency, "state", raw))
        tt = ts + 0.05                                   # ToF는 상태값과 엇갈린 시각에 10 Hz
        if tt <= t_end:
            xt, yt, zt, yawt = truth.at(tt)
            events.append((tt, "tof", (tof_range(occ_t, org_t, res_t, xt, yt, yawt), zt)))
        k += 1
    events.sort(key=lambda e: e[0])
    if inputs is not None:   # 실제로 받은 입력을 받은 순서 그대로 (시각으로 다시 정렬하면 순서가 바뀌어 결과가 달라진다)
        events = []
        for r in csv.DictReader(open(inputs, encoding="utf-8")):
            if r["kind"] == "state":
                events.append((float(r["t"]), "state", r["value"]))
            elif r["kind"] == "spread":
                events.append((float(r["t"]), "spread", json.loads(r["value"])))
            elif r["kind"] == "under":
                events.append((float(r["t"]), "under", json.loads(r["value"])))
            else:
                events.append((float(r["t"]), "tof", (float(r["value"]), None)))

    errs, sigmas, over = [], [], 0
    for t, kind, val in events:
        if kind == "state":
            est.on_state(tello_state.to_si(tello_state.parse(val)), t)
            ex, ey, _, _, sigma = est.estimate()
            tx, ty, _, _ = truth.at(t)
            e = math.hypot(ex - tx, ey - ty)
            errs.append(e)
            sigmas.append(sigma)
            over += e > 2 * sigma + 0.05
            if verbose and len(errs) % 20 == 0:
                print(f"t {t - t0:5.1f} err {e:.2f} σ {sigma:.2f}")
        elif kind == "spread":
            est.spread(float(val["xy"]), float(val["yaw"]))
        elif kind == "under":
            top = val.get("top")
            est.on_under(None if top is None else float(top), float(val["t"]))
        else:
            measured, zt = val
            if abs(est.z - height) <= 0.25:
                est.on_tof(measured, FAN, t)
    return {"mean": float(np.mean(errs)), "max": float(np.max(errs)), "final": errs[-1],
            "sigma_final": sigmas[-1], "overconfident": over / len(errs), "scale": scale}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world_dir", type=Path)
    ap.add_argument("--truth", type=Path, required=True, help="truth_log.csv")
    ap.add_argument("--scale", type=float, default=None, help="속도 배율 오차 (기본: seed로 뽑음)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--set", action="append", default=[], help="EstimatorParams 값 바꾸기 (예: tof_sigma=0.1)")
    ap.add_argument("--porous", type=float, default=0.3, help="가구 상자 속을 비우는 비율 (Gazebo ToF 흉내, 기본 0.3)")
    ap.add_argument("--no-solid", action="store_true", help="벽·가구 구분 없이 (예전 방식)")
    ap.add_argument("--inputs", type=Path, default=None, help="pose_node가 실제로 받은 입력 (pose_inputs.csv)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)
    kinds = {f.name: f.type for f in fields(EstimatorParams)}
    over = {}
    for item in args.set:
        key, value = item.split("=", 1)
        if key not in kinds:
            ap.error(f"모르는 값: {key}")
        over[key] = value.lower() in ("1", "true") if kinds[key] in (bool, "bool") else float(value)
    r = replay(args.world_dir, args.truth, EstimatorParams(**over), args.scale, args.seed, verbose=args.verbose,
               porous=args.porous, use_solid=not args.no_solid, inputs=args.inputs)
    print(f"오차 평균 {r['mean']:.2f} m, 최대 {r['max']:.2f} m, 끝 {r['final']:.2f} m, 끝 σ {r['sigma_final']:.2f} m, "
          f"σ보다 확실히 틀린 시간 {r['overconfident']:.0%} (속도 배율 {r['scale']:.3f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
