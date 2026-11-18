#!/usr/bin/env python3
"""4단계 비행 기록(record/)을 시뮬레이터 없이 다시 돌려 카메라 지도와 위치 맞추기를 평가한다.

비행 노드(nodes/mission_node.py)가 지도에 넣은 깊이 영상·추정 위치·ToF를 같은 순서·같은 방법으로 다시 넣는다.
비행 경로는 기록된 그대로라(보정이 경로를 바꾸지 못함), 위치 맞추기가 위치를 얼마나 바로잡는지만 따로 잴 수 있다.
  위치 오차: 영상마다 |보정한 추정 위치 - 정답 위치|. '보정 전'은 기록된 추정 위치(속도 적분) 그대로
  지도 점수: tools/eval_map.py와 같다
비행 한 번(깊이 영상 약 1500장)에 위치 맞추기를 켜면 약 1분, 끄면 약 20초.

사용:
  python3 tools/replay_map.py worlds/s1_stage4                     # 위치 맞추기 끔 (비행과 같은 지도)
  python3 tools/replay_map.py worlds/s1_stage4 --localize          # 켬
  python3 tools/replay_map.py worlds/s1_stage4 --localize --set gain=0.3 --set match.sigma=0.15
  python3 tools/replay_map.py worlds/s1_stage4 --pose truth        # 정답 위치로 만든 지도 (깊이 추정만의 한계)
기록은 run_mission.sh --record로 남긴다 (WORLD_DIR/record).
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
from autonomy.camera_map import CameraMap, CameraMapParams  # noqa: E402
from autonomy.flight_record import load_record  # noqa: E402
from autonomy.scan_match import Correction, LocalizerParams, MapLocalizer  # noqa: E402
from eval_map import map_scores  # noqa: E402
from perception.depth import floor_scale  # noqa: E402

MARK_PERIOD = 1.0   # 비행 노드가 지나간 자리를 비우는 간격 (s, MAP_REBUILD)


def load_truth(path) -> dict:
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    a = {k: np.array([float(r[k]) for r in rows]) for k in ("t", "x", "y", "z", "yaw")}
    a["yaw"] = np.unwrap(a["yaw"])
    return a


def truth_at(truth: dict, t: float) -> tuple[float, float, float, float]:
    return tuple(float(np.interp(t, truth["t"], truth[k])) for k in ("x", "y", "z", "yaw"))


def set_param(obj, key: str, value: str) -> None:
    """'gain=0.3', 'match.sigma=0.15', 'rng=1.0,3.0' 처럼 dataclass 값을 바꾼다."""
    *path, last = key.split(".")
    for name in path:
        obj = getattr(obj, name)
    old = getattr(obj, last)
    if isinstance(old, tuple):
        new = tuple(float(v) for v in value.split(","))
    elif isinstance(old, bool):
        new = value.lower() in ("1", "true", "yes")
    else:
        new = type(old)(value)
    setattr(obj, last, new)


def raw_track(rec: dict) -> dict:
    """깊이 영상·ToF 때 기록된 추정 위치를 시각 순서로 합친 것 (사이 시각의 추정 위치를 보간하는 데 쓴다)."""
    t = np.concatenate([rec["t"], rec["tof_t"]])
    p = np.concatenate([rec["pose"], rec["tof_pose"]])
    order = np.argsort(t, kind="stable")
    t, p = t[order], p[order]
    keep = np.concatenate([[True], np.diff(t) > 1e-6])
    t, p = t[keep], p[keep]
    return {"t": t, "x": p[:, 0], "y": p[:, 1], "z": p[:, 2], "yaw": np.unwrap(p[:, 3])}


def home_spans(flight_log) -> list[tuple[float, float]]:
    """비행 기록(flight_log.csv)에서 출발점 위 한 바퀴(HOMESPIN) 구간."""
    spans, start = [], None
    for r in csv.DictReader(open(flight_log, encoding="utf-8")):
        t, spinning = float(r["t"]), r["state"] == "HOMESPIN"
        if spinning and start is None:
            start = t
        elif not spinning and start is not None:
            spans.append((start, t))
            start = None
    return spans


def replay(rec: dict, truth: dict, localize: bool = False, loc_params: LocalizerParams | None = None,
           map_params: CameraMapParams | None = None, pose_source: str = "raw", pose_lag: float = 0.0,
           spins=()) -> dict:
    """기록을 다시 돌린다. pose_source: 'raw'(기록된 추정 위치) 또는 'truth'(정답 위치, 위치 맞추기 없음).
    pose_lag: 추정 위치가 이만큼(s) 늦게 나온다고 보고, 영상 시각 + pose_lag의 추정 위치(보간)를 쓴다. 비행 노드가
    이미 당겨 기록한 만큼(meta의 pose_lag)에 더해진다. spins: 출발점 맞추기(home_mode)를 켤 구간 [(시작, 끝)]."""
    meta = rec["meta"]
    hfov, sx, tof_max = meta.get("hfov", 1.4416), meta.get("sensor_x", 0.05), meta.get("tof_max", 1.2)
    cmap = CameraMap(meta["bounds"], map_params or CameraMapParams())
    loc = MapLocalizer(loc_params) if localize and pose_source == "raw" else None
    corr = loc.correction if loc else Correction()
    track = raw_track(rec) if pose_lag else None

    def source(t, raw):
        if pose_source == "truth":
            return truth_at(truth, t)
        return truth_at(track, t + pose_lag) if track is not None else tuple(raw)

    def cam_of(pose):
        return pose[0] + sx * math.cos(pose[3]), pose[1] + sx * math.sin(pose[3]), pose[2]

    events = [(t, 0, i) for i, t in enumerate(rec["t"])] + [(t, 1, i) for i, t in enumerate(rec["tof_t"])]
    events.sort()
    scale, last_mark = 1.0, -math.inf
    out = {"t": [], "err": [], "err_raw": [], "yaw_err": [], "yaw_err_raw": [], "corr": []}
    for t, kind, i in events:
        if kind == 1:
            pose = corr.apply(source(t, rec["tof_pose"][i]))
            cmap.integrate_ray(cam_of(pose), pose[3], float(rec["tof_range"][i]), tof_max)
            continue
        raw = source(t, rec["pose"][i])
        pose = corr.apply(raw)
        depth = rec["depth"][i]
        s = floor_scale(depth, hfov, cam_of(pose)[2])
        if s is not None:
            scale = s
        depth = depth * scale
        if loc:   # 비행 노드처럼 이 영상은 고치기 전 위치로 넣고, 고친 보정은 다음 영상부터 쓴다
            loc.home_mode = any(a <= t <= b for a, b in spins)
            loc.update(t, cmap, depth, cam_of(pose), pose, hfov, raw=raw)
        cmap.integrate_depth(depth, cam_of(pose), pose[3], hfov)
        if t - last_mark >= MARK_PERIOD:
            cmap.mark_free_sphere(pose[:3])
            last_mark = t
        tx, ty, _, tyaw = truth_at(truth, t)
        out["t"].append(t)
        out["err"].append(math.hypot(pose[0] - tx, pose[1] - ty))
        out["err_raw"].append(math.hypot(raw[0] - tx, raw[1] - ty))
        out["yaw_err"].append(math.degrees(math.remainder(pose[3] - tyaw, 2 * math.pi)))
        out["yaw_err_raw"].append(math.degrees(math.remainder(raw[3] - tyaw, 2 * math.pi)))
        out["corr"].append((corr.tx, corr.ty, corr.yaw))
    for k in list(out):
        out[k] = np.array(out[k])
    out.update(cmap=cmap, localizer=loc)
    return out


def summarize(res: dict) -> dict:
    e, r = res["err"], res["err_raw"]
    loc = res["localizer"]
    return {"frames": int(len(e)), "mean": float(e.mean()), "p90": float(np.percentile(e, 90)), "max": float(e.max()),
            "final": float(e[-1]), "raw_mean": float(r.mean()), "raw_p90": float(np.percentile(r, 90)),
            "raw_max": float(r.max()), "raw_final": float(r[-1]),
            "better": float((e < r - 1e-6).mean()), "worse": float((e > r + 1e-6).mean()),
            "yaw_mean": float(np.abs(res["yaw_err"]).mean()), "yaw_raw_mean": float(np.abs(res["yaw_err_raw"]).mean()),
            "tries": loc.tries if loc else 0, "fixes": loc.fixes if loc else 0}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world_dir", type=Path, help="record/, truth_log.csv, ground_truth.json이 있는 폴더")
    ap.add_argument("--record", type=Path, default=None)
    ap.add_argument("--truth", type=Path, default=None)
    ap.add_argument("--gt", type=Path, default=None)
    ap.add_argument("--localize", action="store_true")
    ap.add_argument("--pose", choices=("raw", "truth"), default="raw")
    ap.add_argument("--pose-lag", type=float, default=0.0, help="추정 위치가 늦게 나오는 시간 (s), 보간으로 당겨 쓴다")
    ap.add_argument("--spins", type=Path, default=None,
                    help="이 비행 기록(flight_log.csv)의 출발점 한 바퀴 구간에서 출발점 맞추기를 켠다")
    ap.add_argument("--set", action="append", default=[], help="위치 맞추기 값 바꾸기 (LocalizerParams), 예: gain=0.3")
    ap.add_argument("--map-set", action="append", default=[], help="지도 값 바꾸기 (CameraMapParams)")
    args = ap.parse_args(argv)
    wd = args.world_dir
    rec = load_record(args.record or wd / "record")
    truth = load_truth(args.truth or wd / "truth_log.csv")
    gt = json.loads((args.gt or wd / "ground_truth.json").read_text(encoding="utf-8"))
    lp, mp = LocalizerParams(), CameraMapParams()
    for kv in args.set:
        set_param(lp, *kv.split("=", 1))
    for kv in args.map_set:
        set_param(mp, *kv.split("=", 1))
    res = replay(rec, truth, args.localize, lp, mp, args.pose, args.pose_lag,
                 home_spans(args.spins) if args.spins else ())
    s = summarize(res)
    cm = res["cmap"]
    sc = map_scores(cm.logodds, cm.seen, cm.origin, cm.p.res, gt, rec["meta"].get("height", 1.0))
    print(f"깊이 영상 {s['frames']}장" + (f", 위치 맞춤 {s['fixes']}번 (시도 {s['tries']}번)" if args.localize else ""))
    print(f"위치 오차   평균 {s['mean']:.2f} m, 90% {s['p90']:.2f} m, 최대 {s['max']:.2f} m, 끝 {s['final']:.2f} m, "
          f"방향 평균 {s['yaw_mean']:.1f}°")
    if args.pose == "raw":
        print(f"보정 전     평균 {s['raw_mean']:.2f} m, 90% {s['raw_p90']:.2f} m, 최대 {s['raw_max']:.2f} m, "
              f"끝 {s['raw_final']:.2f} m, 방향 평균 {s['yaw_raw_mean']:.1f}°  (보정이 나은 때 {s['better']:.0%}, "
              f"나쁜 때 {s['worse']:.0%})")
    print(f"지도        막힘 정확도 {sc['precision']:.0%}, 찾은 비율 {sc['recall']:.0%}, "
          f"비어 있다고 잘못 본 곳 {sc['wrong_free']:.1%}, 담은 넓이 {sc['explored'] / max(sc['gt_free_area'], 1e-6):.0%}")
    print("값: " + ", ".join(f"{f.name}={getattr(lp, f.name)}" for f in dataclasses.fields(lp) if f.name != "match"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
