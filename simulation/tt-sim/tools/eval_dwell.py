#!/usr/bin/env python3
"""비행 기록(flight_log.csv)에서 떠 있는 동안 '제자리에 머문 시간'을 잰다.

  움직임     수평 속도가 move_speed(기본 0.05 m/s) 이상
  제자리 회전 수평으로는 거의 안 움직이고 기수만 돈다 (turn_rate 이상, 기본 5°/s)
  정지       위치도 방향도 그대로 (같은 자리, 같은 자세)

이륙(TAKEOFF)·착륙(LAND) 단계와 그 뒤는 빼고, 비행 단계별로 정지·제자리 회전 시간을 나눠 보인다.
속도는 0.5초 창으로 계산한다(추정 위치의 잔떨림을 움직임으로 세지 않게).

  python3 tools/eval_dwell.py worlds/lab_search/flight_log.csv
  python3 tools/eval_dwell.py worlds/*/flight_log.csv --summary
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

SKIP = ("WAIT_POSE", "READY", "TAKEOFF", "LAND", "DONE", "ABORT")


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def analyze(path: Path, move_speed: float = 0.05, turn_rate_deg: float = 5.0, window: float = 0.5) -> dict:
    rows = [r for r in csv.DictReader(open(path, encoding="utf-8"))]
    t = [float(r["t"]) for r in rows]
    x = [float(r["x"]) for r in rows]
    y = [float(r["y"]) for r in rows]
    yaw = [float(r["yaw"]) for r in rows]
    states = [r["state"] for r in rows]
    out = {"move": 0.0, "turn": 0.0, "still": 0.0, "by_state": defaultdict(lambda: [0.0, 0.0]), "longest_still": 0.0}
    j = 0
    run = 0.0
    for i in range(1, len(rows)):
        dt = t[i] - t[i - 1]
        if dt <= 0 or dt > 1.0 or states[i] in SKIP:
            run = 0.0
            continue
        while t[i] - t[j] > window:
            j += 1
        span = max(t[i] - t[j], 1e-6)
        speed = math.hypot(x[i] - x[j], y[i] - y[j]) / span
        rate = abs(wrap(yaw[i] - yaw[j])) / span
        if speed >= move_speed:
            out["move"] += dt
            run = 0.0
        elif rate >= math.radians(turn_rate_deg):
            out["turn"] += dt
            out["by_state"][states[i]][1] += dt
            run = 0.0
        else:
            out["still"] += dt
            out["by_state"][states[i]][0] += dt
            run += dt
            out["longest_still"] = max(out["longest_still"], run)
    out["air"] = out["move"] + out["turn"] + out["still"]
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="비행 중 제자리에 머문 시간")
    ap.add_argument("logs", nargs="+", type=Path)
    ap.add_argument("--summary", action="store_true", help="기록마다 한 줄만")
    args = ap.parse_args()
    for p in args.logs:
        r = analyze(p)
        air = r["air"] or 1e-9
        line = (f"{p.parent.name:14s} 비행 {air:5.1f} s: 움직임 {r['move'] / air:4.0%}, "
                f"제자리 회전 {r['turn'] / air:4.0%} ({r['turn']:.1f} s), 정지 {r['still'] / air:4.0%} ({r['still']:.1f} s), "
                f"가장 긴 정지 {r['longest_still']:.1f} s")
        print(line)
        if not args.summary:
            for st, (still, turn) in sorted(r["by_state"].items(), key=lambda kv: -sum(kv[1])):
                print(f"    {st:9s} 정지 {still:5.1f} s · 제자리 회전 {turn:5.1f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
