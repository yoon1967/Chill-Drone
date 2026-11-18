#!/usr/bin/env python3
"""비행 기록에서 장애물과 가장 가까웠던 순간들을 짚는다: 언제, 어디서, 어느 상자의 어느 면에, 기수 기준 어느 쪽에서, 어떤 상태로.

eval_flight.py는 최소 간격 하나만 내므로, 왜 붙었는지(길이 없어 여유를 줄인 직후인지, 옆으로 스쳤는지, 위치 오차인지)를
볼 때 쓴다. 정답 궤적(truth_log.csv)과 비행 기록(flight_log.csv: 추정 위치·상태)을 시각으로 맞춰 본다.

  python3 tools/eval_clearance.py worlds/lab_explore            # 가장 가까웠던 순간 6개 (4초 이상 떨어진 것끼리)
  python3 tools/eval_clearance.py worlds/lab_explore --top 10
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

DRONE_HALF = (0.09, 0.09, 0.025)   # tools/eval_flight.py와 같게


def clearance(p, b) -> float:
    g = [abs(p[k] - b[c]) - b[s] / 2 - DRONE_HALF[k] for k, c, s in ((0, "x", "sx"), (1, "y", "sy"), (2, "z", "sz"))]
    if all(v < 0 for v in g):
        return max(g)
    return math.sqrt(sum(max(v, 0.0) ** 2 for v in g))


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    wd = Path(sys.argv[1])
    top = int(sys.argv[sys.argv.index("--top") + 1]) if "--top" in sys.argv else 6
    truth = json.loads((wd / "ground_truth.json").read_text(encoding="utf-8"))
    boxes = truth["walls"] + truth.get("lintels", []) + truth["obstacles"]
    for b in boxes:
        b.setdefault("name", "?")
    rows = list(csv.DictReader((wd / "truth_log.csv").open(encoding="utf-8")))
    flog = list(csv.DictReader((wd / "flight_log.csv").open(encoding="utf-8")))
    ft = [float(r["t"]) for r in flog]

    def flight_at(t: float) -> dict:
        lo, hi = 0, len(ft) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if ft[mid] < t:
                lo = mid + 1
            else:
                hi = mid
        return flog[lo]

    samples = []
    for r in rows:
        p = (float(r["x"]), float(r["y"]), float(r["z"]))
        if p[2] <= 0.3:   # 이착륙 직전·직후 바닥 근처는 뺀다 (eval_flight.py와 같게)
            continue
        c, b = min(((clearance(p, b), b) for b in boxes), key=lambda v: v[0])
        samples.append((c, float(r["t"]), p, float(r["yaw"]), b))
    samples.sort(key=lambda s: s[0])
    picked: list = []
    for s in samples:
        if all(abs(s[1] - q[1]) >= 4.0 for q in picked):
            picked.append(s)
        if len(picked) >= top:
            break
    print(f"비행 {rows[0]['t']}–{rows[-1]['t']} s, 상자 {len(boxes)}개. 가장 가까웠던 순간 {len(picked)}개 (4초 이상 떨어진 것끼리):")
    for c, t, p, yaw, b in picked:
        f = flight_at(t)
        dx, dy = p[0] - b["x"], p[1] - b["y"]
        side = []
        if abs(dx) - b["sx"] / 2 > 0:
            side.append("+x면" if dx > 0 else "-x면")
        if abs(dy) - b["sy"] / 2 > 0:
            side.append("+y면" if dy > 0 else "-y면")
        bearing = math.degrees(wrap(math.atan2(b["y"] - p[1], b["x"] - p[0]) - yaw))
        where = "앞" if abs(bearing) < 35 else "뒤" if abs(bearing) > 145 else ("왼쪽" if bearing > 0 else "오른쪽")
        est = (float(f["x"]), float(f["y"]))
        err = math.hypot(est[0] - p[0], est[1] - p[1])
        print(f"  {c:5.2f} m  t={t:6.1f}  실제 ({p[0]:.2f}, {p[1]:.2f}) 추정 ({est[0]:.2f}, {est[1]:.2f}) 오차 {err:.2f} m  "
              f"상태 {f['state']:8s} 기수 {math.degrees(yaw):6.1f}°  {b['name']} {'·'.join(side) or '겹침'} "
              f"방향 {where}({bearing:+.0f}°)")
    if picked:
        _, t, *_ = picked[0]
        seq, last = [], None
        for f in flog:
            if abs(float(f["t"]) - t) <= 8 and f["state"] != last:
                seq.append(f"{float(f['t']):.1f}:{f['state']}")
                last = f["state"]
        print("  최소 순간 ±8초 상태 흐름:", " → ".join(seq))
    return 0


if __name__ == "__main__":
    sys.exit(main())
