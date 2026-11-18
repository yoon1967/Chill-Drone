#!/usr/bin/env python3
"""비행 기록(flight_log.csv)을 정답(ground_truth.json)과 비교해 결과를 낸다.

지표: 최종 상태, 목적지 도착 오차, 장애물과의 최소 간격(드론 외곽 기준), 비행 시간·거리.
비행 중에 만든 장애물(spawned.json, tools/spawn_obstacles.py)은 생긴 시각 뒤의 궤적만 간격 계산에 넣는다.
정답 기록(truth_log.csv, tools/log_truth.py)이 있으면 충돌·목적지 오차는 정답 궤적으로 재고,
비행 기록(비행 코드가 믿은 추정 위치)과 비교해 위치 추정 오차도 낸다.
지도에 없는 물건이 있는 맵이면: 받은 지도로 짠 원래 경로를 막은 물건, 재계획 횟수,
찾은 장애물(discovered.json)이 실제 숨은 물건과 맞는지도 낸다.
world_dir/flight.png에 위에서 본 궤적을 그린다. 실패면 종료 코드 1.

예) python3 tools/eval_flight.py worlds/s7_stage2
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from autonomy import PlanError, VoxelMap, path_clear, plan  # noqa: E402
from worldgen.images import TopView  # noqa: E402

DRONE_HALF = (0.09, 0.09, 0.025)   # 드론 외곽 반폭: 프로펠러 가드 포함 수평 0.09 m(가정), 높이 41 mm의 절반
DRONE_RADIUS = 0.25                # nodes/mission_node.py와 같게


def box_distance(p, b) -> float:
    """점과 상자 표면 사이 거리 (안에 있으면 0)."""
    d = [max(abs(p[k] - b[c]) - b[s] / 2, 0.0) for k, c, s in ((0, "x", "sx"), (1, "y", "sy"), (2, "z", "sz"))]
    return math.sqrt(sum(v * v for v in d))


def clearance(p, b) -> float:
    """드론 외곽 상자와 장애물 상자 사이 간격. 겹치면 음수(가장 얕게 파고든 깊이)."""
    g = [abs(p[k] - b[c]) - b[s] / 2 - DRONE_HALF[k] for k, c, s in ((0, "x", "sx"), (1, "y", "sy"), (2, "z", "sz"))]
    if all(v < 0 for v in g):
        return max(g)
    return math.sqrt(sum(max(v, 0.0) ** 2 for v in g))


def footprint_gap(a, b) -> float:
    gx = max(abs(a["x"] - b["x"]) - (a["sx"] + b["sx"]) / 2, 0.0)
    gy = max(abs(a["y"] - b["y"]) - (a["sy"] + b["sy"]) / 2, 0.0)
    return math.hypot(gx, gy)


def outline(view: TopView, b, color, t=0.04) -> None:
    x0, y0, x1, y1 = b["x"] - b["sx"] / 2, b["y"] - b["sy"] / 2, b["x"] + b["sx"] / 2, b["y"] + b["sy"] / 2
    for r in ((x0, y0, x1, y0 + t), (x0, y1 - t, x1, y1), (x0, y0, x0 + t, y1), (x1 - t, y0, x1, y1)):
        view.rect(*r, color)


def original_plan(world_dir: Path, truth: dict):
    """드론이 받은 지도로 출발 때 짠 경로 (재계획 전)."""
    data = json.loads((world_dir / "known_map.json").read_text(encoding="utf-8"))
    if not truth["goal"]:
        return None
    s, g = truth["spawn"], truth["goal"]
    try:
        inflated = VoxelMap.from_known_map(data, 0.1).inflate(DRONE_RADIUS)
        return plan(inflated, (s["x"], s["y"], g["z"]), (g["x"], g["y"], g["z"]))
    except PlanError:
        return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="비행 기록 평가")
    ap.add_argument("world_dir", type=Path)
    ap.add_argument("--log", type=Path, default=None, help="기본: world_dir/flight_log.csv")
    ap.add_argument("--discovered", type=Path, default=None, help="기본: world_dir/discovered.json")
    ap.add_argument("--png", type=Path, default=None, help="기본: world_dir/flight.png")
    ap.add_argument("--spawned", type=Path, default=None, help="기본: world_dir/spawned.json")
    ap.add_argument("--truth", type=Path, default=None, help="기본: world_dir/truth_log.csv")
    args = ap.parse_args(argv)

    truth = json.loads((args.world_dir / "ground_truth.json").read_text(encoding="utf-8"))
    rows = list(csv.DictReader(open(args.log or args.world_dir / "flight_log.csv", encoding="utf-8")))
    if not rows:
        print("비행 기록이 비어 있습니다.")
        return 1
    pts = [(float(r["x"]), float(r["y"]), float(r["z"])) for r in rows]
    times = [float(r["t"]) for r in rows]
    est_pts, est_times = pts, times
    truth_path = args.truth or args.world_dir / "truth_log.csv"
    truth_rows = list(csv.DictReader(open(truth_path, encoding="utf-8"))) if truth_path.exists() else []
    if truth_rows:   # 충돌·도착은 실제(정답) 궤적으로 잰다
        t_end = times[-1] + 0.5
        truth_rows = [r for r in truth_rows if times[0] - 0.5 <= float(r["t"]) <= t_end]
        pts = [(float(r["x"]), float(r["y"]), float(r["z"])) for r in truth_rows]
        times = [float(r["t"]) for r in truth_rows]
    humans = [{"x": h["x"], "y": h["y"], "z": 0.875, "sx": 2 * h["radius"], "sy": 2 * h["radius"], "sz": 1.75,
               "kind": "human"} for h in truth.get("humans", [])]
    boxes = truth["walls"] + truth["lintels"] + truth["obstacles"] + humans   # 사람과도 부딪히면 안 된다
    hidden = [o for o in truth["obstacles"] if not o["in_known_map"]]
    spawned_path = args.spawned or args.world_dir / "spawned.json"
    spawned = json.loads(spawned_path.read_text(encoding="utf-8"))["obstacles"] if spawned_path.exists() else []

    found = []
    disc_path = args.discovered or args.world_dir / "discovered.json"
    if disc_path.exists():
        found = json.loads(disc_path.read_text(encoding="utf-8"))["obstacles"]

    airborne = [p for p in pts if p[2] > 0.3]   # 이착륙 직전·직후 바닥 근처는 뺀다
    timed = [(t, p) for t, p in zip(times, pts) if p[2] > 0.3]

    def clear_to(o) -> float:
        """o와의 최소 간격. 비행 중 생긴 장애물은 생긴 뒤의 궤적만 본다."""
        return min((clearance(p, o) for t, p in timed if t >= o.get("t", -math.inf)), default=math.inf)

    min_clear = min([min((clearance(p, b) for p in airborne for b in boxes), default=math.inf)]
                    + [clear_to(o) for o in spawned])
    final_state = rows[-1]["state"]
    goal = truth["goal"]
    goal_err = math.hypot(pts[-1][0] - goal["x"], pts[-1][1] - goal["y"]) if goal else float("nan")
    # 목적지가 없는 지도(3단계)는 탐색을 마치고 출발점으로 돌아와 내려야 한다
    home_err = None if goal else math.hypot(pts[-1][0] - truth["spawn"]["x"], pts[-1][1] - truth["spawn"]["y"])
    dist = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    follow_t = [float(r["t"]) for r in rows if r["state"] == "FOLLOW"]
    flight_time = float(rows[-1]["t"]) - float(rows[0]["t"])
    states = [r["state"] for r in rows]
    entered = lambda s: sum(1 for a, b in zip(states, states[1:]) if b == s and a != s)  # noqa: E731

    print(f"최종 상태        {final_state}")
    print("위치 기준        " + ("정답 궤적으로 평가 (비행 코드는 추정 위치 사용)" if truth_rows else "비행 기록 위치로 평가"))
    if truth_rows and est_pts:
        errs = []
        k = 0
        for t, p in zip(times, pts):
            while k + 1 < len(est_times) and abs(est_times[k + 1] - t) <= abs(est_times[k] - t):
                k += 1
            if abs(est_times[k] - t) < 0.2:
                errs.append(math.hypot(est_pts[k][0] - p[0], est_pts[k][1] - p[1]))
        if errs:
            print(f"위치 추정 오차    평균 {sum(errs) / len(errs):.2f} m, 최대 {max(errs):.2f} m, "
                  f"착륙 때 {errs[-1]:.2f} m")
    print(f"목적지 수평 오차  {goal_err:.2f} m" if goal else f"출발점 복귀 오차  {home_err:.2f} m (목적지가 없는 지도)")
    print(f"장애물 최소 간격  {min_clear:.2f} m (드론 외곽 기준, 음수면 충돌)")
    print(f"비행 거리·시간    {dist:.2f} m, {flight_time:.1f} s (경로 추종 {follow_t[-1] - follow_t[0] if follow_t else 0:.1f} s)")

    first = original_plan(args.world_dir, truth) if hidden else None
    if hidden or spawned:
        unknown = hidden + spawned
        hidden_clear = min((clear_to(o) for o in unknown), default=math.inf)
        print(f"숨은 물건 간격    {hidden_clear:.2f} m (지도에 없던 물건 {len(hidden)}개"
              f"{f', 비행 중 생긴 것 {len(spawned)}개' if spawned else ''} 중 가장 가까운 것)")
        for o in spawned:
            seen = any(footprint_gap(o, f) < 0.3 for f in found)
            print(f"  비행 중 생성: {o['kind']} ({o['x']:.2f}, {o['y']:.2f}), 생길 때 {o['ahead']:.1f} m 앞 → "
                  f"{'찾음' if seen else '못 찾음'}, 간격 {clear_to(o):.2f} m")
        if first is not None:
            blocking = [o for o in hidden if not path_clear(   # 경로 선이 물건 상자를 그대로 지나가는지
                VoxelMap.from_known_map({"bounds": _bounds(truth), "obstacles": [o]}, 0.1), first)]
            names = ", ".join(o["kind"] for o in blocking) or "없음"
            print(f"원래 경로를 막은 물건 {len(blocking)}개 ({names})")
        print(f"재계획·비상 정지  {entered('REPLAN')}번, {entered('BACKOFF')}번, 위치 다시 잡기 {entered('RELOC')}번")
        # 비행 높이까지 닿는 숨은 물건 중 드론이 1.3 m 안으로 지나간 것 (낮은 물건은 위로 지나가므로 뺀다)
        tall = [o for o in unknown if o["sz"] >= 0.9]
        near = [o for o in tall if any(box_distance(p, o) < 1.3 for t, p in timed if t >= o.get("t", -math.inf))]
        hit = [o for o in near if any(footprint_gap(o, f) < 0.3 for f in found)]
        wrong = [f for f in found if all(footprint_gap(o, f) >= 0.3 for o in unknown + humans)]   # 사람도 맞힌 것
        by = {}
        for f in found:
            for s in f["sources"]:
                by[s] = by.get(s, 0) + 1
        src = ", ".join(f"{k} {v}" for k, v in sorted(by.items())) or "-"
        print(f"찾은 장애물      {len(found)}개 (센서별 {src})")
        print(f"  가까이 지나간 높은 숨은 물건 {len(near)}개 중 {len(hit)}개를 찾음, 엉뚱한 자리 {len(wrong)}개")

    people_path = args.world_dir / "people.json"
    if truth.get("humans") and people_path.exists():   # 3단계 사람 찾기
        found_p = json.loads(people_path.read_text(encoding="utf-8"))
        people = found_p["people"]
        matched = [h for h in truth["humans"] if any(math.hypot(h["x"] - q["x"], h["y"] - q["y"]) < 1.0 for q in people)]
        wrong_p = [q for q in people if all(math.hypot(h["x"] - q["x"], h["y"] - q["y"]) >= 1.0 for h in truth["humans"])]
        print(f"사람 찾기        {len(truth['humans'])}명 중 {len(matched)}명 찾음, 엉뚱한 자리 {len(wrong_p)}개, "
              f"본 곳 {100 * (found_p.get('coverage') or 0):.0f}%, 탐색 {found_p.get('search_time', 0):.0f} s")
        t0 = found_p.get("t_start")
        if people and t0:
            firsts = sorted(q["t_first"] - t0 for q in people)
            print(f"  처음 찾기까지 {firsts[0]:.0f} s" + (f", 마지막 사람까지 {firsts[-1]:.0f} s" if len(firsts) > 1 else ""))
        for h in truth["humans"]:
            near_q = min((math.hypot(h["x"] - q["x"], h["y"] - q["y"]) for q in people), default=math.inf)
            print(f"  {h['name']} ({h['x']:.1f}, {h['y']:.1f}): " + (f"찾음, 위치 오차 {near_q:.2f} m" if near_q < 1.0 else "못 찾음"))

    p = truth["params"]
    view = TopView(p["width"], p["depth"])
    for o in truth["obstacles"]:
        color = (220, 60, 60) if not o["in_known_map"] else (150, 150, 150)
        view.rect(o["x"] - o["sx"] / 2, o["y"] - o["sy"] / 2, o["x"] + o["sx"] / 2, o["y"] + o["sy"] / 2, color)
    for b in truth["walls"]:
        view.rect(b["x"] - b["sx"] / 2, b["y"] - b["sy"] / 2, b["x"] + b["sx"] / 2, b["y"] + b["sy"] / 2, (30, 30, 30))
    for o in spawned:   # 비행 중 생긴 장애물: 주황
        view.rect(o["x"] - o["sx"] / 2, o["y"] - o["sy"] / 2, o["x"] + o["sx"] / 2, o["y"] + o["sy"] / 2, (245, 120, 30))
    for f in found:
        outline(view, f, (150, 60, 200))
    if first is not None:
        for a, b in zip(first, first[1:]):
            n = max(1, int(math.dist(a[:2], b[:2]) / 0.1))
            for k in range(n + 1):
                x, y = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
                view.rect(x - 0.02, y - 0.02, x + 0.02, y + 0.02, (170, 170, 170))
    if truth_rows:   # 비행 코드가 믿은 추정 궤적: 하늘색
        for x, y, _ in est_pts[::2]:
            view.rect(x - 0.02, y - 0.02, x + 0.02, y + 0.02, (120, 190, 240))
    for x, y, _ in pts[::2]:   # 실제(정답) 궤적: 파랑
        view.rect(x - 0.03, y - 0.03, x + 0.03, y + 0.03, (40, 90, 220))
    for h in truth.get("humans", []):   # 사람: 분홍
        view.rect(h["x"] - 0.2, h["y"] - 0.2, h["x"] + 0.2, h["y"] + 0.2, (230, 90, 170))
    if people_path.exists():           # 찾은 사람 추정 위치: 보라 점
        for q in json.loads(people_path.read_text(encoding="utf-8"))["people"]:
            view.rect(q["x"] - 0.1, q["y"] - 0.1, q["x"] + 0.1, q["y"] + 0.1, (120, 40, 160))
    s = truth["spawn"]
    view.rect(s["x"] - 0.15, s["y"] - 0.15, s["x"] + 0.15, s["y"] + 0.15, (40, 170, 70))
    if goal:
        view.rect(goal["x"] - 0.15, goal["y"] - 0.15, goal["x"] + 0.15, goal["y"] + 0.15, (240, 160, 0))
    view.save(args.png or args.world_dir / "flight.png")

    ok = final_state == "DONE" and min_clear > 0 and (goal_err < 0.3 if goal else home_err < 0.3)
    print("결과            " + ("성공" if ok else "실패"))
    return 0 if ok else 1


def _bounds(truth: dict) -> dict:
    p = truth["params"]
    return {"xmin": 0, "ymin": 0, "xmax": p["width"], "ymax": p["depth"], "zmax": p["wall_height"]}


if __name__ == "__main__":
    sys.exit(main())
