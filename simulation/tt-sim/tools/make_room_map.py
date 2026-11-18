#!/usr/bin/env python3
"""실기체 시험 공간의 지도(known_map.json)를 손으로 만든다. 줄자로 잰 사각형 공간 + (선택) 장애물 상자.

좌표: 출발점 쪽 한 모서리를 원점(0, 0)으로, 드론이 처음 바라보는 방향을 기준으로 x(앞)·y(왼쪽)를 잡으면 헷갈리지 않는다.
드론은 --start 자리에 --start-yaw 방향(도, 기본 0 = +x)을 보게 놓고 시작한다. 공간 테두리는 벽으로 넣어
경로가 테두리에서 드론 반지름(0.25 m)만큼 떨어지게 한다.

  python3 tools/make_room_map.py --size 4 3 --start 0.6 1.5 --goal 3.2 1.5 --out ~/rooms/lab/known_map.json
  python3 tools/make_room_map.py --size 5 4 --start 0.6 2 --goal 4.2 2 --box 2.5 2 0.6 0.6 1.2 --out ...  # 가운데 상자

목적지는 공간 안, 테두리·장애물에서 --margin(기본 0.5 m) 넘게 떨어져야 받아들인다.
--search면 목적지 없는 지도(3단계 탐색)를 만든다: 비행 노드가 아직 못 본 곳을 골라 둘러보다가
search_time이 지나거나 더 볼 곳이 없으면 출발점으로 돌아와 착륙한다.

  python3 tools/make_room_map.py --size 5 4 --start 0.6 2 --search --box 2.5 2 0.6 0.6 2.4 --out ~/rooms/lab/search.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

WALL = 0.1   # 테두리 벽 두께 (m). 공간 안쪽 면이 잰 크기와 맞도록 바깥에 둔다


def make_map(size_x: float, size_y: float, start: tuple[float, float], start_yaw_deg: float,
             goal: tuple[float, float] | None, boxes: list[tuple[float, ...]], height: float = 1.0,
             ceiling: float = 2.5, explore: bool = False) -> dict:
    sx, sy = size_x, size_y
    walls = [
        {"name": "wall_south", "x": sx / 2, "y": -WALL / 2, "z": ceiling / 2, "sx": sx + 2 * WALL, "sy": WALL, "sz": ceiling},
        {"name": "wall_north", "x": sx / 2, "y": sy + WALL / 2, "z": ceiling / 2, "sx": sx + 2 * WALL, "sy": WALL, "sz": ceiling},
        {"name": "wall_west", "x": -WALL / 2, "y": sy / 2, "z": ceiling / 2, "sx": WALL, "sy": sy, "sz": ceiling},
        {"name": "wall_east", "x": sx + WALL / 2, "y": sy / 2, "z": ceiling / 2, "sx": WALL, "sy": sy, "sz": ceiling},
    ]
    obstacles = [{"name": f"box_{i:02d}", "x": b[0], "y": b[1], "z": b[4] / 2, "sx": b[2], "sy": b[3], "sz": b[4],
                  "kind": "box"} for i, b in enumerate(boxes)]
    if explore:   # 4단계: 벽·가구를 적지 않는다. 비행 노드는 벽 없는 지도를 받으면 카메라 깊이로 지도를 만들며 탐색한다
        walls, obstacles = [], []
    return {
        "format": "tt-sim/known-map@2",
        "stage": 4 if explore else 1 if goal else 3,
        "description": f"실기체 시험 공간 {sx:g}×{sy:g} m (make_room_map.py"
                       f"{', 카메라 탐색(벽·가구 없음)' if explore else '' if goal else ', 탐색'})",
        "frame": "world: 출발점 쪽 모서리가 원점, x·y는 잰 방향, z 위, 단위 m. 상자는 중심(x, y, z)과 크기(sx, sy, sz)",
        "bounds": {"xmin": -WALL, "ymin": -WALL, "xmax": sx + WALL, "ymax": sy + WALL, "zmax": ceiling},
        "flight_height": height,
        "spawn": {"x": start[0], "y": start[1], "z": 0.0, "yaw": round(math.radians(start_yaw_deg), 4)},
        "goal": {"x": goal[0], "y": goal[1], "z": height} if goal else None,
        "walls": walls,
        "lintels": [],
        "obstacles": obstacles,
    }


def open_map(size: float = 20.0, height: float = 1.0, ceiling: float = 2.5,
             center: tuple[float, float] = (0.0, 0.0), yaw: float = 0.0) -> dict:
    """지도 없이 띄울 때 쓰는 빈 지도: 한 변 size m인 정사각형, 출발점은 가운데(center), 출발 방향 yaw(라디안, 기본 +x).

    실습실 밖(복도 등)에서 mode:=explore·wander로 날릴 때 real.launch가 map:=를 비우면 이것을 만든다 (2026-10-30 복도 비행에
    실습실 지도를 그대로 써서 방 크기·탐색 목표가 맞지 않았다). 벽·가구·목적지는 없고 드론이 카메라·ToF로 찾는다.
    """
    half = size / 2
    return {
        "format": "tt-sim/known-map@2",
        "stage": 4,
        "description": f"빈 지도 {size:g}×{size:g} m (make_room_map.py open_map: 지도 없이 탐색, 출발점이 가운데)",
        "frame": "world: 출발점이 가운데, x는 출발 방향, y 왼쪽, z 위, 단위 m. 벽·가구 없음(드론이 카메라·ToF로 찾는다)",
        "bounds": {"xmin": center[0] - half, "ymin": center[1] - half, "xmax": center[0] + half, "ymax": center[1] + half,
                   "zmax": ceiling},
        "flight_height": height,
        "spawn": {"x": center[0], "y": center[1], "z": 0.0, "yaw": round(yaw, 4)},
        "goal": None,
        "walls": [],
        "lintels": [],
        "obstacles": [],
        "open": True,
    }


def clearance(p: tuple[float, float], size_x: float, size_y: float, boxes: list[tuple[float, ...]]) -> float:
    """점에서 테두리·장애물 상자까지 가장 가까운 거리 (안에 있으면 음수)."""
    d = min(p[0], size_x - p[0], p[1], size_y - p[1])
    for b in boxes:
        dx = abs(p[0] - b[0]) - b[2] / 2
        dy = abs(p[1] - b[1]) - b[3] / 2
        d = min(d, math.hypot(max(dx, 0), max(dy, 0)) if dx > 0 or dy > 0 else max(dx, dy))
    return d


def check(name: str, p: tuple[float, float], size_x: float, size_y: float, boxes, margin: float) -> str | None:
    c = clearance(p, size_x, size_y, boxes)
    if c < 0:
        return f"{name} ({p[0]:g}, {p[1]:g})가 공간 밖이거나 장애물 안입니다."
    if c < margin:
        return f"{name} ({p[0]:g}, {p[1]:g})가 테두리·장애물에서 {c:.2f} m라 {margin:g} m보다 가깝습니다."
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description="실기체 시험 공간 지도 만들기")
    ap.add_argument("--size", nargs=2, type=float, metavar=("X", "Y"), help="공간 크기 (m). --open이 아니면 필수")
    ap.add_argument("--start", nargs=2, type=float, metavar=("X", "Y"), help="출발점 (m). --open이 아니면 필수")
    ap.add_argument("--start-yaw", type=float, default=0.0, help="출발 때 드론이 보는 방향 (도, 0 = +x, 반시계 +)")
    target = ap.add_mutually_exclusive_group(required=True)
    target.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"), help="목적지 (m)")
    target.add_argument("--search", action="store_true", help="목적지 없이 방을 둘러보는 탐색 지도")
    target.add_argument("--explore", action="store_true",
                        help="벽·가구 없이 테두리와 출발점만 적는 지도: 카메라 깊이로 지도를 만들며 탐색 (4단계, real.launch mode:=explore)")
    target.add_argument("--open", type=float, metavar="SIZE",
                        help="빈 지도: 한 변 SIZE m, 출발점이 가운데 (0, 0). 실습실 밖에서 탐색·둘러보기 (real.launch는 map:=을 비우면 알아서 만든다)")
    ap.add_argument("--box", nargs=5, type=float, action="append", default=[], metavar=("X", "Y", "SX", "SY", "SZ"),
                    help="장애물 상자: 중심 x y, 크기 sx sy sz (m). 여러 번 줄 수 있다")
    ap.add_argument("--height", type=float, default=1.0, help="비행 높이 (m)")
    ap.add_argument("--margin", type=float, default=0.5, help="출발점·목적지가 테두리·장애물에서 떨어져야 하는 거리 (m)")
    ap.add_argument("--out", required=True, help="쓸 known_map.json 경로")
    args = ap.parse_args()
    if args.open:
        out = Path(args.out).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(open_map(args.open, args.height), ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"지도: {out} (빈 지도 {args.open:g}×{args.open:g} m, 출발점 가운데 (0, 0), 목적지 없음, 벽·가구 없음)")
        return 0
    if args.size is None or args.start is None:
        ap.error("--size와 --start가 필요합니다 (--open이 아니면)")
    sx, sy = args.size
    boxes = [tuple(b) for b in args.box]
    goal = None if args.search or args.explore else tuple(args.goal)
    errors = [e for e in (check("출발점", tuple(args.start), sx, sy, boxes, args.margin),
                          goal and check("목적지", goal, sx, sy, boxes, args.margin)) if e]
    if args.explore and boxes:
        errors.append("--explore 지도에는 --box를 넣지 않습니다 (벽·가구는 드론이 카메라로 찾습니다).")
    if sx <= 0 or sy <= 0:
        errors.append("공간 크기는 0보다 커야 합니다.")
    if goal and math.dist(args.start, goal) < 0.5:
        errors.append("출발점과 목적지가 0.5 m보다 가깝습니다.")
    if errors:
        for e in errors:
            print("거절:", e, file=sys.stderr)
        return 1
    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(make_map(sx, sy, tuple(args.start), args.start_yaw, goal, boxes, args.height,
                                       explore=args.explore),
                              ensure_ascii=False, indent=1), encoding="utf-8")
    target = (f"목적지 ({goal[0]:g}, {goal[1]:g}), 직선 거리 {math.dist(args.start, goal):.2f} m" if goal
              else "목적지 없음(카메라 탐색, 벽·가구 없음)" if args.explore else "목적지 없음(탐색)")
    print(f"지도: {out} ({sx:g}×{sy:g} m, 출발 ({args.start[0]:g}, {args.start[1]:g}) {args.start_yaw:g}°, "
          f"{target}, 장애물 {len(boxes)}개)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
