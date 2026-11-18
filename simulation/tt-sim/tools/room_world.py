#!/usr/bin/env python3
"""실기체 시험 공간 지도(make_room_map.py)를 Gazebo 월드로 만든다. 실기체 비행 전에 같은 지도·경로로 미리 날려 본다.

지도의 벽·상자는 그대로 넣고, --hidden으로 지도에 없는 물건(실기체 비행 때 사람이 길에 놓을 물건)을 더한다.
드론이 받는 known_map.json은 입력 지도를 그대로 복사한다(실기체와 같은 지도). 지도에 없는 물건은 ground_truth.json에만 있다.
--known으로 드론이 받을 지도를 따로 주면(make_room_map.py --explore: 벽·가구 없음) 월드는 입력 지도대로 짓고
드론은 아무것도 모르는 채 카메라로 지도를 만들며 탐색한다(4단계, real.launch mode:=explore와 같은 조건).

  python3 tools/room_world.py rooms/lab/route_a_to_b.json --out worlds/lab_f1
  python3 tools/room_world.py rooms/lab/route_a_to_b.json --hidden 3.9 4.45 0.5 0.5 1.4 --out worlds/lab_f2
  python3 tools/room_world.py rooms/lab/search_lab.json --known rooms/lab/explore_lab.json --out worlds/lab_explore
  bash scripts/run_mission.sh worlds/lab_f2 --ext-tof --speed 0.2
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from worldgen.export import write_world  # noqa: E402
from worldgen.layout import Box, LayoutParams, Obstacle, Room, World  # noqa: E402


def _box(d: dict, name: str, texture: str = "paint") -> Box:
    return Box(name, d["x"], d["y"], d["z"], d["sx"], d["sy"], d["sz"], texture=texture)


def build(room: dict, hidden: list[tuple[float, ...]], kind: str = "box_stack", stage: int | None = None) -> World:
    b = room["bounds"]
    wall = -b["xmin"]   # make_room_map.py는 공간 바깥에 벽 두께만큼 둔다
    sx, sy = b["xmax"] - wall, b["ymax"] - wall
    params = LayoutParams(width=sx, depth=sy, wall_height=b["zmax"], humans=0,
                          flight_height=room.get("flight_height", 1.0))
    known = [Obstacle(_box(o, o["name"]), kind="cabinet", w=o["sx"], d=o["sy"]) for o in room["obstacles"]]
    extra = [Obstacle(Box(f"hidden_{i:02d}", h[0], h[1], h[4] / 2, h[2], h[3], h[4]), kind=kind, w=h[2], d=h[3],
                      hidden=True, blocker=True, seed=i) for i, h in enumerate(hidden)]
    return World(
        seed=0, stage=stage or (3 if room.get("goal") is None else 2 if hidden else 1), params=params,
        rooms=[Room(0.0, 0.0, sx, sy, kind="office", floor="tile")], doors=[],
        walls=[_box(w, w["name"]) for w in room["walls"]], lintels=[],
        floor=[Box("floor_00", sx / 2, sy / 2, -0.05, sx, sy, 0.1, texture="floor_0")],
        obstacles=known + extra, humans=[],
        spawn={**room["spawn"], "z": 0.1}, goal=room["goal"], decor=[], paint=(236, 234, 226),
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="시험 공간 지도 → Gazebo 월드")
    ap.add_argument("map", type=Path, help="make_room_map.py가 만든 지도 JSON")
    ap.add_argument("--hidden", nargs=5, type=float, action="append", default=[], metavar=("X", "Y", "SX", "SY", "SZ"),
                    help="지도에 없는 물건 (중심 x·y, 크기 m). 여러 번 줄 수 있다")
    ap.add_argument("--kind", default="box_stack", help="지도에 없는 물건 모양 (worldgen/furnish.py, 기본 상자 더미)")
    ap.add_argument("--out", type=Path, required=True, help="월드 폴더 (mission.launch.py world_dir)")
    ap.add_argument("--known", type=Path, default=None,
                    help="드론이 받을 지도 (기본: 입력 지도 그대로). make_room_map.py --explore 지도를 주면 4단계 월드")
    args = ap.parse_args()
    room = json.loads(args.map.read_text(encoding="utf-8"))
    known = json.loads(args.known.read_text(encoding="utf-8")) if args.known else room
    if args.known and (known["bounds"] != room["bounds"] or known["spawn"] != room["spawn"]):
        print("거절: --known 지도의 크기(bounds)·출발점(spawn)이 월드 지도와 다릅니다.", file=sys.stderr)
        return 1
    explore = args.known is not None and not known.get("walls")
    world = build(room, [tuple(h) for h in args.hidden], args.kind, stage=4 if explore else None)
    paths = write_world(world, args.out, human_model="simple")
    shutil.copyfile(args.known or args.map, paths["known_map"])   # 드론은 실기체와 같은 지도를 받는다
    s, g = room["spawn"], known["goal"]
    print(f"월드 {paths['world']}: 상자 {len(room['obstacles'])}개 + 지도에 없는 물건 {len(args.hidden)}개, "
          f"출발 ({s['x']}, {s['y']}) {math.degrees(s['yaw']):.0f}° → "
          + (f"목적지 ({g['x']}, {g['y']})" if g else "카메라로 지도 만들며 탐색(벽·가구 모름)" if explore else "탐색"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
