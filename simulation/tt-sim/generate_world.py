#!/usr/bin/env python3
"""랜덤 실내 맵을 만들어 Gazebo 월드, 드론용 지도, 평가용 정답, 미리보기를 쓴다.

예)
  python3 generate_world.py --seed 7 --stage 2
  python3 generate_world.py --seed 7 --stage 2 --blockers 2   # 최단 경로 위에 지도에 없는 물건 2개
  python3 generate_world.py --seed 7 --stage 3 --humans 4 --human-model simple
"""
from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path

from worldgen import STAGES, LayoutParams, generate, write_world

ROOT = Path(__file__).resolve().parent
ROOM_NAMES = {"office": "사무실", "living": "거실", "bedroom": "침실", "meeting": "회의실", "storage": "창고"}


def main(argv: list[str] | None = None) -> None:
    defaults = LayoutParams()
    parser = argparse.ArgumentParser(description="TT 시뮬레이션용 랜덤 실내 맵 생성기")
    parser.add_argument("--seed", type=int, default=0, help="같은 seed는 단계가 달라도 같은 건물을 만든다")
    parser.add_argument("--stage", type=int, choices=sorted(STAGES), default=1,
                        help="; ".join(f"{k}: {v['desc']}" for k, v in STAGES.items()))
    parser.add_argument("--out", type=Path, help="출력 폴더 (기본: worlds/s{seed}_stage{stage})")
    parser.add_argument("--width", type=float, default=defaults.width, help="맵 가로 (m)")
    parser.add_argument("--depth", type=float, default=defaults.depth, help="맵 세로 (m)")
    parser.add_argument("--humans", type=int, default=defaults.humans, help="3·4단계 사람 수")
    parser.add_argument("--clutter", type=int, default=None,
                        help="지도에 없는 물건(상자·라바콘·카트·의자) 수. 기본은 방 수에 맞춰 자동")
    parser.add_argument("--blockers", type=int, default=0,
                        help="출발점→목적지 최단 경로 위에 일부러 놓는 지도에 없는 물건 수 (2단계 회피 시험용)")
    parser.add_argument("--human-model", choices=["actor", "simple"], default="actor",
                        help="actor: Fuel 사람 메시(인터넷 필요), simple: 원기둥 모양(오프라인)")
    args = parser.parse_args(argv)

    params = LayoutParams(width=args.width, depth=args.depth, humans=args.humans, clutter=args.clutter,
                          blockers=args.blockers)
    world = generate(args.seed, args.stage, params)
    out = args.out or ROOT / "worlds" / f"s{args.seed}_stage{args.stage}"
    paths = write_world(world, out, human_model=args.human_model)

    known = sum(world.is_known(o) for o in world.obstacles)
    kinds = Counter(ROOM_NAMES[r.kind] for r in world.rooms)
    print(f"[stage {world.stage}] {STAGES[world.stage]['desc']}")
    print(f"  방 {len(world.rooms)}개 ({', '.join(f'{k} {v}' for k, v in kinds.items())}), 문 {len(world.doors)}개")
    blockers = [o for o in world.obstacles if o.blocker]
    print(f"  가구·물건 {len(world.obstacles)}개 (지도에 있음 {known} / 없음 {len(world.obstacles) - known}), "
          f"사람 {len(world.humans)}명")
    if blockers:
        print(f"  경로를 막는 물건 {len(blockers)}개: " + ", ".join(f"{o.kind} ({o.box.x:.1f}, {o.box.y:.1f})" for o in blockers))
    print(f"  출발점 {world.spawn}")
    print(f"  목적지 {world.goal if world.goal else '없음'}")
    for name, path in paths.items():
        print(f"  {name:12s} {path}")


if __name__ == "__main__":
    main()
