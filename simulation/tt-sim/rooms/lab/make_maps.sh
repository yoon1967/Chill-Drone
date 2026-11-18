#!/bin/bash
# 실습실(room_v2.json, 7.4 × 8.8 × 2.7 m) 비행 지도 → route_a_to_b.json
# --sim: 시뮬레이션 월드도 만든다 → tt-sim/worlds/lab_f1(비행 1), lab_f2(비행 2)
#
# 좌표는 room_v2.json과 같다: 원점 창문 벽 × 시계 벽 바닥 모서리, +x 출입문 벽, +y 스크린 벽.
# 가구 좌표는 손그림을 옮긴 추정치다(실측 아님). 책상은 의자 확인구역(R_*, 책상 둘레 0.5 m)까지 상자로 넣고,
# 높이는 2.4 m로 막는다. 책상 위로 지나가면 하방 ToF가 책상 면을 재서 높이가 틀어진다.
# 줄자로 잰 값이 0.2 m 넘게 다르면 아래 숫자를 고치고 다시 실행한다.
set -e
cd "$(dirname "$0")"
TOOLS=../../tools
H=2.4
BOXES=(
  --box 2.15 6.6 4.3 2.3 $H      # R_02 전면 긴 책상열 + 의자    x 0–4.3,     y 5.45–7.75
  --box 0.775 8.15 1.15 0.8 $H   # 01 전면 장비장·강연대          x 0.2–1.35,  y 7.75–8.55
  --box 0.7 3.95 1.4 3.6 $H      # R_03 창가 장비·작업 책상열 + 의자 x 0–1.4,   y 2.15–5.75
  --box 1.325 1.375 2.65 2.25 $H # R_04 후면 창가 작업 책상 + 의자  x 0–2.65,  y 0.25–2.5
  --box 3.775 1.9 1.85 3.1 $H    # R_05A 후면 회의책상 세로 + 의자  x 2.85–4.7, y 0.35–3.45
  --box 5.125 2.025 3.15 2.25 $H # R_05B 후면 회의책상 가로 + 의자  x 3.55–6.7, y 0.9–3.15
  --box 6.83 1.975 0.9 1.65 $H   # 06 이동식 디스플레이           x 6.38–7.28, y 1.15–2.8
  --box 7.075 6.2 0.65 1.6 $H    # 07 출입문 옆 보관장            x 6.75–7.4,  y 5.4–7.0
  --box 5.98 8.11 0.6 0.6 $H     # 08 스탠드 에어컨              x 5.68–6.28, y 7.81–8.41
)

# 비행 1·2: A (5.5, 4.45)에서 창문 벽(−x)을 보고 출발 → 가운데 통로 3.1 m → B (2.4, 4.45)
python3 $TOOLS/make_room_map.py --size 7.4 8.8 --start 5.5 4.45 --start-yaw 180 --goal 2.4 4.45 "${BOXES[@]}" \
  --out route_a_to_b.json
# 첫 실기체 비행용 짧은 길: 같은 출발에서 앞으로 2 m → (3.5, 4.45). 속도값 배율을 재고 나서 3.1 m로 늘린다
python3 $TOOLS/make_room_map.py --size 7.4 8.8 --start 5.5 4.45 --start-yaw 180 --goal 3.5 4.45 "${BOXES[@]}" \
  --out route_a_fwd2m.json

# 회전 확인: 같은 출발(창문 벽 쪽)에서 오른쪽으로 90° 돌아 스크린 벽 쪽 2 m → (5.5, 6.5).
# 속도값 기준(vel_frame)이 맞는지 탐색 전에 본다. 탐색은 계속 방향을 바꾼다
python3 $TOOLS/make_room_map.py --size 7.4 8.8 --start 5.5 4.45 --start-yaw 180 --goal 5.5 6.5 "${BOXES[@]}" \
  --out route_a_turn.json

# 방 탐색: 같은 출발에서 목적지 없이 아직 못 본 곳을 골라 둘러보다가 search_time이 지나면 돌아와 내린다
python3 $TOOLS/make_room_map.py --size 7.4 8.8 --start 5.5 4.45 --start-yaw 180 --search "${BOXES[@]}" \
  --out search_lab.json

# 지도 없이 둘러보기(mode:=wander): 테두리와 출발점만. 가구 상자는 넣지 않는다 (wander_node가 ToF로 스스로 찾는다)
python3 $TOOLS/make_room_map.py --size 7.4 8.8 --start 5.5 4.45 --start-yaw 180 --search --out wander_lab.json

# 카메라로 지도를 만들며 탐색(mode:=explore, 4단계): 벽·가구를 아예 적지 않는다. 드론이 깊이 영상과 ToF로 스스로 그린다
python3 $TOOLS/make_room_map.py --size 7.4 8.8 --start 5.5 4.45 --start-yaw 180 --explore --out explore_lab.json

# 시뮬레이션: 비행 1은 지도 그대로, 비행 2는 통로 가운데 (3.9, 4.45)에 지도에 없는 상자 더미 0.5 × 0.5 × 1.4 m
# lab_explore: 월드는 실습실 가구 그대로, 드론은 벽 없는 explore_lab.json을 받는다
if [ "$1" = "--sim" ]; then
  python3 $TOOLS/room_world.py route_a_to_b.json --out ../../worlds/lab_f1
  python3 $TOOLS/room_world.py route_a_to_b.json --hidden 3.9 4.45 0.5 0.5 1.4 --out ../../worlds/lab_f2
  python3 $TOOLS/room_world.py search_lab.json --hidden 3.9 4.45 0.5 0.5 1.4 --out ../../worlds/lab_search
  python3 $TOOLS/room_world.py search_lab.json --hidden 3.9 4.45 0.5 0.5 1.4 --known explore_lab.json --out ../../worlds/lab_explore
fi
