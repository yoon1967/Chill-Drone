#!/usr/bin/env bash
# 화면 없이 자율비행을 한 번 돌리고 결과만 낸다. 여러 맵을 연달아 시험할 때 쓴다.
#
# 사용: bash scripts/run_mission.sh worlds/s7_stage2
#       bash scripts/run_mission.sh worlds/s7_stage2 --detect      # YOLO도 함께 (가상환경 필요)
#       bash scripts/run_mission.sh worlds/s7_stage2 --no-avoid    # 회피 없이 (멈추고 중단) 비교용
#       bash scripts/run_mission.sh worlds/s7_stage1 --spawn 2     # 비행 중 드론 앞에 장애물 2개를 갑자기 만든다
#       bash scripts/run_mission.sh worlds/s7_stage1 --gt-pose     # 비교: 시뮬레이션 정답 위치로 비행
#       bash scripts/run_mission.sh worlds/s7_stage1 --no-map-fix  # 비교: 지도 보정 없이 속도 적분만
#       bash scripts/run_mission.sh worlds/s7_stage1 --direct      # 비교: Tello 드라이버·rc 없이 속도를 Gazebo에 바로
#       bash scripts/run_mission.sh worlds/s7_stage1 --seed 3      # Tello 상태값 잡음 seed
#       bash scripts/run_mission.sh worlds/lab_f2 --ext-tof --speed 0.2 # 실기체처럼
#       bash scripts/run_mission.sh worlds/lab_explore --ext-tof --speed 0.2 --fast 0.4 --depth  # 앞이 비면 0.4 m/s까지
#       bash scripts/run_mission.sh worlds/lab_explore --ext-tof --depth --tof-scale  # 깊이 배율을 정면 ToF로 (실기체와 같게, 11/04)
#       bash scripts/run_mission.sh worlds/lab_search --ext-tof --speed 0.2 --wander --search-time 60  # 지도 없이 ToF로 둘러보기: 전방 ToF 한 줄을 드라이버가 EXT tof?로
#       bash scripts/run_mission.sh worlds/door_test --ext-tof --speed 0.3 --wander --depth --doors  # + 카메라로 본 틈(문) 지나기 (기본은 끔)
#       bash scripts/run_mission.sh worlds/s7_stage3 --detect --search-time 240   # 3단계 사람 찾기
#       bash scripts/run_mission.sh worlds/s7_stage4 --detect --depth             # 4단계: 지도 없이 카메라로 지도 만들며 탐색
#       bash scripts/run_mission.sh worlds/s7_stage4 --detect --depth --localize  # 4단계 + 카메라 지도로 위치 바로잡기 (권장)
#       bash scripts/run_mission.sh worlds/s7_stage4 --detect --depth --record    # 깊이 영상·추정 위치를 WORLD_DIR/record에 기록
#       bash scripts/run_mission.sh worlds/s7_stage4 --detect --depth --web       # 웹 화면(http://localhost:8080)도 띄워 보며 시험 (시작은 이 스크립트가 누름)
# 기본은 정답 위치 없이(추정 위치로), 실기체와 같은 Tello 드라이버(rc·takeoff·land)를 거쳐 난다.
# 정답은 평가용 truth_log.csv에만 남는다.
#       bash scripts/run_mission.sh worlds/s7_stage2 --timeout 400 # 기본 300초(실제 시간)
#
# 기록: WORLD_DIR/flight_log.csv, discovered.json, flight.png, mission_run.log
source /opt/ros/jazzy/setup.bash
WORLD_DIR=$(realpath "${1:?world 폴더를 주세요. 예: bash scripts/run_mission.sh worlds/s7_stage2}")
shift
DETECT=false
DEPTH=false
LOCALIZE=false
RECORD=false
AVOID=true
SPAWN=0
POSE=est
POSE_MAP=true
LINK=tello
SEED=0
SEARCH_TIME=300
TIMEOUT=300
TOF=direct
SPEED=0.4
FAST=0
RC_LIMIT=30
ARC=0.12
MODE=route
DOORS=false
SCALE_MODE=floor
WEB=false
while [ $# -gt 0 ]; do
  case "$1" in
    --detect) DETECT=true ;;
    --depth) DEPTH=true ;;
    --localize) LOCALIZE=true ;;
    --record) RECORD=true ;;
    --no-avoid) AVOID=false ;;
    --spawn) SPAWN=$2; shift ;;
    --gt-pose) POSE=gt ;;
    --no-map-fix) POSE_MAP=false ;;
    --direct) LINK=direct ;;
    --seed) SEED=$2; shift ;;
    --search-time) SEARCH_TIME=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    --ext-tof) TOF=ext ;;
    --speed) SPEED=$2; shift ;;
    --fast) FAST=$2; shift ;;
    --rc-limit) RC_LIMIT=$2; shift ;;
    --arc-speed) ARC=$2; shift ;;
    --wander) MODE=wander ;;
    --doors) DOORS=true ;;
    --tof-scale) SCALE_MODE=tof ;;
    --web) WEB=true ;;
    *) echo "모르는 옵션: $1"; exit 1 ;;
  esac
  shift
done
ROOT=$(cd "$(dirname "$0")/.." && pwd)
LOG="$WORLD_DIR/mission_run.log"
[ -f "$WORLD_DIR/world.sdf" ] || { echo "$WORLD_DIR/world.sdf가 없습니다. generate_world.py로 먼저 만드세요."; exit 1; }

cleanup() {
  pkill -f "[m]ission_node.py"; pkill -f "[d]etector_node.py"; pkill -f "[d]epth_node.py"; pkill -f "[s]pawn_obstacles.py"
  pkill -f "[s]im_tello_state.py"; pkill -f "[p]ose_node.py"; pkill -f "[l]og_truth.py"
  pkill -f "[g]z_tello.py"; pkill -f "[t]ello_driver_node.py"; pkill -f "[a]utonomy_web.py"
  pkill -f "[r]os2 launch.*mission.launch.py"; pkill -f "[p]arameter_bridge"; pkill -f "[l]atency_relay"
  pkill -9 -f "[g]z sim"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# 이전 실행(화면 보기 포함)이 남아 있으면 같은 토픽에 명령을 섞어 보내므로 먼저 정리한다
pkill -f "[w]atch_mission.sh"; pkill -f "[r]qt_image_view"
cleanup
sleep 1

# 지난 실행의 mission_run.log도 지운다. 남아 있으면 새 실행이 그 파일을 비우기 전에 아래 대기가 지난 'DONE'을 읽고
# 바로 끝나, 날지도 않고 '비행 기록이 비어 있습니다'로 끝난 적이 있다
rm -f "$WORLD_DIR/flight_log.csv" "$WORLD_DIR/discovered.json" "$WORLD_DIR/spawned.json" "$WORLD_DIR/truth_log.csv" \
  "$WORLD_DIR/people.json" "$WORLD_DIR/pose_inputs.csv" "$WORLD_DIR/camera_map.npz" "$LOG"
rm -rf "$WORLD_DIR/tello_log"
echo "$(basename "$WORLD_DIR"): 명령 경로 $LINK, 전방 ToF $TOF, 속도 $SPEED, 위치 $POSE(지도 보정 $POSE_MAP), 회피 $AVOID, YOLO $DETECT, 깊이 $DEPTH, 위치 맞추기 $LOCALIZE, 기록 $RECORD, 비행 중 생성 $SPAWN개"
# ros2 launch는 빈 값(record:=)을 잘못된 인자로 거절하므로 기록할 때만 넘긴다
REC_ARG=()
[ "$RECORD" = true ] && REC_ARG=(record:="$WORLD_DIR/record")
ros2 launch "$ROOT/launch/mission.launch.py" world_dir:="$WORLD_DIR" headless:=true \
  detect:=$DETECT depth:=$DEPTH localize:=$LOCALIZE avoid:=$AVOID spawn:=$SPAWN pose:=$POSE pose_map:=$POSE_MAP seed:=$SEED link:=$LINK \
  search_time:=$SEARCH_TIME tof:=$TOF speed:=$SPEED fast:=$FAST rc_limit:=$RC_LIMIT arc_speed:=$ARC mode:=$MODE doors:=$DOORS depth_scale_mode:=$SCALE_MODE web:=$WEB "${REC_ARG[@]}" > "$LOG" 2>&1 &
if [ "$WEB" = true ]; then   # 웹 화면을 켜면 비행 노드가 [자율비행 시작]을 기다린다: 준비되면 대신 누른다
  ( for _ in $(seq 1 100); do
      grep -q "시작 요청을 받았습니다" "$LOG" 2>/dev/null && break
      grep -q "상태: READY" "$LOG" 2>/dev/null && curl -s -X POST http://127.0.0.1:8080/api/mission/start > /dev/null
      sleep 3
    done ) &
fi
end=$((SECONDS + TIMEOUT))
last_size=-1
still=0
until grep -qE "상태: (DONE|ABORT)|Traceback" "$LOG" 2>/dev/null; do
  if [ $SECONDS -ge $end ]; then echo "시간 초과 (${TIMEOUT}초)"; break; fi
  # 비행 기록은 시뮬레이션 시간 0.05초마다 한 줄씩 는다. 30초 동안 그대로면 Gazebo가 멈춘 것이다
  size=$(stat -c %s "$WORLD_DIR/flight_log.csv" 2>/dev/null || echo 0)
  if [ "$size" -gt 0 ] && [ "$size" = "$last_size" ]; then still=$((still + 2)); else still=0; fi
  last_size=$size
  if [ $still -ge 30 ]; then
    echo "시뮬레이터 멈춤: 30초 동안 시뮬레이션 시간이 흐르지 않았습니다 (비행 코드와 무관한 Gazebo 문제일 수 있음)"
    break
  fi
  sleep 2
done
sleep 1
grep -E "장애물 생성|지도에 없는|막혔|새 경로|비상|제자리|계획 실패|불확실|사람 발견|다음 볼 자리|돌아갑니다|탐색 결과|착륙 완료|드라이버|takeoff 결과|land 결과|끊겼|Traceback|Error|돌아 |물러|울타리|둘러보기 시간|기둥|벽\(|이어집니다|비켜|아무도|비어 " "$LOG" \
  | sed -E 's/^.*\[tt_mission\]: /  /; s/^.*\[tt_detector\]: /  [YOLO] /; s/^.*\[tt_obstacle_spawner\]: /  [생성기] /; s/^.*\[tt_tello_driver\]: /  [드라이버] /'
python3 "$ROOT/tools/eval_flight.py" "$WORLD_DIR"
python3 "$ROOT/tools/eval_dwell.py" "$WORLD_DIR/flight_log.csv" --summary
[ -f "$WORLD_DIR/camera_map.npz" ] && python3 "$ROOT/tools/eval_map.py" "$WORLD_DIR"
