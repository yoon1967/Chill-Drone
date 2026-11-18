#!/usr/bin/env bash
# Gazebo 화면을 띄우고, 카메라가 드론을 따라가게 한 뒤 자율비행을 시작한다.
# 비행이 끝나면 결과를 출력하고 창은 열어 둔다. Ctrl+C를 누르면 Gazebo 창까지 함께 닫는다.
#
# 사용: bash scripts/watch_mission.sh worlds/s7_stage1
#       bash scripts/watch_mission.sh worlds/s7_stage2 --detect   # YOLO 탐지 화면을 함께 띄운다
#       bash scripts/watch_mission.sh worlds/s7_stage1 --spawn 2  # 비행 중 드론 앞에 장애물 2개를 갑자기 만든다
#       (둘 다 줄 수 있다: --detect --spawn 2)
#       --gt-pose: 비교용으로 시뮬레이션 정답 위치로 비행 (기본은 정답 없이 추정 위치로 비행)
#       --direct: 비교용으로 Tello 드라이버 없이 속도 명령을 Gazebo에 바로 (기본은 실기체와 같은 드라이버·rc 경로)
#
# 지도에 없는 장애물을 찾으면 지도에 더하고 경로를 다시 짠다 (찾은 것은 WORLD_DIR/discovered.json).
# 목적지가 없는 지도(3단계)는 사람 찾기 탐색을 한다: 못 본 곳을 골라 둘러보고, 끝나면 출발점으로 돌아와 착륙.
source /opt/ros/jazzy/setup.bash
WORLD_DIR=$(realpath "${1:?world 폴더를 주세요. 예: bash scripts/watch_mission.sh worlds/s7_stage1}")
shift
DETECT=0
SPAWN=0
POSE=est
LINK=tello
while [ $# -gt 0 ]; do
  case "$1" in
    --detect) DETECT=1 ;;
    --spawn) SPAWN=$2; shift ;;
    --gt-pose) POSE=gt ;;
    --direct) LINK=direct ;;
    *) echo "모르는 옵션: $1"; exit 1 ;;
  esac
  shift
done
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DRIVER="$ROOT/../../modules/autonomy/tello_driver/tello_driver_node.py"
DRIVER_LOG=/tmp/tt_driver.log
PY="${TT_VENV:-$HOME/venvs/tt}/bin/python"
SIM_LOG=/tmp/tt_sim.log
MISSION_LOG=/tmp/tt_mission.log
DETECT_LOG=/tmp/tt_detector.log
SPAWN_LOG=/tmp/tt_spawner.log
POSE_LOG=/tmp/tt_pose.log
[ -f "$WORLD_DIR/world.sdf" ] || { echo "$WORLD_DIR/world.sdf가 없습니다. generate_world.py로 먼저 만드세요."; exit 1; }
if [ "$LINK" = tello ] && [ ! -f "$DRIVER" ]; then
  echo "$DRIVER가 없습니다. 팀 저장소 전체를 받거나 --direct로 실행하세요."; exit 1
fi
if [ "$DETECT" = 1 ] && [ ! -x "$PY" ]; then
  echo "$PY가 없습니다. 먼저 bash scripts/setup_yolo.sh를 실행하세요."; exit 1
fi

cleanup() {
  echo; echo "정리하는 중..."
  pkill -f "[m]ission_node.py"; pkill -f "[d]etector_node.py"; pkill -f "[r]qt_image_view"; pkill -f "[s]pawn_obstacles.py"
  pkill -f "[s]im_tello_state.py"; pkill -f "[p]ose_node.py"; pkill -f "[l]og_truth.py"
  pkill -f "[g]z_tello.py"; pkill -f "[t]ello_driver_node.py"
  pkill -f "[r]os2 launch.*sim.launch.py"; pkill -f "[p]arameter_bridge"; pkill -f "[l]atency_relay"
  pkill -9 -f "[g]z sim"   # WSL에서는 Gazebo 화면이 일반 종료 신호를 무시할 때가 있다
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# 이전 실행이 남아 있으면 같은 토픽에 명령을 섞어 보내므로 먼저 정리한다
pkill -f "[r]un_mission.sh"; pkill -f "[r]os2 launch.*mission.launch.py"
cleanup > /dev/null
sleep 1

echo "Gazebo를 띄우는 중... (처음엔 사람 모델을 내려받느라 조금 걸릴 수 있어요)"
CMD_TOPIC=/tt/cmd_vel
[ "$LINK" = tello ] && CMD_TOPIC=/tt/sim/cmd_vel
ros2 launch "$ROOT/launch/sim.launch.py" world:="$WORLD_DIR/world.sdf" cmd_topic:="$CMD_TOPIC" > "$SIM_LOG" 2>&1 &
for _ in $(seq 1 90); do gz service -l 2>/dev/null | grep -q "^/gui/follow$" && break; sleep 1; done
sleep 5   # 월드와 드론이 다 올라올 때까지 기다린다

gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'data: "tt_drone"' > /dev/null
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'x: -2.0, y: 0.0, z: 1.6' > /dev/null
echo "카메라가 드론 뒤 위쪽에서 따라갑니다."

if [ "$DETECT" = 1 ]; then
  echo "YOLO 탐지를 켭니다. 사람을 잡은 사진은 $WORLD_DIR/detections/ 에 저장됩니다."
  "$PY" "$ROOT/nodes/detector_node.py" --ros-args -p use_sim_time:=true \
    -p out_dir:="$WORLD_DIR/detections" > "$DETECT_LOG" 2>&1 &
  for _ in $(seq 1 60); do grep -q "YOLO .* 준비" "$DETECT_LOG" && break; sleep 1; done
  ros2 run rqt_image_view rqt_image_view /tt/detections/image > /dev/null 2>&1 &
  tail --pid=$$ -n +1 -f "$DETECT_LOG" | grep --line-buffered -E "사람 탐지|추론|Error|Traceback" \
    | sed -u 's/^\[[^]]*\] \[[^]]*\] \[tt_detector\]: /[YOLO] /' &
fi
rm -f "$WORLD_DIR/spawned.json"
if [ "$SPAWN" -gt 0 ]; then
  echo "비행 중에 드론 앞 2–3 m에 장애물 ${SPAWN}개를 갑자기 만듭니다."
  python3 "$ROOT/tools/spawn_obstacles.py" --ros-args -p world_dir:="$WORLD_DIR" -p count:="$SPAWN" \
    -p use_sim_time:=true > "$SPAWN_LOG" 2>&1 &
  tail --pid=$$ -n +1 -f "$SPAWN_LOG" | grep --line-buffered -E "장애물 생성|실패|Error|Traceback" \
    | sed -u 's/^\[[^]]*\] \[[^]]*\] \[tt_obstacle_spawner\]: /[생성기] /' &
fi
rm -f "$WORLD_DIR/truth_log.csv"
python3 "$ROOT/tools/log_truth.py" --ros-args -p out:="$WORLD_DIR/truth_log.csv" -p use_sim_time:=true   > /dev/null 2>&1 &
if [ "$LINK" = tello ]; then
  echo "실기체와 같은 Tello 드라이버로 납니다: 비행 노드 → 드라이버 → UDP SDK 명령(rc·takeoff·land) → Gazebo Tello 흉내."
  python3 "$ROOT/nodes/gz_tello.py" --ros-args -p cmd_port:=28889 -p state_port:=28890 -p use_sim_time:=true > "$DRIVER_LOG" 2>&1 &
  python3 "$DRIVER" --ros-args -p host:=127.0.0.1 -p bind_ip:=127.0.0.1 -p cmd_port:=28889 -p local_cmd_port:=28888 \
    -p state_port:=28890 -p operator_confirmed:=true -p source:=gazebo -p log_root:="$WORLD_DIR/tello_log" \
    -p use_sim_time:=true >> "$DRIVER_LOG" 2>&1 &
fi
if [ "$POSE" = est ]; then
  echo "정답 위치 없이 납니다: Tello 상태값 → 위치 추정 → 비행."
  [ "$LINK" = direct ] && python3 "$ROOT/nodes/sim_tello_state.py" --ros-args -p use_sim_time:=true > "$POSE_LOG" 2>&1 &
  python3 "$ROOT/nodes/pose_node.py" --ros-args -p map:="$WORLD_DIR/known_map.json"     -p use_sim_time:=true >> "$POSE_LOG" 2>&1 &
  POSE_TOPIC=/tt/pose_est
else
  echo "비교용: 시뮬레이션 정답 위치로 납니다."
  POSE_TOPIC=/tt/odometry_gt
fi
echo "3초 뒤 자율비행을 시작합니다."
sleep 3

python3 "$ROOT/nodes/mission_node.py" --ros-args -p map:="$WORLD_DIR/known_map.json" \
  -p use_sim_time:=true -p log_csv:="$WORLD_DIR/flight_log.csv" \
  -p discovered_out:="$WORLD_DIR/discovered.json" -p people_out:="$WORLD_DIR/people.json" \
  -p pose_topic:="$POSE_TOPIC" -p driver:="$([ "$LINK" = tello ] && echo true || echo false)" > "$MISSION_LOG" 2>&1 &
MISSION_PID=$!
# 비행 노드가 끝나면 tail도 같이 끝난다
tail --pid="$MISSION_PID" -n +1 -f "$MISSION_LOG" | grep --line-buffered -E "상태|경로|착륙|목적지|장애물|실패|재계획|제자리|비상|불확실|볼 자리|사람 발견|돌아갑니다|탐색 결과|Error|Traceback" \
  | sed -u 's/^\[[^]]*\] \[[^]]*\] \[tt_mission\]: //' &

until grep -qE "상태: (DONE|ABORT)|Traceback" "$MISSION_LOG"; do sleep 1; done
sleep 1
echo
python3 "$ROOT/tools/eval_flight.py" "$WORLD_DIR"
echo
echo "Gazebo 창은 열어 두었어요. 다 봤으면 이 터미널에서 Ctrl+C를 누르세요."
while true; do sleep 1; done
