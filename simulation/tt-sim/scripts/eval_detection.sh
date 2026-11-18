#!/usr/bin/env bash
# 사람 인식 성능 평가: Gazebo를 화면 없이 띄우고 tools/eval_detection.py를 돌린 뒤 정리한다.
#
# 사용: bash scripts/eval_detection.sh worlds/s7_stage3            (YOLO 가상환경 기본값 ~/venvs/tt)
#       bash scripts/eval_detection.sh worlds/s7_stage3 --distances 1,3,5
source /opt/ros/jazzy/setup.bash
WORLD_DIR=$(realpath "${1:?world 폴더를 주세요. 예: bash scripts/eval_detection.sh worlds/s7_stage3}")
shift
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PY="${TT_VENV:-$HOME/venvs/tt}/bin/python"
[ -x "$PY" ] || { echo "$PY가 없습니다. 먼저 bash scripts/setup_yolo.sh를 실행하세요."; exit 1; }

cleanup() {
  pkill -f "[r]os2 launch.*sim.launch.py"; pkill -f "[p]arameter_bridge"; pkill -f "[l]atency_relay"
  pkill -9 -f "[g]z sim"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

echo "Gazebo를 화면 없이 띄우는 중..."
ros2 launch "$ROOT/launch/sim.launch.py" world:="$WORLD_DIR/world.sdf" headless:=true > /tmp/tt_eval_sim.log 2>&1 &
for _ in $(seq 1 60); do gz service -l 2>/dev/null | grep -q "^/world/tt_world/set_pose$" && break; sleep 1; done
sleep 8   # 사람 모델·센서가 다 올라올 때까지
"$PY" "$ROOT/tools/eval_detection.py" "$WORLD_DIR" "$@" --ros-args -p use_sim_time:=true
