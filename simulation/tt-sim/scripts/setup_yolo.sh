#!/usr/bin/env bash
# YOLO(ultralytics)를 ROS 2와 함께 쓸 파이썬 가상환경을 만든다. 일반 계정으로 실행한다.
#
# 사용: bash scripts/setup_yolo.sh            → ~/venvs/tt 에 만든다
#       bash scripts/setup_yolo.sh <경로>
#
# ROS 2 메시지의 C 확장은 numpy 1.x로 빌드돼 있어서 numpy 2를 쓰면 일부 메시지가 깨진다.
# 그래서 numpy<2, 그와 맞는 opencv-python<4.11을 함께 고정한다.
set -e
VENV="${1:-$HOME/venvs/tt}"

if ! python3 -c "import ensurepip" 2>/dev/null; then
  echo "python3-venv가 없습니다. 먼저 실행하세요: sudo apt install -y python3-venv"
  exit 1
fi

python3 -m venv --system-site-packages "$VENV"
"$VENV/bin/pip" install --upgrade pip
"$VENV/bin/pip" install "numpy<2" "opencv-python<4.11" ultralytics
"$VENV/bin/python" - <<'PY'
import numpy, torch, ultralytics
print(f"numpy {numpy.__version__}, torch {torch.__version__}, ultralytics {ultralytics.__version__}")
print("GPU:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "없음 (CPU로 실행)")
PY
echo "완료. 탐지 노드는 $VENV/bin/python 으로 실행합니다."
