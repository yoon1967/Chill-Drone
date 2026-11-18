#!/usr/bin/env bash
# 자율 비행 시뮬레이터 시연(start.cmd sim)을 위한 WSL 준비. 한 번만 돌린다(약 10 GB, 인터넷 1시간 안팎).
# WSL Ubuntu-24.04에서 root로 돌린다(비밀번호를 묻지 않는다). start.cmd sim이 모자란 것을 보면 이 명령을 알려 준다:
#   wsl -d Ubuntu-24.04 -u root -- bash <이 파일의 WSL 경로> <Ubuntu 사용자 이름> <코드 폴더의 WSL 경로> [git 폴더] [브랜치]
# git 폴더·브랜치: worktree의 .git은 Windows 경로를 적은 파일이라 WSL git이 못 읽어서, 실행기가 공통 .git 폴더와 지금 브랜치를 넘긴다
# (없으면 코드 폴더·HEAD).
# 자율 비행 README·scripts/setup_jazzy.sh·setup_yolo.sh 순서 그대로이고, 빠진 것(av·transformers·scipy·opencv)과
# 모델 미리 받기(비행 중에는 드론 Wi-Fi라 인터넷이 없다)를 더한다. 자율 비행 코드는 고치지 않는다.
set -euo pipefail
U="${1:?Ubuntu 사용자 이름을 주세요}"
SRC="${2:?코드 폴더의 WSL 경로를 주세요(예: /mnt/c/Users/me/S15P31B205)}"
GITSRC="${3:-$SRC}"
REF="${4:-HEAD}"
H="/home/$U"
[ -d "$H" ] || { echo "사용자 홈이 없습니다: $H" >&2; exit 1; }
[ -f "$SRC/simulation/tt-sim/launch/mission.launch.py" ] || {
  echo "이 저장소에 자율 비행 시뮬레이터(simulation/tt-sim)가 없습니다. 자율 비행 코드가 합쳐진 브랜치에서 돌리세요: $SRC" >&2; exit 1; }
TT="$H/ttrepo/simulation/tt-sim"
as_user() { sudo -H -u "$U" "$@"; }
step() { echo; echo "===== $(date +%H:%M:%S) $*"; }

step "1/7 기본 패키지(git·venv·scipy·opencv)"
export DEBIAN_FRONTEND=noninteractive
apt-get update -q
apt-get install -y -q git python3-venv python3-scipy python3-opencv curl sudo

step "2/7 시뮬레이터 사본 ~/ttrepo (Windows 저장소의 지금 커밋을 얕게 복사, 실행기가 켤 때마다 맞춘다)"
as_user git config --global --get-all safe.directory | grep -qxF "$GITSRC" || as_user git config --global --add safe.directory "$GITSRC"
if [ ! -d "$H/ttrepo/.git" ]; then
  as_user git init -q "$H/ttrepo"
fi
as_user git -C "$H/ttrepo" fetch -q --depth 1 "file://$GITSRC" "$REF"
as_user git -C "$H/ttrepo" checkout -q -B launcher-sim FETCH_HEAD
as_user git -C "$H/ttrepo" log -1 --format="   %h %s"

step "3/7 ROS 2 Jazzy + Gazebo Harmonic (자율 비행 scripts/setup_jazzy.sh)"
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  bash "$TT/scripts/setup_jazzy.sh" "$U"
else
  echo "   이미 있음"
fi

step "4/7 YOLO 환경 ~/venvs/tt (자율 비행 scripts/setup_yolo.sh)"
if [ ! -x "$H/venvs/tt/bin/python" ]; then
  as_user bash "$TT/scripts/setup_yolo.sh"
else
  echo "   이미 있음"
fi

step "5/7 드라이버 영상(PyAV)·깊이 추정(transformers)"
as_user "$H/venvs/tt/bin/pip" install -q av transformers

step "6/7 모델 미리 받기(YOLO 가중치, 깊이 모델)"
as_user mkdir -p "$TT/weights"
if [ ! -f "$TT/weights/yolo26n-seg.pt" ]; then
  as_user bash -c "cd '$TT/weights' && '$H/venvs/tt/bin/python' -c \"from ultralytics import YOLO; YOLO('yolo26n-seg.pt')\""
fi
as_user "$H/venvs/tt/bin/python" - <<'PY'
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
m = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"
AutoImageProcessor.from_pretrained(m, use_fast=True)
AutoModelForDepthEstimation.from_pretrained(m)
import torch, ultralytics, av, cv2, numpy
print("   깊이 모델 받음 ·", "torch", torch.__version__, "· GPU", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "없음",
      "· ultralytics", ultralytics.__version__, "· av", av.__version__, "· numpy", numpy.__version__)
PY

step "7/7 시뮬레이터 세계 만들기(Gazebo를 돌릴 PC에서 만든다)"
as_user bash -c "source /opt/ros/jazzy/setup.bash && cd '$TT' && python3 generate_world.py --seed 7 --stage 3" | tail -3

step "끝. Windows에서 scripts\\start.cmd sim"
