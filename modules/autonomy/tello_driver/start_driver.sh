#!/usr/bin/env bash
# WSL에서 실기체 드라이버를 띄운다. WSL 기본(NAT) 네트워크에서는 Windows에서 tello_relay.py가 먼저 돌고 있어야 한다.
#
# 사용: bash start_driver.sh            # 연결만 확인 (이륙 요청은 모두 거절)
#       bash start_driver.sh --fly      # 운용자 확인을 켠다: 주변 3 m 비움·프로펠러 가드·지켜볼 사람을 확인한 뒤에만
#       bash start_driver.sh --video    # 영상도 /tt/camera/image_raw로 낸다 (PyAV 필요)
#       bash start_driver.sh --mirrored # WSL 미러 네트워크라 중계 없이 Tello(192.168.10.1)에 바로 붙을 때
# 끄기: Ctrl+C (떠 있으면 착륙한다)
source /opt/ros/jazzy/setup.bash
HERE=$(cd "$(dirname "$0")" && pwd)
WIN=$(ip route show default | awk '{print $3; exit}')   # WSL에서 본 Windows 주소
ARGS=(-p host:="$WIN" -p cmd_port:=9889 -p local_cmd_port:=8889 -p state_port:=8890)
for a in "$@"; do
  case "$a" in
    --fly) ARGS+=(-p operator_confirmed:=true) ;;
    --video) ARGS+=(-p video:=true) ;;
    --mirrored) ARGS+=(-p host:=192.168.10.1 -p cmd_port:=8889) ;;
    *) echo "모르는 옵션: $a"; exit 1 ;;
  esac
done
exec python3 "$HERE/tello_driver_node.py" --ros-args "${ARGS[@]}"
