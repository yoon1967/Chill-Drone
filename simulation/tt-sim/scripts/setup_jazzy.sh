#!/usr/bin/env bash
# Ubuntu 24.04(WSL2 포함)에 ROS 2 Jazzy + Gazebo Harmonic을 설치한다.
# Gazebo Harmonic은 ros-jazzy-ros-gz가 ROS 벤더 패키지로 함께 설치한다.
#
# 사용: sudo bash scripts/setup_jazzy.sh            (sudo를 쓴 계정의 ~/.bashrc에 ROS 환경을 추가)
#       sudo bash scripts/setup_jazzy.sh <계정이름>  (지정한 계정의 ~/.bashrc에 추가)
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "root 권한이 필요합니다: sudo bash $0" >&2
  exit 1
fi
. /etc/os-release
if [ "${VERSION_ID}" != "24.04" ]; then
  echo "Ubuntu 24.04 전용입니다 (현재 ${VERSION_ID})." >&2
  exit 1
fi
export DEBIAN_FRONTEND=noninteractive

echo "== 로케일·기본 도구"
apt-get update
apt-get install -y locales software-properties-common curl
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe

echo "== ROS 2 apt 저장소"
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME:-${VERSION_CODENAME}}_all.deb"
dpkg -i /tmp/ros2-apt-source.deb

echo "== ROS 2 Jazzy + Gazebo Harmonic"
apt-get update
apt-get upgrade -y
apt-get install -y ros-jazzy-desktop ros-jazzy-ros-gz python3-colcon-common-extensions

TARGET_USER="${1:-${SUDO_USER:-}}"
if [ -n "${TARGET_USER}" ] && [ -d "/home/${TARGET_USER}" ]; then
  BASHRC="/home/${TARGET_USER}/.bashrc"
  if ! grep -qF "/opt/ros/jazzy/setup.bash" "${BASHRC}"; then
    echo "source /opt/ros/jazzy/setup.bash" >> "${BASHRC}"
    echo "== ${BASHRC}에 ROS 환경을 추가했습니다"
  fi
fi

echo "== 설치 확인"
bash -c "source /opt/ros/jazzy/setup.bash && ros2 --help >/dev/null && echo 'ros2 OK' && gz sim --versions"
