#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /opt/mona_ws/devel_isolated/setup.bash
export ROS_PACKAGE_PATH="/opt/mona_runtime:${ROS_PACKAGE_PATH}"
case "${1:-check}" in
  check) exec python3 /opt/mona_runtime/check_stack.py ;;
  graph) exec roslaunch --nodes mona_integration full.launch ;;
  run) shift; exec roslaunch mona_integration full.launch "$@" ;;
  shell) exec bash ;;
  *) exec "$@" ;;
esac
