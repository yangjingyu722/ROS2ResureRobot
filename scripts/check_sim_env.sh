#!/usr/bin/env bash
set -Eeuo pipefail

failed=0

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

check() {
  local label="$1"
  shift
  if "$@" >/tmp/ican_sim_check.out 2>/tmp/ican_sim_check.err; then
    printf '\033[1;32m[OK]\033[0m %s\n' "${label}"
  else
    printf '\033[1;31m[FAIL]\033[0m %s\n' "${label}" >&2
    sed -n '1,20p' /tmp/ican_sim_check.err >&2
    failed=1
  fi
}

check "ROS 2 setup" test -f /opt/ros/humble/setup.bash
check "仿真环境 setup" test -f /opt/ican_shoot_sim/setup.bash

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source_setup_file /opt/ros/humble/setup.bash
fi

if [[ -f /opt/ican_shoot_sim/setup.bash ]]; then
  source_setup_file /opt/ican_shoot_sim/setup.bash
fi

check "ros2 命令" command -v ros2
check "Gazebo 命令" command -v gazebo
check "gazebo_ros 包" ros2 pkg prefix gazebo_ros
check "Navigation2 包" ros2 pkg prefix nav2_bringup
check "SLAM Toolbox 包" ros2 pkg prefix slam_toolbox
check "cv_bridge 包" ros2 pkg prefix cv_bridge
check "image_transport 包" ros2 pkg prefix image_transport
check "image_geometry 包" ros2 pkg prefix image_geometry
check "Python OpenCV" python3 -c "import cv2"
check "AprilTag 基础工具" dpkg -s apriltag
check "ican_shoot_sim 包" ros2 pkg prefix ican_shoot_sim
check "ican_land_sim 包" ros2 pkg prefix ican_land_sim
check "ican_land_control 包" ros2 pkg prefix ican_land_control
check "launch 参数" ros2 launch ican_shoot_sim sim.launch.py --show-args

rm -f /tmp/ican_sim_check.out /tmp/ican_sim_check.err
exit "${failed}"
