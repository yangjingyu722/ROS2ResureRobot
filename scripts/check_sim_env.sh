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

# 本工作空间自带的包（如 nav2_bringup）只有 source install 后才可见，
# 否则会对随工作空间分发的包误报 FAIL。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  source_setup_file "${WORKSPACE_DIR}/install/setup.bash"
fi

check "ros2 命令" command -v ros2
check "Gazebo 命令" command -v gazebo
check "gazebo_ros 包" ros2 pkg prefix gazebo_ros
check "Navigation2 包" ros2 pkg prefix nav2_bringup
# 示例导航固定使用地图 + AMCL 定位模式（slam:=False），slam_toolbox 仅在
# 需要重新建图时才用到，缺少不影响示例运行，因此只警告不判失败。
if ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
  printf '\033[1;32m[OK]\033[0m %s\n' "SLAM Toolbox 包"
else
  printf '\033[1;33m[WARN]\033[0m %s\n' "SLAM Toolbox 包（可选，仅重新建图时需要）" >&2
fi
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
