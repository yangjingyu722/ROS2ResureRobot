#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 ROS 2 Humble，请先运行：bash ${SCRIPT_DIR}/00_install_ros2_gazebo.sh" >&2
  exit 1
fi

if [[ ! -f /opt/ican_shoot_sim/setup.bash ]]; then
  echo "未找到 iCAN 仿真环境，请先运行：bash ${SCRIPT_DIR}/01_install_sim_deb.sh" >&2
  exit 1
fi

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

source_setup_file /opt/ros/humble/setup.bash
source_setup_file /opt/ican_shoot_sim/setup.bash

exec ros2 launch ican_shoot_sim sim.launch.py "$@"
