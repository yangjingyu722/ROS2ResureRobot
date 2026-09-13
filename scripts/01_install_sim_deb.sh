#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEB_PATH="${WORKSPACE_DIR}/packages/ican-shoot-sim-practice_0.1.0_amd64.deb"

log() {
  printf '\033[1;34m[install-sim]\033[0m %s\n' "$*"
}

die() {
  printf '\033[1;31m[install-sim]\033[0m %s\n' "$*" >&2
  exit 1
}

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

[[ -f "${DEB_PATH}" ]] || die "找不到安装包：${DEB_PATH}"

log "准备安装：${DEB_PATH}"
log "如依赖解析失败，请先运行：bash scripts/00_install_ros2_gazebo.sh"

sudo apt-get update
sudo apt-get install -y "${DEB_PATH}"

if [[ -f /opt/ros/humble/setup.bash && -f /opt/ican_shoot_sim/setup.bash ]]; then
  source_setup_file /opt/ros/humble/setup.bash
  source_setup_file /opt/ican_shoot_sim/setup.bash
  ros2 pkg prefix ican_shoot_sim >/dev/null
  log "安装完成，仿真包可用：$(ros2 pkg prefix ican_shoot_sim)"
else
  die "安装后未找到 ROS 或仿真环境 setup.bash"
fi

cat <<'EOF'

下一步：
  source scripts/setup_env.sh
  bash scripts/run_sim.sh
EOF
