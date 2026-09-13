#!/usr/bin/env bash
set -Eeuo pipefail

# 面向国内网络环境的一键 ROS 2 + Gazebo 安装脚本。
# 脚本只使用临时 APT 源文件，不会修改：
#   /etc/apt/sources.list
#   /etc/apt/sources.list.d/*

SCRIPT_NAME="$(basename "$0")"
UBUNTU_MIRROR="${UBUNTU_MIRROR:-https://mirrors.ustc.edu.cn/ubuntu/}"
UBUNTU_FALLBACK_MIRRORS="${UBUNTU_FALLBACK_MIRRORS:-https://mirrors.aliyun.com/ubuntu/ https://repo.huaweicloud.com/ubuntu/ https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ http://cn.archive.ubuntu.com/ubuntu/}"
ROS2_MIRROR="${ROS2_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu}"
ROS_KEYRING="${ROS_KEYRING:-/usr/share/keyrings/ros-archive-keyring.gpg}"

APT_SOURCE_FILE=""
APT_SOURCE_WITH_ROS=0

log() {
  printf '\033[1;34m[%s]\033[0m %s\n' "$SCRIPT_NAME" "$*"
}

warn() {
  printf '\033[1;33m[%s]\033[0m %s\n' "$SCRIPT_NAME" "$*" >&2
}

die() {
  printf '\033[1;31m[%s]\033[0m %s\n' "$SCRIPT_NAME" "$*" >&2
  exit 1
}

cleanup() {
  if [[ -n "${APT_SOURCE_FILE}" && -f "${APT_SOURCE_FILE}" ]]; then
    rm -f "${APT_SOURCE_FILE}"
  fi
}
trap cleanup EXIT

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "缺少必要命令：$1"
}

usage() {
  cat <<EOF
用法：
  bash scripts/00_install_ros2_gazebo.sh [options]

说明：
  脚本会使用国内镜像安装 iCAN 仿真常用 ROS 2 Humble / Gazebo 环境，
  包括 Gazebo、RViz、Nav2、SLAM、图像处理、AprilTag/AR 相关基础依赖和开发工具。
  安装完成后会自动验证结果，并把 ROS 2 Humble 环境配置写入 ~/.bashrc。

选项：
  --ubuntu-mirror URL       临时使用的 Ubuntu APT 镜像源
  --ros2-mirror URL         临时使用的 ROS 2 APT 镜像源
  -h, --help                显示本帮助信息

环境变量：
  UBUNTU_MIRROR             默认值：${UBUNTU_MIRROR}
  UBUNTU_FALLBACK_MIRRORS   Ubuntu APT 备用镜像列表，空格分隔
  ROS2_MIRROR               默认值：${ROS2_MIRROR}

示例：
  bash scripts/00_install_ros2_gazebo.sh
  UBUNTU_MIRROR=https://mirrors.ustc.edu.cn/ubuntu/ \\
  ROS2_MIRROR=https://mirrors.ustc.edu.cn/ros2/ubuntu \\
    bash scripts/00_install_ros2_gazebo.sh
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ubuntu-mirror)
      UBUNTU_MIRROR="${2:-}"
      [[ -n "${UBUNTU_MIRROR}" ]] || die "--ubuntu-mirror 需要提供 URL"
      shift 2
      ;;
    --ros2-mirror)
      ROS2_MIRROR="${2:-}"
      [[ -n "${ROS2_MIRROR}" ]] || die "--ros2-mirror 需要提供 URL"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "未知选项：$1"
      ;;
  esac
done

need_cmd sudo
need_cmd awk
need_cmd sed
need_cmd mktemp

if [[ $EUID -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
else
  die "无法读取 /etc/os-release"
fi

if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" || "${VERSION_CODENAME:-}" != "jammy" ]]; then
  cat >&2 <<EOF
${SCRIPT_NAME}：不支持当前操作系统。

本安装脚本固定用于 Ubuntu 22.04 LTS (jammy)。

当前检测结果：
  ID=${ID:-unknown}
  VERSION_ID=${VERSION_ID:-unknown}
  VERSION_CODENAME=${VERSION_CODENAME:-unknown}
  PRETTY_NAME=${PRETTY_NAME:-unknown}

请在 Ubuntu 22.04 LTS 系统中运行本脚本。
EOF
  exit 1
fi

CODENAME="${VERSION_CODENAME:-}"
ROS_DISTRO="humble"

ARCH="$(dpkg --print-architecture)"
APT_SOURCE_FILE="$(mktemp /tmp/ros2-gazebo-cn-sources.XXXXXX.list)"

apt_opts=(
  -o "Dir::Etc::sourcelist=${APT_SOURCE_FILE}"
  -o "Dir::Etc::sourceparts=-"
  -o "APT::Get::List-Cleanup=0"
)

mirror_candidates() {
  local seen=" "
  local mirror

  for mirror in "${UBUNTU_MIRROR}" ${UBUNTU_FALLBACK_MIRRORS}; do
    [[ -n "${mirror}" ]] || continue
    case "${seen}" in
      *" ${mirror} "*) continue ;;
    esac
    seen="${seen}${mirror} "
    printf '%s\n' "${mirror}"
  done
}

apt_update_once() {
  "${SUDO[@]}" apt-get "${apt_opts[@]}" update
}

apt_update() {
  local original_mirror="${UBUNTU_MIRROR}"
  local mirror

  while IFS= read -r mirror; do
    UBUNTU_MIRROR="${mirror}"
    write_temp_sources "${APT_SOURCE_WITH_ROS}"
    log "正在使用 Ubuntu 镜像源：${UBUNTU_MIRROR}"
    if apt_update_once; then
      return 0
    fi
    warn "Ubuntu 镜像源更新失败，准备尝试下一个源。"
  done < <(mirror_candidates)

  UBUNTU_MIRROR="${original_mirror}"
  write_temp_sources "${APT_SOURCE_WITH_ROS}"
  return 1
}

apt_install_once() {
  "${SUDO[@]}" DEBIAN_FRONTEND=noninteractive apt-get "${apt_opts[@]}" install -y --no-install-recommends "$@"
}

apt_install() {
  local original_mirror="${UBUNTU_MIRROR}"
  local mirror
  local first_attempt=1

  while IFS= read -r mirror; do
    if [[ "${first_attempt}" == "0" || "${mirror}" != "${UBUNTU_MIRROR}" ]]; then
      UBUNTU_MIRROR="${mirror}"
      write_temp_sources "${APT_SOURCE_WITH_ROS}"
      log "正在切换 Ubuntu 镜像源并重新更新：${UBUNTU_MIRROR}"
      apt_update_once || {
        warn "Ubuntu 镜像源更新失败，准备尝试下一个源。"
        first_attempt=0
        continue
      }
    fi

    log "正在使用 Ubuntu 镜像源安装软件包：${UBUNTU_MIRROR}"
    if apt_install_once "$@"; then
      return 0
    fi

    warn "当前 Ubuntu 镜像源安装失败，准备尝试下一个源。"
    first_attempt=0
  done < <(mirror_candidates)

  UBUNTU_MIRROR="${original_mirror}"
  write_temp_sources "${APT_SOURCE_WITH_ROS}"
  return 1
}

write_temp_sources() {
  local with_ros="${1:-0}"
  APT_SOURCE_WITH_ROS="${with_ros}"

  cat > "${APT_SOURCE_FILE}" <<EOF
deb [arch=${ARCH}] ${UBUNTU_MIRROR} ${CODENAME} main restricted universe multiverse
deb [arch=${ARCH}] ${UBUNTU_MIRROR} ${CODENAME}-updates main restricted universe multiverse
deb [arch=${ARCH}] ${UBUNTU_MIRROR} ${CODENAME}-backports main restricted universe multiverse
deb [arch=${ARCH}] ${UBUNTU_MIRROR} ${CODENAME}-security main restricted universe multiverse
EOF

  if [[ "${with_ros}" == "1" ]]; then
    cat >> "${APT_SOURCE_FILE}" <<EOF
deb [arch=${ARCH} signed-by=${ROS_KEYRING}] ${ROS2_MIRROR} ${CODENAME} main
EOF
  fi
}

install_ros_key() {
  log "正在安装 ROS 软件源密钥到 ${ROS_KEYRING}"
  local key_tmp
  key_tmp="$(mktemp /tmp/ros-archive-key.XXXXXX)"

  local urls=(
    "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
    "https://gitee.com/ros2-cn/rosdistro/raw/master/ros.key"
  )

  local ok=0
  for url in "${urls[@]}"; do
    log "正在尝试下载 ROS 密钥：${url}"
    if curl -fsSL --connect-timeout 15 --retry 3 "${url}" -o "${key_tmp}"; then
      ok=1
      break
    fi
  done

  [[ "${ok}" == "1" ]] || die "ROS 软件源密钥下载失败，请确认网络或代理可用后重新运行。"

  "${SUDO[@]}" install -d -m 0755 "$(dirname "${ROS_KEYRING}")"
  "${SUDO[@]}" install -m 0644 "${key_tmp}" "${ROS_KEYRING}"
  rm -f "${key_tmp}"
}

configure_shell_env() {
  local setup_line="source /opt/ros/${ROS_DISTRO}/setup.bash"
  local bashrc="${HOME}/.bashrc"

  touch "${bashrc}"
  if grep -Fxq "${setup_line}" "${bashrc}"; then
    log "永久环境配置已存在：${bashrc}"
  else
    {
      printf '\n# ROS 2 Humble 环境配置\n'
      printf '%s\n' "${setup_line}"
    } >> "${bashrc}"
    log "已写入永久环境配置：${bashrc}"
  fi
}

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

verify_install() {
  log "正在验证 ROS 2 Humble 与 Gazebo 安装结果"

  source_setup_file "/opt/ros/${ROS_DISTRO}/setup.bash"

  local failed=0

  if [[ "${ROS_DISTRO:-}" == "humble" ]]; then
    log "ROS_DISTRO=humble"
  else
    warn "ROS_DISTRO 检查异常，当前值：${ROS_DISTRO:-未设置}"
    failed=1
  fi

  if command -v ros2 >/dev/null 2>&1; then
    log "ros2 命令可用：$(command -v ros2)"
  else
    warn "未找到 ros2 命令"
    failed=1
  fi

  if ros2 pkg prefix rclcpp >/dev/null 2>&1; then
    log "ROS 2 核心包检查通过：rclcpp"
  else
    warn "ROS 2 核心包检查失败：rclcpp"
    failed=1
  fi

  if ros2 pkg prefix gazebo_ros >/dev/null 2>&1; then
    log "Gazebo ROS 插件包检查通过：gazebo_ros"
  else
    warn "Gazebo ROS 插件包检查失败：gazebo_ros"
    failed=1
  fi

  if ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
    log "Navigation2 检查通过：nav2_bringup"
  else
    warn "Navigation2 检查失败：nav2_bringup"
    failed=1
  fi

  if ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
    log "SLAM Toolbox 检查通过：slam_toolbox"
  else
    warn "SLAM Toolbox 检查失败：slam_toolbox"
    failed=1
  fi

  if ros2 pkg prefix cv_bridge >/dev/null 2>&1; then
    log "图像桥接检查通过：cv_bridge"
  else
    warn "图像桥接检查失败：cv_bridge"
    failed=1
  fi

  if command -v apriltag_demo >/dev/null 2>&1 || dpkg -s apriltag >/dev/null 2>&1; then
    log "AprilTag 基础工具检查通过"
  else
    warn "AprilTag 基础工具检查失败：apriltag"
    failed=1
  fi

  if command -v gazebo >/dev/null 2>&1; then
    log "$(gazebo --version | head -n 1)"
  else
    warn "未找到 gazebo 命令"
    failed=1
  fi

  return "${failed}"
}

log "检测到 Ubuntu ${VERSION_ID:-unknown} (${CODENAME})，系统架构：${ARCH}"
log "将安装 ROS 2 发行版：${ROS_DISTRO}"
log "首选 Ubuntu 镜像源：${UBUNTU_MIRROR}"
log "备用 Ubuntu 镜像源：${UBUNTU_FALLBACK_MIRRORS}"
log "临时使用 ROS 2 镜像源：${ROS2_MIRROR}"

write_temp_sources 0
log "正在使用临时 Ubuntu 镜像源更新 APT 软件包列表"
apt_update

log "正在安装基础工具"
apt_install ca-certificates curl gnupg lsb-release locales software-properties-common

install_ros_key

write_temp_sources 1
log "正在使用临时 Ubuntu + ROS 2 镜像源更新 APT 软件包列表"
apt_update

packages=(
  "ros-${ROS_DISTRO}-desktop"
  "ros-${ROS_DISTRO}-xacro"
  "ros-${ROS_DISTRO}-robot-state-publisher"
  "ros-${ROS_DISTRO}-joint-state-publisher"
  "ros-${ROS_DISTRO}-joint-state-publisher-gui"
  "ros-${ROS_DISTRO}-tf-transformations"
  "ros-${ROS_DISTRO}-gazebo-ros-pkgs"
  "ros-${ROS_DISTRO}-gazebo-plugins"
  "ros-${ROS_DISTRO}-rviz2"
  "ros-${ROS_DISTRO}-navigation2"
  "ros-${ROS_DISTRO}-nav2-bringup"
  "ros-${ROS_DISTRO}-nav2-msgs"
  "ros-${ROS_DISTRO}-slam-toolbox"
  "ros-${ROS_DISTRO}-cv-bridge"
  "ros-${ROS_DISTRO}-image-transport"
  "ros-${ROS_DISTRO}-image-geometry"
  "ros-${ROS_DISTRO}-camera-info-manager"
  "ros-${ROS_DISTRO}-sensor-msgs-py"
  gazebo
  apriltag
  libapriltag-dev
  python3-apriltag
  python3-opencv
  libopencv-dev
  libopencv-contrib4.5d
  python3-colcon-common-extensions
  python3-rosdep
  python3-vcstool
  python3-pip
  ros-dev-tools
)

log "正在安装 ROS 2 + Gazebo + Nav2 + 视觉识别相关软件包"
apt_install "${packages[@]}"

if command -v rosdep >/dev/null 2>&1; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "正在初始化 rosdep"
    "${SUDO[@]}" rosdep init || warn "rosdep init 失败。若提示已初始化，可忽略；否则请检查网络后重试。"
  fi

  if [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "正在更新 rosdep 数据库"
    rosdep update || warn "rosdep update 失败。你可以稍后手动重新运行：rosdep update"
  else
    warn "rosdep 初始化未完成。你可以稍后手动运行：sudo rosdep init && rosdep update"
  fi
fi

configure_shell_env

if verify_install; then
  VERIFY_RESULT="验证通过"
else
  VERIFY_RESULT="验证存在异常，请查看上方警告"
fi

cat <<EOF

ROS 2 Humble 安装完成。
Gazebo Classic 安装完成。
Gazebo ROS 插件安装完成。
Navigation2 / SLAM Toolbox 安装完成。
图像处理与 AprilTag/AR 基础依赖安装完成。
永久环境配置已完成，新终端会自动加载 ROS 2 Humble。

安装验证结果：${VERIFY_RESULT}

如需在当前终端立即使用，请运行：
  source /opt/ros/${ROS_DISTRO}/setup.bash

手动检查命令：
  echo \$ROS_DISTRO
  ros2 pkg prefix rclcpp
  ros2 pkg prefix gazebo_ros
  ros2 pkg prefix nav2_bringup
  ros2 pkg prefix slam_toolbox
  ros2 pkg prefix cv_bridge
  python3 -c "import cv2; print(cv2.__version__)"
  gazebo --version

本脚本使用的临时 APT 源文件为：
  ${APT_SOURCE_FILE}

本脚本没有修改：
  /etc/apt/sources.list
  /etc/apt/sources.list.d/*
EOF
