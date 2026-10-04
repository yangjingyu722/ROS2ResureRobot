#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_CMD="cd \"${WORKSPACE_DIR}\"; source /opt/ros/humble/setup.bash; source /opt/ican_shoot_sim/setup.bash; source \"${WORKSPACE_DIR}/install/setup.bash\""
WAIT_READY="python3 \"${SCRIPT_DIR}/wait_for_ready.py\""

# 旧栈没退干净就重新启动时，两套 nav2 / 两个任务节点会同时运行：
# 机器人不动、一次发射两颗子弹都是这么来的。启动前先清一遍。
echo "启动前清理残留的旧栈进程..."
bash "${SCRIPT_DIR}/cleanup_stack.sh"

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "未检测到图形界面（DISPLAY 为空），无法打开 gnome-terminal 标签页。" >&2
  echo "无界面环境（SSH/服务器）请改用以下等价命令分步启动：" >&2
  echo "  source scripts/setup_env.sh" >&2
  echo "  1) ros2 launch ican_shoot_sim sim.launch.py gui:=false          # 启动仿真" >&2
  echo "  2) python3 scripts/wait_for_ready.py clock --timeout 180 --stable-seconds 5 \\" >&2
  echo "       && ros2 launch ican_example_navigation navigation.launch.py  # 等时钟稳定后启动导航" >&2
  echo "  3) python3 scripts/wait_for_ready.py nav2 --timeout 240 \\" >&2
  echo "       && ros2 launch ican_example_mission full_mission.launch.py \\" >&2
  echo "         prompt_target_config:=false target_2_id:=1 target_3_id:=6 target_4_id:=8  # 启动任务" >&2
  exit 1
fi

gnome-terminal --window --title="sim" -- \
  bash -c "${SETUP_CMD}; ros2 launch ican_shoot_sim sim.launch.py; exec bash"
gnome-terminal --tab --title="navigation" -- \
  bash -c "${SETUP_CMD}; ${WAIT_READY} clock --timeout 180 --stable-seconds 5 && ros2 launch ican_example_navigation navigation.launch.py; exec bash"
gnome-terminal --tab --title="example" -- \
  bash -c "${SETUP_CMD}; ${WAIT_READY} nav2 --timeout 240 && ros2 launch ican_example_mission full_mission.launch.py; exec bash"
