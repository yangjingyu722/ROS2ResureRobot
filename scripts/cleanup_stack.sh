#!/usr/bin/env bash
# 清理本机残留的仿真 / 导航 / 任务栈进程。
#
# 为什么需要：旧栈没退干净就重新启动时，两套 nav2 和两个任务节点会同时
# 运行——互相取消导航目标（表现为机器人完全不动）、各自调用射击服务
# （表现为一次冒出两颗子弹）。重复启动前先跑一遍本脚本。
#
# 匹配一律用可执行文件路径特征（含 / 前缀），避免误杀恰好提到这些名字的
# 编辑器、tail、grep 等无关进程。
#
# 用法：bash scripts/cleanup_stack.sh
set -u

SELF_PID=$$
PARENT_PID=$PPID

# 按 /完整路径特征/ 匹配（pgrep -f）
PATH_PATTERNS=(
  "bin/ros2 launch"
  "/gzserver"            # 兜底，另有 pgrep -x 精确匹配
  "/robot_state_publisher"
  "/target_motion_node"
  "/shooter_node"
  "/referee_node"
  "/ring_aim_node"
  "/full_mission_node"
  "/one_target_mission_node"
  "/initial_pose_publisher"
  "/wait_for_ready.py"
  "nav2_map_server/map_server"
  "nav2_amcl/amcl"
  "nav2_controller/controller_server"
  "nav2_smoother/smoother_server"
  "nav2_planner/planner_server"
  "nav2_bt_navigator/bt_navigator"
  "nav2_behavior_tree/behavior_server"
  "nav2_waypoint_follower/waypoint_follower"
  "nav2_velocity_smoother/velocity_smoother"
  "nav2_lifecycle_manager/lifecycle_manager"
)
# 按进程名精确匹配（pgrep -x）
NAME_PATTERNS=(
  "gzserver"
  "gzclient"
)

collect_pids() {
  local pattern="$1" mode="$2"
  local result=""
  if [[ "$mode" == "name" ]]; then
    result=$(pgrep -x "$pattern" 2>/dev/null || true)
  else
    result=$(pgrep -f "$pattern" 2>/dev/null || true)
  fi
  # 排除脚本自身与直接父进程，防止自残。
  printf '%s\n' "$result" | grep -vx -e "$SELF_PID" -e "$PARENT_PID" || true
}

kill_phase() {
  local signal="$1" wait_sec="$2"
  local raw=""
  for pattern in "${PATH_PATTERNS[@]}"; do
    raw+=$(collect_pids "$pattern" path)$'\n'
  done
  for pattern in "${NAME_PATTERNS[@]}"; do
    raw+=$(collect_pids "$pattern" name)$'\n'
  done
  mapfile -t pids < <(printf '%s' "$raw" | sort -nu | grep -v '^$' || true)
  if [[ ${#pids[@]} -eq 0 ]]; then
    return 1
  fi
  echo "  发送 SIG$signal -> PID: ${pids[*]}"
  kill "-$signal" "${pids[@]}" 2>/dev/null || true
  sleep "$wait_sec"
  return 0
}

echo "清理残留的仿真/导航/任务栈进程..."
if kill_phase TERM 2; then
  echo "  已请求优雅退出。"
fi
if kill_phase KILL 1; then
  echo "  已对未退出的进程强制结束。"
fi

# 刷新 ROS 图缓存，避免 daemon 继续报告已死节点。
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

remaining=0
for pattern in "${PATH_PATTERNS[@]}"; do
  count=$(collect_pids "$pattern" path | grep -c . || true)
  remaining=$((remaining + count))
done
for pattern in "${NAME_PATTERNS[@]}"; do
  count=$(collect_pids "$pattern" name | grep -c . || true)
  remaining=$((remaining + count))
done
if [[ "$remaining" -eq 0 ]]; then
  echo "清理完成：未发现残留进程。"
else
  echo "警告：仍有 $remaining 个相关进程未退出，请手动检查。" >&2
  exit 1
fi
