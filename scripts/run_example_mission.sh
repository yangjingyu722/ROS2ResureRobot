#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_CMD="cd \"${WORKSPACE_DIR}\"; source /opt/ros/humble/setup.bash; source /opt/ican_shoot_sim/setup.bash; source \"${WORKSPACE_DIR}/install/setup.bash\""
WAIT_READY="python3 \"${SCRIPT_DIR}/wait_for_ready.py\""

gnome-terminal --window --title="sim" -- \
  bash -c "${SETUP_CMD}; ros2 launch ican_shoot_sim sim.launch.py; exec bash"
gnome-terminal --tab --title="navigation" -- \
  bash -c "${SETUP_CMD}; ${WAIT_READY} clock --timeout 180 --stable-seconds 5 && ros2 launch ican_example_navigation navigation.launch.py; exec bash"
gnome-terminal --tab --title="example" -- \
  bash -c "${SETUP_CMD}; ${WAIT_READY} nav2 --timeout 240 && ros2 launch ican_example_mission full_mission.launch.py; exec bash"
