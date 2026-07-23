#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source "$HOME/bimanual_ws/devel/setup.bash"

NS="/panda_left"
CTRL="left_fixed_pose_controller"

echo "Requesting Franka error recovery..."

rostopic pub -1 \
  "${NS}/franka_control/error_recovery/goal" \
  franka_msgs/ErrorRecoveryActionGoal \
  "{}"

sleep 2

echo "Restarting Cartesian controller..."

rosservice call "${NS}/controller_manager/switch_controller" \
"start_controllers: []
stop_controllers: ['${CTRL}']
strictness: 1
start_asap: false
timeout: 5.0" >/dev/null || true

sleep 1

rosservice call "${NS}/controller_manager/switch_controller" \
"start_controllers: ['${CTRL}']
stop_controllers: []
strictness: 2
start_asap: false
timeout: 5.0"

echo "Left Panda recovered and controller restarted."
