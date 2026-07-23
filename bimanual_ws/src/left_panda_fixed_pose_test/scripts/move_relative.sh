#!/usr/bin/env bash
set -e

DX="${1:-0.0}"
DY="${2:-0.0}"
DZ="${3:-0.01}"

source /opt/ros/noetic/setup.bash
source "$HOME/bimanual_ws/devel/setup.bash"

python3 \
  "$HOME/bimanual_ws/src/left_panda_fixed_pose_test/scripts/publish_relative_target.py" \
  --dx "$DX" \
  --dy "$DY" \
  --dz "$DZ"

sleep 0.5

rosservice call \
  /panda_left/left_fixed_pose_controller/start \
  "{}"
