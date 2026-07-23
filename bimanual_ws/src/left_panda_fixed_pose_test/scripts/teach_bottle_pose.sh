#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source "$HOME/bimanual_ws/devel/setup.bash"

NAME="${1:-bottle_approach}"

exec python3 \
  "$HOME/bimanual_ws/src/left_panda_fixed_pose_test/scripts/record_bottle_pose.py" \
  --name "$NAME"
