#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source "$HOME/bimanual_ws/devel/setup.bash"

POSE="${1:-latest}"

if [[ $# -gt 0 ]]; then
    shift
fi

exec python3 \
  "$HOME/bimanual_ws/src/left_panda_fixed_pose_test/scripts/move_to_saved_pose.py" \
  "$POSE" "$@"
