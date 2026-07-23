#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/scstln/jonghynun/bimanual_ws/devel/setup.bash
export MUJOCO_DIR=/root/mujoco-3.3.5
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_DIR/lib
roslaunch bimanual_robot_project_mujoco bimanual_control.launch
