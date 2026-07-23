#!/bin/bash
set -e

echo "=== 1. Installing apt packages ==="
apt update
apt install -y \
  git nano curl python3-pip \
  python3-catkin-tools \
  mesa-utils \
  ros-noetic-pybind11-catkin \
  ros-noetic-joint-trajectory-controller \
  ros-noetic-position-controllers \
  ros-noetic-effort-controllers \
  ros-noetic-velocity-controllers

echo "=== 2. Downloading MuJoCo C++ library ==="
if [ ! -d "/root/mujoco-3.3.5" ]; then
  cd /root
  curl -L -O https://github.com/google-deepmind/mujoco/releases/download/3.3.5/mujoco-3.3.5-linux-x86_64.tar.gz
  tar -xzf mujoco-3.3.5-linux-x86_64.tar.gz
fi

echo "=== 3. Installing Python mujoco bindings ==="
pip3 install --upgrade pip
pip3 install mujoco==3.2.3

echo "=== 4. Moving to workspace src folder ==="
cd /home/scstln/jonghynun/bimanual_ws/src

echo "=== 5. Cloning mujoco_ros_pkgs ==="
if [ ! -d "mujoco_ros_pkgs" ]; then
  git clone https://github.com/ubi-agni/mujoco_ros_pkgs.git
fi

echo "=== 6. Cloning py_binding_tools ==="
if [ ! -d "py_binding_tools" ]; then
  git clone https://github.com/moveit/py_binding_tools.git
fi

echo "=== 7. Cloning franka_ros ==="
if [ ! -d "franka_ros" ]; then
  git clone https://github.com/frankaemika/franka_ros.git
fi

echo "=== 8. Cloning Bimanual_Robot_Project_MuJoCo (with submodules) ==="
if [ ! -d "Bimanual_Robot_Project_MuJoCo" ]; then
  git clone --recurse-submodules https://github.com/jongkim228/Bimanual_Robot_Project_MuJoCo.git
  # Remove duplicate franka_ros submodule inside the repo (we already cloned it separately)
  rm -rf Bimanual_Robot_Project_MuJoCo/franka_ros
fi

echo "=== 9. Marking hardware-only packages (require libfranka) as CATKIN_IGNORE ==="
touch franka_ros/franka_gripper/CATKIN_IGNORE 2>/dev/null || true
touch franka_ros/franka_hw/CATKIN_IGNORE 2>/dev/null || true
touch franka_ros/franka_control/CATKIN_IGNORE 2>/dev/null || true
touch franka_ros/franka_visualization/CATKIN_IGNORE 2>/dev/null || true
touch franka_ros/franka_example_controllers/CATKIN_IGNORE 2>/dev/null || true
touch left_panda_bottle_pose_controller/CATKIN_IGNORE 2>/dev/null || true
touch left_panda_fixed_pose_test/CATKIN_IGNORE 2>/dev/null || true

echo "=== 10. Setting environment variables and sourcing ROS ==="
export MUJOCO_DIR=/root/mujoco-3.3.5
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_DIR/lib
source /opt/ros/noetic/setup.bash

echo "=== 11. Running rosdep install ==="
cd /home/scstln/jonghynun/bimanual_ws
rosdep install --from-paths src --ignore-src -r -y

echo "=== 12. Building the workspace ==="
catkin build --cmake-args -DCMAKE_CXX_FLAGS="-Wno-error=pedantic"

echo ""
echo "=== DONE ==="
echo "Next: source devel/setup.bash && ./run.sh"
