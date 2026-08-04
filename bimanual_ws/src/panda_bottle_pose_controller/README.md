# Left Panda fixed Cartesian-pose test

This is a ROS1 `franka_ros` Cartesian trajectory generator. It starts from the current commanded end-effector pose and smoothly interpolates to a fixed target using a minimum-jerk translation and quaternion SLERP.

It is **not** a planner: there is no IK search, no collision checking and no obstacle avoidance. Test a target only a few centimetres from the current pose first.

## Build
```bash
cd ~/bimanual_ws/src
unzip ~/Downloads/left_panda_bottle_pose_controller.zip
cd ~/bimanual_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Run after the left hardware is already running
```bash
roslaunch left_panda_bottle_pose_controller after_left_hardware.launch
```
The controller holds position until explicitly started.

## Read current Cartesian pose
```bash
rostopic echo -n 1 /panda_left/franka_state_controller/franka_states
```
`O_T_EE` is column-major; x,y,z are entries 12,13,14.

## Start and stop
```bash
rosservice call /panda_left/left_bottle_pose_controller/start
rosservice call /panda_left/left_bottle_pose_controller/stop
```

## Send a new target
```bash
rostopic pub -1 /panda_left/left_bottle_pose_controller/target_pose geometry_msgs/PoseStamped "header: {frame_id: 'panda_left_link0'}
pose:
  position: {x: 0.45, y: 0.20, z: 0.35}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
rosservice call /panda_left/left_bottle_pose_controller/start
```
