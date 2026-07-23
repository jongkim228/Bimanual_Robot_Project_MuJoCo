# Left Panda Cartesian Controller
**Author:** Nguyen Thanh Luan

---

# Overview

This package provides a custom Cartesian pose controller for the **left Franka Panda** using **franka_ros (ROS Noetic)**.

The objective is to gradually build a complete manipulation framework for a dual Panda + Allegro hand system.

Development is intentionally incremental:

```
Robot bringup
      ↓
Relative Cartesian motion
      ↓
Absolute Cartesian motion
      ↓
Bottle approach
      ↓
MoveIt integration
      ↓
Bottle grasping
      ↓
Dual-arm coordination
      ↓
Diffusion Policy
```

Current milestone:

✅ Move the Panda smoothly from its current pose to a commanded Cartesian pose.

---

# Workspace

```
~/bimanual_ws
```

---

# Robot Configuration

| Item | Value |
|------|------|
| Robot | Left Panda |
| Namespace | /panda_left |
| IP | 172.16.0.20 |
| ROS | Noetic |
| Controller | FrankaPoseCartesianInterface |

---

# Build

Whenever source code changes:

```bash
cd ~/bimanual_ws

catkin_make

source devel/setup.bash
```

---

# Launch Robot

Start everything with one command.

```bash
source ~/bimanual_ws/devel/setup.bash

roslaunch left_panda_fixed_pose_test left_panda_all.launch
```

This launches

- franka_control
- robot_state_publisher
- joint_state_publisher
- franka_state_controller
- left_fixed_pose_controller

---

# Verify Controller

```bash
rosservice call \
/panda_left/controller_manager/list_controllers
```

Expected:

```
franka_state_controller

left_fixed_pose_controller
```

Both should be

```
running
```

---

# Move Robot

Move upwards

10 mm

```bash
~/bimanual_ws/src/left_panda_fixed_pose_test/scripts/move_relative.sh \
0 0 0.01
```

Move upwards

40 mm

```bash
~/bimanual_ws/src/left_panda_fixed_pose_test/scripts/move_relative.sh \
0 0 0.04
```

Move downwards

```bash
~/bimanual_ws/src/left_panda_fixed_pose_test/scripts/move_relative.sh \
0 0 -0.02
```

Move along X

```bash
~/bimanual_ws/src/left_panda_fixed_pose_test/scripts/move_relative.sh \
0.02 0 0
```

Move along Y

```bash
~/bimanual_ws/src/left_panda_fixed_pose_test/scripts/move_relative.sh \
0 0.02 0
```

Current safety limit

```
Maximum translation = 0.05 m
```

Larger motions are rejected automatically.

---

# Expected Output

```
Current pose:
x=...
y=...
z=...

Published target:
x=...
y=...
z=...

Motion to configured target started.
```

The robot should then execute a smooth Cartesian trajectory.

---

# Monitoring Actual Motion

To compare the actual robot pose against the desired pose:

```bash
python3 - <<'PY'
import rospy
import numpy as np
from franka_msgs.msg import FrankaState

rospy.init_node("monitor", anonymous=True)

while not rospy.is_shutdown():
    msg=rospy.wait_for_message(
        "/panda_left/franka_state_controller/franka_states",
        FrankaState)

    actual=np.asarray(msg.O_T_EE).reshape((4,4),order="F")
    desired=np.asarray(msg.O_T_EE_d).reshape((4,4),order="F")

    print(
        "Desired: %.4f %.4f %.4f | Actual: %.4f %.4f %.4f"
        %(
        desired[0,3],
        desired[1,3],
        desired[2,3],
        actual[0,3],
        actual[1,3],
        actual[2,3]))
PY
```

This is useful for verifying tracking accuracy.

Current observed tracking error

```
0.3–0.7 mm
```

---

# Current Controller Features

✓ Cartesian pose controller

✓ Smooth interpolation

✓ Workspace checking

✓ Velocity limits

✓ Translation limits

✓ Motion duration

✓ Home pose

✓ Start / Stop services

✓ Relative pose publisher

✓ Cartesian trajectory generation

✓ Final pose holding

---

# Current Limitations

The controller currently accepts only

```
Current Pose

+

(dx,dy,dz)
```

It **cannot yet** move to a fixed world pose.

---

# Next Milestone

Move from

ANY current pose

to

ONE predefined bottle pose.

Instead of

```
Current

+

Δx
```

the controller should accept

```
Bottle Pose

x

y

z

qx

qy

qz

qw
```

and automatically generate a Cartesian trajectory.

Example

```
Robot starts here

↓

Read current pose

↓

Bottle pose

↓

Cartesian interpolation

↓

Move

↓

Hold bottle approach pose
```

---

# Planned Improvements

Version 2

- Continuous desired vs actual XYZ printing

- Quaternion printing

- Translation error

- Rotation error

- Progress percentage

- CSV logging

- Motion statistics

Version 3

- Absolute Cartesian target

Version 4

- RViz interactive marker

Version 5

- Bottle calibration

Version 6

- MoveIt planning

Version 7

- Grasp pose

Version 8

- Dual Panda

Version 9

- Allegro hands

Version 10

- Diffusion Policy

---

# Operating Procedure

1.

Power robot.

2.

Release emergency stop.

3.

Enable FCI.

4.

Launch controller

```bash
roslaunch left_panda_fixed_pose_test left_panda_all.launch
```

5.

Hold enable switch.

6.

Execute motion

```bash
~/bimanual_ws/src/left_panda_fixed_pose_test/scripts/move_relative.sh 0 0 0.04
```

7.

Release enable switch after motion finishes.

---

# Notes

The controller has been validated with successful Cartesian motion.

Next development objective:

```
Random robot pose

↓

Known bottle pose

↓

Cartesian motion

↓

Bottle grasp

↓

Dual-arm bottle opening
```

