```bash
apptainer shell --nv --writable --bind /uolstore/home/users/sc23j3k:/uolstore/home/users/sc23j3k /uolstore/home/users/sc23j3k/ros_noetic_sandbox/
```

```bash
export LD_LIBRARY_PATH=/uolstore/home/users/sc23j3k/miniforge3/lib:/uolstore/home/users/sc23j3k/.mujoco/mujoco-3.3.5/lib:$LD_LIBRARY_PATH
```

```bash
source ~/Bimanual_Robot_Project_MuJoCo/bimanual_ws/devel/setup.bash
```

```bash
roslaunch bimanual_robot_project_mujoco bimanual_control.launch 2>&1 | tee /tmp/launch_output6.log
```
