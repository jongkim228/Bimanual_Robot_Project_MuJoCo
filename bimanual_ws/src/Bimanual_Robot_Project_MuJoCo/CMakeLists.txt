cmake_minimum_required(VERSION 3.0.2)
project(bimanual_robot_project_mujoco)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  main.py
  diffik_nullspace.py
  init.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)