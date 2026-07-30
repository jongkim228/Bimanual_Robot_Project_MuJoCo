cmake_minimum_required(VERSION 3.0.2)
project(left_panda_fixed_pose_test)

add_compile_options(-std=c++17 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  eigen_conversions
  franka_hw
  franka_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  roscpp
  std_srvs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES left_panda_fixed_pose_test
  CATKIN_DEPENDS
    controller_interface
    eigen_conversions
    franka_hw
    franka_msgs
    geometry_msgs
    hardware_interface
    pluginlib
    roscpp
    std_srvs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(left_panda_fixed_pose_test
  src/fixed_pose_controller.cpp
)

target_link_libraries(left_panda_fixed_pose_test
  ${catkin_LIBRARIES}
)

add_dependencies(left_panda_fixed_pose_test
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS left_panda_fixed_pose_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES left_panda_fixed_pose_test.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)