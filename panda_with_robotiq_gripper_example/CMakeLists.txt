cmake_minimum_required(VERSION 2.8.3)
project(panda_with_robotiq_gripper_example)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  controller_manager_msgs
  franka_control
  franka_description
  geometry_msgs
  panda_moveit_config
  robotiq_2f_gripper_control
  robotiq_2f_gripper_msgs
  rosparam
  rospy
)

find_package(Franka 0.4.0 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    actionlib_msgs
    actionlib
    controller_manager_msgs
    franka_control
    franka_description
    geometry_msgs
    panda_moveit_config
    robotiq_2f_gripper_control
    robotiq_2f_gripper_msgs
    rosparam
    rospy
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/panda_with_robotiq_gripper_example.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
