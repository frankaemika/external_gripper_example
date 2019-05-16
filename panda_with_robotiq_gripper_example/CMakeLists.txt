cmake_minimum_required(VERSION 2.8.3)
project(panda_with_robotiq_gripper_example)

find_package(catkin REQUIRED COMPONENTS
  # TODO(jaeh_ch)
  actionlib #??
  actionlib_msgs
  controller_manager_msgs
  controller_interface
  moveit_commander
  geometry_msgs
  franka_control
  rosparam
  rospy
  robotiq_2f_gripper_msgs
)

find_package(Franka 0.4.0 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

catkin_package(# TODO(jaeh_ch))
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
