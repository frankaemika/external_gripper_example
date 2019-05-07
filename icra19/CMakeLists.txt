cmake_minimum_required(VERSION 2.8.3)
project(icra19)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  franka_control
  franka_hw
  dynamic_reconfigure
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
)

find_package(Franka 0.4.0 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
    )
#generate_messages()

catkin_package()

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
