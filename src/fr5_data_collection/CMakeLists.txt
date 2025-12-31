cmake_minimum_required(VERSION 3.0.2)
project(fr5_data_collection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  moveit_commander
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/collect_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
