cmake_minimum_required(VERSION 2.8.3)
project(imu_recalibration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  sensor_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  rospy
  tf
  sensor_msgs
  nav_msgs
  geometry_msgs
  DEPENDS
)

install(DIRECTORY nodes DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
