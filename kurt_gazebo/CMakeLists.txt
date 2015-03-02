cmake_minimum_required(VERSION 2.8.3)
project(kurt_gazebo)

find_package(catkin REQUIRED COMPONENTS
  uos_gazebo_worlds
  kurt_description
  controller_manager
  gazebo_ros_control
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  uos_gazebo_worlds
  kurt_description
  controller_manager
  gazebo_ros_control
  DEPENDS
)

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
