cmake_minimum_required(VERSION 2.8.3)
project(kurt_bringup)

find_package(catkin REQUIRED COMPONENTS
  kurt_description
  kurt_base
  uos_diffdrive_teleop
  robot_state_publisher
  sicktoolbox_wrapper
  laser_assembler
  rotunit_snapshotter
  robot_self_filter
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  kurt_description
  kurt_base
  uos_diffdrive_teleop
  robot_state_publisher
  sicktoolbox_wrapper
  laser_assembler
  rotunit_snapshotter
  robot_self_filter
  DEPENDS
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
