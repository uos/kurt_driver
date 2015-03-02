cmake_minimum_required(VERSION 2.8.3)
project(kurt_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
  gazebo_plugins
  diffdrive_gazebo_plugin
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  urdf
  xacro
  gazebo_plugins
  diffdrive_gazebo_plugin
  DEPENDS
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY robots DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY tops DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
