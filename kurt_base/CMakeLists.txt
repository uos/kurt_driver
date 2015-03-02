cmake_minimum_required(VERSION 2.8.3)
project(kurt_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  transmission_interface
  gazebo_ros_control
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  transmission_interface
  gazebo_ros_control
  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(kurt_base src/can.cc src/kurt.cc src/kurt_base.cc)
target_link_libraries(kurt_base ${catkin_LIBRARIES})
add_dependencies(kurt_base ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(speedtable src/can.cc src/kurt.cc src/mytime.cc src/speedtable.cc)
target_link_libraries(speedtable ${catkin_LIBRARIES})
add_dependencies(speedtable ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(countticks src/can.cc src/kurt.cc src/mytime.cc src/countticks.cc)
target_link_libraries(countticks ${catkin_LIBRARIES})
add_dependencies(countticks ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS kurt_base speedtable countticks
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY nodes RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY speedtables DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY tools DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
