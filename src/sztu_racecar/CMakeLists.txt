cmake_minimum_required(VERSION 2.8.3)
project(sztu_racecar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  joy
  move_base
  tf
  roscpp
  rospy
  std_msgs
  visualization_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/imu.cfg)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_racecar
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

find_package(PCL REQUIRED QUIET)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY src scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(pure_pursuit
	src/pure_pursuit.cpp 
	src/PID.h
)
target_link_libraries(pure_pursuit
	${catkin_LIBRARIES}
)

add_executable(path_recording
	src/path_recording.cpp 
)
target_link_libraries(path_recording
	${catkin_LIBRARIES}
)





