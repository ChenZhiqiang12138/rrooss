cmake_minimum_required(VERSION 2.8.3)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  move_base
  tf
  roscpp
  rospy
  std_msgs
  visualization_msgs
  dynamic_reconfigure
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

SET(LIB_ART_DRIVER_LIB art_controller.cpp)
ADD_LIBRARY(art_controller SHARED ${LIB_ART_DRIVER_LIB})
ADD_LIBRARY (art_controller_STATIC STATIC ${LIB_ART_DRIVER_LIB})
SET_TARGET_PROPERTIES(art_controller_STATIC PROPERTIES OUTPUT_NAME "art_controller")

