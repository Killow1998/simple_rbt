cmake_minimum_required(VERSION 3.0.2)
project(simple_rbt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  move_base_msgs 
  actionlib
  tf
  cv_bridge 
  image_transport
  nav_msgs
  controller_manager
  joint_state_controller
  robot_state_publisher
)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_rbt
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(wpr_plugin SHARED src/wpr_plugin.cpp)
target_link_libraries(wpr_plugin ${GAZEBO_LIBRARIES})
