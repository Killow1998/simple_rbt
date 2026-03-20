cmake_minimum_required(VERSION 3.0.2)
project(simple_rbt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  move_base_msgs 
  actionlib
  tf
  tf2
  tf2_ros
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

# WPR Gazebo plugin
add_library(wpr_plugin SHARED src/wpr_plugin.cpp)
target_link_libraries(wpr_plugin ${GAZEBO_LIBRARIES})

# Tracer skid steer controller
add_library(tracer_gazebo STATIC src/tracer_skid_steer.cpp)
target_link_libraries(tracer_gazebo ${catkin_LIBRARIES})

add_executable(tracer_skid_steer_controller src/tracer_skid_steer_controller.cpp)
target_link_libraries(tracer_skid_steer_controller tracer_gazebo ${catkin_LIBRARIES})

# Limo Ackermann Plugin
add_library(gazebo_ros_ackerman_drive SHARED src/gazebo_ros_ackerman_drive.cpp)
target_link_libraries(gazebo_ros_ackerman_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(dummy_cmd_vel src/dummy_cmd_vel.cpp)
target_link_libraries(dummy_cmd_vel ${catkin_LIBRARIES})

# Omni Commander Script
catkin_install_python(PROGRAMS
  scripts/omni_commander.py
  scripts/go2w_high_level_publisher.py
  scripts/go2w_joint_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
