cmake_minimum_required(VERSION 3.14)
project(livox_ros_driver2)

if("$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
  add_message_files(FILES CustomPoint.msg CustomMsg.msg)
  generate_messages(DEPENDENCIES std_msgs)
  catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
  install(FILES LICENSE.txt README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  return()
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Keep the upstream generator target and library names for existing consumers.
rosidl_generate_interfaces(livox_interfaces2
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)
ament_export_dependencies(rosidl_default_runtime)
install(FILES LICENSE.txt README.md DESTINATION share/${PROJECT_NAME})
ament_package()
