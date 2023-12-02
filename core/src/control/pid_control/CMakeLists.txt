cmake_minimum_required(VERSION 3.5)
project(pid_control)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_interfaces REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
)
set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  nav2_msgs
  nav2_util
  nav2_core
  control_interfaces
  control_toolbox
  ackermann_msgs
  std_msgs
  visualization_msgs
)

add_executable(pid_control_node src/pid_control_node.cpp)
ament_target_dependencies(pid_control_node
  ${dependencies}
)

target_include_directories(pid_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS pid_control_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
