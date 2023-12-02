cmake_minimum_required(VERSION 3.5)
project(costmap_node_manager)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)

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
)

# add executables
add_executable(costmap_node_manager src/costmap_node_manager.cpp)
ament_target_dependencies(costmap_node_manager
  ${dependencies}
)


target_include_directories(costmap_node_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS costmap_node_manager
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
  

ament_package()
