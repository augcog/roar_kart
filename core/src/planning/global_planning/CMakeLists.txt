cmake_minimum_required(VERSION 3.5)
project(global_planning)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
ament_auto_find_build_dependencies()
find_package(GeographicLib REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  visualization_msgs
  std_msgs
  message_filters
  sensor_msgs
  nav_msgs
  tf2_ros
  GeographicLib
  planning_interfaces
  nav_msgs
)

ament_auto_add_executable(global_planner_manager 
  src/global_planner_manager.cpp 
  src/global_planners/race_planner.cpp
  src/global_planners/parking_planner.cpp
  src/global_planners/potential_field_algo.cpp
)

# add_executable(global_planner_node src/global_planner_node.cpp)
# target_include_directories(global_planner_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
ament_target_dependencies(global_planner_manager ${dependencies})

# ament_auto_add_library(${PROJECT_NAME} SHARED
#   ${${PROJECT_NAME}_IMPL}
#   global_planner_manager
# )


install(DIRECTORY include DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY param DESTINATION share/${PROJECT_NAME})


# ament_package()
ament_auto_package()
