cmake_minimum_required(VERSION 3.5)
project(roar_septentrio_interface)

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
  gps_msgs
)

ament_auto_add_executable(roar_septentrio_interface_node src/roar_septentrio_interface_node.cpp)
ament_target_dependencies(roar_septentrio_interface_node ${dependencies})


install(TARGETS roar_septentrio_interface_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY param DESTINATION share/${PROJECT_NAME})
ament_package()
