cmake_minimum_required(VERSION 3.5)
project(roar_gokart_localization)

# Require that dependencies from package.xml be available.
find_package(ament_cmake_auto REQUIRED)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gps_msgs REQUIRED)
include_directories(
  include
)

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

add_executable(localizationHack src/localization_hack.cpp)

ament_target_dependencies(localizationHack ${dependencies})

target_include_directories(localizationHack PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS localizationHack
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
ament_package()