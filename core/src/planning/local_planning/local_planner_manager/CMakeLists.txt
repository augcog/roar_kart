cmake_minimum_required(VERSION 3.5)
project(local_planner_manager)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

include_directories(
  include
)


set(${PROJECT_NAME}_SRC
  src/local_planner_manager_node.cpp
)

set(${PROJECT_NAME}_PLUGIN_SRC
  src/plugins/simple_distance_planner.cpp
  src/plugins/simple_frenet_frame_planner.cpp
)

# Create this project as an executable.
ament_auto_add_executable(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_SRC}
)

# Plugins
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_PLUGIN_SRC}
  ${${PROJECT_NAME}_SRC}
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  launch
  params
  include
)