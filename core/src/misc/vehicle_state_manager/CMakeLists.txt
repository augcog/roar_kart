cmake_minimum_required(VERSION 3.5)
project(vehicle_state_manager)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

include_directories(
  include
)
set(${PROJECT_NAME}_SRC
  src/vehicle_state_manager_node.cpp
)
ament_auto_add_executable(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_SRC}
)


# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  launch
  include
  params
)
