cmake_minimum_required(VERSION 3.5)
project(roar_mapping)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY src launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
