cmake_minimum_required(VERSION 3.5)
project(roar-indy-launches)

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(
  include
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
