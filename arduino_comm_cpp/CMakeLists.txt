cmake_minimum_required(VERSION 3.8)
project(arduino_comm_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

## ADDED
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(roar_gokart_msgs REQUIRED) ### MSG

set(RAPIDJSON_INCLUDE_DIRS /usr/include/rapidjson/)  # Replace with the actual path to RapidJSON
# find_package(rapidjson REQUIRED) ### ADDED


include_directories(
  include
  # ${RAPIDJSON_INCLUDE_DIRS} ### ADD THIS LINE
)


add_executable(arduino_comm_node src/arduino_comm_node.cpp)
ament_target_dependencies(arduino_comm_node rclcpp std_msgs roar_gokart_msgs) ## ADDED ROAR_GOKART_MSGS

# target_link_libraries(arduino_communicator_node ${RAPIDJSON_INCLUDE_DIRS}) ### ADD THIS LINE


install(TARGETS
  arduino_comm_node
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(arduino_comm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
