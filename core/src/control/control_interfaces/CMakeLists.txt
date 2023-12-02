cmake_minimum_required(VERSION 3.5)
project(control_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Control.action"
  DEPENDENCIES nav_msgs
)

ament_package()
