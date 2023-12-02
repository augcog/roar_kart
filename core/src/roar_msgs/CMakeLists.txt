cmake_minimum_required(VERSION 3.5)
project(roar_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleControl.msg"
  "msg/VehicleState.msg"
  "srv/ToggleControlSafetySwitch.srv"
  "msg/VehicleStatus.msg"
  "msg/BehaviorStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs gps_msgs sensor_msgs vision_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
