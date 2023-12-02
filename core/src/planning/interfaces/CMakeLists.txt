cmake_minimum_required(VERSION 3.5)
project(planning_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Trajectories.msg"
  "msg/TrajectoryScore.msg"
  "msg/Trajectory.msg"
  "srv/TrajectoryScoring.srv"
  "srv/ResetTrajectoryPicker.srv"
  "srv/LoadMap.srv"
  "action/TrajectoryGeneration.action"
  DEPENDENCIES nav_msgs nav2_msgs geometry_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
