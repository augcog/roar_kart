cmake_minimum_required(VERSION 3.5)
project(gokart_hardware_launches)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


set(dependencies
  rclcpp
  std_msgs
  roar_msgs
  roar_gokart_msgs
)

add_executable(roar_gokart_converter src/roar_gokart_converter.cpp)
ament_target_dependencies(roar_gokart_converter
  ${dependencies}
)
# ament_auto_add_executable(roar_gokart_converter src/roar_gokart_converter.cpp)

install(TARGETS
  roar_gokart_converter
  RUNTIME DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY include/
  DESTINATION include/
)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY param DESTINATION share/${PROJECT_NAME})
ament_package()
