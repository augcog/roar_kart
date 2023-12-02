source install/setup.bash
ros2 service call /roar/controller_manager/safety_toggle roar_msgs/srv/ToggleControlSafetySwitch "{is_safety_on: True}"