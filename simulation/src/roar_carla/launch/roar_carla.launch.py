# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import os

import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    manual_control_args = DeclareLaunchArgument(
        "carla_manual_control",
        default_value="False",  # default_value=[], has the same problem
        description="True to start manual control, false otherwise",
    )

    ld = launch.LaunchDescription(
        [
            manual_control_args,
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("roar_carla_interface"),
                        "launch",
                        "roar_carla_with_vehicle.launch.py",
                    )
                )
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("roar_carla_manual_controller"),
                        "launch",
                        "roar_carla_manual_controller.launch.py",
                    )
                ),
                condition=IfCondition(LaunchConfiguration("carla_manual_control")),  # 3
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
