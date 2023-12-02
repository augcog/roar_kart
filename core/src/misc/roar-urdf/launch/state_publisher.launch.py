# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="roar-urdf"
    ).find("roar-urdf")
    default_model_path = os.path.join(pkg_share, "src/gokart/main.urdf")

    joint_state_publisher_args = DeclareLaunchArgument(
        "joint_state_publisher_args",
        default_value="False",  # default_value=[], has the same problem
        description="True to start joint_state_publisher_args, false otherwise",
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )

    return launch.LaunchDescription(
        [
            joint_state_publisher_args,
            launch.actions.DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            robot_state_publisher_node,
        ]
    )
