from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from pathlib import Path


def generate_launch_description():
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="arduino_comm",
                executable="arduino_comm_node",
                name="arduino_comm_node",
                remappings=[
                    ("ego_vehicle_control", "/roar/gokart/control"),
                    ("vehicle_status", "/roar/gokart/status"),
                ],
            ),
        ]
    )
