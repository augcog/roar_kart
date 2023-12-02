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

    ld = LaunchDescription()
    odom_publisher = launch_ros.actions.Node(
        package="roar_septentrio_interface",
        executable="roar_septentrio_interface_node",
        name="septentrio_interface_node",
        namespace="roar",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("roar_septentrio_interface"),
                "param",
                "config.yaml",
            ),
        ],
        remappings=
        [
            ("/roar/odometry", "/roar/odometry"),
            ("/gps/fix", "/gps/gps"),
        ]
    )
    ld.add_action(odom_publisher)
    return ld 