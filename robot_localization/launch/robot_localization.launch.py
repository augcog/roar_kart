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

    # """Static publisher"""
    # static_publisher = launch_ros.actions.Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="swri_transform",
    #     arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    # )
    # ld.add_action(static_publisher)

    """localization_hack"""
    odom_publisher = launch_ros.actions.Node(
        package="roar_gokart_localization",
        executable="localizationHack",
        name="localization_hack",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("roar_gokart_localization"),
                "params",
                "configs.yaml",
            ),
        ],
        remappings=[
            ("/gps/fix", "/gps/fix"),
            ("/gps/imu", "/gps/imu"),
            ("/gps/pose", "/gps/pose"),
        ],
    )
    ld.add_action(odom_publisher)
    return ld
