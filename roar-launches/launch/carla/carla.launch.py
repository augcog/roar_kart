"""
Launches all Carla launch files

"""
import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import LogInfo
from launch.actions import GroupAction
from launch_ros.actions import SetRemap

def generate_launch_description():
    roar_carla_launch_file_path: Path = (
        Path(get_package_share_directory("roar_carla_interface"))
        / "launch"
        / "roar_carla_with_vehicle.launch.py"
    )
    assert (
        roar_carla_launch_file_path.exists()
    ), f"[{roar_carla_launch_file_path}] does not exist"
    carla_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            roar_carla_launch_file_path.as_posix()
        )
    )
    ld = launch.LaunchDescription()
    ld.add_action(carla_client)
    return ld
    
