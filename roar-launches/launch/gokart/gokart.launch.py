"""
Launches all GoKart launch files

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

def generate_launch_description():
    gokart_launches_path: Path = (
        Path(get_package_share_directory("gokart_hardware_launches"))
        / "launch"
        / "gokart_hardware_launch_with_auto_control.launch.py"
    )
    assert (
        gokart_launches_path.exists()
    ), f"[{gokart_launches_path}] does not exist"
    gokart_launches = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            gokart_launches_path.as_posix()
        )
    )
    ld = launch.LaunchDescription()
    ld.add_action(gokart_launches)
    return ld
    
