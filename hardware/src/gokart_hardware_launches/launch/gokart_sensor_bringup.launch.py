"""_summary_description = Launch file to bring up the gokart sensors.
"""

# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition  # 1
from launch import LaunchDescription

os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "{time}: [{name}] [{severity}]\t{message}"


def generate_launch_description():
    base_path = Path(get_package_share_directory("gokart_hardware_launches"))
    ld = LaunchDescription()

    """
    Vision System
    """

    # LiDAR node
    livox_file_path: Path = (
        base_path
        / "launch"
        / "lidar.launch.py"
    )
    assert livox_file_path.exists()
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_file_path.as_posix()),
        launch_arguments={
            "frame_id": "center_lidar",
            "user_config_path": (
                base_path / "center_livox_lidar_config.json"
            ).as_posix(),
            "lidar_publish_freq": "10.0",
        }.items(),
    )
    # ZED 2i Center
    zed_file_path: Path = (
        Path(get_package_share_directory("gokart_hardware_launches"))
        / "launch"
        / "zed.launch.py"
    )
    zed_config_file_path: Path = Path(base_path / "param" / "zed2i_config.yaml")
    assert zed_file_path.exists(), f"{zed_file_path} does not exist"
    assert zed_config_file_path.exists(), f"{zed_config_file_path} does not exist"
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_file_path.as_posix())
    )

    """
    Localization System
    """
    # GPS 
    gps_launch_path: Path = Path(get_package_share_directory('point_one_gps_driver')) / 'launch' / 'point_one_gps_driver.launch.py'
    assert gps_launch_path.exists(), f"[{gps_launch_path}] does not exist"
    gps_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(gps_launch_path.as_posix()))

    """
    Hardware Control
    """
    arduino_launch_path: Path = Path(get_package_share_directory('arduino_comm')) / 'launch' / 'arduino.launch.py'
    arduino_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(arduino_launch_path.as_posix()))

    # vision systems
    ld.add_action(zed_launch) 
    ld.add_action(lidar_launch)

    # localization systems
    ld.add_action(gps_launch)
    ld.add_action(vehicle_urdf_launch)

    # hardware control
    ld.add_action(arduino_launch)

    return ld


if __name__ == "__main__":
    generate_launch_description()
