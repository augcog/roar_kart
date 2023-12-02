"""_summary_description = Launch file to bring up the gokart hardware with manual control
"""

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

os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "{time}: [{name}] [{severity}]\t{message}"


def generate_launch_description():
    base_path = Path(get_package_share_directory("gokart_hardware_launches"))

    """
    Sensor Bring up
    """
    hardware_launch_path: Path = (base_path
        / "launch"
        / "gokart_sensor_bringup.launch.py"
    )
    assert hardware_launch_path.exists(), f"[{hardware_launch_path}] does not exist"
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hardware_launch_path.as_posix())
    )

    """
    Manual Control
    """
    manual_control_launch_path: Path = Path(get_package_share_directory('gokart_manual_controller')) / 'launch' / 'manual_control.launch.py'
    manual_control_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(manual_control_launch_path.as_posix()))


    ld = launch.LaunchDescription()
    ld.add_action(manual_control_launch)
    ld.add_action(hardware_launch)
    return ld


if __name__ == "__main__":
    generate_launch_description()
