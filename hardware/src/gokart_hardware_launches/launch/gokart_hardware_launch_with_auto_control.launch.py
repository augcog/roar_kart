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
    converter_node
    """
    converter_node = launch_ros.actions.Node(
        package="gokart_hardware_launches",
        executable="roar_gokart_converter",
        namespace="converter_node",
        output="screen",
        remappings=[("/converter_node/roar_control", "/roar/vehicle/control"),("/converter_node/gokart_control","/vehicle/control")]
    )


    ld = launch.LaunchDescription()
    ld.add_action(converter_node)
    ld.add_action(hardware_launch)
    return ld


if __name__ == "__main__":
    generate_launch_description()
