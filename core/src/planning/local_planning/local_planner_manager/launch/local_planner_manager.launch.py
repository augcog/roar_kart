# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.actions import LogInfo

def generate_launch_description():
    ld = launch.LaunchDescription()
    base_path = Path(get_package_share_directory("local_planner_manager"))

    config_file = base_path / "params" / "configs.yaml"
    assert config_file.exists()
    params_file = LaunchConfiguration('params_file')  

    ld.add_action(LogInfo(msg=[f"Local Planner config file: [{config_file}]"]))

    ld.add_action(launch.actions.DeclareLaunchArgument(name="params_file",
                                                       default_value=config_file.as_posix()))
    
    local_planner_manager_node = Node(
        name="local_planner_manager_node",
        executable="local_planner_manager_node",
        package="local_planner_manager",
        parameters=[launch.substitutions.LaunchConfiguration("params_file")],
        namespace="roar",
        emulate_tty=True,
        remappings=[
            ("/odometry","/roar/odometry"),
            ("/footprint","/local_costmap/published_footprint"),
            ("/global_path", "/roar/global_planning/global_path"),
            ("/occupancy_map", "/local_costmap/costmap"),
            ("/best_path", "/roar/local_planning/best_path"),
        ]
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_local_planning",
        output="screen",
        parameters=[launch.substitutions.LaunchConfiguration("params_file")],

    )

    # node
    ld.add_action(local_planner_manager_node)
    ld.add_action(lifecycle_manager)

    return ld
