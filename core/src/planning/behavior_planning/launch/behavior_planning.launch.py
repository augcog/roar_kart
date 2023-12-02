from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import datetime
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import LogInfo

def generate_launch_description():
    ld = LaunchDescription()
    base_path = Path(get_package_share_directory("behavior_planning"))

    # Declare arguments
    ld.add_action(DeclareLaunchArgument("params_file", default_value=(base_path/"param"/"default.yml").as_posix()))

    # behavior_planning
    node = Node(package='behavior_planning',
                executable='default_behavior_planner_exe',
                name='behavior_planning_node',
                namespace="roar",
                output='screen',
                emulate_tty=True,
                parameters=[LaunchConfiguration('params_file')],
                remappings=[
                    ("/roar/vehicle_state", "/roar/vehicle_state"),
                    ("/roar/behavior_status","/roar/behavior/status"),
                    ("controller_set_param_service","/roar/controller_manager/set_parameters_atomically")
                    ],
    )
    ld.add_action(node)
    ld.add_action(LogInfo(msg=f"Behavior Planning launched"))

    # lifecycle manager
    life_cycle_node = Node(package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name='lifecycle_manager_behavior_planning_node',
                namespace="roar",
                output='screen',
                emulate_tty=True,
                parameters=[LaunchConfiguration('params_file')])
    ld.add_action(life_cycle_node)
    return ld 