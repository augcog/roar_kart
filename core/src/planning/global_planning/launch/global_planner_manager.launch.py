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


def generate_launch_description():

    ld = launch.LaunchDescription()
    base_path = Path(get_package_share_directory("global_planning"))
    params_file = LaunchConfiguration('params_file')  

    ld.add_action(DeclareLaunchArgument('params_file', 
                                        description="params_file"))
    ld.add_action(LogInfo(msg=f"Global Planning config file path: [{params_file}]"))
    
    global_planner_manager = launch_ros.actions.Node(
        package="global_planning",
        executable="global_planner_manager",
        name="global_planner_manager",
        output="screen",
        namespace="roar",
        parameters=[launch.substitutions.LaunchConfiguration("params_file")],
        remappings=[
            ("/roar/global_path","/roar/global_planning/global_path")
        ]
    )
    ld.add_action(global_planner_manager)

    map_server = Node(
        parameters=[launch.substitutions.LaunchConfiguration("params_file")],
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        namespace='roar',
    )
    ld.add_action(map_server)


    """ Lifecycle Manager """
    lifecycle_manager_global_planning = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_global_planning",
        output="screen",
        namespace="roar",
        parameters=[launch.substitutions.LaunchConfiguration("params_file")],
    )
    ld.add_action(lifecycle_manager_global_planning)

    lifecycle_manager_map_server = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        output="screen",
        namespace="roar",
        parameters=[launch.substitutions.LaunchConfiguration("params_file")],
    )
    ld.add_action(lifecycle_manager_map_server)
    return ld 