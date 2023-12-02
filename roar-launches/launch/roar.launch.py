"""
main ROAR launch file
"""

import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import LogInfo
import time 
from launch.actions import ExecuteProcess



def generate_launch_description():
    ld = launch.LaunchDescription()
    params_file = LaunchConfiguration('params_file')    
    
    ld.add_action(DeclareLaunchArgument(
        'params_file',
        description='Full path to the ROS2 parameters file to use for all launched nodes'))

    """
    Gokart section
    """
    should_launch_gokart = DeclareLaunchArgument('gokart', default_value="False", description="Launches gokart drivers") 
    ld.add_action(should_launch_gokart)

    gokart_launch_path: Path = (
        Path(get_package_share_directory("roar-indy-launches"))
        / "launch" / "gokart"
        / "gokart.launch.py"
    )
    assert (
        gokart_launch_path.exists()
    ), f"[{gokart_launch_path}] does not exist"
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gokart_launch_path.as_posix()),
        condition=IfCondition(LaunchConfiguration('gokart', default=False))  
    )
    ld.add_action(hardware_launch)
    ld.add_action(LogInfo(msg=["Gokart launched"], condition=IfCondition(LaunchConfiguration('gokart', default=False))))

    """
    Carla section
    """
    should_launch_carla_client = DeclareLaunchArgument('carla', default_value="False", description="Launches carla client") 
    ld.add_action(should_launch_carla_client)

    carla_launch_path: Path = (
        Path(get_package_share_directory("roar-indy-launches"))
        / "launch" / "carla"
        / "carla.launch.py"
    )
    assert (
        carla_launch_path.exists()
    ), f"[{carla_launch_path}] does not exist"
    carla_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(carla_launch_path.as_posix()),
        condition=IfCondition(LaunchConfiguration('carla', default=False))  
    )
    ld.add_action(carla_launch)
    ld.add_action(LogInfo(msg=["carla launched"], condition=IfCondition(LaunchConfiguration('carla', default=False))))

    """
    Manual control section
    """
    should_launch_manual_control = DeclareLaunchArgument('manual_control', default_value="False", description="Launch manual control")
    ld.add_action(should_launch_manual_control)
    manual_control_launch_path: Path = Path(get_package_share_directory("roar_manual_control")) / "launch" / "roar_manual_control.launch.py"
    assert (
        manual_control_launch_path.exists()
    ), f"[{manual_control_launch_path}] does not exist"
    manual_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(manual_control_launch_path.as_posix()),
        condition=IfCondition(LaunchConfiguration('manual_control', default=False))
        )
    ld.add_action(LogInfo(msg=["manual_control launched"], condition=IfCondition(LaunchConfiguration('manual_control', default=False))))
    ld.add_action(manual_control_launch)

    """
    Core auto drive section
    """
    should_launch_core = DeclareLaunchArgument('core', default_value="False", description="Launches core auto drive logics")
    ld.add_action(should_launch_core)

    core_launch_path: Path = (
        Path(get_package_share_directory("roar-indy-launches"))
        / "launch" / "core"
        / "core.launch.py"
    )
    assert (
        core_launch_path.exists()
    ), f"[{core_launch_path}] does not exist"
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(core_launch_path.as_posix()),
        condition=IfCondition(LaunchConfiguration('core', default=False)),
        launch_arguments={
            "params_file": params_file,
            "manual_control": LaunchConfiguration('manual_control', default=False)
        }.items()
    )
    ld.add_action(core_launch)
    ld.add_action(LogInfo(msg=["Core launched"], condition=IfCondition(LaunchConfiguration('core', default=False))))

    """
    Visualization section
    """
    should_launch_visualization = DeclareLaunchArgument('visualization', default_value="False", description="Launch visualization")
    ld.add_action(should_launch_visualization)



    visualization_launch_path: Path = (
        Path(get_package_share_directory("roar-indy-launches"))
        / "launch" / "visualization"
        / "visualization.launch.py"
    )
    assert (
        visualization_launch_path.exists()
    ), f"[{visualization_launch_path}] does not exist"
    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(visualization_launch_path.as_posix()),
        condition=IfCondition(LaunchConfiguration('visualization', default=False))
    )
    ld.add_action(visualization_launch)
    ld.add_action(LogInfo(msg=["Visualization launched"], condition=IfCondition(LaunchConfiguration('visualization', default=False))))

    """launch waypoint recording"""
    should_record_waypoint = DeclareLaunchArgument('should_record_waypoint', default_value="False", description="Launch waypoint recording")
    ld.add_action(should_record_waypoint)
    waypoint_recording_launch_path: Path = Path(get_package_share_directory("waypoint_recorder")) / "launch" / "waypoint_recorder.launch.py"
    assert (
        waypoint_recording_launch_path.exists()
    ), f"[{waypoint_recording_launch_path}] does not exist"
    waypoint_recording_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waypoint_recording_launch_path.as_posix()),
        condition=IfCondition(LaunchConfiguration('should_record_waypoint', default=False)),
        )
    ld.add_action(LogInfo(msg=["recording waypoint"], condition=IfCondition(LaunchConfiguration('should_record_waypoint', default=False))))
    ld.add_action(waypoint_recording_launch)


    """bag record"""
    ld.add_action(DeclareLaunchArgument('record', default_value="False", description="record bag file"))
    default_path = f'./data/bag_{int(time.time())}'
    ld.add_action(DeclareLaunchArgument("recording_path", default_value=default_path))
    record_node = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration("recording_path"), '-a'], 
                output='screen', 
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration('record', default=False))
                )
    ld.add_action(record_node)


    """base station client"""
    should_launch_base_station = DeclareLaunchArgument('base_station', default_value="True", description="Launch base station")
    ld.add_action(should_launch_base_station)
    base_station_launch_path: Path = Path(get_package_share_directory("roar-base-station")) / "launch" / "main.launch.py"
    assert base_station_launch_path.exists(), f"{base_station_launch_path} does not exist"
    base_station_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_station_launch_path.as_posix()),
        condition=IfCondition(LaunchConfiguration('base_station', default=False)))
    ld.add_action(base_station_launch)


    """mapping"""
    should_launch_mapping = DeclareLaunchArgument('mapping', default_value="False", description="Launch mapping")
    ld.add_action(should_launch_mapping)
    mapping_launch_path: Path = Path(get_package_share_directory("roar_mapping")) / "launch" / "mapping.launch.py"
    assert mapping_launch_path.exists(), f"{mapping_launch_path} does not exist"
    mapping_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(mapping_launch_path.as_posix()), condition=IfCondition(LaunchConfiguration('mapping', default=False)))
    ld.add_action(mapping_launch)


    ld.add_action(SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'
    ))

    ld.add_action(SetEnvironmentVariable(
        'RCUTILS_COLORIZED_OUTPUT', '1'
    ))
    return ld
    
