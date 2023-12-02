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
    ld = launch.LaunchDescription()
    params_file = LaunchConfiguration('params_file')  
    manual_control = LaunchConfiguration('manual_control', default="False")  
    ld.add_action(DeclareLaunchArgument(
        'params_file',
        description='Full path to the ROS2 parameters file to use for all launched nodes'))

    """ URDF """
    base_path = Path(get_package_share_directory("roar-urdf"))
    urdf_launch_path = base_path / "launch" / "state_publisher.launch.py"
    assert urdf_launch_path.exists(), f"{urdf_launch_path} does not exist"
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urdf_launch_path.as_posix()),
        launch_arguments={
            "param_file": params_file
        }.items()
    )
    ld.add_action(urdf_launch)

    """
    Costmap
    """
    costmap_manager_launch_file_path: Path = (
        Path(get_package_share_directory("costmap_node_manager"))
        / "launch"
        / "costmap_node_manager.launch.py"
    )
    assert costmap_manager_launch_file_path.exists(), f"{costmap_manager_launch_file_path} does not exist"
    costmap_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(costmap_manager_launch_file_path.as_posix()),
        launch_arguments={
            "params_file": params_file
        }.items())
    ld.add_action(costmap_manager)

    """ Global Planner """
    global_planner_path = Path(get_package_share_directory("global_planning")) / "launch" / "global_planner_manager.launch.py"
    assert global_planner_path.exists(), f"{global_planner_path} does not exist"
    global_planner_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(global_planner_path.as_posix()),
        launch_arguments={
            "params_file": params_file
        }.items())
    ld.add_action(global_planner_launch)
    
    """ Local Planner """
    local_planner_manager_launch_file_path: Path = (
        Path(get_package_share_directory("local_planner_manager"))
        / "launch"
        / "local_planner_manager.launch.py"
    )
    assert local_planner_manager_launch_file_path.exists(), f"{local_planner_manager_launch_file_path} does not exist"
    local_planner_manager = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            local_planner_manager_launch_file_path.as_posix()
        ),
        launch_arguments={
            "params_files": params_file
        }.items())
    ld.add_action(local_planner_manager)


    """ Safety Controller """
    controller_manager_launch_file_path: Path = (Path(get_package_share_directory("controller_manager")) / "launch" / "controller.launch.py")
    assert controller_manager_launch_file_path.exists(), f"{controller_manager_launch_file_path} does not exist"
    controller_manager = IncludeLaunchDescription(PythonLaunchDescriptionSource(controller_manager_launch_file_path.as_posix()),
        launch_arguments={
            "params_file": params_file,
            "manual_control": manual_control
        }.items())
    ld.add_action(controller_manager)
    ld.add_action(LogInfo(msg=["Controller Manager launched"]))


    """ State Manager """
    state_manager_launch_file_path: Path = (Path(get_package_share_directory("vehicle_state_manager")) / "launch" / "vehicle_state_manager.launch.py")
    assert state_manager_launch_file_path.exists(), f"{state_manager_launch_file_path} does not exist"
    state_manager = IncludeLaunchDescription(PythonLaunchDescriptionSource(state_manager_launch_file_path.as_posix()),
        launch_arguments={
            "params_file": params_file
        }.items())
    ld.add_action(state_manager)
    ld.add_action(LogInfo(msg=["State Manager launched"]))


    """ Behavior Planner """

    behavior_planner_launch_file_path: Path = (Path(get_package_share_directory("behavior_planning")) / "launch" / "behavior_planning.launch.py")
    assert behavior_planner_launch_file_path.exists(), f"{behavior_planner_launch_file_path} does not exist"
    behavior_planner = IncludeLaunchDescription(PythonLaunchDescriptionSource(behavior_planner_launch_file_path.as_posix()),
        launch_arguments={
            "params_file": params_file
        }.items())
    ld.add_action(behavior_planner)

    return ld
    
