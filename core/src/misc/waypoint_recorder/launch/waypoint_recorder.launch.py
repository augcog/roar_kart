import launch
from pathlib import Path 
from ament_index_python.packages import get_package_share_directory
import launch_ros 
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import time 
from launch.conditions import IfCondition 
from launch.actions import LogInfo
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

def generate_launch_description():
    ld = launch.LaunchDescription()
    default_path = f'./data/log_{int(time.time())}'
    ld.add_action(DeclareLaunchArgument("recording_path", default_value=default_path))
    ld.add_action(DeclareLaunchArgument("should_record_to_txt", default_value="True"))
    ld.add_action(DeclareLaunchArgument("should_show_mapviz", default_value="False"))
    ld.add_action(LogInfo(msg=["Recording path: ", LaunchConfiguration("recording_path")]))
    record_node = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration("recording_path"), '/roar/odometry'], 
                   output='screen', 
                   emulate_tty=True)
    ld.add_action(record_node)

    waypoint_txt_node = launch_ros.actions.Node(
        package="waypoint_recorder",
        executable="record_waypoint_to_txt_node",
        name="record_waypoint_to_txt_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("should_record_to_txt")),
        parameters=[
            {"recording_path": LaunchConfiguration("recording_path")},
        ]
    )

    delayed_nodes = TimerAction(period=1.0, actions=[waypoint_txt_node, 
                                                     LogInfo(msg=["odom to txt node launched"])])
    ld.add_action(delayed_nodes)

    return ld 
