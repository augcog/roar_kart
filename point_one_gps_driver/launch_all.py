import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
#this is a top level launch file to start the gps waypoint following process
#This should launch the point_one_nav gps driver using the launch file (should be no user arguments)

def generate_launch_description():
    # Get the package directory

    my_package_dir = os.path.join(get_package_share_directory('point_one_gps_driver'),'launch')
    # Define the launch description
    ld = LaunchDescription()

    # Launch my_launch_file.py
    gps_driver_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([my_package_dir, '/point_one_gps_driver.launch.py'])
    )
    ld.add_action(gps_driver_launch_file)
    #use execute process to do rosbag record 
    spawn_turtle = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )

    # Other nodes and configurations

    return ld

