# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import os

import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from pathlib import Path


def generate_launch_description():

    base_path = os.path.realpath(get_package_share_directory("roar_carla_interface"))
    vehicle_config_path = base_path + "/param/berkeley_gokart.json"
    base_path = Path(get_package_share_directory("roar_carla_interface"))
    config_path: Path = base_path / "param" / "interface_config.yaml"
    assert config_path.exists(), f"{config_path} does not exist"

    ld = launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="host", default_value="localhost"
            ),
            launch.actions.DeclareLaunchArgument(name="port", default_value="2000"),
            launch.actions.DeclareLaunchArgument(name="timeout", default_value="5"),
            launch.actions.DeclareLaunchArgument(
                name="role_name", default_value="ego_vehicle"
            ),
            launch.actions.DeclareLaunchArgument(
                name="vehicle_filter", default_value="vehicle.*"
            ),
            launch.actions.DeclareLaunchArgument(
                name="spawn_point",
                default_value="11.90, 4.70, 1.0,0.0,0.0,90.0" # x,y,z,roll,pitch,yaw

            ),
            launch.actions.DeclareLaunchArgument(
                name="spawn_point_ego_vehicle", default_value="spawn_point_hero0"
            ),
            launch.actions.DeclareLaunchArgument(name="town", default_value="Town04"),
            launch.actions.DeclareLaunchArgument(name="passive", default_value="False"),
            launch.actions.DeclareLaunchArgument(
                name="synchronous_mode_wait_for_vehicle_control_command",
                default_value="False",
            ),
            launch.actions.DeclareLaunchArgument(
                name="fixed_delta_seconds", default_value="0.05"
            ),
            launch.actions.DeclareLaunchArgument(
                name="objects_definition_file", default_value=vehicle_config_path
            ),
            launch.actions.DeclareLaunchArgument(
                name="role_name", default_value="ego_vehicle"
            ),
            launch.actions.DeclareLaunchArgument(
                name="spawn_sensors_only", default_value="False"
            ),
            launch.actions.DeclareLaunchArgument(
                name="control_id", default_value="control"
            ),
            launch_ros.actions.Node(
            package='carla_ros_bridge',
            executable='bridge',
            name='carla_ros_bridge',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            remappings=[
                ('/carla/ego_vehicle/odometry', '/roar/odometry'),
                ('/carla/ego_vehicle/front_depth/camera_info', '/roar/front/depth/camera_info'),
                ('/carla/ego_vehicle/front_rgb/camera_info', '/roar/front/rgb/camera_info'),
                ('/carla/ego_vehicle/front_depth/image', '/roar/front/depth/image'),
                ('/carla/ego_vehicle/front_rgb/image', '/roar/front/rgb/image'),
                ('/carla/ego_vehicle/imu', '/roar/imu'),
                ('/carla/ego_vehicle/gnss', '/roar/gnss'),
                ('/carla/ego_vehicle/center_lidar', '/roar/front/lidar'),
            ],
            parameters=[{
                    "host": launch.substitutions.LaunchConfiguration("host"),
                    "port": launch.substitutions.LaunchConfiguration("port"),
                    "town": launch.substitutions.LaunchConfiguration("town"),
                    "timeout": launch.substitutions.LaunchConfiguration("timeout"),
                    "passive": launch.substitutions.LaunchConfiguration("passive"),
                    "synchronous_mode_wait_for_vehicle_control_command": launch.substitutions.LaunchConfiguration(
                        "synchronous_mode_wait_for_vehicle_control_command"
                    ),
                    "fixed_delta_seconds": launch.substitutions.LaunchConfiguration(
                        "fixed_delta_seconds"
                    )}
                    ]
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("carla_spawn_objects"),
                        "carla_spawn_objects.launch.py",
                    )
                ),
                launch_arguments={
                    "host": launch.substitutions.LaunchConfiguration("host"),
                    "port": launch.substitutions.LaunchConfiguration("port"),
                    "timeout": launch.substitutions.LaunchConfiguration("timeout"),
                    "vehicle_filter": launch.substitutions.LaunchConfiguration(
                        "vehicle_filter"
                    ),
                    "role_name": launch.substitutions.LaunchConfiguration("role_name"),
                    "spawn_point": launch.substitutions.LaunchConfiguration(
                        "spawn_point"
                    ),
                    "spawn_point_ego_vehicle": LaunchConfiguration("spawn_point"),
                    "objects_definition_file": launch.substitutions.LaunchConfiguration(
                        "objects_definition_file"
                    ),
                    "spawn_sensors_only": launch.substitutions.LaunchConfiguration(
                        "spawn_sensors_only"
                    ),
                }.items(),
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("carla_spawn_objects"),
                        "set_initial_pose.launch.py",
                    )
                ),
                launch_arguments={
                    "role_name": launch.substitutions.LaunchConfiguration("role_name"),
                    "control_id": launch.substitutions.LaunchConfiguration(
                        "control_id"
                    ),
                }.items(),
            ),
            launch_ros.actions.Node(
                package="roar_carla_interface",
                executable="carla_pid_node",
                name="roar_carla_pid_node",
                parameters=[config_path.as_posix()],
                remappings=[
                    ("vehicle_control", "/sim/vehicle/control"),
                    ("carla_control", "/carla/ego_vehicle/vehicle_control_cmd"),
                    ("vehicle_status", "/carla/ego_vehicle/vehicle_status"),
                ],
            ),
            launch_ros.actions.Node(
                package="roar_carla_interface",
                executable="roar_carla_state_publisher",
                name="roar_carla_state_publisher",
                parameters=[config_path.as_posix()],
                remappings=[
                    ("vehicle_status", "/roar/vehicle/status"),
                    ("vehicle_control", "/roar/vehicle/control"),
                    ("vehicle_speed", "/carla/ego_vehicle/speedometer")
                ],
            ),
            launch_ros.actions.Node(
                package="roar_carla_interface",
                executable="roar_carla_converter_node",
                name="roar_carla_converter_node",
                remappings=[
                    ("sim_vehicle_control", "/sim/vehicle/control"),
                    ("roar_vehicle_control", "/roar/vehicle/control"),
                ]
            ),
            launch_ros.actions.Node(
                package="roar_carla_interface",
                executable="carla_depth_img_converter_node",
                name="carla_depth_img_converter_node",
                remappings=[
                    ("carla_depth", "/roar/front/depth/image"),
                    ("carla_image_processed", "/roar/front/depth/image/processed"),
                ]
            )
        ]
    )

    static_transform_publisher = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="carla_ros_static_transform_publisher_ego_vehicle_base_link",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "ego_vehicle", "base_link"],
    )
    ld.add_action(static_transform_publisher)


    static_transform_publisher_map_odom = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="carla_ros_static_transform_publisher_map_odom",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )
    ld.add_action(static_transform_publisher_map_odom)

    return ld


if __name__ == "__main__":
    generate_launch_description()
