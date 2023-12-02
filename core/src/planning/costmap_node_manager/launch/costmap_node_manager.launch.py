import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    base_path = Path(get_package_share_directory("costmap_node_manager"))
    default_config_file_path: Path = base_path / "config" / "config.yaml"

    pointcloud_to_laser = Node(
        name="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        package="pointcloud_to_laserscan",
        parameters=[launch.substitutions.LaunchConfiguration("params_file")],
        remappings=[
            ("cloud_in", "/roar/front/lidar"),
            ("scan", "/roar/front/scan"),
        ],
    )
    
    costmap_manager = Node(
        executable="costmap_node_manager",
        package="costmap_node_manager",
        parameters=[
            launch.substitutions.LaunchConfiguration("params_file")
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_costmap",
        output="screen",
        parameters=[launch.substitutions.LaunchConfiguration("params_file")],
    )

    ld = launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="params_file",
                default_value=default_config_file_path.as_posix(),
            ),
            costmap_manager,
            pointcloud_to_laser,
            lifecycle_manager,
            LogInfo(msg=["Costmap launched"])
        ]
    )
    return ld
