import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import launch_ros
from pathlib import Path

pkg_share = launch_ros.substitutions.FindPackageShare(package="livox_ros2_driver").find(
    "livox_ros2_driver"
)
cur_config_path = Path(pkg_share) / "livox_ros2_driver" / "config"
user_config_path = os.path.join(cur_config_path, "livox_lidar_config.json")


def generate_launch_description():
    livox_driver = Node(
        package="livox_ros2_driver",
        executable="livox_ros2_driver_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {
                "xfer_format": 0  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
            },
            {"multi_topic": 1},
            {"data_src": 0},
            {
                "publish_freq": launch.substitutions.LaunchConfiguration(
                    "lidar_publish_freq"
                ),
            },
            {"output_data_type": 0},
            {
                "frame_id": launch.substitutions.LaunchConfiguration("frame_id"),
            },
            {
                "user_config_path": launch.substitutions.LaunchConfiguration(
                    "user_config_path"
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="lidar_publish_freq",
                default_value="10.0",
            ),
            launch.actions.DeclareLaunchArgument(
                name="frame_id",
                default_value="lidar",
            ),
            launch.actions.DeclareLaunchArgument(
                name="user_config_path",
                default_value=user_config_path,
            ),
            livox_driver,
        ]
    )
