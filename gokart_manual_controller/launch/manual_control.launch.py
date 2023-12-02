from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    base_path = Path(get_package_share_directory("gokart_manual_controller"))
    config_path: Path = base_path / "params" / "config.yaml"
    manual_control_node = Node(
        package="gokart_manual_controller",
        executable="manual_controller_node",
        name="manual_control",
        parameters=[config_path.as_posix()],
        remappings=[
            ("/vehicle/control", "/vehicle/control"),
            ("rgb_topic", "/zed2i/center_camera/rgb/image_rect_color"),
        ],
    )
    ld = LaunchDescription()

    ld.add_action(manual_control_node)
    return ld
