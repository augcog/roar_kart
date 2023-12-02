from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import LogInfo
from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = launch.LaunchDescription()

    """ ros bridge """

    ld.add_action(LogInfo(msg=["ROSBridge launched"]))
    rosbridge_launch_path: Path = Path(get_package_share_directory("rosbridge_server")) / "launch" / "rosbridge_websocket_launch.xml"
    assert rosbridge_launch_path.exists(), f"{rosbridge_launch_path} does not exist"
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch_path.as_posix()), 
        launch_arguments={"port": "9090","address":"0.0.0.0"}.items())
    ld.add_action(rosbridge_launch)

    return ld 