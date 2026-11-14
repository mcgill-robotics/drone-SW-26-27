from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default = os.path.join(
        get_package_share_directory("drone_video"), "config", "synthetic.json"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default),
            Node(
                package="drone_video",
                executable="sender",
                arguments=["--config", LaunchConfiguration("config")],
                output="screen",
            ),
        ]
    )
