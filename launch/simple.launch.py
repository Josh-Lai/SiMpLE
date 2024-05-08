import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("simple"), "config", "live.yaml")

    start_simple = Node(
        package="simple", executable="simple", name="simple", parameters=[config]
    )

    ld = LaunchDescription()
    ld.add_action(start_simple)
    return ld
