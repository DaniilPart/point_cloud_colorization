from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("pointcloud_colorizer")
    params_file = os.path.join(package_share, "config", "colorizers.yaml")

    raw_colorizer = Node(
        package="pointcloud_colorizer",
        executable="raw_cloud_colorizer",
        name="raw_colorizer",
        output="screen",
        parameters=[params_file],
    )

    registered_colorizer = Node(
        package="pointcloud_colorizer",
        executable="registered_cloud_colorizer",
        name="registered_colorizer",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        raw_colorizer,
        registered_colorizer,
    ])
