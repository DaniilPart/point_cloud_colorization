import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('pointcloud_colorizer')
    params_file = os.path.join(package_share, 'config', 'registered_cloud_colorizer.yaml')

    registered_colorizer = Node(
        package='pointcloud_colorizer',
        executable='registered_cloud_colorizer',
        name='registered_cloud_colorizer',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([registered_colorizer])