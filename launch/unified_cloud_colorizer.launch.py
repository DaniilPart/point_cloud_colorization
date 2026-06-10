import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('pointcloud_colorizer')
    params_file = os.path.join(package_share, 'config', 'unified_cloud_colorizer.yaml')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='registered',
        description='Unified node mode: raw or registered',
    )

    unified_colorizer = Node(
        package='pointcloud_colorizer',
        executable='unified_cloud_colorizer',
        name='unified_cloud_colorizer',
        output='screen',
        parameters=[params_file, {'mode': LaunchConfiguration('mode')}],
    )

    return LaunchDescription([mode_arg, unified_colorizer])