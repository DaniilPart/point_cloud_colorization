import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('pointcloud_colorizer')
    color_params_file = os.path.join(package_share, 'config', 'spot', 'raw_cloud_colorizer.yaml')
    rviz_config_file = os.path.join(package_share, 'rviz', 'colorizer.rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz along with the colorization node',
    )

    raw_colorizer = Node(
        package='pointcloud_colorizer',
        executable='raw_cloud_colorizer_color',
        name='raw_cloud_colorizer',
        output='screen',
        parameters=[color_params_file],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([rviz_arg, raw_colorizer, rviz])
