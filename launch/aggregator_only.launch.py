import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('pointcloud_colorizer')
    map_params_file = os.path.join(package_share, 'config', 'colored_cloud_map_aggregator.yaml')
    rviz_config_file = os.path.join(package_share, 'rviz', 'reconstruction.rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz along with the map aggregator node',
    )

    map_aggregator = Node(
        package='pointcloud_colorizer',
        executable='raw_cloud_map_aggregator',
        name='colored_cloud_map_aggregator',
        output='screen',
        parameters=[map_params_file],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([rviz_arg, map_aggregator, rviz])
