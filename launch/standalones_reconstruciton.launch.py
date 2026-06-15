import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    shared_runtime_params = {'run_id': run_id}

    package_share = get_package_share_directory('pointcloud_colorizer')
    color_params_file = os.path.join(package_share, 'config', 'raw_cloud_colorizer.yaml')
    map_params_file = os.path.join(package_share, 'config', 'colored_cloud_map_aggregator.yaml')
    rviz_config_file = os.path.join(package_share, 'rviz', 'reconstruction.rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz along with colorizer nodes',
    )

    raw_colorizer_standalone = Node(
        package='pointcloud_colorizer',
        executable='raw_cloud_colorizer_color',
        name='raw_cloud_colorizer',
        output='screen',
        parameters=[color_params_file],
    )

    raw_map_aggregator_standalone = Node(
        package='pointcloud_colorizer',
        executable='raw_cloud_map_aggregator',
        name='colored_cloud_map_aggregator',
        output='screen',
        parameters=[map_params_file, shared_runtime_params],
    )

    odom_earth_pose_logger_standalone = Node(
        package='pointcloud_colorizer',
        executable='odom_earth_pose_logger',
        name='odom_earth_pose_logger',
        output='screen',
        parameters=[map_params_file, shared_runtime_params],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        raw_colorizer_standalone,
        raw_map_aggregator_standalone,
        odom_earth_pose_logger_standalone,
        rviz,
    ])
