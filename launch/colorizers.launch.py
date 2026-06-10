import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_share = get_package_share_directory('pointcloud_colorizer')
    color_params_file = os.path.join(package_share, 'config', 'raw_cloud_colorizer.yaml')
    map_params_file = os.path.join(package_share, 'config', 'colored_cloud_map_aggregator.yaml')
    rviz_config_file = os.path.join(package_share, 'rviz', 'colorizer.rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz along with colorizer nodes',
    )

    raw_colorizer_component = ComposableNode(
        package='pointcloud_colorizer',
        plugin='RawCloudColorizerColorNode',
        name='raw_cloud_colorizer',
        parameters=[color_params_file],
    )

    raw_map_aggregator_component = ComposableNode(
        package='pointcloud_colorizer',
        plugin='ColoredCloudMapAggregatorNode',
        name='colored_cloud_map_aggregator',
        parameters=[map_params_file],
    )

    container = ComposableNodeContainer(
        name='colorizer_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[raw_colorizer_component, raw_map_aggregator_component],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([rviz_arg, container, rviz])
