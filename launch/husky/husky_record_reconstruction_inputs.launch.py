import os
import sys
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

LAUNCH_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if LAUNCH_DIR not in sys.path:
    sys.path.append(LAUNCH_DIR)

from topic_config_utils import resolve_record_topics


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context):
    aggregator_config = os.path.expanduser(LaunchConfiguration('aggregator_config').perform(context))
    bag_root_dir = os.path.expanduser(LaunchConfiguration('bag_root_dir').perform(context))
    bag_name = LaunchConfiguration('bag_name').perform(context)
    storage_id = LaunchConfiguration('storage_id').perform(context)
    rviz_enabled = LaunchConfiguration('rviz')
    aggregation_enabled = _as_bool(LaunchConfiguration('enable_aggregation').perform(context))

    os.makedirs(bag_root_dir, exist_ok=True)
    bag_output_dir = os.path.join(bag_root_dir, bag_name)

    colored_cloud_topic, odometry_topic = resolve_record_topics(aggregator_config)

    package_share = get_package_share_directory('pointcloud_colorizer')
    color_params_file = os.path.join(package_share, 'config', 'husky', 'raw_cloud_colorizer.yaml')
    rviz_config_file = os.path.join(package_share, 'rviz', 'colorizer.rviz')
    shared_runtime_params = {'run_id': bag_name}

    raw_colorizer_component = ComposableNode(
        package='pointcloud_colorizer',
        plugin='RawCloudColorizerColorNode',
        name='raw_cloud_colorizer',
        parameters=[color_params_file],
    )

    components = [raw_colorizer_component]
    if aggregation_enabled:
        components.extend(
            [
                ComposableNode(
                    package='pointcloud_colorizer',
                    plugin='ColoredCloudMapAggregatorNode',
                    name='colored_cloud_map_aggregator',
                    parameters=[aggregator_config, shared_runtime_params],
                ),
                ComposableNode(
                    package='pointcloud_colorizer',
                    plugin='OdomEarthPoseLoggerNode',
                    name='odom_earth_pose_logger',
                    parameters=[aggregator_config, shared_runtime_params],
                ),
            ]
        )

    container = ComposableNodeContainer(
        name='colorizer_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=components,
        output='screen',
    )

    recorder = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'record',
            '-s',
            storage_id,
            '-o',
            bag_output_dir,
            colored_cloud_topic,
            odometry_topic,
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz_enabled),
    )

    return [container, recorder, rviz]


def generate_launch_description():
    package_share = get_package_share_directory('pointcloud_colorizer')
    default_aggregator_config = os.path.join(
        package_share, 'config', 'husky', 'colored_cloud_map_aggregator.yaml'
    )
    default_bag_name = f"reconstruction_inputs_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'aggregator_config',
                default_value=default_aggregator_config,
                description='Aggregator YAML used to resolve odometry and colored cloud topics.',
            ),
            DeclareLaunchArgument(
                'bag_root_dir',
                default_value='~/bag',
                description='Root directory where MCAP recordings are created.',
            ),
            DeclareLaunchArgument(
                'bag_name',
                default_value=default_bag_name,
                description='Subfolder name of this recording session.',
            ),
            DeclareLaunchArgument(
                'storage_id',
                default_value='mcap',
                description='rosbag2 storage plugin id.',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description='Launch RViz along with recording.',
            ),
            DeclareLaunchArgument(
                'enable_aggregation',
                default_value='true',
                description='Run map aggregation and odom-earth logging in parallel with recording.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )