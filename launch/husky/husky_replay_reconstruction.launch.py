import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events.process import ShutdownProcess, matches_name
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

LAUNCH_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if LAUNCH_DIR not in sys.path:
    sys.path.append(LAUNCH_DIR)

from topic_config_utils import resolve_record_topics


def _find_latest_recording(bag_root_dir: str, bag_prefix: str) -> str:
    root = os.path.expanduser(bag_root_dir)
    if not os.path.isdir(root):
        raise RuntimeError(f'Bag root directory does not exist: {root}')

    candidates = []
    for entry in os.scandir(root):
        if not entry.is_dir():
            continue
        if bag_prefix and not entry.name.startswith(bag_prefix):
            continue
        metadata_path = os.path.join(entry.path, 'metadata.yaml')
        if os.path.isfile(metadata_path):
            candidates.append(entry.path)

    if not candidates:
        raise RuntimeError(
            f'No bag directories found in {root} with prefix "{bag_prefix}" and metadata.yaml.'
        )

    return max(candidates, key=os.path.getmtime)


def _launch_setup(context):
    package_share = get_package_share_directory('pointcloud_colorizer')

    aggregator_config = os.path.expanduser(LaunchConfiguration('aggregator_config').perform(context))
    bag_root_dir = LaunchConfiguration('bag_root_dir').perform(context)
    bag_prefix = LaunchConfiguration('bag_prefix').perform(context)
    bag_path_arg = LaunchConfiguration('bag_path').perform(context).strip()
    play_rate = LaunchConfiguration('play_rate').perform(context)
    map_voxel_size = float(LaunchConfiguration('map_voxel_size').perform(context))
    rviz_enabled = LaunchConfiguration('rviz')

    selected_bag_path = (
        os.path.expanduser(bag_path_arg)
        if bag_path_arg
        else _find_latest_recording(bag_root_dir=bag_root_dir, bag_prefix=bag_prefix)
    )

    colored_cloud_topic, odometry_topic = resolve_record_topics(aggregator_config)
    run_id = os.path.basename(os.path.normpath(selected_bag_path))
    rviz_config_file = os.path.join(package_share, 'rviz', 'reconstruction.rviz')

    map_aggregator_component = ComposableNode(
        package='pointcloud_colorizer',
        plugin='ColoredCloudMapAggregatorNode',
        name='colored_cloud_map_aggregator',
        parameters=[
            aggregator_config,
            {
                'run_id': run_id,
                'use_sim_time': True,
                'map_voxel_size': map_voxel_size,
            },
        ],
    )

    container = ComposableNodeContainer(
        name='colorizer_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[map_aggregator_component],
        output='screen',
    )

    bag_play = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'play',
            selected_bag_path,
            '--clock',
            '--rate',
            play_rate,
            '--topics',
            colored_cloud_topic,
            odometry_topic,
        ],
        output='screen',
    )

    stop_reconstruction_when_replay_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[
                LogInfo(msg='[spot_replay_reconstruction] Bag replay exited, stopping reconstruction container.'),
                EmitEvent(
                    event=ShutdownProcess(process_matcher=matches_name('colorizer_container'))
                ),
            ],
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz_enabled),
    )

    return [
        LogInfo(msg=f'[spot_replay_reconstruction] Using bag: {selected_bag_path}'),
        LogInfo(msg=f'[spot_replay_reconstruction] Replay topics: {colored_cloud_topic}, {odometry_topic}'),
        container,
        bag_play,
        stop_reconstruction_when_replay_exits,
        rviz,
    ]


def generate_launch_description():
    package_share = get_package_share_directory('pointcloud_colorizer')
    default_aggregator_config = os.path.join(
        package_share, 'config', 'husky', 'colored_cloud_map_aggregator.yaml'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'aggregator_config',
                default_value=default_aggregator_config,
                description='Aggregator YAML used to resolve replay topics and base reconstruction params.',
            ),
            DeclareLaunchArgument(
                'bag_root_dir',
                default_value='~/bag',
                description='Root directory where recorded bags are stored.',
            ),
            DeclareLaunchArgument(
                'bag_prefix',
                default_value='reconstruction_inputs_',
                description='Prefix used when auto-selecting latest bag directory.',
            ),
            DeclareLaunchArgument(
                'bag_path',
                default_value='',
                description='Specific bag directory to replay. If empty, latest matching bag is selected.',
            ),
            DeclareLaunchArgument(
                'play_rate',
                default_value='3.0',
                description='Playback speed multiplier.',
            ),
            DeclareLaunchArgument(
                'map_voxel_size',
                default_value='0.05',
                description='Map voxel resolution used during replay reconstruction.',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description='Launch RViz along with replay reconstruction.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )