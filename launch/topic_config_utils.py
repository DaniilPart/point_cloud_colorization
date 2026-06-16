import os
from typing import Any, Dict, Tuple

import yaml


def _read_yaml(config_path: str) -> Dict[str, Any]:
    expanded_path = os.path.expanduser(config_path)
    with open(expanded_path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    return data if isinstance(data, dict) else {}


def _node_params(config: Dict[str, Any], node_name: str) -> Dict[str, Any]:
    section = config.get(node_name, {})
    if not isinstance(section, dict):
        return {}
    params = section.get('ros__parameters', {})
    return params if isinstance(params, dict) else {}


def resolve_record_topics(
    aggregator_config_path: str,
    default_colored_cloud_topic: str = '/colorizer/raw/colored_cloud',
    default_odometry_topic: str = '/liorf/mapping/odometry_incremental',
) -> Tuple[str, str]:
    """Resolve recorder topics from aggregator config with safe fallbacks."""
    config = _read_yaml(aggregator_config_path)

    global_params = _node_params(config, '/**')
    aggregator_params = _node_params(config, 'colored_cloud_map_aggregator')

    colored_cloud_topic = aggregator_params.get(
        'input_colored_cloud_topic', default_colored_cloud_topic
    )
    odometry_topic = global_params.get('input_odometry_topic', default_odometry_topic)

    if not isinstance(colored_cloud_topic, str) or not colored_cloud_topic:
        colored_cloud_topic = default_colored_cloud_topic
    if not isinstance(odometry_topic, str) or not odometry_topic:
        odometry_topic = default_odometry_topic

    return colored_cloud_topic, odometry_topic