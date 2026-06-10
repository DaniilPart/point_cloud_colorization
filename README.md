# pointcloud_colorizer

ROS 2 package for colorizing LiDAR point clouds with synchronized camera images and building a voxelized colored map.

Current architecture uses a split pipeline:

1. Color node: projects raw LiDAR points into the image and publishes colored cloud.
2. Map node: consumes colored cloud and odometry, transforms into odometry frame, aggregates voxel map, publishes map.

## Executables

- rclcpp_components component_container_mt (used by launch as the composite container)
- raw_cloud_colorizer_color (standalone colorizer node)
- raw_cloud_map_aggregator (standalone map aggregator node)
- save_map (CLI utility that calls the aggregator save Trigger service)

## Launch Files

Available launch files:
- launch/colorizers_composed.launch.py
  - Runs both nodes as composable components in one component_container_mt process.
- launch/colorizer_only.launch.py
  - Runs only the standalone colorization node executable.
- launch/aggregator_only.launch.py
  - Runs only the standalone map aggregator node executable.

Examples:

Run composed pipeline with RViz (default):
  ros2 launch pointcloud_colorizer colorizers_composed.launch.py

Run only colorization:
  ros2 launch pointcloud_colorizer colorizer_only.launch.py

Run only map aggregator:
  ros2 launch pointcloud_colorizer aggregator_only.launch.py

Disable RViz on any launch file:
  ros2 launch pointcloud_colorizer <launch_file>.launch.py rviz:=false

## Config Files

- config/raw_cloud_colorizer.yaml
  - Parameters for color node (raw cloud + image + camera info -> colored cloud)
- config/colored_cloud_map_aggregator.yaml
  - Parameters for map aggregator (colored cloud + odometry -> naive_map)
  - Includes periodic PLY save controls:
    - `map_save_interval_sec` (default `5.0`)
      - Set to `0.0` to disable periodic saving.
    - `map_save_ply_path` (output PLY file path)
    - `map_save_append_start_timestamp` (default `true`, appends experiment start time to filename)
    - `map_save_service_name` (default `~/save_map`, resolved under node namespace/name)

Manual map save:
- Set `map_save_interval_sec: 0.0` when you want periodic saving disabled and service-only saving.
- Run `ros2 run pointcloud_colorizer save_map` to trigger save via service.
- Service auto-discovery prefers node-scoped save services and may resolve to `/colored_cloud_map_aggregator/save_map` or namespaced equivalents.
- Override service and timeout when needed:
  `ros2 run pointcloud_colorizer save_map --service /my_ns/colored_cloud_map_aggregator/save_map --timeout 10`

Named/Path-based map save:
- `ros2 run pointcloud_colorizer save_map run1`
  - Saves as `run1_YYYYMMDD_HHMMSS.ply` in the configured output directory.
- `ros2 run pointcloud_colorizer save_map /tmp/my_map.ply`
  - Saves exactly to `/tmp/my_map.ply` (no timestamp appended).
- `ros2 run pointcloud_colorizer save_map /tmp/maps/`
  - Treats argument as output folder and saves `<configured_base_name>_YYYYMMDD_HHMMSS.ply` inside that folder.

## Main Topics

Inputs:
- /liorf/deskew/cloud_deskewed
- /basler_front/image_color/compressed
- /basler_front/camera_info
- /liorf/mapping/odometry

Outputs:
- /colorizer/raw/colored_cloud
- /colorizer/raw/naive_map

## Build

From workspace root:

  colcon build --packages-select pointcloud_colorizer --symlink-install
  source install/setup.bash

Check package presence:

  ros2 pkg list | grep pointcloud_colorizer

## Components

The two runtime nodes are loaded as components into one process container:

- RawCloudColorizerColorNode
- ColoredCloudMapAggregatorNode

Launch file colorizers.launch.py loads both components into a single component_container_mt.

## RViz

RViz config file is installed with the package:
- rviz/colorizer.rviz

Manual run:

  rviz2 -d $(ros2 pkg prefix pointcloud_colorizer)/share/pointcloud_colorizer/rviz/colorizer.rviz

## Notes

- Legacy monolithic nodes and their configs have been removed from the build and source tree.
- If old processes are still running from previous sessions, restart terminals and relaunch.
