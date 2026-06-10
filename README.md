# pointcloud_colorizer

ROS 2 package for colorizing LiDAR point clouds with synchronized camera images and building a voxelized colored map.

Current architecture uses a split pipeline:

1. Color node: projects raw LiDAR points into the image and publishes colored cloud.
2. Map node: consumes colored cloud and odometry, transforms into odometry frame, aggregates voxel map, publishes map.

## Executables

- pointcloud_colorizer raw_cloud_colorizer_color
- pointcloud_colorizer raw_cloud_map_aggregator

## Default Launch

Main launch file:
- launch/colorizers.launch.py

What it starts:
- node name raw_cloud_colorizer, executable raw_cloud_colorizer_color
- node name colored_cloud_map_aggregator, executable raw_cloud_map_aggregator
- optional rviz2 controlled by launch arg rviz (default true)

Examples:

Launch with RViz (default):
  ros2 launch pointcloud_colorizer colorizers.launch.py

Launch without RViz:
  ros2 launch pointcloud_colorizer colorizers.launch.py rviz:=false

## Config Files

- config/raw_cloud_colorizer.yaml
  - Parameters for color node (raw cloud + image + camera info -> colored cloud)
- config/colored_cloud_map_aggregator.yaml
  - Parameters for map aggregator (colored cloud + odometry -> naive_map)

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

Check executables:

  ros2 pkg executables pointcloud_colorizer

## Run Nodes Directly

Color node:

  PARAMS_FILE=$(ros2 pkg prefix pointcloud_colorizer)/share/pointcloud_colorizer/config/raw_cloud_colorizer.yaml
  ros2 run pointcloud_colorizer raw_cloud_colorizer_color --ros-args --params-file "$PARAMS_FILE"

Map aggregator node:

  PARAMS_FILE=$(ros2 pkg prefix pointcloud_colorizer)/share/pointcloud_colorizer/config/colored_cloud_map_aggregator.yaml
  ros2 run pointcloud_colorizer raw_cloud_map_aggregator --ros-args --params-file "$PARAMS_FILE"

## RViz

RViz config file is installed with the package:
- rviz/colorizer.rviz

Manual run:

  rviz2 -d $(ros2 pkg prefix pointcloud_colorizer)/share/pointcloud_colorizer/rviz/colorizer.rviz

## Notes

- Legacy monolithic nodes and their configs have been removed from the build and source tree.
- If old processes are still running from previous sessions, restart terminals and relaunch.
