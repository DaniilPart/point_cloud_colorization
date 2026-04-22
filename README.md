# pointcloud_colorizer

ROS 2 package for colorizing LiDAR point clouds with synchronized camera images.

The repository contains:

- a raw pipeline for coloring the current LiDAR scan
- a registered pipeline for coloring the SLAM-registered cloud
- a unified node that can run in either mode

## Demo

### Raw Point Cloud Colorization

The current LiDAR scan is projected into the camera image and receives RGB values from the image.

![Raw colored point cloud demo](data/demo.gif)

### Registered Map Accumulation

The registered pipeline colors incoming registered clouds and accumulates a downsampled map over time.

![Registered map accumulation demo](data/prew_fast.gif)

## Executables

| Executable | Description |
| --- | --- |
| `raw_cloud_colorizer` | Colors the raw LiDAR cloud from `/os1/points` |
| `registered_cloud_colorizer` | Colors the registered cloud from `/liorf/mapping/cloud_registered` and publishes a naive accumulated map |
| `unified_cloud_colorizer` | Single configurable node with `mode:=raw` or `mode:=registered` |

## Output Topics

| Topic | Meaning |
| --- | --- |
| `/colorizer/raw/colored_cloud` | Current colored raw LiDAR scan |
| `/colorizer/registered/colored_cloud` | Current colored registered scan in the global frame |
| `/colorizer/registered/naive_map` | Accumulated downsampled registered map |

## Requirements

Tested on:

- ROS 2 Jazzy
- Ubuntu Linux

Main dependencies:

- `rclcpp`
- `sensor_msgs`
- `std_msgs`
- `nav_msgs`
- `message_filters`
- `tf2`
- `tf2_ros`
- `cv_bridge`
- `OpenCV`
- `PCL`

## Workspace Layout

Expected workspace structure:

```text
~/ros2_ws/
├── src/
│   └── pointcloud_colorizer/
└── compile.sh
```

Recommended `compile.sh`:

```bash
#!/usr/bin/env bash
set -e
cd ~/ros2_ws
colcon build --packages-select pointcloud_colorizer --symlink-install
```

Make it executable once:

```bash
chmod +x ~/ros2_ws/compile.sh
```

## Build

```bash
cd ~/ros2_ws
./compile.sh
source ~/ros2_ws/install/setup.bash
```

Verify the package:

```bash
ros2 pkg list | grep pointcloud_colorizer
ros2 pkg executables pointcloud_colorizer
```

Expected executables:

```text
pointcloud_colorizer raw_cloud_colorizer
pointcloud_colorizer registered_cloud_colorizer
pointcloud_colorizer unified_cloud_colorizer
```

## Configuration

Default parameters are stored in:

- `config/colorizers.yaml`

Launch file:

- `launch/colorizers.launch.py`

Parameter groups:

### Raw node

- `input_cloud_topic`
- `input_image_topic`
- `camera_info_topic`
- `output_cloud_topic`
- `publish_only_colored_points`

### Registered node

- `input_registered_cloud_topic`
- `input_odometry_topic`
- `input_image_topic`
- `camera_info_topic`
- `output_cloud_topic`
- `output_map_topic`
- `map_voxel_size`
- `publish_only_colored_points`

### Unified node

- `mode`
- `input_cloud_topic`
- `input_registered_cloud_topic`
- `input_odometry_topic`
- `input_image_topic`
- `camera_info_topic`
- `output_cloud_topic`
- `output_map_topic`
- `map_voxel_size`
- `publish_only_colored_points`

## Run The Dedicated Nodes

### Start both dedicated nodes from the launch file

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch pointcloud_colorizer colorizers.launch.py
```

You can verify the loaded parameters with:

```bash
ros2 node list | grep colorizer
ros2 param list /raw_colorizer
ros2 param list /registered_colorizer
ros2 param get /registered_colorizer map_voxel_size
```

### Run the raw node directly

```bash
source ~/ros2_ws/install/setup.bash
ros2 run pointcloud_colorizer raw_cloud_colorizer
```

### Run the registered node directly

```bash
source ~/ros2_ws/install/setup.bash
ros2 run pointcloud_colorizer registered_cloud_colorizer
```

## Run The Unified Node

### Raw mode

```bash
source ~/ros2_ws/install/setup.bash
ros2 run pointcloud_colorizer unified_cloud_colorizer --ros-args -p mode:=raw
```

### Registered mode

```bash
source ~/ros2_ws/install/setup.bash
ros2 run pointcloud_colorizer unified_cloud_colorizer --ros-args -p mode:=registered
```

The unified node fails fast if `mode` is not `raw` or `registered`.

## Visualizing In RViz2

Start RViz2:

```bash
source ~/ros2_ws/install/setup.bash
rviz2
```

Recommended displays:

### Raw pipeline

- `Fixed Frame`: `os1/os_lidar`
- add `PointCloud2`
- topic: `/colorizer/raw/colored_cloud`
- `Color Transformer`: `RGB8`

### Registered pipeline

- `Fixed Frame`: `odom`
- add `PointCloud2`
- topic: `/colorizer/registered/colored_cloud`
- add another `PointCloud2`
- topic: `/colorizer/registered/naive_map`
- `Color Transformer`: `RGB8`

## Useful Commands

```bash
source ~/ros2_ws/install/setup.bash
ros2 node list | grep colorizer
ros2 topic list | grep /colorizer/
ros2 topic info /colorizer/raw/colored_cloud
ros2 topic info /colorizer/registered/colored_cloud
ros2 topic info /colorizer/registered/naive_map
```

If you use Zenoh middleware:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

If it says `Address already in use`, the router is usually already running.

## Troubleshooting

### RViz shows the topic but nothing is visible

Check:

- the correct `Fixed Frame`
- the correct topic
- `Color Transformer` set to `RGB8`
- old nodes are not still running

### A raw topic appears while testing a registered setup

Usually this means:

- another raw node is still running
- or RViz is still subscribed to the raw topic

Useful commands:

```bash
ros2 node list | grep colorizer
ros2 topic info /colorizer/raw/colored_cloud --verbose
```

### TF_OLD_DATA warnings

This usually happens when the same bag was replayed multiple times without stopping old nodes.

Clean restart:

```bash
pkill -f raw_cloud_colorizer
pkill -f registered_cloud_colorizer
pkill -f unified_cloud_colorizer
pkill -f "ros2 bag play"
```

## Repository Files

Main source files:

- `src/raw_cloud_colorizer_node.cpp`
- `src/registered_cloud_colorizer_node.cpp`
- `src/unified_cloud_colorizer_node.cpp`

Support files:

- `config/colorizers.yaml`
- `launch/colorizers.launch.py`
- `data/demo.gif`
- `data/prew_fast.gif`
