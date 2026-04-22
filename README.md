# pointcloud_colorizer

ROS 2 package for colorizing LiDAR point clouds with camera images.

It supports three executables:

| Executable | Purpose |
| --- | --- |
| `raw_cloud_colorizer` | Colors the current raw LiDAR scan from `/os1/points` |
| `registered_cloud_colorizer` | Colors the registered SLAM cloud and builds a naive accumulated map |
| `unified_cloud_colorizer` | Single configurable node with `mode:=raw` or `mode:=registered` |

## Demo

### Raw LiDAR Scan Colorization

This demo shows the current LiDAR scan being colored directly from the camera image.

![Raw colored point cloud demo](data/demo.gif)

### Registered Map Accumulation

This demo shows the registered pipeline building a colored map over time while the rosbag is playing.

![Registered map accumulation demo](data/prew_fast.gif)

## Published Topics

| Topic | Meaning |
| --- | --- |
| `/colorizer/raw/colored_cloud` | Current colored raw LiDAR scan |
| `/colorizer/registered/colored_cloud` | Current colored registered scan in the global frame |
| `/colorizer/registered/naive_map` | Accumulated downsampled colored map |

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

Expected workspace layout:

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

Expected result:

```text
pointcloud_colorizer raw_cloud_colorizer
pointcloud_colorizer registered_cloud_colorizer
pointcloud_colorizer unified_cloud_colorizer
```

## Parameters And Launch

Parameter file:

- `config/colorizers.yaml`

Launch file:

- `launch/colorizers.launch.py`

Start both dedicated nodes:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch pointcloud_colorizer colorizers.launch.py
```

Check that parameters were loaded:

```bash
ros2 node list | grep colorizer
ros2 param list /raw_colorizer
ros2 param list /registered_colorizer
ros2 param get /registered_colorizer map_voxel_size
```

Note:

- `colorizers.launch.py` is useful for startup and parameter verification
- for clean functional testing, it is usually easier to run one node at a time

## Before Running Tests

If old nodes are still running, stop them first:

```bash
pkill -f raw_cloud_colorizer
pkill -f registered_cloud_colorizer
pkill -f unified_cloud_colorizer
pkill -f "ros2 bag play"
```

This avoids duplicate publishers and confusing ROS graphs.

## Test 1: `raw_cloud_colorizer`

This test needs a raw bag with:

- `/os1/points`
- `/camera_front/image_raw/compressed`
- `/camera_front/camera_info`
- `/tf`
- `/tf_static`

Terminal 1:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run pointcloud_colorizer raw_cloud_colorizer
```

Terminal 2:

```bash
source ~/ros2_ws/install/setup.bash
ros2 bag play <raw_bag>.mcap --clock --topics /os1/points /camera_front/image_raw/compressed /camera_front/camera_info /tf /tf_static
```

Terminal 3:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic info /colorizer/raw/colored_cloud
ros2 topic hz /colorizer/raw/colored_cloud
ros2 topic echo /colorizer/raw/colored_cloud --once --field header.frame_id
```

What you should see:

- `/colorizer/raw/colored_cloud` exists
- messages are being published
- `header.frame_id` is usually `os1/os_lidar`

RViz2:

- `Fixed Frame`: `os1/os_lidar`
- Add `PointCloud2`
- Topic: `/colorizer/raw/colored_cloud`
- `Color Transformer`: `RGB8`

Expected result:

- colored current LiDAR scan
- no accumulated map
- no gray placeholder points when `publish_only_colored_points=true`

## Test 2: `registered_cloud_colorizer`

This test needs a registered bag with:

- `/liorf/mapping/odometry`
- `/liorf/mapping/cloud_registered`
- `/camera_front/image_raw/compressed`
- `/camera_front/camera_info`
- `/tf`
- `/tf_static`

Terminal 1:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run pointcloud_colorizer registered_cloud_colorizer
```

Terminal 2:

```bash
source ~/ros2_ws/install/setup.bash
ros2 bag play <registered_bag_folder> --clock
```

If camera topics begin later in the bag, use:

```bash
source ~/ros2_ws/install/setup.bash
ros2 bag play <registered_bag_folder> --clock --start-offset 66
```

Terminal 3:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic info /colorizer/registered/colored_cloud
ros2 topic info /colorizer/registered/naive_map
ros2 topic hz /colorizer/registered/colored_cloud
ros2 topic hz /colorizer/registered/naive_map
ros2 topic echo /colorizer/registered/colored_cloud --once --field header.frame_id
ros2 topic echo /colorizer/registered/naive_map --once --field header.frame_id
```

What you should see:

- node logs `dt = 0 ns` or a very small value
- `/colorizer/registered/colored_cloud` exists
- `/colorizer/registered/naive_map` exists
- `header.frame_id` is usually `odom`

RViz2:

- `Fixed Frame`: `odom`
- Add `PointCloud2` for `/colorizer/registered/colored_cloud`
- Add `PointCloud2` for `/colorizer/registered/naive_map`
- `Color Transformer`: `RGB8`

Expected result:

- current colored registered scan
- accumulated colored map that grows over time

## Test 3: `unified_cloud_colorizer` In Raw Mode

Terminal 1:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run pointcloud_colorizer unified_cloud_colorizer --ros-args -p mode:=raw
```

Terminal 2:

```bash
source ~/ros2_ws/install/setup.bash
ros2 bag play <raw_bag>.mcap --clock --topics /os1/points /camera_front/image_raw/compressed /camera_front/camera_info /tf /tf_static
```

Terminal 3:

```bash
source ~/ros2_ws/install/setup.bash
ros2 param get /unified_cloud_colorizer mode
ros2 topic info /colorizer/raw/colored_cloud
ros2 topic hz /colorizer/raw/colored_cloud
```

Expected result:

- parameter `mode` is `raw`
- `/colorizer/raw/colored_cloud` is published
- behavior matches `raw_cloud_colorizer`

## Test 4: `unified_cloud_colorizer` In Registered Mode

Terminal 1:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run pointcloud_colorizer unified_cloud_colorizer --ros-args -p mode:=registered
```

Terminal 2:

```bash
source ~/ros2_ws/install/setup.bash
ros2 bag play <registered_bag_folder> --clock --start-offset 66
```

Terminal 3:

```bash
source ~/ros2_ws/install/setup.bash
ros2 param get /unified_cloud_colorizer mode
ros2 topic info /colorizer/registered/colored_cloud
ros2 topic info /colorizer/registered/naive_map
ros2 topic hz /colorizer/registered/colored_cloud
ros2 topic hz /colorizer/registered/naive_map
```

Expected result:

- parameter `mode` is `registered`
- logs contain `Registered mode dt = ...`
- both registered topics are published
- behavior matches `registered_cloud_colorizer`

## Configuration

Default parameters are stored in `config/colorizers.yaml`.

Raw parameters:

- `input_cloud_topic`
- `input_image_topic`
- `camera_info_topic`
- `output_cloud_topic`
- `publish_only_colored_points`

Registered parameters:

- `input_registered_cloud_topic`
- `input_odometry_topic`
- `input_image_topic`
- `camera_info_topic`
- `output_cloud_topic`
- `output_map_topic`
- `map_voxel_size`
- `publish_only_colored_points`

Unified parameters:

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

### RViz shows the topic, but nothing is visible

Check:

- correct `Fixed Frame`
- correct topic selected
- `Color Transformer` set to `RGB8`
- old nodes are not still running

### You see `/colorizer/raw/colored_cloud` while testing registered mode

Usually this means:

- another raw node is still running
- or RViz is still subscribed to the raw topic

Check:

```bash
ros2 node list | grep colorizer
ros2 topic info /colorizer/raw/colored_cloud --verbose
```

### Registered bag starts, but no output appears

Possible reasons:

- `camera_info` has not arrived yet
- you need `--start-offset 66`
- wrong `Fixed Frame`
- missing `/tf` or `/tf_static`

### TF_OLD_DATA warnings

This usually happens when:

- you replayed the same bag multiple times without stopping old nodes
- TF buffer still contains newer transforms

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
