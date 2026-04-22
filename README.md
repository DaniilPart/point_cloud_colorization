# LiDAR Point Cloud Colorization in ROS 2

A ROS 2 C++ node developed to colorize a 3D LiDAR point cloud using an aligned 2D camera image. The package synchronizes sensor data streams and performs 3D-to-2D geometric projections to output a fully colorized `XYZRGB` point cloud.

## Demonstration

<p align="center">
  <img src="https://github.com/DaniilPart/point_cloud_colorization/blob/main/data/demo.gif?raw=true" width="700" alt="Point Cloud Colorization Demo" />
</p>

## Overview

The algorithm dynamically extracts spatial transformations (extrinsics) via the TF2 framework and intrinsic camera parameters from the `/camera_info` topic. To ensure accurate color mapping, particularly at the periphery of the field of view, the system applies OpenCV-based distortion correction (`cv::undistort`) to the incoming image before projecting the 3D points.

Key capabilities include:
- Approximate time synchronization of `sensor_msgs/PointCloud2` and `sensor_msgs/CompressedImage`.
- Real-time TF2 extrinsic extraction between the LiDAR and camera reference frames.
- Dynamic intrinsic calibration integration.

## Dependencies

- **Framework:** ROS 2 (tested on Jazzy)
- **Libraries:** PCL, OpenCV, cv_bridge, Eigen3
- **ROS 2 Packages:** `tf2_ros`, `tf2_eigen`, `message_filters`, `sensor_msgs`

## Build and Execution

Compile the package using `colcon`:

```bash
cd ~/ros2_ws
colcon build --packages-select pointcloud_colorizer --symlink-install
source install/setup.bash
```

Run the node alongside your sensor data (or `.mcap` bag file):

```bash
ros2 run pointcloud_colorizer minimal_subscriber
```

The output can be visualized in RViz2 by subscribing to the `/colorizer/debug/colored_cloud` topic and setting the Color Transformer to `RGB8`.