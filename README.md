# pointcloud_colorizer

ROS 2 package for colorizing LiDAR point clouds with synchronized camera images and building a voxelized colored map.

## Quick Start (Recommended)


Before the map starts appearing, the robot has to move for one meter, or turn a bit - so that liorf produces another keyframe. 
Alternatively, launch liorf the colorizer

Use `reconstruction.launch.py` as the main entrypoint.

Build and source:
```bash
  ws build pointcloud_colorizer
  source ~/.bashrc
```
Run full reconstruction pipeline (colorizer + aggregator + [RViz]):
```bash
  ros2 launch pointcloud_colorizer reconstruction.launch.py [rviz:=false]
```

For the default input topics, ensure liorf is up and running:
```bash
  ros2 node list | grep liorf
```
Even though you can take the pcl, image and odometry topics from any source, liorf is the one tested.

## Day-To-Day Usage

Map saving (manual service call):

- `ros2 run pointcloud_colorizer save_map`

Named/path-based map save:

- `ros2 run pointcloud_colorizer save_map run1`
  - Saves as `run1_YYYYMMDD_HHMMSS.ply` in configured output directory.
- `ros2 run pointcloud_colorizer save_map /tmp/my_map.ply`
  - Saves exactly to `/tmp/my_map.ply` (no timestamp appended).
- `ros2 run pointcloud_colorizer save_map /tmp/maps/`
  - Treats argument as output folder and saves `<configured_base_name>_YYYYMMDD_HHMMSS.ply` in that folder.

## Configuration

### Raw colorizer config: `config/raw_cloud_colorizer.yaml`

#### Input pointcloud topic (sensor frame)
- `input_cloud_topic`: pointcloud topic used
  - `/liorf/mapping/keyframes/cloud_deskewed_downsampled` (default) is good for performance.
  - `/liorf/deskew/cloud_deskewed` is the full deskewed cloud topic, but may be too dense and too frequent.
    - if using this topic, increase `min_processing_interval_sec` to prevent overload. 
  - `min_processing_interval_sec` throttles processing frequency (default 0.0).

#### Input image topic
- The node automatically selects if to use compressed or raw image topics based on availability, prioritizing raw if both are present.
  - `input_image_topic_raw` is the primary image topic parameter.
  - If `input_image_topic_compressed` is empty, it is auto-derived as `input_image_topic_raw + "/compressed"`.

#### LiDAR-camera transform
- The transformation is either looked up from TF or taken from the config parameters, based on `transform_source`.
- If using TF, ensure the transform is being published by your system (e.g. static transform publisher or robot_state_publisher).
- If using config, set the `camera_to_lidar_matrix` parameter to the 4x4 homogeneous transformation matrix from camera frame to LiDAR frame.

Getting the transformation:
```
ros2 run tf2_ros tf2_echo --frame1 pylon_camera --frame2 os_lidar
```

### Aggregator config: `config/colored_cloud_map_aggregator.yaml`

- `pose_source` selects pose provider:
  - `odometry` (default): synchronized odometry + colored cloud.
  - `csv_keyframes`: uses `keyframes_csv_path` and subscribes only to colored cloud.
- `keyframes_csv_path` points to CSV with columns:
  - `timestamp_sec,x_local,y_local,z_local,roll,pitch,yaw`
- `csv_pose_match_tolerance_sec` controls nearest timestamp matching tolerance.
  - No interpolation is used in this mode.
- `csv_pose_frame_id` is used as fallback parent frame for TF and map headers when cloud frame is empty.

- `map_save_interval_sec` controls periodic save interval (set `0.0` to disable periodic saving).
  - Do this if suspect the performance issues, although it should not
- 

## Map saving

- Map saving is triggered by a ROS service call. Use `ros2 run pointcloud_colorizer save_map` to trigger a save on demand.
- The map is also saved every 5 seconds by default, but you can disable this by setting `map_save_interval_sec` to `0.0` in the aggregator config.

## Map processing and visualization
This takes under 1 minute, but requires dependency installation. Follow the MAP_VISUALIZATION.md.


## Running from the bagfile
Record Spot reconstruction inputs (MCAP):
```bash
ros2 launch pointcloud_colorizer spot/spot_record_reconstruction_inputs.launch.py
```

This launch runs the full pipeline by default:
- raw colorizer
- colored cloud map aggregator
- odom-earth pose logger
- rosbag2 recorder

Disable aggregation if you only want recording + colorizer:
```bash
ros2 launch pointcloud_colorizer spot/spot_record_reconstruction_inputs.launch.py enable_aggregation:=false
```

By default this records:
- `/colorizer/raw/colored_cloud`
- `/liorf/mapping/odometry_incremental`

Topics are resolved from `config/spot/colored_cloud_map_aggregator.yaml`, and output is written to `~/bag/reconstruction_inputs_YYYYMMDD_HHMMSS`.

Example bagfile playing:
```bash
ros2 bag play recording_20260423_153545 --topics \
 /basler_front/camera_info /basler_front/image_color/compressed /ouster/imu /ouster/points /tf_static \
 --clock
```

