# Assignment: Point Cloud Colorization in ROS 2

## Table of Contents
- [Assignment: Point Cloud Colorization in ROS 2](#assignment-point-cloud-colorization-in-ros-2)
  - [Table of Contents](#table-of-contents)
  - [Objective](#objective)
  - [Prerequisites](#prerequisites)
  - [Development Workflow \& Build Instructions](#development-workflow--build-instructions)
  - [ROS 2 CLI Cheatsheet \& Troubleshooting](#ros-2-cli-cheatsheet--troubleshooting)
    - [Essential Commands](#essential-commands)
    - [Advanced CLI Tricks](#advanced-cli-tricks)
    - [Common Blockers \& Developer Tips](#common-blockers--developer-tips)
  - [Visualizing Data with RViz2](#visualizing-data-with-rviz2)
  - [Step-by-Step Assignment Instructions](#step-by-step-assignment-instructions)
    - [Step 1: Package Initialization](#step-1-package-initialization)
    - [Step 2: Point Cloud Ingestion](#step-2-point-cloud-ingestion)
    - [Step 3: Basic Colorization and Visualization (Purple Cloud)](#step-3-basic-colorization-and-visualization-purple-cloud)
    - [Step 4: Image Ingestion](#step-4-image-ingestion)
    - [Step 5: Image Decompression and Republishing](#step-5-image-decompression-and-republishing)
    - [Step 6: Time Synchronization](#step-6-time-synchronization)
    - [Step 7: Initial Accurate Colorization (Hardcoded Pipeline)](#step-7-initial-accurate-colorization-hardcoded-pipeline)
    - [Step 8: Dynamic Extrinsic Extraction (TF2)](#step-8-dynamic-extrinsic-extraction-tf2)
    - [Step 9: Dynamic Intrinsic Calibration Integration](#step-9-dynamic-intrinsic-calibration-integration)
    - [Step 10: Final Accurate Colorization](#step-10-final-accurate-colorization)


## Objective
Implement a ROS 2 C++ node that colorizes a 3D LiDAR point cloud using an aligned 2D camera image. You will build this step-by-step, starting with basic message passing, moving to hardcoded math to prove your pipeline, and finally implementing dynamic parameter extraction for a robust solution.


## Prerequisites
* **Environment:** ROS 2 Jazzy with Zenoh middleware installed.
* **Data:** A provided ROS 2 `.mcap` bagfile containing:
    * Compressed Image: `/camera_front/image_raw/compressed` (Frame: `pylon_camera`)
    * Camera Info: `/camera_front/camera_info`
    * Point Cloud: `/os1/points` (Frame: `os1/os_lidar`)
    * Transformations via `/tf` and `/tf_static`

---

## Development Workflow & Build Instructions
To keep your environment organized and ensure smooth development, please adhere to the following workflow throughout the assignment:

1. **Workspace Setup:** Your actual package code must reside in your repositories folder (`~/repos`). Link it into your ROS 2 workspace's source directory rather than creating it there directly:
```bash
cd ~/ros2_ws/src
ln -s ~/repos/pointcloud_colorizer .

```

2. **Building the Package:** Always execute your build commands from the root of your workspace (`~/ros2_ws`). Use the `--symlink-install` flag so that any configuration files (like launch files or params) update dynamically without needing a full rebuild:

```bash
cd ~/ros2_ws
colcon build --packages-select pointcloud_colorizer --symlink-install

```

3. **Progress Tracking:** Version control and communication are mandatory. Upon completing and verifying **each** step below:

* Make a Git commit with a clear, descriptive message (e.g., `git commit -m "Complete Step 1: Package Initialization"`).
* Send a notification message in Teams to Seva to report that the step is complete and ready for review.

---

## ROS 2 CLI Cheatsheet & Troubleshooting

Here are some essential command-line tools and common pitfalls to help you debug your node and inspect the ROS 2 environment while completing the assignment:

### Essential Commands

* **List all active topics:** `ros2 topic list`
* **List all active nodes:** `ros2 node list`
* **Topic info:** `ros2 topic info <topic_name>`
* **Check a topic's publishing rate (frequency):** `ros2 topic hz <topic_name>`
* **Inspect a specific node (view its publishers and subscribers):** `ros2 node info <node_name>`
* **View the actual message data being published:** `ros2 topic echo <topic_name>`

### Advanced CLI Tricks

* **Read a specific field of a message:** Use the `--field` flag to avoid flooding your terminal with massive arrays (like point clouds or images).
* *Example:* `ros2 topic echo /os1/points --field header.frame_id`


* **Verify a package is installed/recognized:** If you aren't sure if your build succeeded, check if ROS 2 can find your package or its executables:
* `ros2 pkg list | grep pointcloud_colorizer`
* `ros2 pkg executables pointcloud_colorizer`



### Common Blockers & Developer Tips

* **Workspace Sourcing:** Your environment (`~/.bashrc`) is pre-configured to automatically source the workspace. However, **this only works if you build your package from the `~/ros2_ws` directory**. If `ros2 run` says your package cannot be found, ensure you built it from the correct path, and try opening a new terminal (or run `source ~/.bashrc`) to refresh the environment.
* **Quality of Service (QoS) Mismatches:** In ROS 2, publishers and subscribers must have compatible QoS settings to communicate. The most common culprit is `Reliability` (e.g., a "Best Effort" sensor publisher won't connect to a "Reliable" subscriber). If your callback isn't triggering despite the topic being active, use `ros2 topic info <topic_name> --verbose` to check for QoS incompatibilities between the publisher and your subscriber.
* **RViz2 and Compressed Images:** RViz2 cannot natively display `sensor_msgs/msg/CompressedImage` messages without external transport plugins. If you try to view the raw camera topic, it will fail. This is why Step 5 requires you to decompress and republish it as a standard `sensor_msgs/msg/Image`.

---

## Visualizing Data with RViz2

RViz2 is your primary tool for verifying spatial data. Here is how to use it effectively for this assignment:

* **Setting the Fixed Frame:** Before you can see any 3D data, RViz2 needs to know the base coordinate system. In the left panel, under **Global Options**, find **Fixed Frame** and type in `os1/os_lidar`.
* **Visualizing a New Topic:** 1. Click the **Add** button at the bottom left.
2. Select the **By topic** tab.
3. Expand the topic you want to view (e.g., `/colorizer/debug/pointcloud`) and double-click the corresponding display type (e.g., `PointCloud2` or `Image`).
* **Displaying Point Cloud Colors:** For your colored point clouds to show up properly, expand your PointCloud2 display on the left panel, find **Color Transformer**, and change it to **RGB8**.
* **Saving Your Layout:** Don't rebuild your RViz2 layout every time you close it. Go to **File > Save Config As...** and save it (e.g., `~/repos/pointcloud_colorizer/config/colorizer.rviz`).
* **Opening a Saved Config:** You can launch RViz2 directly with your saved configuration using the `-d` flag:
`rviz2 -d ~/repos/pointcloud_colorizer/config/colorizer.rviz`

---

## Step-by-Step Assignment Instructions

### Step 1: Package Initialization

**Task:** Create a new ROS 2 C++ package named `pointcloud_colorizer` in `~/repos`. Inside this package, create a basic ROS 2 node class that inherits from `rclcpp::Node`. For now, the node should simply initialize and spin without subscribing or publishing anything.

**Verification:** Build using the command from the workflow instructions. The package must compile successfully without warnings or errors. Running the node via `ros2 run` should keep it active in the terminal without crashing.

### Step 2: Point Cloud Ingestion

**Task:** Add a subscriber to the node that listens to the `/os1/points` topic. Create a callback function. Inside the callback, calculate the total number of points in the incoming cloud and print a log message to the terminal.

**Verification:** Play the `.mcap` bagfile. Run your node. The terminal should continuously print messages stating: "Pointcloud received with [N] points".

### Step 3: Basic Colorization and Visualization (Purple Cloud)

**Task:** Add a publisher that outputs `sensor_msgs/msg/PointCloud2` messages to a new topic (e.g., `/colorizer/debug/purple_cloud`). Modify your point cloud callback. Instead of just printing the count, create a new point cloud message. Copy the spatial coordinates (X, Y, Z) from the incoming cloud, but append an RGB field to make it an `XYZRGB` cloud. Set the color of every single point to purple (e.g., R=128, G=0, B=128) and publish this new cloud.
*(Hint: You may want to look into `sensor_msgs::PointCloud2Modifier` or converting to/from a PCL point cloud format).*

**Verification:** Play the bagfile. Open RViz2, set the fixed frame to `os1/os_lidar`, and add a PointCloud2 display mapped to your debug topic. Set the "Color Transformer" to "RGB8". The point cloud must be visible and entirely purple.

### Step 4: Image Ingestion

**Task:** Create a second subscriber to listen to `/camera_front/image_raw/compressed`. Write a callback that extracts the image dimensions (height and width) from the compressed message structure or by partially decoding it.

**Verification:** Run the node alongside the bagfile. The terminal should print: "Image received with shape: [width] x [height]".

### Step 5: Image Decompression and Republishing

**Task:** Inside the image callback, use `cv_bridge` to decompress the incoming compressed image into a standard OpenCV `cv::Mat` format (BGR8). Convert it back to a standard `sensor_msgs/msg/Image` and publish it to a new topic (e.g., `/colorizer/debug/image`).

**Verification:** Open RViz2 and add an Image display mapped to your debug image topic. You should see the live, uncompressed video feed.

### Step 6: Time Synchronization

**Task:** Remove the standalone subscribers for the image and point cloud. Replace them with a `message_filters::Subscriber` for each, and feed them into a `message_filters::Synchronizer` using an `ApproximateTime` policy. Create a unified callback that receives both the image and the point cloud simultaneously.

**Verification:** Inside the synchronized callback, print a single message: "Synchronized pair received at time [timestamp]". Run the node and verify this prints consistently.

### Step 7: Initial Accurate Colorization (Hardcoded Pipeline)

**Task:** To prove the projection math works before dealing with dynamic lookups, colorize the point cloud using hardcoded parameters.

1. **Hardcoded Extrinsics:** Transform each point from `os1/os_lidar` to `pylon_camera` using the following known transformation matrix:

```text
 0.019  0.000 -1.000 -0.058
-1.000  0.000 -0.019 -0.001
 0.000  1.000  0.000 -0.145
 0.000  0.000  0.000  1.000

```

(if something works wrong, try using an inverse of this matrix instead, as the original is from camera to lidar, and you need lidar to camera for projection)

2. **Hardcoded Intrinsics:** Since we are not reading dynamic camera calibration topics yet, use the following known intrinsic camera matrix ($K$) to project the 3D points onto the 2D image plane:

$$K = \begin{bmatrix} 1194.40697 & 0.0 & 974.09376 \\ 0.0 & 1197.38886 & 596.0597 \\ 0.0 & 0.0 & 1.0 \end{bmatrix}$$

*(Note: You can ignore lens distortion ($D$) for this step).*

3. **Projection:** Map the 3D points to 2D pixels using the $K$ matrix, extract the RGB values from the image, attach them to the points, and publish the `XYZRGB` point cloud.

**Verification:** View the republished point cloud in RViz2 (set "Color Transformer" to "RGB8"). The point cloud will be colored, though the alignment might be slightly flawed at the edges due to the lack of lens distortion correction. This proves your fundamental 3D-to-2D projection logic is working.

### Step 8: Dynamic Extrinsic Extraction (TF2)

**Task:** Discard the hardcoded extrinsic matrix. Add a `tf2_ros::Buffer` and `tf2_ros::TransformListener`. Inside your synchronized callback, dynamically look up the transform from `os1/os_lidar` to `pylon_camera` at the exact timestamp of the synchronized messages. Use this dynamic transform for your 3D point transformation.

**Verification:** Run the node. Print the extracted translation vector to the terminal and verify it roughly matches the `[-0.058, -0.001, -0.145]` baseline. The colored point cloud in RViz2 should look identical to Step 7.

### Step 9: Dynamic Intrinsic Calibration Integration

**Task:** Discard the hardcoded intrinsic matrix. Create a subscriber for `/camera_front/camera_info`. Store the real intrinsic camera matrix ($K$) and distortion coefficients ($D$) in private class variables.

**Verification:** Print a confirmation message to the terminal once the calibration parameters are successfully received and stored.

### Step 10: Final Accurate Colorization

**Task:** Combine everything for the final result.

1. Use OpenCV (`cv::undistort`) with your stored $K$ and $D$ matrices to undistort the incoming image before processing.
2. Project the 3D points (transformed via TF2) onto the 2D plane using the real intrinsic matrix $K$.
3. Publish the final, accurately colored point cloud.

**Verification:** Inspect the point cloud in RViz2. The camera texture should now map cleanly and accurately onto the 3D geometric structures without warping at the edges.
