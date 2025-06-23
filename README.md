# Hexapod RealSense Visual + Depth SLAM with SLAM Toolbox

This repository implements 2D SLAM on a hexapod robot using a RealSense D455 camera. It uses **visual odometry** (`rtabmap_odom`) and **depth-to-laserscan** conversion for `slam_toolbox` to build a 2D occupancy map in ROS 2 Humble.

---

## 📦 Launch Sequence

The following commands should be run in **order**:

```bash
# 1. Start SLAM Toolbox for asynchronous SLAM mapping
ros2 launch hexapod_mapping toolbox.launch.py

# 2. Broadcast static transforms (base_link, camera, etc.)
ros2 launch hexapod_mapping transform.launch.py

# 3. Start the RealSense camera driver
ros2 launch realsense2_camera rs_launch.py \
    rgb_camera.profile:=640x480x15 \
    depth_module.profile:=640x480x15 \
    enable_gyro:=true \
    enable_accel:=true \
    unite_imu_method:=2

# 4. Start RGB-D visual odometry (used as odometry input to slam_toolbox)
ros2 run rtabmap_odom rgbd_odometry \
    --ros-args \
    --remap rgb/image:=/camera/camera/color/image_raw \
    --remap depth/image:=/camera/camera/depth/image_rect_raw \
    --remap rgb/camera_info:=/camera/camera/color/camera_info \
    -p frame_id:=base_link \
    -p odom_frame_id:=odom \
    -p publish_tf:=true \
    -p approx_sync:=true \
    -p approx_sync_max_interval:=0.02 \
    -p Reg/MinInliers:=10 \
    -p queue_size:=30

# 5. Start the custom node that converts depth images to LaserScan format
ros2 run depth_to_laserscan depth_to_laserscan_node

# 6. (Optional) Mapping launch with RViz and configuration
ros2 launch hexapod_mapping mapping.launch.py
