# Hexapod RealSense Visual + Depth SLAM with SLAM Toolbox

This repository demonstrates **2D SLAM** on a hexapod robot using an Intel RealSense D455 camera. It combines **visual odometry** (`rtabmap_odom`) with **depth-to-laserscan** conversion, allowing `slam_toolbox` to produce a 2D occupancy map in ROS 2 Humble.

---

## 🚀 Launch Sequence

**Run the following commands in order:**

1. **Start SLAM Toolbox (asynchronous SLAM mapping):**
    ```bash
    ros2 launch hexapod_mapping toolbox.launch.py
    ```

2. **Broadcast static transforms (`base_link`, camera, etc.):**
    ```bash
    ros2 launch hexapod_mapping transform.launch.py
    ```

3. **Start the RealSense camera driver:**
    ```bash
    ros2 launch realsense2_camera rs_launch.py \
        rgb_camera.profile:=640x480x15 \
        depth_module.profile:=640x480x15 \
        enable_gyro:=true \
        enable_accel:=true \
        unite_imu_method:=2
    ```

4. **Start RGB-D visual odometry (input odometry for SLAM Toolbox):**
    ```bash
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
    ```

5. **Start the depth-to-laserscan converter:**
    ```bash
    ros2 run depth_to_laserscan depth_to_laserscan_node
    ```

6. **(Optional) Launch mapping with RViz and configuration:**
    ```bash
    ros2 launch hexapod_mapping mapping.launch.py
    ```

---

## 🌐 TF Tree Overview

```
map
 └── odom
      └── base_link
           └── camera_link
                ├── camera_color_frame
                │    └── camera_color_optical_frame
                └── camera_depth_frame
                     └── camera_depth_optical_frame
```

- `base_link` is the root of the robot.
- `camera_link` is fixed to the robot chassis.
- `odom` is published by `rtabmap_odom`.
- `map → odom` is published by `slam_toolbox` (for loop closure and drift correction).

---

## 🔁 Important Topics

| Topic   | Type                        | Publisher                    | Consumer           | Frame      |
|---------|-----------------------------|------------------------------|--------------------|------------|
| /scan   | `sensor_msgs/LaserScan`     | depth_to_laserscan_node      | slam_toolbox       | base_link  |
| /odom   | `nav_msgs/Odometry`         | rtabmap_odom                 | slam_toolbox       | odom       |
| /tf, /tf_static | TF transforms        | all nodes                    | everyone           | -          |

---

## ⚙️ Key Parameters

### **slam_toolbox** (in `toolbox.launch.py`)
- `scan_topic`: `/scan`
- `base_frame`: `base_link`
- `odom_frame`: `odom`
- `map_frame`: `map`
- `use_sim_time`: `false`

### **rtabmap_odom**
- `publish_tf`: `true`
- `approx_sync`: `true`
- `frame_id`: `base_link`
- `odom_frame_id`: `odom`
- `queue_size`: `30`
- `Reg/MinInliers`: `10` (for odometry stability)

---

## 📸 Sensor Calibration

Intel RealSense D455 intrinsics (adjust as needed):
- `fx = 617.0`
- `fy = 617.0`
- `cx = 320.0`
- `cy = 240.0`

---

## 🛠️ Node Roles

- **rtabmap_odom**: Provides robust odometry from RGB-D data.
- **slam_toolbox**: Performs 2D mapping with loop closure and map serialization.
- **depth_to_laserscan_node**: Generates 2D laser scans from depth images (replaces a real LiDAR).
- **realsense2_camera**: Publishes depth, RGB, and IMU data.
- **transform.launch.py**: Establishes all required static transforms for SLAM and navigation.

---

## 🧪 Testing Tips

- Use **rviz2** to visualize `/map`, `/odom`, `/scan`, and the TF tree.
- If you see `Message Filter dropping message` errors, try reducing the `/scan` rate or ensure TF timestamps are synchronized.
- Run `ros2 topic hz /scan` — a frequency of ~5 Hz works best with slam_toolbox.

---

## 🧼 Future Improvements

- Add rosbag2 recording support.
- Integrate joystick/keyboard teleop.
- Add YAML config files for launch parameters.

---
