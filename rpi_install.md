# RTAB-Map with RealSense D455 on Raspberry Pi 4 (ROS 2 Jazzy)

This documentation covers the full setup and usage of **RTAB-Map** with **Intel RealSense D455** on **Raspberry Pi 4** running **ROS 2 Jazzy**.

---

## 🛠️ Prerequisites

* Raspberry Pi 4 (4GB or 8GB recommended)
* Ubuntu 24.04 Server or Desktop
* ROS 2 Jazzy installed
* Workspace at `~/ros2_ws`

---

## ✅ Step-by-Step Setup

### 1. Install Required Packages

Install build tools and utilities:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions ccache
```

### 2. Add Swap Space (Important for Pi Builds)

```bash
sudo swapoff /swapfile
sudo rm /swapfile
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

Verify:

```bash
free -h
```

---

## 📦 Clone Required Repositories

### RTAB-Map ROS 2 Wrapper

```bash
cd ~/ros2_ws/src

git clone https://github.com/introlab/rtabmap.git
cd rtabmap && git checkout jazzy-devel

cd ~/ros2_ws/src
git clone https://github.com/introlab/rtabmap_ros.git
cd rtabmap_ros && git checkout jazzy-devel
```

### RealSense ROS 2 Driver

```bash
cd ~/ros2_ws/src

git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
```

> Make sure librealsense SDK is installed. If not, install via: [https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution\_linux.md](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

---

## 🔨 Build the Workspace

```bash
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --parallel-workers 2
```

Source the environment:

```bash
. install/setup.bash
```

---

## 🎯 Run RTAB-Map with RealSense D455

### Terminal 1: Launch RealSense D455

```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.profile:=640x480x15 \
  depth_module.profile:=640x480x15 \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  enable_sync:=true \
  depth_module.enable_auto_exposure:=true \
  color_backlight_compensation:=1
```

### Terminal 2: Launch RTAB-Map

```bash
. ~/ros2_ws/install/setup.bash

ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  frame_id:=camera_link \
  approx_sync:=false
```

---

## 🧭 Visualize with RViz

```bash
. ~/ros2_ws/install/setup.bash
rviz2
```

Add the following in RViz:

* **Map**: `/map`
* **PointCloud2**: `/rtabmap/cloud_map`
* **TF**
* **Odometry**: `/odom`
* **RobotModel** (optional)

---

## ✅ Notes

* `approx_sync:=false` ensures depth and RGB images are exactly synchronized.
* Use `frame_id:=camera_link` to match the D455’s base frame.
* IMU settings are used to enhance pose estimation, though D455 does not provide odometry directly (visual odometry fallback works).

---

## 📂 Directory Reference

```
~/ros2_ws/src/
├── rtabmap
├── rtabmap_ros
└── realsense-ros
```

---

## ✅ Tested On

* Raspberry Pi 4 8GB
* Ubuntu 24.04 Server
* ROS 2 Jazzy
* RealSense D455 (USB 3)

---

For any issues, check `/tf` tree and image topics using:

```bash
ros2 topic list
ros2 run tf2_tools view_frames
```


