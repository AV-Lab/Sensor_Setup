
# Sensor Setup

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://releases.ubuntu.com/22.04/)
[![ZED SDK](https://img.shields.io/badge/ZED_SDK-4.1+-red)](https://www.stereolabs.com/developers/release/)
[![Ouster SDK](https://img.shields.io/badge/Ouster_SDK-0.11.1-lightred)](https://static.ouster.dev/sdk-docs/index.html)

A ROS2 package for configuring, testing, and operating sensors, specifically:
- ZED2 Camera (Monocular Mode)
- Ouster OS-1 LiDAR

## 📋 Table of Contents
- [System Requirements](#-system-requirements)
  - [Basic Requirements](#basic-requirements)
  - [ZED SDK Installation](#zed-sdk-installation)
  - [Ouster SDK Installation](#ouster-sdk-installation)
- [Quick Start](#-quick-start)
- [Usage](#-usage)
  - [Launch Options](#launch-options)
  - [Individual Sensor Operation](#individual-sensor-operation)
  - [Data Recording](#data-recording)
- [Sensor Details](#-sensor-details)
  - [Frame Orientations](#frame-orientations)
  - [ZED Camera Setup](#zed-camera-setup)
  - [Ouster LiDAR Setup](#ouster-lidar-setup)
- [Working with ROS2 Bags](#-working-with-ros2-bags)
  - [Recording](#recording)
  - [Playback](#playback)


## 💻 System Requirements

### Basic Requirements
- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS2 Distribution**: Humble Hawksbill
- **Python**: 3.10 or higher
- **CUDA**: 12.1 or higher (for ZED SDK)

### ZED SDK Installation
1. Download ZED SDK for Ubuntu 22.04:
```bash
wget https://download.stereolabs.com/zedsdk/4.0/cu117/ubuntu22 -O zed_sdk.run
```

2. Make the installer executable:
```bash
chmod +x zed_sdk.run
```

3. Run the installer:
```bash
./zed_sdk.run
```

4. Install ZED ROS2 wrapper:
```bash
cd ~/ros2_sensor_ws/src
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

### Ouster SDK Installation
- Install Ouster SDK:
```bash
pip3 install ouster-sdk
```

### Network Configuration for Ouster LiDAR
1. after connecting the lidar to your IPC / computer via Eternet cable check if network is working.
 follow this video for more information : [Connecting ouster tutorial](https://www.youtube.com/watch?v=nTm2HY2OEfs&ab_channel=Ouster)

2. To verify connection and test visulization with ouster-cli, follow this video [Visualize Ouster](https://www.youtube.com/watch?v=m0ANVFunObU&ab_channel=Ouster)

## 🚀 Quick Start

1. Create a ROS2 workspace:
```bash
mkdir -p ~/ros2_sensor_ws/src
cd ~/ros2_sensor_ws/src
```

2. Clone the repository:
```bash
git clone https://github.com/AV-Lab/Sensor_Setup .
```

3. Build and source:
```bash
cd ~/ros2_sensor_ws
colcon build && source install/setup.bash
```

## 🎮 Usage

### Launch Options

Launch all sensors with a single command:
```bash
ros2 launch sensors launch_all_sensors.py
```

### Individual Sensor Operation

Start Ouster LiDAR:
```bash
ros2 run sensors ouster_node --ros-args --remap use_sim_time:=false
```

Start ZED Camera:
```bash
ros2 run sensors zed_node --ros-args --remap use_sim_time:=false
```

### Data Recording

Save synchronized samples from rosbag:
```bash
ros2 run sensors save_node 100 'images_x' 'pcds_x' 10 10 1 --ros-args -p use_sim_time:=true
```

Save synchronized samples in real-time:
```bash
ros2 run sensors save_node 100 'images_x' 'pcds_x' 10 10 1 --ros-args -p use_sim_time:=false
```

Parameters for save_node:
```
ros2 run sensors save_node --num_saves --image_folder --pcd_folder --set_size --set_delay --frame_delay --ros-args -p use_sim_time:=true
```

## 📊 Sensor Details

### Frame Orientations

#### Ouster LiDAR (Right-hand Rule)
- X: Forward (depth)
- Y: Left
- Z: Up

#### Camera Frame (CV2 Convention)
- X: Right
- Y: Down
- Z: Forward (depth)

### ZED Camera Setup

The ZED camera operates in monocular mode using the left lens and publishes:
1. **Camera Image**: Provides the raw image feed from the camera.
2. **Camera Info**: Publishes intrinsic parameters of the camera as a `CameraInfo` type message.


#### Configuration
- Configuration file: [sensors/config/zed_config.yaml](sensors/config/zed_config.yaml).
- Default settings apply if distortion parameters are not set.
- Rectification matrix is identity (monocular mode).

### Ouster LiDAR Setup

- Configured for OS-1 Ouster
- Configuration file: [sensors/config/ouster_config.yaml](sensors/config/ouster_config.yaml).
- Publishes PointCloud2 messages (x, y, z, intensity).

⚠️ **Important Notes**:
- Update LiDAR IP/hostname in config file before use.
- FPS is determined by LiDAR mode (e.g., 512x20 = 20 fps)

## 📦 Working with ROS2 Bags

### Recording

1. Set `use_sim_time` to false for all nodes:
```bash
ros2 param set /your_node use_sim_time false
# Or in launch file:
ros2 run sensors node_to_run --ros-args -p use_sim_time:=false
```

2. Record all topics:
```bash
ros2 bag record -a -o my_rosbag
```

### Playback

1. Set `use_sim_time` to true for nodes receiving playback:
```bash
ros2 param set /your_node use_sim_time true
# Or in launch file:
ros2 run sensors save_node --ros-args -p use_sim_time:=true
```

2. Play rosbag with clock publishing:
```bash
ros2 bag play my_rosbag --clock 100
```

#### Best Practices
- Ensure consistent time source across all nodes
- Set `use_sim_time` in node constructor for nodes created after playback
- Configure time settings in launch files when possible
- Verify settings: `ros2 param get /your_node use_sim_time`
- Review config files before sensor startup



