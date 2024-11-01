# Sensor Setup

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://releases.ubuntu.com/22.04/)
[![ZED SDK](https://img.shields.io/badge/ZED_SDK-4.1+-red)](https://www.stereolabs.com/developers/release/)
[![Ouster SDK](https://img.shields.io/badge/Ouster_SDK-0.11.1-lightred)](https://static.ouster.dev/sdk-docs/index.html)

A ROS2 package for configuring, testing, and operating sensors:
- 📸 ZED2 Camera (Monocular Mode)
- 🔄 Ouster OS-1 LiDAR

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
- **Python**: 3.10 or higher (tested with 3.10.12)
- **CUDA**: 12.0 or higher (required for ZED SDK)
- **Network**: Ethernet port for LiDAR connection

### ZED SDK Installation
1. Download ZED SDK for Ubuntu 22.04:
```bash
wget https://download.stereolabs.com/zedsdk/4.2/cu12/ubuntu22 -O zed_sdk.run
```

2. Make the installer executable and run:
```bash
chmod +x zed_sdk.run
./zed_sdk.run
```

### Ouster SDK Installation
Install using pip:
```bash
pip3 install ouster-sdk
```

### Network Configuration for Ouster LiDAR
1. 🎥 **Initial Setup**: Follow the [Ouster Connection Tutorial](https://www.youtube.com/watch?v=nTm2HY2OEfs) to properly connect your LiDAR via Ethernet.

2. 🎥 **Visualization**: For testing visualization with ouster-cli, watch the [Ouster Visualization Guide](https://www.youtube.com/watch?v=m0ANVFunObU).

## 🚀 Quick Start

1. Create and enter a ROS2 workspace:
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

Start all sensors with a single command:
```bash
ros2 launch sensors launch_all_sensors.py
```

### Individual Sensor Operation

Run LiDAR or camera independently:
```bash
# Start Ouster LiDAR
ros2 run sensors ouster_node --ros-args --remap use_sim_time:=false

# Start ZED Camera
ros2 run sensors zed_node --ros-args --remap use_sim_time:=false
```

### Data Recording

📊 **Synchronized Data Collection**
- The first step in calibration is collecting synchronized data from all sensors
- The save_node enables synchronized capture of image frames and pointcloud data
- Data format: `.png` for images and `.pcd` for pointclouds

#### Configuration
- 📁 Update topic names in [`sensors/config/save_sample.yaml`](sensors/config/save_sample.yaml)
- Configurable parameters include:
  - Synchronization threshold
  - Sample folder paths
  - Delay between frames
  - Number of samples
  - Other sampling parameters

#### Recording Synchronized Data
```bash
# From rosbag playback
ros2 run sensors save_node --ros-args -p use_sim_time:=true

# Real-time sampling
ros2 run sensors save_node --ros-args -p use_sim_time:=false
```

⚠️ **Note**: Refer to [Working with ROS2 Bags](#-working-with-ros2-bags) section for detailed guidance on when and how to use `use_sim_time`.

## 📊 Sensor Details

### Frame Orientations

#### Ouster LiDAR (Right-hand Rule)
```
X ➡️ Forward (depth)
Y ⬅️ Left
Z ⬆️ Up
```

#### Camera Frame (CV2 Convention)
```
X ➡️ Right
Y ⬇️ Down
Z ➡️ Forward (depth)
```

### ZED Camera Setup

The ZED camera operates in monocular mode using the left lens and publishes:
1. **Camera Image**: Raw image feed
2. **Camera Info**: Camera intrinsic parameters (`CameraInfo` type)

#### Configuration
- 📁 Config file: [`sensors/config/zed_config.yaml`](sensors/config/zed_config.yaml)
- Uses default settings if distortion parameters aren't specified
- Identity rectification matrix (monocular mode)

### Ouster LiDAR Setup

- Compatible with OS-1 Ouster
- 📁 Config file: [`sensors/config/ouster_config.yaml`](sensors/config/ouster_config.yaml)
- Publishes `PointCloud2` messages (x, y, z, intensity)

⚠️ **Important Notes**:
- Update LiDAR IP/hostname in config file
- FPS is tied to LiDAR mode (e.g., 512x20 = 20 fps)
- Check sensor status in ouster-cli before launching ROS2 nodes

## 📦 Working with ROS2 Bags

### Recording

1. Configure time source:
```bash
# Set system time for recording
ros2 param set /your_node use_sim_time false
# Or via launch file:
ros2 run sensors node_to_run --ros-args -p use_sim_time:=false
```

2. Start recording:
```bash
ros2 bag record -a -o my_rosbag
```

### Playback

1. Set simulation time:
```bash
# Enable simulation time for playback
ros2 param set /your_node use_sim_time true
# Or via launch file:
ros2 run sensors save_node --ros-args -p use_sim_time:=true
```

2. Play recorded data:
```bash
ros2 bag play my_rosbag --clock 100
```

#### Best Practices
- ✅ Maintain consistent time sources across nodes
- ✅ Set `use_sim_time` in node constructor for new nodes
- ✅ Configure timing in launch files when possible
- ✅ Verify settings: `ros2 param get /your_node use_sim_time`
- ✅ Review all config files before starting sensors

---
For issues or feature requests, please [open an issue](https://github.com/AV-Lab/Sensor_Setup/issues) on our GitHub repository.