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
- [Sensor Calibration](#sensor-calibration)
  - [Data Collection](#data-collection-tips)
  - [Camera Intrinsic Calibration](#Camera-Intrinsic-Calibration)
  - [Camera-to-LiDAR Calibration](#Camera-to-LiDAR-Calibration)

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

# Sensor Calibration

This implementation focuses on two key calibration procedures:
Camera intrinsic calibration
Camera-to-LiDAR extrinsic calibration

## Data Collection Tips
- Use a large checkerboard (at least 30x30cm) for better detection by both sensors
  - Generate calibration pattern: [calib.io Pattern Generator](https://calib.io/pages/camera-calibration-pattern-generator)
  - Print and mount on rigid, flat surface
- Ensure good lighting conditions but avoid direct sunlight
- Clear the calibration area of objects with similar patterns to checkerboard
- Keep checkerboard flat and stable during capture
- Capture frames with checkerboard at different:
  - Distances (1-5 meters recommended)
  - Angles relative to sensors
  - Positions in the field of view
- Minimum 10-15 good frame pairs recommended

## Camera Intrinsic Calibration

### Purpose
Camera intrinsic calibration determines the internal parameters of the camera that affect how 3D points are projected onto the 2D image plane.

### Parameters Calibrated
- Focal length (fx, fy)
- Principal point (cx, cy)
- Distortion coefficients
  - Radial distortion (k1, k2)
  - Tangential distortion (p1, p2)

### Method
Capture multiple images of a checkerboard pattern
Detect corners in the images
Solve for intrinsic parameters using Zhang's method

After calibration, update in config files ([Zed_camera](/sensors/config/zed_config.yaml), [Elp_camera](/sensors/config/elp_config.yaml)):
```yaml
# camera_matrix_K maps 3D points in camera coordinate frame to 2D image points
distortion: [k1, k2, p1, p2, k3]
camera_matrix_K: [fx, 0, cx, 0, fy, cy, 0, 0, 1] 
rectification: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
```

### Calibration Options

#### ROS2 Based Calibration
```bash
# Install ROS calibration package
sudo apt-get install ros-<ros2-distro>-camera-calibration
```
```bash
# Run calibration node for monocular camera
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camera/camera_info

# Parameters:
# --size: Number of inner corners (width x height)
# --square: Size of each square in meters
# /camera/image_raw: Raw image topic
# /camera/camera_info: Camera info topic 
```
Follow calibration steps:
- Move checkerboard to fill calibration bars
- Click CALIBRATE when ready
- Click SAVE after successful calibration
- Find results in ~/.ros/camera_info/

#### MATLAB Based Calibration
MATLAB can perform both intrinsic and extrinsic calibration together:

1. Start MATLAB and run Calibrator:
```matlab
% Option 1: Use command line
cameraCalibrator  % For intrinsic only
lidarCameraCalibrator  % For both intrinsic and extrinsic

% Option 2: Use Apps tab in MATLAB
% Click 'Lidar Camera Calibrator' under Apps
```

2. Troubleshooting checkerboard detection:
```matlab
% If automatic plane detection fails:
% a) Manual plane selection:
% - Click 'Select Region' in the toolbar
% - Draw region around checkerboard in pointcloud

% b) Adjust detection parameters:
% - Click 'Settings' in the toolbar
% - Modify 'Plane Detection Threshold'
% - Try values between 0.01 and 0.1
```
## Camera-to-LiDAR Calibration

### Purpose
Determines the geometric transformation between the camera and LiDAR sensor, enabling fusion of 2D images with 3D point clouds.

### Parameters Calibrated
- Rotation matrix (R)
- Translation vector (t)
- Together they form the transformation from LiDAR to camera coordinate system

### Calibration Workflow
- Collect synchronized data:
  - Use save_node for synchronized frame capture (see [Data Recording](#data-recording))
  - Configure sampling parameters in [save_sample.yaml](/sensors/config/save_sample.yaml)

- Perform calibration using MATLAB:
  - Use collected synchronized frames
  - MATLAB Lidar Camera Calibrator App can compute both intrinsic and extrinsic parameters together

- Update projection matrix in config:
```yaml
projection: [P11, P12, P13, P14, P21, P22, P23, P24, P31, P32, P33, P34]
```

### Refinement
Fine-tune calibration results using:
```bash
ros2 run sensors interactive_node
```

## Verification
- Visualize projected pointcloud on image
- Check alignment at different distances
- Verify with new data not used in calibration
- Use interactive_node for manual adjustments
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