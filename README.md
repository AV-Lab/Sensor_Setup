# Sensor Setup
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://releases.ubuntu.com/22.04/)
[![ZED SDK](https://img.shields.io/badge/ZED_SDK-4.1+-red)](https://www.stereolabs.com/developers/release/)
[![Ouster SDK](https://img.shields.io/badge/Ouster_SDK-0.15.1-lightred)](https://static.ouster.dev/sdk-docs/index.html)

A ROS2 package for configuring, testing, and operating sensors:
- 📸 See3CAM_24CUG V4L2 camera
- 📸 ZED2 Camera (legacy/optional monocular path)
- 🔄 Ouster OS-1 LiDAR

For the staged Ouster/See3CAM clock, PTP, and external-trigger design, see the
[sensor synchronization plan](SYNC.md).

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
  - [Calibration Step by Step](#calibration-step-by-step)
  - [Data Collection Tips](#data-collection-tips)
  - [Camera Intrinsic Calibration](#camera-intrinsic-calibration)
    - [ROS2 Based Calibration](#ros2-based-calibration)
    - [MATLAB Based Calibration](#matlab-based-calibration)
  - [Camera-to-LiDAR Calibration](#camera-to-lidar-calibration)
    - [Solve the Extrinsic](#solve-the-extrinsic)
    - [Calibration Review](#calibration-review)

- [Working with ROS2 Bags](#-working-with-ros2-bags)
  - [Recording](#recording)
  - [Playback](#playback)


## 💻 System Requirements

### Basic Requirements
- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS2 Distribution**: Humble Hawksbill
- **Python**: 3.10 (tested with 3.10.12)
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
pip3 install -r requirements.txt
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

Start the Ouster and See3CAM publishers with a single command:
```bash
ros2 launch sensors launch_all_sensors.py
```

### Individual Sensor Operation

Run LiDAR or camera independently:
```bash
# Start Ouster LiDAR
ros2 run sensors ouster_node --ros-args --remap use_sim_time:=false
```
```bash
# Start ZED Camera
ros2 run sensors zed_node --ros-args --remap use_sim_time:=false
```
```bash
# Start Camera
ros2 run sensors camera_node --ros-args --remap use_sim_time:=false
```

### Data Recording

📊 **Synchronized Data Collection**
- The first step in calibration is collecting synchronized data from all sensors
- `save_node` approximately synchronizes image and point-cloud header stamps
- Manual capture is the default: hold each checkerboard pose still and request
  one pair
- Images are saved as `.png`; point clouds are binary `.pcd` files retaining
  `reflectivity` (or the source driver's `intensity` field)
- Every run creates a non-overwriting session with timestamp/frame metadata

#### Configuration
- 📁 Update topic names in [`sensors/config/save_sample.yaml`](sensors/config/save_sample.yaml)
- Configurable parameters include:
  - Synchronization threshold and queue size
  - Manual or non-blocking interval capture
  - Minimum interval, fresh-pair age, and valid-point gates
  - Output root and number of calibration poses

#### Recording Synchronized Data
```bash
# From rosbag playback
ros2 run sensors save_node --ros-args -p use_sim_time:=true

# Real-time sampling
ros2 run sensors save_node --ros-args -p use_sim_time:=false
```

For the recommended manual workflow, wait until both topics are matching, hold
the target still, and request one fresh pair:

```bash
ros2 service call /calibration_capture/save_pair std_srvs/srv/Trigger "{}"
```

Repeat for each distinct board pose. The default target is 40 poses. Output is
written under:

```text
calibration_data/session_<UTC>/
├── session.json       # saver plus requested sensor configurations
├── manifest.jsonl     # one timestamp/frame/field record per complete pair
├── summary.json       # matched, saved, rejected, and completion counters
├── images/img_0000.png
└── pcds/pc_0000.pcd
```

The service succeeds only after both files and the manifest record are written.
It refuses stale, duplicate, too-close, wrong-frame, undersized, and malformed
pairs. For each pose, vary board position, distance, yaw, pitch, and roll; many
nearly identical frames do not improve calibration.

Audit the completed session before moving it to another machine:

```bash
ros2 run sensors calibration_audit \
  --session calibration_data/session_<UTC> \
  --write-overlays
```

The report checks manifest integrity, synchronization, frame IDs, PCD validity,
checkerboard detection, image sharpness/exposure, board coverage, estimated
camera-frame board pose, and possible duplicate poses. It writes
`quality_report.json` and, when requested, `quality_overlays/` inside the
session. A passing result does not claim that the same board was automatically
isolated in the LiDAR cloud; inspect that in the calibration solver or add a
verified target-specific LiDAR selection stage.

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
- Publishes `PointCloud2` messages (`x`, `y`, `z`, `reflectivity`)

⚠️ **Important Notes**:
- Update LiDAR IP/hostname in config file
- FPS is tied to LiDAR mode (e.g., 512x20 = 20 fps)
- Check sensor status in ouster-cli before launching ROS2 nodes

# 📊 Sensor Calibration

This implementation focuses on two key calibration procedures:
- 📸 Camera intrinsic calibration
- 🔄 Camera-to-LiDAR extrinsic calibration

## Calibration Step by Step

This is the canonical end-to-end workflow. The later sections explain each
tool in more detail, but do not change this order.

### Step 1: Fix the physical camera configuration

Before calculating intrinsics, choose the final camera configuration in
[`sensors/config/camera_config.yaml`](sensors/config/camera_config.yaml):

- Resolution: `1920x1080`
- Pixel format: `UYVY`
- Frame rate: `20 Hz`
- Final lens and focus position

Connect the See3CAM and inspect the controls it actually exposes:

```bash
v4l2-ctl --list-devices
v4l2-ctl -d "/dev/v4l/by-id/<actual-See3CAM-video-index0-name>" \
  --list-ctrls-menus
```

Do not guess exposure-control values from another camera. Find usable values on
the connected unit, then lock focus, exposure, gain, and white balance before
collecting the final extrinsic dataset. Changing focus or resolution after the
intrinsic calibration invalidates that intrinsic calibration.

After confirming the camera-specific control names and ranges, the final
calibration profile should look conceptually like this:

```yaml
controls:
  apply_standard_controls: true
  auto_exposure: false
  auto_white_balance: false
  exposure: <verified camera value>
  gain: <verified camera value>
```

Do not copy the numeric exposure or gain from another device. Confirm the
requested and active values in the publisher log.

### Step 2: Build and start the camera

From the ROS workspace root:

```bash
pip3 install -r src/requirements.txt  # adjust if the repo has another name
colcon build --packages-select sensors
source install/setup.bash
ros2 run sensors camera_node --ros-args -p use_sim_time:=false
```

The camera publisher automatically selects the single matching See3CAM under
`/dev/v4l/by-id`. Verify the final mode in its startup log.

### Step 3: Calibrate and install camera intrinsics

In another terminal, run the ROS camera calibrator using the exact number of
inner corners and measured square size:

```bash
source install/setup.bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.108 \
  image:=/camera/image_raw \
  camera:=/camera/camera_info
```

Replace the placeholder arrays in `camera_config.yaml` with the calibration
result: `distortion`, `camera_matrix_K`, `rectification`, and `projection`.
Then mark the result deliberately:

```yaml
intrinsics:
  calibrated: true
  distortion_model: "plumb_bob"
  provenance:
    method: "ros_camera_calibration"
    calibrated_at_utc: "YYYY-MM-DDTHH:MM:SSZ"
    rms_reprojection_error_px: null  # replace when the tool reports it
    lens_focus_locked: true
```

Rebuild after editing an installed package configuration:

```bash
colcon build --packages-select sensors
source install/setup.bash
```

Restart `camera_node` and verify that `CameraInfo.k[0]` is nonzero and that its
width and height match the published images:

```bash
ros2 topic echo /camera/camera_info --once
```

The calibration audit and review tools intentionally refuse placeholder
intrinsics.

### Step 4: Measure and configure the calibration target

Update [`sensors/config/calibrate.yaml`](sensors/config/calibrate.yaml):

```yaml
target:
  type: "checkerboard"
  inner_corners: [8, 6]   # columns first, then rows; not square count
  square_size_m: 0.108    # physically measured, not nominal print size
```

Use a large, rigid, flat, matte target visible to both sensors. Measure several
squares with a ruler or caliper and check that the printed pattern is not
scaled differently in X and Y.

### Step 5: Verify both live sensor streams

Stop the standalone `camera_node` used for intrinsic calibration, then start
both publishers:

```bash
ros2 launch sensors launch_all_sensors.py
```

Check the streams before saving anything:

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /lidar_points
ros2 topic echo /camera/camera_info --once
ros2 topic echo /lidar_points --field header --once
```

Expected defaults are approximately 20 Hz for both topics, image frame
`camera_optical_frame`, and LiDAR frame `os_sensor`. The current host-receipt
timestamps are acceptable only because the target and sensor platform are held
stationary for calibration.

### Step 6: Capture a calibration session

Use the manual capture mode in
[`sensors/config/save_sample.yaml`](sensors/config/save_sample.yaml). You can
pass the source configuration directly while developing, avoiding a rebuild:

```bash
ros2 run sensors save_node --ros-args \
  -p use_sim_time:=false \
  -p config_file:=/absolute/path/to/sensors/config/save_sample.yaml
```

For every target pose:

1. Move the board to a new distance, image position, yaw, pitch, and roll.
2. Make sure the full checkerboard is visible in the camera.
3. Make sure the board surface is inside the LiDAR field of view.
4. Hold the board and sensor platform still for several frames.
5. Request exactly one fresh pair:

```bash
ros2 service call /calibration_capture/save_pair \
  std_srvs/srv/Trigger "{}"
```

Collect approximately 30–40 diverse poses for solving. Then stop and restart
`save_node` to create a separate validation session with roughly 8–12 new
poses. Do not give the validation session to the solver.

### Step 7: Audit both sessions locally

Run the audit separately on the solver and held-out sessions:

```bash
ros2 run sensors calibration_audit \
  --session calibration_data/session_<solver_UTC> \
  --config /absolute/path/to/sensors/config/calibrate.yaml \
  --write-overlays

ros2 run sensors calibration_audit \
  --session calibration_data/session_<validation_UTC> \
  --config /absolute/path/to/sensors/config/calibrate.yaml \
  --write-overlays
```

Inspect `quality_report.json` and `quality_overlays/`. Before solving, require:

- No manifest, frame, timing, or PCD hard failures.
- Checkerboard detection on every pair you plan to use.
- No obviously blurred or overwhelmingly clipped board images.
- Low duplicate-pose count.
- Board centers distributed across the image.
- Multiple distances and clearly different plane normals.

`candidate_usable` does not prove that the board was hit in the LiDAR cloud.
It means the image target, timing, frames, and general cloud passed. The board
plane still must be isolated and visually verified in the extrinsic solver.

### Step 8: Solve the LiDAR-camera extrinsic

Choose one route:

1. **MATLAB reference route:** move only the audited solver session to the
   MATLAB machine and run `lidarCameraCalibrator`. Use only the indices marked
   usable in the audit, and verify that the selected plane is the physical
   board in every accepted PCD.
2. **Local targetless route:** use
   [direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration)
   on the ROS2 machine. This is a different calibration method with its own
   collection and input requirements; the current checkerboard audit does not
   automatically prepare all of its inputs.
3. **TIER IV route:** use
   [CalibrationTools](https://github.com/tier4/CalibrationTools) when its larger
   dependency and integration footprint is acceptable.

This repository does not yet claim to contain a complete automatic Python
extrinsic solver. OpenCV can detect the image target, but reliable LiDAR-board
association must be tested on real sensor data before such a solver is trusted.

### Step 9: Install the solved transform

Determine the matrix direction returned by the selected solver. This repo
requires:

```text
p_camera_optical = T_camera_lidar × p_os_sensor
```

If the solver returns the opposite direction, invert it before installation.
Then update `calibrate.yaml`:

```yaml
extrinsic:
  valid: true
  method: "actual_solver_name"
  source_frame: "os_sensor"
  target_frame: "camera_optical_frame"
  convention: "p_target = T_target_source * p_source"
  translation_unit: "m"
  matrix: [R11, R12, R13, tx,
           R21, R22, R23, ty,
           R31, R32, R33, tz,
           0,   0,   0,   1]
```

The loader checks that this is a finite, right-handed, orthonormal rigid
transform. `CameraInfo.P` is not the LiDAR-camera extrinsic.

### Step 10: Review only on the held-out session

Start in read-only mode:

```bash
ros2 run sensors calibration_review \
  --session calibration_data/session_<validation_UTC> \
  --config /absolute/path/to/sensors/config/calibrate.yaml
```

Use `n`, `p`, and `j <index>` to inspect every held-out pose. Check alignment
at the image center and edges, at near and far distances, and on multiple board
orientations.

Only if the error is small and consistent may you test refinement:

```bash
ros2 run sensors calibration_review \
  --session calibration_data/session_<validation_UTC> \
  --config /absolute/path/to/sensors/config/calibrate.yaml \
  --mode refine
```

Refinement never overwrites the accepted transform. `save` creates an
unaccepted candidate containing the original matrix and total change. If the
tool warns that the change is too large, rerun the solver or investigate
intrinsics, frame direction, board selection, and timing.

### Step 11: Publish and inspect the accepted TF

After accepting and installing the final matrix:

```bash
ros2 run sensors calibration_tf --ros-args \
  -p config_file:=/absolute/path/to/sensors/config/calibrate.yaml
```

In another terminal:

```bash
ros2 run tf2_ros tf2_echo os_sensor camera_optical_frame
```

The stored matrix maps LiDAR points into the camera frame. Because the TF tree
uses `os_sensor` as parent, the publisher broadcasts the mathematically correct
inverse for the parent-to-child TF edge.

### Step 12: Acceptance and archiving

Accept the calibration only when:

- All held-out poses show consistent alignment.
- Alignment does not drift systematically with image position or distance.
- Repeating the solve with another diverse subset gives a similar transform.
- Any manual adjustment is small and improves all held-out views, not one view.

Archive the solver and validation session directories, quality reports, exact
camera intrinsics, accepted extrinsic, requested sensor configurations, and
software versions together. For moving-platform use, separately complete the
PTP, exposure timestamp, and LiDAR deskew steps in [`SYNC.md`](SYNC.md).

## 🎯 Data Collection Tips

### Essential Setup
- Use a large checkerboard (at least 30x30cm) for reliable detection
  - 🔗 Generate pattern: [calib.io Pattern Generator](https://calib.io/pages/camera-calibration-pattern-generator)
  - 📏 Mount on rigid, flat surface (foam board works well)
  - ⚠️ Verify printout dimensions are exact
  - Black squares should be truly black (matte finish preferred)
  
### Environment
- ☀️ Good lighting but avoid direct sunlight (causes glare)
- 🧹 Clear area of objects with checkerboard-like patterns
- Ensure even lighting but avoid reflective surfaces

### Collection Strategy
- 📸 Capture checkerboard at various:
  - Distances: 1-5 meters (start close, then move back)
  - Angles: 15-45 degrees from sensor axis
  - Positions: cover entire sensor field of view
- 🎯 Aim for 15-50 diverse, high-quality frame pairs
- 🖐️ Hold pattern still during capture (motion blur affects accuracy)

### Pro Tips
- Check image exposure - avoid over/underexposed areas
- Mark floor positions for repeatable captures
- Run `calibration_audit` on a few samples before collecting the full dataset
- Back up raw data before processing

## 🎥 Camera Intrinsic Calibration

### Purpose
Camera intrinsic calibration determines the internal parameters affecting 3D-to-2D projection.

### Parameters Calibrated
- 📏 Focal length (fx, fy)
- 🎯 Principal point (cx, cy)
- 🔧 Distortion coefficients
  - Radial (k1, k2)
  - Tangential (p1, p2)

### Method
1. Capture checkerboard images
2. Detect corners
3. Apply Zhang's method

After calibration, update in config files ([Zed_camera](/sensors/config/zed_config.yaml), [Camera](/sensors/config/camera_config.yaml)):
```yaml
# Maps 3D camera coordinates → 2D image points
distortion: [k1, k2, p1, p2, k3]
camera_matrix_K: [fx, 0, cx, 0, fy, cy, 0, 0, 1] 
rectification: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
```

### Calibration Options

#### 🤖 ROS2 Based Calibration
```bash
# Install ROS calibration package
sudo apt-get install ros-$ROS-camera-calibration

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

#### 📐 MATLAB Based Calibration
```matlab
% Launch Camera Calibrator App
cameraCalibrator

% Or use Apps tab -> Camera Calibrator
```
For detailed MATLAB calibration workflow (including both intrinsic and extrinsic calibration), see [MATLAB Based Calibration](#2-matlab-based-calibration) in Camera-to-LiDAR section.

## 🔄 Camera-to-LiDAR Calibration

### Purpose
Determines geometric transformation between sensors for point cloud projection.

### Parameters Calibrated
- 🔄 Rotation matrix (R)
- 📏 Translation vector (t)

### Solve the Extrinsic

Follow Steps 7–9 in the canonical workflow above. MATLAB is the reference path
for the current checkerboard dataset, while the linked ROS2 tools provide local
alternatives with different input requirements. Regardless of solver, install
only a matrix whose direction is explicitly:

```text
p_camera_optical = T_camera_lidar × p_os_sensor
```

The config loader rejects placeholders, malformed homogeneous matrices,
non-orthonormal rotations, reflections, and non-finite values.

### Calibration Review

Follow Steps 10–11 above. Review mode is read-only; refinement mode creates an
unaccepted candidate and never overwrites the solver output. Available review
commands are `n`, `p`, `j <index>`, `matrix`, and `q`. Refinement adds
`tx+/-`, `ty+/-`, `tz+/-`, `roll+/-`, `pitch+/-`, `yaw+/-`, `reset`, and
`save`.

### Validation

- Use static held-out board poses that were not used by the solver.
- Check the image center and edges at multiple distances and orientations.
- Repeat the full calibration and compare rotation and translation.
- Treat depth-dependent overlay error as a possible intrinsic or translation
  problem; do not hide it with a single-view manual adjustment.
- Validate dynamic timing separately. Dynamic scenes are not a clean geometric
  calibration check when the camera exposure time and LiDAR scan time differ.

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

```v4l2-ctl --list-devices```
