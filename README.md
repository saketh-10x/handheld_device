# Handheld Device ROS Package

ROS package for handheld mapping device with USB camera and Livox MID360 LiDAR sensor, configured for FAST-LIVO2 SLAM system.

## Overview

This package provides configuration files and launch scripts for a handheld mapping setup consisting of:
- **USB Camera**: For RGB image capture
- **Livox MID360 LiDAR**: Solid-state LiDAR for point cloud data
- **FAST-LIVO2**: Visual-Inertial-LiDAR Odometry and Mapping system

## Hardware Requirements

- Livox MID360 LiDAR sensor
- USB camera (compatible with `usb_cam` ROS package)
- IMU (integrated with MID360)

## Network Configuration for Livox MID360

### IP Address Setup

The Livox MID360 LiDAR requires network configuration to communicate with the host computer:

1. **Default LiDAR IP**: `192.168.1.12`
2. **Host Computer IP**: Should be in the same subnet, e.g., `192.168.1.xxx`

### Network Configuration Steps

1. **Connect LiDAR to computer** via Ethernet cable or connect both to the same network switch

2. **Configure host network interface**:
   ```bash
   sudo ifconfig eth0 192.168.1.100 netmask 255.255.255.0
   # Or use NetworkManager:
   sudo nmcli connection modify <connection-name> ipv4.addresses 192.168.1.100/24
   ```

3. **Verify connectivity**:
   ```bash
   ping 192.168.1.12
   ```

4. **Livox ROS Driver Configuration**:
   - The driver is configured via `livox_ros_driver2` package
   - Default launch file: `$(find livox_ros_driver2)/launch_ROS1/msg_MID360.launch`
   - The driver automatically discovers MID360 on the network

### Troubleshooting Network Issues

- **Cannot ping LiDAR**: Check Ethernet connection and IP configuration
- **Driver not detecting LiDAR**: Verify firewall settings (may need to allow UDP ports)
- **Connection timeout**: Ensure both devices are on the same subnet

## USB Camera Configuration

### Device Setup

The launch file is configured for `/dev/video2` by default. To find your camera device:

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

Update the `video_device` parameter in `launch/usb_cam_livox_mid360.launch` if your camera is on a different device.

### Camera Parameters

Current configuration (in `config/camera_MID360.yaml`):
- **Resolution**: 640x480
- **Model**: Pinhole camera model
- **Focal Length**: fx=535.11, fy=521.31
- **Principal Point**: cx=401.71, cy=270.66
- **Distortion**: None (d0-d3 = 0.0)

**Note**: These parameters should be calibrated for your specific camera setup.

## Configuration Files

### `config/MID360.yaml`

Main configuration file for FAST-LIVO2 system:

#### Key Sections:

- **`common`**: Topic names and enable flags
  - `img_topic`: Camera image topic (`/usb_cam/image_raw`)
  - `lid_topic`: LiDAR point cloud topic (`/livox/lidar`)
  - `imu_topic`: IMU data topic (`/livox/imu`)

- **`extrin_calib`**: Extrinsic calibration parameters
  - `extrinsic_T`: IMU to LiDAR translation [x, y, z] in meters
  - `extrinsic_R`: IMU to LiDAR rotation matrix (3x3)
  - `Rcl`: LiDAR to Camera rotation matrix
  - `Pcl`: LiDAR to Camera translation [x, y, z] in meters

- **`preprocess`**: Point cloud preprocessing
  - `point_filter_num`: Point filtering parameter
  - `filter_size_surf`: Surface filter size (0.1m)
  - `lidar_type`: 1 for Livox Avia/MID360
  - `blind`: Blind zone distance (0.8m)

- **`vio`**: Visual-Inertial Odometry parameters
  - `max_iterations`: Optimization iterations
  - `outlier_threshold`: Feature outlier rejection threshold
  - `patch_size`: Image patch size for feature tracking

- **`lio`**: LiDAR-Inertial Odometry parameters
  - `voxel_size`: Voxel grid size (0.5m)
  - `max_layer`: Maximum voxel layers
  - `max_points_num`: Maximum points per voxel

### `config/camera_MID360.yaml`

Camera intrinsic parameters:
- Camera model (Pinhole)
- Image dimensions (640x480)
- Focal lengths (fx, fy)
- Principal point (cx, cy)
- Distortion coefficients (currently zero)

## Launch Files

### 1. `usb_cam_livox_mid360.launch`

Launches USB camera and Livox MID360 driver for data recording:

```bash
roslaunch handheld_device usb_cam_livox_mid360.launch
```

**What it does**:
- Starts USB camera node (`/dev/video2`, 640x480@30fps)
- Launches Livox MID360 driver
- Publishes topics:
  - `/usb_cam/image_raw` (camera images)
  - `/livox/lidar` (point clouds)
  - `/livox/imu` (IMU data)

**Recording data**:
```bash
# Record all topics
rosbag record -a -O data/your_recording.bag

# Or record specific topics
rosbag record /usb_cam/image_raw /livox/lidar /livox/imu -O data/your_recording.bag
```

### 2. `mapping_mid360.launch`

Launches FAST-LIVO2 mapping system:

```bash
roslaunch handheld_device mapping_mid360.launch
```

**Arguments**:
- `rviz` (default: `true`): Launch RViz visualization

**What it does**:
- Loads MID360 configuration
- Sets up static transform from `camera_init` to `livox_frame`
- Starts FAST-LIVO2 mapping node
- Optionally launches RViz for visualization

**Using with rosbag**:
```bash
# Play bag file
rosbag play data/your_recording.bag --clock

# In another terminal, launch mapping
roslaunch handheld_device mapping_mid360.launch
```

## Calibration

### Camera-LiDAR Calibration

The extrinsic calibration between camera and LiDAR is stored in `config/MID360.yaml`:
- `Rcl`: Rotation matrix (LiDAR → Camera)
- `Pcl`: Translation vector (LiDAR → Camera) in meters

**To recalibrate**:
1. Use calibration tools (e.g., `lidar_camera_calibration` package)
2. Update `Rcl` and `Pcl` values in `MID360.yaml`

### IMU-LiDAR Calibration

IMU to LiDAR transformation:
- `extrinsic_T`: Translation [x, y, z]
- `extrinsic_R`: Rotation matrix (3x3)

## Usage Workflow

### 1. Initial Setup

```bash
# Ensure network is configured for LiDAR
sudo ifconfig eth0 192.168.1.100 netmask 255.255.255.0

# Verify LiDAR connectivity
ping 192.168.1.12
```

### 2. Recording Data

```bash
# Terminal 1: Launch sensors
roslaunch handheld_device usb_cam_livox_mid360.launch

# Terminal 2: Record data
rosbag record -a -O data/my_scan.bag
```

### 3. Mapping

```bash
# Terminal 1: Play recorded data
rosbag play data/my_scan.bag --clock

# Terminal 2: Run mapping
roslaunch handheld_device mapping_mid360.launch rviz:=true
```

## Dependencies

- `usb_cam`: USB camera driver
- `livox_ros_driver2`: Livox LiDAR driver
- `fast_livo`: FAST-LIVO2 SLAM package
- `tf`: Transform library
- `rviz`: Visualization
- `image_transport`: Image transport utilities

## Troubleshooting

### LiDAR Not Detected
- Check network configuration and connectivity
- Verify firewall settings
- Check if Livox driver is running: `rostopic list | grep livox`

### Camera Not Working
- Check device path: `ls -l /dev/video*`
- Update `video_device` parameter in launch file
- Verify camera permissions: `sudo chmod 666 /dev/video2`

### Mapping Issues
- Verify all topics are publishing: `rostopic list`
- Check calibration parameters in `MID360.yaml`
- Ensure time synchronization between camera and LiDAR

## File Structure

```
handheld_device/
├── CMakeLists.txt          # Build configuration
├── package.xml             # ROS package manifest
├── README.md               # This file
├── config/
│   ├── MID360.yaml         # FAST-LIVO2 configuration
│   └── camera_MID360.yaml  # Camera intrinsics
├── launch/
│   ├── usb_cam_livox_mid360.launch  # Sensor recording
│   └── mapping_mid360.launch       # SLAM mapping
└── data/                   # Recorded rosbag files (gitignored)
    ├── fast_livo2_office.bag
    └── fast_livo2_office_v2.bag
```

## License

BSD License

## Maintainer

saketh-10x
