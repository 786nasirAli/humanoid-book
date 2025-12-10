# Sensor Simulation Setup Guide

## Overview
This guide details how to set up and validate the sensor simulation in Gazebo for the Digital Twin project, including LiDAR, depth camera, and IMU sensors.

## Supported Sensors

### 1. LiDAR (2D)
- **Type**: Ray sensor with 180° horizontal field of view
- **Resolution**: 720 samples (0.5° resolution)
- **Range**: 0.1m to 30m
- **Update Rate**: 10 Hz
- **Output**: sensor_msgs/LaserScan
- **Noise Model**: Gaussian noise with 1cm standard deviation

### 2. Depth Camera
- **Type**: Depth sensor with RGB capabilities
- **Resolution**: 640x480 pixels
- **Field of View**: 60° horizontal
- **Range**: 0.1m to 10m
- **Update Rate**: 30 Hz
- **Output**: sensor_msgs/Image (depth) and sensor_msgs/CameraInfo
- **Noise Model**: Distance-dependent noise with optical center effects

### 3. IMU (Inertial Measurement Unit)
- **Type**: Inertial measurement unit
- **Update Rate**: 100 Hz
- **Output**: sensor_msgs/Imu
- **Measurements**:
  - Angular velocity (x, y, z axes) with noise ~0.1°/s stddev
  - Linear acceleration (x, y, z axes) with noise ~17mg stddev
- **Noise Model**: Includes bias drift and random walk characteristics

## Sensor Models

### LiDAR Sensor Model
Location: `simulation/gazebo/models/lidar_sensor.sdf`

The LiDAR model includes:
- Visual representation (black cylinder)
- Collision geometry
- Ray sensor plugin with realistic parameters
- ROS 2 integration via `libgazebo_ros_ray_sensor.so`

### Depth Camera Sensor Model
Location: `simulation/gazebo/models/depth_camera.sdf`

The depth camera model includes:
- Visual representation (white box)
- Collision geometry
- Depth camera sensor with realistic parameters
- ROS 2 integration via `libgazebo_ros_camera.so`

### IMU Sensor Model
Location: `simulation/gazebo/models/imu_sensor.sdf`

The IMU model includes:
- Visual representation (yellow box)
- Collision geometry
- IMU sensor with realistic parameters
- ROS 2 integration via `libgazebo_ros_imu.so`

## Mounting Sensors on Robot

### Robot with Sensors URDF
Location: `simulation/gazebo/models/robot_with_sensors.urdf`

The robot model includes mounting positions for all three sensors:
- LiDAR mounted on top of sensor mount
- Depth camera mounted on front of sensor mount
- IMU mounted internally in sensor mount

Each sensor has appropriate joints and Gazebo plugins configured.

## Noise Models

### LiDAR Noise Model
Location: `simulation/ros2/src/sensor_simulator/src/lidar_noise_model.cpp`

The LiDAR noise model simulates:
- Gaussian measurement noise
- Distance-dependent error characteristics
- Bias that may drift over time
- Realistic signal-to-noise ratio

### Depth Camera Noise Model
Location: `simulation/ros2/src/sensor_simulator/src/depth_noise_model.cpp`

The depth camera noise model simulates:
- Distance-dependent noise (increases with distance squared)
- Optical center-dependent error
- Realistic depth measurement characteristics

### IMU Noise Model
Location: `simulation/ros2/src/sensor_simulator/src/imu_noise_model.cpp`

The IMU noise model simulates:
- Gyroscope noise with bias drift and random walk
- Accelerometer noise with bias drift and random walk
- Realistic IMU error characteristics based on sensor specifications

## ROS 2 Integration

### Sensor Publisher
Location: `simulation/ros2/src/sensor_simulator/src/sensor_publisher.py`

The sensor publisher node:
- Subscribes to Gazebo sensor data
- Applies additional realistic noise models
- Publishes sensor data on appropriate ROS 2 topics

### Topics
- LiDAR: `/lidar/scan` (sensor_msgs/LaserScan)
- Depth Camera: 
  - `/depth_cam/image_raw` (sensor_msgs/Image)
  - `/depth_cam/camera_info` (sensor_msgs/CameraInfo)
- IMU: `/imu/data` (sensor_msgs/Imu)

## Setting Up Sensor Simulation

### 1. Launch Gazebo with Sensors
```bash
# Navigate to ROS 2 workspace
cd simulation/ros2

# Source ROS 2 environment
source /opt/ros/foxy/setup.bash
source install/setup.bash

# Launch robot with sensors in Gazebo
ros2 launch robot_description robot_spawn.launch.py
```

### 2. Verify Sensor Data
```bash
# Check available topics
ros2 topic list | grep sensor

# Monitor LiDAR data
ros2 topic echo /lidar/scan

# Monitor depth camera data
ros2 topic echo /depth_cam/image_raw

# Monitor IMU data
ros2 topic echo /imu/data
```

### 3. Launch Sensor Publisher (if needed)
```bash
# Run the sensor publisher node
python3 simulation/ros2/src/sensor_simulator/src/sensor_publisher.py
```

## Validation Steps

### 1. Sensor Data Validation
Run the sensor validation script to verify realistic data:

```bash
# Run sensor validation
python3 simulation/ros2/src/sensor_simulator/test/sensor_validation.py
```

This script validates:
- LiDAR ranges are within expected bounds
- Depth camera produces realistic depth values
- IMU data shows expected characteristics (gravity, small movements)

### 2. Noise Characteristics Verification
- Check that LiDAR data has reasonable noise levels
- Verify depth camera noise increases with distance
- Confirm IMU shows appropriate bias drift over time

### 3. Geometric Accuracy
- Verify that sensor measurements match visual inspection in Gazebo
- Check that depth camera perspective matches rendering
- Validate IMU readings when robot is stationary vs. moving

## Troubleshooting

### Common Issues

**No Sensor Data:**
- Verify Gazebo plugins are properly configured in URDF/SDF
- Check ROS 2 network configuration
- Ensure sensor topics are being published

**Unrealistic Data:**
- Check sensor noise models and parameters
- Verify coordinate frame transformations
- Validate sensor mounting positions

**Performance Issues:**
- Reduce sensor update rates if needed
- Simplify noise calculations for real-time performance
- Use appropriate image resolutions

### Debugging Tips
- Use `rviz2` to visualize sensor data
- Monitor sensor update rates with `ros2 topic hz`
- Check Gazebo console for sensor plugin errors

## Next Steps

After setting up sensor simulation:
- Validate sensor data against geometric expectations
- Test sensor fusion algorithms
- Integrate with perception and navigation systems
- Document sensor characteristics for algorithm development