# Gazebo-Unity Synchronization Setup Guide

This document explains how to set up and use the synchronization system between Gazebo physics simulation and Unity visualization environments.

## Overview

The synchronization system ensures that the physics simulation in Gazebo is accurately reflected in the Unity visualization environment. This is critical for providing consistent user experience when operating the digital twin simulation.

## Architecture

The synchronization system consists of:

1. **State Messages**: Custom ROS 2 messages to represent robot state
2. **Transform Synchronization Node**: Handles pose and joint state synchronization
3. **Launch File**: Orchestrates the synchronization system
4. **Validation Tools**: Ensures synchronization accuracy

## Components

### StateSync Message

The `StateSync.msg` defines the message structure for synchronizing state between environments:

```text
# Header
std_msgs/Header header

# Pose information
geometry_msgs/Pose pose

# Twist information (velocity)
geometry_msgs/Twist twist

# Timestamp for synchronization
builtin_interfaces/Time sync_time

# Additional state data as needed
float64[] joint_states
string[] joint_names
```

### Transform Synchronization Node

The `transform_sync` node performs these functions:

- Subscribes to Gazebo's odometry and joint state topics
- Broadcasts TF transforms for visualization
- Publishes synchronized joint states to Unity
- Maintains consistent timing across environments

### Launch Configuration

The `sync_simulation.launch.py` file launches all required components:

- Gazebo simulation environment
- Transform synchronization node
- Unity communication bridge

## Setup Instructions

### 1. Build the Package

```bash
cd simulation/ros2
colcon build --packages-select sensor_simulator
source install/setup.bash
```

### 2. Run Synchronized Simulation

```bash
ros2 launch sensor_simulator sync_simulation.launch.py
```

### 3. Launch Unity Environment

Make sure your Unity project is configured to connect to the ROS network and subscribe to the synchronization topics.

## Validation

Use the synchronization validation tool to verify accuracy:

```bash
ros2 run sensor_simulator sync_validation.py
```

This will continuously check the synchronization between the two environments and report any discrepancies.

## Performance Considerations

- The system maintains synchronization with &lt;5cm position accuracy
- Joint state synchronization is within 0.02 rad tolerance
- Timing synchronization is maintained with high precision
- Network latency should be minimized for best results

## Troubleshooting

### Synchronization Issues

If synchronization is not accurate:

1. Check that both Gazebo and Unity are running at consistent update rates
2. Verify network connectivity between the environments
3. Ensure ROS 2 network configuration is properly set up
4. Run the validation tool to identify specific synchronization problems

### Performance Problems

If performance is degraded:

1. Verify hardware meets minimum requirements
2. Check CPU and GPU utilization in both environments
3. Consider reducing simulation complexity if needed
4. Optimize Unity rendering settings if necessary

## Reference

For detailed information about the synchronization implementation, refer to the source code in `simulation/ros2/src/sensor_simulator/src/transform_sync.cpp`.