# Unity Visualization Setup Guide

## Overview
This guide details how to set up the Unity visualization environment that mirrors the Gazebo simulation for the Digital Twin project.

## Prerequisites

- Unity Hub installed
- Unity 2021.3 LTS installed (matching the project requirement)
- ROS# package (for ROS communication)
- Python and ROS 2 running with the robot simulation

## Setting Up the Unity Project

### 1. Opening the Project
1. Launch Unity Hub
2. Click "Add" and navigate to `simulation/unity`
3. Select the folder and open it as a Unity project
4. Wait for Unity to import assets and build the project

### 2. Project Structure
The Unity project contains:
- `Assets/Scenes/` - Contains the main scene that mirrors Gazebo
- `Assets/Scripts/` - Contains all custom scripts for the project
- `Assets/Models/` - Contains 3D models for the robot and environment
- `Assets/Settings/` - Contains project settings for performance

### 3. Scene Configuration
The main scene (`gazebo_mirror.unity`) includes:
- Main camera with navigation controls
- Directional light matching Gazebo's sun
- Ground plane
- Robot model placeholder

## Camera Navigation Controls

The camera system supports:
- **WASD/Arrow Keys**: Move camera horizontally
- **Right Mouse Button + Mouse Move**: Rotate camera view
- **Mouse Wheel**: Zoom in/out
- **Limited zoom**: Prevents getting too close to objects

## Human-Robot Interaction System

### Interaction Controls
- **E Key**: Interact with highlighted objects
- **Mouse hover**: Highlights interactable objects with yellow outline
- **Interaction distance**: Limited to 5 units

### Interaction Types
The system recognizes different object types:
- **Robot** (`Robot` tag): Enables robot control or inspection
- **Sensor** (`Sensor` tag): Activates or configures sensors
- **Environment** (`Environment` tag): Allows environment modifications
- **General**: Standard interaction for other objects

## ROS Communication Setup

### Connection Configuration
1. Update the ROS bridge server URL in `RosCommunication.cs`
   - Default: `ws://192.168.1.1:9090`
   - Should match your ROS bridge server address

2. Ensure ROS bridge is running:
   ```bash
   # Terminal 1 - Start ROS bridge
   roslaunch rosbridge_server rosbridge_websocket.launch
   
   # Terminal 2 - Start your robot simulation in Gazebo
   ros2 launch robot_description robot_spawn.launch.py
   ```

### Available Topics
- `/robot_pose` - Robot position and orientation from Gazebo
- `/unity_robot_pose` - Robot position and orientation from Unity
- `/gazebo_odom` - Odometry data for physics synchronization

## Physics Synchronization

### Synchronization Rate
- Default sync rate: 60 times per second
- Adjust in `PhysicsSync.cs` if needed

### Object Management
- Objects in Gazebo are automatically registered for synchronization
- Unity objects are updated to match Gazebo positions and orientations
- Support for object creation/destruction synchronization

## Performance Optimization

### Quality Settings
Quality settings are configured to achieve >30 FPS:

- **Low Setting**: For lower-end hardware
- **Medium Setting**: Balanced performance and quality
- **High Setting**: For recommended hardware (RTX-enabled workstation)

### Recommended Hardware
- NVIDIA RTX-enabled workstation or equivalent
- At least 8GB RAM
- Multi-core processor

## Running the Visualization

### Steps to Launch
1. Ensure Gazebo simulation is running with ROS bridge
2. Open the Unity project in Unity Hub
3. Open `Assets/Scenes/gazebo_mirror.unity`
4. Press Play in Unity Editor

### Verification
- Camera navigation should work with WASD and mouse controls
- Robot should appear in the scene
- ROS connection indicator should show "Connected"
- Robot movement in Gazebo should be reflected in Unity

## Troubleshooting

### Common Issues

**ROS Connection Issues:**
- Verify ROS bridge server is running at the configured address
- Check firewall settings for port 9090
- Ensure ROS_IP and ROS_MASTER_URI are properly configured

**Performance Issues:**
- Reduce quality settings in `Assets/Settings/QualitySettings.asset`
- Close unnecessary applications
- Verify graphics drivers are up to date

**Synchronization Issues:**
- Check that both Gazebo and Unity are using the same coordinate system
- Verify that the robot name in topics matches between systems
- Ensure both systems are publishing at consistent rates

### Debugging
- Enable Unity's "Stats" view to monitor performance
- Use ROS tools like `rostopic echo` to verify topic data
- Check Unity console for errors or warnings

## Next Steps

After setting up Unity visualization:
- Test interaction with robot and environment
- Verify physics synchronization with Gazebo
- Test sensor data visualization (to be implemented in next phase)
- Optimize visualization for target hardware