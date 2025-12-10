# Digital Twin Simulation Setup Guide

## Overview
This guide will help you set up the Digital Twin Simulation environment with Gazebo and Unity.

## Prerequisites
- Linux (Ubuntu 20.04 LTS recommended) or Windows 10/11
- NVIDIA RTX-enabled workstation or equivalent cloud instance
- ROS 2 Foxy or Fortress installed
- Unity Hub with Unity 2021.3 LTS installed
- Gazebo 11+ installed
- Git for version control

## Installation Steps

### 1. Clone the Repository
```bash
git clone [repository-url]
cd [repository-name]
```

### 2. Set up ROS 2 Environment
```bash
# Source ROS 2 environment
source /opt/ros/foxy/setup.bash  # or fortress

# Navigate to the ROS 2 workspace
cd simulation/ros2

# Build the workspace
colcon build
source install/setup.bash
```

### 3. Configure Gazebo Environment
```bash
# Launch the basic physics simulation
ros2 launch robot_description robot_spawn.launch.py

# Or launch the advanced physics simulation
ros2 launch gazebo_ros gazebo.launch.py world:=basic_physics.world

# Or launch the complex environment simulation
ros2 launch gazebo_ros gazebo.launch.py world:=advanced_physics.world
```

### 4. Set up Unity Project
```bash
# Open Unity Hub
# Click "Add" and navigate to simulation/unity
# Open the project in Unity 2021.3 LTS
```

## Configuration Details

### Physics Parameters
- Gravity: 9.81 m/s² (standard Earth gravity)
- Real-time update rate: 1000 Hz
- Max step size: 0.001 seconds
- Physics engine: ODE (Open Dynamics Engine)

### Robot Model
- Base link: 0.5m x 0.5m x 0.2m box
- Sensors mounted on sensor mount:
  - LiDAR: Mounted on top
  - Depth camera: Mounted on front
  - IMU: Mounted internally

### Available Simulation Environments
- **Basic Physics World**: Simple environment for fundamental physics validation
  - File: `simulation/gazebo/worlds/basic_physics.world`
  - Parameters: Accurate gravity, collision test objects
- **Advanced Physics World**: Complex environment with multiple objects and scenarios
  - File: `simulation/gazebo/worlds/advanced_physics.world`
  - Parameters: Ramps, obstacles, multi-body objects

### Unity Scenes
- **Gazebo Mirror Scene**: Mirrors the Gazebo environment for visualization
  - File: `simulation/unity/Assets/Scenes/gazebo_mirror.unity`
  - Features: Camera controls, interaction system, ROS communication
- **Advanced Scenario Scene**: Complex environment with multiple obstacles
  - File: `simulation/unity/Assets/Scenes/advanced_scenario.unity`
  - Features: Inclined planes, obstacles, advanced interaction

## Verification Steps

1. Launch the robot in Gazebo
2. Verify physics parameters are correct
3. Check that sensors are properly attached to the robot
4. Validate that ROS 2 communication is working
5. Test Unity visualization synchronization
6. Run sensor validation tests

## Troubleshooting

- If Gazebo doesn't start, ensure graphics drivers are properly installed
- If ROS 2 nodes fail to communicate, check network configuration
- For performance issues, verify hardware meets minimum requirements
- If Unity-ROS connection fails, check ROS bridge server
- For synchronization issues, verify clock settings and timing parameters