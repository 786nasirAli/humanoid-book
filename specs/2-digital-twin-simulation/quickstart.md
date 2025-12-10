# Quickstart Guide: Digital Twin Simulation Project

## Overview
This guide will help you set up and run the Digital Twin Simulation Project using Gazebo and Unity.

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

# Create a workspace
mkdir -p ~/digital-twin-sim/src
cd ~/digital-twin-sim

# Copy ROS 2 packages from the repository
cp -r [repository-path]/ros2/* src/

# Build the workspace
colcon build
source install/setup.bash
```

### 3. Configure Gazebo Environment
```bash
# Navigate to the gazebo directory
cd [repository-path]/simulation/gazebo

# Launch the first demo environment
ros2 launch gazebo_ros empty_world.launch.py world:=worlds/demo1.world
```

### 4. Set up Unity Project
```bash
# Open Unity Hub
# Click "Add" and navigate to [repository-path]/simulation/unity
# Open the project in Unity 2021.3 LTS
```

## Running the Simulation

### Physics Simulation (Gazebo)
1. Source your ROS 2 environment:
   ```bash
   source ~/digital-twin-sim/install/setup.bash
   ```

2. Launch a sample environment:
   ```bash
   ros2 launch gazebo_ros empty_world.launch.py world:=worlds/basic_physics.world
   ```

3. Spawn a robot model:
   ```bash
   ros2 run gazebo_ros spawn_entity.py -entity my_robot -file [path-to-urdf]/robot.urdf
   ```

### Visualization (Unity)
1. In Unity Editor, open the main scene from `Assets/Scenes/MainScene.unity`
2. Press Play to start the visualization
3. The scene should connect to the ROS 2 network and mirror the Gazebo environment

### Sensor Simulation
1. Enable sensors on the robot:
   ```bash
   ros2 run sensor_simulator enable_sensors --robot my_robot
   ```

2. Monitor sensor data:
   ```bash
   ros2 topic echo /my_robot/lidar_scan sensor_msgs/msg/LaserScan
   ros2 topic echo /my_robot/depth_camera/image_raw sensor_msgs/msg/Image
   ros2 topic echo /my_robot/imu/data sensor_msgs/msg/Imu
   ```

## Troubleshooting

### Common Issues
- **Gazebo not starting**: Ensure graphics drivers are properly installed
- **Unity-ROS connection failing**: Check ROS_IP and ROS_MASTER_URI environment variables
- **Low performance**: Verify NVIDIA RTX drivers are up to date

### Performance Tips
- Close unnecessary applications to free up GPU resources
- Adjust Unity quality settings for your hardware (Edit > Project Settings > Quality)
- Consider using smaller meshes for complex objects in Gazebo

## Verification Steps

1. **Physics Simulation Test**:
   - Launch the physics test world
   - Verify gravity acceleration is approximately 9.81 m/s²
   - Check collision detection between objects

2. **Visualization Test**:
   - Launch Unity scene
   - Verify 30+ FPS performance
   - Confirm environment mirrors Gazebo scene

3. **Sensor Simulation Test**:
   - Activate all sensors on the test robot
   - Verify realistic sensor outputs with appropriate noise profiles
   - Confirm data formats match ROS 2 message types

## Next Steps

- Follow the detailed tutorials in the `docs/tutorials/` directory
- Explore different environments in the `simulation/gazebo/worlds/` directory
- Experiment with different robot models in the `simulation/gazebo/models/` directory