# Robot Models

This directory contains 3D models for the robot used in the Digital Twin simulation.

## Importing URDF Models

To import robot models from URDF files:

1. Install the Unity URDF Importer package
2. Use the URDF file from: `simulation/ros2/src/robot_description/urdf/basic_robot.urdf`
3. Follow the import steps in the Unity URDF Importer documentation

## Available Models

- Basic Robot: `basic_robot.urdf` (source location)
- Physics Robot: `robot_with_physics.urdf` (source location)

## Model Structure

The robot model should include:
- Base link (main body)
- Sensor mount
- LiDAR sensor
- Depth camera
- IMU
- Appropriate joint connections