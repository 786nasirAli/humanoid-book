# Physics Simulation Setup Guide

## Overview
This guide details how to set up and validate the physics simulation in Gazebo for the Digital Twin project.

## Physics Parameters

### Gravity Configuration
- Standard Earth gravity: 9.81 m/s² (Z-axis negative)
- Implemented in: `simulation/gazebo/worlds/basic_physics.world`
- Validation: `simulation/gazebo/worlds/gravity_params.world`

### Other Physics Parameters
- Real-time update rate: 1000 Hz
- Maximum step size: 0.001 seconds
- Physics engine: ODE (Open Dynamics Engine)
- Solver type: Quick with 10 iterations
- Surface constraints: Contact correction velocity 100 m/s

## Setting Up the Physics Environment

### 1. Basic Physics World
The basic physics world includes:
- Accurate gravity parameters (9.81 m/s²)
- Test objects for validation
- Physics validation plugin

Location: `simulation/gazebo/worlds/basic_physics.world`

### 2. Launch Physics Simulation
To launch the physics simulation:

```bash
# Navigate to ROS 2 workspace
cd simulation/ros2

# Source ROS 2 environment
source /opt/ros/foxy/setup.bash
source install/setup.bash

# Launch physics simulation
ros2 launch robot_description robot_spawn.launch.py
```

### 3. Physics Validation Test
To run the physics validation test:

```bash
# Run the validation script
python3 simulation/ros2/src/sensor_simulator/test/physics_validation.py
```

This script will:
- Drop an object from a known height (5m)
- Measure the time to fall to the ground
- Calculate the acceleration due to gravity
- Compare against the expected value of 9.81 m/s²
- Validate the result is within 5% tolerance

## Robot Physics Model

### Inertial Properties
The robot model (`simulation/ros2/src/robot_description/urdf/basic_robot.urdf`) includes:
- Mass values for each link
- Inertia tensors for realistic physics
- Collision and visual properties
- Properly configured joints

### Physics Plugins
A physics validation plugin is included at:
`simulation/gazebo/plugins/physics_validator.cc`

This plugin:
- Validates the gravity parameter at startup
- Prints validation results to the console
- Runs continuously during simulation

## Verification Steps

1. Launch the physics world:
   ```bash
   ros2 launch gazebo_ros gazebo.launch.py world:=basic_physics.world
   ```

2. Verify gravity value in Gazebo console (should show -9.81 on Z-axis)

3. Spawn test objects and observe realistic physics behavior

4. Run the validation script to confirm physics parameters

## Troubleshooting

- If physics seem incorrect, check:
  - Gravity value in the world file
  - Maximum step size setting
  - Real-time update rate
  - Collision properties of objects

- For performance issues:
  - Reduce the maximum step size
  - Adjust the real-time update rate
  - Simplify collision meshes if needed

## Next Steps
After validating physics simulation, proceed to:
- Set up sensor simulation (LiDAR, Depth Camera, IMU)
- Configure Unity visualization
- Implement robot control systems