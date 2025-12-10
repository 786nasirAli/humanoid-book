# Data Model: Digital Twin Simulation Project

## Overview
Data model for the Digital Twin Simulation Project using Gazebo and Unity.

## Core Entities

### GazeboEnvironment
- **id**: Unique identifier for the environment
- **name**: Display name of the environment
- **description**: Text description of the environment
- **physics_parameters**: Object containing gravity, friction coefficients, etc.
- **models**: Collection of robot and object models in the environment
- **world_file**: Path to the Gazebo world file
- **lighting_settings**: Configuration for environmental lighting
- **collision_settings**: Configuration for collision detection parameters

### UnityScene
- **id**: Unique identifier for the scene
- **name**: Display name of the scene
- **description**: Text description of the scene
- **camera_settings**: Configuration for camera position, field of view, etc.
- **rendering_settings**: Configuration for lighting, shadows, post-processing
- **linked_environment**: Reference to corresponding GazeboEnvironment
- **assets**: Collection of 3D assets used in the scene

### RobotModel
- **id**: Unique identifier for the robot
- **name**: Display name of the robot
- **description**: Text description of the robot
- **urdf_path**: Path to the URDF file describing the robot
- **joint_configurations**: Collection of joint positions and constraints
- **link_properties**: Properties for each link (mass, geometry, etc.)
- **sensor_mounts**: Positions and types of sensors mounted on the robot
- **kinematic_properties**: Forward/inverse kinematic properties

### SensorData
- **id**: Unique identifier for the sensor data instance
- **sensor_type**: Type of sensor (LiDAR, Depth Camera, IMU, etc.)
- **timestamp**: Time when the data was captured
- **sensor_id**: Reference to the sensor that generated the data
- **data_format**: Format of the sensor data (point cloud, image, quaternion, etc.)
- **raw_data**: Raw sensor output
- **processed_data**: Processed sensor output with noise applied
- **accuracy_metrics**: Accuracy measures and noise characteristics

## Relationships
- GazeboEnvironment 1---* RobotModel (Environment contains multiple robot models)
- UnityScene 1---1 GazeboEnvironment (Scene mirrors single environment)
- RobotModel 1---* SensorData (Robot has multiple sensors generating data)
- GazeboEnvironment 1---* SensorData (Environment contains multiple sensor data streams)

## State Transitions

### RobotModel States
- **DESIGN**: Model is being designed, URDF is being created
- **CONFIGURED**: Model is configured with sensors and kinematic properties
- **VALIDATED**: Model has been validated in simulation
- **DEPLOYED**: Model is ready for use in environments

### SimulationSession States
- **SETUP**: Environment and robot are being configured
- **RUNNING**: Simulation is actively running
- **PAUSED**: Simulation is temporarily stopped
- **COMPLETED**: Simulation session has ended
- **ANALYZED**: Results have been analyzed and documented