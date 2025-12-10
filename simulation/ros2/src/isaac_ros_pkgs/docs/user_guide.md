# AI-Robot Brain User Guide

## Overview

This guide explains how to operate the AI-Robot Brain system for the humanoid robot. The system provides advanced perception and navigation capabilities using NVIDIA Isaac technologies.

## Prerequisites

Before using the AI-Robot Brain system, ensure you have:

- A humanoid robot platform with necessary sensors (stereo cameras, IMU, etc.)
- NVIDIA GPU with appropriate drivers
- ROS2 Humble Hawksbill installed
- NVIDIA Isaac ROS packages installed
- Nav2 navigation stack installed

## System Setup

### 1. Environment Configuration
Set up your environment variables for Isaac Sim and ROS2:

```bash
source /opt/ros/humble/setup.bash
export ISAAC_SIM_PATH=/path/to/isaac-sim
export NVIDIA_VISIBLE_DEVICES=all
```

### 2. Launch the AI-Robot Brain System
Use the launch file to start all necessary components:

```bash
ros2 launch isaac_ros_pkgs bipedal_ai_brain.launch.xml
```

This will start:
- Isaac ROS perception nodes
- Nav2 navigation stack
- Robot state publisher
- (If in simulation) Isaac Sim bridge

## Operating the System

### 1. Perception System
The perception system will automatically start processing sensor data:
- VSLAM will generate pose estimates
- Environment mapping will update continuously
- Obstacle detection will run in real-time

Monitor the system status using:
```bash
# View camera feeds
ros2 run rqt_image_view rqt_image_view

# View robot pose and map
ros2 run rviz2 rviz2

# Check system status
ros2 topic list
```

### 2. Navigation System
To send navigation goals to the bipedal humanoid:

1. Open RViz2 interface
2. Set initial pose if localization is lost
3. Use "Nav2 Goal" tool to set navigation destination
4. Monitor progress in RViz2

Command line alternative:
```bash
# Send a goal via command line
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    position: {x: 1.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### 3. Simulation Mode
When running in Isaac Sim:
- Use the Isaac Sim interface to control simulation parameters
- Adjust lighting, physics, and scenario settings
- Monitor synthetic data generation

## Configuration

### Adjusting Navigation Parameters
Edit the Nav2 configuration file to adjust navigation behavior:
`simulation/ros2/src/nav2_bipedal_configs/bipedal_nav2_params.yaml`

Key parameters for bipedal navigation:
- Footprint: Adjust according to your robot's base
- Velocity limits: Set appropriate for stable bipedal locomotion
- Tolerances: Configure position and orientation tolerances

### Performance Tuning
For optimal performance:
- Ensure GPU acceleration is enabled
- Adjust processing rates based on available computational resources
- Configure sensor settings for optimal frame rates

## Troubleshooting

### Common Issues:

1. **Localization Failure**
   - Verify robot is in mapped area
   - Reinitialize pose if needed
   - Check sensor data availability

2. **Navigation Not Starting**
   - Verify map is loaded
   - Check if robot is in collision space
   - Confirm Nav2 server is running

3. **Performance Issues**
   - Monitor system resources
   - Reduce sensor data rates if needed
   - Verify GPU acceleration is working

### Debugging Commands:
```bash
# View system status
ros2 lifecycle list
# Check topics
ros2 topic echo /pose
# View logs
ros2 launch ros2 launch
```

## Performance Monitoring

Monitor key performance metrics:
- VSLAM processing frame rate
- Localization accuracy
- Path planning response time
- Navigation success rate

The system logs these metrics to standard ROS2 log files and can be visualized using ROS2 tools.

## Safety Considerations

- Always maintain a safe distance during autonomous operation
- Have an emergency stop mechanism available
- Verify navigation paths before execution
- Monitor robot balance during movement
- Use simulation for initial testing

## Advanced Usage

### Synthetic Data Generation
To generate synthetic training data:
1. Configure the pipeline in `simulation/ros2/src/synthetic_data_pipeline/pipeline_config.yaml`
2. Run the simulation scenario in Isaac Sim
3. Monitor data generation progress

### Custom Navigation Behaviors
The system supports custom navigation behaviors for specialized tasks:
1. Create custom behavior trees
2. Configure in the Nav2 parameter file
3. Test in simulation before deployment

## Support and Resources

For additional support:
- Check the technical documentation
- Report issues via the project issue tracker
- Consult the ROS2 and Isaac ROS community forums