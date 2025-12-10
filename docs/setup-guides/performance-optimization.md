# Performance Optimization Guide

## Overview
This guide provides optimization strategies to ensure the Digital Twin simulation achieves optimal performance, particularly maintaining 30+ FPS in Unity as specified in the project requirements.

## Performance Targets

- **Unity Rendering**: ≥30 FPS on RTX-enabled hardware
- **Gazebo Physics**: 1000 Hz update rate
- **ROS Communication**: Minimal latency between environments
- **Synchronization**: ≤50ms delay between Gazebo and Unity states

## Unity Performance Optimization

### 1. Quality Settings
The project includes optimized quality settings for different hardware configurations:

- **Low**: For basic hardware (minimum requirements)
- **Medium**: Balanced performance and quality
- **High**: Default setting for RTX-enabled workstations
- **Very High/Ultra**: For high-end systems

### 2. Renderer Optimization
- Use occlusion culling to avoid rendering hidden objects
- Implement Level of Detail (LOD) systems for distant objects
- Minimize draw calls by batching similar objects
- Use texture atlasing to reduce texture switches

### 3. Physics Optimization in Unity
- Limit physics update rate to match Gazebo (60-100 Hz vs default 500 Hz)
- Optimize collider geometry (simplify where possible)
- Control the number of simultaneous physics interactions

### 4. Lighting and Shading
- Use baked lighting where possible instead of real-time shadows
- Limit the number of real-time lights
- Use light probes for complex lighting scenarios

### 5. Asset Optimization
- Compress textures appropriately (ASTC for mobile, DXT for desktop)
- Limit polygon count on 3D models
- Use object pooling for frequently instantiated objects
- Remove unused assets to reduce memory usage

## Gazebo Performance Optimization

### 1. Physics Engine Settings
In `simulation/gazebo/worlds/*.world` files:

```xml
&lt;physics name="default_physics" type="ode"&gt;
  &lt;max_step_size&gt;0.001&lt;/max_step_size&gt;
  &lt;real_time_factor&gt;1&lt;/real_time_factor&gt;
  &lt;real_time_update_rate&gt;1000&lt;/real_time_update_rate&gt;
  &lt;gravity&gt;0 0 -9.81&lt;/gravity&gt;
&lt;/physics&gt;
```

### 2. Model Complexity
- Simplify collision meshes where visual detail isn't needed
- Use compound collision shapes instead of trimesh where possible
- Limit the number of joints in complex models

### 3. Sensor Optimization
Each sensor includes performance considerations:
- **LiDAR**: 720 samples at 10Hz (balance between quality and performance)
- **Depth Camera**: 640x480 resolution at 30Hz
- **IMU**: 100Hz update rate optimized for responsiveness

## Cross-Environment Synchronization Performance

### 1. Network Optimization
- Use localhost communication when possible
- Optimize ROS message sizes
- Implement selective synchronization only for necessary objects

### 2. Timing Synchronization
- Align update rates between environments
- Implement interpolation for smoother visualization
- Use timestamp validation to handle network delays

## Monitoring and Profiling

### Unity Profiling
1. Use Unity Profiler to monitor CPU/GPU usage
2. Check frame timing for rendering bottlenecks
3. Monitor memory allocation patterns

### Gazebo Profiling
1. Use Gazebo's built-in performance metrics
2. Monitor physics update timing
3. Check sensor generation rates

### ROS Communication Monitoring
```bash
# Monitor communication rates
rostopic hz /gazebo/robot_pose
rostopic hz /unity/robot_pose

# Monitor bandwidth usage
rostopic bw /sensor_data
```

## Troubleshooting Performance Issues

### Low FPS in Unity
1. Check Quality Settings in Edit > Project Settings > Quality
2. Verify graphics drivers are up to date
3. Monitor system resources (GPU, RAM usage)
4. Reduce scene complexity temporarily to isolate issues

### Physics Instability in Gazebo
1. Verify step size and update rate settings
2. Check for model penetration or instability
3. Adjust solver parameters if needed

### Synchronization Lag
1. Check network connectivity between environments
2. Verify ROS bridge performance
3. Monitor clock synchronization between systems

## Performance Testing Procedures

### Automated Performance Test
Run the performance test script to validate FPS targets:
```bash
# From your ROS workspace
ros2 run sensor_simulator performance_test
```

### Manual Performance Validation
1. Launch the simulation with the basic physics world
2. Monitor FPS in Unity (enable Stats in Game view)
3. Run for 5 minutes continuous operation
4. Record minimum, average, and maximum FPS
5. Verify no significant drops below 30 FPS

### Benchmarking
The system should maintain:
- Unity: ≥30 FPS average during simulation
- Physics: ≤2% real-time factor variance
- Sensor data: ≤50ms delay in visualization
- ROS communication: ≤10ms message delay

## Hardware Recommendations

### Minimum Requirements
- CPU: Quad-core processor (Intel i5 / AMD Ryzen 5)
- GPU: Equivalent to GTX 1060 / RTX 2060
- RAM: 8GB
- OS: Ubuntu 20.04 LTS or Windows 10/11

### Recommended Requirements
- CPU: Hexa-core or higher (Intel i7 / AMD Ryzen 7)
- GPU: NVIDIA RTX series (RTX 2070S or better)
- RAM: 16GB or more
- Storage: SSD for faster asset loading

## Conclusion

Following these optimization strategies should ensure the Digital Twin simulation meets performance requirements. Regular monitoring and adjustment of settings may be needed based on specific use cases and hardware configurations.

Always validate performance after implementing new features or making significant changes to the simulation environment.