---
title: Troubleshooting Common Issues
sidebar_position: 7
description: Common issues and solutions when working with ROS 2 for humanoid robots
---

# Troubleshooting Common ROS 2 Issues

This section covers common issues when working with ROS 2 for humanoid robotics and their solutions.

## Common Installation Issues

### ROS 2 Installation Problems
- **Issue**: Error during ROS 2 installation
- **Solution**: Ensure your Ubuntu version matches the supported version for your ROS 2 distribution. Check the installation guide at https://docs.ros.org/en/humble/Installation.html

### Package Not Found
- **Issue**: `apt` cannot find ROS 2 packages
- **Solution**: Make sure you've properly added the ROS 2 repository and updated your package list:
  ```bash
  sudo apt update
  ```

## Common Runtime Issues

### Nodes Cannot Communicate
- **Issue**: Nodes cannot send/receive messages to/from each other
- **Solution**: Check that nodes are on the same ROS domain ID. You can set this with:
  ```bash
  export ROS_DOMAIN_ID=your_domain_id
  ```
  Nodes with different domain IDs cannot communicate.

### Topic/Service Names Don't Match
- **Issue**: Publishers and subscribers aren't connecting
- **Solution**: Verify that topic names are exactly the same using:
  ```bash
  ros2 topic list
  ros2 topic info <topic_name>
  ```

### Quality of Service Mismatch
- **Issue**: Messages aren't being delivered despite topics being connected
- **Solution**: Ensure that QoS policies match between publishers and subscribers. For example, a RELIABLE publisher won't deliver to a BEST_EFFORT subscriber if there are network issues.

## Performance Issues

### High CPU Usage
- **Issue**: ROS 2 nodes consuming excessive CPU
- **Solution**: Check timer frequencies - extremely high frequency timers can overload the system. Also check for infinite loops or unoptimized algorithms.

### Memory Leaks
- **Issue**: Memory usage increasing over time
- **Solution**: Ensure nodes are properly cleaned up and use `node.destroy_node()` before shutting down.

### Latency Issues
- **Issue**: High delay between publishing and receiving messages
- **Solution**: Check network configuration, ensure QoS settings are appropriate, and consider using intra-process communication for nodes that run in the same process.

## Humanoid Robot Specific Issues

### Joint Limit Violations
- **Issue**: Joint commands exceed physical limits
- **Solution**: Set proper joint limits in your URDF and implement software limits in your controllers.

### Balance Problems
- **Issue**: Humanoid robot falls over
- **Solution**: Check IMU calibration, ensure control loop runs at appropriate frequency (typically 100Hz+ for balance), and verify control algorithms are properly tuned.

### Sensor Noise
- **Issue**: Noisy sensor data affecting robot behavior
- **Solution**: Implement appropriate filtering (Kalman filters, moving averages) and check physical sensor mounting.

## Debugging Tips

### Using ROS 2 Tools
- Check active nodes: `ros2 node list`
- Check active topics: `ros2 topic list`
- Monitor messages: `ros2 topic echo <topic_name>`
- Check node connections: `ros2 node info <node_name>`

### Logging
- Use appropriate log levels: DEBUG, INFO, WARN, ERROR
- Include relevant values in logs: joint positions, command values, error states
- Be careful not to overload the system with too much logging

### Simulation First
- Always test new code in simulation before running on real hardware
- Use Gazebo or similar simulators to verify functionality
- Gradually increase complexity and range of motion when testing on hardware