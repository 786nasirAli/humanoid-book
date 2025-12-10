---
title: Real-time Data Flow & Middleware
sidebar_position: 3
description: Understanding how data flows through the ROS 2 middleware in humanoid robots
---

# Real-time Data Flow & Middleware

## Introduction to ROS 2 Middleware

The ROS 2 middleware is the communication layer that enables data flow between different nodes in a robotic system. It's based on DDS (Data Distribution Service), which provides a standardized, high-performance communication infrastructure.

For humanoid robots, the middleware is crucial because these robots typically have many sensors and actuators that need to coordinate in real-time. The middleware handles the complexity of routing messages between these components so that developers can focus on the robot's behavior.

## Real-time Data Flow Patterns

In a humanoid robot system, data flows in specific patterns to ensure coordinated behavior:

### 1. Sensor-Processing-Action Loop

The fundamental loop in any robot control system is:

```
Sensors → Middleware → Processing → Middleware → Actuators
```

For a humanoid robot, this might look like:
- **Sensors**: IMU, cameras, force/torque sensors, joint encoders
- **Middleware**: DDS/RMW layer handling message delivery
- **Processing**: Balance controllers, vision processing, decision-making
- **Actuators**: Joint controllers, grippers, displays

### 2. Multi-Loop Architecture

Humanoid robots often have multiple control loops running at different frequencies:

- **High-frequency loop (1-10kHz)**: Joint control, immediate safety responses
- **Medium-frequency loop (100-500Hz)**: Balance control, basic reflexes
- **Low-frequency loop (1-50Hz)**: Planning, high-level decision making

Each loop operates on different data and may involve different sets of nodes.

### 3. Feedback and Feedforward Control

Humanoid robots typically use both feedback (reactive) and feedforward (predictive) control:

- **Feedback**: Uses sensor data to correct errors (e.g., balance control using IMU)
- **Feedforward**: Anticipates required actions based on plans (e.g., sending pre-planned joint commands)

## Message Flow in Humanoid Robots

Messages flow through the ROS 2 middleware based on topics, services, and actions. Understanding these patterns is key to designing efficient humanoid systems:

### Topic-based Communication (Asynchronous)

Most sensor data and control commands flow through topics using the publish-subscribe pattern:

```
Sensor Node → Topic → Processing Node
                       ↓
                       → Control Node
```

This pattern allows multiple nodes to receive the same data simultaneously, which is common in humanoid robots where the same sensor data might be used for perception, control, and logging.

### Service-based Communication (Synchronous)

Configuration and one-time requests typically use services:

```
Requesting Node → Service → Server Node
        ↑                      ↓
        ← Response ←−−−−−−−−−−−
```

This pattern is useful for tasks like calibrating sensors, changing robot modes, or requesting specific robot behaviors.

### Action-based Communication (Asynchronous with Feedback)

Long-running tasks with progress feedback use actions:

```
Action Client → Goal → Action Server
     ↑                    ↓
     ← Feedback −−−−−−−−−−
     ↑                    ↓
     ← Result (when done) −−
```

This is ideal for navigation, manipulation tasks, and complex behaviors that take time to complete and may need to be canceled.

## Quality of Service (QoS) Settings

QoS settings determine how messages are delivered and are critical for real-time applications like humanoid robots:

### Reliability Policy
- **RELIABLE**: All messages are guaranteed to be delivered, potentially with retries
  - Use for critical control commands and safety-related data
- **BEST_EFFORT**: Messages may be lost but with lower latency
  - Use for sensor data where occasional loss is acceptable

### Durability Policy
- **TRANSIENT_LOCAL**: Late-joining subscribers receive the last published value
  - Use for static data like robot descriptions
- **VOLATILE**: No attempt to store values for late joiners
  - Use for all other data

### History Policy
- **KEEP_LAST**: Store the most recent N messages
  - Use for sensor streams and command queues
- **KEEP_ALL**: Store all messages (be careful with memory)

### Depth Parameter
- The number of messages to store in the history
- Higher values allow for more tolerance to processing delays
- Lower values reduce memory usage and latency

## Example QoS Configuration for Humanoid Robots

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# For critical control commands
control_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# For high-frequency sensor data
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)

# For static robot description
robot_description_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

## Quality of Service (QoS) Settings for Humanoid Applications

Quality of Service (QoS) settings are critical for humanoid robots to ensure proper timing and reliability of communication. Here are guidelines for common humanoid robot applications:

### Control Command Messages
For messages commanding joint positions, velocities, or efforts:
```
Reliability: RELIABLE - All command messages must be delivered
Durability: VOLATILE - Commands are only valid for current time window
History: KEEP_LAST - Only most recent command matters
Depth: 1-5 - Small queue to handle brief timing mismatches
```

This ensures that control commands are delivered reliably but don't accumulate to cause delayed responses.

### Sensor Data Streams
For messages from sensors like cameras, IMUs, or joint encoders:
```
Reliability: BEST_EFFORT or RELIABLE depending on criticality
Durability: VOLATILE - Only current sensor data is relevant
History: KEEP_LAST - Only recent sensor readings needed
Depth: 1-10 - Larger for high-frequency sensors to handle processing delays
```

For safety-critical sensors (like collision detection), use RELIABLE. For perception, BEST_EFFORT is often acceptable.

### Configuration and State Messages
For robot configuration, mode changes, or global state:
```
Reliability: RELIABLE - Configuration must be delivered
Durability: TRANSIENT_LOCAL - New nodes need current configuration
History: KEEP_LAST - Only most recent configuration matters
Depth: 1
```

### Planning and High-Level Commands
For navigation goals, manipulation plans, or task specifications:
```
Reliability: RELIABLE - Plans must be delivered completely
Durability: VOLATILE - Plans are valid only for current planning cycle
History: KEEP_ALL or KEEP_LAST depending on need
Depth: Variable - For Actions, use sufficient depth for entire plan
```

## Timing and Synchronization

In humanoid robots, timing is critical for stable control and safe operation:

### Time Synchronization
- Use ROS time (`ClockType.ROS_TIME`) for simulation or replay
- Use system time (`ClockType.SYSTEM_TIME`) for real robot operation
- Consider clock drift and adjust accordingly for precise control

### Rate Limiting
- Control loop frequencies should be appropriate for the task
- Use `Rate` objects to maintain consistent update rates
- Monitor timing to detect performance issues

## Performance Considerations

For real-time humanoid applications:

- Minimize message sizes to reduce latency
- Use appropriate QoS settings for each data type
- Consider using Intra-Process Communication (IPC) for nodes in the same process
- Monitor CPU and memory usage to avoid performance degradation
- Use real-time kernel if strict timing is required