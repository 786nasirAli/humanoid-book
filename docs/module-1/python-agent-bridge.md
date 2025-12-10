---
title: Python Agent Bridge & Data Flow
sidebar_position: 4
description: Understanding how Python agents integrate with ROS 2 controllers
---

# Python Agent Bridge & Data Flow

## Introduction to Python Agent Integration

One of the key features of ROS 2 is its multi-language support, allowing different components of a robotic system to be implemented in different programming languages. Python is particularly popular for AI agents, data processing, and high-level robot control due to its rich ecosystem of scientific libraries and ease of use.

In this section, we'll explore how Python agents can be integrated with the ROS 2 middleware using the rclpy client library, creating a bridge between high-level decision-making and low-level control.

## Using rclpy for ROS 2 Communication

The rclpy library is the Python client library for ROS 2. It provides Python bindings for the ROS 2 client library (rcl), allowing Python programs to interact with the ROS 2 ecosystem.

### Key Features of rclpy:

1. **Node creation and management**: Create ROS 2 nodes from Python
2. **Publisher and subscriber**: Send and receive messages on topics
3. **Service clients and servers**: Request-response communication
4. **Action clients and servers**: Long-running tasks with feedback
5. **Parameter management**: Configure nodes at runtime
6. **Timers**: Execute callbacks at regular intervals
7. **Lifecycle management**: Control node state transitions

### Basic rclpy Structure:

```python
import rclpy
from rclpy.node import Node

class MyPythonAgentNode(Node):
    def __init__(self):
        super().__init__('python_agent_node')
        # Initialize publishers, subscribers, etc.
    
    # Define callbacks for handling messages/services

def main(args=None):
    rclpy.init(args=args)
    node = MyPythonAgentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Agent-Controller Communication Patterns

In a humanoid robot system, AI agents (often written in Python) typically communicate with controllers in a structured way:

1. **Perception-to-Decision Flow**: Sensor data flows from hardware sensors to Python agents for processing and decision-making
2. **Decision-to-Action Flow**: Commands flow from Python agents to controllers that execute the actions on hardware
3. **Feedback Loop**: Controller feedback flows back to agents to inform future decisions

### Example: High-Level Motion Planning

```python
# Python agent for path planning
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        
        # Publisher for planned paths
        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)
        
        # Subscriber for goal poses
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )
    
    def goal_callback(self, msg):
        # Plan a path to the goal
        planned_path = self.plan_path(msg.pose)
        
        # Publish the planned path
        self.path_publisher.publish(planned_path)
    
    def plan_path(self, goal_pose):
        # Path planning implementation
        # This could use Python libraries like numpy, scipy, etc.
        pass
```

## Quality of Service (QoS) for Humanoid Applications

Quality of Service settings determine how messages are delivered between publishers and subscribers. For humanoid robot applications, the right QoS settings are crucial for safety and performance:

- **Reliability**: For critical control commands, use `ReliabilityPolicy.RELIABLE` to ensure all messages are delivered
- **Durability**: For static data like robot descriptions, use `DurabilityPolicy.TRANSIENT_LOCAL` to keep the last message for late-joining subscribers
- **History**: For sensor streams, use `HistoryPolicy.KEEP_LAST` with appropriate depth to manage memory usage

## Real-time Data Flow Patterns in Humanoid Robots

Humanoid robots require careful consideration of timing and data flow patterns to ensure responsive and safe operation:

### Sensor-Processing-Action Loop

```
Sensors → Middleware → AI Agent → Middleware → Controllers → Actuators
  ↓         ↓            ↓          ↓            ↓           ↓
  Fast      Low          Medium     Low          Fast      Fast
  Rate      Latency      Rate       Latency      Rate      Rate
```

This loop typically has different timing requirements at each stage:
- Sensors: High frequency (100Hz+ for IMU, 30Hz+ for cameras)
- AI Processing: Moderate frequency (10-50Hz) depending on complexity
- Control Commands: High frequency (200Hz+ for joint controllers)
- Safety Monitoring: Continuous or very high frequency

### Example Integration

Here's an example of how a Python agent might bridge between perception and control in a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration

class PythonControlBridge(Node):
    def __init__(self):
        super().__init__('python_control_bridge')
        
        # Subscribe to sensor data
        self.sensor_subscriber = self.create_subscription(
            JointState,
            'humanoid_sensors/joint_states',
            self.sensor_callback,
            10
        )
        
        # Publish control commands
        self.control_publisher = self.create_publisher(
            Float64MultiArray,
            'joint_commands',
            10
        )
        
        # Timer for processing loop
        self.timer = self.create_timer(0.01, self.process_callback)  # 100Hz
        
        self.current_sensor_data = None
    
    def sensor_callback(self, msg):
        """Receive sensor data from the robot."""
        self.current_sensor_data = msg
    
    def process_callback(self):
        """Process sensor data and send control commands."""
        if self.current_sensor_data is not None:
            # Apply control algorithm
            commands = self.compute_control_commands(self.current_sensor_data)
            
            # Publish commands
            self.control_publisher.publish(commands)
    
    def compute_control_commands(self, sensor_data):
        """Compute control commands based on sensor data."""
        # This could implement balance control, inverse kinematics, etc.
        # using Python's rich ecosystem of scientific libraries
        pass
```

This example demonstrates how a Python agent can receive sensor data, process it using Python libraries, and send control commands, forming an important bridge between perception and action in the robot's "nervous system".