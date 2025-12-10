#!/usr/bin/env python3

"""
Robot controller subscriber example.

This node simulates a controller that receives commands from 
a Python agent and executes them on a humanoid robot. This 
demonstrates the bridge between the ROS 2 communication layer 
and the actual robot control system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time


class ControllerSubscriber(Node):
    """
    A controller that receives commands and simulates their execution.
    
    This example simulates a controller that receives joint commands
    from an AI agent and simulates applying them to the robot's joints.
    """
    
    def __init__(self):
        """Initialize the controller subscriber."""
        super().__init__('controller_subscriber')
        
        # Subscriber for joint commands from the agent
        self.command_subscriber = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.command_callback,
            10
        )
        
        # Publisher for joint states (simulated feedback)
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'humanoid_sensors/joint_states',
            10
        )
        
        # Initialize joint data
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]
        
        self.current_positions = [0.0] * len(self.joint_names)
        self.target_positions = [0.0] * len(self.joint_names)
        self.command_received_time = self.get_clock().now()
        
        # Timer for simulating control loop and state publishing
        timer_period = 0.01  # 100 Hz
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info(f"Controller Subscriber initialized for {len(self.joint_names)} joints")

    def command_callback(self, msg):
        """Receive and process joint commands."""
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warning(
                f"Command has {len(msg.data)} values but we have {len(self.joint_names)} joints"
            )
            return
        
        # Store command as target positions
        self.target_positions = list(msg.data)
        self.command_received_time = self.get_clock().now()
        
        self.get_logger().info(
            f"Received command: "
            f"Left hip: {self.target_positions[0]:.3f}, "
            f"Right hip: {self.target_positions[3]:.3f}, "
            f"Left shoulder: {self.target_positions[6]:.3f}"
        )

    def control_loop(self):
        """Simulate the control loop that applies commands to joints."""
        # Simulate joint movement toward target positions
        # (In a real robot, this would interface with hardware)
        for i in range(len(self.joint_names)):
            # Simple first-order approach to target
            # In a real system, this would use PID control or more sophisticated methods
            diff = self.target_positions[i] - self.current_positions[i]
            step = diff * 0.1  # 10% of the difference per cycle
            
            # Add some simulated dynamics
            self.current_positions[i] += step
        
        # Publish current joint states
        self.publish_joint_states()
        
        # Log position updates periodically
        if self.get_clock().now().nanoseconds % 2000000000 < 10000000:  # Every ~2 seconds
            self.get_logger().info(
                f"Current positions: "
                f"Left hip: {self.current_positions[0]:.3f}, "
                f"Right hip: {self.current_positions[3]:.3f}, "
                f"Left shoulder: {self.current_positions[6]:.3f}"
            )

    def publish_joint_states(self):
        """Publish current joint states to simulate sensor feedback."""
        msg = JointState()
        
        # Set header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Set joint data
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = [0.0] * len(self.joint_names)  # Simplified
        msg.effort = [0.0] * len(self.joint_names)    # Simplified
        
        self.joint_state_publisher.publish(msg)


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    controller_subscriber = ControllerSubscriber()
    
    try:
        rclpy.spin(controller_subscriber)
    except KeyboardInterrupt:
        controller_subscriber.get_logger().info("Controller subscriber stopped by user")
    finally:
        controller_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()