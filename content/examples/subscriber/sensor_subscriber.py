#!/usr/bin/env python3

"""
Humanoid robot sensor subscriber example.

This node subscribes to sensor data from a humanoid robot,
specifically focusing on joint state sensors which are critical
for humanoid robot control and awareness.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


class SensorSubscriber(Node):
    """A subscriber that processes humanoid robot sensor data."""
    
    def __init__(self):
        """Initialize the sensor subscriber."""
        super().__init__('humanoid_sensor_subscriber')
        
        # Create subscription for joint states
        self.subscription = self.create_subscription(
            JointState,
            'humanoid_sensors/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.subscription  # prevent unused variable warning
        
        # Track statistics
        self.message_count = 0
        self.last_positions = None
        
        self.get_logger().info("Sensor Subscriber initialized")

    def joint_state_callback(self, msg):
        """Process joint state data."""
        self.message_count += 1
        
        # Log basic information about the received message
        self.get_logger().info(
            f"Sensor Data #{self.message_count}: "
            f"Received {len(msg.name)} joints, "
            f"timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        )
        
        # Store current positions to compare with previous
        current_positions = msg.position
        
        if self.last_positions is not None:
            # Calculate and log some basic statistics
            position_changes = [
                abs(curr - prev) 
                for curr, prev in zip(current_positions, self.last_positions)
            ]
            
            max_change = max(position_changes) if position_changes else 0
            avg_change = sum(position_changes) / len(position_changes) if position_changes else 0
            
            self.get_logger().info(
                f"  Max position change: {max_change:.4f}, "
                f"Avg position change: {avg_change:.4f}"
            )
        
        # Log first few joint positions
        for i in range(min(5, len(msg.name))):
            self.get_logger().info(
                f"  {msg.name[i]}: pos={msg.position[i]:.3f}, "
                f"vel={msg.velocity[i]:.3f}, "
                f"eff={msg.effort[i]:.3f}"
            )
        
        # Store current positions for next comparison
        self.last_positions = current_positions.copy()
        
        # Perform any additional processing here
        # For example, publish a control command based on sensor data
        self.process_sensor_data(msg)

    def process_sensor_data(self, joint_state):
        """Process sensor data and potentially trigger other actions."""
        # Example: Check if any joint is approaching limits
        for name, pos in zip(joint_state.name, joint_state.position):
            # In a real robot, these would be actual joint limits
            if abs(pos) > 2.0:  # arbitrary limit for example
                self.get_logger().warning(f"Joint {name} position {pos:.3f} approaching limit")


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    sensor_subscriber = SensorSubscriber()
    
    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        sensor_subscriber.get_logger().info("Sensor subscriber stopped by user")
    finally:
        sensor_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()