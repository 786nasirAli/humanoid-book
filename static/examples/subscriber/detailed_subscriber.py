#!/usr/bin/env python3

"""
Detailed ROS 2 subscriber example for humanoid robot.

This node subscribes to multiple topics and processes different types of messages.
It demonstrates advanced subscriber concepts in ROS 2 including custom
message types, QoS settings, and message filtering for humanoid robots.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy


class DetailedSubscriber(Node):
    """A detailed subscriber node demonstrating advanced ROS 2 concepts."""

    def __init__(self):
        """Initialize the detailed subscriber node."""
        super().__init__('detailed_subscriber')

        # Create a subscription for joint states with custom QoS settings
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            qos_profile
        )
        
        # Create a subscription for status messages
        self.status_subscription = self.create_subscription(
            String,
            'status',
            self.status_callback,
            10
        )

        # Prevent unused variable warnings
        self.joint_subscription  # type: ignore
        self.status_subscription  # type: ignore

        self.get_logger().info('Detailed Subscriber node initialized')

    def joint_states_callback(self, msg):
        """Callback method for processing joint state messages."""
        # Log information about received joint states
        self.get_logger().info(f'Received joint states for {len(msg.name)} joints')
        
        # Process each joint
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                pos = msg.position[i]
                if i < len(msg.velocity):
                    vel = msg.velocity[i]
                else:
                    vel = 0.0
                
                if i < len(msg.effort):
                    eff = msg.effort[i]
                else:
                    eff = 0.0
                
                self.get_logger().info(f'  Joint: {name}, Position: {pos:.3f}, Velocity: {vel:.3f}, Effort: {eff:.3f}')
        
        # Additional processing could go here
        # For example: safety checks, data filtering, etc.

    def status_callback(self, msg):
        """Callback method for processing status messages."""
        # Log the received status message
        self.get_logger().info(f'Status received: "{msg.data}"')


def main(args=None):
    """Main function that initializes the node and starts spinning."""
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the detailed subscriber node
    detailed_subscriber = DetailedSubscriber()

    # Start spinning to execute callbacks
    try:
        rclpy.spin(detailed_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
        detailed_subscriber.destroy_node()

        # Shutdown the rclpy library
        rclpy.shutdown()


if __name__ == '__main__':
    main()