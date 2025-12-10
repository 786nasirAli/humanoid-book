#!/usr/bin/env python3

"""
Controller example that subscribes to commands from Python agents.

This node demonstrates how a ROS 2 controller subscribes to commands from
Python agents and processes them for a humanoid robot, using appropriate
message types and QoS settings.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


class ControllerSubscriber(Node):
    """A controller that subscribes to commands from Python agents."""

    def __init__(self):
        """Initialize the controller subscriber node."""
        super().__init__('controller_subscriber')
        
        # Create QoS profile for real-time control
        qos_profile = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Subscription for joint commands
        self.joint_cmd_subscription = self.create_subscription(
            JointState,
            '/joint_commands',
            self.joint_cmd_callback,
            qos_profile
        )
        
        # Subscription for base movement commands
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        # Subscription for agent status
        self.status_subscription = self.create_subscription(
            String,
            '/agent_status',
            self.status_callback,
            10
        )

        # Prevent unused variable warnings
        self.joint_cmd_subscription  # type: ignore
        self.cmd_vel_subscription  # type: ignore
        self.status_subscription  # type: ignore

        self.get_logger().info('Controller Subscriber node initialized')

    def joint_cmd_callback(self, msg):
        """Callback method for processing joint command messages."""
        self.get_logger().info(f'Received joint commands for {len(msg.name)} joints')
        
        # Process each joint command
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
                
                self.get_logger().info(f'  Processing command for {name}: pos={pos:.3f}, vel={vel:.3f}, eff={eff:.3f}')
                
                # Simulate sending command to actual hardware
                # In a real implementation, this would interface with the actual robot controller
                self.execute_joint_command(name, pos, vel, eff)

    def cmd_vel_callback(self, msg):
        """Callback method for processing base movement command messages."""
        self.get_logger().info(f'Received base command: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')
        
        # Process base movement command
        # In a real implementation, this would control the robot's base movement
        self.execute_base_command(msg.linear.x, msg.linear.y, msg.linear.z,
                                 msg.angular.x, msg.angular.y, msg.angular.z)

    def status_callback(self, msg):
        """Callback method for processing agent status messages."""
        self.get_logger().info(f'Agent status: {msg.data}')

    def execute_joint_command(self, joint_name, position, velocity, effort):
        """
        Simulate executing a joint command.
        
        In a real implementation, this method would interface with the actual
        hardware controller to execute the command.
        """
        # Simulate command execution delay
        self.get_logger().info(f'  Executing command for {joint_name}...')

    def execute_base_command(self, linear_x, linear_y, linear_z, 
                            angular_x, angular_y, angular_z):
        """
        Simulate executing a base movement command.
        
        In a real implementation, this method would control the robot's
        base movement actuators.
        """
        self.get_logger().info(f'  Executing base movement: ({linear_x}, {linear_y}, {linear_z}) and ({angular_x}, {angular_y}, {angular_z})')


def main(args=None):
    """Main function that initializes the node and starts spinning."""
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the controller subscriber node
    controller_subscriber = ControllerSubscriber()

    # Start spinning to execute callbacks
    try:
        rclpy.spin(controller_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        controller_subscriber.destroy_node()

        # Shutdown the rclpy library
        rclpy.shutdown()


if __name__ == '__main__':
    main()