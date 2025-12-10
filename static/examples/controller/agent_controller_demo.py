#!/usr/bin/env python3

"""
Complete agent-controller communication example for humanoid robot.

This demonstrates a complete communication system between a Python agent
and humanoid robot controllers, including both publishing commands and
receiving feedback.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TwistStamped
from builtin_interfaces.msg import Time
import math
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from threading import Thread
import time


class AgentNode(Node):
    """A Python agent that sends commands to the humanoid robot."""

    def __init__(self):
        """Initialize the agent node."""
        super().__init__('agent_node')
        
        # Create QoS profile for real-time control
        qos_profile = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Publishers
        self.joint_cmd_publisher = self.create_publisher(JointState, '/joint_commands', qos_profile)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.status_publisher = self.create_publisher(String, '/agent_status', 10)

        # Create a timer that calls the timer_callback method at 20Hz
        timer_period = 0.05  # seconds (20Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize a counter
        self.i = 0
        
        # Define joint names for a simple humanoid
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        self.get_logger().info('Agent node initialized')

    def timer_callback(self):
        """Callback method that publishes commands at regular intervals."""
        # Publish joint commands
        joint_cmd_msg = JointState()
        joint_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        joint_cmd_msg.header.frame_id = 'base_link'
        joint_cmd_msg.name = self.joint_names
        
        # Generate joint command positions using coordinated patterns
        positions = []
        velocities = []
        efforts = []
        
        for idx, name in enumerate(self.joint_names):
            # Coordinated motion pattern for walking simulation
            phase_offset = idx * 0.5  # Different phase for each joint
            if 'hip' in name:
                pos = math.sin(self.i / 10.0 + phase_offset) * 0.2
            elif 'knee' in name:
                pos = math.sin(self.i / 10.0 + phase_offset + 0.5) * 0.15
            elif 'ankle' in name:
                pos = math.sin(self.i / 10.0 + phase_offset + 1.0) * 0.1
            elif 'shoulder' in name:
                pos = math.sin(self.i / 15.0 + phase_offset) * 0.25
            else:  # elbow
                pos = math.sin(self.i / 15.0 + phase_offset + 0.7) * 0.2
            positions.append(pos)
            velocities.append(0.0)
            efforts.append(0.0)

        joint_cmd_msg.position = positions
        joint_cmd_msg.velocity = velocities
        joint_cmd_msg.effort = efforts

        self.joint_cmd_publisher.publish(joint_cmd_msg)

        # Publish base movement command
        twist_msg = Twist()
        # Simple movement: forward with slight oscillation
        twist_msg.linear.x = 0.15  # Move forward at 0.15 m/s
        twist_msg.angular.z = math.sin(self.i / 50.0) * 0.1  # Slow oscillating rotation
        
        self.cmd_vel_publisher.publish(twist_msg)

        # Publish status message
        status_msg = String()
        status_msg.data = f'Agent cycle: {self.i}, sending coordinated commands'
        self.status_publisher.publish(status_msg)

        # Increment the counter
        self.i += 1


class ControllerNode(Node):
    """A controller that receives commands and sends feedback."""

    def __init__(self):
        """Initialize the controller node."""
        super().__init__('controller_node')
        
        # Create QoS profile for real-time control
        qos_profile = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Subscriptions
        self.joint_cmd_subscription = self.create_subscription(
            JointState,
            '/joint_commands',
            self.joint_cmd_callback,
            qos_profile
        )
        
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        self.status_subscription = self.create_subscription(
            String,
            '/agent_status',
            self.status_callback,
            10
        )

        # Publishers for feedback
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', qos_profile)
        self.feedback_publisher = self.create_publisher(String, '/controller_feedback', 10)

        # Prevent unused variable warnings
        self.joint_cmd_subscription  # type: ignore
        self.cmd_vel_subscription  # type: ignore
        self.status_subscription  # type: ignore

        # Initialize joint positions for feedback simulation
        self.current_joint_positions = [0.0] * len([
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ])

        self.get_logger().info('Controller node initialized')

    def joint_cmd_callback(self, msg):
        """Callback method for processing joint command messages."""
        self.get_logger().info(f'Controller: Received {len(msg.name)} joint commands')

        # Update current joint positions based on commands (simulated execution)
        for i, name in enumerate(msg.name):
            if i < len(msg.position) and i < len(self.current_joint_positions):
                # Simulate movement toward commanded position
                current_pos = self.current_joint_positions[i]
                cmd_pos = msg.position[i]
                # Move 10% of the way to the command position
                self.current_joint_positions[i] = current_pos * 0.9 + cmd_pos * 0.1

        # Publish updated joint states (feedback)
        feedback_msg = JointState()
        feedback_msg.header.stamp = self.get_clock().now().to_msg()
        feedback_msg.header.frame_id = 'base_link'
        feedback_msg.name = msg.name
        feedback_msg.position = self.current_joint_positions[:]
        
        self.joint_state_publisher.publish(feedback_msg)

        # Publish feedback status
        feedback_status = String()
        feedback_status.data = f'Controller executed {len(msg.name)} commands, sent feedback'
        self.feedback_publisher.publish(feedback_status)

    def cmd_vel_callback(self, msg):
        """Callback method for processing base movement command messages."""
        self.get_logger().info(f'Controller: Received base command: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')

    def status_callback(self, msg):
        """Callback method for processing agent status messages."""
        self.get_logger().info(f'Controller: Agent status: {msg.data}')


def main(args=None):
    """Main function that runs both nodes in separate threads."""
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create nodes
    agent_node = AgentNode()
    controller_node = ControllerNode()

    # Function to run the agent node
    def run_agent():
        try:
            rclpy.spin(agent_node)
        except KeyboardInterrupt:
            pass
        finally:
            agent_node.destroy_node()

    # Function to run the controller node
    def run_controller():
        try:
            rclpy.spin(controller_node)
        except KeyboardInterrupt:
            pass
        finally:
            controller_node.destroy_node()

    # Start both nodes in separate threads
    agent_thread = Thread(target=run_agent)
    controller_thread = Thread(target=run_controller)
    
    agent_thread.start()
    controller_thread.start()

    try:
        # Keep the main thread alive
        while agent_thread.is_alive() and controller_thread.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown the rclpy library
        rclpy.shutdown()

        # Wait for threads to finish
        agent_thread.join()
        controller_thread.join()


if __name__ == '__main__':
    main()