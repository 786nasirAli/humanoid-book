#!/usr/bin/env python3

"""
Detailed ROS 2 subscriber example for humanoid robot sensors.

This node subscribes to various topics from a humanoid robot,
demonstrating more advanced subscriber concepts like different
message types, Quality of Service settings, and proper data processing.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time


class DetailedSubscriber(Node):
    """
    A detailed subscriber node processing various humanoid robot sensor data.
    
    This subscriber demonstrates:
    - Handling different message types (JointState, Float64MultiArray)
    - Quality of Service settings
    - Proper data processing and logging
    - Error handling and validation
    """
    
    def __init__(self):
        """Initialize the detailed subscriber node."""
        super().__init__('detailed_subscriber')
        
        # Create subscriptions for different types of data
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'humanoid_joint_states',
            self.joint_state_callback,
            10  # QoS history depth
        )
        
        self.imu_data_subscription = self.create_subscription(
            Float64MultiArray,
            'imu_data',
            self.imu_data_callback,
            10  # QoS history depth
        )
        
        # Track message counts
        self.joint_msg_count = 0
        self.imu_msg_count = 0
        
        self.get_logger().info("Detailed Subscriber initialized")

    def joint_state_callback(self, msg):
        """Callback method for processing JointState messages."""
        self.joint_msg_count += 1
        
        # Log received joint states
        self.get_logger().info(
            f"Joint State #{self.joint_msg_count}: "
            f"Received {len(msg.name)} joints, "
            f"timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        )
        
        # Process and validate joint data
        if len(msg.position) != len(msg.name):
            self.get_logger().error("Position array length doesn't match joint names length")
            return
            
        # Log first few joint positions
        for i in range(min(3, len(msg.name))):
            self.get_logger().info(
                f"  {msg.name[i]}: pos={msg.position[i]:.3f}, "
                f"vel={msg.velocity[i]:.3f}, "
                f"eff={msg.effort[i]:.3f}"
            )
        
        # Perform any additional processing here
        # For example, check if joint limits are exceeded
        self.check_joint_limits(msg)

    def imu_data_callback(self, msg):
        """Callback method for processing IMU data messages."""
        self.imu_msg_count += 1
        
        # Validate message size (should have 10 values: 4 orientation + 3 angular vel + 3 linear acc)
        if len(msg.data) != 10:
            self.get_logger().error(f"Expected 10 IMU values, got {len(msg.data)}")
            return
        
        # Extract data
        orientation = msg.data[0:4]  # [x, y, z, w]
        angular_velocity = msg.data[4:7]  # [x, y, z]
        linear_acceleration = msg.data[7:10]  # [x, y, z]
        
        self.get_logger().info(
            f"IMU Data #{self.imu_msg_count}: "
            f"Orientation=[{orientation[0]:.3f}, {orientation[1]:.3f}, {orientation[2]:.3f}, {orientation[3]:.3f}], "
            f"AngVel=[{angular_velocity[0]:.3f}, {angular_velocity[1]:.3f}, {angular_velocity[2]:.3f}], "
            f"LinAcc=[{linear_acceleration[0]:.3f}, {linear_acceleration[1]:.3f}, {linear_acceleration[2]:.3f}]"
        )
        
        # Perform any additional processing here
        # For example, check for unusual values
        self.check_imu_values(orientation, angular_velocity, linear_acceleration)

    def check_joint_limits(self, joint_state):
        """Check if joint positions exceed reasonable limits."""
        for i, (name, pos) in enumerate(zip(joint_state.name, joint_state.position)):
            # Example: check if joint position is in reasonable range
            # These limits would be specific to the actual robot in practice
            if abs(pos) > 3.14:  # Greater than 180 degrees
                self.get_logger().warning(f"Joint {name} position {pos:.3f} exceeds normal limits")

    def check_imu_values(self, orientation, angular_velocity, linear_acceleration):
        """Check IMU values for reasonable ranges."""
        # Check if orientation quaternion is normalized (approximately)
        norm = sum(x*x for x in orientation)
        if abs(norm - 1.0) > 0.1:
            self.get_logger().warning(f"Quaternion not normalized: {norm:.3f}")
        
        # Check for unusual angular velocities
        max_ang_vel = max(abs(v) for v in angular_velocity)
        if max_ang_vel > 10.0:  # Greater than 10 rad/s
            self.get_logger().warning(f"High angular velocity detected: {max_ang_vel:.3f}")


def main(args=None):
    """Main function that initializes the node and starts spinning."""
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the detailed subscriber node
    detailed_subscriber = DetailedSubscriber()

    try:
        # Start spinning to execute callbacks
        rclpy.spin(detailed_subscriber)
    except KeyboardInterrupt:
        detailed_subscriber.get_logger().info("Interrupted by user")
    finally:
        # Destroy the node explicitly
        detailed_subscriber.destroy_node()
        
        # Shutdown the rclpy library
        rclpy.shutdown()


if __name__ == '__main__':
    main()