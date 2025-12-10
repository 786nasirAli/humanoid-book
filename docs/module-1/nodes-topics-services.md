---
title: ROS 2 Communication Fundamentals for Humanoid Robots
sidebar_position: 2
description: Practical implementation of nodes, topics, and services for humanoid robot communication
---

# ROS 2 Communication Fundamentals for Humanoid Robots

This section provides hands-on experience with the fundamental communication mechanisms in ROS 2 as applied to humanoid robot systems. You'll learn to implement practical nodes, topics, and services that form the communication backbone of humanoid robots.

## Humanoid Communication Architecture

In humanoid robots, the communication architecture resembles the biological nervous system:
- **Nodes** represent specialized neural centers (vision, motor control, etc.)
- **Topics** serve as neural pathways for continuous sensor and actuator data
- **Services** function as command pathways for specific actions and configurations
- **Actions** enable complex behaviors with feedback and cancellation capabilities

## Practical Assignment 1: Creating a Humanoid Sensor Node

**Objective**: Create a realistic sensor node that simulates humanoid robot sensory systems.

**Detailed Steps**:

1. **Create a Joint State Publisher Node**:
   ```python
   #!/usr/bin/env python3
   """
   Joint state publisher for humanoid robot simulation
   """
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import JointState
   from std_msgs.msg import Header
   import math
   import numpy as np

   class HumanoidJointStatePublisher(Node):
       def __init__(self):
           super().__init__('humanoid_joint_state_publisher')

           # Create publisher for joint states
           self.publisher = self.create_publisher(JointState, '/joint_states', 10)

           # Create timer to publish at 50 Hz
           timer_period = 0.02  # seconds (50 Hz)
           self.timer = self.create_timer(timer_period, self.timer_callback)

           # Initialize joint names for humanoid robot
           self.joint_names = [
               'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch',
               'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
               'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch',
               'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
               'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
               'left_elbow', 'left_wrist_yaw', 'left_wrist_pitch',
               'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
               'right_elbow', 'right_wrist_yaw', 'right_wrist_pitch',
               'neck_yaw', 'neck_pitch'
           ]

           # Initialize joint positions with neutral poses
           self.joint_positions = [0.0] * len(self.joint_names)
           self.joint_velocities = [0.0] * len(self.joint_names)
           self.joint_efforts = [0.0] * len(self.joint_names)

           # Initialize time counter for simulated motion
           self.time_counter = 0.0

           self.get_logger().info(f'Humanoid joint state publisher started with {len(self.joint_names)} joints')

       def timer_callback(self):
           # Update joint positions with realistic humanoid motion
           self.update_joint_positions()

           # Create joint state message
           msg = JointState()
           msg.header = Header()
           msg.header.stamp = self.get_clock().now().to_msg()
           msg.header.frame_id = 'base_link'

           msg.name = self.joint_names
           msg.position = self.joint_positions
           msg.velocity = self.joint_velocities
           msg.effort = self.joint_efforts

           # Publish the message
           self.publisher.publish(msg)

           # Log a message every 50 publications
           if int(self.time_counter / 0.02) % 50 == 0:
               self.get_logger().info(f'Published joint states with {len(self.joint_names)} joints')

       def update_joint_positions(self):
           """Update joint positions with realistic humanoid motion patterns"""
           # Update time counter
           self.time_counter += 0.02  # Timer period is 0.02s

           # Create realistic motion for different joint groups
           for i, joint_name in enumerate(self.joint_names):
               # Different motion patterns for different joint types
               if 'hip' in joint_name:
                   # Hip joints with slight oscillation
                   self.joint_positions[i] = 0.1 * math.sin(self.time_counter * 0.5 + i * 0.1)
                   self.joint_velocities[i] = 0.1 * 0.5 * math.cos(self.time_counter * 0.5 + i * 0.1)
               elif 'knee' in joint_name:
                   # Knee joints following hip motion with phase shift
                   self.joint_positions[i] = 0.15 * math.sin(self.time_counter * 0.5 + i * 0.1 + math.pi/4)
                   self.joint_velocities[i] = 0.15 * 0.5 * math.cos(self.time_counter * 0.5 + i * 0.1 + math.pi/4)
               elif 'shoulder' in joint_name:
                   # Shoulder joints with different pattern
                   self.joint_positions[i] = 0.2 * math.sin(self.time_counter * 0.3 + i * 0.15)
                   self.joint_velocities[i] = 0.2 * 0.3 * math.cos(self.time_counter * 0.3 + i * 0.15)
               elif 'elbow' in joint_name:
                   # Elbow joints with complementary motion
                   self.joint_positions[i] = 0.1 * math.sin(self.time_counter * 0.4 + i * 0.1 - math.pi/6)
                   self.joint_velocities[i] = 0.1 * 0.4 * math.cos(self.time_counter * 0.4 + i * 0.1 - math.pi/6)
               elif 'neck' in joint_name:
                   # Neck joints with independent motion
                   self.joint_positions[i] = 0.3 * math.sin(self.time_counter * 0.2)
                   self.joint_velocities[i] = 0.3 * 0.2 * math.cos(self.time_counter * 0.2)
               else:
                   # Ankle and other joints with minimal motion
                   self.joint_positions[i] = 0.05 * math.sin(self.time_counter * 0.7 + i * 0.2)
                   self.joint_velocities[i] = 0.05 * 0.7 * math.cos(self.time_counter * 0.7 + i * 0.2)

   def main(args=None):
       rclpy.init(args=args)

       joint_publisher = HumanoidJointStatePublisher()

       try:
           rclpy.spin(joint_publisher)
       except KeyboardInterrupt:
           joint_publisher.get_logger().info('Joint state publisher stopped')
       finally:
           joint_publisher.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. **Create a Humanoid Sensor Processing Node**:
   ```python
   #!/usr/bin/env python3
   """
   Humanoid sensor processing node - analyzes incoming sensor data
   """
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import JointState, Imu, LaserScan
   from geometry_msgs.msg import Twist
   from std_msgs.msg import Float32
   import numpy as np

   class HumanoidSensorProcessor(Node):
       def __init__(self):
           super().__init__('humanoid_sensor_processor')

           # Subscriptions for different sensor types
           self.joint_sub = self.create_subscription(
               JointState,
               '/joint_states',
               self.joint_state_callback,
               10
           )

           self.imu_sub = self.create_subscription(
               Imu,
               '/imu/data',
               self.imu_callback,
               10
           )

           self.lidar_sub = self.create_subscription(
               LaserScan,
               '/scan',
               self.lidar_callback,
               10
           )

           # Publishers for processed information
           self.balance_score_pub = self.create_publisher(Float32, '/balance_score', 10)
           self.collision_warning_pub = self.create_publisher(Twist, '/collision_avoidance_command', 10)

           # Internal state storage
           self.joint_states = {}
           self.imu_data = None
           self.lidar_data = None

           # Initialize timers for processing
           self.process_timer = self.create_timer(0.05, self.process_sensors)  # 20 Hz

           self.get_logger().info('Humanoid sensor processor initialized')

       def joint_state_callback(self, msg):
           """Process joint state messages"""
           # Store joint positions for balance calculation
           for i, name in enumerate(msg.name):
               if i < len(msg.position):
                   self.joint_states[name] = {
                       'position': msg.position[i],
                       'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                       'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                   }

       def imu_callback(self, msg):
           """Process IMU data for balance estimation"""
           self.imu_data = {
               'orientation': {
                   'x': msg.orientation.x,
                   'y': msg.orientation.y,
                   'z': msg.orientation.z,
                   'w': msg.orientation.w
               },
               'angular_velocity': {
                   'x': msg.angular_velocity.x,
                   'y': msg.angular_velocity.y,
                   'z': msg.angular_velocity.z
               },
               'linear_acceleration': {
                   'x': msg.linear_acceleration.x,
                   'y': msg.linear_acceleration.y,
                   'z': msg.linear_acceleration.z
               }
           }

       def lidar_callback(self, msg):
           """Process LiDAR data for obstacle detection"""
           self.lidar_data = {
               'ranges': msg.ranges,
               'angle_min': msg.angle_min,
               'angle_max': msg.angle_max,
               'angle_increment': msg.angle_increment
           }

       def process_sensors(self):
           """Process all sensor data and publish results"""
           # Calculate balance score from IMU and joint data
           balance_score = self.calculate_balance_score()
           balance_msg = Float32()
           balance_msg.data = balance_score
           self.balance_score_pub.publish(balance_msg)

           # Check for obstacles in LiDAR data
           collision_risk = self.check_collision_risk()
           if collision_risk:
               cmd = Twist()
               cmd.linear.x = -0.5  # Move backward to avoid collision
               cmd.angular.z = 0.5  # Add turning motion
               self.collision_warning_pub.publish(cmd)

       def calculate_balance_score(self):
           """Calculate robot balance score based on IMU and joint data"""
           if self.imu_data is None:
               return 0.5  # Unknown state

           # Extract roll and pitch from orientation (simplified)
           # In practice, you'd use proper quaternion to RPY conversion
           # This is a simplified approximation
           w = self.imu_data['orientation']['w']
           x = self.imu_data['orientation']['x']
           y = self.imu_data['orientation']['y']
           z = self.imu_data['orientation']['z']

           # Calculate roll and pitch (simplified for demo)
           sinr_cosp = 2 * (w * x + y * z)
           cosr_cosp = 1 - 2 * (x * x + y * y)
           roll = math.atan2(sinr_cosp, cosr_cosp)

           sinp = 2 * (w * y - z * x)
           if abs(sinp) >= 1:
               pitch = math.copysign(math.pi / 2, sinp)
           else:
               pitch = math.asin(sinp)

           # Calculate deviation from upright position (|roll| + |pitch|)
           tilt_angle = abs(roll) + abs(pitch)

           # Balance score (1.0 is perfectly balanced, 0.0 is fallen)
           # Using 0.3 radians (about 17 degrees) as the limit for good balance
           balance_score = max(0.0, 1.0 - (tilt_angle / 0.3))

           return balance_score

       def check_collision_risk(self):
           """Check LiDAR data for collision risks"""
           if self.lidar_data is None:
               return False

           # Check for obstacles within 1 meter
           for distance in self.lidar_data['ranges']:
               if not math.isnan(distance) and distance < 1.0:
                   return True

           return False

   def main(args=None):
       rclpy.init(args=args)

       sensor_processor = HumanoidSensorProcessor()

       try:
           rclpy.spin(sensor_processor)
       except KeyboardInterrupt:
           sensor_processor.get_logger().info('Sensor processor stopped')
       finally:
           sensor_processor.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. **Test the Communication Nodes**:
   ```bash
   # Terminal 1: Run the joint state publisher
   ros2 run your_robot_nodes humanoid_joint_publisher

   # Terminal 2: Run the sensor processor
   ros2 run your_robot_nodes humanoid_sensor_processor
   ```

**Expected Outcome**: Two interconnected nodes where one generates realistic humanoid joint states and another processes them to determine balance and collision risks.

**Learning Points**:
- Understanding how to create realistic sensor data for humanoid robots
- Learning to process multiple sensor types in a single node
- Recognizing the importance of sensor fusion in humanoid systems

## Practical Assignment 2: Service Implementation for Humanoid Control

**Objective**: Create a service-based control system for humanoid robot configuration and commands.

**Detailed Steps**:
1. **Create a Custom Service Message**:
   ```bash
   # Create the service definition file
   # File: humanoid_robot_interfaces/srv/HumanoidCommand.srv
   string command_type  # "calibrate", "reset_pose", "change_mode", etc.
   string target_joint  # Joint name if command applies to specific joint
   float64[] parameters  # Additional parameters for the command
   bool success
   string message
   ```

2. **Create the Service Server**:
   ```python
   #!/usr/bin/env python3
   """
   Service server for humanoid robot commands
   """
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import JointState
   from std_msgs.msg import String
   from humanoid_robot_interfaces.srv import HumanoidCommand  # Custom service
   import time

   class HumanoidCommandService(Node):
       def __init__(self):
           super().__init__('humanoid_command_service')

           # Create service
           self.srv = self.create_service(
               HumanoidCommand,
               'humanoid_command',
               self.command_callback
           )

           # Publisher for joint commands
           self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)

           # Store current robot state
           self.current_joint_positions = {}

           self.get_logger().info('Humanoid command service initialized')

       def command_callback(self, request, response):
           """Handle humanoid command requests"""
           self.get_logger().info(f'Received command: {request.command_type}')

           # Process different command types
           if request.command_type == 'calibrate':
               success = self.handle_calibration_command(request)
               response.success = success
               response.message = 'Calibration completed' if success else 'Calibration failed'

           elif request.command_type == 'reset_pose':
               success = self.handle_reset_pose_command(request)
               response.success = success
               response.message = 'Reset pose completed' if success else 'Reset pose failed'

           elif request.command_type == 'change_mode':
               success = self.handle_change_mode_command(request)
               response.success = success
               response.message = 'Mode change completed' if success else 'Mode change failed'

           elif request.command_type == 'move_joint':
               success = self.handle_move_joint_command(request)
               response.success = success
               response.message = 'Joint movement completed' if success else 'Joint movement failed'

           else:
               response.success = False
               response.message = f'Unknown command: {request.command_type}'

           return response

       def handle_calibration_command(self, request):
           """Handle sensor calibration request"""
           self.get_logger().info('Starting sensor calibration process...')

           # In a real implementation, this would:
           # - Move to calibration poses
           # - Check sensor readings
           # - Adjust calibration parameters

           # Simulate calibration process
           time.sleep(2.0)  # Simulate time for calibration

           self.get_logger().info('Sensor calibration completed')
           return True

       def handle_reset_pose_command(self, request):
           """Handle reset to home pose command"""
           self.get_logger().info('Resetting to home pose...')

           # Create joint state message to reset to neutral position
           msg = JointState()
           msg.header.stamp = self.get_clock().now().to_msg()
           msg.header.frame_id = 'base_link'

           # Define home positions for humanoid robot
           joint_names = [
               'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch',
               'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
               'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch',
               'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
               'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
               'left_elbow', 'left_wrist_yaw', 'left_wrist_pitch',
               'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
               'right_elbow', 'right_wrist_yaw', 'right_wrist_pitch',
               'neck_yaw', 'neck_pitch'
           ]

           # Set all joints to zero (neutral position)
           joint_positions = [0.0] * len(joint_names)

           msg.name = joint_names
           msg.position = joint_positions

           # Publish the reset command
           self.joint_cmd_pub.publish(msg)

           time.sleep(3.0)  # Simulate time for movement

           self.get_logger().info('Reset to home pose completed')
           return True

       def handle_change_mode_command(self, request):
           """Handle control mode change"""
           if len(request.parameters) < 1:
               self.get_logger().error('No mode specified in parameters')
               return False

           mode = int(request.parameters[0])
           self.get_logger().info(f'Changing to mode: {mode}')

           # In a real implementation, this would change the control mode
           # of various subsystems

           # Simulate the mode change
           time.sleep(1.0)

           self.get_logger().info(f'Mode changed to: {mode}')
           return True

       def handle_move_joint_command(self, request):
           """Handle moving a specific joint"""
           if len(request.parameters) < 1:
               self.get_logger().error('No position specified for joint movement')
               return False

           if not request.target_joint:
               self.get_logger().error('No target joint specified')
               return False

           target_position = request.parameters[0]
           self.get_logger().info(f'Moving {request.target_joint} to {target_position:.2f}')

           # Create joint state message for specific joint
           msg = JointState()
           msg.header.stamp = self.get_clock().now().to_msg()
           msg.header.frame_id = 'base_link'

           # Get current positions
           joint_names = list(self.current_joint_positions.keys()) if self.current_joint_positions else [request.target_joint]
           joint_positions = []

           for joint_name in joint_names:
               if joint_name == request.target_joint:
                   joint_positions.append(target_position)
               else:
                   # Keep other joints at current position or 0 if unknown
                   if joint_name in self.current_joint_positions:
                       joint_positions.append(self.current_joint_positions[joint_name])
                   else:
                       joint_positions.append(0.0)

           msg.name = joint_names
           msg.position = joint_positions

           # Publish the joint command
           self.joint_cmd_pub.publish(msg)

           time.sleep(2.0)  # Simulate time for movement

           self.get_logger().info(f'Moved {request.target_joint} to {target_position:.2f}')
           return True

   def main(args=None):
       rclpy.init(args=args)

       command_service = HumanoidCommandService()

       try:
           rclpy.spin(command_service)
       except KeyboardInterrupt:
           command_service.get_logger().info('Command service stopped')
       finally:
           command_service.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. **Create a Service Client**:
   ```python
   #!/usr/bin/env python3
   """
   Service client to test humanoid commands
   """
   import rclpy
   from rclpy.node import Node
   from humanoid_robot_interfaces.srv import HumanoidCommand

   class HumanoidCommandClient(Node):
       def __init__(self):
           super().__init__('humanoid_command_client')

           # Create client
           self.cli = self.create_client(HumanoidCommand, 'humanoid_command')

           # Wait for service to become available
           while not self.cli.wait_for_service(timeout_sec=1.0):
               self.get_logger().info('Service not available, waiting again...')

           self.request = HumanoidCommand.Request()

       def send_command(self, command_type, target_joint="", parameters=[]):
           """Send a command to the humanoid robot"""
           self.request.command_type = command_type
           self.request.target_joint = target_joint
           self.request.parameters = parameters

           self.get_logger().info(f'Sending command: {command_type}')

           future = self.cli.call_async(self.request)
           rclpy.spin_until_future_complete(self, future)

           result = future.result()
           if result:
               self.get_logger().info(f'Command result: {result.success}, Message: {result.message}')
           else:
               self.get_logger().error('Failed to get command result')

           return result

       def run_tests(self):
           """Run a series of command tests"""
           self.get_logger().info('Starting command tests...')

           # Test 1: Reset pose
           result = self.send_command('reset_pose')
           if result and result.success:
               self.get_logger().info('✓ Reset pose command successful')
           else:
               self.get_logger().error('✗ Reset pose command failed')

           time.sleep(2)  # Wait between commands

           # Test 2: Calibrate sensors
           result = self.send_command('calibrate')
           if result and result.success:
               self.get_logger().info('✓ Calibration command successful')
           else:
               self.get_logger().error('✗ Calibration command failed')

           time.sleep(2)  # Wait between commands

           # Test 3: Move a specific joint
           result = self.send_command('move_joint', 'left_elbow', [0.5])
           if result and result.success:
               self.get_logger().info('✓ Joint movement command successful')
           else:
               self.get_logger().error('✗ Joint movement command failed')

   def main(args=None):
       rclpy.init(args=args)

       command_client = HumanoidCommandClient()

       try:
           command_client.run_tests()
       except KeyboardInterrupt:
           command_client.get_logger().info('Command client stopped')
       finally:
           command_client.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

4. **Execute Service Communication**:
   ```bash
   # Terminal 1: Run the service server
   ros2 run your_robot_nodes humanoid_command_service

   # Terminal 2: Run the service client
   ros2 run your_robot_nodes humanoid_command_client
   ```

**Expected Outcome**: A service-based control system that can handle different types of humanoid robot commands with proper response handling.

**Learning Points**:
- Understanding the request-response pattern in ROS2 services
- Learning to implement service servers and clients
- Recognizing when to use services vs topics for robot communication

## Practical Assignment 3: Advanced Communication Patterns with Actions

**Objective**: Implement complex behaviors using ROS2 Actions for long-running humanoid robot tasks.

**Detailed Steps**:
1. **Create a Custom Action Message**:
   ```bash
   # Create action definition file
   # File: humanoid_robot_interfaces/action/HumanoidWalk.action
   # Goal definition
   float64 step_size
   float64 step_height
   int32 num_steps
   string gait_type  # "walk", "trot", "pace", etc.

   # Result definition
   bool success
   int32 steps_completed
   string error_message

   # Feedback definition
   int32 steps_completed
   float32 progress_percentage
   string current_phase  # "lifting_leg", "moving_leg", "placing_leg", "balancing"
   ```

2. **Create an Action Server for Walking**:
   ```python
   #!/usr/bin/env python3
   """
   Action server for humanoid walking behaviors
   """
   import rclpy
   import time
   from rclpy.action import ActionServer, GoalResponse, CancelResponse
   from rclpy.node import Node
   from sensor_msgs.msg import JointState
   from humanoid_robot_interfaces.action import HumanoidWalk
   from std_msgs.msg import Float32

   class HumanoidWalkActionServer(Node):
       def __init__(self):
           super().__init__('humanoid_walk_action_server')

           # Create action server
           self._action_server = ActionServer(
               self,
               HumanoidWalk,
               'humanoid_walk',
               execute_callback=self.execute_callback,
               goal_callback=self.goal_callback,
               cancel_callback=self.cancel_callback
           )

           # Publisher for joint commands
           self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)

           # Publisher for step progress
           self.progress_pub = self.create_publisher(Float32, '/walk_progress', 10)

           self.get_logger().info('Humanoid walk action server initialized')

       def goal_callback(self, goal_request):
           """Accept or reject a goal"""
           # Check if the requested gait is supported
           supported_gaits = ['walk', 'trot', 'pace', 'crawl']
           if goal_request.gait_type not in supported_gaits:
               self.get_logger().warn(f'Unsupported gait type: {goal_request.gait_type}')
               return GoalResponse.REJECT

           # Check if we can execute the requested number of steps
           if goal_request.num_steps <= 0 or goal_request.num_steps > 100:
               self.get_logger().warn(f'Invalid number of steps: {goal_request.num_steps}')
               return GoalResponse.REJECT

           self.get_logger().info(f'Accepting walk goal: {goal_request.num_steps} steps using {goal_request.gait_type} gait')
           return GoalResponse.ACCEPT

       def cancel_callback(self, goal_handle):
           """Accept or reject a cancel request"""
           self.get_logger().info('Received cancel request for walk action')
           return CancelResponse.ACCEPT

       def execute_callback(self, goal_handle):
           """Execute the walking action"""
           self.get_logger().info('Executing walking action...')

           # Get goal parameters
           step_size = goal_handle.request.step_size
           step_height = goal_handle.request.step_height
           num_steps = goal_handle.request.num_steps
           gait_type = goal_handle.request.gait_type

           # Initialize result
           result = HumanoidWalk.Result()
           result.success = False
           result.steps_completed = 0
           result.error_message = ""

           # Execute the walking pattern
           for step_idx in range(num_steps):
               # Check if we've been cancelled
               if goal_handle.is_cancel_requested:
                   result.error_message = "Action was cancelled"
                   goal_handle.canceled()
                   self.get_logger().info('Walking action was cancelled')
                   return result

               # Update goal status with feedback
               feedback = HumanoidWalk.Feedback()
               feedback.steps_completed = step_idx + 1
               feedback.progress_percentage = float(step_idx + 1) / float(num_steps) * 100.0

               # Simple walking gait based on type
               if gait_type == 'walk':
                   feedback.current_phase = self.execute_walk_step(step_size, step_height, step_idx)
               elif gait_type == 'trot':
                   feedback.current_phase = self.execute_trot_step(step_size, step_height, step_idx)
               # Add more gaits as needed

               goal_handle.publish_feedback(feedback)

               # Publish progress as a Float32 message
               progress_msg = Float32()
               progress_msg.data = feedback.progress_percentage
               self.progress_pub.publish(progress_msg)

               self.get_logger().info(f'Step {step_idx + 1}/{num_steps} completed, phase: {feedback.current_phase}')

           # If we got here, all steps were completed
           result.success = True
           result.steps_completed = num_steps
           result.error_message = ""
           goal_handle.succeed()

           self.get_logger().info(f'Walking action completed successfully: {num_steps} steps using {gait_type}')
           return result

       def execute_walk_step(self, step_size, step_height, step_idx):
           """Execute a single walk step"""
           # In a real implementation, this would coordinate multiple joint movements
           # For simulation, we'll just simulate the movement
           self.get_logger().debug(f'Executing walk step {step_idx} with size {step_size}, height {step_height}')

           # Simulate lifting and moving leg
           time.sleep(0.5)  # Simulate lift phase

           # Simulate moving leg forward
           time.sleep(0.3)  # Simulate move phase

           # Simulate placing leg
           time.sleep(0.4)  # Simulate place phase

           # Simulate balancing
           time.sleep(0.2)  # Simulate balance phase

           return "balancing"

       def execute_trot_step(self, step_size, step_height, step_idx):
           """Execute a single trot step"""
           self.get_logger().debug(f'Executing trot step {step_idx} with size {step_size}, height {step_height}')

           # Similar to walk but with different timing
           time.sleep(0.3)  # Lift phase
           time.sleep(0.2)  # Move phase
           time.sleep(0.3)  # Place phase
           time.sleep(0.1)  # Balance phase

           return "balanced"

   def main(args=None):
       rclpy.init(args=args)

       walk_action_server = HumanoidWalkActionServer()

       try:
           rclpy.spin(walk_action_server)
       except KeyboardInterrupt:
           walk_action_server.get_logger().info('Walk action server stopped')
       finally:
           walk_action_server.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. **Create an Action Client**:
   ```python
   #!/usr/bin/env python3
   """
   Action client to test humanoid walking
   """
   import rclpy
   from rclpy.action import ActionClient
   from rclpy.node import Node
   from std_msgs.msg import Float32
   from humanoid_robot_interfaces.action import HumanoidWalk
   import time

   class HumanoidWalkClient(Node):
       def __init__(self):
           super().__init__('humanoid_walk_client')

           # Create action client
           self._action_client = ActionClient(
               self,
               HumanoidWalk,
               'humanoid_walk'
           )

           # Progress subscriber
           self.progress_sub = self.create_subscription(
               Float32,
               '/walk_progress',
               self.progress_callback,
               10
           )

           self.current_progress = 0.0

           self.get_logger().info('Humanoid walk client initialized')

       def progress_callback(self, msg):
           """Receive progress updates"""
           self.current_progress = msg.data
           # Only log every 10% to avoid excessive output
           if int(self.current_progress) % 10 == 0 and abs(self.current_progress % 10 - 10) > 9.9:
               self.get_logger().info(f'Walk progress: {self.current_progress:.1f}%')

       def send_walk_goal(self, step_size=0.1, step_height=0.05, num_steps=5, gait_type='walk'):
           """Send a walk goal to the action server"""
           self.get_logger().info(f'Waiting for action server...')
           self._action_client.wait_for_server()

           # Create goal
           goal_msg = HumanoidWalk.Goal()
           goal_msg.step_size = step_size
           goal_msg.step_height = step_height
           goal_msg.num_steps = num_steps
           goal_msg.gait_type = gait_type

           self.get_logger().info(f'Sending walk goal: {num_steps} steps with {gait_type} gait')

           # Send goal
           self._send_goal_future = self._action_client.send_goal_async(
               goal_msg,
               feedback_callback=self.feedback_callback
           )

           self._send_goal_future.add_done_callback(self.goal_response_callback)

       def goal_response_callback(self, future):
           """Handle goal response"""
           goal_handle = future.result()
           if not goal_handle.accepted:
               self.get_logger().info('Goal was rejected :(')
               return

           self.get_logger().info('Goal accepted :)')

           # Get result
           self._get_result_future = goal_handle.get_result_async()
           self._get_result_future.add_done_callback(self.get_result_callback)

       def feedback_callback(self, feedback_msg):
           """Handle feedback from action server"""
           feedback = feedback_msg.feedback
           self.get_logger().debug(f'Received feedback: {feedback.steps_completed} steps, {feedback.progress_percentage:.1f}% progress')

       def get_result_callback(self, future):
           """Handle result callback"""
           result = future.result().result
           self.get_logger().info(f'Result: Success={result.success}, Steps completed={result.steps_completed}, Message={result.error_message}')

   def main(args=None):
       rclpy.init(args=args)

       walk_client = HumanoidWalkClient()

       # Send a walk goal after a short delay
       time.sleep(1.0)
       walk_client.send_walk_goal(step_size=0.15, step_height=0.07, num_steps=10, gait_type='walk')

       try:
           rclpy.spin(walk_client)
       except KeyboardInterrupt:
           walk_client.get_logger().info('Walk client stopped')
       finally:
           walk_client.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

4. **Test Advanced Communication**:
   ```bash
   # Terminal 1: Run the action server
   ros2 run your_robot_nodes humanoid_walk_action_server

   # Terminal 2: Run the action client
   ros2 run your_robot_nodes humanoid_walk_client
   ```

**Expected Outcome**: Advanced communication using ROS2 Actions that can handle long-running humanoid robot behaviors with feedback and cancellation capabilities.

**Learning Points**:
- Understanding the use of Actions for long-running tasks
- Learning to implement action servers and clients
- Recognizing when to use actions instead of services or topics

## Troubleshooting Communication Issues

### Common Problems and Solutions

1. **Node Connection Problems**:
   - **Problem**: Nodes not connecting to each other
   - **Solution**: Check ROS_DOMAIN_ID, network settings, firewall rules

2. **Message Lost Issues**:
   - **Problem**: Messages not reaching subscribers
   - **Solution**: Check QoS settings, topic names, node namespaces

3. **Performance Bottlenecks**:
   - **Problem**: High message latency or dropped messages
   - **Solution**: Adjust QoS profiles, reduce message frequency, optimize message content

## Performance Optimization

### Quality of Service (QoS) Settings

For humanoid robots, selecting appropriate QoS profiles is critical:

```python
# Example of different QoS profiles for different data types
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

# For high-frequency sensor data (LiDAR, cameras)
sensor_qos = QoSProfile(
    depth=5,
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.BEST_EFFORT
)

# For critical control commands
control_qos = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.RELIABLE
)
```

### Resource Management

```python
# Managing memory for high-frequency data
class EfficientDataBuffer:
    def __init__(self, max_size=1000):
        self.buffer = []
        self.max_size = max_size
        self.index = 0

    def add_data(self, data):
        if len(self.buffer) < self.max_size:
            self.buffer.append(data)
        else:
            # Circular buffer implementation
            self.buffer[self.index] = data
            self.index = (self.index + 1) % self.max_size
```

## Best Practices for Humanoid Communication

1. **Use Appropriate Message Types**: Select standard ROS messages when possible, but create custom messages for humanoid-specific data
2. **Implement Proper Error Handling**: Always check for valid data and handle exceptions gracefully
3. **Consider Timing Requirements**: Humanoid robots often have strict timing requirements for balance and control
4. **Optimize Message Sizes**: Only include necessary data in messages to reduce network overhead
5. **Use Namespacing**: Organize topics and services using appropriate namespaces for clarity

## Resources and Further Learning

- [ROS 2 Documentation](https://docs.ros.org/en/rolling/index.html)
- [ROS 2 Tutorials](https://docs.ros.org/en/rolling/Tutorials.html)
- [Quality of Service in ROS 2](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-settings.html)
- [ROS 2 Actions Documentation](https://docs.ros.org/en/rolling/Concepts/About-Actions.html)