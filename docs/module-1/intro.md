---
title: Introduction to the Robotic Nervous System
sidebar_position: 1
description: Understanding how ROS 2 functions as the nervous system of humanoid robots
---

# Introduction to the Robotic Nervous System

Welcome to Module 1 of the Humanoid AI Book, where we explore how ROS 2 (Robot Operating System 2) functions as the "nervous system" of humanoid robots. This module will provide you with a comprehensive understanding of ROS 2's communication infrastructure, focusing on practical examples that demonstrate how different components of a humanoid robot work together.

## ROS 2 as a Nervous System

Think of ROS 2 as the nervous system of a humanoid robot. Just as the biological nervous system enables communication between the brain, sensory organs, and muscles, ROS 2 enables communication between different software components (nodes) that control the robot's sensors, actuators, and decision-making processes. Information flows from sensors (like touch, vision, and proprioception) to processing units (perception and decision-making), and then to actuators (motors) that execute actions.

This architecture allows for a distributed system where different parts of the robot can operate independently while staying coordinated through the ROS 2 middleware. This modularity is essential for humanoid robots, which require complex coordination between many different subsystems.

## Prerequisites

Before starting this module, ensure you have:

1. **ROS 2 Humble Hawksbill** installed on your system
   - Follow the official installation guide for your OS: https://docs.ros.org/en/humble/Installation.html
   - Install the desktop version which includes development tools

2. **Python 3.8+** with pip package manager

3. **Basic Python knowledge** (variables, functions, classes)

## ROS 2 Humble Hawksbill Installation Prerequisites

### Ubuntu Linux (Recommended)

#### System Requirements
- Ubuntu 22.04 (Jammy Jellyfish) - 64-bit
- At least 5 GB of free disk space
- Recommended: 8+ GB RAM for development

### Installation Steps
1. Set locale to support UTF-8:
   ```bash
   sudo locale-gen en_US.UTF-8
   ```

2. Add the ROS 2 apt repository:
   ```bash
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. Install ROS 2 development packages:
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-desktop
   ```

4. Install Python 3 dependencies for rclpy:
   ```bash
   sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
   ```

5. Initialize rosdep:
   ```bash
   sudo rosdep init
   rosdep update
   ```

6. Source the ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

   To automatically source in new terminals, add to your `.bashrc`:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

### Windows 10/11

#### Prerequisites
- Windows 10 (Build 19044 or higher) or Windows 11
- WSL2 (Windows Subsystem for Linux) with Ubuntu 22.04 recommended
- At least 10 GB free disk space

#### Installation via WSL2
1. Install WSL2 with Ubuntu 22.04
2. Follow the Ubuntu installation steps above

## ROS 2 Workspace Setup

Once you have ROS 2 installed, you'll need to create a workspace for your humanoid robot project:

1. Create a workspace directory:
   ```bash
   mkdir -p ~/humanoid_ws/src
   cd ~/humanoid_ws
   ```

2. Build the workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

## Key ROS 2 Concepts

ROS 2 is built around several core concepts that enable this "nervous system" behavior:

### Nodes
A Node is the fundamental unit of execution in ROS 2. Think of a Node as an individual brain region responsible for a specific function. For example, one Node might handle camera data processing, another might manage motor control, and a third might handle path planning. Each Node performs computations and can communicate with other nodes.

### Topics
Topics are the communication channels that allow Nodes to exchange data. Data flow through Topics is unidirectional - one Node publishes data to a Topic, and other Nodes subscribe to that Topic to receive the data. This follows the publish-subscribe pattern and is ideal for sensor data and continuous state updates.

### Services
Services enable request-response communication, where one Node sends a request and waits for a response from another Node. This is similar to asking a specific question and waiting for an answer. Services are useful for operations that require confirmation or return a specific result, such as requesting the robot to move to a specific position.

### Actions
Actions are for long-running tasks that may take significant time to complete. They combine the features of Topics and Services, providing feedback during execution, the ability to cancel the task, and a final result. Actions are ideal for navigation tasks, complex manipulations, or any operation that takes time and might need status updates.

## Real-world Humanoid Robot Examples

Many humanoid robots use ROS 2 as their communication backbone:

- **PAL Robotics' TALOS**: A research humanoid robot that uses ROS for coordinating its many sensors and actuators [1].
- **ROBOTIS OP3**: An open platform humanoid robot designed for research and education, using ROS for sensor integration and motion control [2].
- **Boston Dynamics' robots**: Though primarily proprietary, similar architecture principles apply to their communication systems [3].

These robots demonstrate how ROS 2's distributed architecture allows for complex coordination of many subsystems while maintaining flexibility and modularity.

## References
1. PAL Robotics. (2023). TALOS Humanoid Robot. Retrieved from https://pal-robotics.com/robots/talos/
2. ROBOTIS. (2023). OP3 Humanoid Robot. Retrieved from https://emanual.robotis.com/docs/en/platform/op3/introduction/
3. Boston Dynamics. (2023). Robotics Principles. Retrieved from https://www.bostondynamics.com/resources

## Practical Assignment - First ROS2 Package for Humanoid Robot

### Assignment 1: Creating Your First Humanoid Robot Package

**Objective**: Create a basic ROS2 package for the humanoid robot with essential nodes and message types.

**Detailed Steps**:
1. **Create the Package Structure**:
   ```bash
   # Create a new ROS2 package for the humanoid robot
   ros2 pkg create --build-type ament_python humanoid_robot_interfaces
   ```

2. **Create a Custom Message Type**:
   ```bash
   # Create a custom message for humanoid joint commands
   # Create the msg directory in your package
   mkdir -p humanoid_robot_interfaces/msg

   # Create the HumanoidJointCommand.msg file
   ```

   Content for `msg/HumanoidJointCommand.msg`:
   ```
   # Custom message for humanoid robot joint commands
   string joint_name
   float64 position
   float64 velocity
   float64 effort
   ```

3. **Update package.xml**:
   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>humanoid_robot_interfaces</name>
     <version>0.0.0</version>
     <description>Custom interfaces for humanoid robot control</description>
     <maintainer email="developer@example.com">Your Name</maintainer>
     <license>Apache-2.0</license>

     <depend>builtin_interfaces</depend>
     <depend>std_msgs</depend>

     <buildtool_depend>ament_cmake</buildtool_depend>
     <buildtool_depend>rosidl_default_generators</buildtool_depend>

     <exec_depend>builtin_interfaces</exec_depend>
     <exec_depend>std_msgs</exec_depend>
     <exec_depend>rosidl_default_runtime</exec_depend>

     <member_of_group>rosidl_interface_packages</member_of_group>

     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>
   ```

4. **Update setup.py**:
   ```python
   from setuptools import setup
   from pathlib import Path

   package_name = 'humanoid_robot_interfaces'

   setup(
       name=package_name,
       version='0.0.0',
       packages=[package_name],
       data_files=[
           ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='Your Name',
       maintainer_email='developer@example.com',
       description='Custom interfaces for humanoid robot control',
       license='Apache License 2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
           ],
       },
   )
   ```

5. **Create a Basic Publisher Node**:
   ```python
   #!/usr/bin/env python3
   # File: humanoid_robot_interfaces/examples/joint_command_publisher.py

   import rclpy
   from rclpy.node import Node
   import math
   import time

   # Import your custom message
   from humanoid_robot_interfaces.msg import HumanoidJointCommand


   class JointCommandPublisher(Node):
       def __init__(self):
           super().__init__('joint_command_publisher')

           # Create publisher
           self.publisher = self.create_publisher(HumanoidJointCommand, 'joint_commands', 10)

           # Timer for publishing messages
           timer_period = 0.1  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)

           # Counter for message
           self.i = 0

           self.get_logger().info('Joint Command Publisher node initialized')

       def timer_callback(self):
           msg = HumanoidJointCommand()

           # Create a simple oscillating pattern
           msg.joint_name = 'left_hip_joint'
           msg.position = math.sin(self.i / 10.0) * 0.5
           msg.velocity = math.cos(self.i / 10.0) * 0.5
           msg.effort = 0.0  # For now

           self.publisher.publish(msg)
           self.get_logger().info(f'Publishing: Joint={msg.joint_name}, Pos={msg.position:.2f}')
           self.i += 1


   def main(args=None):
       rclpy.init(args=args)

       joint_command_publisher = JointCommandPublisher()

       try:
           rclpy.spin(joint_command_publisher)
       except KeyboardInterrupt:
           pass
       finally:
           joint_command_publisher.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

6. **Create a Basic Subscriber Node**:
   ```python
   #!/usr/bin/env python3
   # File: humanoid_robot_interfaces/examples/joint_command_subscriber.py

   import rclpy
   from rclpy.node import Node

   # Import your custom message
   from humanoid_robot_interfaces.msg import HumanoidJointCommand


   class JointCommandSubscriber(Node):
       def __init__(self):
           super().__init__('joint_command_subscriber')

           # Create subscriber
           self.subscription = self.create_subscription(
               HumanoidJointCommand,
               'joint_commands',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

           self.get_logger().info('Joint Command Subscriber node initialized')

       def listener_callback(self, msg):
           self.get_logger().info(f'Received: Joint={msg.joint_name}, Pos={msg.position:.2f}, Vel={msg.velocity:.2f}')


   def main(args=None):
       rclpy.init(args=args)

       joint_command_subscriber = JointCommandSubscriber()

       try:
           rclpy.spin(joint_command_subscriber)
       except KeyboardInterrupt:
           pass
       finally:
           joint_command_subscriber.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

7. **Build the Package**:
   ```bash
   # Build the package
   colcon build --packages-select humanoid_robot_interfaces

   # Source the workspace
   source install/setup.bash
   ```

8. **Run the Publisher and Subscriber**:
   ```bash
   # Terminal 1: Run the publisher
   ros2 run humanoid_robot_interfaces joint_command_publisher

   # Terminal 2: Run the subscriber
   ros2 run humanoid_robot_interfaces joint_command_subscriber
   ```

**Expected Outcome**: Two ROS2 nodes communicating using a custom message type for humanoid joint commands.

**Learning Points**:
- Understanding how to create custom message types in ROS2
- Learning to build and run ROS2 Python nodes
- Recognizing the publisher-subscriber pattern in ROS2

### Assignment 2: Creating a Humanoid Robot State Publisher

**Objective**: Create a node that publishes the current state of the humanoid robot.

**Detailed Steps**:
1. **Create a Robot State Message**:
   ```
   # In msg/HumanoidRobotState.msg
   builtin_interfaces/Time header
   string[] joint_names
   float64[] joint_positions
   float64[] joint_velocities
   float64[] joint_efforts
   geometry_msgs/Pose[] link_poses
   ```

2. **Create the State Publisher Node**:
   ```python
   #!/usr/bin/env python3
   # File: humanoid_robot_interfaces/examples/robot_state_publisher.py

   import rclpy
   from rclpy.node import Node
   import math
   from builtin_interfaces.msg import Time

   # Import standard and custom messages
   from humanoid_robot_interfaces.msg import HumanoidRobotState
   from geometry_msgs.msg import Pose


   class RobotStatePublisher(Node):
       def __init__(self):
           super().__init__('robot_state_publisher')

           # Create publisher for robot state
           self.publisher = self.create_publisher(HumanoidRobotState, 'robot_state', 10)

           # Timer for publishing state
           timer_period = 0.05  # seconds (20Hz)
           self.timer = self.create_timer(timer_period, self.timer_callback)

           # Initialize counter
           self.time_counter = 0

           self.get_logger().info('Robot State Publisher node initialized')

       def timer_callback(self):
           msg = HumanoidRobotState()

           # Set header
           msg.header.stamp = self.get_clock().now().to_msg()
           msg.header.frame_id = 'base_link'

           # Define joint names for a simple humanoid
           joint_names = [
               'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
               'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
               'left_shoulder_joint', 'left_elbow_joint',
               'right_shoulder_joint', 'right_elbow_joint'
           ]
           msg.joint_names = joint_names

           # Generate simulated joint positions (oscillating for demo)
           positions = []
           velocities = []
           efforts = []

           for i, name in enumerate(joint_names):
               # Create different motion patterns for different joints
               if 'hip' in name:
                   pos = math.sin(self.time_counter / 20.0 + i) * 0.3
                   vel = math.cos(self.time_counter / 20.0 + i) * 0.3 / 20.0
               elif 'knee' in name:
                   pos = math.sin(self.time_counter / 15.0 + i) * 0.2
                   vel = math.cos(self.time_counter / 15.0 + i) * 0.2 / 15.0
               elif 'ankle' in name:
                   pos = math.sin(self.time_counter / 25.0 + i) * 0.1
                   vel = math.cos(self.time_counter / 25.0 + i) * 0.1 / 25.0
               elif 'shoulder' in name:
                   pos = math.sin(self.time_counter / 18.0 + i) * 0.4
                   vel = math.cos(self.time_counter / 18.0 + i) * 0.4 / 18.0
               else:  # elbow
                   pos = math.sin(self.time_counter / 12.0 + i) * 0.3
                   vel = math.cos(self.time_counter / 12.0 + i) * 0.3 / 12.0

               positions.append(pos)
               velocities.append(vel)
               efforts.append(0.0)  # For now, no actual effort

           msg.joint_positions = positions
           msg.joint_velocities = velocities
           msg.joint_efforts = efforts

           # For simplicity, just publish a few link poses
           # In a real robot, these would come from FK calculations
           link_poses = []
           for i in range(5):  # Just 5 example poses
               pose = Pose()
               pose.position.x = math.sin(self.time_counter / 10.0 + i) * 0.1
               pose.position.y = math.cos(self.time_counter / 10.0 + i) * 0.1
               pose.position.z = 0.0
               pose.orientation.w = 1.0
               pose.orientation.x = 0.0
               pose.orientation.y = 0.0
               pose.orientation.z = 0.0
               link_poses.append(pose)

           msg.link_poses = link_poses

           self.publisher.publish(msg)
           self.get_logger().info(f'Published robot state for {len(joint_names)} joints')
           self.time_counter += 1


   def main(args=None):
       rclpy.init(args=args)

       robot_state_publisher = RobotStatePublisher()

       try:
           rclpy.spin(robot_state_publisher)
       except KeyboardInterrupt:
           pass
       finally:
           robot_state_publisher.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

3. **Create a Simple Service Server**:
   ```python
   #!/usr/bin/env python3
   # File: humanoid_robot_interfaces/examples/robot_control_server.py

   import rclpy
   from rclpy.node import Node
   from std_srvs.srv import Trigger


   class RobotControlServer(Node):
       def __init__(self):
           super().__init__('robot_control_server')

           # Create service
           self.srv = self.create_service(
               Trigger,
               'robot_reset',
               self.reset_callback
           )

           self.get_logger().info('Robot Control Service server initialized')

       def reset_callback(self, request, response):
           self.get_logger().info('Reset command received!')

           # In a real implementation, this would reset robot state
           # For now, just simulate the reset
           response.success = True
           response.message = 'Robot reset command processed successfully'

           return response


   def main(args=None):
       rclpy.init(args=args)

       robot_control_server = RobotControlServer()

       try:
           rclpy.spin(robot_control_server)
       except KeyboardInterrupt:
           pass
       finally:
           robot_control_server.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

4. **Build and Run the Nodes**:
   ```bash
   # Rebuild the package with new files
   colcon build --packages-select humanoid_robot_interfaces

   # Source the workspace
   source install/setup.bash

   # Terminal 1: Run the state publisher
   ros2 run humanoid_robot_interfaces robot_state_publisher

   # Terminal 2: Call the reset service
   ros2 service call /robot_reset std_srvs/srv/Trigger
   ```

**Expected Outcome**: A complete ROS2 package with custom message types, publishers, subscribers, and services for humanoid robot control.

**Learning Points**:
- Understanding how to create complex robot state messages
- Learning to implement ROS2 services
- Recognizing how to structure robot control systems in ROS2

### Assignment 3: Creating a URDF Model for the Humanoid Robot

**Objective**: Create a proper URDF model for the humanoid robot with all necessary joints and links.

**Detailed Steps**:
1. **Create the URDF Directory Structure**:
   ```bash
   mkdir -p humanoid_robot_description/urdf
   mkdir -p humanoid_robot_description/meshes
   mkdir -p humanoid_robot_description/config
   ```

2. **Create the Main URDF File**:
   ```xml
   <!-- humanoid_robot_description/urdf/humanoid_robot.urdf -->
   <?xml version="1.0"?>
   <robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
     <!-- Include other files -->
     <xacro:include filename="$(find humanoid_robot_description)/urdf/materials.urdf.xacro" />
     <xacro:include filename="$(find humanoid_robot_description)/urdf/transmission_macros.urdf.xacro" />

     <!-- Constants -->
     <xacro:property name="PI" value="3.1415926535897931" />

     <!-- Base link -->
     <link name="base_link">
       <visual>
         <origin xyz="0 0 0.5" rpy="0 0 0"/>
         <geometry>
           <box size="0.3 0.3 1.0"/>
         </geometry>
         <material name="blue"/>
       </visual>
       <collision>
         <origin xyz="0 0 0.5" rpy="0 0 0"/>
         <geometry>
           <box size="0.3 0.3 1.0"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="10"/>
         <origin xyz="0 0 0.5" rpy="0 0 0"/>
         <inertia ixx="0.3" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.1"/>
       </inertial>
     </link>

     <!-- Torso -->
     <joint name="torso_joint" type="fixed">
       <parent link="base_link"/>
       <child link="torso"/>
       <origin xyz="0 0 1.0" rpy="0 0 0"/>
     </joint>

     <link name="torso">
       <visual>
         <geometry>
           <box size="0.25 0.25 0.5"/>
         </geometry>
         <material name="white"/>
       </visual>
       <collision>
         <geometry>
           <box size="0.25 0.25 0.5"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="5"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.2"/>
       </inertial>
     </link>

     <!-- Head -->
     <joint name="neck_joint" type="revolute">
       <parent link="torso"/>
       <child link="head"/>
       <origin xyz="0 0 0.3" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="${-PI/4}" upper="${PI/4}" effort="10" velocity="1"/>
     </joint>

     <link name="head">
       <visual>
         <geometry>
           <sphere radius="0.15"/>
         </geometry>
         <material name="gray"/>
       </visual>
       <collision>
         <geometry>
           <sphere radius="0.15"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <!-- Left Arm -->
     <joint name="left_shoulder_joint" type="revolute">
       <parent link="torso"/>
       <child link="left_upper_arm"/>
       <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="${-PI/2}" upper="${PI/2}" effort="10" velocity="1"/>
     </joint>

     <link name="left_upper_arm">
       <visual>
         <geometry>
           <cylinder radius="0.05" length="0.4"/>
         </geometry>
         <material name="red"/>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.05" length="0.4"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1"/>
         <origin xyz="0 0 -0.2"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <joint name="left_elbow_joint" type="revolute">
       <parent link="left_upper_arm"/>
       <child link="left_lower_arm"/>
       <origin xyz="0 0 -0.4" rpy="0 0 0"/>
       <axis xyz="0 0 1"/>
       <limit lower="${-PI/2}" upper="${PI/2}" effort="10" velocity="1"/>
     </joint>

     <link name="left_lower_arm">
       <visual>
         <geometry>
           <cylinder radius="0.04" length="0.35"/>
         </geometry>
         <material name="red"/>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.04" length="0.35"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <origin xyz="0 0 -0.175"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <!-- Right Arm -->
     <joint name="right_shoulder_joint" type="revolute">
       <parent link="torso"/>
       <child link="right_upper_arm"/>
       <origin xyz="-0.15 0 0.1" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="${-PI/2}" upper="${PI/2}" effort="10" velocity="1"/>
     </joint>

     <link name="right_upper_arm">
       <visual>
         <geometry>
           <cylinder radius="0.05" length="0.4"/>
         </geometry>
         <material name="green"/>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.05" length="0.4"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1"/>
         <origin xyz="0 0 -0.2"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <joint name="right_elbow_joint" type="revolute">
       <parent link="right_upper_arm"/>
       <child link="right_lower_arm"/>
       <origin xyz="0 0 -0.4" rpy="0 0 0"/>
       <axis xyz="0 0 1"/>
       <limit lower="${-PI/2}" upper="${PI/2}" effort="10" velocity="1"/>
     </joint>

     <link name="right_lower_arm">
       <visual>
         <geometry>
           <cylinder radius="0.04" length="0.35"/>
         </geometry>
         <material name="green"/>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.04" length="0.35"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <origin xyz="0 0 -0.175"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <!-- Left Leg -->
     <joint name="left_hip_joint" type="revolute">
       <parent link="base_link"/>
       <child link="left_thigh"/>
       <origin xyz="0.07 0 0" rpy="0 0 0"/>
       <axis xyz="0 0 1"/>
       <limit lower="${-PI/2}" upper="${PI/2}" effort="20" velocity="1"/>
     </joint>

     <link name="left_thigh">
       <visual>
         <geometry>
           <cylinder radius="0.07" length="0.5"/>
         </geometry>
         <material name="blue"/>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.07" length="0.5"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="2"/>
         <origin xyz="0 0 -0.25"/>
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <joint name="left_knee_joint" type="revolute">
       <parent link="left_thigh"/>
       <child link="left_shin"/>
       <origin xyz="0 0 -0.5" rpy="0 0 0"/>
       <axis xyz="0 0 1"/>
       <limit lower="0" upper="${PI/2}" effort="20" velocity="1"/>
     </joint>

     <link name="left_shin">
       <visual>
         <geometry>
           <cylinder radius="0.06" length="0.45"/>
         </geometry>
         <material name="blue"/>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.06" length="0.45"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1.5"/>
         <origin xyz="0 0 -0.225"/>
         <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <joint name="left_ankle_joint" type="revolute">
       <parent link="left_shin"/>
       <child link="left_foot"/>
       <origin xyz="0 0 -0.45" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="${-PI/4}" upper="${PI/4}" effort="10" velocity="1"/>
     </joint>

     <link name="left_foot">
       <visual>
         <geometry>
           <box size="0.2 0.1 0.05"/>
         </geometry>
         <material name="black"/>
       </visual>
       <collision>
         <geometry>
           <box size="0.2 0.1 0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <!-- Right Leg -->
     <joint name="right_hip_joint" type="revolute">
       <parent link="base_link"/>
       <child link="right_thigh"/>
       <origin xyz="-0.07 0 0" rpy="0 0 0"/>
       <axis xyz="0 0 1"/>
       <limit lower="${-PI/2}" upper="${PI/2}" effort="20" velocity="1"/>
     </joint>

     <link name="right_thigh">
       <visual>
         <geometry>
           <cylinder radius="0.07" length="0.5"/>
         </geometry>
         <material name="yellow"/>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.07" length="0.5"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="2"/>
         <origin xyz="0 0 -0.25"/>
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <joint name="right_knee_joint" type="revolute">
       <parent link="right_thigh"/>
       <child link="right_shin"/>
       <origin xyz="0 0 -0.5" rpy="0 0 0"/>
       <axis xyz="0 0 1"/>
       <limit lower="0" upper="${PI/2}" effort="20" velocity="1"/>
     </joint>

     <link name="right_shin">
       <visual>
         <geometry>
           <cylinder radius="0.06" length="0.45"/>
         </geometry>
         <material name="yellow"/>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.06" length="0.45"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1.5"/>
         <origin xyz="0 0 -0.225"/>
         <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <joint name="right_ankle_joint" type="revolute">
       <parent link="right_shin"/>
       <child link="right_foot"/>
       <origin xyz="0 0 -0.45" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="${-PI/4}" upper="${PI/4}" effort="10" velocity="1"/>
     </joint>

     <link name="right_foot">
       <visual>
         <geometry>
           <box size="0.2 0.1 0.05"/>
         </geometry>
         <material name="black"/>
       </visual>
       <collision>
         <geometry>
           <box size="0.2 0.1 0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
       </inertial>
     </link>
   </robot>
   ```

3. **Create Materials Definition**:
   ```xml
   <!-- humanoid_robot_description/urdf/materials.urdf.xacro -->
   <?xml version="1.0"?>
   <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
     <material name="blue">
       <color rgba="0.0 0.0 0.8 1.0"/>
     </material>
     <material name="green">
       <color rgba="0.0 0.8 0.0 1.0"/>
     </material>
     <material name="red">
       <color rgba="0.8 0.0 0.0 1.0"/>
     </material>
     <material name="white">
       <color rgba="1.0 1.0 1.0 1.0"/>
     </material>
     <material name="gray">
       <color rgba="0.5 0.5 0.5 1.0"/>
     </material>
     <material name="black">
       <color rgba="0.0 0.0 0.0 1.0"/>
     </material>
     <material name="yellow">
       <color rgba="1.0 1.0 0.0 1.0"/>
     </material>
   </robot>
   ```

4. **Create a Launch File to Visualize the Robot**:
   ```xml
   <!-- humanoid_robot_description/launch/display_robot.launch.py -->
   import launch
   import launch.actions
   import launch.substitutions
   import launch_ros.actions
   from pathlib import Path


   def generate_launch_description():
       pkg_share = Path(__file__).resolve().parent.parent
       urdf_file = pkg_share / 'urdf' / 'humanoid_robot.urdf'

       robot_state_publisher_node = launch_ros.actions.Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           parameters=[{'robot_description': open(urdf_file).read()}]
       )

       joint_state_publisher_gui_node = launch_ros.actions.Node(
           package='joint_state_publisher_gui',
           executable='joint_state_publisher_gui'
       )

       rviz_node = launch_ros.actions.Node(
           package='rviz2',
           executable='rviz2',
           arguments=['-d', str(pkg_share / 'config' / 'display_robot.rviz')]
       )

       return launch.LaunchDescription([
           robot_state_publisher_node,
           joint_state_publisher_gui_node,
           rviz_node
       ])
   ```

5. **Create a Configuration File for RViz**:
   ```yaml
   # humanoid_robot_description/config/display_robot.rviz
   Panels:
     - Class: rviz_common/Displays
       Name: Displays
       Property Tree Widget:
         Expanded:
           - /Global Options1
           - /Status1
           - /RobotModel1
         Splitter Ratio: 0.5
     - Class: rviz_common/Selection
       Name: Selection
     - Class: rviz_common/Tool Properties
       Expanded:
         - /2D Goal Pose1
         - /Publish Point1
       Name: Tool Properties
       Splitter Ratio: 0.5886790156364441
     - Class: rviz_common/Views
       Expanded:
         - /Current View1
       Name: Views
       Splitter Ratio: 0.5
     - Class: rviz_common/Time
       Experimental: false
       Name: Time
       SyncMode: 0
       SyncSource: ""
   Visualization Manager:
     Class: ""
     Displays:
       - Alpha: 0.5
         Cell Size: 1
         Class: rviz_default_plugins/Grid
         Color: 160; 160; 164
         Enabled: true
         Line Style:
           Line Width: 0.029999999329447746
           Value: Lines
         Name: Grid
         Normal Cell Count: 0
         Offset:
           X: 0
           Y: 0
           Z: 0
         Plane: XY
         Plane Cell Count: 10
         Reference Frame: <Fixed Frame>
         Value: true
       - Alpha: 1
         Class: rviz_default_plugins/RobotModel
         Collision Enabled: false
         Description File: ""
         Description Source: Topic
         Description Topic:
           Depth: 5
           Durability Policy: Volatile
           History Policy: Keep Last
           Reliability Policy: Reliable
           Value: /robot_description
         Enabled: true
         Links:
           All Links Enabled: true
           Expand Joint Details: false
           Expand Link Details: false
           Expand Tree: false
           Link Tree Style: Links in Alphabetic Order
           base_link:
             Alpha: 1
             Show Axes: false
             Show Trail: false
             Value: true
         Mass Properties:
           Inertia: false
           Mass: false
         Name: RobotModel
         TF Prefix: ""
         Update Interval: 0
         Value: true
         Visual Enabled: true
     Enabled: true
     Global Options:
       Background Color: 48; 48; 48
       Fixed Frame: base_link
       Frame Rate: 30
     Name: root
     Tools:
       - Class: rviz_default_plugins/Interact
         Hide Inactive Objects: true
       - Class: rviz_default_plugins/MoveCamera
       - Class: rviz_default_plugins/Select
       - Class: rviz_default_plugins/FocusCamera
       - Class: rviz_default_plugins/Measure
         Line color: 128; 128; 0
       - Class: rviz_default_plugins/SetInitialPose
         Topic:
           Depth: 5
           Durability Policy: Volatile
           History Policy: Keep Last
           Reliability Policy: Reliable
           Value: /initialpose
       - Class: rviz_default_plugins/SetGoal
         Topic:
           Depth: 5
           Durability Policy: Volatile
           History Policy: Keep Last
           Reliability Policy: Reliable
           Value: /goal_pose
       - Class: rviz_default_plugins/PublishPoint
         Single click: true
         Topic:
           Depth: 5
           Durability Policy: Volatile
           History Policy: Keep Last
           Reliability Policy: Reliable
           Value: /clicked_point
     Transformation:
       Current:
         Class: rviz_default_plugins/TF
     Value: true
     Views:
       Current:
         Class: rviz_default_plugins/Orbit
         Distance: 3.1744678020477295
         Enable Stereo Rendering:
           Stereo Eye Separation: 0.05999999865889549
           Stereo Focal Distance: 1
           Swap Stereo Eyes: false
           Value: false
         Focal Point:
           X: 0.04100522771477699
           Y: -0.1556701362133026
           Z: 0.5330824851989746
         Focal Shape Fixed Size: true
         Focal Shape Size: 0.05000000074505806
         Invert Z Axis: false
         Name: Current View
         Near Clip Distance: 0.009999999776482582
         Pitch: 0.32539838552474976
         Target Frame: <Fixed Frame>
         Value: Orbit (rviz)
         Yaw: 5.19857931137085
       Saved: ~
   Window Geometry:
     Displays:
       collapsed: false
     Height: 846
     Hide Left Dock: false
     Hide Right Dock: false
     QMainWindow State: 000000ff00000000fd000000040000000000000156000002b4fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002b4000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002b4fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002b4000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d006501000000000000045000000000000000000000023b000002b400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
     Width: 1200
     X: 72
     Y: 60
   ```

6. **Test the URDF Model**:
   ```bash
   # Launch the robot visualization
   ros2 launch humanoid_robot_description display_robot.launch.py
   ```

**Expected Outcome**: A complete URDF model of a humanoid robot that can be visualized in RViz with all necessary joints and links properly defined.

**Learning Points**:
- Understanding how to create URDF models for complex robots
- Learning to define joints, links, and physical properties
- Recognizing the importance of proper kinematic chains in robot modeling

## Resources and Further Learning

- [ROS2 Documentation](https://docs.ros.org/en/rolling/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [Robotics Stack Exchange](https://robotics.stackexchange.com/)
- [ROS2 Actions and Services](https://docs.ros.org/en/rolling/Concepts/About-Actions.html)

Now let's explore the core concepts of ROS 2 that make it function as a robot's nervous system.

import ContentPersonalization from '@site/src/components/UserAuth/ContentPersonalization';
import UrduTranslation from '@site/src/components/Translation/UrduTranslationEnhanced';

<ContentPersonalization title="Introduction to the Robotic Nervous System" />

<div style={{marginTop: '20px'}}>
  <UrduTranslation contentId="module1-intro" />
</div>