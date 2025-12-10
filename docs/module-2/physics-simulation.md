---
sidebar_position: 2
title: Physics Simulation Setup
---

# Physics Simulation in Gazebo

This section covers the setup and configuration of accurate physics simulation in Gazebo, ensuring realistic behavior that matches real-world physics.

## Understanding Physics Parameters

Physics simulation in Gazebo is governed by several key parameters:

- **Gravity**: The acceleration due to gravity, typically 9.81 m/s² on Earth
- **Friction**: Coefficients that define how objects interact with surfaces
- **Collision Detection**: Algorithms that determine when objects intersect
- **Solver Settings**: Parameters that control how physics equations are solved

## Setting Up Basic Physics

The basic physics world includes:

```xml
<physics name="basic_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.81</gravity>
</physics>
```

This configuration ensures:
- Accurate Earth gravity simulation
- High-frequency updates (1000 Hz) for realistic responses
- Small time steps for stable computation

## Creating Physics Models

Physics models define how objects behave. For example, a basic model might define:

- Mass and inertia properties
- Collision shapes
- Friction and restitution coefficients
- Joint constraints

## Validation

Ensure your physics simulation is accurate by testing:
- Gravity acceleration (objects should fall at ~9.81 m/s²)
- Collision responses (objects should bounce/stop correctly)
- Momentum conservation
- Energy dissipation

## Troubleshooting Physics Issues

Common physics issues include:
- Object interpenetration (increase collision margin)
- Unstable simulation (decrease step size)
- Excessive jittering (adjust solver parameters)

## Practical Assignment - Physics Simulation Configuration

### Assignment 1: Tuning Physics Parameters for Humanoid Stability

**Objective**: Configure physics parameters for stable humanoid robot simulation.

**Detailed Steps**:
1. **Create a Physics Configuration File for Humanoid**:
   ```xml
   <!-- config/humanoid_physics.gazebo -->
   <gazebo>
     <world name="humanoid_world">
       <physics type="ode">
         <!-- For humanoid simulation, use smaller step size for better stability -->
         <max_step_size>0.0005</max_step_size>
         <!-- Update at high frequency for accurate physics -->
         <real_time_update_rate>2000.0</real_time_update_rate>
         <gravity>0 0 -9.8</gravity>

         <ode>
           <solver>
             <!-- Use quick solver for real-time performance -->
             <type>quick</type>
             <!-- Increase iterations for better constraint resolution -->
             <iters>200</iters>
             <!-- SOR parameter for convergence -->
             <sor>1.2</sor>
           </solver>
           <constraints>
             <!-- Constraint Force Mixing - avoid zero for stability -->
             <cfm>1e-5</cfm>
             <!-- Error Reduction Parameter - 0.2-0.8 for humanoid stability -->
             <erp>0.4</erp>
             <!-- Limit velocity correction to prevent excessive forces -->
             <contact_max_correcting_vel>10.0</contact_max_correcting_vel>
             <!-- Surface layer thickness for contact stability -->
             <contact_surface_layer>0.001</contact_surface_layer>
           </constraints>
         </ode>
       </physics>
     </world>
   </gazebo>
   ```

2. **Create a Humanoid Model with Physics Properties**:
   ```xml
   <!-- urdf/humanoid_with_physics.urdf.xacro -->
   <?xml version="1.0" ?>
   <robot name="stable_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
     <!-- Include Gazebo plugins -->
     <xacro:include filename="$(find gazebo_ros)/urdf/spawn.urdf.xacro" />

     <!-- Define properties -->
     <xacro:property name="M_PI" value="3.14159"/>

     <!-- Base link with appropriate mass for humanoid -->
     <link name="base_link">
       <visual>
         <origin xyz="0 0 0.5" rpy="0 0 0"/>
         <geometry>
           <box size="0.2 0.2 1.0"/>
         </geometry>
         <material name="light_grey">
           <color rgba="0.7 0.7 0.7 1.0"/>
         </material>
       </visual>
       <collision>
         <origin xyz="0 0 0.5" rpy="0 0 0"/>
         <geometry>
           <box size="0.2 0.2 1.0"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="15.0"/>
         <origin xyz="0 0 0.5" rpy="0 0 0"/>
         <inertia ixx="0.3" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.1"/>
       </inertial>
     </link>

     <!-- Torso link -->
     <joint name="torso_joint" type="fixed">
       <parent link="base_link"/>
       <child link="torso"/>
       <origin xyz="0 0 0.8" rpy="0 0 0"/>
     </joint>

     <link name="torso">
       <visual>
         <geometry>
           <box size="0.3 0.2 0.5"/>
         </geometry>
         <material name="dark_grey">
           <color rgba="0.3 0.3 0.3 1.0"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.3 0.2 0.5"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="8.0"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
       </inertial>
     </link>

     <!-- Head -->
     <joint name="neck_joint" type="revolute">
       <parent link="torso"/>
       <child link="head"/>
       <origin xyz="0 0 0.3" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="${-M_PI/3}" upper="${M_PI/3}" effort="10" velocity="1"/>
       <!-- Dynamics parameters for smooth movement -->
       <dynamics damping="0.5" friction="0.1"/>
     </joint>

     <link name="head">
       <visual>
         <geometry>
           <sphere radius="0.15"/>
         </geometry>
         <material name="skin_color">
           <color rgba="0.8 0.6 0.4 1.0"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <sphere radius="0.15"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="2.0"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <!-- Left leg -->
     <joint name="left_hip_joint" type="revolute">
       <parent link="base_link"/>
       <child link="left_thigh"/>
       <origin xyz="-0.05 0 0" rpy="0 0 0"/>
       <axis xyz="0 0 1"/>
       <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="20" velocity="1.5"/>
       <dynamics damping="1.0" friction="0.2"/>
     </joint>

     <link name="left_thigh">
       <visual>
         <geometry>
           <cylinder length="0.5" radius="0.08"/>
         </geometry>
         <material name="blue">
           <color rgba="0 0 1 1.0"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.5" radius="0.08"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="3.0"/>
         <origin xyz="0 0 -0.25"/>
         <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.001"/>
       </inertial>
     </link>

     <!-- Physics properties in Gazebo -->
     <gazebo reference="base_link">
       <mu1>0.8</mu1>
       <mu2>0.8</mu2>
       <kp>1000000.0</kp>
       <kd>100.0</kd>
       <material>Gazebo/Grey</material>
     </gazebo>

     <gazebo reference="torso">
       <mu1>0.8</mu1>
       <mu2>0.8</mu2>
       <kp>1000000.0</kp>
       <kd>100.0</kd>
       <material>Gazebo/Grey</material>
     </gazebo>

     <gazebo reference="head">
       <mu1>0.8</mu1>
       <mu2>0.8</mu2>
       <kp>1000000.0</kp>
       <kd>100.0</kd>
       <material>Gazebo/White</material>
     </gazebo>

     <gazebo reference="left_thigh">
       <mu1>0.8</mu1>
       <mu2>0.8</mu2>
       <kp>1000000.0</kp>
       <kd>100.0</kd>
       <material>Gazebo/Blue</material>
     </gazebo>
   </robot>
   ```

3. **Create a Launch File for Physics Testing**:
   ```xml
   <!-- launch/physics_test.launch -->
   <launch>
     <!-- Load robot description -->
     <param name="robot_description" command="$(find xacro)/xacro '$(find your_robot_description)/urdf/humanoid_with_physics.urdf.xacro'" />

     <!-- Launch Gazebo with custom physics -->
     <include file="$(find gazebo_ros)/launch/empty_world.launch">
       <arg name="world_name" value="$(find your_robot_gazebo)/config/humanoid_physics.gazebo"/>
       <arg name="paused" value="false"/>
       <arg name="use_sim_time" value="true"/>
       <arg name="gui" value="true"/>
       <arg name="headless" value="false"/>
       <arg name="debug" value="false"/>
     </include>

     <!-- Spawn the robot -->
     <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
           args="-param robot_description -urdf -model stable_humanoid -x 0 -y 0 -z 0.5" />

     <!-- Robot state publisher -->
     <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
   </launch>
   ```

4. **Test Physics Configuration**:
   ```bash
   # Launch the physics test
   roslaunch your_robot_gazebo physics_test.launch
   ```

**Expected Outcome**: A stable humanoid robot that maintains balance in the Gazebo physics simulation.

**Learning Points**:
- Understanding physics parameters for humanoid stability
- Learning to configure appropriate mass and inertia properties
- Recognizing the impact of damping and friction settings

### Assignment 2: Implementing Contact Sensors for Humanoid Locomotion

**Objective**: Add contact sensors to the humanoid feet to detect ground contact for locomotion control.

**Detailed Steps**:
1. **Modify the Robot Model to Include Contact Sensors**:
   ```xml
   <!-- urdf/humanoid_with_contact_sensors.urdf.xacro -->
   <?xml version="1.0" ?>
   <robot name="humanoid_with_sensors" xmlns:xacro="http://www.ros.org/wiki/xacro">
     <!-- Include Gazebo plugins -->
     <xacro:include filename="$(find gazebo_ros)/urdf/spawn.urdf.xacro" />

     <!-- Properties -->
     <xacro:property name="M_PI" value="3.14159"/>

     <!-- Base link -->
     <link name="base_link">
       <inertial>
         <mass value="15.0"/>
         <origin xyz="0 0 0.5" rpy="0 0 0"/>
         <inertia ixx="0.3" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.1"/>
       </inertial>
     </link>

     <!-- Left foot -->
     <joint name="left_ankle_joint" type="fixed">
       <parent link="base_link"/>
       <child link="left_foot"/>
       <origin xyz="-0.1 0 0" rpy="0 0 0"/>
     </joint>

     <link name="left_foot">
       <visual>
         <geometry>
           <box size="0.2 0.1 0.05"/>
         </geometry>
         <material name="red">
           <color rgba="1 0 0 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.2 0.1 0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
       </inertial>
     </link>

     <!-- Add contact sensor to the left foot -->
     <gazebo reference="left_foot">
       <sensor name="left_foot_contact" type="contact">
         <always_on>true</always_on>
         <update_rate>100</update_rate>
         <contact>
           <collision>left_foot_collision</collision>
         </contact>
         <plugin name="left_foot_contact_plugin" filename="libgazebo_ros_bumper.so">
           <alwaysOn>true</alwaysOn>
           <updateRate>100.0</updateRate>
           <bumperTopicName>left_foot_bumper</bumperTopicName>
           <frameName>left_foot</frameName>
         </plugin>
       </sensor>
     </gazebo>

     <!-- Right foot -->
     <joint name="right_ankle_joint" type="fixed">
       <parent link="base_link"/>
       <child link="right_foot"/>
       <origin xyz="0.1 0 0" rpy="0 0 0"/>
     </joint>

     <link name="right_foot">
       <visual>
         <geometry>
           <box size="0.2 0.1 0.05"/>
         </geometry>
         <material name="green">
           <color rgba="0 1 0 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.2 0.1 0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
       </inertial>
     </link>

     <!-- Add contact sensor to the right foot -->
     <gazebo reference="right_foot">
       <sensor name="right_foot_contact" type="contact">
         <always_on>true</always_on>
         <update_rate>100</update_rate>
         <contact>
           <collision>right_foot_collision</collision>
         </contact>
         <plugin name="right_foot_contact_plugin" filename="libgazebo_ros_bumper.so">
           <alwaysOn>true</alwaysOn>
           <updateRate>100.0</updateRate>
           <bumperTopicName>right_foot_bumper</bumperTopicName>
           <frameName>right_foot</frameName>
         </plugin>
       </sensor>
     </gazebo>

     <!-- Gazebo materials -->
     <gazebo reference="left_foot">
       <material>Gazebo/Red</material>
     </gazebo>

     <gazebo reference="right_foot">
       <material>Gazebo/Green</material>
     </gazebo>
   </robot>
   ```

2. **Create a Contact Sensor Test Node**:
   ```python
   #!/usr/bin/env python3
   """
   Test node for contact sensors
   """
   import rospy
   from gazebo_msgs.msg import ContactsState
   from std_msgs.msg import Bool

   class ContactSensorTester:
       def __init__(self):
           rospy.init_node('contact_sensor_tester', anonymous=True)

           # Subscribe to contact sensors
           self.left_contact_sub = rospy.Subscriber('/left_foot_bumper', ContactsState, self.left_contact_callback)
           self.right_contact_sub = rospy.Subscriber('/right_foot_bumper', ContactsState, self.right_contact_callback)

           # Publishers to indicate contact state
           self.left_contact_pub = rospy.Publisher('/left_foot_contact', Bool, queue_size=1)
           self.right_contact_pub = rospy.Publisher('/right_foot_contact', Bool, queue_size=1)

           self.left_in_contact = False
           self.right_in_contact = False

           # Set up rate
           self.rate = rospy.Rate(10)  # 10 Hz

       def left_contact_callback(self, data):
           self.left_in_contact = len(data.states) > 0
           contact_msg = Bool()
           contact_msg.data = self.left_in_contact
           self.left_contact_pub.publish(contact_msg)

       def right_contact_callback(self, data):
           self.right_in_contact = len(data.states) > 0
           contact_msg = Bool()
           contact_msg.data = self.right_in_contact
           self.right_contact_pub.publish(contact_msg)

       def run(self):
           while not rospy.is_shutdown():
               rospy.loginfo(f"Left foot contact: {self.left_in_contact}, Right foot contact: {self.right_in_contact}")
               self.rate.sleep()

   if __name__ == '__main__':
       tester = ContactSensorTester()
       tester.run()
   ```

3. **Test Contact Sensors**:
   ```bash
   # Launch the robot with contact sensors
   roslaunch your_robot_gazebo contact_sensors_test.launch

   # Run the contact sensor tester
   rosrun your_robot_control contact_sensor_tester.py
   ```

**Expected Outcome**: Contact sensors on the robot feet that detect when the feet touch the ground.

**Learning Points**:
- Understanding how to add contact sensors to a humanoid robot
- Learning to process contact sensor data
- Recognizing the importance of contact feedback for locomotion

### Assignment 3: Implementing Balance Control with Physics Feedback

**Objective**: Create a simple balance controller that uses physics simulation feedback to maintain humanoid balance.

**Detailed Steps**:
1. **Create a Balance Control Node**:
   ```python
   #!/usr/bin/env python3
   """
   Balance controller for humanoid robot based on physics simulation
   """
   import rospy
   import numpy as np
   from sensor_msgs.msg import Imu
   from geometry_msgs.msg import Vector3
   from std_msgs.msg import Float64

   class BalanceController:
       def __init__(self):
           rospy.init_node('balance_controller', anonymous=True)

           # Subscribe to IMU data
           self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

           # Publishers for joint control
           self.neck_pub = rospy.Publisher('/humanoid/neck_joint_position_controller/command', Float64, queue_size=1)
           self.left_hip_pub = rospy.Publisher('/humanoid/left_hip_joint_position_controller/command', Float64, queue_size=1)
           self.right_hip_pub = rospy.Publisher('/humanoid/right_hip_joint_position_controller/command', Float64, queue_size=1)

           # PID parameters
           self.kp = 1.0  # Proportional gain
           self.ki = 0.1  # Integral gain
           self.kd = 0.05  # Derivative gain

           # Error accumulation for integral term
           self.integral_error = 0.0
           self.previous_error = 0.0

           # Target angles (balanced position)
           self.target_roll = 0.0
           self.target_pitch = 0.0

           # Control rate
           self.rate = rospy.Rate(50)  # 50 Hz

       def imu_callback(self, data):
           # Extract roll and pitch from quaternion
           # This is a simplified calculation; in practice, you might use tf.transformations
           w, x, y, z = data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z

           # Roll (rotation around x-axis)
           sinr_cosp = 2 * (w * x + y * z)
           cosr_cosp = 1 - 2 * (x * x + y * y)
           current_roll = np.arctan2(sinr_cosp, cosr_cosp)

           # Pitch (rotation around y-axis)
           sinp = 2 * (w * y - z * x)
           if np.abs(sinp) >= 1:
               current_pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
           else:
               current_pitch = np.arcsin(sinp)

           # Calculate error
           error_roll = self.target_roll - current_roll
           error_pitch = self.target_pitch - current_pitch

           # Use pitch error for balance control (simplified)
           control_output = self.pid_control(error_pitch)

           # Publish control commands
           neck_cmd = Float64()
           neck_cmd.data = -control_output * 0.5  # Scale appropriately
           self.neck_pub.publish(neck_cmd)

           # Adjust hip joints to maintain balance
           left_hip_cmd = Float64()
           right_hip_cmd = Float64()
           left_hip_cmd.data = control_output * 0.2
           right_hip_cmd.data = -control_output * 0.2
           self.left_hip_pub.publish(left_hip_cmd)
           self.right_hip_pub.publish(right_hip_cmd)

       def pid_control(self, error):
           # Calculate integral and derivative terms
           self.integral_error += error
           derivative_error = error - self.previous_error

           # Calculate PID output
           output = (self.kp * error) + (self.ki * self.integral_error) + (self.kd * derivative_error)

           # Update previous error
           self.previous_error = error

           # Limit output to reasonable range
           output = np.clip(output, -0.5, 0.5)

           return output

       def run(self):
           rospy.loginfo("Balance controller started")
           while not rospy.is_shutdown():
               self.rate.sleep()

   if __name__ == '__main__':
       controller = BalanceController()
       controller.run()
   ```

2. **Create a Launch File with Balance Control**:
   ```xml
   <!-- launch/balance_control_test.launch -->
   <launch>
     <!-- Load robot description -->
     <param name="robot_description" command="$(find xacro)/xacro '$(find your_robot_description)/urdf/humanoid_with_contact_sensors.urdf.xacro'" />

     <!-- Launch Gazebo with custom physics -->
     <include file="$(find gazebo_ros)/launch/empty_world.launch">
       <arg name="world_name" value="$(find your_robot_gazebo)/config/humanoid_physics.gazebo"/>
       <arg name="paused" value="false"/>
       <arg name="use_sim_time" value="true"/>
       <arg name="gui" value="true"/>
       <arg name="headless" value="false"/>
       <arg name="debug" value="false"/>
     </include>

     <!-- Spawn the robot -->
     <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
           args="-param robot_description -urdf -model humanoid_with_sensors -x 0 -y 0 -z 1.0" />

     <!-- Robot state publisher -->
     <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

     <!-- Balance controller -->
     <node name="balance_controller" pkg="your_robot_control" type="balance_controller.py" output="screen" />
   </launch>
   ```

3. **Test Balance Control**:
   ```bash
   # Launch the balance control test
   roslaunch your_robot_gazebo balance_control_test.launch
   ```

**Expected Outcome**: A humanoid robot that attempts to maintain balance using physics simulation feedback.

**Learning Points**:
- Understanding how to use physics simulation data for control
- Learning to implement simple feedback control in simulation
- Recognizing the relationship between physics simulation and real-world control

## Troubleshooting Physics Issues

### Robot Falls Through Ground
- **Cause**: Insufficient collision detection or improper mass/inertia
- **Solution**: Check collision geometries and mass parameters

### Robot Jittering or Unstable
- **Cause**: Poor physics parameters or high solver iterations
- **Solution**: Adjust ERP, CFM, and solver parameters

### Joint Constraints Not Working
- **Cause**: Improper joint limits or dynamics settings
- **Solution**: Verify joint configuration in URDF