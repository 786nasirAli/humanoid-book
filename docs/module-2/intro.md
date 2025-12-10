---
sidebar_position: 1
title: Introduction to Digital Twin Simulation
---

# Module 2: The Digital Twin (Gazebo & Unity)

Welcome to the Digital Twin simulation module! This module focuses on creating a complete digital twin environment that combines accurate physics simulation in Gazebo with high-fidelity visualization in Unity.

## Overview

In this module, you'll learn to:
- Set up realistic physics simulation using Gazebo's physics engine
- Create high-fidelity visualizations in Unity for human-robot interaction
- Simulate various sensors (LiDAR, Depth Cameras, IMUs) with realistic data outputs
- Build a synchronized environment that mirrors real-world physics and behavior

## Learning Objectives

By the end of this module, you will be able to:
1. Configure and run accurate physics simulations in Gazebo
2. Create interactive visualization environments in Unity
3. Implement realistic sensor simulation with appropriate noise models
4. Establish synchronization between Gazebo and Unity environments

## Prerequisites

Before starting this module, ensure you have:
- Completed Module 1 (The Robotic Nervous System)
- Access to a machine with NVIDIA RTX-enabled GPU
- ROS 2 (Foxy/Fortress) installed and configured
- Unity Hub with Unity 2021.3 LTS installed

## Getting Started

Let's begin by understanding the physics simulation setup in the next section.

## Practical Assignment - Getting Started with Digital Twin Simulation

### Assignment 1: Setting Up Your First Digital Twin Environment

**Objective**: Create a basic simulation environment with a humanoid robot in Gazebo.

**Detailed Steps**:
1. **Create a Simple Robot Model**:
   ```xml
   <!-- simple_humanoid.urdf.xacro -->
   <?xml version="1.0" ?>
   <robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
     <!-- Include Gazebo plugins -->
     <xacro:include filename="$(find gazebo_ros)/urdf/spawn.urdf.xacro" />

     <!-- Base Link -->
     <link name="base_link">
       <visual>
         <geometry>
           <box size="0.3 0.3 0.3"/>
         </geometry>
         <material name="blue">
           <color rgba="0 0 1 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.3 0.3 0.3"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="10"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
       </inertial>
     </link>

     <!-- Head -->
     <joint name="neck_joint" type="revolute">
       <parent link="base_link"/>
       <child link="head"/>
       <origin xyz="0 0 0.3" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
     </joint>

     <link name="head">
       <visual>
         <geometry>
           <sphere radius="0.15"/>
         </geometry>
         <material name="white">
           <color rgba="1 1 1 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <sphere radius="0.15"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="2"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <!-- Left Arm -->
     <joint name="left_shoulder_joint" type="revolute">
       <parent link="base_link"/>
       <child link="left_upper_arm"/>
       <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
       <axis xyz="1 0 0"/>
       <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
     </joint>

     <link name="left_upper_arm">
       <visual>
         <geometry>
           <cylinder radius="0.05" length="0.3"/>
         </geometry>
         <material name="red">
           <color rgba="1 0 0 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.05" length="0.3"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1"/>
         <origin xyz="0 0 -0.15"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <!-- Gazebo plugin for physics -->
     <gazebo reference="base_link">
       <material>Gazebo/Blue</material>
     </gazebo>

     <gazebo reference="head">
       <material>Gazebo/White</material>
     </gazebo>

     <gazebo reference="left_upper_arm">
       <material>Gazebo/Red</material>
     </gazebo>
   </robot>
   ```

2. **Create a Gazebo Launch File**:
   ```xml
   <!-- launch/humanoid_simulation.launch -->
   <launch>
     <!-- Set the robot description parameter -->
     <param name="robot_description" command="$(find xacro)/xacro '$(find your_robot_description)/urdf/simple_humanoid.urdf.xacro'" />

     <!-- Spawn the robot in Gazebo -->
     <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model simple_humanoid" />

     <!-- Start the Gazebo simulation -->
     <include file="$(find gazebo_ros)/launch/empty_world.launch">
       <arg name="paused" value="false"/>
       <arg name="use_sim_time" value="true"/>
       <arg name="gui" value="true"/>
       <arg name="headless" value="false"/>
       <arg name="debug" value="false"/>
     </include>

     <!-- Robot state publisher -->
     <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
   </launch>
   ```

3. **Test the Simulation**:
   ```bash
   # Launch the simulation
   roslaunch your_robot_gazebo humanoid_simulation.launch
   ```

4. **Control the Robot**:
   ```python
   # Simple joint controller node
   #!/usr/bin/env python3
   import rospy
   from std_msgs.msg import Float64
   import math
   import time

   class SimpleJointController:
       def __init__(self):
           rospy.init_node('simple_joint_controller', anonymous=True)

           # Create publishers for each joint
           self.neck_pub = rospy.Publisher('/simple_humanoid/neck_joint_position_controller/command',
                                          Float64, queue_size=1)
           self.shoulder_pub = rospy.Publisher('/simple_humanoid/left_shoulder_joint_position_controller/command',
                                              Float64, queue_size=1)

           # Control rate
           self.rate = rospy.Rate(10)  # 10 Hz

       def move_joints(self):
           while not rospy.is_shutdown():
               # Create oscillating motion
               neck_pos = 0.2 * math.sin(rospy.Time.now().to_sec())
               shoulder_pos = 0.5 * math.sin(rospy.Time.now().to_sec() * 0.5)

               # Publish positions
               self.neck_pub.publish(neck_pos)
               self.shoulder_pub.publish(shoulder_pos)

               self.rate.sleep()

   if __name__ == '__main__':
       controller = SimpleJointController()
       controller.move_joints()
   ```

**Expected Outcome**: A simple humanoid robot model loaded in Gazebo with basic joint control.

**Learning Points**:
- Understanding URDF/Xacro for robot modeling
- Learning to launch Gazebo simulations
- Recognizing the basics of joint control

### Assignment 2: Understanding Physics Simulation Parameters

**Objective**: Configure physics parameters for accurate humanoid robot simulation.

**Detailed Steps**:
1. **Create a Physics Configuration File**:
   ```xml
   <!-- config/physics_world.gazebo -->
   <gazebo>
     <world name="default">
       <physics type="ode">
         <max_step_size>0.001</max_step_size>
         <real_time_factor>1.0</real_time_factor>
         <real_time_update_rate>1000.0</real_time_update_rate>
         <gravity>0 0 -9.8</gravity>

         <ode>
           <solver>
             <type>quick</type>
             <iters>10</iters>
             <sor>1.3</sor>
           </solver>
           <constraints>
             <cfm>0.0</cfm>
             <erp>0.2</erp>
             <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
             <contact_surface_layer>0.001</contact_surface_layer>
           </constraints>
         </ode>
       </physics>
     </world>
   </gazebo>
   ```

2. **Implement Joint Dynamics**:
   ```xml
   <!-- Add to URDF for more realistic joint dynamics -->
   <joint name="neck_joint" type="revolute">
     <parent link="base_link"/>
     <child link="head"/>
     <origin xyz="0 0 0.3" rpy="0 0 0"/>
     <axis xyz="0 1 0"/>
     <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
     <!-- Dynamics parameters for realistic movement -->
     <dynamics damping="0.1" friction="0.01"/>
   </joint>
   ```

3. **Test Physics Parameters**:
   ```bash
   # Launch with custom physics parameters
   roslaunch gazebo_ros empty_world.launch world_name:=config/physics_world.gazebo
   ```

**Expected Outcome**: Understanding how physics parameters affect robot simulation.

**Learning Points**:
- Understanding Gazebo physics configuration
- Learning to tune parameters for realistic simulation
- Recognizing the impact of damping and friction

### Assignment 3: Advanced Environment Setup

**Objective**: Create an advanced simulation environment with multiple objects and sensors.

**Detailed Steps**:
1. **Create an Environment World File**:
   ```xml
   <!-- worlds/humanoid_test_world.world -->
   <?xml version="1.0" ?>
   <sdf version="1.6">
     <world name="humanoid_test_world">
       <include>
         <uri>model://sun</uri>
       </include>

       <include>
         <uri>model://ground_plane</uri>
       </include>

       <!-- Add a table -->
       <model name="table">
         <pose>2 2 0.5 0 0 0</pose>
         <link name="table_bottom">
           <collision name="collision">
             <geometry>
               <box>
                 <size>1.0 0.8 0.8</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>1.0 0.8 0.8</size>
               </box>
             </geometry>
             <material>
               <ambient>0.8 0.6 0.2 1</ambient>
               <diffuse>0.8 0.6 0.2 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>10</mass>
             <inertia>
               <ixx>1.0</ixx>
               <ixy>0.0</ixy>
               <ixz>0.0</ixz>
               <iyy>1.0</iyy>
               <iyz>0.0</iyz>
               <izz>1.0</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <!-- Add a ball that the robot can interact with -->
       <model name="interaction_ball">
         <pose>1.5 1.5 1 0 0 0</pose>
         <link name="ball_link">
           <collision name="collision">
             <geometry>
               <sphere>
                 <radius>0.1</radius>
               </sphere>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <sphere>
                 <radius>0.1</radius>
               </sphere>
             </geometry>
             <material>
               <ambient>1 0 0 1</ambient>
               <diffuse>1 0 0 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>0.1</mass>
             <inertia>
               <ixx>0.0001</ixx>
               <ixy>0.0</ixy>
               <ixz>0.0</ixz>
               <iyy>0.0001</iyy>
               <iyz>0.0</iyz>
               <izz>0.0001</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <!-- Physics parameters -->
       <physics type="ode">
         <max_step_size>0.001</max_step_size>
         <real_time_factor>1.0</real_time_factor>
         <real_time_update_rate>1000.0</real_time_update_rate>
       </physics>
     </world>
   </sdf>
   ```

2. **Create a Launch File for the World**:
   ```xml
   <!-- launch/advanced_simulation.launch -->
   <launch>
     <!-- Load robot description -->
     <param name="robot_description" command="$(find xacro)/xacro '$(find your_robot_description)/urdf/simple_humanoid.urdf.xacro'" />

     <!-- Launch Gazebo with custom world -->
     <include file="$(find gazebo_ros)/launch/empty_world.launch">
       <arg name="world_name" value="$(find your_robot_gazebo)/worlds/humanoid_test_world.world"/>
       <arg name="paused" value="false"/>
       <arg name="use_sim_time" value="true"/>
       <arg name="gui" value="true"/>
       <arg name="headless" value="false"/>
       <arg name="debug" value="false"/>
     </include>

     <!-- Spawn the robot -->
     <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
           args="-param robot_description -urdf -model simple_humanoid -x 0 -y 0 -z 0.5" />

     <!-- Robot state publisher -->
     <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
   </launch>
   ```

3. **Test Advanced Environment**:
   ```bash
   # Launch the advanced simulation
   roslaunch your_robot_gazebo advanced_simulation.launch
   ```

**Expected Outcome**: A complex simulation environment with multiple objects that the robot can interact with.

**Learning Points**:
- Understanding how to create complex simulation worlds
- Learning to include multiple objects in simulations
- Recognizing the importance of physics parameters in multi-object simulations

## Resources and Further Learning

- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [ROS URDF Documentation](http://wiki.ros.org/urdf)
- [Xacro Documentation](http://wiki.ros.org/xacro)

import ContentPersonalization from '@site/src/components/UserAuth/ContentPersonalization';
import UrduTranslation from '@site/src/components/Translation/UrduTranslationEnhanced';

<ContentPersonalization title="Introduction to Digital Twin Simulation" />

<div style={{marginTop: '20px'}}>
  <UrduTranslation contentId="module2-intro" />
</div>