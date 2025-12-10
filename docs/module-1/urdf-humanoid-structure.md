---
title: URDF for Humanoid Robot Structure
sidebar_position: 5
description: Understanding how to represent humanoid robot structure using URDF
---

# URDF for Humanoid Robot Structure

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links (rigid parts), joints (connections between links), and other properties like inertial parameters, visual meshes, and collision properties.

For humanoid robots, URDF is essential to define the robot's structure, which is critical for simulation, visualization, motion planning, and control.

## Core Components of URDF

### Links
Links represent rigid bodies in the robot. They define the physical properties of each part of the robot, including:
- **Visual properties**: How the link appears in visualizations
- **Collision properties**: How the link interacts with other objects in simulation
- **Inertial properties**: Mass, center of mass, and moments of inertia for physics simulation

### Joints
Joints connect links and define how they can move relative to each other. URDF supports several joint types:
- **Fixed**: No movement between links (0 DOF)
- **Revolute**: Rotational movement around a single axis (1 DOF), with position limits
- **Continuous**: Like revolute but without position limits
- **Prismatic**: Linear sliding movement along a single axis (1 DOF), with position limits
- **Floating**: Free movement in all 6 DOF
- **Planar**: Movement in a plane (2 DOF)

## URDF for Humanoid Robots

Humanoid robots require special attention in their URDF descriptions because of their complex structure with multiple limbs and degrees of freedom. Here's what makes humanoid URDFs unique:

### Complex Kinematic Chains
Humanoid robots have multiple kinematic chains (arms, legs) that all connect to a central body (torso). This creates a tree structure in the URDF with the torso as the root.

### Balance and Stability Considerations
The inertial properties in humanoid URDFs are crucial for maintaining balance in simulation and control. The center of mass of the entire robot and its individual parts must be accurately represented.

### Control Interface Requirements
Humanoid robots typically require more sophisticated controllers, which need accurate joint definitions with appropriate limits and dynamics parameters.

## Sample Humanoid URDF Structure

The following is a simplified representation of the humanoid structure in our example:

```xml
<robot name="simple_humanoid">
  <!-- Links -->
  <link name="base_link">...</link>
  <link name="torso">...</link>
  <link name="head">...</link>
  <link name="left_upper_arm">...</link>
  <link name="left_lower_arm">...</link>
  <!-- Additional links for right arm, legs, etc. -->
  
  <!-- Joints -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.25"/>
  </joint>
  
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
  <!-- Additional joints... -->
</robot>
```

This example shows the torso connected to the left arm via a revolute shoulder joint, which allows rotation around the y-axis (front-to-back movement of the arm).

## How URDF Connects to ROS 2 Controllers

URDF models are used by ROS 2 controllers to understand the robot's structure. Controllers like joint_state_controller and effort_controllers use the URDF to:

1. Know which joints exist on the robot
2. Understand the kinematic structure (which joints affect which parts)
3. Apply appropriate control algorithms based on the physical properties

The joint names in the URDF must match those used in controller configurations for the system to work properly.

## Best Practices for Humanoid URDF Design

1. **Accurate Inertial Properties**: Ensure mass, center of mass, and inertia values are as close to real values as possible for accurate simulation.

2. **Proper Joint Limits**: Set realistic limits on joint positions, velocities, and efforts based on the actual robot capabilities.

3. **Collision Avoidance**: Design visual and collision geometry that accurately represents the physical robot to prevent self-collisions and environmental collisions.

4. **Consistent Naming**: Use consistent naming conventions for joints and links to make it easier to write controllers and interface code.

5. **Modular Design**: Structure your URDF so that components can be easily modified or replaced without affecting the whole model.

6. **Xacro for Complex Models**: For complex humanoid robots, consider using Xacro (XML Macros) to make your URDF more maintainable by allowing parameterization, macros, and includes.

## Running URDF Examples

To visualize the URDF files we've created, you can use ROS 2 tools like RViz:

1. Source ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. To check if your URDF is syntactically correct:
   ```bash
   # Install check_urdf if you don't have it
   sudo apt-get install ros-humble-urdf-tutorial
   
   # Check your URDF file
   check_urdf path/to/your/robot.urdf
   ```

3. To visualize the robot in RViz:
   ```bash
   # Launch RViz with robot state publisher
   ros2 launch urdf_tutorial display.launch.py model:=path/to/your/robot.urdf
   ```

To run the Python URDF loader example:
```bash
cd path/to/urdf/examples
python3 urdf_loader.py
```

This will parse the URDF file and show you its structure.