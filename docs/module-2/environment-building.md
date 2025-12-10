---
sidebar_position: 5
title: Creating Digital Twin Environments for Humanoid Robots
---

# Creating Digital Twin Environments for Humanoid Robots

This section provides hands-on experience creating comprehensive simulation environments that serve as digital twins for humanoid robots. You'll learn to build both physics-accurate Gazebo environments and visually-rich Unity environments.

## Environment Design Principles for Humanoid Robots

Effective humanoid simulation environments should:

- **Match Humanoid Scale**: Accommodate human-sized dimensions appropriately
- **Include Human-Relevant Obstacles**: Furniture, doorways, stairs, etc.
- **Support Bipedal Navigation**: Flat surfaces, appropriate step heights
- **Enable Interaction Scenarios**: Objects for manipulation, interaction spaces
- **Allow Repeatable Testing**: Consistent conditions for validation

## Creating Humanoid-Focused Indoor Environments

### Basic Indoor Room Setup

When designing environments for humanoid robots, it's important to create spaces with appropriate human-scale features:

```xml
<!-- worlds/humanoid_indoor.world -->
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_indoor">
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Interior walls -->
    <model name="north_wall">
      <pose>0 5 1 0 0 0</pose>
      <link name="wall_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
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

    <model name="east_wall">
      <pose>5 0 1 0 0 1.5708</pose> <!-- 90-degree rotation -->
      <link name="wall_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
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

    <!-- Doorway for humanoid passage -->
    <model name="doorway_obstacle">
      <pose>0 3.5 0.5 0 0 0</pose>
      <!-- Left doorpost -->
      <link name="left_doorpost">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 0.1 1.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.1 1.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>20</mass>
          <inertia>
            <ixx>0.5</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.5</iyy>
            <iyz>0.0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>

      <!-- Right doorpost -->
      <link name="right_doorpost">
        <pose>1.5 0 0 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 0.1 1.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.1 1.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>20</mass>
          <inertia>
            <ixx>0.5</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.5</iyy>
            <iyz>0.0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Humanoid-appropriate furniture -->
    <model name="humanoid_table">
      <pose>2 2 0.75 0 0 0</pose>
      <link name="table_top">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.2 0.6 0.05</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.2 0.6 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.5 0.3 1</ambient>
            <diffuse>0.7 0.5 0.3 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>15</mass>
          <inertia>
            <ixx>0.8</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.8</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>

      <!-- Table legs -->
      <link name="leg1">
        <pose>-0.5 -0.25 0.375 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.75</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.75</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>2</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.01</iyy>
            <iyz>0.0</iyz>
            <izz>0.0025</izz>
          </inertia>
        </inertial>
      </link>

      <link name="leg2">
        <pose>0.5 -0.25 0.375 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.75</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.75</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>2</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.01</iyy>
            <iyz>0.0</iyz>
            <izz>0.0025</izz>
          </inertia>
        </inertial>
      </link>

      <link name="leg3">
        <pose>-0.5 0.25 0.375 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.75</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.75</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>2</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.01</iyy>
            <iyz>0.0</iyz>
            <izz>0.0025</izz>
          </inertia>
        </inertial>
      </link>

      <link name="leg4">
        <pose>0.5 0.25 0.375 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.75</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.75</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>2</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.01</iyy>
            <iyz>0.0</iyz>
            <izz>0.0025</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Chair for humanoid interaction -->
    <model name="humanoid_chair">
      <pose>0 -1 0.45 0 0 1.5708</pose> <!-- 90 degrees rotated -->
      <link name="seat">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.05</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>8</mass>
          <inertia>
            <ixx>0.2</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.2</iyy>
            <iyz>0.0</iyz>
            <izz>0.2</izz>
          </inertia>
        </inertial>
      </link>

      <!-- Back support -->
      <link name="back_support">
        <pose>0 0 0.4 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.05 0.5 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.05 0.5 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>2</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.1</iyy>
            <iyz>0.0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>

      <!-- Chair legs -->
      <link name="leg1">
        <pose>-0.2 -0.2 0.225 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.03</radius>
              <length>0.45</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.03</radius>
              <length>0.45</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.01</iyy>
            <iyz>0.0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Creating Unity Scene Synchronization

Unity scenes mirror the Gazebo environment for high-fidelity visualization:

### Unity Scene Setup
- Configure coordinate system alignment (ROS uses ENU, Unity uses left-handed)
- Set up lighting to match Gazebo environment
- Create terrain meshes that correspond to Gazebo models
- Implement object instantiation that matches Gazebo entities

### Unity Scene Components
- Main camera with adjustable navigation controls
- Lighting system with shadow settings matching Gazebo
- Material and texture mapping for visual consistency
- Physics visualization with Unity's physics engine
- Interactive controls for human-in-the-loop validation

### Synchronization Strategies
To maintain synchronization between environments:

- Implement transform synchronization with interpolation for smooth visual updates
- Establish ROS communication bridges using unity-ros-bridge
- Implement timing alignment to account for transmission delays
- Create state validation nodes to detect desync issues

## Outdoor Environments for Humanoid Navigation

Outdoor environments for humanoid robots should include specific elements that test bipedal locomotion:

1. **Create a Humanoid-Specific Outdoor World**:
   ```xml
   <!-- worlds/humanoid_outdoor.world -->
   <?xml version="1.0" ?>
   <sdf version="1.6">
     <world name="humanoid_outdoor">
       <!-- Include standard models -->
       <include>
         <uri>model://ground_plane</uri>
       </include>
       <include>
         <uri>model://sun</uri>
       </include>

       <!-- Terrain with slight variations for walking -->
       <model name="terrain_patch_1">
         <pose>0 0 0 0 0 0</pose>
         <static>true</static>
         <link name="terrain_link">
           <collision name="collision">
             <geometry>
               <mesh>
                 <uri>file://meshes/slight_slope.dae</uri>
               </mesh>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <mesh>
                 <uri>file://meshes/slight_slope.dae</uri>
               </mesh>
             </geometry>
             <material>
               <ambient>0.4 0.6 0.3 1</ambient>
               <diffuse>0.4 0.6 0.3 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>1000</mass>
             <inertia>
               <ixx>100</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>100</iyy>
               <iyz>0</iyz>
               <izz>100</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <!-- Stepped surface for testing stair navigation -->
       <model name="stepped_surface">
         <pose>5 0 0 0 0 0</pose>
         <link name="step_1">
           <collision name="collision">
             <geometry>
               <box>
                 <size>2 2 0.15</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>2 2 0.15</size>
               </box>
             </geometry>
             <material>
               <ambient>0.6 0.6 0.6 1</ambient>
               <diffuse>0.6 0.6 0.6 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>50</mass>
             <inertia>
               <ixx>20</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>20</iyy>
               <iyz>0</iyz>
               <izz>40</izz>
             </inertia>
           </inertial>
         </link>

         <link name="step_2">
           <pose>0 0 0.3 0 0 0</pose>
           <collision name="collision">
             <geometry>
               <box>
                 <size>2 2 0.15</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>2 2 0.15</size>
               </box>
             </geometry>
             <material>
               <ambient>0.6 0.6 0.7 1</ambient>
               <diffuse>0.6 0.6 0.7 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>50</mass>
             <inertia>
               <ixx>20</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>20</iyy>
               <iyz>0</iyz>
               <izz>40</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <!-- Obstacle course for navigation testing -->
       <model name="obstacle_1">
         <pose>-3 2 0.5 0 0 0</pose>
         <link name="obstacle_link">
           <collision name="collision">
             <geometry>
               <cylinder>
                 <radius>0.2</radius>
                 <length>1.0</length>
               </cylinder>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <cylinder>
                 <radius>0.2</radius>
                 <length>1.0</length>
               </cylinder>
             </geometry>
             <material>
               <ambient>0.8 0.3 0.3 1</ambient>
               <diffuse>0.8 0.3 0.3 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>10</mass>
             <inertia>
               <ixx>0.5</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.5</iyy>
               <iyz>0</iyz>
               <izz>0.2</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <model name="obstacle_2">
         <pose>-3 -2 0.3 0 0 0.5</pose>
         <link name="obstacle_link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.4 0.4 0.6</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.4 0.4 0.6</size>
               </box>
             </geometry>
             <material>
               <ambient>0.3 0.8 0.3 1</ambient>
               <diffuse>0.3 0.8 0.3 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>5</mass>
             <inertia>
               <ixx>0.1</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.1</iyy>
               <iyz>0</iyz>
               <izz>0.08</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <!-- Narrow passage for tight navigation -->
       <model name="narrow_passage">
         <pose>0 -5 0 0 0 0</pose>
         <!-- Left barrier -->
         <link name="left_barrier">
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.1 4 1.5</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.1 4 1.5</size>
               </box>
             </geometry>
             <material>
               <ambient>0.7 0.7 0.7 1</ambient>
               <diffuse>0.7 0.7 0.7 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>50</mass>
             <inertia>
               <ixx>20</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>20</iyy>
               <iyz>0</iyz>
               <izz>1</izz>
             </inertia>
           </inertial>
         </link>

         <!-- Right barrier -->
         <link name="right_barrier">
           <pose>1.5 0 0 0 0 0</pose>
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.1 4 1.5</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.1 4 1.5</size>
               </box>
             </geometry>
             <material>
               <ambient>0.7 0.7 0.7 1</ambient>
               <diffuse>0.7 0.7 0.7 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>50</mass>
             <inertia>
               <ixx>20</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>20</iyy>
               <iyz>0</iyz>
               <izz>1</izz>
             </inertia>
           </inertial>
         </link>
       </model>
     </world>
   </sdf>
   ```

2. **Create Launch Files for Different Environments**:
   ```xml
   <!-- launch/humanoid_indoor.launch -->
   <launch>
     <!-- Load robot description -->
     <param name="robot_description" command="$(find xacro)/xacro '$(find humanoid_description)/urdf/humanoid_with_sensors.urdf.xacro'" />

     <!-- Launch Gazebo with humanoid indoor world -->
     <include file="$(find gazebo_ros)/launch/empty_world.launch">
       <arg name="world_name" value="$(find humanoid_gazebo)/worlds/humanoid_indoor.world"/>
       <arg name="paused" value="false"/>
       <arg name="use_sim_time" value="true"/>
       <arg name="gui" value="true"/>
       <arg name="headless" value="false"/>
       <arg name="debug" value="false"/>
     </include>

     <!-- Spawn the robot -->
     <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
           args="-param robot_description -urdf -model humanoid_indoor_test -x 0 -y 0 -z 0.5" />

     <!-- Robot state publisher -->
     <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
   </launch>
   ```

3. **Create Unity Environment Scripts**:
   ```csharp
   // Scripts/HumanoidEnvironmentController.cs
   using UnityEngine;
   using System.Collections;

   public class HumanoidEnvironmentController : MonoBehaviour
   {
       [Header("Environment Configuration")]
       public GameObject[] indoorElements;  // Wall, floor, furniture prefabs
       public GameObject[] outdoorElements;  // Terrain, obstacles, etc.

       [Header("Humanoid-Specific Elements")]
       public GameObject doorwayPrefab;
       public GameObject stairPrefab;
       public GameObject narrowPassagePrefab;

       [Header("Physics Configuration")]
       public float humanoidScale = 1.0f;  // Human-scale
       public float stepHeight = 0.15f;    // Maximum step height for bipedal
       public float obstacleSizeMin = 0.1f; // Minimum obstacle size to recognize

       void Start()
       {
           // Initialize environment based on type
           SetupEnvironment();
       }

       void SetupEnvironment()
       {
           // For indoor environment
           if (isIndoorEnvironment)
           {
               InstantiateIndoorElements();
           }
           else
           {
               InstantiateOutdoorElements();
           }

           // Add humanoid-specific elements
           AddHumanoidNavigationFeatures();
       }

       void InstantiateIndoorElements()
       {
           foreach (GameObject element in indoorElements)
           {
               // Scale elements to appropriate humanoid size
               element.transform.localScale *= humanoidScale;
           }

           // Place doorway for navigation
           if (doorwayPrefab != null)
           {
               Vector3 doorwayPosition = new Vector3(0, 3.5f, 0.5f);
               Instantiate(doorwayPrefab, doorwayPosition, Quaternion.identity);
           }

           // Place furniture with appropriate spacing
           if (tablePrefab != null)
           {
               Vector3 tablePosition = new Vector3(2, 2, 0.75f);
               GameObject table = Instantiate(tablePrefab, tablePosition, Quaternion.identity);
               table.transform.localScale *= humanoidScale;
           }
       }

       void InstantiateOutdoorElements()
       {
           // For outdoor environment
           foreach (GameObject element in outdoorElements)
           {
               element.transform.localScale *= humanoidScale;
           }

           // Create stepped surface for stair testing
           if (stairPrefab != null)
           {
               Vector3 stairPosition = new Vector3(5, 0, 0);
               Instantiate(stairPrefab, stairPosition, Quaternion.identity);
           }

           // Create narrow passage
           if (narrowPassagePrefab != null)
           {
               Vector3 passagePosition = new Vector3(0, -5, 0);
               Instantiate(narrowPassagePrefab, passagePosition, Quaternion.identity);
           }
       }

       void AddHumanoidNavigationFeatures()
       {
           // Add features specifically for humanoid navigation
           // These would be visual aids for Unity but also reference points for navigation

           // Add waypoints for navigation testing
           GameObject waypointSystem = new GameObject("NavigationWaypoints");

           // Define waypoints for a test path
           Vector3[] waypoints = {
               new Vector3(-4, 0, 0.5f),   // Start position
               new Vector3(-3, 2, 0.5f),   // Around first obstacle
               new Vector3(0, 0, 0.5f),    // Center area
               new Vector3(3, -2, 0.5f),   // Around second obstacle
               new Vector3(5, 0, 0.5f)     // End position
           };

           for (int i = 0; i < waypoints.Length; i++)
           {
               GameObject waypointObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
               waypointObj.name = $"Waypoint_{i}";
               waypointObj.transform.position = waypoints[i];
               waypointObj.transform.SetParent(waypointSystem.transform);

               // Make it transparent visual aid
               Renderer rend = waypointObj.GetComponent<Renderer>();
               rend.material.color = new Color(0, 1, 1, 0.5f);  // Cyan semi-transparent

               Destroy(waypointObj.GetComponent<SphereCollider>());  // Remove collider
           }
       }

       // Method to dynamically modify environment based on simulation needs
       public void ModifyEnvironment(string modificationType, Vector3 position, float size = 1.0f)
       {
           switch (modificationType)
           {
               case "add_obstacle":
                   AddObstacle(position, size);
                   break;
               case "remove_obstacle":
                   RemoveObstacle(position, size);
                   break;
               case "change_terrain":
                   ModifyTerrain(position, size);
                   break;
               default:
                   Debug.LogWarning($"Unknown environment modification: {modificationType}");
                   break;
           }
       }

       void AddObstacle(Vector3 position, float size)
       {
           if (obstaclePrefab != null)
           {
               GameObject obstacle = Instantiate(obstaclePrefab, position, Quaternion.identity);
               obstacle.transform.localScale = Vector3.one * size * humanoidScale;
           }
       }

       void RemoveObstacle(Vector3 position, float size)
       {
           // Find and remove obstacles near the specified position
           Collider[] colliders = Physics.OverlapSphere(position, size * 2);
           foreach (Collider col in colliders)
           {
               if (col.CompareTag("Obstacle") && Vector3.Distance(col.transform.position, position) < size * 2)
               {
                   Destroy(col.gameObject);
               }
           }
       }

       void ModifyTerrain(Vector3 position, float size)
       {
           // This would change terrain properties in a real implementation
           Debug.Log($"Terrain modification requested at {position} with size {size}");
       }
   }
   ```

## Digital Twin Synchronization Mechanisms

To ensure consistent simulation between the physics-accurate Gazebo environment and the visually-rich Unity environment, we need robust synchronization mechanisms:

```python
#!/usr/bin/env python3
"""
Synchronization node for Gazebo-Unity digital twin
"""
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
import tf2_ros
from geometry_msgs.msg import TransformStamped
import json

class GazeboUnitySynchronizer(Node):
    def __init__(self):
        super().__init__('gazebo_unity_synchronizer')

        # Create publishers for Unity visualization
        self.unity_pose_pub = self.create_publisher(Pose, '/unity/robot_pose', 10)
        self.unity_env_pub = self.create_publisher(String, '/unity/environment_state', 10)

        # Create subscribers for Gazebo state
        self.gazebo_sub = self.create_subscription(
            ModelState,
            '/gazebo/model_states',
            self.gazebo_state_callback,
            10
        )

        # TF broadcaster for Unity transforms
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Timer for environment synchronization
        self.sync_timer = self.create_timer(0.033, self.synchronize_environments)  # ~30 Hz

        # Store environment state
        self.gazebo_unity_transform = {
            'translation': [0, 0, 0],
            'rotation': [0, 0, 0, 1]  # quaternion
        }

        self.get_logger().info('Gazebo-Unity synchronizer initialized')

    def gazebo_state_callback(self, msg):
        """Receive Gazebo model states and broadcast to Unity"""
        # Find humanoid robot in the model states
        for i, name in enumerate(msg.name):
            if 'humanoid' in name.lower() or 'robot' in name.lower():
                if i < len(msg.pose):
                    # Publish robot pose to Unity
                    pose_msg = Pose()
                    pose_msg.position = msg.pose[i].position
                    pose_msg.orientation = msg.pose[i].orientation

                    self.unity_pose_pub.publish(pose_msg)

                    # Broadcast TF transform
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'gazebo_world'
                    t.child_frame_id = 'unity_robot'
                    t.transform.translation.x = msg.pose[i].position.x
                    t.transform.translation.y = msg.pose[i].position.y
                    t.transform.translation.z = msg.pose[i].position.z
                    t.transform.rotation = msg.pose[i].orientation

                    self.tf_broadcaster.sendTransform(t)

                    self.get_logger().debug(f'Synchronized {name} state to Unity')
                    break

    def synchronize_environments(self):
        """Synchronize environment elements between Gazebo and Unity"""
        # Create environment state message
        env_state = {
            'timestamp': self.get_clock().now().seconds_nanoseconds(),
            'elements': [],
            'physics_params': {
                'gravity': [0, 0, -9.81],
                'damping': 0.01,
                'friction': 0.8
            }
        }

        # In a real implementation, this would gather environment state from Gazebo
        # and send it to Unity for consistent rendering

        # Publish environment state
        env_msg = String()
        env_msg.data = json.dumps(env_state)
        self.unity_env_pub.publish(env_msg)

def main(args=None):
    rclpy.init(args=args)

    synchronizer = GazeboUnitySynchronizer()

    try:
        rclpy.spin(synchronizer)
    except KeyboardInterrupt:
        synchronizer.get_logger().info('Synchronizer terminated')
    finally:
        synchronizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Assignment - Creating a Complete Humanoid Testing Environment

### Assignment 1: Building a Multi-Level Humanoid Navigation Environment

**Objective**: Create a complex environment with multiple levels and navigation challenges specifically designed for humanoid robots.

**Detailed Steps**:
1. **Design the Multi-Level Structure**:
   ```xml
   <!-- worlds/humanoid_multi_floor.world -->
   <?xml version="1.0" ?>
   <sdf version="1.6">
     <world name="humanoid_multi_floor">
       <!-- Include standard models -->
       <include>
         <uri>model://ground_plane</uri>
       </include>
       <include>
         <uri>model://sun</uri>
       </include>

       <!-- Ground floor -->
       <model name="ground_floor">
         <static>true</static>
         <link name="floor_link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>20 20 0.2</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>20 20 0.2</size>
               </box>
             </geometry>
             <material>
               <ambient>0.7 0.7 0.7 1</ambient>
               <diffuse>0.7 0.7 0.7 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>1000</mass>
             <inertia>
               <ixx>10000</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>10000</iyy>
               <iyz>0</iyz>
               <izz>20000</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <!-- First level platform -->
       <model name="first_level">
         <pose>0 0 1.5 0 0 0</pose>
         <static>true</static>
         <link name="level_link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>15 15 0.1</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>15 15 0.1</size>
               </box>
             </geometry>
             <material>
               <ambient>0.6 0.6 0.6 1</ambient>
               <diffuse>0.6 0.6 0.6 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>500</mass>
             <inertia>
               <ixx>3000</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>3000</iyy>
               <iyz>0</iyz>
               <izz>6000</izz>
             </inertia>
           </inertial>
         </link>

         <!-- Stairs to first level -->
         <model name='stairs'>
           <pose>-5 -5 0 0 0 0</pose>
           <!-- Create 5 steps -->
           <link name="step_1">
             <pose>0 0 0.15 0 0 0</pose>
             <collision name="collision">
               <geometry>
                 <box>
                   <size>2 1 0.3</size>
                 </box>
               </geometry>
             </collision>
             <visual name="visual">
               <geometry>
                 <box>
                   <size>2 1 0.3</size>
                 </box>
               </geometry>
               <material>
                 <ambient>0.5 0.5 0.5 1</ambient>
                 <diffuse>0.5 0.5 0.5 1</diffuse>
               </material>
             </visual>
             <inertial>
               <mass>20</mass>
               <inertia>
                 <ixx>1</ixx>
                 <ixy>0</ixy>
                 <ixz>0</ixz>
                 <iyy>1</iyy>
                 <iyz>0</iyz>
                 <izz>2</izz>
               </inertia>
             </inertial>
           </link>

           <link name="step_2">
             <pose>0 0 0.3 0 0 0</pose>
             <collision name="collision">
               <geometry>
                 <box>
                   <size>2 1 0.3</size>
                 </box>
               </geometry>
             </collision>
             <visual name="visual">
               <geometry>
                 <box>
                   <size>2 1 0.3</size>
                 </box>
               </geometry>
               <material>
                 <ambient>0.5 0.5 0.5 1</ambient>
                 <diffuse>0.5 0.5 0.5 1</diffuse>
               </material>
             </visual>
             <inertial>
               <mass>20</mass>
               <inertia>
                 <ixx>1</ixx>
                 <ixy>0</ixy>
                 <ixz>0</ixz>
                 <iyy>1</iyy>
                 <iyz>0</iyz>
                 <izz>2</izz>
               </inertia>
             </inertial>
           </link>
         </model>

         <!-- Elevator platform -->
         <model name="elevator_platform">
           <pose>5 5 0 0 0 0</pose>
           <link name="elevator_base">
             <collision name="collision">
               <geometry>
                 <box>
                   <size>2 2 0.1</size>
                 </box>
               </geometry>
             </collision>
             <visual name="visual">
               <geometry>
                 <box>
                   <size>2 2 0.1</size>
                 </box>
               </geometry>
               <material>
                 <ambient>0.8 0.6 0.2 1</ambient>
                 <diffuse>0.8 0.6 0.2 1</diffuse>
               </material>
             </visual>
             <inertial>
               <mass>50</mass>
               <inertia>
                 <ixx>5</ixx>
                 <ixy>0</ixy>
                 <ixz>0</ixz>
                 <iyy>5</iyy>
                 <iyz>0</iyz>
                 <izz>10</izz>
               </inertia>
             </inertial>
           </link>
         </model>

         <!-- Navigation obstacles -->
         <model name="maze_wall_1">
           <pose>2 2 0.5 0 0 0</pose>
           <link name="wall_link">
             <collision name="collision">
               <geometry>
                 <box>
                   <size>0.2 4 1.0</size>
                 </box>
               </geometry>
             </collision>
             <visual name="visual">
               <geometry>
                 <box>
                   <size>0.2 4 1.0</size>
                 </box>
               </geometry>
               <material>
                 <ambient>0.4 0.4 0.8 1</ambient>
                 <diffuse>0.4 0.4 0.8 1</diffuse>
               </material>
             </visual>
             <inertial>
               <mass>20</mass>
               <inertia>
                 <ixx>1</ixx>
                 <ixy>0</ixy>
                 <ixz>0</ixz>
                 <iyy>1</iyy>
                 <iyz>0</iyz>
                 <izz>2</izz>
               </inertia>
             </inertial>
           </link>
         </model>

         <model name="maze_wall_2">
           <pose>4 0 0.5 0 0 0</pose>
           <link name="wall_link">
             <collision name="collision">
               <geometry>
                 <box>
                   <size>4 0.2 1.0</size>
                 </box>
               </geometry>
             </collision>
             <visual name="visual">
               <geometry>
                 <box>
                   <size>4 0.2 1.0</size>
                 </box>
               </geometry>
               <material>
                 <ambient>0.4 0.4 0.8 1</ambient>
                 <diffuse>0.4 0.4 0.8 1</diffuse>
               </material>
             </visual>
             <inertial>
               <mass>20</mass>
               <inertia>
                 <ixx>1</ixx>
                 <ixy>0</ixy>
                 <ixz>0</ixz>
                 <iyy>1</iyy>
                 <iyz>0</iyz>
                 <izz>2</izz>
               </inertial>
             </inertial>
           </link>
         </model>

         <!-- Goal marker -->
         <model name="navigation_goal">
           <pose>8 8 0.1 0 0 0</pose>
           <link name="goal_link">
             <visual name="visual">
               <geometry>
                 <cylinder>
                   <radius>0.2</radius>
                   <length>0.2</length>
                 </cylinder>
               </geometry>
               <material>
                 <ambient>1 0 0 1</ambient>
                 <diffuse>1 0 0 1</diffuse>
               </material>
             </visual>
             <inertial>
               <mass>1</mass>
               <inertia>
                 <ixx>0.1</ixx>
                 <ixy>0</ixy>
                 <ixz>0</ixz>
                 <iyy>0.1</iyy>
                 <iyz>0</iyz>
                 <izz>0.05</izz>
               </inertia>
             </inertial>
           </link>
         </model>
       </world>
     </sdf>
     ```

2. **Create an Environmental Control Node**:
     ```python
     #!/usr/bin/env python3
     """
     Environmental control node for humanoid testing environment
     """
     import rclpy
     from rclpy.node import Node
     from std_msgs.msg import String
     from geometry_msgs.msg import Point
     from gazebo_msgs.srv import SpawnEntity, DeleteEntity
     import xml.etree.ElementTree as ET
     import tempfile
     import os

     class EnvironmentController(Node):
         def __init__(self):
             super().__init__('environment_controller')

             # Create service clients
             self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
             self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

             # Publisher for environment state
             self.env_state_pub = self.create_publisher(String, '/environment/state', 10)

             # Timer for environment monitoring
             self.monitor_timer = self.create_timer(1.0, self.monitor_environment)

             self.get_logger().info('Environment controller initialized')

         def create_dynamic_obstacle(self, position, size=0.5, obstacle_type='box'):
             """Create a dynamic obstacle in the environment"""
             # Define obstacle based on type
             if obstacle_type == 'box':
                 obstacle_xml = f"""
                 <sdf version='1.6'>
                   <model name='dynamic_obstacle_{int(position.x*100)}_{int(position.y*100)}'>
                     <pose>{position.x} {position.y} {position.z} 0 0 0</pose>
                     <link name='obstacle_link'>
                       <collision name='collision'>
                         <geometry>
                           <box>
                             <size>{size} {size} {size}</size>
                           </box>
                         </geometry>
                       </collision>
                       <visual name='visual'>
                         <geometry>
                           <box>
                             <size>{size} {size} {size}</size>
                           </box>
                         </geometry>
                         <material>
                           <ambient>0.8 0.3 0.3 1</ambient>
                           <diffuse>0.8 0.3 0.3 1</diffuse>
                         </material>
                       </visual>
                       <inertial>
                         <mass>10</mass>
                         <inertia>
                           <ixx>1</ixx>
                           <ixy>0</ixy>
                           <ixz>0</ixz>
                           <iyy>1</iyy>
                           <iyz>0</iyz>
                           <izz>1</izz>
                         </inertia>
                       </inertial>
                     </link>
                   </model>
                 </sdf>
                 """
             elif obstacle_type == 'cylinder':
                 obstacle_xml = f"""
                 <sdf version='1.6'>
                   <model name='dynamic_cylinder_{int(position.x*100)}_{int(position.y*100)}'>
                     <pose>{position.x} {position.y} {position.z} 0 0 0</pose>
                     <link name='cylinder_link'>
                       <collision name='collision'>
                         <geometry>
                           <cylinder>
                             <radius>{size/2}</radius>
                             <length>{size}</length>
                           </cylinder>
                         </geometry>
                       </collision>
                       <visual name='visual'>
                         <geometry>
                           <cylinder>
                             <radius>{size/2}</radius>
                             <length>{size}</length>
                           </cylinder>
                         </geometry>
                         <material>
                           <ambient>0.3 0.8 0.3 1</ambient>
                           <diffuse>0.3 0.8 0.3 1</diffuse>
                         </material>
                       </visual>
                       <inertial>
                         <mass>10</mass>
                         <inertia>
                           <ixx>1</ixx>
                           <ixy>0</ixy>
                           <ixz>0</ixz>
                           <iyy>1</iyy>
                           <iyz>0</iyz>
                           <izz>0.5</izz>
                         </inertia>
                       </inertial>
                     </link>
                   </model>
                 </sdf>
                 """

             # Create temporary file for the entity
             with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.sdf') as tmp_file:
                 tmp_file.write(obstacle_xml)
                 entity_path = tmp_file.name

             # Call spawn service
             request = SpawnEntity.Request()
             request.name = f"dynamic_obstacle_{int(position.x*100)}_{int(position.y*100)}"
             request.xml = obstacle_xml
             request.initial_pose.position = position

             future = self.spawn_client.call_async(request)
             rclpy.spin_until_future_complete(self, future)

             # Cleanup temp file
             os.unlink(entity_path)

             result = future.result()
             if result.success:
                 self.get_logger().info(f'Dynamic obstacle spawned at {position}')
             else:
                 self.get_logger().error(f'Failed to spawn obstacle: {result.status_message}')

             return result.success

         def remove_dynamic_obstacle(self, obstacle_name):
             """Remove dynamic obstacle from environment"""
             request = DeleteEntity.Request()
             request.name = obstacle_name

             future = self.delete_client.call_async(request)
             rclpy.spin_until_future_complete(self, future)

             result = future.result()
             if result.success:
                 self.get_logger().info(f'Dynamic obstacle removed: {obstacle_name}')
             else:
                 self.get_logger().error(f'Failed to remove obstacle: {result.status_message}')

             return result.success

         def monitor_environment(self):
             """Monitor environment state and publish updates"""
             # This would typically gather information about the environment state
             # For now, we'll just publish a simple status message
             env_msg = String()
             env_msg.data = '{"status": "active", "obstacles": 2, "floors": 2, "navigation_goals": 1}'
             self.env_state_pub.publish(env_msg)

     def main(args=None):
         rclpy.init(args=args)

         env_controller = EnvironmentController()

         try:
             rclpy.spin(env_controller)
         except KeyboardInterrupt:
             env_controller.get_logger().info('Environment controller stopped')
         finally:
             env_controller.destroy_node()
             rclpy.shutdown()

     if __name__ == '__main__':
         main()
     ```

3. **Test the Environment**:
     ```bash
     # Terminal 1: Launch the multi-level environment
     ros2 launch humanoid_environment multi_floor.launch.py

     # Terminal 2: Control the environment
     ros2 run humanoid_environment environment_controller
     ```

**Expected Outcome**: A complex multi-level environment with navigation challenges specifically designed for humanoid robot testing, including stairs, obstacles, and multiple navigation levels.

**Learning Points**:
- Understanding how to create complex multi-level environments
- Learning to dynamically modify environments during simulation
- Recognizing the importance of human-scale obstacles and features

## Validation and Quality Assurance

### Verifying Environment Accuracy

To ensure that the environments accurately represent real-world physics:

1. **Physics Validation Tests**:
   ```python
   #!/usr/bin/env python3
   """
   Physics validation tests for humanoid environments
   """
   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import Point
   from std_msgs.msg import Float32
   import math

   class PhysicsValidator(Node):
       def __init__(self):
           super().__init__('physics_validator')

           self.gravity_pub = self.create_publisher(Float32, '/validation/gravity_measurement', 10)
           self.friction_pub = self.create_publisher(Float32, '/validation/friction_coefficient', 10)

           # Timer for validation tests
           self.validation_timer = self.create_timer(0.1, self.run_validation_tests)

           self.start_time = self.get_clock().now()
           self.test_object_position = Point()
           self.test_object_initial_position = None
           self.test_mode = "gravity"  # gravity, friction, etc.

           self.get_logger().info('Physics validator initialized')

       def run_validation_tests(self):
           """Run physics validation tests"""
           current_time = self.get_clock().now()
           elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

           if self.test_mode == "gravity":
               # Test gravity by dropping an object
               expected_drop_distance = 0.5 * 9.81 * elapsed_time**2

               # Calculate actual drop (this would be measured from simulation)
               if self.test_object_initial_position:
                   actual_drop_distance = (
                       self.test_object_initial_position.z - self.test_object_position.z
                   )

                   # Compare with expected
                   gravity_accuracy = abs(actual_drop_distance - expected_drop_distance) / expected_drop_distance

                   # Publish measurement
                   gravity_msg = Float32()
                   gravity_msg.data = 9.81 * (1 - gravity_accuracy)  # Adjusted measurement
                   self.gravity_pub.publish(gravity_msg)

                   if abs(gravity_accuracy) > 0.05:  # More than 5% error
                       self.get_logger().warn(f'Gravity validation error: {gravity_accuracy*100:.2f}%')

   def main(args=None):
       rclpy.init(args=args)

       validator = PhysicsValidator()

       try:
           rclpy.spin(validator)
       except KeyboardInterrupt:
           validator.get_logger().info('Physics validator stopped')
       finally:
           validator.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

## Troubleshooting Environment Issues

### Common Problems and Solutions

1. **Performance Issues**:
   - **Problem**: Environment running slowly?
   - **Solution**: Simplify geometries, reduce number of objects, adjust physics parameters

2. **Synchronization Problems**:
   - **Problem**: Gazebo and Unity environments out of sync?
   - **Solution**: Verify ROS bridge configuration, adjust update rates, check transform frames

3. **Physics Instability**:
   - **Problem**: Objects behaving unexpectedly?
   - **Solution**: Check mass/inertia parameters, adjust physics solver settings, verify collision models

4. **Collision Detection Issues**:
   - **Problem**: Objects passing through each other?
   - **Solution**: Check collision geometries, adjust surface parameters, verify object properties

## Resources for Environment Development

- [Gazebo Model Database](http://models.gazebosim.org/)
- [ROS URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [Unity Robotics Package](https://docs.unity3d.com/Packages/com.unity.robotics@latest)
- [Humanoid Robot Simulation Guidelines](https://humanoids.ros.org/simulation-guidelines)

**Expected Outcome**: A Unity scene that mirrors the Gazebo environment and can receive real-time updates from the ROS system.

**Learning Points**:
- Understanding how to create matching environments in Unity and Gazebo
- Learning to implement ROS connection in Unity
- Recognizing the importance of visual synchronization in digital twins

### Assignment 3: Creating a Dynamic Environment with Moving Obstacles

**Objective**: Enhance the environment with dynamic elements that move over time, challenging the robot's real-time navigation capabilities.

**Detailed Steps**:
1. **Create a Dynamic World with Moving Obstacles**:
   ```xml
   <!-- worlds/dynamic_indoor.world -->
   <?xml version="1.0" ?>
   <sdf version="1.6">
     <world name="dynamic_indoor">
       <!-- Include standard models -->
       <include>
         <uri>model://ground_plane</uri>
       </include>
       <include>
         <uri>model://sun</uri>
       </include>

       <!-- Static environment -->
       <model name="wall_1">
         <pose>0 5 0 0 0 0</pose>
         <link name="wall_1_link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>10 0.2 2.5</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>10 0.2 2.5</size>
               </box>
             </geometry>
             <material>
               <ambient>0.8 0.8 0.8 1</ambient>
               <diffuse>0.8 0.8 0.8 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>100</mass>
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

       <!-- Moving obstacle -->
       <model name="moving_cart">
         <pose>-2 0 0 0 0 0</pose>
         <link name="cart_chassis">
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.8 0.6 0.6</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.8 0.6 0.6</size>
               </box>
             </geometry>
             <material>
               <ambient>0.8 0.7 0.2 1</ambient>
               <diffuse>0.8 0.7 0.2 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>10</mass>
             <inertia>
               <ixx>0.1</ixx>
               <ixy>0.0</ixy>
               <ixz>0.0</ixz>
               <iyy>0.1</iyy>
               <iyz>0.0</iyz>
               <izz>0.1</izz>
             </inertia>
           </inertial>
         </link>

         <!-- Wheels for movement -->
         <link name="front_wheel">
           <pose>0.3 0.3 0 0 0 0</pose>
           <collision name="collision">
             <geometry>
               <cylinder>
                 <radius>0.1</radius>
                 <length>0.05</length>
               </cylinder>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <cylinder>
                 <radius>0.1</radius>
                 <length>0.05</length>
               </cylinder>
             </geometry>
           </visual>
           <inertial>
             <mass>0.5</mass>
             <inertia>
               <ixx>0.001</ixx>
               <ixy>0.0</ixy>
               <ixz>0.0</ixz>
               <iyy>0.001</iyy>
               <iyz>0.0</iyz>
               <izz>0.001</izz>
             </inertia>
           </inertial>
         </link>

         <link name="rear_wheel">
           <pose>-0.3 0.3 0 0 0 0</pose>
           <collision name="collision">
             <geometry>
               <cylinder>
                 <radius>0.1</radius>
                 <length>0.05</length>
               </cylinder>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <cylinder>
                 <radius>0.1</radius>
                 <length>0.05</length>
               </cylinder>
             </geometry>
           </visual>
           <inertial>
             <mass>0.5</mass>
             <inertia>
               <ixx>0.001</ixx>
               <ixy>0.0</ixy>
               <ixz>0.0</ixz>
               <iyy>0.001</iyy>
               <iyz>0.0</iyz>
               <izz>0.001</izz>
             </inertia>
           </inertial>
         </link>

         <!-- Joint to connect wheels to chassis -->
         <joint name="front_wheel_joint" type="continuous">
           <parent>cart_chassis</parent>
           <child>front_wheel</child>
           <axis>
             <xyz>0 1 0</xyz>
           </axis>
         </joint>

         <joint name="rear_wheel_joint" type="continuous">
           <parent>cart_chassis</parent>
           <child>rear_wheel</child>
           <axis>
             <xyz>0 1 0</xyz>
           </axis>
         </joint>

         <!-- Plugin to move the cart -->
         <plugin name="cart_controller" filename="libgazebo_ros_planar_move.so">
           <commandTopic>/moving_cart/cmd_vel</commandTopic>
           <odometryTopic>/moving_cart/odom</odometryTopic>
           <odometryFrame>odom</odometryFrame>
           <robotBaseFrame>cart_chassis</robotBaseFrame>
           <publishOdomTF>true</publishOdomTF>
         </plugin>
       </model>

       <!-- Rotating obstacle -->
       <model name="rotating_barrier">
         <pose>2 2 0.5 0 0 0</pose>
         <link name="barrier_base">
           <collision name="collision">
             <geometry>
               <cylinder>
                 <radius>0.2</radius>
                 <length>1.0</length>
               </cylinder>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <cylinder>
                 <radius>0.2</radius>
                 <length>1.0</length>
               </cylinder>
             </geometry>
             <material>
               <ambient>1 0 0 1</ambient>
               <diffuse>1 0 0 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>5</mass>
             <inertia>
               <ixx>0.1</ixx>
               <ixy>0.0</ixy>
               <ixz>0.0</ixz>
               <iyy>0.1</iyy>
               <iyz>0.0</iyz>
               <izz>0.1</izz>
             </inertia>
           </inertial>
         </link>

         <link name="rotating_bar">
           <pose>0.5 0 0 0 0 0</pose>
           <collision name="collision">
             <geometry>
               <box>
                 <size>1.0 0.1 0.1</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>1.0 0.1 0.1</size>
               </box>
             </geometry>
             <material>
               <ambient>0 0 1 1</ambient>
               <diffuse>0 0 1 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>2</mass>
             <inertia>
               <ixx>0.1</ixx>
               <ixy>0.0</ixy>
               <ixz>0.0</ixz>
               <iyy>0.1</iyy>
               <iyz>0.0</iyz>
               <izz>0.1</izz>
             </inertia>
           </inertial>
         </link>

         <!-- Revolute joint to rotate the bar -->
         <joint name="rotation_joint" type="revolute">
           <parent>barrier_base</parent>
           <child>rotating_bar</child>
           <axis>
             <xyz>0 0 1</xyz>
             <limit>
               <lower>-1000</lower>
               <upper>1000</upper>
             </limit>
           </axis>
         </joint>

         <!-- Plugin to rotate the bar -->
         <plugin name="rotation_controller" filename="libgazebo_ros_pids.so">
           <robotNamespace>/rotating_barrier</robotNamespace>
           <jointName>rotation_joint</jointName>
           <serviceName>set_rotation_speed</serviceName>
           <commandTopic>rotation_cmd</commandTopic>
           <odometryTopic>rotation_state</odometryTopic>
           <feedbackType>position</feedbackType>
           <pid>
             <p>10.0</p>
             <i>0.1</i>
             <d>0.5</d>
           </pid>
         </plugin>
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

2. **Create a Dynamic Obstacle Controller**:
   ```python
   #!/usr/bin/env python3
   """
   Dynamic obstacle controller to move obstacles in the environment
   """
   import rospy
   import math
   from geometry_msgs.msg import Twist
   from std_msgs.msg import Float64

   class DynamicObstacleController:
       def __init__(self):
           rospy.init_node('dynamic_obstacle_controller', anonymous=True)

           # Publishers for moving obstacles
           self.moving_cart_pub = rospy.Publisher('/moving_cart/cmd_vel', Twist, queue_size=10)
           self.rotating_barrier_pub = rospy.Publisher('/rotating_barrier/rotation_cmd', Float64, queue_size=10)

           # Parameters
           self.cart_linear_velocity = 0.3
           self.cart_angular_velocity = 0.0
           self.rotation_speed = 0.5  # rad/s

           # Rate control
           self.rate = rospy.Rate(10)  # 10 Hz

           # Timer for obstacle state changes
           self.timer_counter = 0
           self.cart_direction = 1  # 1 for forward, -1 for backward

       def run(self):
           rospy.loginfo("Starting dynamic obstacle controller")

           while not rospy.is_shutdown():
               # Control the moving cart
               cart_cmd = Twist()
               cart_cmd.linear.x = self.cart_linear_velocity * self.cart_direction
               cart_cmd.angular.z = self.cart_angular_velocity
               self.moving_cart_pub.publish(cart_cmd)

               # Control the rotating barrier
               rotation_cmd = Float64()
               rotation_cmd.data = self.rotation_speed * math.sin(rospy.Time.now().to_sec())
               self.rotating_barrier_pub.publish(rotation_cmd)

               # Occasionally change cart direction
               self.timer_counter += 1
               if self.timer_counter > 50:  # Change direction every 5 seconds
                   self.cart_direction *= -1  # Reverse direction
                   self.timer_counter = 0

               self.rate.sleep()

   if __name__ == '__main__':
       controller = DynamicObstacleController()
       controller.run()
   ```

3. **Create a Dynamic Navigation Test**:
   ```python
   #!/usr/bin/env python3
   """
   Script to test navigation in dynamic environment
   """
   import rospy
   import actionlib
   import time
   from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
   from geometry_msgs.msg import Pose, Point, Quaternion
   from tf.transformations import quaternion_from_euler
   from sensor_msgs.msg import LaserScan

   class DynamicNavigationTest:
       def __init__(self):
           rospy.init_node('dynamic_navigation_test', anonymous=True)

           # Connect to the move_base action server
           self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
           rospy.loginfo("Waiting for move_base action server...")
           self.move_base.wait_for_server(rospy.Duration(60))
           rospy.loginfo("Connected to move_base action server")

           # Subscribe to laser scan to detect dynamic obstacles
           self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
           self.latest_scan = None

           # Define navigation goals in the dynamic environment
           self.goals = [
               self.create_goal(4, 4, 0.0),    # Goal in open area
               self.create_goal(-4, 4, 0.0),   # Goal near moving obstacle
               self.create_goal(2, -4, 0.0),   # Goal near rotating barrier
           ]

       def scan_callback(self, data):
           """Store latest scan data for dynamic obstacle detection"""
           self.latest_scan = data

       def detect_dynamic_obstacles(self):
           """Detect potential dynamic obstacles from laser scan"""
           if self.latest_scan is None:
               return False

           # Simple heuristic: if close-range readings are changing rapidly,
           # it might indicate dynamic obstacles
           min_ranges = self.latest_scan.ranges[:len(self.latest_scan.ranges)//2]
           filtered_ranges = [r for r in min_ranges if not (r < 0.1 or r > 10.0)]

           if len(filtered_ranges) < 10:
               return False

           # Calculate variance in nearby distances (could indicate movement)
           import numpy as np
           if len(filtered_ranges) > 1:
               variance = np.var(filtered_ranges)
               # If variance is high, it might indicate moving objects
               return variance > 0.1  # Adjust threshold as needed

           return False

       def create_goal(self, x, y, theta):
           """Create a navigation goal"""
           goal = MoveBaseGoal()
           goal.target_pose.header.frame_id = "map"
           goal.target_pose.header.stamp = rospy.Time.now()

           # Set position
           goal.target_pose.pose.position = Point(x, y, 0.0)

           # Set orientation
           angle = quaternion_from_euler(0, 0, theta)
           goal.target_pose.pose.orientation = Quaternion(*angle)

           return goal

       def execute_dynamic_navigation_test(self):
           """Execute the navigation test in dynamic environment"""
           rospy.loginfo("Starting dynamic navigation test sequence")

           for i, goal in enumerate(self.goals):
               rospy.loginfo(f"Moving to dynamic goal {i+1}")

               # Send goal to move_base
               self.move_base.send_goal(goal)

               # Monitor progress and reactivity to dynamic obstacles
               start_time = rospy.Time.now()
               last_replan_time = start_time

               while not self.move_base.wait_for_result(rospy.Duration(0.1)):
                   # Check if we need to abort due to dynamic obstacles
                   if self.detect_dynamic_obstacles():
                       elapsed = (rospy.Time.now() - last_replan_time).to_sec()
                       if elapsed > 3:  # Only replan if it's been stable for 3 seconds
                           rospy.loginfo("Dynamic obstacle detected, re-evaluating path...")
                           # In a real implementation, we would cancel and re-plan
                           # For this simulation, just log the event
                           last_replan_time = rospy.Time.now()

                   # Check timeout
                   elapsed_total = (rospy.Time.now() - start_time).to_sec()
                   if elapsed_total > 90:  # 90 second timeout
                       rospy.logerr(f"Goal {i+1} timed out after 90 seconds")
                       self.move_base.cancel_goal()
                       break

               # Check result
               state = self.move_base.get_state()
               if state == 3:  # Goal succeeded
                   rospy.loginfo(f"Dynamic goal {i+1} reached successfully")
               else:
                   rospy.logerr(f"Failed to reach dynamic goal {i+1}, state: {state}")

               # Wait between goals
               time.sleep(3)

           rospy.loginfo("Dynamic navigation test completed")

       def run(self):
           self.execute_dynamic_navigation_test()
           rospy.spin()

   if __name__ == '__main__':
       test = DynamicNavigationTest()
       test.run()
   ```

4. **Create a Launch File for the Dynamic Environment**:
   ```xml
   <!-- launch/dynamic_env_test.launch -->
   <launch>
     <!-- Load robot description -->
     <param name="robot_description" command="$(find xacro)/xacro '$(find your_robot_description)/urdf/humanoid_with_sensors.urdf.xacro'" />

     <!-- Launch Gazebo with dynamic world -->
     <include file="$(find gazebo_ros)/launch/empty_world.launch">
       <arg name="world_name" value="$(find your_robot_gazebo)/worlds/dynamic_indoor.world"/>
       <arg name="paused" value="false"/>
       <arg name="use_sim_time" value="true"/>
       <arg name="gui" value="true"/>
       <arg name="headless" value="false"/>
       <arg name="debug" value="false"/>
     </include>

     <!-- Spawn the robot -->
     <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
           args="-param robot_description -urdf -model humanoid_dynamic_test -x 0 -y 0 -z 0.5" />

     <!-- Robot state publisher -->
     <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

     <!-- Dynamic obstacle controller -->
     <node name="dynamic_obstacle_controller" pkg="your_robot_gazebo" type="dynamic_obstacle_controller.py" output="screen" />

     <!-- Navigation stack -->
     <node name="move_base" pkg="move_base" type="move_base" output="screen">
       <rosparam file="$(find your_robot_navigation)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
       <rosparam file="$(find your_robot_navigation)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
       <rosparam file="$(find your_robot_navigation)/config/local_costmap_params.yaml" command="load" />
       <rosparam file="$(find your_robot_navigation)/config/global_costmap_params.yaml" command="load" />
       <rosparam file="$(find your_robot_navigation)/config/base_local_planner_params.yaml" command="load" />
       <rosparam file="$(find your_robot_navigation)/config/move_base_params.yaml" command="load" />
     </node>
   </launch>
   ```

5. **Test the Dynamic Environment**:
   ```bash
   # Launch the dynamic environment
   roslaunch your_robot_gazebo dynamic_env_test.launch

   # In another terminal, run the dynamic navigation test
   rosrun your_robot_navigation dynamic_navigation_test.py
   ```

**Expected Outcome**: A dynamic environment with moving obstacles that challenges the robot's ability to navigate in real-time changing conditions.

**Learning Points**:
- Understanding how to create dynamic elements in simulation
- Learning to implement real-time navigation in changing environments
- Recognizing the challenges of dynamic obstacle avoidance

## Troubleshooting Environment Issues

### Performance Issues
- **Problem**: Low simulation frame rate
- **Solution**: Simplify geometry, reduce number of active models, optimize physics parameters

### Physics Instability
- **Problem**: Objects behaving unrealistically
- **Solution**: Check mass/inertia values, adjust solver parameters in physics config

### Sensor Occlusion
- **Problem**: Sensors blocked by environment geometry
- **Solution**: Verify visual and collision geometries don't interfere with sensors

### Navigation Problems
- **Problem**: Path planner failing in complex environments
- **Solution**: Adjust costmap parameters, simplify environment geometry, tune planner

### Synchronization Problems
- **Problem**: Unity visualization not matching Gazebo simulation
- **Solution**: Verify ROS connection, check coordinate system alignment, validate timing