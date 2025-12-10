---
sidebar_position: 3
title: Unity Visualization and Interaction
---

# Unity Visualization and Human-Robot Interaction

This section focuses on creating high-fidelity visualizations in Unity and enabling meaningful human-robot interaction.

## Setting Up the Unity Environment

To create an effective digital twin, the Unity environment should mirror the Gazebo simulation as closely as possible:

- Match coordinate systems between Unity and Gazebo
- Recreate lighting conditions
- Ensure similar environmental geometry
- Implement equivalent camera controls

## Camera Navigation in Unity

The Unity scene includes navigation controls to allow exploration of the environment:

```csharp
public class CameraController : MonoBehaviour
{
    public float moveSpeed = 10f;
    public float lookSpeed = 2f;

    void Update()
    {
        HandleMovement();
        HandleRotation();
    }

    void HandleMovement()
    {
        float horizontal = 0f;
        float vertical = 0f;

        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
        {
            vertical = 1f;
        }
        // Additional inputs for movement...
    }

    void HandleRotation()
    {
        if (Input.GetMouseButton(1)) // Right mouse button
        {
            float mouseX = Input.GetAxis("Mouse X") * lookSpeed;
            float mouseY = Input.GetAxis("Mouse Y") * lookSpeed;
            transform.Rotate(-mouseY, mouseX, 0, Space.Self);
        }
    }
}
```

## Implementing Interaction Systems

Human-robot interaction in Unity is facilitated through:

- Object highlighting on hover
- Context-sensitive interaction prompts
- Physics-based interaction feedback
- Sensor data visualization

## Synchronization with Gazebo

Maintaining synchronization between Unity and Gazebo environments is critical:

- Implement transform synchronization
- Use consistent coordinate systems
- Maintain timing alignment
- Validate positional accuracy

## Performance Optimization

To maintain 30+ FPS on RTX-enabled hardware:

- Use occlusion culling for complex scenes
- Implement Level of Detail (LOD) systems
- Optimize shader complexity
- Batch similar objects
- Use compressed textures appropriately

## Troubleshooting Unity Issues

Common Unity issues include:
- Performance drops (optimize scene complexity)
- Synchronization delays (check networking)
- Visual inconsistencies (match lighting/materials)
- Interaction problems (review collision layers)

## Practical Assignment - Unity Visualization Implementation

### Assignment 1: Basic Humanoid Robot Visualization in Unity

**Objective**: Create a basic humanoid robot model in Unity that reflects poses from a simulation.

**Detailed Steps**:
1. **Create Humanoid Robot in Unity**:
   ```csharp
   // RobotModel.cs
   using UnityEngine;

   public class RobotModel : MonoBehaviour
   {
       [Header("Body Parts")]
       public Transform body;
       public Transform head;
       public Transform leftArm;
       public Transform rightArm;
       public Transform leftLeg;
       public Transform rightLeg;

       [Header("Joint Limits")]
       public float headYawLimit = 45f;
       public float headPitchLimit = 30f;
       public float armSwingLimit = 90f;

       // Position and rotation targets for each part
       private Vector3 targetBodyPosition;
       private Vector3 targetHeadPosition;
       private Vector3 targetHeadRotation;
       private Vector3 targetLeftArmRotation;
       private Vector3 targetRightArmRotation;
       private Vector3 targetLeftLegRotation;
       private Vector3 targetRightLegRotation;

       // Interpolation speed
       public float movementSpeed = 5f;
       public float rotationSpeed = 10f;

       void Start()
       {
           // Initialize targets to current positions
           targetBodyPosition = body.position;
           targetHeadPosition = head.localPosition;
           targetHeadRotation = head.localEulerAngles;
           targetLeftArmRotation = leftArm.localEulerAngles;
           targetRightArmRotation = rightArm.localEulerAngles;
           targetLeftLegRotation = leftLeg.localEulerAngles;
           targetRightLegRotation = rightLeg.localEulerAngles;
       }

       void Update()
       {
           // Smoothly move body parts to target positions/rotations
           body.position = Vector3.Lerp(body.position, targetBodyPosition, Time.deltaTime * movementSpeed);
           head.localPosition = Vector3.Lerp(head.localPosition, targetHeadPosition, Time.deltaTime * movementSpeed);
           head.localRotation = Quaternion.Slerp(head.localRotation,
                                               Quaternion.Euler(targetHeadRotation),
                                               Time.deltaTime * rotationSpeed);
           leftArm.localRotation = Quaternion.Slerp(leftArm.localRotation,
                                                   Quaternion.Euler(targetLeftArmRotation),
                                                   Time.deltaTime * rotationSpeed);
           rightArm.localRotation = Quaternion.Slerp(rightArm.localRotation,
                                                    Quaternion.Euler(targetRightArmRotation),
                                                    Time.deltaTime * rotationSpeed);
           leftLeg.localRotation = Quaternion.Slerp(leftLeg.localRotation,
                                                   Quaternion.Euler(targetLeftLegRotation),
                                                   Time.deltaTime * rotationSpeed);
           rightLeg.localRotation = Quaternion.Slerp(rightLeg.localRotation,
                                                    Quaternion.Euler(targetRightLegRotation),
                                                    Time.deltaTime * rotationSpeed);
       }

       // Public methods to set joint positions/rotations
       public void SetHeadRotation(float yaw, float pitch)
       {
           // Clamp values to prevent excessive movement
           yaw = Mathf.Clamp(yaw, -headYawLimit, headYawLimit);
           pitch = Mathf.Clamp(pitch, -headPitchLimit, headPitchLimit);

           targetHeadRotation = new Vector3(pitch, yaw, 0);
       }

       public void SetArmRotation(float leftYaw, float rightYaw)
       {
           leftYaw = Mathf.Clamp(leftYaw, -armSwingLimit, armSwingLimit);
           rightYaw = Mathf.Clamp(rightYaw, -armSwingLimit, armSwingLimit);

           targetLeftArmRotation = new Vector3(0, leftYaw, 0);
           targetRightArmRotation = new Vector3(0, rightYaw, 0);
       }

       public void SetLegRotation(float leftYaw, float rightYaw)
       {
           leftYaw = Mathf.Clamp(leftYaw, -armSwingLimit/2, armSwingLimit/2);
           rightYaw = Mathf.Clamp(rightYaw, -armSwingLimit/2, armSwingLimit/2);

           targetLeftLegRotation = new Vector3(0, leftYaw, 0);
           targetRightLegRotation = new Vector3(0, rightYaw, 0);
       }
   }
   ```

2. **Create a Visualization Controller**:
   ```csharp
   // VisualizationController.cs
   using UnityEngine;

   public class VisualizationController : MonoBehaviour
   {
       public RobotModel robotModel;

       [Header("Test Controls")]
       public float headYaw = 0f;
       public float headPitch = 0f;
       public float leftArmYaw = 0f;
       public float rightArmYaw = 0f;
       public float leftLegYaw = 0f;
       public float rightLegYaw = 0f;

       void Start()
       {
           if (robotModel == null)
           {
               robotModel = FindObjectOfType<RobotModel>();
           }
       }

       void Update()
       {
           // Update robot based on test controls
           if (robotModel != null)
           {
               robotModel.SetHeadRotation(headYaw, headPitch);
               robotModel.SetArmRotation(leftArmYaw, rightArmYaw);
               robotModel.SetLegRotation(leftLegYaw, rightLegYaw);
           }

           // Add keyboard controls for testing
           if (Input.GetKey(KeyCode.UpArrow)) headPitch += 100f * Time.deltaTime;
           if (Input.GetKey(KeyCode.DownArrow)) headPitch -= 100f * Time.deltaTime;
           if (Input.GetKey(KeyCode.LeftArrow)) headYaw -= 100f * Time.deltaTime;
           if (Input.GetKey(KeyCode.RightArrow)) headYaw += 100f * Time.deltaTime;

           if (Input.GetKey(KeyCode.A)) leftArmYaw -= 100f * Time.deltaTime;
           if (Input.GetKey(KeyCode.D)) leftArmYaw += 100f * Time.deltaTime;
           if (Input.GetKey(KeyCode.J)) rightArmYaw -= 100f * Time.deltaTime;
           if (Input.GetKey(KeyCode.L)) rightArmYaw += 100f * Time.deltaTime;

           if (Input.GetKey(KeyCode.Z)) leftLegYaw -= 100f * Time.deltaTime;
           if (Input.GetKey(KeyCode.C)) leftLegYaw += 100f * Time.deltaTime;
           if (Input.GetKey(KeyCode.N)) rightLegYaw -= 100f * Time.deltaTime;
           if (Input.GetKey(KeyCode.M)) rightLegYaw += 100f * Time.deltaTime;

           // Reset on spacebar
           if (Input.GetKeyDown(KeyCode.Space))
           {
               headYaw = 0f;
               headPitch = 0f;
               leftArmYaw = 0f;
               rightArmYaw = 0f;
               leftLegYaw = 0f;
               rightLegYaw = 0f;
           }
       }
   }
   ```

3. **Set Up the Scene**:
   - Create a new Unity scene
   - Create a main camera and position it to view the robot
   - Create a cube for the robot's body (0.4x1.6x0.2)
   - Create a sphere for the head (radius 0.15) and attach to body
   - Create cylinders for arms (0.05 radius, 0.6 length) and attach to body
   - Create cylinders for legs (0.06 radius, 0.7 length) and attach to body
   - Arrange the parts to form a humanoid shape
   - Add the RobotModel script to the parent object
   - Add the VisualizationController script to another object
   - Connect the robot model reference in the VisualizationController

4. **Test the Visualization**:
   - Run the scene in Unity
   - Use arrow keys to move the head (up/down for pitch, left/right for yaw)
   - Use A/D keys to move the left arm
   - Use J/L keys to move the right arm
   - Use Z/C keys to move the left leg
   - Use N/M keys to move the right leg
   - Press Space to reset all positions

**Expected Outcome**: A basic humanoid robot model in Unity that can be controlled with keyboard inputs.

**Learning Points**:
- Understanding Unity transforms and rotations
- Learning how to control robot parts in Unity
- Recognizing the fundamentals of 3D visualization

### Assignment 2: ROS Integration with Unity

**Objective**: Implement communication between ROS and Unity to visualize real robot data.

**Detailed Steps**:
1. **Install ROS# Unity Package** (Unity Robotics Package):
   - In Unity, go to Window → Package Manager
   - Click the + button and select "Add package from git URL..."
   - Add the Unity Robotics package: `com.unity.robotics.ros-tcp-connector`
   - Install the package

2. **Create a ROS Communication Handler**:
   ```csharp
   // ROSCommunicationHandler.cs
   using UnityEngine;
   using Unity.Robotics.ROSTCPConnector;
   using RosMessageTypes.Sensor;
   using RosMessageTypes.Geometry;
   using RosMessageTypes.Std;
   using Unity.Robotics.ROSTCPConnector.MessageGeneration;

   public class ROSCommunicationHandler : MonoBehaviour
   {
       ROSConnection ros;
       public RobotModel robotModel;

       [Header("ROS Topics")]
       public string jointStatesTopic = "/joint_states";
       public string imuTopic = "/imu/data";

       // Joint position storage
       private float neckJointPos = 0f;
       private float leftShoulderJointPos = 0f;
       private float rightShoulderJointPos = 0f;
       private float leftHipJointPos = 0f;
       private float rightHipJointPos = 0f;

       void Start()
       {
           ros = ROSConnection.GetOrCreateInstance();
           ros.Subscribe<sensor_msgs.JointStateMsg>(jointStatesTopic, JointStateCallback);
           ros.Subscribe<sensor_msgs.ImuMsg>(imuTopic, ImuCallback);

           if (robotModel == null)
           {
               robotModel = FindObjectOfType<RobotModel>();
           }
       }

       void JointStateCallback(sensor_msgs.JointStateMsg jointState)
       {
           // Parse joint positions from ROS message
           for (int i = 0; i < jointState.name.Length; i++)
           {
               string jointName = jointState.name[i];

               switch (jointName)
               {
                   case "neck_joint":
                       neckJointPos = (float)jointState.position[i];
                       break;
                   case "left_shoulder_joint":
                       leftShoulderJointPos = (float)jointState.position[i];
                       break;
                   case "right_shoulder_joint":
                       rightShoulderJointPos = (float)jointState.position[i];
                       break;
                   case "left_hip_joint":
                       leftHipJointPos = (float)jointState.position[i];
                       break;
                   case "right_hip_joint":
                       rightHipJointPos = (float)jointState.position[i];
                       break;
               }
           }
       }

       void ImuCallback(sensor_msgs.ImuMsg imuData)
       {
           // Process IMU data if needed
           // This is where you'd handle orientation data for the robot
       }

       void Update()
       {
           // Update robot model with received joint positions
           if (robotModel != null)
           {
               // Convert joint positions to appropriate rotation values
               // (This may require scaling based on your robot model)
               robotModel.SetHeadRotation(0, neckJointPos * 57.3f); // Convert radians to degrees
               robotModel.SetArmRotation(leftShoulderJointPos * 57.3f, rightShoulderJointPos * 57.3f);
               robotModel.SetLegRotation(leftHipJointPos * 57.3f, rightHipJointPos * 57.3f);
           }
       }
   }
   ```

3. **Connect Unity to ROS**:
   - In the Unity editor, add the ROSCommunicationHandler script to an empty GameObject
   - Add the RobotModel component to your robot's parent object
   - In the ROSCommunicationHandler component, assign the RobotModel reference
   - Ensure ROS master is running (roslaunch or rosbridge)
   - In Unity, set the ROS IP address to your ROS master IP (typically localhost:11311)

4. **Test ROS-Unity Integration**:
   ```bash
   # Run Gazebo simulation
   roslaunch your_robot_gazebo humanoid_simulation.launch

   # In Unity, hit Play and observe the robot moving based on ROS joint states
   ```

**Expected Outcome**: Unity visualizing real-time joint positions from the Gazebo simulation.

**Learning Points**:
- Understanding ROS-Unity integration
- Learning to process ROS messages in Unity
- Recognizing the importance of real-time data visualization

### Assignment 3: Advanced Visualization with Physics Simulation

**Objective**: Create an advanced visualization with realistic rendering and physics simulation in Unity.

**Detailed Steps**:
1. **Set up Unity's Universal Render Pipeline (URP)**:
   - In Unity, go to Assets → Create → Rendering → Universal Render Pipeline → Pipeline Asset
   - Create a new URP asset
   - In Project Settings → Graphics, add the URP asset to the Scriptable Render Pipeline Settings

2. **Create Realistic Materials**:
   ```csharp
   // RobotMaterialController.cs
   using UnityEngine;

   public class RobotMaterialController : MonoBehaviour
   {
       [Header("Robot Materials")]
       public Material bodyMaterial;
       public Material headMaterial;
       public Material limbMaterial;

       [Header("Visual Effects")]
       public Light leftEyeLight;
       public Light rightEyeLight;
       public GameObject statusLight;

       // Material properties
       private Renderer bodyRenderer;
       private Renderer headRenderer;
       private Renderer[] limbRenderers;

       void Start()
       {
           // Get renderers
           bodyRenderer = transform.Find("Body").GetComponent<Renderer>();
           headRenderer = transform.Find("Head").GetComponent<Renderer>();

           limbRenderers = new Renderer[]
           {
               transform.Find("LeftArm").GetComponent<Renderer>(),
               transform.Find("RightArm").GetComponent<Renderer>(),
               transform.Find("LeftLeg").GetComponent<Renderer>(),
               transform.Find("RightLeg").GetComponent<Renderer>()
           };

           // Apply materials
           if (bodyRenderer != null && bodyMaterial != null)
               bodyRenderer.material = bodyMaterial;
           if (headRenderer != null && headMaterial != null)
               headRenderer.material = headMaterial;

           foreach (var renderer in limbRenderers)
           {
               if (renderer != null && limbMaterial != null)
                   renderer.material = limbMaterial;
           }
       }

       // Update robot status based on simulation state
       public void UpdateRobotStatus(bool isWorking, float batteryLevel)
       {
           // Change material color based on status
           if (bodyRenderer != null)
           {
               if (isWorking)
                   bodyRenderer.material.color = Color.green;
               else
                   bodyRenderer.material.color = Color.gray;
           }

           // Update status light
           if (statusLight != null)
           {
               statusLight.SetActive(true);
               if (batteryLevel < 0.2f)
                   statusLight.GetComponent<Renderer>().material.color = Color.red;
               else if (batteryLevel < 0.5f)
                   statusLight.GetComponent<Renderer>().material.color = Color.yellow;
               else
                   statusLight.GetComponent<Renderer>().material.color = Color.green;
           }

           // Control eye lights
           if (leftEyeLight != null)
               leftEyeLight.enabled = isWorking;
           if (rightEyeLight != null)
               rightEyeLight.enabled = isWorking;
       }
   }
   ```

3. **Implement Physics-Based Interactions**:
   ```csharp
   // PhysicsInteraction.cs
   using UnityEngine;

   public class PhysicsInteraction : MonoBehaviour
   {
       [Header("Interaction Settings")]
       public float interactionDistance = 5f;
       public LayerMask interactionLayer;

       private Camera mainCamera;
       private RobotModel robotModel;

       void Start()
       {
           mainCamera = Camera.main;
           robotModel = FindObjectOfType<RobotModel>();
       }

       void Update()
       {
           // Check for interaction using raycasting
           if (Input.GetMouseButtonDown(0))
           {
               Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
               RaycastHit hit;

               if (Physics.Raycast(ray, out hit, interactionDistance, interactionLayer))
               {
                   // Handle interaction with clicked object
                   HandleInteraction(hit.collider.gameObject);
               }
           }
       }

       void HandleInteraction(GameObject target)
       {
           // Example interaction: make the robot look at the clicked object
           if (robotModel != null && target.CompareTag("Interactive"))
           {
               Vector3 direction = target.transform.position - robotModel.head.position;
               direction.y = 0; // Keep head level

               float angle = Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg;
               robotModel.SetHeadRotation(angle, 0);

               Debug.Log($"Robot looking at {target.name}");
           }
       }
   }
   ```

4. **Enhance the Scene with Lighting and Effects**:
   - Add ambient lighting to match Gazebo environment
   - Use reflection probes for realistic reflections
   - Add post-processing effects (bloom, ambient occlusion)
   - Implement shadows for realistic depth perception

5. **Test Advanced Visualization**:
   - Run the Unity scene with advanced rendering
   - Verify realistic materials and lighting
   - Test physics-based interactions
   - Confirm visual quality matches the requirements

**Expected Outcome**: A fully functional Unity visualization with realistic rendering, physics interactions, and material properties that match the Gazebo simulation.

**Learning Points**:
- Understanding realistic rendering techniques in Unity
- Learning to implement physics-based interactions
- Recognizing the importance of visual quality in digital twins

## Troubleshooting Unity Visualization Issues

### Connection Problems
- **Problem**: ROS and Unity not communicating
- **Solution**: Check network configuration, ensure ROS bridge is running, verify topic names

### Performance Issues
- **Problem**: Low frame rate in Unity
- **Solution**: Reduce rendering complexity, optimize materials, use level of detail (LOD)

### Synchronization Problems
- **Problem**: Unity visualization out of sync with Gazebo
- **Solution**: Implement proper time synchronization, adjust update rates

### Material Problems
- **Problem**: Materials appear incorrectly in Unity
- **Solution**: Verify shader compatibility, check texture formats and import settings