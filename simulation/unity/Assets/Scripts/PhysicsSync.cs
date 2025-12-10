using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.Messages.Geometry;
using RosSharp.Messages.Std;

public class PhysicsSync : MonoBehaviour
{
    [Header("Synchronization Settings")]
    public float syncRate = 60f; // Sync 60 times per second
    private float syncTimer = 0f;
    
    [Header("ROS Communication")]
    public RosCommunication rosComm;
    
    [Header("Gazebo Objects")]
    public List<GameObject> gazeboObjects = new List<GameObject>();
    public Dictionary<string, GameObject> objectMap = new Dictionary<string, GameObject>();
    
    [Header("Physics Visualization")]
    public Material defaultMaterial;
    public Material selectedMaterial;
    
    private RosBridgeClient.MessageSubscribers.TwistStampedSubscriber velocitySubscriber;
    private RosBridgeClient.MessageSubscribers.OdometrySubscriber odometrySubscriber;
    
    void Start()
    {
        InitializeSynchronization();
        StartCoroutine(SetupRosSubscribers());
    }
    
    void Update()
    {
        syncTimer += Time.deltaTime;
        
        if (syncTimer >= (1.0f / syncRate) && rosComm != null && rosComm.isConnected)
        {
            SyncPhysicsWithGazebo();
            syncTimer = 0f;
        }
    }
    
    void InitializeSynchronization()
    {
        // Initialize the object map with any existing gazebo objects
        foreach (GameObject obj in gazeboObjects)
        {
            if (!objectMap.ContainsKey(obj.name))
            {
                objectMap.Add(obj.name, obj);
            }
        }
    }
    
    IEnumerator SetupRosSubscribers()
    {
        // Wait for ROS communication to be established
        yield return new WaitUntil(() => rosComm != null && rosComm.isConnected);
        
        if (rosComm.rosSocket != null && rosComm.rosSocket.State == WebSocketSharp.WebSocketState.Open)
        {
            // Subscribe to odometry messages for physics synchronization
            odometrySubscriber = new RosBridgeClient.MessageSubscribers.OdometrySubscriber(
                rosComm.rosSocket,
                "/gazebo_odom",
                OnOdomReceived
            );
        }
    }
    
    void OnOdomReceived(Odometry odom)
    {
        string robotName = odom.child_frame_id; // Usually contains the robot name
        
        // Update the robot's position and orientation based on odometry
        if (objectMap.ContainsKey(robotName))
        {
            GameObject robot = objectMap[robotName];
            
            // Update position
            Vector3 newPosition = new Vector3(
                (float)odom.pose.pose.position.x,
                (float)odom.pose.pose.position.z, // ROS to Unity coordinate conversion
                (float)odom.pose.pose.position.y
            );
            
            // Update rotation
            Quaternion newRotation = new Quaternion(
                -(float)odom.pose.pose.orientation.x, // Negating to match coordinate system
                -(float)odom.pose.pose.orientation.z,
                -(float)odom.pose.pose.orientation.y,
                (float)odom.pose.pose.orientation.w
            );
            
            // Smoothly update the robot's transform
            robot.transform.position = newPosition;
            robot.transform.rotation = newRotation;
        }
    }
    
    void SyncPhysicsWithGazebo()
    {
        // Process any physics synchronization updates
        foreach (var obj in objectMap)
        {
            // For each object, verify its state matches what's expected from Gazebo
            GameObject unityObject = obj.Value;
            string objectName = obj.Key;
            
            // In a real implementation, we would query Gazebo for the state of each object
            // and update the Unity representation accordingly
            
            // For now, we'll just log that synchronization is happening
            // Debug.Log($"Syncing object: {objectName}");
        }
    }
    
    // Method to add an object to the synchronization system
    public void RegisterObjectForSync(string objectName, GameObject unityObject)
    {
        if (!objectMap.ContainsKey(objectName))
        {
            objectMap.Add(objectName, unityObject);
        }
        else
        {
            objectMap[objectName] = unityObject;
        }
        
        if (!gazeboObjects.Contains(unityObject))
        {
            gazeboObjects.Add(unityObject);
        }
    }
    
    // Method to update an object's properties from Gazebo
    public void UpdateObjectProperties(string objectName, Vector3 position, Quaternion rotation, Vector3 scale)
    {
        if (objectMap.ContainsKey(objectName))
        {
            GameObject obj = objectMap[objectName];
            obj.transform.position = position;
            obj.transform.rotation = rotation;
            obj.transform.localScale = scale;
        }
    }
    
    // Method to handle object creation/destruction synchronization
    public void HandleObjectCreation(string objectName, string objectType, Vector3 position, Quaternion rotation)
    {
        // In a real implementation, this would create a new Unity object
        // matching the type and properties of the object created in Gazebo
        Debug.Log($"Object created in Gazebo: {objectName} ({objectType}) at {position}");
    }
    
    public void HandleObjectDestruction(string objectName)
    {
        // In a real implementation, this would destroy the corresponding Unity object
        Debug.Log($"Object destroyed in Gazebo: {objectName}");
        
        if (objectMap.ContainsKey(objectName))
        {
            GameObject obj = objectMap[objectName];
            objectMap.Remove(objectName);
            gazeboObjects.Remove(obj);
            
            if (obj != null)
            {
                Destroy(obj);
            }
        }
    }
    
    // Visualize physics properties for debugging
    void OnValidate()
    {
        // Validate that we have required references
        if (rosComm == null)
        {
            rosComm = FindObjectOfType<RosCommunication>();
        }
    }
}