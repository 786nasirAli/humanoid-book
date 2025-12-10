using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.Messages.Geometry;
using RosSharp.Messages.Sensor;

public class RosCommunication : MonoBehaviour
{
    public string rosBridgeServerUrl = "ws://192.168.1.1:9090";  // Default ROS bridge URL
    private RosBridgeClient.RosBridgeWebSocketConnection rosSocket;
    
    [Header("Robot Status")]
    public bool isConnected = false;
    public float connectionTimer = 0f;
    
    [Header("Robot State")]
    public Transform robotTransform;
    public Vector3 rosRobotPosition = Vector3.zero;
    public Quaternion rosRobotRotation = Quaternion.identity;
    
    [Header("Sensor Data")]
    public float lidarUpdateRate = 10f; // Hz
    public float cameraUpdateRate = 5f;  // Hz
    public float imuUpdateRate = 50f;    // Hz
    
    // Publishers and Subscribers
    private RosBridgeClient.MessageSubscribers.TransformStampedSubscriber poseSubscriber;
    private RosBridgeClient.MessagePublishers.PoseStampedPublisher posePublisher;
    
    void Start()
    {
        InitializeRosConnection();
    }
    
    void Update()
    {
        UpdateConnectionStatus();
        UpdateRobotFromRos();
    }
    
    void InitializeRosConnection()
    {
        // Initialize ROS connection
        rosSocket = new RosBridgeClient.RosBridgeWebSocketConnection(rosBridgeServerUrl);
        
        // Add event handlers
        rosSocket.OnConnected += OnRosConnected;
        rosSocket.OnClosed += OnRosDisconnected;
        rosSocket.OnError += OnRosError;
        
        // Attempt to connect
        rosSocket.Open();
    }
    
    void OnRosConnected()
    {
        Debug.Log("Connected to ROS bridge: " + rosBridgeServerUrl);
        isConnected = true;
        
        // Subscribe to robot pose
        SubscribeToRobotPose();
        
        // Setup publishers if needed
        SetupPublishers();
    }
    
    void OnRosDisconnected()
    {
        Debug.Log("Disconnected from ROS bridge");
        isConnected = false;
    }
    
    void OnRosError(string errorMessage)
    {
        Debug.LogError("ROS Error: " + errorMessage);
        isConnected = false;
    }
    
    void SubscribeToRobotPose()
    {
        if (rosSocket != null && rosSocket.State == WebSocketSharp.WebSocketState.Open)
        {
            // Subscribe to the robot's pose topic
            poseSubscriber = new RosBridgeClient.MessageSubscribers.TransformStampedSubscriber(
                rosSocket, 
                "/robot_pose", 
                OnRobotPoseReceived
            );
        }
    }
    
    void OnRobotPoseReceived(TransformStamped transformStamped)
    {
        // Update robot position and rotation from ROS
        rosRobotPosition = new Vector3(
            (float)transformStamped.transform.translation.x,
            (float)transformStamped.transform.translation.z, // Swapping Y and Z for Unity coordinate system
            (float)transformStamped.transform.translation.y
        );
        
        rosRobotRotation = new Quaternion(
            -(float)transformStamped.transform.rotation.x, // Negating to match coordinate system
            -(float)transformStamped.transform.rotation.z,
            -(float)transformStamped.transform.rotation.y,
            (float)transformStamped.transform.rotation.w
        );
    }
    
    void SetupPublishers()
    {
        if (rosSocket != null && rosSocket.State == WebSocketSharp.WebSocketState.Open)
        {
            // Setup publisher for Unity robot pose
            posePublisher = new RosBridgeClient.MessagePublishers.PoseStampedPublisher(
                rosSocket,
                "/unity_robot_pose"
            );
        }
    }
    
    void UpdateConnectionStatus()
    {
        connectionTimer += Time.deltaTime;
        
        // Try to reconnect if disconnected for more than 5 seconds
        if (!isConnected && connectionTimer > 5f)
        {
            rosSocket?.Close();
            rosSocket?.Open();
            connectionTimer = 0f;
        }
    }
    
    void UpdateRobotFromRos()
    {
        if (isConnected && robotTransform != null)
        {
            // Update the robot's position and rotation based on ROS data
            robotTransform.position = Vector3.Lerp(robotTransform.position, rosRobotPosition, Time.deltaTime * 5f);
            robotTransform.rotation = Quaternion.Slerp(robotTransform.rotation, rosRobotRotation, Time.deltaTime * 5f);
        }
    }
    
    // Method to publish Unity robot pose to ROS
    public void PublishUnityPose()
    {
        if (posePublisher != null && rosSocket != null && rosSocket.State == WebSocketSharp.WebSocketState.Open)
        {
            PoseStamped poseStamped = new PoseStamped();
            poseStamped.header.frame_id = "unity_frame";
            poseStamped.header.stamp = new TimeStamp();
            
            poseStamped.pose.position.x = robotTransform.position.x;
            poseStamped.pose.position.y = robotTransform.position.z; // Unity to ROS coordinate conversion
            poseStamped.pose.position.z = robotTransform.position.y;
            
            poseStamped.pose.orientation.x = robotTransform.rotation.x;
            poseStamped.pose.orientation.y = robotTransform.rotation.z;
            poseStamped.pose.orientation.z = robotTransform.rotation.y;
            poseStamped.pose.orientation.w = robotTransform.rotation.w;
            
            posePublisher.Publish(poseStamped);
        }
    }
    
    // Call this method to publish robot pose periodically
    void LateUpdate()
    {
        PublishUnityPose();
    }
    
    void OnApplicationQuit()
    {
        // Clean up ROS connection
        if (rosSocket != null)
        {
            rosSocket.Close();
        }
    }
}