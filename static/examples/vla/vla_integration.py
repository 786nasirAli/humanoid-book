#!/usr/bin/env python3

"""
Integrated Vision-Language-Action (VLA) system for humanoid robot.

This module integrates vision, language, and action components to create
a unified autonomous robot system.
"""

import rclpy
from rclpy.node import Node
import openai
import json
import os
import threading
import time
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from functools import partial


class VLAIntegrationNode(Node):
    """Integrated VLA system node."""
    
    def __init__(self):
        super().__init__('vla_integration_node')
        
        # Initialize OpenAI API key
        openai.api_key = os.getenv('OPENAI_API_KEY')
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_goal_publisher = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.action_publisher = self.create_publisher(String, '/robot_actions', 10)
        self.task_status_publisher = self.create_publisher(String, '/task_status', 10)
        
        # Subscribers
        self.voice_command_subscription = self.create_subscription(
            String, '/voice_command', self.voice_command_callback, 10)
        
        self.vision_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.vision_callback, 10)
        
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.pointcloud_callback, 10)
        
        self.robot_status_subscription = self.create_subscription(
            String, '/robot_status', self.status_callback, 10)
        
        self.task_execution_subscription = self.create_subscription(
            String, '/task_execution_result', self.task_execution_callback, 10)
        
        # Internal state
        self.robot_status = "idle"
        self.current_task = None
        self.task_queue = []
        self.vision_data = None
        self.environment_objects = []
        self.current_location = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.is_executing = False
        
        # CV Bridge for image processing
        self.cv_bridge = CvBridge()
        
        # Threading lock for safe access to shared state
        self.lock = threading.RLock()
        
        # Timer for system monitoring
        self.monitor_timer = self.create_timer(1.0, self.system_monitor)
        
        self.get_logger().info("Integrated VLA system initialized")

    def system_monitor(self):
        """Monitor system status and handle any issues."""
        with self.lock:
            if not self.is_executing and self.task_queue:
                # Start executing the next task if available
                self.execute_next_task()

    def voice_command_callback(self, msg):
        """Handle voice commands from the user."""
        command_text = msg.data
        self.get_logger().info(f"Received voice command: {command_text}")
        
        # Generate plan using LLM
        plan = self.generate_plan(command_text)
        
        if plan:
            # Add plan to task queue
            with self.lock:
                self.task_queue.extend(plan)
            
            self.get_logger().info(f"Added plan with {len(plan)} steps to queue")
            
            # If not currently executing, start the first task
            if not self.is_executing:
                self.execute_next_task()
        else:
            self.get_logger().error("Could not generate plan for command")

    def vision_callback(self, msg):
        """Handle vision data."""
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Store latest vision data
            with self.lock:
                self.vision_data = cv_image
            
            # Process image to identify objects
            self.environment_objects = self.detect_objects(cv_image)
            
            # If we're currently executing a task that requires vision,
            # check if we need to update our plan
            if self.is_executing and self.current_task:
                self.check_task_requirements()
                
        except Exception as e:
            self.get_logger().error(f"Error processing vision data: {str(e)}")

    def pointcloud_callback(self, msg):
        """Handle depth/pointcloud data."""
        # Process depth information for 3D scene understanding
        # In a real implementation, this would provide depth information
        # for object manipulation and navigation
        pass

    def status_callback(self, msg):
        """Handle robot status updates."""
        with self.lock:
            self.robot_status = msg.data
        self.get_logger().info(f"Robot status updated: {self.robot_status}")

    def task_execution_callback(self, msg):
        """Handle task execution results."""
        result = msg.data
        self.get_logger().info(f"Task execution result: {result}")
        
        # Check if the task completed successfully
        if result == "success":
            with self.lock:
                self.is_executing = False
        else:
            self.get_logger().warn(f"Task failed with result: {result}")
            with self.lock:
                self.is_executing = False

    def detect_objects(self, image):
        """Detect objects in the image using computer vision."""
        # For demonstration, we'll use basic color-based detection
        # In practice, this would use deep learning models like YOLO or Detectron2
        
        # Convert BGR to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define ranges for common colors
        color_ranges = {
            "red": (np.array([0, 50, 50]), np.array([10, 255, 255])),
            "blue": (np.array([100, 50, 50]), np.array([130, 255, 255])),
            "green": (np.array([40, 50, 50]), np.array([80, 255, 255])),
        }
        
        objects = []
        
        for obj_name, (lower, upper) in color_ranges.items():
            # Create mask for the color
            mask = cv2.inRange(hsv, lower, upper)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Filter contours by area to avoid small noise
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    # Calculate the center of the contour
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        
                        # Approximate real-world position (simplified)
                        # In a real system, this would use depth data or calibrated transforms
                        real_x = (cX - image.shape[1]/2) * 0.01  # Rough conversion to meters
                        real_y = (cY - image.shape[0]/2) * 0.01
                        
                        objects.append({
                            "name": f"{obj_name}_object_{len(objects)}",
                            "color": obj_name,
                            "x": real_x,
                            "y": real_y,
                            "area": area
                        })
        
        # Also identify shapes using contour approximation
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 50, 150)
        
        shapes = []
        contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Minimum area threshold
                # Approximate the contour to identify shape
                peri = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                
                if len(approx) == 3:
                    shape = "triangle"
                elif len(approx) == 4:
                    # Check if it's a square or rectangle
                    (x, y, w, h) = cv2.boundingRect(approx)
                    ar = w / float(h)
                    shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
                elif len(approx) == 5:
                    shape = "pentagon"
                else:
                    shape = "circle"
                
                # Calculate center
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    shapes.append({
                        "name": f"{shape}_{len(shapes)}",
                        "type": shape,
                        "x": (cX - image.shape[1]/2) * 0.01,
                        "y": (cY - image.shape[0]/2) * 0.01,
                        "area": area
                    })
        
        # Combine color-based and shape-based objects
        all_objects = objects + shapes
        
        self.get_logger().info(f"Detected {len(all_objects)} objects in image")
        return all_objects

    def generate_plan(self, command_text):
        """Generate a plan using LLM based on command and environmental context."""
        try:
            # Build context about the environment
            env_context = {
                "current_location": self.current_location,
                "detected_objects": [obj for obj in self.environment_objects],
                "robot_capabilities": ["navigate", "grasp", "speak", "manipulate"],
                "robot_status": self.robot_status
            }
            
            system_prompt = f"""
            You are a task planner for a humanoid robot. Your job is to break down high-level natural language commands into sequences of executable robotic actions.
            
            Current Context:
            - Robot Location: {env_context['current_location']}
            - Objects in environment: {[obj['name'] + '(' + str(obj['x']) + ',' + str(obj['y']) + ')' for obj in env_context['detected_objects']]}
            - Robot status: {env_context['robot_status']}
            
            Available actions:
            - navigate_to: Move robot to (x, y) coordinates
            - detect_object: Look for object with name
            - grasp_object: Pick up object at location
            - place_object: Put down object at location
            - speak: Make robot say text
            - inspect_area: Look around current location
            - wait: Pause for duration
            - confirm_action: Ask user for confirmation before proceeding
            
            Each action should be in JSON format:
            {{
                "action": "action_name",
                "parameters": {{
                    "param_name": "param_value"
                }},
                "description": "Brief description"
            }}
            
            Consider the environment context when making plans. Be specific about locations and objects.
            """
            
            user_prompt = f"""
            Command: "{command_text}"
            
            Generate a JSON array of actions to execute this command:
            """
            
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
                max_tokens=1000
            )
            
            response_text = response.choices[0].message.content
            
            # Extract JSON from response
            start_idx = response_text.find('[')
            end_idx = response_text.rfind(']') + 1
            
            if start_idx != -1 and end_idx != 0:
                json_str = response_text[start_idx:end_idx]
                plan = json.loads(json_str)
                
                self.get_logger().info(f"Generated plan with {len(plan)} steps:")
                for i, step in enumerate(plan):
                    self.get_logger().info(f"  {i+1}. {step.get('description', step['action'])}")
                
                return plan
            else:
                self.get_logger().error(f"Could not parse plan from response: {response_text}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error generating plan: {str(e)}")
            return None

    def execute_next_task(self):
        """Execute the next task in the queue."""
        with self.lock:
            if self.is_executing or not self.task_queue:
                return
            
            # Get the next task
            self.current_task = self.task_queue.pop(0)
            self.is_executing = True
        
        action = self.current_task['action']
        params = self.current_task.get('parameters', {})
        
        self.get_logger().info(f"Executing task: {action} with params: {params}")
        
        # Update task status
        status_msg = String()
        status_msg.data = f"executing:{action}"
        self.task_status_publisher.publish(status_msg)
        
        # Execute based on action type
        if action == 'navigate_to':
            self.execute_navigate(params)
        elif action == 'detect_object':
            self.execute_detect_object(params)
        elif action == 'grasp_object':
            self.execute_grasp_object(params)
        elif action == 'place_object':
            self.execute_place_object(params)
        elif action == 'speak':
            self.execute_speak(params)
        elif action == 'inspect_area':
            self.execute_inspect_area(params)
        elif action == 'wait':
            self.execute_wait(params)
        elif action == 'confirm_action':
            self.execute_confirm_action(params)
        else:
            self.get_logger().error(f"Unknown action: {action}")
            self.finish_task()

    def check_task_requirements(self):
        """Check if current task needs updated environmental information."""
        if not self.current_task:
            return
            
        action = self.current_task['action']
        
        # Some actions require fresh environmental information
        if action in ['navigate_to', 'grasp_object', 'detect_object']:
            # Re-process the current vision data to update environment
            with self.lock:
                if self.vision_data is not None:
                    self.environment_objects = self.detect_objects(self.vision_data)

    def execute_navigate(self, params):
        """Execute navigation task."""
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        
        # Create navigation goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        self.nav_goal_publisher.publish(goal_msg.pose)
        
        # In a real implementation, we'd wait for navigation to complete
        # For now, simulate with a timer
        timer = self.create_timer(5.0, self.finish_task)

    def execute_detect_object(self, params):
        """Execute object detection task."""
        object_name = params.get('object_name', '')
        
        # Check if the object is in our current environment
        found_object = None
        for obj in self.environment_objects:
            if object_name.lower() in obj['name'].lower() or obj['name'].lower() in object_name.lower():
                found_object = obj
                break
        
        if found_object:
            self.get_logger().info(f"Found {object_name} at ({found_object['x']}, {found_object['y']})")
            # Publish result for potential next actions
            result_msg = String()
            result_msg.data = f"object_found:{object_name}:{found_object['x']}:{found_object['y']}"
            self.action_publisher.publish(result_msg)
        else:
            self.get_logger().info(f"Could not find {object_name} in environment")
            # Publish result for potential error handling
            result_msg = String()
            result_msg.data = f"object_not_found:{object_name}"
            self.action_publisher.publish(result_msg)
        
        # Complete the task
        timer = self.create_timer(2.0, self.finish_task)

    def execute_grasp_object(self, params):
        """Execute object grasping task."""
        object_name = params.get('object_name', '')
        
        # Find the object to grasp
        target_object = None
        for obj in self.environment_objects:
            if object_name.lower() in obj['name'].lower() or obj['name'].lower() in object_name.lower():
                target_object = obj
                break
        
        if target_object:
            self.get_logger().info(f"Attempting to grasp {object_name} at ({target_object['x']}, {target_object['y']})")
            
            # Navigate to object if needed
            if abs(target_object['x']) > 0.5 or abs(target_object['y']) > 0.5:
                self.get_logger().info("Navigating to object before grasping")
                # In a real system, we'd navigate to the object first
                # For now, just publish the grasping command
                
            # Publish grasping command
            action_msg = String()
            action_msg.data = f"grasp_object:{object_name}"
            self.action_publisher.publish(action_msg)
        else:
            self.get_logger().error(f"Could not find {object_name} to grasp")
            # If the object isn't visible, we might need to search for it
            action_msg = String()
            action_msg.data = f"search_object:{object_name}"
            self.action_publisher.publish(action_msg)
        
        # Complete the task
        timer = self.create_timer(3.0, self.finish_task)

    def execute_place_object(self, params):
        """Execute object placement task."""
        location = params.get('location', 'default')
        
        self.get_logger().info(f"Placing object at {location}")
        
        # Publish placement command
        action_msg = String()
        action_msg.data = f"place_object:{location}"
        self.action_publisher.publish(action_msg)
        
        # Complete the task
        timer = self.create_timer(2.0, self.finish_task)

    def execute_speak(self, params):
        """Execute speaking task."""
        text = params.get('text', 'Hello')
        
        self.get_logger().info(f"Speaking: {text}")
        
        # Publish speaking command
        action_msg = String()
        action_msg.data = f"speak:{text}"
        self.action_publisher.publish(action_msg)
        
        # Complete the task
        timer = self.create_timer(2.0, self.finish_task)

    def execute_inspect_area(self, params):
        """Execute area inspection task."""
        self.get_logger().info("Inspecting current area")
        
        # Update environment objects based on current vision
        if self.vision_data is not None:
            self.environment_objects = self.detect_objects(self.vision_data)
        
        # Publish inspection command
        action_msg = String()
        action_msg.data = "inspect_area"
        self.action_publisher.publish(action_msg)
        
        # Complete the task
        timer = self.create_timer(2.0, self.finish_task)

    def execute_wait(self, params):
        """Execute wait task."""
        duration = params.get('duration', 1.0)
        
        self.get_logger().info(f"Waiting for {duration} seconds")
        
        # Complete the task after the specified duration
        timer = self.create_timer(duration, self.finish_task)

    def execute_confirm_action(self, params):
        """Execute action confirmation task."""
        action_desc = params.get('action_description', 'unknown action')
        
        self.get_logger().info(f"Requesting confirmation for: {action_desc}")
        
        # In a real system, this would ask the user for confirmation
        # For now, we'll automatically confirm after a delay
        action_msg = String()
        action_msg.data = f"request_confirmation:{action_desc}"
        self.action_publisher.publish(action_msg)
        
        # Complete the task
        timer = self.create_timer(3.0, self.finish_task)

    def finish_task(self):
        """Mark current task as finished and start next if available."""
        with self.lock:
            self.is_executing = False
            self.current_task = None
        
        # Publish task completion
        completion_msg = String()
        completion_msg.data = "task_completed"
        self.action_publisher.publish(completion_msg)
        
        # Check if there are more tasks in the queue
        if not self.is_executing:
            self.system_monitor()


def main(args=None):
    """Main function to run the integrated VLA system."""
    rclpy.init(args=args)
    
    vla_integration_node = VLAIntegrationNode()
    
    try:
        rclpy.spin(vla_integration_node)
    except KeyboardInterrupt:
        pass
    finally:
        vla_integration_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()