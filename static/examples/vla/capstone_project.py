#!/usr/bin/env python3

"""
Complete capstone project: Autonomous Humanoid Robot.

This system integrates all course modules into a complete autonomous robot
that can understand voice commands and execute complex tasks.
"""

import rclpy
from rclpy.node import Node
import openai
import json
import os
import threading
import time
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from functools import partial
from nav_msgs.msg import Odometry
import math


class CapstoneNode(Node):
    """Capstone project node integrating all course modules."""
    
    def __init__(self):
        super().__init__('capstone_node')
        
        # Initialize OpenAI API key
        openai.api_key = os.getenv('OPENAI_API_KEY')
        
        # Publishers for all systems
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_goal_publisher = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.action_publisher = self.create_publisher(String, '/robot_actions', 10)
        self.task_status_publisher = self.create_publisher(String, '/task_status', 10)
        
        # Subscribers for all systems
        self.voice_command_subscription = self.create_subscription(
            String, '/voice_command', self.voice_command_callback, 10)
        
        self.vision_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.vision_callback, 10)
        
        self.laser_subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
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
        self.laser_scan = None
        
        # CV Bridge for image processing
        self.cv_bridge = CvBridge()
        
        # Threading lock
        self.lock = threading.RLock()
        
        # Timer for system monitoring
        self.monitor_timer = self.create_timer(0.5, self.system_monitor)
        
        # Performance tracking
        self.start_time = time.time()
        
        self.get_logger().info("Autonomous Humanoid Capstone System Initialized")
        
        # Publish system ready status
        ready_msg = String()
        ready_msg.data = "system_ready"
        self.task_status_publisher.publish(ready_msg)

    def system_monitor(self):
        """Monitor system status and handle any issues."""
        with self.lock:
            if not self.is_executing and self.task_queue:
                # Start executing the next task if available
                self.execute_next_task()

        # Log system status periodically
        if time.time() - self.start_time > 30:  # Log every 30 seconds of operation
            self.get_logger().info(f"System status - Status: {self.robot_status}, "
                                 f"Tasks in queue: {len(self.task_queue)}, "
                                 f"Objects detected: {len(self.environment_objects)}")

    def voice_command_callback(self, msg):
        """Handle voice commands from the user."""
        command_text = msg.data
        self.get_logger().info(f"Received voice command: {command_text}")
        
        # Generate plan using LLM
        plan = self.generate_comprehensive_plan(command_text)
        
        if plan:
            # Add plan to task queue
            with self.lock:
                self.task_queue.extend(plan)
            
            self.get_logger().info(f"Added comprehensive plan with {len(plan)} steps to queue")
            
            # If not currently executing, start the first task
            if not self.is_executing:
                self.execute_next_task()
        else:
            self.get_logger().error("Could not generate comprehensive plan for command")

    def vision_callback(self, msg):
        """Handle vision data."""
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Store latest vision data
            with self.lock:
                self.vision_data = cv_image
            
            # Process image to identify objects
            detected_objects = self.detect_comprehensive_objects(cv_image)
            
            # Update environment objects
            with self.lock:
                self.environment_objects = detected_objects
            
            self.get_logger().info(f"Detected {len(detected_objects)} objects")
                
        except Exception as e:
            self.get_logger().error(f"Error processing vision data: {str(e)}")

    def laser_callback(self, msg):
        """Handle laser scan data for navigation."""
        with self.lock:
            self.laser_scan = msg

    def odom_callback(self, msg):
        """Handle odometry data for localization."""
        with self.lock:
            self.current_location = {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "theta": 2 * math.atan2(msg.pose.pose.orientation.z, 
                                      msg.pose.pose.orientation.w)
            }

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

    def detect_comprehensive_objects(self, image):
        """Detect objects using multiple computer vision techniques."""
        # 1. Color-based detection
        color_objects = self.detect_color_objects(image)
        
        # 2. Shape-based detection  
        shape_objects = self.detect_shape_objects(image)
        
        # 3. Size-based detection (using contour area)
        size_objects = self.detect_size_objects(image)
        
        # Combine all detections with unique IDs
        all_objects = []
        id_counter = 0
        
        for obj in color_objects:
            obj['id'] = f"color_{id_counter}"
            all_objects.append(obj)
            id_counter += 1
            
        for obj in shape_objects:
            obj['id'] = f"shape_{id_counter}"
            all_objects.append(obj)
            id_counter += 1
            
        for obj in size_objects:
            obj['id'] = f"size_{id_counter}"
            all_objects.append(obj)
            id_counter += 1
        
        return all_objects

    def detect_color_objects(self, image):
        """Detect objects based on color."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define ranges for common colors
        color_ranges = {
            "red": (np.array([0, 100, 100]), np.array([10, 255, 255])),
            "blue": (np.array([100, 100, 100]), np.array([130, 255, 255])),
            "green": (np.array([40, 100, 100]), np.array([80, 255, 255])),
            "yellow": (np.array([20, 100, 100]), np.array([30, 255, 255])),
        }
        
        objects = []
        
        for color_name, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, lower, upper)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Filter contours by area
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 300:  # Minimum area threshold
                    # Calculate the center of the contour
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        
                        # Convert to relative coordinates (simplified)
                        rel_x = (cX - image.shape[1]/2) / (image.shape[1]/2)  # -1 to 1
                        rel_y = (cY - image.shape[0]/2) / (image.shape[0]/2)  # -1 to 1
                        
                        objects.append({
                            "name": f"{color_name}_{len(objects)}",
                            "color": color_name,
                            "x": rel_x,
                            "y": rel_y,
                            "area": area,
                            "type": "color"
                        })
        
        return objects

    def detect_shape_objects(self, image):
        """Detect objects based on shape."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 50, 150)
        
        shapes = []
        contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                # Approximate the contour to identify shape
                peri = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                
                if len(approx) == 3:
                    shape = "triangle"
                elif len(approx) == 4:
                    # Check if it's a square or rectangle
                    (x, y, w, h) = cv2.boundingRect(approx)
                    ar = w / float(h)
                    shape = "square" if ar >= 0.8 and ar <= 1.2 else "rectangle"
                elif len(approx) == 5:
                    shape = "pentagon"
                else:
                    # Calculate circularity to distinguish circles from other shapes
                    area = cv2.contourArea(contour)
                    perimeter = cv2.arcLength(contour, True)
                    if perimeter > 0:
                        circularity = 4 * math.pi * area / (perimeter * perimeter)
                        if circularity > 0.8:
                            shape = "circle"
                        else:
                            shape = "complex"
                    else:
                        shape = "complex"
                
                # Calculate center
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    # Convert to relative coordinates (simplified)
                    rel_x = (cX - image.shape[1]/2) / (image.shape[1]/2)  # -1 to 1
                    rel_y = (cY - image.shape[0]/2) / (image.shape[0]/2)  # -1 to 1
                    
                    shapes.append({
                        "name": f"{shape}_{len(shapes)}",
                        "type": shape,
                        "x": rel_x,
                        "y": rel_y,
                        "area": area,
                        "shape": shape
                    })
        
        return shapes

    def detect_size_objects(self, image):
        """Detect objects based on size (contour area)."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Categorize by size
            if 100 < area < 500:
                size_category = "small"
            elif 500 <= area < 2000:
                size_category = "medium"
            elif area >= 2000:
                size_category = "large"
            else:
                continue  # Skip very small contours
            
            # Calculate center
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                # Convert to relative coordinates (simplified)
                rel_x = (cX - image.shape[1]/2) / (image.shape[1]/2)  # -1 to 1
                rel_y = (cY - image.shape[0]/2) / (image.shape[0]/2)  # -1 to 1
                
                objects.append({
                    "name": f"{size_category}_object_{len(objects)}",
                    "size": size_category,
                    "x": rel_x,
                    "y": rel_y,
                    "area": area,
                    "type": "size"
                })
        
        return objects

    def generate_comprehensive_plan(self, command_text):
        """Generate a comprehensive plan using LLM with full environmental context."""
        try:
            # Build comprehensive context
            env_context = {
                "current_location": self.current_location,
                "detected_objects": [
                    {
                        "name": obj['name'],
                        "x": obj['x'],
                        "y": obj['y'],
                        "type": obj.get('type', 'unknown'),
                        "color": obj.get('color', 'unknown'),
                        "shape": obj.get('shape', 'unknown'),
                        "size": obj.get('size', 'unknown')
                    }
                    for obj in self.environment_objects
                ],
                "robot_capabilities": ["navigate", "grasp", "speak", "manipulate", "detect_objects"],
                "robot_status": self.robot_status,
                "environment_constraints": self.get_environment_constraints()
            }
            
            system_prompt = f"""
            You are a sophisticated task planner for an autonomous humanoid robot. Create a comprehensive plan that integrates multiple modules (ROS 2, Gazebo/Unity simulation, NVIDIA Isaac perception, and VLA) based on the user's command.
            
            Current Context:
            - Robot Location: {env_context['current_location']}
            - Objects in environment: {[{'name': o['name'], 'type': o['type'], 'pos': (o['x'], o['y'])} for o in env_context['detected_objects'][:5]]}  # Limiting for brevity
            - Robot status: {env_context['robot_status']}
            - Environmental constraints: {env_context['environment_constraints']}
            
            Available Actions:
            - navigate_to: Move robot to (x, y) coordinates
            - detect_object: Look for object with specific name/type/attribute
            - grasp_object: Pick up object at location
            - place_object: Put down object at location
            - speak: Make robot say text
            - inspect_area: Look around current location
            - wait: Pause for duration
            - confirm_action: Ask user for confirmation before proceeding
            - search_object: Look for object in environment
            - avoid_obstacle: Navigate around detected obstacles
            
            Each action should be in JSON format:
            {{
                "action": "action_name",
                "parameters": {{
                    "param_name": "param_value"
                }},
                "description": "Brief description of what this action does",
                "module_used": "Which course module this action relates to (Module 1-4)"
            }}
            
            Consider the full environmental context when making plans. Account for object locations, robot position, and environmental constraints. The plan should integrate concepts from all 4 course modules.
            """
            
            user_prompt = f"""
            Command: "{command_text}"
            
            Generate a comprehensive JSON array of actions to execute this command, integrating all course modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA):
            """
            
            response = openai.ChatCompletion.create(
                model="gpt-4",  # Using GPT-4 for more complex reasoning
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.2,  # Lower temperature for more consistent planning
                max_tokens=1500   # More tokens for complex plans
            )
            
            response_text = response.choices[0].message.content
            
            # Extract JSON from response
            start_idx = response_text.find('[')
            end_idx = response_text.rfind(']') + 1
            
            if start_idx != -1 and end_idx != 0:
                json_str = response_text[start_idx:end_idx]
                plan = json.loads(json_str)
                
                self.get_logger().info(f"Generated comprehensive plan with {len(plan)} steps:")
                for i, step in enumerate(plan):
                    module = step.get('module_used', 'Unknown')
                    self.get_logger().info(f"  {i+1}. {step.get('description', step['action'])} [Module: {module}]")
                
                return plan
            else:
                self.get_logger().error(f"Could not parse plan from response: {response_text}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error generating comprehensive plan: {str(e)}")
            return None

    def get_environment_constraints(self):
        """Determine environment constraints from sensor data."""
        constraints = {
            "obstacles": [],
            "navigation_zones": ["safe", "caution", "avoid"],
            "manipulation_zones": ["reachable", "unreachable"],
            "object_density": "low" if len(self.environment_objects) < 5 else "high"
        }
        
        # If we have laser scan data, extract obstacles
        if self.laser_scan:
            ranges = np.array(self.laser_scan.ranges)
            # Filter out invalid ranges
            valid_ranges = ranges[np.isfinite(ranges)]
            
            if len(valid_ranges) > 0:
                min_distance = np.min(valid_ranges)
                if min_distance < 0.5:  # Less than 0.5m is considered close obstacle
                    constraints["obstacles"].append({
                        "distance": min_distance,
                        "direction": "front",  # Simplified
                        "type": "close_obstacle"
                    })
        
        return constraints

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
        module_used = self.current_task.get('module_used', 'Unknown')
        
        self.get_logger().info(f"Executing task: {action} [Module: {module_used}] with params: {params}")
        
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
        elif action == 'search_object':
            self.execute_search_object(params)
        elif action == 'avoid_obstacle':
            self.execute_avoid_obstacle(params)
        else:
            self.get_logger().error(f"Unknown action: {action}")
            self.finish_task()

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
        goal_msg.pose.orientation.w = 1.0  # No rotation for simplicity
        
        self.nav_goal_publisher.publish(goal_msg.pose)
        self.get_logger().info(f"Navigating to ({x}, {y})")
        
        # In simulation, we'll use a timer; in real implementation, we'd wait for navigation completion
        timer = self.create_timer(5.0, self.finish_task)

    def execute_detect_object(self, params):
        """Execute object detection task."""
        object_name = params.get('object_name', '')
        
        # Check if the object is in our current environment
        found_objects = [obj for obj in self.environment_objects 
                        if object_name.lower() in obj['name'].lower() or 
                           obj['name'].lower() in object_name.lower()]
        
        if found_objects:
            obj = found_objects[0]  # Use the first match
            self.get_logger().info(f"Found {object_name} at ({obj['x']:.2f}, {obj['y']:.2f})")
            
            # Publish result for potential next actions
            result_msg = String()
            result_msg.data = f"object_found:{object_name}:{obj['x']:.2f}:{obj['y']:.2f}"
            self.action_publisher.publish(result_msg)
        else:
            self.get_logger().info(f"Could not find {object_name} in environment")
            result_msg = String()
            result_msg.data = f"object_not_found:{object_name}"
            self.action_publisher.publish(result_msg)
        
        timer = self.create_timer(2.0, self.finish_task)

    def execute_grasp_object(self, params):
        """Execute object grasping task."""
        object_name = params.get('object_name', '')
        
        # Find the object to grasp
        target_objects = [obj for obj in self.environment_objects 
                         if object_name.lower() in obj['name'].lower() or 
                            obj['name'].lower() in object_name.lower()]
        
        if target_objects:
            target = target_objects[0]
            self.get_logger().info(f"Attempting to grasp {object_name} at ({target['x']:.2f}, {target['y']:.2f})")
            
            # Publish grasping command
            action_msg = String()
            action_msg.data = f"grasp_object:{object_name}"
            self.action_publisher.publish(action_msg)
        else:
            self.get_logger().error(f"Could not find {object_name} to grasp")
            action_msg = String()
            action_msg.data = f"search_object:{object_name}"
            self.action_publisher.publish(action_msg)
        
        timer = self.create_timer(3.0, self.finish_task)

    def execute_place_object(self, params):
        """Execute object placement task."""
        location = params.get('location', 'default')
        
        self.get_logger().info(f"Placing object at {location}")
        
        action_msg = String()
        action_msg.data = f"place_object:{location}"
        self.action_publisher.publish(action_msg)
        
        timer = self.create_timer(2.0, self.finish_task)

    def execute_speak(self, params):
        """Execute speaking task."""
        text = params.get('text', 'Hello')
        
        self.get_logger().info(f"Speaking: {text}")
        
        action_msg = String()
        action_msg.data = f"speak:{text}"
        self.action_publisher.publish(action_msg)
        
        timer = self.create_timer(2.0, self.finish_task)

    def execute_inspect_area(self, params):
        """Execute area inspection task."""
        self.get_logger().info("Inspecting current area")
        
        # Update environment objects based on current vision
        if self.vision_data is not None:
            self.environment_objects = self.detect_comprehensive_objects(self.vision_data)
        
        action_msg = String()
        action_msg.data = "inspect_area"
        self.action_publisher.publish(action_msg)
        
        timer = self.create_timer(2.0, self.finish_task)

    def execute_wait(self, params):
        """Execute wait task."""
        duration = params.get('duration', 1.0)
        
        self.get_logger().info(f"Waiting for {duration} seconds")
        
        timer = self.create_timer(duration, self.finish_task)

    def execute_confirm_action(self, params):
        """Execute action confirmation task."""
        action_desc = params.get('action_description', 'unknown action')
        
        self.get_logger().info(f"Requesting confirmation for: {action_desc}")
        
        action_msg = String()
        action_msg.data = f"request_confirmation:{action_desc}"
        self.action_publisher.publish(action_msg)
        
        timer = self.create_timer(3.0, self.finish_task)

    def execute_search_object(self, params):
        """Execute object search task."""
        object_name = params.get('object_name', 'unknown')
        
        self.get_logger().info(f"Searching for object: {object_name}")
        
        # In a real implementation, this would trigger a more extensive search pattern
        # For now, we'll just update our environment understanding
        if self.vision_data is not None:
            self.environment_objects = self.detect_comprehensive_objects(self.vision_data)
        
        action_msg = String()
        action_msg.data = f"search_object:{object_name}"
        self.action_publisher.publish(action_msg)
        
        timer = self.create_timer(2.0, self.finish_task)

    def execute_avoid_obstacle(self, params):
        """Execute obstacle avoidance task."""
        self.get_logger().info("Executing obstacle avoidance")
        
        # In a real implementation, this would use laser scan data to navigate around obstacles
        # For simulation, we'll just log the action
        action_msg = String()
        action_msg.data = "avoid_obstacle"
        self.action_publisher.publish(action_msg)
        
        timer = self.create_timer(1.5, self.finish_task)

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
    """Main function to run the capstone project."""
    rclpy.init(args=args)
    
    capstone_node = CapstoneNode()
    
    try:
        rclpy.spin(capstone_node)
    except KeyboardInterrupt:
        pass
    finally:
        capstone_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()