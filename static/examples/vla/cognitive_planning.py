#!/usr/bin/env python3

"""
Cognitive planning system for humanoid robot using LLMs.

This module handles task decomposition and planning using Large Language Models.
"""

import rclpy
from rclpy.node import Node
import openai
import json
import os
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Bool
from functools import partial


class CognitivePlanningNode(Node):
    """Cognitive planning node using LLMs for task decomposition."""
    
    def __init__(self):
        super().__init__('cognitive_planning_node')
        
        # Initialize OpenAI API key (should be set in environment)
        openai.api_key = os.getenv('OPENAI_API_KEY')
        
        # Publishers for planning outputs
        self.plan_publisher = self.create_publisher(String, '/task_plan', 10)
        self.nav_goal_publisher = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.action_publisher = self.create_publisher(String, '/robot_actions', 10)
        
        # Subscribers for environmental and status information
        self.command_subscription = self.create_subscription(
            String, '/high_level_command', self.command_callback, 10)
        
        self.status_subscription = self.create_subscription(
            String, '/robot_status', self.status_callback, 10)
        
        self.vision_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.vision_callback, 10)
        
        # Robot status
        self.robot_status = "idle"
        self.current_location = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.environment_objects = []
        
        # CV Bridge for image processing
        self.cv_bridge = CvBridge()
        
        # Task queue for managing multiple tasks
        self.task_queue = []
        
        self.get_logger().info("Cognitive planning node initialized")

    def command_callback(self, msg):
        """Handle high-level commands from user or other nodes."""
        command_text = msg.data
        self.get_logger().info(f"Received command: {command_text}")
        
        # Generate a plan for the command
        plan = self.generate_plan(command_text)
        
        if plan:
            # Execute the plan
            self.execute_plan(plan)
        else:
            self.get_logger().error("Could not generate plan for command")

    def status_callback(self, msg):
        """Handle robot status updates."""
        self.robot_status = msg.data
        self.get_logger().info(f"Robot status updated: {self.robot_status}")

    def vision_callback(self, msg):
        """Handle vision data to understand environment."""
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process image to identify objects (simplified)
            # In a real implementation, this would use YOLO, Detectron2, or similar
            height, width, channels = cv_image.shape
            
            # For demonstration, just identify object positions in image
            # In practice, this would run an object detection model
            objects = self.detect_objects(cv_image)
            self.environment_objects = objects
            
            self.get_logger().info(f"Detected {len(objects)} objects in environment")
            
        except Exception as e:
            self.get_logger().error(f"Error processing vision data: {str(e)}")

    def detect_objects(self, image):
        """Simplified object detection for demonstration."""
        # This would normally use a trained object detection model
        # For demonstration, we'll just create some mock objects
        mock_objects = [
            {"name": "red cup", "x": 1.5, "y": 2.0, "type": "movable"},
            {"name": "book", "x": -0.5, "y": 1.2, "type": "movable"},
            {"name": "chair", "x": 0.0, "y": -1.0, "type": "obstacle"},
            {"name": "desk", "x": 2.0, "y": 0.0, "type": "obstacle"}
        ]
        return mock_objects

    def generate_plan(self, command_text):
        """Generate a plan using LLM for the given command."""
        try:
            # Create the system prompt to instruct the LLM on planning
            system_prompt = """
            You are a task planner for a humanoid robot. Your job is to break down high-level natural language commands into sequences of executable robotic actions.
            
            Available actions:
            - navigate_to: Move the robot to a specified location
            - detect_object: Look for a specific object in the environment
            - grasp_object: Pick up an object
            - place_object: Put down an object at a location
            - inspect_area: Look around the current location
            - wait: Pause for a specified duration
            - speak: Make the robot say something
            
            Each action should have appropriate parameters like location coordinates, object names, etc.
            
            Respond ONLY with a JSON array of action objects. Each action should have the format:
            {
                "action": "action_name",
                "parameters": {
                    "param_name": "param_value"
                },
                "description": "Brief description of what this action does"
            }
            
            Be specific about locations and objects when possible, and include safety checks in the plan.
            """
            
            # Include environmental context in the user message
            env_context = f"Current robot location: {self.current_location}. "
            if self.environment_objects:
                objects_str = ", ".join([obj['name'] for obj in self.environment_objects])
                env_context += f"Objects detected: {objects_str}. "
            
            user_prompt = f"""
            Command: "{command_text}"
            
            Environment: {env_context}
            
            Task Plan:
            """
            
            # Call the OpenAI API
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",  # You can use "gpt-4" for more complex tasks
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,  # Lower temperature for more consistent planning
                max_tokens=1000
            )
            
            # Extract the plan from the response
            response_text = response.choices[0].message.content
            
            # Try to find JSON in the response
            start_idx = response_text.find('[')
            end_idx = response_text.rfind(']') + 1
            
            if start_idx != -1 and end_idx != 0:
                json_str = response_text[start_idx:end_idx]
                plan = json.loads(json_str)
                
                self.get_logger().info(f"Generated plan with {len(plan)} steps")
                for i, step in enumerate(plan):
                    self.get_logger().info(f"  {i+1}. {step.get('description', step['action'])}")
                
                return plan
            else:
                self.get_logger().error(f"Could not parse plan from LLM response: {response_text}")
                return None
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error parsing LLM response as JSON: {str(e)}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error generating plan with LLM: {str(e)}")
            return None

    def execute_plan(self, plan):
        """Execute the generated plan step by step."""
        self.get_logger().info(f"Starting execution of plan with {len(plan)} steps")
        
        # Add plan to task queue
        self.task_queue = plan[:]
        
        # Start executing the first task
        if self.task_queue:
            self.execute_next_task()
    
    def execute_next_task(self):
        """Execute the next task in the queue."""
        if not self.task_queue:
            self.get_logger().info("Plan execution complete")
            return
        
        # Get the next task
        task = self.task_queue.pop(0)
        action = task['action']
        params = task.get('parameters', {})
        
        self.get_logger().info(f"Executing task: {action} with params: {params}")
        
        # Execute based on action type
        if action == 'navigate_to':
            self.execute_navigate(params)
        elif action == 'detect_object':
            self.execute_detect_object(params)
        elif action == 'grasp_object':
            self.execute_grasp_object(params)
        elif action == 'place_object':
            self.execute_place_object(params)
        elif action == 'inspect_area':
            self.execute_inspect_area(params)
        elif action == 'wait':
            self.execute_wait(params)
        elif action == 'speak':
            self.execute_speak(params)
        else:
            self.get_logger().error(f"Unknown action: {action}")
            # Continue with next task
            if self.task_queue:
                timer = self.create_timer(1.0, self.execute_next_task)
    
    def execute_navigate(self, params):
        """Execute navigation action."""
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        
        # Create and publish navigation goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0  # Assuming 2D navigation
        goal_msg.pose.orientation.w = 1.0  # No rotation for simplicity
        
        self.nav_goal_publisher.publish(goal_msg.pose)
        
        self.get_logger().info(f"Navigating to ({x}, {y})")
        
        # In a real implementation, you'd wait for navigation completion
        # For now, just continue to the next task after a delay
        timer = self.create_timer(5.0, self.execute_next_task)
    
    def execute_detect_object(self, params):
        """Execute object detection action."""
        object_name = params.get('object_name', '')
        
        self.get_logger().info(f"Detecting object: {object_name}")
        
        # In a real implementation, this would trigger vision processing
        # to locate a specific object
        
        # For now, just check if the object exists in our environment
        found = any(obj['name'] == object_name for obj in self.environment_objects)
        
        if found:
            self.get_logger().info(f"Found {object_name} in environment")
        else:
            self.get_logger().info(f"{object_name} not found in current view")
        
        # Continue to next task
        timer = self.create_timer(2.0, self.execute_next_task)
    
    def execute_grasp_object(self, params):
        """Execute object grasping action."""
        object_name = params.get('object_name', '')
        
        self.get_logger().info(f"Attempting to grasp: {object_name}")
        
        # In a real implementation, this would control the robot's gripper/arm
        # to grasp the specified object
        
        # For now, publish an action command
        action_msg = String()
        action_msg.data = f"grasp_object:{object_name}"
        self.action_publisher.publish(action_msg)
        
        # Continue to next task after delay
        timer = self.create_timer(3.0, self.execute_next_task)
    
    def execute_place_object(self, params):
        """Execute object placement action."""
        location = params.get('location', 'default')
        
        self.get_logger().info(f"Placing object at: {location}")
        
        # In a real implementation, this would control the robot to
        # place the held object at the specified location
        
        # For now, publish an action command
        action_msg = String()
        action_msg.data = f"place_object:{location}"
        self.action_publisher.publish(action_msg)
        
        # Continue to next task after delay
        timer = self.create_timer(3.0, self.execute_next_task)
    
    def execute_inspect_area(self, params):
        """Execute area inspection action."""
        self.get_logger().info("Inspecting current area")
        
        # In a real implementation, this might trigger
        # a scanning or exploration behavior
        
        # For now, just log and continue
        action_msg = String()
        action_msg.data = "inspect_area"
        self.action_publisher.publish(action_msg)
        
        # Continue to next task after delay
        timer = self.create_timer(2.0, self.execute_next_task)
    
    def execute_wait(self, params):
        """Execute wait action."""
        duration = params.get('duration', 1.0)
        
        self.get_logger().info(f"Waiting for {duration} seconds")
        
        # Continue to next task after specified duration
        timer = self.create_timer(duration, self.execute_next_task)
    
    def execute_speak(self, params):
        """Execute speaking action."""
        text = params.get('text', 'Hello')
        
        self.get_logger().info(f"Speaking: {text}")
        
        # In a real implementation, this would trigger text-to-speech
        
        # For now, publish the text as an action
        action_msg = String()
        action_msg.data = f"speak:{text}"
        self.action_publisher.publish(action_msg)
        
        # Continue to next task after delay
        timer = self.create_timer(2.0, self.execute_next_task)


def main(args=None):
    """Main function to run the cognitive planning node."""
    rclpy.init(args=args)
    
    cognitive_planning_node = CognitivePlanningNode()
    
    try:
        rclpy.spin(cognitive_planning_node)
    except KeyboardInterrupt:
        pass
    finally:
        cognitive_planning_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()