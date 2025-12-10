#!/usr/bin/env python3

"""
Voice-to-Action system for humanoid robot using OpenAI Whisper.

This module handles speech recognition, intent classification,
and command execution for humanoid robots.
"""

import rclpy
from rclpy.node import Node
import openai
import speech_recognition as sr
import pyaudio
import wave
import json
import os
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Bool
from threading import Thread
import time


class VoiceToActionNode(Node):
    """Voice-to-Action node for humanoid robot control."""
    
    def __init__(self):
        super().__init__('voice_to_action_node')
        
        # Initialize OpenAI API key (should be set in environment)
        openai.api_key = os.getenv('OPENAI_API_KEY')
        
        # Publishers for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_publisher = self.create_publisher(String, '/robot_actions', 10)
        
        # Subscriber for robot status
        self.status_subscription = self.create_subscription(
            String, '/robot_status', self.status_callback, 10)
        
        # Audio recording parameters
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.chunk = 1024
        self.record_seconds = 3
        self.wave_output_filename = "command.wav"
        
        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Calibrate for ambient noise
        with self.microphone as source:
            self.get_logger().info("Calibrating for ambient noise...")
            self.recognizer.adjust_for_ambient_noise(source)
        
        # Timer for continuous listening
        self.listen_timer = self.create_timer(5.0, self.listen_for_command)
        
        # Robot status
        self.robot_status = "idle"
        
        # CV Bridge for image processing
        self.cv_bridge = CvBridge()
        
        self.get_logger().info("Voice-to-Action node initialized")

    def status_callback(self, msg):
        """Handle robot status updates."""
        self.robot_status = msg.data
        self.get_logger().info(f"Robot status updated: {self.robot_status}")

    def listen_for_command(self):
        """Listen for a voice command and process it."""
        if self.robot_status != "idle":
            self.get_logger().info("Robot is busy, skipping voice command")
            return
            
        self.get_logger().info("Listening for voice command...")
        
        try:
            with self.microphone as source:
                # Listen for audio with timeout
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                
            self.get_logger().info("Audio captured, processing...")
            
            # Save audio to file temporarily
            with open("temp_command.wav", "wb") as f:
                f.write(audio.get_wav_data())
            
            # Use Whisper API to transcribe
            with open("temp_command.wav", "rb") as audio_file:
                transcript = openai.Audio.transcribe("whisper-1", audio_file)
            
            command_text = transcript.text
            self.get_logger().info(f"Recognized command: {command_text}")
            
            # Process the command
            self.process_command(command_text)
            
        except sr.WaitTimeoutError:
            self.get_logger().info("No speech detected")
        except sr.UnknownValueError:
            self.get_logger().info("Could not understand audio")
        except Exception as e:
            self.get_logger().error(f"Error in voice recognition: {str(e)}")
    
    def process_command(self, command_text):
        """Process the recognized command and map to actions."""
        # For now, we'll use a simple keyword-based approach
        # In a real implementation, we would use an LLM for semantic understanding
        
        command_text = command_text.lower()
        
        # Determine action based on command
        if "move forward" in command_text or "go forward" in command_text:
            self.execute_movement("forward")
        elif "move backward" in command_text or "go backward" in command_text:
            self.execute_movement("backward")
        elif "turn left" in command_text:
            self.execute_movement("left")
        elif "turn right" in command_text:
            self.execute_movement("right")
        elif "stop" in command_text:
            self.execute_movement("stop")
        elif "dance" in command_text:
            self.execute_behavior("dance")
        elif "wave" in command_text:
            self.execute_behavior("wave")
        elif "look" in command_text and "around" in command_text:
            self.execute_behavior("look_around")
        elif "pick up" in command_text or "grasp" in command_text:
            self.execute_behavior("grasp_object")
        else:
            # Use LLM for more complex command interpretation
            action = self.interpret_complex_command(command_text)
            if action:
                self.execute_interpreted_action(action)
    
    def interpret_complex_command(self, command_text):
        """Use LLM to interpret complex natural language commands."""
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system", 
                        "content": "You are a command interpreter for a humanoid robot. Convert the user's command into specific robot actions. Respond with a JSON object containing action type and parameters."
                    },
                    {
                        "role": "user",
                        "content": f"Convert this command to robot actions: '{command_text}'. Provide response in JSON format like {{'action_type': 'move', 'direction': 'forward', 'distance': 1.0}} or {{'action_type': 'grasp', 'object': 'red cup'}} or {{'action_type': 'navigate', 'location': 'kitchen'}}."
                    }
                ]
            )
            
            # Parse the response
            response_text = response.choices[0].message.content
            self.get_logger().info(f"LLM response: {response_text}")
            
            # Extract JSON from response
            start_idx = response_text.find('{')
            end_idx = response_text.rfind('}') + 1
            
            if start_idx != -1 and end_idx != 0:
                action_json = response_text[start_idx:end_idx]
                return json.loads(action_json)
            else:
                self.get_logger().error("Could not parse LLM response as JSON")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error interpreting command with LLM: {str(e)}")
            return None
    
    def execute_interpreted_action(self, action):
        """Execute action as interpreted by LLM."""
        action_type = action.get('action_type', '')
        
        if action_type == 'move':
            direction = action.get('direction', 'stop')
            distance = action.get('distance', 1.0)  # meters
            self.execute_movement(direction, distance)
        elif action_type == 'grasp':
            obj = action.get('object', 'object')
            self.execute_behavior('grasp_object', obj)
        elif action_type == 'navigate':
            location = action.get('location', 'current')
            self.execute_behavior('navigate_to', location)
        elif action_type == 'speak':
            text = action.get('text', 'Hello')
            self.execute_behavior('speak', text)
        else:
            self.get_logger().error(f"Unknown action type: {action_type}")
    
    def execute_movement(self, direction, distance=1.0):
        """Execute movement commands."""
        twist_msg = Twist()
        
        if direction == "forward":
            twist_msg.linear.x = 0.2  # m/s
            self.get_logger().info(f"Moving forward for {distance} meters")
        elif direction == "backward":
            twist_msg.linear.x = -0.2
            self.get_logger().info(f"Moving backward for {distance} meters")
        elif direction == "left":
            twist_msg.angular.z = 0.5  # rad/s
            self.get_logger().info(f"Turning left for {distance} meters")
        elif direction == "right":
            twist_msg.angular.z = -0.5
            self.get_logger().info(f"Turning right for {distance} meters")
        elif direction == "stop":
            self.get_logger().info("Stopping movement")
        else:
            self.get_logger().warn(f"Unknown movement direction: {direction}")
            return
        
        # Publish the movement command
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Stop after the specified distance/time
        if direction in ["forward", "backward"]:
            time.sleep(distance / 0.2)  # Approximate time based on speed
        elif direction in ["left", "right"]:
            time.sleep(distance / 0.5)  # Approximate time based on angular velocity
        
        # Stop the robot after movement
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)
    
    def execute_behavior(self, behavior_type, param=None):
        """Execute complex behaviors."""
        action_msg = String()
        
        if behavior_type == "wave":
            action_msg.data = "wave"
            self.get_logger().info("Executing wave behavior")
        elif behavior_type == "dance":
            action_msg.data = "dance"
            self.get_logger().info("Executing dance behavior")
        elif behavior_type == "look_around":
            action_msg.data = "look_around"
            self.get_logger().info("Executing look around behavior")
        elif behavior_type == "grasp_object":
            action_msg.data = f"grasp_object:{param if param else 'object'}"
            self.get_logger().info(f"Executing grasp behavior for: {param}")
        elif behavior_type == "navigate_to":
            action_msg.data = f"navigate_to:{param if param else 'location'}"
            self.get_logger().info(f"Navigating to: {param}")
        elif behavior_type == "speak":
            action_msg.data = f"speak:{param if param else 'Hello'}"
            self.get_logger().info(f"Speaking: {param}")
        else:
            self.get_logger().warn(f"Unknown behavior type: {behavior_type}")
            return
        
        # Publish the behavior command
        self.action_publisher.publish(action_msg)


def main(args=None):
    """Main function to run the voice-to-action node."""
    rclpy.init(args=args)
    
    voice_to_action_node = VoiceToActionNode()
    
    try:
        rclpy.spin(voice_to_action_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_to_action_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()