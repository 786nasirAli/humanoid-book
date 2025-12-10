#!/usr/bin/env python3

"""
Simple URDF loader example.

This script demonstrates basic URDF loading concepts.
Note that in practice, URDF files are typically loaded by ROS 2 tools like RViz,
Gazebo, or MoveIt rather than directly in Python, but this shows the concept.
"""

import xml.etree.ElementTree as ET
import os


class URDFLoader:
    """Simple URDF loading utility to demonstrate the concept."""
    
    def __init__(self, urdf_path):
        """Initialize with path to URDF file."""
        self.urdf_path = urdf_path
        self.tree = None
        self.root = None
        self.links = []
        self.joints = []
        
        self.load_urdf()
    
    def load_urdf(self):
        """Load and parse the URDF file."""
        try:
            self.tree = ET.parse(self.urdf_path)
            self.root = self.tree.getroot()
            
            # Extract links and joints
            self.links = self.root.findall('link')
            self.joints = self.root.findall('joint')
            
            print(f"Loaded URDF: {self.root.get('name')}")
            print(f"Found {len(self.links)} links and {len(self.joints)} joints")
            
        except ET.ParseError as e:
            print(f"Error parsing URDF file: {e}")
        except FileNotFoundError:
            print(f"URDF file not found: {self.urdf_path}")
    
    def list_links(self):
        """Print all links in the URDF."""
        print("\nLinks in the URDF:")
        for link in self.links:
            name = link.get('name')
            visual = link.find('visual')
            collision = link.find('collision')
            inertial = link.find('inertial')
            
            print(f"  - {name}")
            if visual is not None:
                geom = visual.find('geometry')
                if geom is not None:
                    geom_type = geom[0].tag
                    print(f"    Visual: {geom_type}")
            if collision is not None:
                geom = collision.find('geometry')
                if geom is not None:
                    geom_type = geom[0].tag
                    print(f"    Collision: {geom_type}")
    
    def list_joints(self):
        """Print all joints in the URDF."""
        print("\nJoints in the URDF:")
        for joint in self.joints:
            name = joint.get('name')
            joint_type = joint.get('type')
            parent = joint.find('parent')
            child = joint.find('child')
            
            parent_name = parent.get('link') if parent is not None else 'Unknown'
            child_name = child.get('link') if child is not None else 'Unknown'
            
            print(f"  - {name}: {parent_name} -> {child_name} ({joint_type})")
    
    def find_joint_by_name(self, name):
        """Find a specific joint by name."""
        for joint in self.joints:
            if joint.get('name') == name:
                return joint
        return None


def main():
    """Main function to demonstrate URDF loading."""
    # Use the basic humanoid URDF from our examples
    urdf_path = 'content/examples/urdf/basic_humanoid.urdf'
    
    if not os.path.exists(urdf_path):
        print(f"URDF file not found: {urdf_path}")
        return
    
    # Create the URDF loader
    urdf_loader = URDFLoader(urdf_path)
    
    # Show the contents
    urdf_loader.list_links()
    urdf_loader.list_joints()
    
    # Example: Find a specific joint
    shoulder_joint = urdf_loader.find_joint_by_name('left_shoulder_joint')
    if shoulder_joint is not None:
        print(f"\nDetails for left_shoulder_joint:")
        joint_type = shoulder_joint.get('type')
        print(f"  Type: {joint_type}")
        
        # Find limits if they exist
        limit = shoulder_joint.find('limit')
        if limit is not None:
            lower = limit.get('lower', 'N/A')
            upper = limit.get('upper', 'N/A')
            print(f"  Limits: {lower} to {upper}")


if __name__ == '__main__':
    main()