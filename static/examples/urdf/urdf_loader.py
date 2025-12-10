#!/usr/bin/env python3

"""
URDF loader example for humanoid robot.

This script demonstrates how to load, parse, and work with URDF files
for humanoid robots in a Python environment. It includes functionality
for parsing URDF XML, extracting joint information, and validating
the robot structure.
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Dict, Optional


@dataclass
class Joint:
    """Represents a joint in the URDF model."""
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    axis: Optional[List[float]] = None
    origin_xyz: Optional[List[float]] = None
    origin_rpy: Optional[List[float]] = None
    limit_lower: Optional[float] = None
    limit_upper: Optional[float] = None
    limit_effort: Optional[float] = None
    limit_velocity: Optional[float] = None


@dataclass
class Link:
    """Represents a link in the URDF model."""
    name: str
    visual_geometry: Optional[Dict] = None
    collision_geometry: Optional[Dict] = None
    inertial_mass: Optional[float] = None
    inertial_origin_xyz: Optional[List[float]] = None
    inertial_origin_rpy: Optional[List[float]] = None
    inertial_ixx: Optional[float] = None
    inertial_ixy: Optional[float] = None
    inertial_ixz: Optional[float] = None
    inertial_iyy: Optional[float] = None
    inertial_iyz: Optional[float] = None
    inertial_izz: Optional[float] = None


class URDFLoader:
    """A simple URDF loader for parsing and working with URDF files."""
    
    def __init__(self):
        self.links: Dict[str, Link] = {}
        self.joints: Dict[str, Joint] = {}
        self.robot_name: str = ""
    
    def load_urdf_from_file(self, file_path: str) -> bool:
        """Load a URDF file and parse its contents."""
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            return self._parse_robot_element(root)
        except ET.ParseError as e:
            print(f"Error parsing URDF file: {e}")
            return False
        except FileNotFoundError:
            print(f"URDF file not found: {file_path}")
            return False
    
    def load_urdf_from_string(self, urdf_string: str) -> bool:
        """Load a URDF from a string."""
        try:
            root = ET.fromstring(urdf_string)
            return self._parse_robot_element(root)
        except ET.ParseError as e:
            print(f"Error parsing URDF string: {e}")
            return False
    
    def _parse_robot_element(self, root: ET.Element) -> bool:
        """Parse the root robot element of the URDF."""
        if root.tag != 'robot':
            print("Invalid URDF: Root element is not 'robot'")
            return False
        
        self.robot_name = root.get('name', 'unknown_robot')
        print(f"Loading robot: {self.robot_name}")
        
        # Parse all links
        for link_elem in root.findall('link'):
            link = self._parse_link(link_elem)
            if link:
                self.links[link.name] = link
        
        # Parse all joints
        for joint_elem in root.findall('joint'):
            joint = self._parse_joint(joint_elem)
            if joint:
                self.joints[joint.name] = joint
        
        return True
    
    def _parse_link(self, link_elem: ET.Element) -> Optional[Link]:
        """Parse a link element from the URDF."""
        name = link_elem.get('name')
        if not name:
            print("Link element missing name attribute")
            return None
        
        link = Link(name=name)
        
        # Parse visual
        visual_elem = link_elem.find('visual')
        if visual_elem is not None:
            link.visual_geometry = self._parse_geometry(visual_elem.find('geometry'))
        
        # Parse collision
        collision_elem = link_elem.find('collision')
        if collision_elem is not None:
            link.collision_geometry = self._parse_geometry(collision_elem.find('geometry'))
        
        # Parse inertial
        inertial_elem = link_elem.find('inertial')
        if inertial_elem is not None:
            link.inertial_mass = float(inertial_elem.find('mass').get('value', 0.0))
            
            # Parse origin
            origin_elem = inertial_elem.find('origin')
            if origin_elem is not None:
                xyz_str = origin_elem.get('xyz', '0 0 0')
                rpy_str = origin_elem.get('rpy', '0 0 0')
                link.inertial_origin_xyz = [float(x) for x in xyz_str.split()]
                link.inertial_origin_rpy = [float(x) for x in rpy_str.split()]
            
            # Parse inertia
            inertia_elem = inertial_elem.find('inertia')
            if inertia_elem is not None:
                link.inertial_ixx = float(inertia_elem.get('ixx', 0.0))
                link.inertial_ixy = float(inertia_elem.get('ixy', 0.0))
                link.inertial_ixz = float(inertia_elem.get('ixz', 0.0))
                link.inertial_iyy = float(inertia_elem.get('iyy', 0.0))
                link.inertial_iyz = float(inertia_elem.get('iyz', 0.0))
                link.inertial_izz = float(inertia_elem.get('izz', 0.0))
        
        return link
    
    def _parse_joint(self, joint_elem: ET.Element) -> Optional[Joint]:
        """Parse a joint element from the URDF."""
        name = joint_elem.get('name')
        joint_type = joint_elem.get('type')
        
        if not name or not joint_type:
            print("Joint element missing name or type attribute")
            return None
        
        joint = Joint(name=name, joint_type=joint_type)
        
        # Parse parent and child
        parent_elem = joint_elem.find('parent')
        child_elem = joint_elem.find('child')
        
        if parent_elem is not None:
            joint.parent_link = parent_elem.get('link', '')
        if child_elem is not None:
            joint.child_link = child_elem.get('link', '')
        
        # Parse origin
        origin_elem = joint_elem.find('origin')
        if origin_elem is not None:
            xyz_str = origin_elem.get('xyz', '0 0 0')
            rpy_str = origin_elem.get('rpy', '0 0 0')
            joint.origin_xyz = [float(x) for x in xyz_str.split()]
            joint.origin_rpy = [float(x) for x in rpy_str.split()]
        
        # Parse axis
        axis_elem = joint_elem.find('axis')
        if axis_elem is not None:
            xyz_str = axis_elem.get('xyz', '1 0 0')
            joint.axis = [float(x) for x in xyz_str.split()]
        
        # Parse limits
        limit_elem = joint_elem.find('limit')
        if limit_elem is not None:
            joint.limit_lower = float(limit_elem.get('lower', 0.0))
            joint.limit_upper = float(limit_elem.get('upper', 0.0))
            joint.limit_effort = float(limit_elem.get('effort', 0.0))
            joint.limit_velocity = float(limit_elem.get('velocity', 0.0))
        
        return joint
    
    def _parse_geometry(self, geometry_elem: ET.Element) -> Optional[Dict]:
        """Parse a geometry element."""
        if geometry_elem is None:
            return None
        
        geometry = {}
        
        # Check for different geometry types
        box_elem = geometry_elem.find('box')
        if box_elem is not None:
            size_str = box_elem.get('size', '0 0 0')
            geometry['type'] = 'box'
            geometry['size'] = [float(x) for x in size_str.split()]
            return geometry
        
        cylinder_elem = geometry_elem.find('cylinder')
        if cylinder_elem is not None:
            geometry['type'] = 'cylinder'
            geometry['radius'] = float(cylinder_elem.get('radius', 0.0))
            geometry['length'] = float(cylinder_elem.get('length', 0.0))
            return geometry
        
        sphere_elem = geometry_elem.find('sphere')
        if sphere_elem is not None:
            geometry['type'] = 'sphere'
            geometry['radius'] = float(sphere_elem.get('radius', 0.0))
            return geometry
        
        mesh_elem = geometry_elem.find('mesh')
        if mesh_elem is not None:
            geometry['type'] = 'mesh'
            geometry['filename'] = mesh_elem.get('filename', '')
            scale_str = mesh_elem.get('scale', '1 1 1')
            geometry['scale'] = [float(x) for x in scale_str.split()]
            return geometry
        
        return geometry
    
    def print_robot_structure(self):
        """Print a summary of the loaded robot structure."""
        print(f"\nRobot: {self.robot_name}")
        print(f"Links: {len(self.links)}")
        print(f"Joints: {len(self.joints)}")
        
        print("\nLinks:")
        for link_name, link in self.links.items():
            print(f"  - {link_name}")
            if link.visual_geometry:
                geom_type = link.visual_geometry.get('type', 'unknown')
                print(f"    Visual: {geom_type}")
            if link.collision_geometry:
                geom_type = link.collision_geometry.get('type', 'unknown')
                print(f"    Collision: {geom_type}")
        
        print("\nJoints:")
        for joint_name, joint in self.joints.items():
            print(f"  - {joint_name}: {joint.joint_type} ({joint.parent_link} -> {joint.child_link})")
            if joint.limit_lower is not None and joint.limit_upper is not None:
                print(f"    Limits: [{joint.limit_lower:.2f}, {joint.limit_upper:.2f}]")


def main():
    """Main function to demonstrate URDF loading."""
    print("URDF Loader Example")
    print("=" * 30)
    
    # Create URDF loader instance
    loader = URDFLoader()
    
    # Load the basic humanoid URDF file
    success = loader.load_urdf_from_file('basic_humanoid.urdf')
    
    if success:
        print("URDF loaded successfully!")
        loader.print_robot_structure()
    else:
        print("Failed to load URDF from file.")
        # Try loading a sample URDF as a string
        sample_urdf = '''
        <robot name="simple_robot">
          <link name="base_link">
            <visual>
              <geometry>
                <box size="0.1 0.1 0.1"/>
              </geometry>
            </visual>
            <collision>
              <geometry>
                <box size="0.1 0.1 0.1"/>
              </geometry>
            </collision>
            <inertial>
              <mass value="1.0"/>
              <origin xyz="0 0 0"/>
              <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
            </inertial>
          </link>
          
          <joint name="base_to_arm" type="revolute">
            <parent link="base_link"/>
            <child link="arm_link"/>
            <origin xyz="0.1 0 0"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
          </joint>
          
          <link name="arm_link">
            <visual>
              <geometry>
                <cylinder radius="0.02" length="0.2"/>
              </geometry>
            </visual>
            <collision>
              <geometry>
                <cylinder radius="0.02" length="0.2"/>
              </geometry>
            </collision>
            <inertial>
              <mass value="0.1"/>
              <origin xyz="0 0 0.1"/>
              <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
            </inertial>
          </link>
        </robot>
        '''
        
        if loader.load_urdf_from_string(sample_urdf):
            print("\nSample URDF loaded successfully!")
            loader.print_robot_structure()
        else:
            print("Failed to load sample URDF.")


if __name__ == '__main__':
    main()