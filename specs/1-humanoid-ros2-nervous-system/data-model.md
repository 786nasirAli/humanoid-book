# Data Model: The Robotic Nervous System

## Core Entities

### ROS 2 Node
**Description**: A process that performs computation in the ROS 2 system, representing a specific function within the humanoid robot (e.g., sensor interface, controller, decision-making module)

**Fields**:
- name: string (unique identifier for the node)
- namespace: string (optional, for organizing nodes)
- state: enum (active, inactive, error)
- publishers: list of Publisher objects
- subscribers: list of Subscriber objects
- services: list of Service objects

**Relationships**:
- Contains multiple Publisher objects
- Contains multiple Subscriber objects
- Contains multiple Service objects

**Validation Rules**:
- Node name must be unique within a namespace
- Node must have at least one communication interface (publisher, subscriber, or service)

### Publisher
**Description**: A communication endpoint that sends messages on a specific topic

**Fields**:
- topic_name: string (name of the topic to publish to)
- message_type: string (type of message being published)
- frequency: float (rate of message publishing in Hz)

**Relationships**:
- Belongs to one Node
- Connects to multiple Subscriber objects via Topic

**Validation Rules**:
- Topic name must follow ROS 2 naming conventions
- Message type must be a valid ROS 2 message type

### Subscriber
**Description**: A communication endpoint that receives messages from a specific topic

**Fields**:
- topic_name: string (name of the topic to subscribe to)
- message_type: string (type of message expected)
- callback_function: string (function to handle incoming messages)

**Relationships**:
- Belongs to one Node
- Connects to one Publisher object via Topic

**Validation Rules**:
- Topic name must follow ROS 2 naming conventions
- Message type must match the publisher's message type for successful communication

### Topic
**Description**: A communication channel for passing messages between nodes, enabling one-way data transmission (e.g., sensor data, motor commands)

**Fields**:
- name: string (unique identifier for the topic)
- message_type: string (type of message that flows through this topic)
- qos_profile: object (Quality of Service settings)

**Relationships**:
- Connects multiple Publisher objects to multiple Subscriber objects

**Validation Rules**:
- Topic name must follow ROS 2 naming conventions
- Message type must be consistent across all publishers and subscribers on the topic

### Service
**Description**: A request-reply communication pattern for performing specific tasks (e.g., requesting robot status, configuring parameters)

**Fields**:
- name: string (unique identifier for the service)
- service_type: string (type definition for the service)
- request_type: string (type of request message)
- response_type: string (type of response message)

**Relationships**:
- Belongs to one Node (service server)
- Can be called by multiple Node objects (service clients)

**Validation Rules**:
- Service name must follow ROS 2 naming conventions
- Request and response types must be valid ROS 2 message types

### URDF (Unified Robot Description Format)
**Description**: An XML-based format for representing robot structure including links, joints, and inertial properties specific to the humanoid robot

**Fields**:
- links: list of Link objects
- joints: list of Joint objects
- materials: list of Material objects
- gazebo_extensions: list of Gazebo-specific configurations (optional)

**Relationships**:
- Contains multiple Link objects
- Contains multiple Joint objects
- Contains multiple Material objects

**Validation Rules**:
- Must have at least one link
- Joint parent and child must reference valid links
- All references in URDF must be valid

### Link
**Description**: A rigid component of the robot structure (e.g., torso, upper arm, lower leg)

**Fields**:
- name: string (unique identifier for the link)
- visual: object (visual representation properties)
- collision: object (collision detection properties)
- inertial: object (mass, center of mass, and inertia properties)

**Relationships**:
- Belongs to one URDF object
- Connected to other Link objects via Joint objects

**Validation Rules**:
- Link name must be unique within the URDF
- Inertial properties must be physically valid

### Joint
**Description**: A connection between two links that constrains their relative movement

**Fields**:
- name: string (unique identifier for the joint)
- type: enum (revolute, continuous, prismatic, fixed, floating, planar)
- parent: string (name of parent link)
- child: string (name of child link)
- origin: object (position and orientation offset from parent)

**Relationships**:
- Connects two Link objects
- Belongs to one URDF object

**Validation Rules**:
- Joint name must be unique within the URDF
- Parent and child links must exist in the URDF
- Joint type must be valid for the intended movement

### Python Client Library
**Description**: A library that enables Python-based agents to interact with the robotics middleware

**Fields**:
- name: string (e.g., "rclpy")
- version: string (version requirement)
- node_interface: object (functionality for creating and managing nodes)
- communication_interfaces: object (functionality for publishers, subscribers, services)

**Relationships**:
- Used by multiple Node objects
- Provides communication capabilities for the system

**Validation Rules**:
- Version must be compatible with the selected ROS 2 distribution
- Required functionality must be available in the specified version

### Humanoid Controller
**Description**: A specialized ROS 2 node that processes commands and manages the physical behavior of the humanoid robot's actuators and joints

**Fields**:
- name: string (unique identifier)
- controlled_joints: list of joint names
- command_interfaces: list of available command types
- feedback_interfaces: list of available feedback types

**Relationships**:
- Inherits from Node entity
- Interfaces with Joint objects in URDF
- Subscribes to command topics
- Publishes feedback topics

**Validation Rules**:
- Controlled joints must exist in the associated URDF
- Command interfaces must match joint capabilities