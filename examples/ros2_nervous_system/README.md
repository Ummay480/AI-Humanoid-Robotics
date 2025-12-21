# ROS2 Nervous System Examples

This directory contains complete implementation examples for Module 1: The Robotic Nervous System (ROS 2), implementing all user stories from the specification.

## Components Overview

### 1. Basic Robot Control Node (`01_robot_control_node.py`)
- Implements User Story 2: Build ROS 2 Nodes for Robot Control using rclpy
- Demonstrates proper node initialization, lifecycle management, and real-time logging
- Runs at 50Hz control loop as specified

### 2. Publisher Example (`02_joint_state_publisher.py`)
- Implements User Story 3: Robot Control Topics and Services (Publisher)
- Publishes sensor data (joint states) at 50Hz with real-time performance
- Uses appropriate QoS settings for real-time robot control

### 3. Subscriber Example (`03_joint_command_subscriber.py`)
- Implements User Story 3: Robot Control Topics and Services (Subscriber)
- Subscribes to actuator commands and simulates response
- Validates real-time performance requirements (<20ms latency)

### 4. Service Server Example (`04_robot_control_service_server.py`)
- Implements User Story 3: Robot Control Topics and Services (Service Server)
- Provides emergency stop and robot state services
- Handles request/response communication with proper error handling

### 5. Service Client Example (`05_robot_control_service_client.py`)
- Implements User Story 3: Robot Control Topics and Services (Service Client)
- Calls robot control services with timing validation
- Demonstrates asynchronous service calls

### 6. AI Robot Agent (`06_ai_robot_agent.py`)
- Implements User Story 4: Bridge Python AI Agents to ROS 2 Robot Controllers
- Subscribes to sensor data and publishes control commands
- Demonstrates different robot behaviors with state management

### 7. Humanoid Robot URDF (`humanoid_robot.urdf`)
- Implements User Story 5: Understand URDF for Humanoid Robot Control
- Contains minimum 5 links and 4 joints as specified
- Includes ros2_control interfaces for Gazebo simulation

### 8. Launch File (`launch_pipeline.launch.py`)
- Implements User Story 6: Run Complete ROS 2 Humanoid Robot Control Pipeline
- Coordinates all components in a single launch file
- Ready for integration with Gazebo simulation

## How to Run

### Prerequisites
- ROS 2 Humble or Iron installed
- Python 3.10+
- rclpy package installed

### Running Individual Components

1. **Basic Robot Control Node:**
```bash
python3 01_robot_control_node.py
```

2. **Joint State Publisher:**
```bash
python3 02_joint_state_publisher.py
```

3. **Joint Command Subscriber:**
```bash
python3 03_joint_command_subscriber.py
```

4. **Service Server:**
```bash
python3 04_robot_control_service_server.py
```

5. **Service Client:**
```bash
python3 05_robot_control_service_client.py
```

6. **AI Robot Agent:**
```bash
python3 06_ai_robot_agent.py
```

### Running Complete Pipeline
The launch file can be used with ROS 2 launch system when integrated into a proper ROS 2 package:

```bash
ros2 launch your_robot_package launch_pipeline.launch.py
```

## Architecture Summary

The implementation follows the ROS 2 middleware architecture for robot control:

```
[Joint State Publisher] ----> [Robot Control Node] ----> [Joint Command Subscriber]
         |                                                    |
         |                                                    |
         +---- [AI Robot Agent] <-----------------------------+
         |
         +---- [Service Server/Client]
```

## Real-time Performance Validation

All components are designed to meet the real-time requirements specified:
- Control loops: 50Hz (20ms period)
- Sensor data publishing: 50Hz
- Service response times: <50ms
- System stability: Verified through timing analysis

## Integration with Gazebo

The URDF model is designed for integration with Gazebo simulation using ros2_control. The ros2_control interfaces in the URDF allow the Python nodes to control the simulated humanoid robot.

## Key Features Implemented

✅ **User Story 1**: Middleware architecture understanding through practical examples
✅ **User Story 2**: Node creation with proper initialization and lifecycle management
✅ **User Story 3**: Publisher/Subscriber and Service/Client implementations with real-time performance
✅ **User Story 4**: AI agent bridging to ROS controllers using rclpy
✅ **User Story 5**: Complete URDF with 5+ links, 4+ joints, and ros2_control interfaces
✅ **User Story 6**: Complete pipeline launch file for integrated operation

## Files Structure

```
examples/ros2_nervous_system/
├── 01_robot_control_node.py          # Basic control node
├── 02_joint_state_publisher.py       # Sensor data publisher
├── 03_joint_command_subscriber.py    # Actuator command subscriber
├── 04_robot_control_service_server.py # Service server
├── 05_robot_control_service_client.py # Service client
├── 06_ai_robot_agent.py              # AI agent bridging to controllers
├── humanoid_robot.urdf               # URDF with ros2_control interfaces
├── launch_pipeline.launch.py         # Complete pipeline launch file
└── README.md                         # This file
```

All code examples are executable in a standard ROS 2 environment without modification and meet the real-time performance requirements specified in the functional requirements.