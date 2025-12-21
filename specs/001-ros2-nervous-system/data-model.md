# Data Model: Module 1: The Robotic Nervous System (ROS 2)

**Date**: 2025-12-07
**Status**: Complete
**Purpose**: Define all data entities, message types, and communication contracts for ROS 2 robot control module examples

---

## ROS 2 Message Type Specifications

### Standard Message Types (from standard libraries)

All message types use ROS 2 built-in definitions. No custom messages required for this chapter.

#### std_msgs.Float64 (Sensor Data & Control Values)

```yaml
Entity: SensorData / ControlCommand
ROS Type: std_msgs/Float64
Purpose: Generic floating-point data transmission
Fields:
  - data: float64 (range: -∞ to +∞; interpretation context-dependent)
Example Uses:
  - Sensor reading (temperature, distance, angle)
  - Control command (motor speed, joint position)
  - AI agent decision output (threshold value)
Topic Examples:
  - /sensor_data (Publisher → Subscriber)
  - /robot_commands (AI Agent Publisher → Controller Subscriber)
```

#### std_msgs.String (Text Messages & Logging)

```yaml
Entity: LogMessage / ControlSignal
ROS Type: std_msgs/String
Purpose: Text-based communication
Fields:
  - data: string (UTF-8 encoded)
Example Uses:
  - Agent decision explanation ("MOVE_FORWARD", "STOP", "AVOID_OBSTACLE")
  - Debug messages
  - Command tokens
Topic Examples:
  - /robot_commands (alternative: String-based commands vs. numeric)
```

#### sensor_msgs.LaserScan (Range Sensor Data)

```yaml
Entity: SensorReading (Range/Distance)
ROS Type: sensor_msgs/LaserScan
Purpose: 2D range sensor data (e.g., lidar, sonar)
Fields:
  - header: std_msgs/Header (timestamp, frame_id)
  - angle_min: float32 (start angle of scan in radians)
  - angle_max: float32 (end angle of scan)
  - angle_increment: float32 (angular distance between measurements)
  - time_increment: float32 (time between measurements)
  - scan_time: float32 (time for complete scan)
  - range_min: float32 (minimum detectable range)
  - range_max: float32 (maximum detectable range)
  - ranges: float32[] (distance measurements)
  - intensities: float32[] (intensity/reflectivity (optional))
Example Uses:
  - Obstacle detection in humanoid navigation
  - Mock sensor publisher for testing
Topic Examples:
  - /scan (ROS 2 convention for lidar data)
  - /sensor_data_laser (alternative naming)
Validation Rules:
  - len(ranges) == (angle_max - angle_min) / angle_increment + 1
  - All range values must be >= 0 (or std::inf for no detection)
```

#### geometry_msgs.Twist (Robot Motion Commands)

```yaml
Entity: VelocityCommand
ROS Type: geometry_msgs/Twist
Purpose: 3D linear and angular velocity command for robots
Fields:
  - linear: geometry_msgs/Vector3 (3D linear velocity)
    - x: float64 (forward/backward speed in m/s)
    - y: float64 (left/right speed in m/s)
    - z: float64 (up/down speed in m/s)
  - angular: geometry_msgs/Vector3 (3D angular velocity)
    - x: float64 (roll rate in rad/s)
    - y: float64 (pitch rate in rad/s)
    - z: float64 (yaw rate in rad/s)
Example Uses:
  - Mobile robot velocity commands
  - Humanoid robot base motion
  - AI agent publishing motion commands
Topic Examples:
  - /cmd_vel (ROS 2 convention for velocity command)
  - /robot_motion (alternative naming)
Validation Rules:
  - Linear velocities bounded by robot max speed (typically 0–2 m/s)
  - Angular velocities bounded by robot max turn rate (typically 0–π rad/s)
```

---

## Custom Data Structures (Chapter-Specific)

No custom message definitions in this chapter. All communication uses standard ROS 2 message types for simplicity and focus on learning patterns.

---

## ROS 2 Service Definitions

### Example Service: AddTwoInts (Standard ROS 2 Service)

```yaml
Entity: MathService
Service Name: /add_two_ints
ROS Definition: example_interfaces/srv/AddTwoInts
Purpose: Demonstrate synchronous request/response communication
Request:
  - a: int64 (first number)
  - b: int64 (second number)
Response:
  - sum: int64 (result: a + b)
Example Usage:
  - Client sends request {a: 5, b: 3}
  - Server responds with {sum: 8}
  - Demonstrates Service Server/Client pattern
Validation Rules:
  - Both a and b must be valid int64 values
  - Server must respond within timeout (typically 5 seconds)
```

---

## URDF Data Structure (Humanoid Robot for Control)

### Robot: Humanoid Robot for Control Applications (Minimum 4 Joints)

```yaml
Robot Name: humanoid_robot
Total Links: Minimum 5 (base_link + 4+ body parts)
Total Joints: Minimum 4 (revolute joints with control interfaces)
Total DOF: Minimum 4
Structure Type: Kinematic tree (chain-based) suitable for control applications

Links (Rigid Bodies):
  - base_link: Origin reference frame; no physical body; connects to world
  - torso: Main body; mass ~10 kg; inertia matrix defined; center of robot
  - left_leg_upper: Left thigh; mass ~2 kg; control interface for hip actuator
  - left_leg_lower: Left calf; mass ~1.5 kg; control interface for knee actuator
  - right_leg_upper: Right thigh; mass ~2 kg; control interface for hip actuator
  - (Additional links as needed for complete humanoid: head, arms, etc.)

Joints (DOF with Control Interfaces):
  - base_to_torso: Fixed joint (0 DOF) — connects root frame to torso
  - torso_left_hip: Revolute; 1 DOF; range ±45°; effort limit 50 N⋅m; velocity limit 2 rad/s; controlled via JointState/trajectory_msgs
  - left_hip_left_knee: Revolute; 1 DOF; range ±90°; effort limit 40 N⋅m; velocity limit 2 rad/s; controlled via JointState/trajectory_msgs
  - torso_right_hip: Revolute; 1 DOF; range ±45°; effort limit 50 N⋅m; velocity limit 2 rad/s; controlled via JointState/trajectory_msgs
  - right_hip_right_knee: Revolute; 1 DOF; range ±90°; effort limit 40 N⋅m; velocity limit 2 rad/s; controlled via JointState/trajectory_msgs
  - (Additional joints for complete humanoid control)

Joint Control Interfaces:
  - Each joint defines position, velocity, and effort control capabilities
  - Joint limits defined for safe operation in simulation and real hardware
  - Control interfaces compatible with ros2_control framework
  - Gazebo plugin interfaces for physics simulation

Visual & Collision Properties:
  - Each link has visual geometry (for RViz display): cylinder or box meshes
  - Each link has collision geometry (for Gazebo physics): simplified shapes
  - Materials defined for color (red, green, blue for visual distinction)
  - Collision filtering ensures non-adjacent links don't collide
  - Collision properties tuned for stable humanoid simulation

Inertial Properties:
  - Each link has mass and inertia tensor
  - Inertia calculated assuming uniform density
  - Realistic values for humanoid robot dynamics
  - Sufficient for stable Gazebo simulation with physics

Coordinate Frames (TF):
  - All frames follow ROS 2 conventions (Z-up, X-forward, Y-left)
  - Parent frame specified for each joint to establish kinematic chain
  - Allows transformation trees for kinematic calculations and visualization
  - Base frame for robot localization and navigation

Validation:
  - All joint angles must fall within specified limits
  - Parent/child links must form connected tree (no cycles)
  - Inertia tensors must be positive definite
  - Joint control interfaces properly defined for ros2_control
  - URDF loads without errors in RViz, Gazebo, and urdf_parser
```

---

## Communication Topology

### ROS 2 Topic Graph

```
User Story 2 (Python Robot Control Node using rclpy):
  robot_control_node.py (publishes/subscribes)
    ├── /robot_status → rclpy.logging → console
    └── /robot_control_commands ← rclpy subscriber ← external commands

User Story 3 (Robot Control Topics & Services):
  sensor_publisher.py (publishes)
    └── /joint_states (sensor_msgs/JointState)
        ├── robot_controller.py (subscribes) → processes → publishes
        │   └── /joint_commands (std_msgs/Float64MultiArray)
        └── [other controllers can subscribe]

  robot_service_server.py (serves)
    ├── /robot_control_service (custom service)
    └── robot_service_client.py (calls service) → executes control action

User Story 4 (Python AI Agent bridged to ROS Controllers using rclpy):
  humanoid_sensor_publisher.py (publishes)
    └── /humanoid_sensor_data (sensor_msgs/JointState)
        └── ai_agent.py (subscribes via rclpy) → processes → publishes via rclpy
            └── /humanoid_control_commands (trajectory_msgs/JointTrajectory)
                └── ros2_control_controllers (subscribes) → actuates joints

User Story 5 (URDF):
  humanoid_robot.urdf (static description)
    └── Loaded by Gazebo or RViz
    └── Defines kinematic tree and control interfaces for User Story 6

User Story 6 (Complete Humanoid Robot Control Pipeline):
  gazebo (runs simulation)
    └── spawn_humanoid_robot (loads URDF with ros2_control interfaces)
        └── joint state broadcaster and position/effort controllers

  humanoid_sensor_publisher.py (publishes sensor data from simulated humanoid)
    └── /joint_states (sensor_msgs/JointState)

  ai_agent.py (subscribes to humanoid sensor data via rclpy, publishes control commands via rclpy)
    └── subscribes: /joint_states (sensor_msgs/JointState)
    └── publishes: /joint_trajectory (trajectory_msgs/JointTrajectory)

  ros2_control_controllers (subscribes to commands, updates simulation)
    └── subscribes: /joint_trajectory (trajectory_msgs/JointTrajectory)
    └── controls: joint_position_controllers → Gazebo → physical response
```

### Topic Specifications for Robot Control

| Topic Name | Message Type | Direction | Rate | QoS | Purpose |
|------------|--------------|-----------|------|-----|---------|
| `/joint_states` | sensor_msgs/JointState | Publish | 50 Hz | Reliable | Current joint positions, velocities, and efforts from humanoid robot |
| `/joint_commands` | std_msgs/Float64MultiArray | Publish | 50 Hz | Reliable | Joint position commands for humanoid robot actuators |
| `/joint_trajectory` | trajectory_msgs/JointTrajectory | Publish | 10 Hz | Reliable | Trajectory commands for smooth humanoid robot motion |
| `/humanoid_sensor_data` | sensor_msgs/JointState | Publish | 50 Hz | Reliable | Sensor readings from humanoid robot joints |
| `/humanoid_control_commands` | trajectory_msgs/JointTrajectory | Publish | 10 Hz | Reliable | Control commands from AI agent to humanoid robot |
| `/robot_status` | std_msgs/String | Publish | 1 Hz | Reliable | Robot operational status and health information |

### Service Specifications

| Service Name | Service Type | Request | Response | Purpose |
|--------------|--------------|---------|----------|---------|
| `/add_two_ints` | example_interfaces/srv/AddTwoInts | {a: int64, b: int64} | {sum: int64} | Demonstrate synchronous request/response |

---

## State Transitions & Lifecycle

### ROS 2 Node Lifecycle States (User Story 2)

```
Unconfigured
    ↓ (on_configure)
Inactive
    ↓ (on_activate)
Active
    ↓ (on_deactivate)
Inactive
    ↓ (on_cleanup)
Unconfigured
    ↓ (on_shutdown)
Finalized
```

**Application to Chapter Examples**:
- User Story 2: Demonstrate node creation and logging in Active state (simplified version without full lifecycle)
- User Story 3: Publishers/Subscribers operate in Active state
- User Story 4: AI agent spins in Active state; handles shutdown gracefully
- User Story 6: Launch file coordinates multiple nodes; startup order matters (sensor → agent → controller)

---

## Data Validation & Constraints

### Sensor Data Validation (User Story 4)

```yaml
SensorData (Float64):
  - Valid range: -∞ to +∞ (application-dependent)
  - Common range for mock data: 0–100 (simulates normalized sensor reading)
  - Validation rule: No special constraints; any float64 is valid
  - Agent logic: threshold check (if value > 50: move_forward; else: stop)
```

### URDF Joint Limits (User Story 5)

```yaml
Revolute Joints:
  - Lower limit: -π to 0 radians (typically)
  - Upper limit: 0 to π radians (typically)
  - Effort limit: 10 N⋅m (for Gazebo simulation; realistic for small humanoid)
  - Velocity limit: 2 rad/s (for Gazebo simulation; realistic for educational robot)

Example Limits:
  - Hip flexion: -45° to +45° (-0.785 to +0.785 radians)
  - Knee bend: 0° to +90° (0 to 1.571 radians)
  - Shoulder raise: -90° to +90° (-1.571 to +1.571 radians)
  - Elbow bend: 0° to +120° (0 to 2.094 radians)
```

### Message Rate Constraints for Robot Control

```yaml
Topic /joint_states:
  - Publishing rate: 50 Hz (0.02 second period) - real-time control requirement
  - Subscriber latency: typically <5 ms in ROS 2 for control applications
  - Total loop latency: <20 ms required for stable humanoid control (FR from User Story 4)

Topic /joint_commands:
  - Publishing rate: 50 Hz (0.02 second period) - real-time control requirement
  - Controller response latency: <10 ms expected for stable control
  - Message validation: Joint positions must be within URDF joint limits

Topic /joint_trajectory:
  - Publishing rate: 10 Hz (0.1 second period) - trajectory updates
  - Controller response latency: <50 ms expected for smooth motion execution
  - Message validation: Trajectory points must be kinematically feasible

Topic /humanoid_control_commands:
  - Publishing rate: 10-50 Hz depending on control type (trajectory vs. direct control)
  - AI agent processing latency: <50 ms required for responsive control
  - Message validation: Control commands must respect joint limits and dynamics
```

---

## Summary

This data model defines:

✅ **All message types** used in the module for robot control (sensor_msgs, std_msgs, trajectory_msgs)
✅ **Communication topology** showing topic connections between robot control nodes
✅ **URDF structure** for the humanoid robot with proper control interfaces
✅ **Service definitions** for synchronous robot control communication examples
✅ **Data validation rules** and constraints for real-time robot control
✅ **Lifecycle & state transitions** for ROS 2 robot control nodes
✅ **Message rates & latency** requirements for real-time humanoid robot control

All entities are mapped to user stories and requirements from spec.md. Standard ROS 2 messages are used for robot control applications, with emphasis on real-time performance and proper control interfaces compatible with ros2_control framework.
