# Data Model: Chapter 1 — The Robotic Nervous System (ROS 2)

**Date**: 2025-12-07
**Status**: Complete
**Purpose**: Define all data entities, message types, and communication contracts for ROS 2 chapter examples

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

## URDF Data Structure (Humanoid Robot)

### Robot: Simple Biped Humanoid (7 DOF)

```yaml
Robot Name: simple_humanoid
Total Links: 8 (base_link + 7 body parts)
Total Joints: 7 (revolute joints connecting links)
Total DOF: 7
Structure Type: Kinematic tree (chain-based)

Links (Rigid Bodies):
  - base_link: Origin reference frame; no physical body
  - torso: Main body; mass ~10 kg; inertia matrix defined
  - left_leg_upper: Left thigh; mass ~2 kg
  - left_leg_lower: Left calf; mass ~1.5 kg
  - right_leg_upper: Right thigh; mass ~2 kg
  - right_leg_lower: Right calf; mass ~1.5 kg
  - right_arm_upper: Right upper arm; mass ~1 kg
  - right_arm_lower: Right forearm; mass ~0.5 kg

Joints (DOF):
  - base_to_torso: Fixed joint (0 DOF) — connects root frame to torso
  - torso_left_hip: Revolute; 1 DOF; range ±45°; simulates left hip flexion
  - left_hip_left_knee: Revolute; 1 DOF; range ±90°; simulates left knee bend
  - torso_right_hip: Revolute; 1 DOF; range ±45°; simulates right hip flexion
  - right_hip_right_knee: Revolute; 1 DOF; range ±90°; simulates right knee bend
  - torso_right_shoulder: Revolute; 1 DOF; range ±90°; simulates arm raise
  - right_shoulder_right_elbow: Revolute; 1 DOF; range ±120°; simulates arm bend

Visual & Collision Properties:
  - Each link has visual geometry (for RViz display): cylinder or box meshes
  - Each link has collision geometry (for Gazebo physics): simplified shapes
  - Materials defined for color (red, green, blue for visual distinction)
  - Collision filtering ensures non-adjacent links don't collide

Inertial Properties:
  - Each link has mass and inertia tensor
  - Inertia calculated assuming uniform density
  - Realistic but simplified for educational purposes
  - Sufficient for stable Gazebo simulation

Coordinate Frames (TF):
  - All frames follow ROS 2 conventions (Z-up, X-forward, Y-left)
  - Parent frame specified for each joint
  - Allows transformation trees (e.g., compute end-effector position via kinematic chain)

Validation:
  - All joint angles must fall within specified limits
  - Parent/child links must form connected tree (no cycles)
  - Inertia tensors must be positive definite
```

---

## Communication Topology

### ROS 2 Topic Graph

```
User Story 2 (Python Node):
  simple_node.py (publishes)
    └── /node_info → rclpy.logging → console

User Story 3 (Topics & Services):
  publisher.py (publishes)
    └── /sensor_data (std_msgs/Float64)
        ├── subscriber.py (subscribes) → prints data
        └── [other nodes can subscribe]

  service_server.py (serves)
    ├── /add_two_ints service
    └── service_client.py (calls service) → prints result

User Story 4 (AI Agent):
  mock_sensor_publisher.py (publishes)
    └── /sensor_data (std_msgs/Float64)
        └── ai_agent.py (subscribes) → processes → publishes
            └── /robot_commands (std_msgs/Float64)
                └── controller.py (subscribes) → simulates response

User Story 5 (URDF):
  humanoid_robot.urdf (static description)
    └── Loaded by Gazebo or RViz
    └── Defines kinematic tree for User Story 6

User Story 6 (Complete Pipeline):
  gazebo (runs simulation)
    └── spawn_humanoid_robot (loads URDF)
        └── joint state publisher

  mock_sensor_publisher.py (publishes sensor data)
    └── /sensor_data

  ai_agent.py (subscribes to sensor, publishes commands)
    └── subscribes: /sensor_data
    └── publishes: /robot_commands

  gazebo_controller.py (subscribes to commands, updates simulation)
    └── subscribes: /robot_commands
    └── publishes: /joint_states → Gazebo → visual update
```

### Topic Specifications

| Topic Name | Message Type | Direction | Rate | QoS | Purpose |
|------------|--------------|-----------|------|-----|---------|
| `/sensor_data` | std_msgs/Float64 | Publish | 10 Hz | Reliable | Sensor readings from mock publisher or robot |
| `/robot_commands` | std_msgs/Float64 or geometry_msgs/Twist | Publish | 5 Hz | Reliable | Control commands from AI agent |
| `/scan` | sensor_msgs/LaserScan | Publish | 5 Hz | Best-effort | Lidar data (if implemented in User Story 6) |
| `/cmd_vel` | geometry_msgs/Twist | Publish | 5 Hz | Reliable | Velocity command for humanoid base motion |
| `/joint_states` | sensor_msgs/JointState | Publish | 20 Hz | Best-effort | Current joint positions from Gazebo |

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

### Message Rate Constraints

```yaml
Topic /sensor_data:
  - Publishing rate: 10 Hz (0.1 second period)
  - Subscriber latency: typically <10 ms in ROS 2
  - Total loop latency: <100 ms required (FR from User Story 4)

Topic /robot_commands:
  - Publishing rate: 5 Hz (0.2 second period)
  - Controller response latency: <50 ms expected
```

---

## Summary

This data model defines:

✅ **All message types** used in the chapter (std_msgs, sensor_msgs, geometry_msgs)
✅ **Communication topology** showing topic connections between nodes
✅ **URDF structure** for the humanoid robot (7 DOF biped)
✅ **Service definitions** for synchronous communication examples
✅ **Data validation rules** and constraints
✅ **Lifecycle & state transitions** for ROS 2 nodes
✅ **Message rates & latency** requirements from the specification

All entities are mapped to user stories and requirements from spec.md. No custom message definitions are required; standard ROS 2 messages keep complexity low and focus on learning patterns.
