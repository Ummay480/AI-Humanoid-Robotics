# Topic Contracts: Chapter 1 — The Robotic Nervous System (ROS 2)

**Date**: 2025-12-07
**Status**: Complete
**Purpose**: Define all ROS 2 Topic contracts (pub/sub interfaces) for chapter examples

---

## Topic Contract Pattern

Each topic specifies:
- **Name**: ROS convention topic name (e.g., `/sensor_data`)
- **Message Type**: Full type (e.g., `std_msgs/Float64`)
- **Direction**: Publish (P) or Subscribe (S)
- **Publisher(s)**: Which node(s) publish
- **Subscriber(s)**: Which node(s) subscribe
- **Rate**: Publishing frequency (Hz)
- **QoS**: Quality of Service profile
- **Validation**: Data type constraints
- **User Story**: Which story exercises this contract

---

## Topic Contract 1: Sensor Data (Generic Float64)

```yaml
Name: /sensor_data
Message Type: std_msgs/Float64
Purpose: Generic sensor value transmission (temperature, distance, normalized reading)

Direction: Publish
Publishers:
  - mock_sensor_publisher.py (User Story 4)
  - [gazebo] (User Story 6, if simulated sensor attached)
Subscribers:
  - subscriber.py (User Story 3, demonstrates subscription)
  - ai_agent.py (User Story 4, processes sensor input)
  - gazebo_controller.py (User Story 6, optional)

Publishing Rate: 10 Hz (period: 0.1 seconds)
QoS Profile: Reliable (RMW_QOS_RELIABILITY_RELIABLE)
  - Ensures no message loss
  - Suitable for control feedback loops
  - Default for educational examples

Latency SLA:
  - Message delivery: <10 ms (local ROS 2 domain)
  - Subscriber callback: <100 ms (enforced in User Story 4)

Data Validation:
  - Field: data (float64)
  - Range: -∞ to +∞ (application-dependent)
  - Mock data range: 0–100 (normalized sensor reading)
  - Invalid: NaN, Inf (should be avoided; controller should handle gracefully)

Usage Examples:
  ```python
  # Publisher (User Story 3, example: 02_publisher.py)
  from std_msgs.msg import Float64
  msg = Float64()
  msg.data = 42.5
  publisher.publish(msg)

  # Subscriber (User Story 3, example: 03_subscriber.py)
  def callback(msg):
      print(f"Sensor reading: {msg.data}")
  subscription = node.create_subscription(Float64, '/sensor_data', callback, 10)

  # Agent (User Story 4, example: 06_ai_agent.py)
  def sensor_callback(msg):
      if msg.data > 50:
          publish_command("MOVE_FORWARD")
      else:
          publish_command("STOP")
  ```

Mapping to User Stories:
  - US1: Mentioned in architecture section as example topic
  - US2: Implicitly created by node examples
  - US3: Explicitly created in Publisher/Subscriber examples
  - US4: Core sensor input for AI agent logic
  - US6: Links mock sensor publisher to agent to controller

```

---

## Topic Contract 2: Robot Commands (Control Signal)

```yaml
Name: /robot_commands
Message Type: std_msgs/Float64 or std_msgs/String
Purpose: Actuator control commands from AI agent to robot controller

Direction: Publish
Publishers:
  - ai_agent.py (User Story 4)
  - gazebo_controller.py (User Story 6, alternative: subscriber to /sensor_data and publisher to joint commands)
Subscribers:
  - gazebo_controller.py (User Story 6)
  - [Gazebo] (User Story 6, if properly configured)

Publishing Rate: 5 Hz (period: 0.2 seconds)
QoS Profile: Reliable (RMW_QOS_RELIABILITY_RELIABLE)
  - Ensures control commands are not lost
  - More aggressive than sensor data (control is critical)

Latency SLA:
  - Message delivery: <10 ms
  - Controller response: <50 ms (robot should respond within this time)

Data Validation:
  - Field: data (float64) or data (string)
  - Float64 range: -1.0 to +1.0 (normalized control signal)
    - 0 = neutral/stop
    - >0 = forward/increase
    - <0 = backward/decrease
  - String options (if using std_msgs/String):
    - "MOVE_FORWARD"
    - "MOVE_BACKWARD"
    - "STOP"
    - "TURN_LEFT"
    - "TURN_RIGHT"

Usage Examples:
  ```python
  # AI Agent Publisher (User Story 4, example: 06_ai_agent.py)
  from std_msgs.msg import Float64
  def sensor_callback(msg):
      if msg.data > 50:  # Threshold logic
          command = Float64()
          command.data = 1.0  # Full forward
          command_pub.publish(command)
      else:
          command = Float64()
          command.data = 0.0  # Stop
          command_pub.publish(command)

  # Controller Subscriber (User Story 6, example: gazebo_controller.py)
  def command_callback(msg):
      joint_effort = msg.data * 10.0  # Scale to motor effort (Nm)
      gazebo_interface.set_joint_effort(joint_effort)
  subscription = node.create_subscription(Float64, '/robot_commands', command_callback, 10)
  ```

Mapping to User Stories:
  - US1: Mentioned in architecture as example of pub/sub
  - US3: Implicitly created when demonstrating Topics
  - US4: Core output of AI agent logic; demonstrates agent → controller interaction
  - US6: Links agent output to gazebo simulation

```

---

## Topic Contract 3: Joint States (Robot Status)

```yaml
Name: /joint_states
Message Type: sensor_msgs/JointState
Purpose: Current joint positions and velocities from robot/simulator

Direction: Publish
Publishers:
  - [Gazebo] (User Story 6)
  - joint_state_publisher (User Story 6, from URDF)
Subscribers:
  - RViz (User Story 5–6, visualization)
  - [optional controllers for feedback]

Publishing Rate: 20 Hz (period: 0.05 seconds)
QoS Profile: Best-Effort (RMW_QOS_RELIABILITY_BEST_EFFORT)
  - High-frequency telemetry; occasional loss acceptable
  - Visualization doesn't require perfect reliability

Latency SLA:
  - Message delivery: <5 ms expected
  - Not critical for control; informational only

Data Structure:
  ```yaml
  Header:
    stamp: ROS Time (timestamp when measurements taken)
    frame_id: "base_link" (reference frame)
  name: ["torso_left_hip", "left_hip_left_knee", ...]  # Joint names from URDF
  position: [0.5, 1.2, ...]  # Joint angles in radians
  velocity: [0.1, 0.2, ...]  # Joint angular velocities in rad/s
  effort: [5.0, 3.2, ...]    # Joint torques in N⋅m
  ```

Validation:
  - All arrays (name, position, velocity, effort) must have same length
  - Position values must be within joint limits (from URDF)
  - Velocity values should be continuous (no sudden jumps)

Usage Example:
  ```python
  # RViz automatically visualizes joint states
  # No explicit subscriber needed in User Story 5-6 examples
  # Controllers could optionally subscribe for feedback control

  def joint_state_callback(msg):
      for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
          print(f"Joint {name}: angle={pos:.2f} rad, velocity={vel:.2f} rad/s")
  subscription = node.create_subscription(
      JointState, '/joint_states', joint_state_callback, 10
  )
  ```

Mapping to User Stories:
  - US5: RViz visualizes URDF by subscribing to /joint_states
  - US6: Gazebo publishes joint states; enables animation in simulation

```

---

## Topic Contract 4: LaserScan (Optional Range Sensor Data)

```yaml
Name: /scan
Message Type: sensor_msgs/LaserScan
Purpose: 2D range measurements from lidar/sonar (optional advanced topic)

Direction: Publish
Publishers:
  - [Gazebo] (if lidar sensor configured)
  - mock_lidar_publisher.py (optional, not in core examples)
Subscribers:
  - RViz (visualization)
  - [obstacle avoidance controller]

Publishing Rate: 5 Hz (period: 0.2 seconds)
QoS Profile: Best-Effort (RMW_QOS_RELIABILITY_BEST_EFFORT)

Data Structure:
  ```yaml
  Header:
    stamp: ROS Time
    frame_id: "lidar_frame"
  angle_min: -1.57 (radians, -90°)
  angle_max: 1.57 (radians, +90°)
  angle_increment: 0.02 (radians)
  time_increment: 0.0001 (seconds between measurements)
  scan_time: 0.2 (seconds for full scan)
  range_min: 0.1 (meters)
  range_max: 10.0 (meters)
  ranges: [3.5, 3.4, 3.3, ..., 3.6]  # 160 distance readings
  intensities: [100, 102, 98, ..., 101]  # Reflectivity (optional)
  ```

Validation:
  - len(ranges) == (angle_max - angle_min) / angle_increment + 1
  - All range values: 0 ≤ range ≤ range_max (or inf for no detection)
  - Valid readings should be within [range_min, range_max]

Mapping to User Stories:
  - US1: Mentioned as example sensor data
  - US5-6: Optional; demonstrates RViz visualization of sensor data (not required for core chapter)

Note: Not required for User Story 6 MVP. Included for completeness if readers want to extend examples with lidar-based obstacle avoidance.

```

---

## Topic Interaction Diagram

```
User Story 3: Topics & Services
┌─────────────┐
│ publisher   │ ──publishes──> [/sensor_data: Float64]
└─────────────┘
                                      │
                                      ├──> subscriber (prints)
                                      └──> ai_agent (processes)

User Story 4: AI Agent Bridge
┌──────────────────┐
│ mock_sensor_pub  │ ──> [/sensor_data] ──> ai_agent ──> [/robot_commands] ──> controller
└──────────────────┘

User Story 6: Complete Pipeline
┌──────────────┐
│ Gazebo       │ ──spawns──> humanoid_robot (URDF)
└──────────────┘               │
                               ├──> /joint_states ──> RViz (visualization)
                               │                    ──> [feedback control]
                               │
                               └──> /robot_commands ──> joint_command_controller

┌──────────────────┐
│ mock_sensor_pub  │ ──> /sensor_data ──> ai_agent ──> /robot_commands ──> [above]
└──────────────────┘
```

---

## Summary

All topic contracts are:

✅ **Mapped to User Stories** — Every topic is used in at least one story
✅ **Typed with standard ROS 2 messages** — No custom messages required
✅ **Validated** — Data constraints and ranges specified
✅ **Realistic** — Publishing rates and latencies are achievable in ROS 2
✅ **Documented** — Clear examples of how to publish/subscribe in Python

Topics follow ROS 2 naming conventions and design patterns, ensuring examples are both educational and practical.
