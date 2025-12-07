# Quickstart: Chapter 1 — The Robotic Nervous System (ROS 2)

**Date**: 2025-12-07
**Status**: Complete
**Purpose**: Quick reference for running and validating all chapter examples

---

## Prerequisites

Before starting, ensure you have:

1. **ROS 2 Humble or Iron** installed
   ```bash
   # Verify installation
   ros2 --version
   # Expected output: ROS 2 (Iron or Humble)
   ```

2. **Gazebo** (for User Story 6)
   ```bash
   # Verify installation
   which gazebo
   # Expected: /usr/bin/gazebo
   ```

3. **Python 3.10+**
   ```bash
   python3 --version
   # Expected: Python 3.10.x or higher
   ```

4. **rclpy** (ROS 2 Python client)
   ```bash
   python3 -c "import rclpy; print('rclpy OK')"
   # Expected: rclpy OK
   ```

5. **Standard ROS 2 packages**
   ```bash
   sudo apt-get install ros-<distro>-std-msgs \
     ros-<distro>-sensor-msgs \
     ros-<distro>-geometry-msgs \
     ros-<distro>-example-interfaces
   # Replace <distro> with humble or iron
   ```

---

## Chapter Examples Directory Structure

```bash
chapters/01-ros2-nervous-system/
├── README.md                           # Chapter overview
├── content.md                          # Main chapter content
├── examples/
│   ├── 01_node_creation.py            # User Story 2 example
│   ├── 02_publisher.py                # User Story 3 example
│   ├── 03_subscriber.py               # User Story 3 example
│   ├── 04_service_server.py           # User Story 3 example
│   ├── 05_service_client.py           # User Story 3 example
│   ├── 06_ai_agent.py                 # User Story 4 example
│   ├── mock_sensor_publisher.py       # Helper for testing
│   ├── gazebo_controller.py           # User Story 6 example
│   ├── humanoid_robot.urdf            # User Story 5 example
│   ├── launch_pipeline.launch.py      # User Story 6 example
│   └── README.md                       # Examples navigation
├── tests/
│   ├── test_node_execution.py         # Unit tests
│   ├── test_urdf_validation.py        # URDF validation
│   └── test_pipeline_stability.py     # Integration tests
└── references.bib                      # Bibliography (APA format)
```

---

## Running Examples by User Story

### User Story 1: Understand ROS 2 Architecture

**What You'll Learn**: ROS 2 middleware concepts (no code execution)

**Activity**: Read Chapter 1 section "The Middleware Foundation"
```bash
cd chapters/01-ros2-nervous-system/
cat content.md | grep -A 50 "The Middleware Foundation"
```

**Validation**: Answer conceptual questions at section end
- [ ] Explain the difference between Topics and Services
- [ ] Describe how loosely coupled components communicate
- [ ] Identify when to use Topics vs. Services

---

### User Story 2: Build a ROS 2 Node in Python

**What You'll Learn**: Create a basic ROS 2 node using rclpy

**Example Code**: `examples/01_node_creation.py`

**Step-by-Step**:

1. **Navigate to examples directory**
   ```bash
   cd chapters/01-ros2-nervous-system/examples/
   ```

2. **Source ROS 2 setup** (if not already sourced)
   ```bash
   source /opt/ros/humble/setup.bash
   # or for Iron:
   source /opt/ros/iron/setup.bash
   ```

3. **Run the example**
   ```bash
   python3 01_node_creation.py
   ```

4. **Expected Output**:
   ```
   [INFO] [timestamp] [<node_name>] ROS 2 node started
   [INFO] [timestamp] [<node_name>] Hello from ROS 2
   [INFO] [timestamp] [<node_name>] Shutting down
   ```

5. **Verify in another terminal**:
   ```bash
   ros2 node list
   # Expected output: /<node_name>
   ```

6. **Stop the node**:
   - Press `Ctrl+C` in the first terminal

**Validation Checklist**:
- [ ] Node initializes without errors
- [ ] Node appears in `ros2 node list`
- [ ] Log messages appear in console
- [ ] Node shuts down cleanly on `Ctrl+C`

---

### User Story 3: Implement Topics and Services

**What You'll Learn**: Publishers, Subscribers, Service Servers, and Clients

**Example Code**:
- `examples/02_publisher.py` — Publishes Float64 data
- `examples/03_subscriber.py` — Subscribes to Float64 data
- `examples/04_service_server.py` — Serves AddTwoInts service
- `examples/05_service_client.py` — Calls AddTwoInts service

#### Part A: Topics (Publisher/Subscriber)

**Terminal 1: Start Publisher**
```bash
cd chapters/01-ros2-nervous-system/examples/
source /opt/ros/<distro>/setup.bash
python3 02_publisher.py
# Expected: Publishing [sensor data] to /sensor_data every 0.1 seconds
```

**Terminal 2: Start Subscriber**
```bash
cd chapters/01-ros2-nervous-system/examples/
source /opt/ros/<distro>/setup.bash
python3 03_subscriber.py
# Expected:
# Sensor data received: [value]
# Sensor data received: [value]
# ...
```

**Terminal 3: Inspect Topic (optional)**
```bash
ros2 topic list
# Should show: /sensor_data

ros2 topic echo /sensor_data
# Should print Float64 messages from publisher
```

**Validation Checklist**:
- [ ] Publisher runs without errors
- [ ] Subscriber receives messages
- [ ] Messages appear in `ros2 topic echo`
- [ ] Both nodes appear in `ros2 node list`
- [ ] Can see latency is <10ms

#### Part B: Services (Server/Client)

**Terminal 1: Start Service Server**
```bash
cd chapters/01-ros2-nervous-system/examples/
source /opt/ros/<distro>/setup.bash
python3 04_service_server.py
# Expected: Service server waiting for requests...
```

**Terminal 2: Call Service**
```bash
cd chapters/01-ros2-nervous-system/examples/
source /opt/ros/<distro>/setup.bash
python3 05_service_client.py
# Expected:
# Sending request: {a: 5, b: 3}
# Received response: sum = 8
```

**Terminal 3: Call service manually (optional)**
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"
# Expected: response: sum: 30
```

**Validation Checklist**:
- [ ] Service server starts without errors
- [ ] Service client receives response
- [ ] Manual service call works
- [ ] Response is correct (5 + 3 = 8)

---

### User Story 4: Bridge Python AI Agents to ROS 2 Controllers

**What You'll Learn**: Create an AI agent that subscribes to sensor data and publishes control commands

**Example Code**:
- `examples/06_ai_agent.py` — Decision-making agent
- `examples/mock_sensor_publisher.py` — Simulated sensor data
- `examples/gazebo_controller.py` — Simple controller (for User Story 6)

**Terminal 1: Start Mock Sensor Publisher**
```bash
cd chapters/01-ros2-nervous-system/examples/
source /opt/ros/<distro>/setup.bash
python3 mock_sensor_publisher.py
# Expected: Publishing sensor data: [0-100] every 0.1 seconds
```

**Terminal 2: Start AI Agent**
```bash
cd chapters/01-ros2-nervous-system/examples/
source /opt/ros/<distro>/setup.bash
python3 06_ai_agent.py
# Expected:
# Agent started. Listening to /sensor_data
# Sensor: 45.3 → Decision: STOP
# Sensor: 62.1 → Decision: MOVE_FORWARD
# ...
```

**Terminal 3: Inspect Topics (optional)**
```bash
ros2 topic echo /sensor_data
# Shows Float64 data from mock publisher

ros2 topic echo /robot_commands
# Shows Float64 commands from agent
```

**Validation Checklist**:
- [ ] Mock sensor publisher runs continuously
- [ ] AI agent receives sensor data
- [ ] Agent publishes control commands based on threshold logic
- [ ] Latency <100ms (check timestamps in agent output)
- [ ] Both topics appear in `ros2 topic list`

---

### User Story 5: Understand URDF for Humanoid Structure

**What You'll Learn**: Read and visualize robot URDF file

**Example Code**: `examples/humanoid_robot.urdf`

**Visualization in RViz**:

1. **Start RViz**:
   ```bash
   source /opt/ros/<distro>/setup.bash
   rviz2
   ```

2. **Load Robot Model**:
   - In RViz left panel: Add → RobotModel
   - Select `/robot_description` parameter
   - You should see the humanoid robot structure

3. **Or use command-line**:
   ```bash
   source /opt/ros/<distro>/setup.bash
   ros2 run rviz2 rviz2 -d chapters/01-ros2-nervous-system/examples/humanoid.rviz
   # (if rviz config provided)
   ```

**Validate URDF File**:
```bash
cd chapters/01-ros2-nervous-system/examples/
python3 -c "
from urdf_parser_py.urdf import URDF
robot = URDF.from_xml_file('humanoid_robot.urdf')
print(f'Robot: {robot.name}')
print(f'Links: {len(robot.links)}')
print(f'Joints: {len(robot.joints)}')
for joint in robot.joints:
    print(f'  - {joint.name}: {joint.joint_type}')
"
# Expected:
# Robot: simple_humanoid
# Links: 8
# Joints: 7
#   - base_to_torso: fixed
#   - torso_left_hip: revolute
#   ... etc
```

**Validation Checklist**:
- [ ] URDF file parses without errors
- [ ] Contains 8 links and 7 joints
- [ ] All joint types are valid (fixed, revolute)
- [ ] RViz displays humanoid structure correctly
- [ ] Can rotate view and see all parts

---

### User Story 6: Run Complete Control Pipeline

**What You'll Learn**: Integrate all components (Gazebo, ROS 2 nodes, AI agent, URDF)

**Example Code**:
- `examples/launch_pipeline.launch.py` — Master launch file
- `examples/gazebo_controller.py` — Controller for Gazebo
- `examples/humanoid_robot.urdf` — Robot model

**Full Pipeline Execution**:

**Terminal 1: Start Gazebo with Humanoid**
```bash
cd chapters/01-ros2-nervous-system/examples/
source /opt/ros/<distro>/setup.bash
ros2 launch launch_pipeline.launch.py
# Expected:
# Starting Gazebo simulator...
# Spawning humanoid_robot...
# All nodes ready
```

**Terminal 2: Start Mock Sensor Publisher** (if not in launch)
```bash
cd chapters/01-ros2-nervous-system/examples/
source /opt/ros/<distro>/setup.bash
python3 mock_sensor_publisher.py
```

**Terminal 3: Start AI Agent** (if not in launch)
```bash
cd chapters/01-ros2-nervous-system/examples/
source /opt/ros/<distro>/setup.bash
python3 06_ai_agent.py
```

**Expected Behavior**:
- Gazebo window opens showing humanoid robot
- Sensor values print in terminal
- Agent makes decisions and publishes commands
- Robot joints move in response to commands
- System runs stably for 10+ seconds without crashes

**Validation Checklist**:
- [ ] Gazebo launches without errors
- [ ] Humanoid robot visible in simulation
- [ ] Sensor data flowing through system
- [ ] Agent making decisions
- [ ] Robot responding to commands
- [ ] System stable for 10+ seconds
- [ ] No deadlocks or timeouts
- [ ] All nodes appear in `ros2 node list`
- [ ] All topics appear in `ros2 topic list`

---

## Running Tests

### Unit Tests: Code Execution Validation

```bash
cd chapters/01-ros2-nervous-system/
python3 -m pytest tests/test_node_execution.py -v
# Verifies each code example runs without error
```

### Integration Tests: URDF Validation

```bash
cd chapters/01-ros2-nervous-system/
python3 -m pytest tests/test_urdf_validation.py -v
# Verifies URDF structure is correct
```

### Acceptance Tests: Pipeline Stability

```bash
cd chapters/01-ros2-nervous-system/
python3 -m pytest tests/test_pipeline_stability.py -v
# Verifies complete pipeline runs 10+ seconds without crashes
```

---

## Troubleshooting

### Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| "No module named rclpy" | ROS 2 not sourced | Run `source /opt/ros/<distro>/setup.bash` |
| "Service not available" | Service server not running | Start server in separate terminal first |
| "Topic not found" | Publisher not running | Start publisher before subscriber |
| "URDF parse error" | Invalid URDF syntax | Validate with urdf_parser_py |
| Gazebo crashes | Graphics/ROS_MASTER_URI issue | Check ROS domain ID and graphics |
| High latency | System overloaded | Close other applications; check CPU usage |

### Debugging Commands

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Inspect a topic
ros2 topic echo /sensor_data

# List all services
ros2 service list

# Call a service manually
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# Check ROS 2 domain ID (must match across nodes)
echo $ROS_DOMAIN_ID

# View ROS 2 graph
ros2 run rqt_graph rqt_graph
```

---

## Summary Checklist

After completing all examples, you should be able to:

- [ ] **US1**: Explain ROS 2 middleware, Topics, Services, pub/sub pattern
- [ ] **US2**: Create a ROS 2 node in Python and verify it appears in `ros2 node list`
- [ ] **US3**: Implement and test Publisher/Subscriber and Service Server/Client
- [ ] **US4**: Write an AI agent that subscribes to sensors and publishes commands
- [ ] **US5**: Read URDF and visualize humanoid robot in RViz
- [ ] **US6**: Run complete end-to-end pipeline with Gazebo simulation

If you can complete all 6 items, you've mastered Chapter 1 content!

---

## Next Steps

After Chapter 1:
- **Chapter 2**: Advanced ROS 2 (custom messages, lifecycle management, parameters)
- **Chapter 3**: Robot Kinematics (forward/inverse kinematics, trajectory planning)
- **Chapter 4**: AI Control (learning agents, reinforcement learning in robotics)
- **Chapter 5**: Humanoid robotics (bipedal locomotion, manipulation, whole-body control)
