# Introduction to ROS 2 for Humanoid Robots

Welcome to Module 1! In this lesson, you'll learn about **ROS 2** (Robot Operating System 2), the modern framework that powers humanoid robots and serves as their "nervous system." By the end of this lesson, you'll understand why ROS 2 is essential and how to get started.

---

## What is ROS 2?

**ROS 2** is an open-source middleware framework for building complex robot software. Despite its name, it's not an operating system like Linux or Windows—it runs *on top* of operating systems (usually Ubuntu Linux).

Think of ROS 2 as the **communication backbone** that connects different parts of your robot:
- Sensors (cameras, LIDAR, IMU)
- AI algorithms (object detection, path planning)
- Control systems (motor commands, balance control)
- User interfaces (teleop, dashboards)

**Key Features**:
- **Modular Architecture**: Break complex systems into independent, reusable components
- **Language Support**: Write nodes in Python, C++, or other languages
- **Distributed Computing**: Run components on different computers (onboard + cloud)
- **Rich Ecosystem**: Thousands of open-source packages for navigation, manipulation, perception

---

## Why ROS 2 (Not ROS 1)?

ROS 2 is the successor to ROS 1, redesigned from the ground up to address real-world robotics needs:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Real-Time Support** | ❌ Not guaranteed | ✅ Real-time capable (critical for control loops) |
| **Security** | ❌ No built-in security | ✅ DDS Security for encrypted communication |
| **Multi-Robot** | ⚠️ Complex workarounds | ✅ Native multi-robot support |
| **Quality of Service** | ❌ Best-effort only | ✅ Configurable (reliable/best-effort) |
| **Platforms** | 🐧 Linux only | 🐧🪟🍎 Linux, Windows, macOS |
| **Industry Adoption** | 🔬 Mostly research | 🏭 Production robotics (BMW, Bosch, etc.) |

**Bottom Line**: Use ROS 2 for new projects. It's faster, safer, and more robust.

---

## Core Concepts: The ROS 2 Graph

ROS 2 organizes your robot software as a **computational graph**:

```
┌─────────────┐      /camera/image      ┌──────────────────┐
│ Camera Node │ ──────────────────────> │ Detection Node   │
└─────────────┘                         └──────────────────┘
                                               │
                                               │ /detected_objects
                                               ▼
                                        ┌──────────────────┐
                                        │ Planning Node    │
                                        └──────────────────┘
                                               │
                                               │ /cmd_vel
                                               ▼
                                        ┌──────────────────┐
                                        │ Motor Controller │
                                        └──────────────────┘
```

### 1. Nodes
**Nodes** are independent processes, each responsible for one task (e.g., "read camera," "detect objects").

### 2. Topics
**Topics** are named channels for asynchronous, many-to-many communication (publisher-subscriber pattern).

### 3. Services
**Services** enable synchronous request-reply communication (like function calls across nodes).

### 4. Messages
**Messages** define the data structure sent over topics/services (e.g., `sensor_msgs/Image` for camera data).

**Philosophy**: Build complex behaviors from simple, composable nodes.

---

## Installing ROS 2

### Quick Install (Ubuntu 22.04 Recommended)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble (latest LTS)
sudo apt update
sudo apt install ros-humble-desktop -y

# Source setup (add to ~/.bashrc for persistence)
source /opt/ros/humble/setup.bash
```

### Verify Installation

```bash
# Check ROS 2 version
ros2 --version

# Expected output: ros2 cli version 0.xx.x
```

---

## Your First ROS 2 Command: Hello World

Let's run a pre-built demo to see ROS 2 in action!

### Terminal 1: Run a Talker Node

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py talker
```

**Expected Output**:
```
[INFO] [talker]: Publishing: "Hello World: 0"
[INFO] [talker]: Publishing: "Hello World: 1"
[INFO] [talker]: Publishing: "Hello World: 2"
```

### Terminal 2: Run a Listener Node

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

**Expected Output**:
```
[INFO] [listener]: I heard: "Hello World: 0"
[INFO] [listener]: I heard: "Hello World: 1"
[INFO] [listener]: I heard: "Hello World: 2"
```

**What Happened?**
- `talker` node publishes messages to the `/chatter` topic
- `listener` node subscribes to `/chatter` and receives messages
- They communicate without knowing about each other (decoupled architecture)

---

## Inspecting the ROS 2 Graph

While both nodes are running, open a third terminal:

```bash
# List all running nodes
ros2 node list
# Output: /talker, /listener

# List all topics
ros2 topic list
# Output: /chatter, /parameter_events, /rosout

# See message details on /chatter topic
ros2 topic echo /chatter
# Live stream of messages!

# Check topic info
ros2 topic info /chatter
# Shows publishers, subscribers, message type
```

**Pro Tip**: Use `rqt_graph` for a visual graph:
```bash
ros2 run rqt_graph rqt_graph
```

---

## Mini Exercise: Run a Pre-Built Demo

**Challenge**: Explore the `turtlesim` demo (a classic ROS learning tool).

**Instructions**:
1. **Terminal 1**: Start the simulator
   ```bash
   ros2 run turtlesim turtlesim_node
   ```
   *(A window with a turtle appears)*

2. **Terminal 2**: Control the turtle with keyboard
   ```bash
   ros2 run turtlesim turtle_teleop_key
   ```
   *(Use arrow keys to move the turtle)*

3. **Terminal 3**: Inspect topics
   ```bash
   ros2 topic list
   # Find /turtle1/cmd_vel (velocity commands)
   # Find /turtle1/pose (turtle position)

   ros2 topic echo /turtle1/pose
   # Watch the turtle's position update as you drive!
   ```

**Bonus**: Publish a manual command to move the turtle:
```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

---

## Summary

Congratulations! You've just taken your first steps with ROS 2. Here's what you learned:

✅ **ROS 2 is a middleware framework** for building modular robot software
✅ **Nodes communicate via topics** (pub-sub) and services (request-reply)
✅ **ROS 2 improves on ROS 1** with real-time support, security, and multi-platform compatibility
✅ **Installation is straightforward** on Ubuntu (Docker alternatives exist)
✅ **Command-line tools** (`ros2 node`, `ros2 topic`) help you inspect and debug systems

**Next Up**: In the next lesson, you'll learn how to create your own ROS 2 nodes and understand communication patterns in depth.

Ready to build your first custom node? Let's go! 🤖
