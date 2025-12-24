# URDF: Describing Your Humanoid Robot

Every robot needs a **digital blueprint**—a precise description of its physical structure. In ROS 2, this is done using **URDF** (Universal Robot Description Format). In this lesson, you'll learn how to define your robot's links, joints, and properties for simulation and control.

---

## What is URDF?

**URDF** is an XML-based format that describes:
- **Links**: Rigid body parts (limbs, torso, head)
- **Joints**: Connections between links (how they move)
- **Visual properties**: How the robot looks (geometry, colors, meshes)
- **Collision properties**: Simplified shapes for collision detection
- **Inertial properties**: Mass, center of mass, inertia tensors (for physics)

**Why URDF?**
- **Simulation**: Gazebo, MuJoCo, Isaac Sim use URDF to simulate robots
- **Visualization**: RViz displays robot models from URDF
- **Motion Planning**: MoveIt! uses URDF to plan collision-free paths
- **Control**: Robot state publishers use URDF to compute forward kinematics

---

## URDF Structure: Links and Joints

A robot is a **kinematic tree** of links connected by joints.

```
       Base Link (fixed to ground)
            │
      ┌─────┴─────┐
      │  Joint 1  │ (revolute: shoulder rotation)
      └─────┬─────┘
            │
        Link 1 (upper arm)
            │
      ┌─────┴─────┐
      │  Joint 2  │ (revolute: elbow rotation)
      └─────┬─────┘
            │
        Link 2 (forearm)
            │
      ┌─────┴─────┐
      │  Joint 3  │ (fixed: end-effector)
      └─────┬─────┘
            │
        Link 3 (gripper)
```

---

## Example: Simple 2-Link Robot Arm

Let's build a URDF for a basic robot arm.

### URDF File: `simple_arm.urdf`

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- ========== BASE LINK ========== -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- ========== LINK 1 (Upper Arm) ========== -->
  <link name="link1">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="orange">
        <color rgba="1 0.5 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0"
               iyy="0.004" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- ========== JOINT 1 (Base to Link 1) ========== -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- ========== LINK 2 (Forearm) ========== -->
  <link name="link2">
    <visual>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <box size="0.04 0.04 0.25"/>
      </geometry>
      <material name="orange">
        <color rgba="1 0.5 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <box size="0.04 0.04 0.25"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <mass value="0.3"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0"
               iyy="0.002" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- ========== JOINT 2 (Link 1 to Link 2) ========== -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="5" velocity="1.0"/>
  </joint>

</robot>
```

---

## URDF Components Explained

### 1. Links

**Structure**:
```xml
<link name="link_name">
  <visual>      <!-- How it looks -->
  <collision>   <!-- Collision detection -->
  <inertial>    <!-- Mass and inertia -->
</link>
```

**Visual**:
- `<geometry>`: Shape (box, cylinder, sphere, mesh)
- `<material>`: Color or texture
- `<origin>`: Position and orientation (xyz, rpy)

**Collision**:
- Simplified geometry for faster collision checking
- Often same as visual for simple shapes

**Inertial**:
- `<mass>`: Weight in kg
- `<inertia>`: 3x3 inertia tensor (resistance to rotation)
- Critical for realistic physics simulation

### 2. Joints

**Types**:
- **revolute**: Hinge joint with angle limits (e.g., elbow)
- **continuous**: Hinge without limits (e.g., wheels)
- **prismatic**: Sliding joint (e.g., linear actuator)
- **fixed**: Rigid connection (no movement)
- **floating**: 6 DOF (rarely used, mostly for base)
- **planar**: 2D planar motion

**Properties**:
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>   <!-- Which link above -->
  <child link="child_link"/>     <!-- Which link below -->
  <origin xyz="0 0 0.1" rpy="0 0 0"/>  <!-- Joint position -->
  <axis xyz="0 0 1"/>            <!-- Rotation/translation axis -->
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
</joint>
```

**Limit Parameters**:
- `lower` / `upper`: Motion bounds (radians or meters)
- `effort`: Max torque/force (N·m or N)
- `velocity`: Max speed (rad/s or m/s)

---

## Visualizing URDF in RViz

### Step 1: Install Required Packages

```bash
sudo apt install ros-humble-urdf-tutorial
sudo apt install ros-humble-joint-state-publisher-gui
```

### Step 2: Create Launch File

`display.launch.py`:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='simple_arm.urdf'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': open(LaunchConfiguration('model')).read()
            }]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '$(find urdf_tutorial)/config/urdf.rviz']
        )
    ])
```

### Step 3: Launch

```bash
ros2 launch display.launch.py model:=simple_arm.urdf
```

**What You'll See**:
- RViz window showing your robot
- GUI with sliders to control joints
- Robot updates in real-time as you move sliders

---

## Advanced: Using Meshes

For realistic visuals, use 3D mesh files (STL, DAE, OBJ).

```xml
<visual>
  <geometry>
    <mesh filename="package://my_robot/meshes/link1.stl" scale="0.001 0.001 0.001"/>
  </geometry>
</visual>
```

**Best Practices**:
- Use low-poly meshes for collision (performance)
- Use high-poly meshes for visual (appearance)
- Store meshes in `meshes/` folder in your package

---

## Xacro: Macros for URDF

**Xacro** extends URDF with:
- **Variables**: Reuse values (e.g., `link_length`)
- **Macros**: Template-based link/joint generation
- **Math**: Compute values (e.g., `${pi/2}`)

### Example: Parameterized Arm

`simple_arm.urdf.xacro`:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_arm">

  <!-- Parameters -->
  <xacro:property name="link1_length" value="0.3"/>
  <xacro:property name="link2_length" value="0.25"/>

  <!-- Macro for a link -->
  <xacro:macro name="arm_link" params="name length">
    <link name="${name}">
      <visual>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
        <geometry>
          <box size="0.05 0.05 ${length}"/>
        </geometry>
        <material name="orange">
          <color rgba="1 0.5 0 1"/>
        </material>
      </visual>
    </link>
  </xacro:macro>

  <!-- Use macro -->
  <xacro:arm_link name="link1" length="${link1_length}"/>
  <xacro:arm_link name="link2" length="${link2_length}"/>

</robot>
```

**Convert Xacro to URDF**:
```bash
ros2 run xacro xacro simple_arm.urdf.xacro > simple_arm.urdf
```

---

## Mini Exercise: Build Your Robot Arm

**Challenge**: Create a 3-link robot arm with the following specs:

**Requirements**:
1. **Base link**: Cylinder (radius 0.1m, height 0.05m)
2. **Link 1**: Box (0.05 x 0.05 x 0.4m), revolute joint around Z-axis
3. **Link 2**: Box (0.04 x 0.04 x 0.3m), revolute joint around Y-axis
4. **Link 3**: Sphere (radius 0.03m), fixed joint (end-effector)
5. All joints have limits: ±90° (±1.57 rad)

**Hints**:
- Start from the base and work outward
- Each joint's `origin` is at the end of the parent link
- Use RViz to check your work

**Extension**: Add a gripper using two prismatic joints!

---

## Summary

You've mastered the basics of URDF:

✅ **URDF describes robot structure** (links + joints + properties)
✅ **Links** define rigid bodies with visual, collision, and inertial properties
✅ **Joints** connect links and specify motion constraints
✅ **Joint types** include revolute, continuous, prismatic, fixed
✅ **RViz visualizes** URDF models interactively
✅ **Xacro extends URDF** with variables and macros for maintainability

**Next Module**: You'll dive into **digital twins** and simulation with Gazebo and Unity!

Ready to bring your robot to life in simulation? Let's go! 🎮🤖
