# Gazebo: Physics-Based Robot Simulation

**Gazebo** is the gold standard for robotics simulation in ROS 2. In this lesson, you'll learn how to spawn robots, configure physics, add sensors, and integrate with your ROS 2 control nodes. Let's bring your URDF models to life!

---

## What is Gazebo?

**Gazebo** is an open-source 3D robot simulator with accurate physics, sensor models, and ROS 2 integration.

**Key Features**:
- **Multiple Physics Engines**: ODE (default), Bullet, DART, Simbody
- **Sensor Plugins**: Camera, LIDAR, IMU, GPS, force/torque
- **ROS 2 Native**: Topics, services, URDF loading
- **Scalable**: Headless mode for large-scale parallel simulations
- **Extensible**: Write custom plugins in C++

**Use Cases**:
- Test navigation algorithms (SLAM, path planning)
- Validate manipulation (pick-and-place, grasping)
- Train reinforcement learning agents
- Multi-robot coordination

**Versions**:
- **Gazebo Classic** (v11): Older, stable, widely used
- **Gazebo Ignition** (now "Gazebo"): Modern rewrite, better performance
- For ROS 2 Humble: Use Gazebo Classic (ros-humble-gazebo-ros-pkgs)

---

## Installing Gazebo

```bash
# Install Gazebo Classic for ROS 2 Humble
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Verify installation
gazebo --version

# Expected output: Gazebo multi-robot simulator, version 11.x.x
```

---

## Launching Gazebo

### Method 1: Empty World

```bash
# Launch Gazebo GUI with empty world
gazebo
```

**What You See**:
- 3D viewport (drag to rotate, scroll to zoom)
- Ground plane
- Light source
- Insert tab (add models like boxes, spheres)

### Method 2: Pre-Built World

```bash
# Launch a world with obstacles
gazebo worlds/willowgarage.world
```

**Included Worlds** (`/usr/share/gazebo-11/worlds/`):
- `empty.world`: Just ground and sky
- `willowgarage.world`: Office environment
- `cafe.world`: Restaurant scene
- `robocup14_spl_field.world`: Soccer field

---

## Spawning a Robot from URDF

### Step 1: Create Launch File

`spawn_robot.launch.py`:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Path to URDF file
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'my_robot.urdf'
    )

    # Read URDF
    with open(urdf_file, 'r') as file:
        robot_desc = file.read()

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'my_robot',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.5'  # Initial position
            ],
            output='screen'
        ),

        # Publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        )
    ])
```

### Step 2: Launch

```bash
ros2 launch my_robot_bringup spawn_robot.launch.py
```

**Result**: Your URDF robot appears in Gazebo at position (0, 0, 0.5)!

---

## Physics Configuration

### World File Structure

`.world` files define environments:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    <!-- Physics Engine Settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>  <!-- 1ms timestep -->
      <real_time_factor>1.0</real_time_factor>  <!-- Real-time (1x speed) -->
      <real_time_update_rate>1000</real_time_update_rate>  <!-- 1000 Hz -->
    </physics>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun (lighting) -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom Model: Box Obstacle -->
    <model name="box">
      <pose>2 0 0.5 0 0 0</pose>  <!-- x y z roll pitch yaw -->
      <static>true</static>  <!-- Doesn't move -->
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>  <!-- Red -->
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### Physics Engine Options

| Engine | Accuracy | Speed | Use Case |
|--------|----------|-------|----------|
| **ODE** | Good | Fast | General robotics (default) |
| **Bullet** | Good | Fast | Collisions, manipulation |
| **DART** | Excellent | Moderate | Precise dynamics, legged robots |
| **Simbody** | Excellent | Slow | Biomechanics, medical robotics |

**Change Physics Engine** (in world file):
```xml
<physics type="bullet">
```

---

## Adding Sensors to Your Robot

Sensors in Gazebo are implemented as **plugins** in URDF or SDF.

### Camera Sensor

Add to your URDF:
```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>  <!-- 30 FPS -->
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>

    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/</namespace>
        <remapping>~/image_raw:=/camera/image_raw</remapping>
        <remapping>~/camera_info:=/camera/camera_info</remapping>
      </ros>
      <camera_name>my_camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Result**: Camera publishes images to `/camera/image_raw` topic!

### LIDAR Sensor

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>  <!-- Show rays in Gazebo GUI -->
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>

    <plugin name="gazebo_ros_lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/</namespace>
        <remapping>~/out:=/scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Result**: LIDAR publishes scan data to `/scan` topic!

### IMU Sensor

```xml
<gazebo reference="imu_link">
  <sensor type="imu" name="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>  <!-- 100 Hz -->
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/</namespace>
        <remapping>~/out:=/imu/data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

**Result**: IMU publishes orientation, angular velocity, linear acceleration to `/imu/data`!

---

## Controlling Your Robot in Gazebo

### Option 1: Diff Drive Plugin (Wheeled Robots)

```xml
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <namespace>/</namespace>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=odom</remapping>
    </ros>

    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.5</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>

    <max_wheel_torque>20</max_wheel_torque>
    <max_wheel_acceleration>1.0</max_wheel_acceleration>

    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <publish_wheel_tf>true</publish_wheel_tf>

    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
  </plugin>
</gazebo>
```

**Result**: Robot subscribes to `/cmd_vel` (Twist messages) and moves accordingly!

**Test**:
```bash
# Publish velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

### Option 2: Joint State Publisher (Manipulators)

For robot arms, use `joint_state_publisher` and `joint_trajectory_controller`:

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find my_robot_bringup)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

---

## Mini Exercise: Build a Simulation World

**Challenge**: Create a custom Gazebo world with obstacles and spawn a robot.

**Requirements**:
1. Create `.world` file with:
   - Ground plane
   - Sun (lighting)
   - 3 box obstacles (different sizes/positions)
   - Custom gravity (e.g., Mars: -3.71 m/s²)
2. Launch Gazebo with your world
3. Spawn a robot using a launch file
4. Drive the robot and test collision with obstacles

**Hints**:
- Start from `empty.world` template
- Use `<include>` for ground and sun
- Use `<model>` for custom boxes
- Set `<static>true</static>` for obstacles (they don't move)

**Extension**: Add a ramp (use `<pose>` to tilt a box 30 degrees)!

---

## Summary

You've mastered Gazebo for physics-based robot simulation:

✅ **Gazebo simulates** robots with accurate physics and sensor models
✅ **URDF spawning** loads robots into Gazebo worlds
✅ **World files** define environments (ground, obstacles, lighting, physics)
✅ **Sensor plugins** (camera, LIDAR, IMU) publish ROS 2 topics
✅ **Control plugins** (diff_drive, ros2_control) enable robot actuation
✅ **Physics tuning** (timestep, engine) affects simulation accuracy

**Next Up**: Explore **Unity** for photorealistic HRI simulation!

Ready for beautiful visuals? Let's continue! 🎨🤖
