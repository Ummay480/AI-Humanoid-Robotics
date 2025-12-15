# LIDAR Sensors for Navigation & Mapping

Your robot needs to **see its environment** to navigate safely. **LIDAR** (Light Detection and Ranging) sensors emit laser beams to measure distances, creating detailed 3D maps of the surroundings. In this lesson, you'll learn how LIDAR works, how to process point cloud data, and how to integrate LIDAR with ROS 2 for autonomous navigation.

---

## What is LIDAR?

**LIDAR** measures distance by timing how long laser pulses take to bounce back from objects.

**How It Works**:
```
1. Emit laser pulse
2. Pulse hits object and reflects
3. Sensor detects reflected light
4. Calculate distance: d = (c × t) / 2
   - c = speed of light
   - t = round-trip time
5. Rotate/scan to measure multiple points
```

**Output**: **Point cloud** — thousands of 3D coordinates (x, y, z) representing surfaces.

---

## 2D vs. 3D LIDAR

### 2D LIDAR (Planar Scanning)

**Characteristics**:
- Scans in a single horizontal plane (360° rotation)
- Typical range: 0.1m - 30m
- Output: Distance measurements at fixed angular intervals
- **Use Cases**: Indoor navigation, obstacle avoidance, 2D SLAM

**Example**: Hokuyo UTM-30LX (270° field of view, 0.25° resolution)

**ROS 2 Message**: `sensor_msgs/LaserScan`

### 3D LIDAR (Volumetric Scanning)

**Characteristics**:
- Multiple laser beams at different vertical angles (e.g., 16, 32, 64, 128 channels)
- Captures full 3D environment
- **Use Cases**: Autonomous vehicles, terrain mapping, 3D object detection

**Example**: Velodyne VLP-16 (16 channels, 360° horizontal, ±15° vertical)

**ROS 2 Message**: `sensor_msgs/PointCloud2`

---

## LIDAR Data in ROS 2

### LaserScan Message (2D LIDAR)

**Structure**:
```python
# sensor_msgs/LaserScan
header:
  stamp: time
  frame_id: "lidar_link"
angle_min: -3.14159  # Start angle (radians)
angle_max: 3.14159   # End angle (radians)
angle_increment: 0.0174533  # Angular resolution (1°)
time_increment: 0.0001
scan_time: 0.1       # Time for full scan
range_min: 0.1       # Min valid distance (m)
range_max: 10.0      # Max valid distance (m)
ranges: [3.2, 3.5, inf, 2.1, ...]  # Distance array
intensities: [100, 95, 0, 120, ...]  # Reflectivity (optional)
```

**Key Fields**:
- **ranges**: Distance to obstacles at each angle (in meters)
- **inf**: No object detected (beam went beyond range_max)
- **intensities**: Surface reflectivity (useful for detecting materials)

### PointCloud2 Message (3D LIDAR)

**Structure**:
```python
# sensor_msgs/PointCloud2
header:
  stamp: time
  frame_id: "lidar_link"
height: 1  # Unordered cloud
width: 10000  # Number of points
fields:  # Data structure per point
  - name: "x", offset: 0, datatype: FLOAT32
  - name: "y", offset: 4, datatype: FLOAT32
  - name: "z", offset: 8, datatype: FLOAT32
  - name: "intensity", offset: 12, datatype: FLOAT32
point_step: 16  # Bytes per point
row_step: 160000  # Bytes per row
data: [binary blob]  # Packed point data
is_dense: false  # Contains invalid points (NaN, inf)
```

**Each Point**: (x, y, z, intensity) relative to sensor frame.

---

## Subscribing to LIDAR Data

### Example: Obstacle Detection with 2D LIDAR

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np


class ObstacleDetector(Node):
    """Detects obstacles using 2D LIDAR."""

    def __init__(self):
        super().__init__('obstacle_detector')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.obstacle_threshold = 0.8  # meters
        self.get_logger().info('Obstacle detector started!')

    def scan_callback(self, msg):
        """Process LIDAR scan."""
        # Convert to numpy array
        ranges = np.array(msg.ranges)

        # Replace inf with max_range
        ranges[np.isinf(ranges)] = msg.range_max

        # Find closest obstacle
        min_distance = np.min(ranges)
        min_index = np.argmin(ranges)

        # Calculate angle of closest obstacle
        angle = msg.angle_min + (min_index * msg.angle_increment)
        angle_deg = np.degrees(angle)

        # Alert if obstacle too close
        if min_distance < self.obstacle_threshold:
            self.get_logger().warn(
                f'⚠️  Obstacle detected at {min_distance:.2f}m, '
                f'angle {angle_deg:.1f}°'
            )
        else:
            self.get_logger().info(f'✅ Path clear (closest: {min_distance:.2f}m)')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Test**:
```bash
# Terminal 1: Launch robot with LIDAR (e.g., TurtleBot3)
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Run obstacle detector
python3 obstacle_detector.py
```

---

## Visualizing LIDAR in RViz

**Steps**:
1. Launch RViz:
   ```bash
   ros2 run rviz2 rviz2
   ```

2. **Add LaserScan Display**:
   - Click **Add** → **By topic** → `/scan` → **LaserScan**
   - Set **Fixed Frame**: `map` or `odom`
   - Set **Size**: 0.05 (point size)
   - Set **Color**: Intensity or Flat Color (red)

3. **Add PointCloud2 Display** (for 3D LIDAR):
   - Click **Add** → **By topic** → `/velodyne_points` → **PointCloud2**
   - Set **Style**: Points (or Flat Squares)
   - Set **Color Transformer**: Intensity or AxisColor

**Result**: You'll see laser beams radiating from the robot, with points showing detected surfaces!

---

## Applications in Robotics

### 1. SLAM (Simultaneous Localization and Mapping)

**Goal**: Build a map while estimating robot position.

**Popular Algorithms**:
- **Cartographer** (Google): 2D/3D SLAM with loop closure
- **SLAM Toolbox**: Real-time 2D SLAM for ROS 2

**Launch Example**:
```bash
ros2 launch slam_toolbox online_async_launch.py
```

### 2. Obstacle Avoidance

**Goal**: Navigate without collisions.

**Approach**:
- Subscribe to `/scan`
- Divide field of view into sectors (left, center, right)
- If center has obstacle < threshold: turn
- If clear: move forward

### 3. Terrain Mapping

**Goal**: Create elevation maps for outdoor navigation.

**Use Case**: 3D LIDAR on legged robots (e.g., Boston Dynamics Spot) to detect stairs, slopes, and rough terrain.

---

## Mini Exercise: Simulate and Log LIDAR Data

**Challenge**: Launch a simulated LIDAR in Gazebo and log scan data to a file.

**Requirements**:
1. Launch TurtleBot3 in Gazebo (includes 2D LIDAR)
2. Write a ROS 2 node that subscribes to `/scan`
3. Log the following every second:
   - Timestamp
   - Minimum distance in scan
   - Maximum distance in scan
   - Number of valid points (not inf)
4. Save to CSV file: `lidar_log.csv`

**Hints**:
- Use `time.time()` for timestamp
- Filter out `inf` values before computing min/max
- Use Python's `csv` module to write data

**Extension**: Visualize the logged data with matplotlib (distance vs. time plot)!

---

## Summary

You've learned how LIDAR sensors enable robots to perceive their environment:

✅ **LIDAR measures distance** using Time-of-Flight laser ranging
✅ **2D LIDAR** provides planar scans (LaserScan message)
✅ **3D LIDAR** provides volumetric point clouds (PointCloud2 message)
✅ **ROS 2 integration** via sensor_msgs topics
✅ **RViz visualization** shows laser beams and point clouds
✅ **Applications** include SLAM, obstacle avoidance, terrain mapping

**Next Up**: Explore **Depth Cameras** for visual perception and 3D reconstruction!

Ready to see the world in 3D? Let's continue! 📷🤖
