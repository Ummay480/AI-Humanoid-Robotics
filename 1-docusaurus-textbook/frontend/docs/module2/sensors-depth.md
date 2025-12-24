# Depth Cameras for Visual Perception

**RGB-D cameras** combine color imaging with depth sensing, enabling robots to perceive 3D structure from visual data. Unlike LIDAR, depth cameras are compact, affordable, and integrate seamlessly with computer vision pipelines. In this lesson, you'll learn how depth cameras work, how to process RGB-D data in ROS 2, and how to build perception applications.

---

## What is a Depth Camera?

**Depth Camera (RGB-D)** = **RGB** (color image) + **D** (depth map)

**Output**:
- **RGB Image**: Standard color camera (640×480, 1920×1080, etc.)
- **Depth Map**: Per-pixel distance to objects (same resolution as RGB)
- **Point Cloud**: 3D coordinates (x, y, z) + color (r, g, b) for each pixel

**Example Devices**:
- **Intel RealSense D435** (stereo depth, IR-based)
- **Microsoft Kinect v2** (Time-of-Flight)
- **Orbbec Astra** (structured light)
- **Luxonis OAK-D** (stereo + AI accelerator)

---

## How Depth Cameras Work

### Method 1: Structured Light

**Principle**: Project infrared pattern, measure distortion.

```
1. IR projector emits dot pattern
2. Pattern deforms on object surfaces
3. IR camera captures deformed pattern
4. Software triangulates depth from distortion
```

**Pros**: High accuracy indoors (±1mm at 1m)
**Cons**: Fails in bright sunlight (IR washed out)

**Example**: Kinect v1, Orbbec Astra

### Method 2: Time-of-Flight (ToF)

**Principle**: Measure round-trip time of modulated IR light.

```
1. Emit modulated IR pulse
2. Measure phase shift of reflected light
3. Calculate depth: d = (c × Δt) / 2
```

**Pros**: Works in sunlight, fast (60+ FPS)
**Cons**: Lower resolution, limited range (0.3m - 5m)

**Example**: Kinect v2, PMD CamBoard

### Method 3: Stereo Vision

**Principle**: Two cameras (like human eyes) compute depth from disparity.

```
1. Capture images from left and right cameras
2. Match corresponding pixels (stereo matching)
3. Compute disparity (pixel offset)
4. Depth = (baseline × focal_length) / disparity
```

**Pros**: No active illumination (works outdoors), scalable range
**Cons**: Requires texture (fails on blank walls), computationally intensive

**Example**: Intel RealSense D435, ZED 2

---

## RGB-D Data in ROS 2

### Message Types

**1. RGB Image**:
```python
# sensor_msgs/Image
header:
  stamp: time
  frame_id: "camera_rgb_optical_frame"
height: 480
width: 640
encoding: "rgb8"  # or "bgr8"
data: [byte array]  # Raw pixel data
```

**2. Depth Image**:
```python
# sensor_msgs/Image
header:
  stamp: time
  frame_id: "camera_depth_optical_frame"
height: 480
width: 640
encoding: "16UC1"  # 16-bit unsigned int (mm) or "32FC1" (meters)
data: [byte array]  # Depth values
```

**3. Camera Info**:
```python
# sensor_msgs/CameraInfo
header: ...
height: 480
width: 640
distortion_model: "plumb_bob"
D: [k1, k2, t1, t2, k3]  # Distortion coefficients
K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]  # Intrinsic matrix
P: [fx', 0, cx', Tx, 0, fy', cy', Ty, 0, 0, 1, 0]  # Projection matrix
```

**4. Point Cloud**:
```python
# sensor_msgs/PointCloud2 (generated from RGB + Depth)
fields:
  - name: "x", "y", "z", "rgb"
data: [(x1,y1,z1,rgb1), (x2,y2,z2,rgb2), ...]
```

---

## Example: Processing Depth Images

### Install RealSense ROS 2 Driver

```bash
sudo apt install ros-humble-realsense2-camera
```

### Launch RealSense Camera

```bash
ros2 launch realsense2_camera rs_launch.py
```

**Published Topics**:
- `/camera/color/image_raw` (RGB)
- `/camera/depth/image_rect_raw` (depth)
- `/camera/depth/color/points` (point cloud)

### Depth Image Processing Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthAnalyzer(Node):
    """Analyzes depth images to detect close objects."""

    def __init__(self):
        super().__init__('depth_analyzer')

        self.bridge = CvBridge()

        # Subscribe to depth image
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.close_threshold = 500  # mm (0.5 meters)
        self.get_logger().info('Depth analyzer started!')

    def depth_callback(self, msg):
        """Process depth image."""
        # Convert ROS Image to OpenCV (depth in mm)
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Replace 0 (invalid) with max value
        depth_image[depth_image == 0] = np.max(depth_image)

        # Find minimum depth
        min_depth = np.min(depth_image)
        min_location = np.unravel_index(np.argmin(depth_image), depth_image.shape)

        # Alert if object too close
        if min_depth < self.close_threshold:
            self.get_logger().warn(
                f'⚠️  Close object at {min_depth}mm, '
                f'pixel: {min_location}'
            )
        else:
            self.get_logger().info(f'✅ Clear (closest: {min_depth}mm)')

        # Visualize (normalize for display)
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )
        cv2.imshow('Depth Image', depth_colormap)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DepthAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Run**:
```bash
# Terminal 1: Launch RealSense
ros2 launch realsense2_camera rs_launch.py

# Terminal 2: Run analyzer
python3 depth_analyzer.py
```

---

## Point Cloud Processing

### Converting Depth to Point Cloud

**Manual Conversion** (if you have depth + camera intrinsics):
```python
import numpy as np

def depth_to_pointcloud(depth_image, camera_matrix):
    """
    Convert depth image to 3D point cloud.

    Args:
        depth_image: (H, W) array of depths in meters
        camera_matrix: 3x3 intrinsic matrix [fx, 0, cx; 0, fy, cy; 0, 0, 1]

    Returns:
        (N, 3) array of (x, y, z) points
    """
    h, w = depth_image.shape
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]

    # Create pixel grid
    u, v = np.meshgrid(np.arange(w), np.arange(h))

    # Compute 3D coordinates
    z = depth_image
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # Stack and filter invalid points
    points = np.stack([x, y, z], axis=-1)
    points = points[z > 0]  # Remove invalid depths

    return points
```

### Point Cloud Filtering

**Use Case**: Remove background, extract objects.

```python
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_filter')

        self.pc_sub = self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.pc_callback, 10
        )

        self.filtered_pub = self.create_publisher(
            PointCloud2, '/filtered_points', 10
        )

    def pc_callback(self, msg):
        # Read point cloud
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        # Filter: keep points within 0.5m - 2m
        filtered = [p for p in points if 0.5 < p[2] < 2.0]

        # Publish filtered cloud
        filtered_msg = pc2.create_cloud_xyz32(msg.header, filtered)
        self.filtered_pub.publish(filtered_msg)

        self.get_logger().info(f'Filtered {len(points)} → {len(filtered)} points')
```

---

## Applications in Robotics

### 1. Object Detection and 6D Pose Estimation

**Goal**: Detect objects and estimate their 3D position + orientation.

**Approach**:
1. Segment RGB image (e.g., YOLO)
2. Extract corresponding depth region
3. Compute 3D bounding box or pose (ICP, PnP)

### 2. Obstacle Avoidance

**Goal**: Navigate around obstacles using depth.

**Approach**:
- Divide depth image into grid
- Compute average depth per cell
- Cells with depth < threshold = obstacle

### 3. Hand Tracking and Gesture Recognition

**Goal**: Detect hand position and gestures for HRI.

**Tools**: MediaPipe, OpenPose on RGB + depth for 3D skeleton

### 4. 3D Reconstruction

**Goal**: Build 3D model of environment.

**Tools**: Open3D, CloudCompare, or ROS packages like `rtabmap_ros`

---

## Mini Exercise: Measure Object Distance

**Challenge**: Use a depth camera to measure the distance to the closest object.

**Requirements**:
1. Launch RealSense (or simulate in Gazebo)
2. Subscribe to `/camera/depth/image_rect_raw`
3. Find the pixel with minimum depth
4. Convert pixel coordinates to 3D position (x, y, z)
5. Publish as `geometry_msgs/PointStamped` on `/closest_point`
6. Visualize in RViz

**Hints**:
- Use `cv_bridge` to convert depth image
- Use camera intrinsics from `/camera/depth/camera_info`
- Formula: `x = (u - cx) * z / fx`, `y = (v - cy) * z / fy`

**Extension**: Draw a bounding box around the closest object in the RGB image!

---

## Summary

You've learned how depth cameras enable rich 3D perception for robots:

✅ **RGB-D cameras** combine color and depth in one sensor
✅ **Three technologies**: Structured light, Time-of-Flight, Stereo vision
✅ **ROS 2 messages**: sensor_msgs/Image (RGB, depth), PointCloud2
✅ **cv_bridge** converts images for OpenCV processing
✅ **Point clouds** provide colored 3D representations
✅ **Applications**: Object detection, obstacle avoidance, hand tracking, 3D reconstruction

**Congratulations!** You've completed Module 2: Digital Twins & Simulation! 🎉

**Next Module**: Advanced topics in control, planning, and AI for humanoid robots!

Ready to build intelligence? Let's continue! 🧠🤖
