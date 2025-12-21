# Building Python AI Agents with rclpy

In this lesson, you'll learn how to **bridge Python AI libraries with ROS 2**, creating intelligent agents that can perceive, reason, and act in the physical world. Welcome to the intersection of AI and robotics!

---

## Why Python for AI + Robotics?

Python has become the **de facto language** for AI and machine learning:

**AI/ML Libraries**:
- **TensorFlow** / **PyTorch**: Deep learning frameworks
- **OpenCV**: Computer vision
- **scikit-learn**: Classical machine learning
- **NumPy** / **Pandas**: Data manipulation
- **Transformers**: Pre-trained language models (GPT, BERT)

**ROS 2 Integration**:
- **rclpy**: Official Python client library for ROS 2
- **cv_bridge**: Converts between ROS images and OpenCV/NumPy arrays
- **tf2_py**: Coordinate frame transformations

**Trade-offs**:
- ✅ **Pros**: Rapid development, rich libraries, easy debugging
- ⚠️ **Cons**: Slower than C++ (but often fast enough with NumPy/GPU acceleration)

**Bottom Line**: Use Python for high-level logic and AI; use C++ for real-time control loops.

---

## Architecture: AI Agent as a ROS 2 Node

**Design Pattern**: Wrap your AI model in a ROS 2 node.

```
┌──────────────────────────────────────────┐
│          AI Agent Node (Python)          │
│                                          │
│  ┌────────────┐      ┌──────────────┐   │
│  │  Subscribe │ ───> │  AI Model    │   │
│  │  (Sensor)  │      │ (TensorFlow) │   │
│  └────────────┘      └──────────────┘   │
│                             │            │
│                             ▼            │
│                      ┌──────────────┐   │
│                      │   Publish    │   │
│                      │  (Decision)  │   │
│                      └──────────────┘   │
└──────────────────────────────────────────┘
           ▲                    │
           │ /camera/image      │ /detected_objects
           │                    ▼
    ┌──────────┐          ┌──────────┐
    │  Camera  │          │ Planner  │
    │   Node   │          │   Node   │
    └──────────┘          └──────────┘
```

**Flow**:
1. Subscribe to sensor data (e.g., camera images)
2. Process with AI model (e.g., object detection)
3. Publish results (e.g., detected objects with bounding boxes)
4. Other nodes use results for decision-making

---

## Example 1: Object Detection Node

Let's build a node that detects objects in camera images using a pre-trained model.

### Prerequisites

```bash
pip install opencv-python
pip install ultralytics  # YOLOv8 for object detection
```

### Object Detector Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class ObjectDetectorNode(Node):
    """AI-powered object detection using YOLOv8."""

    def __init__(self):
        super().__init__('object_detector')

        # Load pre-trained YOLOv8 model
        self.model = YOLO('yolov8n.pt')  # 'n' = nano (fastest)
        self.get_logger().info('YOLOv8 model loaded!')

        # Bridge for ROS <-> OpenCV conversion
        self.bridge = CvBridge()

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detected_objects',
            10
        )

        # Optional: Publish annotated image
        self.annotated_pub = self.create_publisher(
            Image,
            '/camera/annotated',
            10
        )

        self.get_logger().info('Object detector ready!')

    def image_callback(self, msg):
        """Process incoming camera image."""
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run object detection
        results = self.model(cv_image, verbose=False)

        # Parse results
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        for result in results:
            for box in result.boxes:
                detection = Detection2D()

                # Bounding box
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                detection.bbox.center.position.x = (x1 + x2) / 2.0
                detection.bbox.center.position.y = (y1 + y2) / 2.0
                detection.bbox.size_x = x2 - x1
                detection.bbox.size_y = y2 - y1

                # Class and confidence
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(box.cls[0]))
                hypothesis.hypothesis.score = float(box.conf[0])
                detection.results.append(hypothesis)

                detections_msg.detections.append(detection)

                # Draw on image (for visualization)
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f'{self.model.names[int(box.cls[0])]} {box.conf[0]:.2f}'
                cv2.putText(cv_image, label, (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish detections
        self.detection_pub.publish(detections_msg)
        self.get_logger().info(f'Detected {len(detections_msg.detections)} objects')

        # Publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.annotated_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Detector

```bash
# Terminal 1: Start camera (replace with your camera driver)
ros2 run usb_cam usb_cam_node

# Terminal 2: Run object detector
python3 object_detector_node.py

# Terminal 3: Visualize detections
ros2 run rqt_image_view rqt_image_view /camera/annotated
```

---

## Example 2: Obstacle Avoidance with AI

Let's build a simple obstacle avoidance agent using sensor data.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class ObstacleAvoidanceAgent(Node):
    """AI-powered obstacle avoidance using LIDAR data."""

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Subscribe to LIDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters
        self.safe_distance = 0.5  # meters
        self.max_speed = 0.3  # m/s
        self.turn_speed = 0.5  # rad/s

        self.get_logger().info('Obstacle avoidance agent started!')

    def scan_callback(self, msg):
        """Process LIDAR scan and decide motion."""
        # Convert ranges to numpy array
        ranges = np.array(msg.ranges)

        # Replace inf with max_range
        ranges[np.isinf(ranges)] = msg.range_max

        # Split scan into sectors
        num_ranges = len(ranges)
        front = ranges[:num_ranges//3]
        left = ranges[num_ranges//3:2*num_ranges//3]
        right = ranges[2*num_ranges//3:]

        # Find minimum distance in each sector
        min_front = np.min(front)
        min_left = np.min(left)
        min_right = np.min(right)

        # Decision logic
        cmd = Twist()

        if min_front > self.safe_distance:
            # Path clear: move forward
            cmd.linear.x = self.max_speed
            cmd.angular.z = 0.0
            self.get_logger().info('✅ Moving forward')

        elif min_left > min_right:
            # Obstacle ahead: turn left
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
            self.get_logger().warn(f'⚠️  Obstacle at {min_front:.2f}m, turning left')

        else:
            # Obstacle ahead: turn right
            cmd.linear.x = 0.0
            cmd.angular.z = -self.turn_speed
            self.get_logger().warn(f'⚠️  Obstacle at {min_front:.2f}m, turning right')

        # Publish command
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Test in Simulation**:
```bash
# Terminal 1: Launch robot simulator (e.g., TurtleBot3)
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Run obstacle avoidance
python3 obstacle_avoidance_agent.py
```

---

## Handling Real-Time Constraints

Python is interpreted and has a Global Interpreter Lock (GIL), which can cause timing issues.

### Best Practices

**1. Keep Callbacks Fast**
```python
def image_callback(self, msg):
    # ❌ BAD: Heavy computation in callback
    result = heavy_ai_model(msg)  # Blocks other callbacks

    # ✅ GOOD: Offload to separate thread
    self.image_queue.put(msg)  # Process in worker thread
```

**2. Use Threading for AI Processing**
```python
import threading
import queue


class SmartNode(Node):
    def __init__(self):
        super().__init__('smart_node')
        self.image_queue = queue.Queue(maxsize=5)

        # Start worker thread for AI
        self.worker = threading.Thread(target=self.process_images, daemon=True)
        self.worker.start()

        self.image_sub = self.create_subscription(Image, '/camera', self.image_callback, 10)

    def image_callback(self, msg):
        # Quick: just enqueue
        try:
            self.image_queue.put_nowait(msg)
        except queue.Full:
            self.get_logger().warn('Queue full, dropping frame')

    def process_images(self):
        while True:
            msg = self.image_queue.get()  # Blocks until image available
            # Run AI model (heavy computation)
            result = self.ai_model(msg)
            # Publish result
            self.result_pub.publish(result)
```

**3. Leverage GPU Acceleration**
```python
# TensorFlow/PyTorch can offload to GPU
import torch

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = model.to(device)  # Move model to GPU

# Process on GPU
input_tensor = input_tensor.to(device)
output = model(input_tensor)
```

---

## Integrating Pre-Trained Models

### Example: Using Hugging Face Transformers

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transformers import pipeline


class ChatbotNode(Node):
    """AI chatbot using pre-trained language model."""

    def __init__(self):
        super().__init__('chatbot')

        # Load pre-trained model (runs on first call, caches locally)
        self.get_logger().info('Loading language model...')
        self.chatbot = pipeline('conversational', model='microsoft/DialoGPT-medium')
        self.get_logger().info('Model loaded!')

        # Subscribe to user input
        self.input_sub = self.create_subscription(
            String,
            '/user_input',
            self.input_callback,
            10
        )

        # Publish chatbot response
        self.response_pub = self.create_publisher(String, '/chatbot_response', 10)

    def input_callback(self, msg):
        """Generate response to user input."""
        user_text = msg.data
        self.get_logger().info(f'User: {user_text}')

        # Generate response
        response = self.chatbot(user_text)
        bot_text = response[0]['generated_text']

        # Publish
        response_msg = String()
        response_msg.data = bot_text
        self.response_pub.publish(response_msg)

        self.get_logger().info(f'Bot: {bot_text}')


def main(args=None):
    rclpy.init(args=args)
    node = ChatbotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Mini Exercise: Build a Gesture Recognition Node

**Challenge**: Create a node that detects hand gestures from a depth camera and publishes commands.

**Requirements**:
1. Subscribe to `/camera/depth/image_raw` (depth image)
2. Detect hand position using OpenCV (find closest point in depth image)
3. Classify gesture: "wave" (hand moving left-right), "stop" (hand stationary)
4. Publish `std_msgs/String` on `/gesture_command` topic

**Hints**:
- Use `cv_bridge` to convert ROS images to NumPy
- Track hand position over time (store in a list)
- Compute variance: high variance = waving, low variance = stop

**Extension**: Use MediaPipe for more robust hand tracking.

---

## Summary

You've learned how to build intelligent ROS 2 agents in Python:

✅ **Python + ROS 2** combines rich AI libraries with robot communication
✅ **Wrap AI models in nodes** following the sense-think-act pattern
✅ **cv_bridge** converts between ROS and OpenCV image formats
✅ **Threading** helps manage real-time constraints in Python
✅ **Pre-trained models** (YOLO, Transformers) accelerate development
✅ **Publish AI outputs** for other nodes to consume (decoupled architecture)

**Next Up**: You'll learn about URDF, the format for describing your humanoid robot's structure!

Ready to define your robot? Let's continue! 🦾🤖
