# Quickstart: Perception and Sensors Module

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Python 3.10 or higher
- At least 8GB RAM (16GB recommended for full sensor processing)
- Compatible GPU for accelerated computer vision (optional but recommended)

### Required Dependencies
```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2 python3-rosinstall python3-vcstool

# Install Python dependencies
pip3 install opencv-python numpy scipy pyquaternion transforms3d ultralytics

# Install additional ROS 2 packages
sudo apt install ros-humble-vision-opencv ros-humble-cv-bridge
sudo apt install ros-humble-image-transport ros-humble-compressed-image-transport
sudo apt install ros-humble-robot-localization ros-humble-navigation2
sudo apt install ros-humble-octomap ros-humble-octomap-ros
```

## Setup

### 1. Clone and Build the Perception Package
```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the perception package (or copy your implementation)
# Assuming the package is in src/perception directory

# Build the workspace
cd ~/ros2_ws
colcon build --packages-select perception
source install/setup.bash
```

### 2. Configure Sensors
Create sensor configuration files in the `config/` directory:

**config/perception_params.yaml**:
```yaml
sensors:
  camera:
    frequency: 30.0
    resolution: [640, 480]
    topic: "/camera_front/image_raw"

  lidar:
    frequency: 10.0
    range_max: 30.0
    topic: "/lidar_3d/scan"

  imu:
    frequency: 100.0
    topic: "/imu/data"

object_detection:
  model_path: "models/yolov8n.pt"
  confidence_threshold: 0.5
  target_classes: ["human", "chair", "table", "door", "stair", "obstacle"]

fusion:
  sensor_weights:
    camera: 0.3
    lidar: 0.5
    imu: 0.2

mapping:
  resolution: 0.05  # 5cm
  max_range: 10.0
  update_frequency: 5.0
```

**config/camera_front.yaml**:
```yaml
sensor_id: "camera_front"
sensor_type: "camera"
topic: "/camera_front/image_raw"
calibration_file: "calibration/camera_front.yaml"
enabled: true
processing_frequency: 30.0
```

**config/lidar_3d.yaml**:
```yaml
sensor_id: "lidar_3d"
sensor_type: "lidar"
topic: "/lidar_3d/scan"
calibration_file: "calibration/lidar_3d.yaml"
enabled: true
processing_frequency: 10.0
```

**config/imu_main.yaml**:
```yaml
sensor_id: "imu_main"
sensor_type: "imu"
topic: "/imu/data"
calibration_file: "calibration/imu_main.yaml"
enabled: true
processing_frequency: 100.0
```

### 3. Launch the Perception System
```bash
# Launch the complete perception pipeline
ros2 launch perception perception_pipeline.launch.py

# With visualization enabled
ros2 launch perception perception_pipeline.launch.py enable_visualization:=true

# With custom model path
ros2 launch perception perception_pipeline.launch.py model_path:=/path/to/model.pt

# With simulation time
ros2 launch perception perception_pipeline.launch.py use_sim_time:=true
```

## Basic Usage Examples

### Example 1: Basic Perception Pipeline
```bash
# Terminal 1: Launch the perception system
ros2 launch perception perception_pipeline.launch.py

# Terminal 2: Monitor detected objects
ros2 topic echo /perception/objects_detected

# Terminal 3: Monitor environmental map
ros2 topic echo /perception/environmental_map
```

### Example 2: Simulated Environment
```bash
# Terminal 1: Launch Gazebo simulation with robot
ros2 launch your_robot_gazebo robot.launch.py

# Terminal 2: Launch perception system with simulation time
ros2 launch perception perception_pipeline.launch.py use_sim_time:=true

# Terminal 3: Send commands to robot to move around
ros2 run your_robot_control move_to_goal.py --x 1.0 --y 2.0
```

### Example 3: Real Hardware Setup
```bash
# Terminal 1: Launch camera driver (example with realsense)
ros2 launch realsense2_camera rs_launch.py

# Terminal 2: Launch LIDAR driver (example with velodyne)
ros2 launch velodyne_driver velodyne_driver_nodelet.launch.py

# Terminal 3: Launch perception system
ros2 launch perception perception_pipeline.launch.py

# Terminal 4: Monitor system status
ros2 topic echo /perception/system_status
```

### Example 4: Custom Configuration
```bash
# Launch with custom configuration directory
ros2 launch perception perception_pipeline.launch.py config_dir:=/custom/config/path

# Launch with specific camera topics
ros2 launch perception perception_pipeline.launch.py camera_topics:="['/camera_left/image_raw', '/camera_right/image_raw']"

# Launch with visualization and custom model
ros2 launch perception perception_pipeline.launch.py enable_visualization:=true model_path:=/custom/models/custom_model.pt
```

## Advanced Usage

### Monitoring System Performance
```bash
# Check node status
ros2 node info /perception/sensor_acquisition_node
ros2 node info /perception/object_detection_node
ros2 node info /perception/sensor_fusion_node

# Monitor topic frequencies
ros2 topic hz /perception/objects_detected
ros2 topic hz /camera_front/image_raw

# Monitor system resources
ros2 run perception sensor_monitor_node
```

### Parameter Tuning
```bash
# Change detection confidence threshold at runtime
ros2 param set /perception/object_detection_node object_detection.confidence_threshold 0.7

# Adjust mapping resolution
ros2 param set /perception/sensor_fusion_node mapping.resolution 0.025

# Enable/disable specific sensors
ros2 param set /perception/sensor_acquisition_node sensors.camera.enabled false
```

### Data Collection and Analysis
```bash
# Record sensor data for analysis
ros2 bag record /camera_front/image_raw /lidar_3d/scan /imu/data /perception/objects_detected

# Playback recorded data
ros2 bag play your_recording_bag --clock

# Analyze system performance
ros2 run perception performance_analyzer_node
```

## API Usage Examples

### Subscribing to Perception Data in Your Own Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Perception data is published as JSON strings

class PerceptionUser(Node):
    def __init__(self):
        super().__init__('perception_user')

        # Subscribe to detected objects
        self.objects_sub = self.create_subscription(
            String,
            '/perception/objects_detected',
            self.objects_callback,
            10
        )

        # Subscribe to environmental map
        self.map_sub = self.create_subscription(
            String,
            '/perception/environmental_map',
            self.map_callback,
            10
        )

    def objects_callback(self, msg):
        # Process detected objects (JSON format)
        import json
        data = json.loads(msg.data)
        self.get_logger().info(f'Detected objects: {data}')

    def map_callback(self, msg):
        # Process environmental map (JSON format)
        import json
        data = json.loads(msg.data)
        self.get_logger().info(f'Environmental map received: {len(data)} points')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionUser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Calls (if available)
```bash
# Request immediate object detection
ros2 service call /perception/object_detection/trigger_detection std_srvs/Trigger

# Request map update
ros2 service call /perception/mapping/update_map std_srvs/Trigger

# Calibrate sensors
ros2 service call /perception/calibration/calibrate std_srvs/Trigger
```

## Testing

### Unit Tests
```bash
# Run unit tests for individual components
cd ~/ros2_ws
source install/setup.bash
python3 -m pytest tests/unit/test_sensor_acquisition/
python3 -m pytest tests/unit/test_computer_vision/
python3 -m pytest tests/unit/test_sensor_fusion/
```

### Integration Tests
```bash
# Run complete pipeline tests
python3 -m pytest tests/integration/test_perception_pipeline.py

# Run performance tests
python3 -m pytest tests/integration/test_perception_pipeline.py::TestPerformanceRequirements

# Run with coverage
python3 -m pytest tests/integration/ --cov=perception --cov-report=html
```

## Troubleshooting

### Common Issues

1. **High Latency (>20ms)**
   - Check CPU usage and consider reducing sensor frequencies:
     ```bash
     ros2 param set /perception/sensor_acquisition_node sensors.camera.frequency 15.0
     ```
   - Verify QoS settings are configured for real-time performance
   - Consider using multi-threaded ROS 2 executor
   - Reduce image resolution in configuration

2. **Object Detection Accuracy <85%**
   - Verify lighting conditions meet requirements
   - Check camera calibration files
   - Adjust confidence threshold:
     ```bash
     ros2 param set /perception/object_detection_node object_detection.confidence_threshold 0.3
     ```
   - Consider retraining the model with domain-specific data

3. **Mapping Inaccuracy >5cm**
   - Verify sensor calibration
   - Check for synchronization issues between sensors
   - Ensure sufficient feature points in environment
   - Adjust fusion weights:
     ```bash
     ros2 param set /perception/sensor_fusion_node fusion.sensor_weights "{camera: 0.4, lidar: 0.4, imu: 0.2}"
     ```

4. **Sensor Connection Issues**
   - Verify sensor drivers are installed and running:
     ```bash
     ros2 topic list | grep -E "(camera|lidar|imu)"
     ros2 topic hz /camera_front/image_raw
     ```
   - Check topic names match configuration
   - Confirm proper permissions for sensor access
   - Use `ros2 doctor` to diagnose ROS 2 issues

5. **Memory Usage Too High**
   - Reduce the number of sensors being processed
   - Lower image resolution in configuration
   - Reduce processing frequency
   - Use lightweight YOLO model variants (e.g., yolov8n instead of yolov8x)

6. **No Objects Being Detected**
   - Verify camera is publishing images:
     ```bash
     ros2 topic echo /camera_front/image_raw --field data --field header
     ```
   - Check model file exists and is accessible
   - Verify target classes match your model's classes
   - Lower confidence threshold temporarily to test

### Debugging Commands
```bash
# Check all running nodes
ros2 node list

# Check all topics
ros2 topic list

# Check topic connections
ros2 topic info /camera_front/image_raw

# Check system status
ros2 run perception system_monitor_node

# Enable debug logging
ros2 launch perception perception_pipeline.launch.py log_level:=debug
```

### Performance Monitoring
```bash
# Monitor node performance
ros2 run performance_test rosbag_performance_tester

# Check system resources
htop
nvidia-smi  # if using GPU

# Monitor specific topics
ros2 topic hz /perception/objects_detected
```