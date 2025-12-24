# Perception Module - Code Examples

Practical examples demonstrating the Perception and Sensors module capabilities.

## Overview

This directory contains executable examples that demonstrate key features of the perception system:

1. **simple_detection.py** - Basic object detection using YOLO
2. **sensor_fusion_demo.py** - Multi-sensor fusion with Kalman filtering
3. **mapping_demo.py** - 3D environmental mapping and point cloud processing
4. **performance_monitoring.py** - Performance monitoring and optimization

## Prerequisites

```bash
# Ensure perception module is built
cd /path/to/hackathon
colcon build --packages-select perception
source install/setup.bash

# Install Python dependencies
pip install -r requirements.txt
```

## Running the Examples

### 1. Simple Object Detection

Minimal example showing YOLO-based object detection.

```bash
# Make executable
chmod +x examples/perception/simple_detection.py

# Run as ROS 2 node
ros2 run python examples/perception/simple_detection.py

# Or with parameters
ros2 run python examples/perception/simple_detection.py \
    --ros-args \
    -p model_path:=models/yolov8n.pt \
    -p confidence_threshold:=0.5 \
    -p camera_topic:=/camera_front/image_raw
```

**Expected Output:**
- Subscribes to camera topic
- Publishes detected objects to `/detections`
- Logs detection count every second

**Test with fake camera:**
```bash
# Terminal 1: Run detection node
ros2 run python examples/perception/simple_detection.py

# Terminal 2: Publish test images
ros2 run image_tools cam2image --ros-args -r /image:=/camera_front/image_raw

# Terminal 3: Monitor detections
ros2 topic echo /detections
```

### 2. Sensor Fusion Demo

Demonstrates multi-sensor Kalman filtering for position tracking.

```bash
# Make executable
chmod +x examples/perception/sensor_fusion_demo.py

# Run standalone (no ROS required)
python3 examples/perception/sensor_fusion_demo.py
```

**Expected Output:**
```
==============================================================
Sensor Fusion Demonstration
==============================================================

1. Setting up Kalman filter and fusion engine...
   Initialized at position: [0. 0. 0.]

2. Simulating object motion and sensor measurements...

   Sensor Specifications:
   - camera: noise=15cm, confidence=70%, rate=30Hz
   - lidar: noise=5cm, confidence=90%, rate=10Hz
   - imu_derived: noise=25cm, confidence=50%, rate=100Hz

   Simulating 5.0s of motion at velocity [1.  0.5 0. ]

3. Running fusion loop...

   t=0.0s: True=[0. 0. 0.], Filtered=[0.007 0.003 0.001], Error=0.008m
   t=1.0s: True=[1.  0.5 0. ], Filtered=[0.998 0.497 0.002], Error=0.005m
   ...

4. Results Summary:
==============================================================

   Position Tracking Accuracy:
   - Average error: 0.52 cm
   - Maximum error: 1.23 cm
   - Final error:   0.48 cm

   Velocity Estimation:
   - True velocity:      [1.  0.5 0. ]
   - Estimated velocity: [0.999 0.501 0.001]
   - Velocity error:     0.002 m/s
```

**What it demonstrates:**
- Multi-sensor Kalman filtering
- Sensor fusion with confidence weighting
- Position and velocity estimation
- Uncertainty reduction through fusion

### 3. 3D Mapping Demo

Shows 3D occupancy grid mapping and point cloud processing.

```bash
# Make executable
chmod +x examples/perception/mapping_demo.py

# Run standalone
python3 examples/perception/mapping_demo.py
```

**Expected Output:**
```
======================================================================
3D Environmental Mapping Demonstration
======================================================================

1. Creating 3D Occupancy Grid...
   Resolution: 5 cm
   Grid size: 200 x 200 x 60 voxels
   Coverage: 10.0m x 10.0m x 3.0m

2. Generating synthetic room point cloud...
   Generated 8,200 points

3. Processing point cloud...
   - Downsampling...
     Reduced from 8,200 to 1,854 points (12.3ms)
   - Removing outliers...
     Removed 47 outliers (45.2ms)
   - Extracting ground plane...
     Ground plane: [0.001 -0.002 1.0] (normal), 0.01 (offset)
     Ground points: 612, Non-ground: 1,195 (78.5ms)
   - Clustering obstacles...
     Found 3 obstacle clusters (23.1ms)

4. Updating occupancy grid...
   Updated grid with 1,807 points (156.4ms)

...

9. Performance Summary:
   - Total processing time: 315.5ms
     * Downsampling:  12.3ms
     * Outlier removal: 45.2ms
     * Ground extraction: 78.5ms
     * Clustering: 23.1ms
     * Grid update: 156.4ms

   ✗ Exceeds real-time requirement (target: <20ms)
```

**What it demonstrates:**
- Point cloud downsampling
- Outlier removal
- Ground plane extraction (RANSAC)
- Obstacle clustering (DBSCAN)
- 3D occupancy grid mapping
- Ray tracing for free space
- Performance profiling

**Note:** Full point cloud processing may exceed 20ms, but individual sensor updates are optimized to meet real-time requirements.

### 4. Performance Monitoring

Comprehensive performance monitoring and optimization demonstration.

```bash
# Make executable
chmod +x examples/perception/performance_monitoring.py

# Run standalone
python3 examples/perception/performance_monitoring.py
```

**Expected Output:**
```
======================================================================
Performance Monitoring Demonstration
======================================================================

1. Setting up monitoring infrastructure...
   ✓ Logger configured
   ✓ Performance monitor ready
   ✓ Health monitor ready
   ✓ Performance optimizer ready

2. Simulating perception pipeline (100 iterations)...
   Processed 25/100 iterations...
   Processed 50/100 iterations...
   Processed 75/100 iterations...
   Processed 100/100 iterations...
   ✓ Simulation complete

3. Performance Metrics Analysis:
======================================================================

   Latency Metrics:
   sensor_acquisition:
      Mean:   5.03 ms
      Std:    0.12 ms
      ...
      Status: ✓ Meeting <20ms requirement

   object_detection:
      Mean:   12.34 ms
      ...
      Status: ✓ Meeting <20ms requirement

...

5. Optimization Analysis:
======================================================================

   Optimization Suggestions:
   - High CPU usage (92.3%). Consider reducing processing threads...

8. Summary:
======================================================================

   Total operations processed: 300
   Average pipeline latency: 6.79ms
   System uptime: 12.45s
   Overall health: OK

   ✓ System performing within specifications

Logs saved to: logs/perf_demo.log
Metrics saved to: metrics/demo_metrics.json
```

**What it demonstrates:**
- Structured logging
- Performance metrics collection
- Health monitoring
- Latency optimization
- Memory optimization
- Resource management
- Metrics export (JSON/CSV)

## Example Usage Patterns

### Pattern 1: ROS 2 Integration

```python
import rclpy
from rclpy.node import Node
from src.perception.computer_vision.object_detector import YOLODetector

class MyPerceptionNode(Node):
    def __init__(self):
        super().__init__('my_perception')
        self.detector = YOLODetector(model_path='models/yolov8n.pt')

    # ... add subscribers, publishers, callbacks
```

### Pattern 2: Standalone Processing

```python
from src.perception.sensor_fusion.kalman_filter import KalmanFilter
import numpy as np

# Create filter
kf = KalmanFilter(dim_x=6, dim_z=3)
kf.initialize(np.array([0, 0, 0]))

# Process measurements
for measurement in measurements:
    kf.predict()
    kf.update(measurement)
    state = kf.get_state()
```

### Pattern 3: Performance Monitoring

```python
from src.perception.monitoring.logger import TimingContext, PerformanceMonitor

monitor = PerformanceMonitor()

with TimingContext('my_operation', monitor):
    # Your code here
    process_data()

stats = monitor.get_statistics('my_operation')
print(f"Average time: {stats['mean']:.2f}ms")
```

## Customizing Examples

### Modify Detection Classes

Edit `simple_detection.py`:
```python
self.detector = YOLODetector(
    model_path='models/yolov8n.pt',
    confidence_threshold=0.5,
    target_classes=['person', 'car', 'bicycle']  # Add your classes
)
```

### Adjust Fusion Weights

Edit `sensor_fusion_demo.py`:
```python
sensor_specs = {
    'camera': {
        'noise_std': 0.15,
        'confidence': 0.8,  # Increase camera confidence
        'update_rate': 30
    },
    # ...
}
```

### Change Grid Resolution

Edit `mapping_demo.py`:
```python
grid = OccupancyGrid3D(
    resolution=0.02,  # 2cm instead of 5cm
    size=(500, 500, 150),  # Larger grid
    origin=np.array([-5.0, -5.0, 0.0])
)
```

## Troubleshooting

### Import Errors

```bash
# Ensure PYTHONPATH includes project root
export PYTHONPATH=$PYTHONPATH:/path/to/hackathon

# Or run from project root
cd /path/to/hackathon
python3 examples/perception/sensor_fusion_demo.py
```

### YOLO Model Not Found

```bash
# Download YOLOv8 model
mkdir -p models
cd models
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

### ROS 2 Not Found

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace
source install/setup.bash
```

### Permission Denied

```bash
# Make scripts executable
chmod +x examples/perception/*.py
```

## Next Steps

- Modify examples for your specific use case
- Integrate with full perception pipeline
- Add custom sensors or detectors
- Implement custom fusion algorithms
- Create visualization tools

## Related Documentation

- [Perception Module README](../../src/perception/README.md)
- [Quickstart Guide](../../docs/perception_quickstart.md)
- [Module 002 Specification](../../specs/007-module-002-perception-and-sensors/spec.md)
- [Integration Tests](../../tests/integration/test_perception_pipeline.py)

## Support

For questions or issues:
- Check the main [README](../../src/perception/README.md)
- Review [integration tests](../../tests/integration/)
- File an issue on GitHub
