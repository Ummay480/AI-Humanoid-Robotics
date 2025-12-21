# Base Sensor Handler Implementation Summary

## Overview
The `base_sensor.py` file implements a comprehensive base sensor handler system for the perception module. It provides a foundation for managing different types of sensors in a ROS2 environment.

## Key Components

### 1. BaseSensorHandler (Abstract Base Class)
- Provides common functionality for all sensor types
- Handles initialization with sensor configuration
- Manages sensor lifecycle (start/stop)
- Implements data validation and processing
- Includes health monitoring capabilities
- Thread-safe callback notification system

### 2. SensorManager
- Centralized management of multiple sensors
- Registration and unregistration of sensors
- Bulk operations (start/stop all sensors)
- Health monitoring across all sensors
- Status reporting for all managed sensors

## Key Features

### Thread Safety
- Added threading locks for safe concurrent access
- Safe callback notification methods

### Health Monitoring
- Health status determination (healthy/degraded/faulty/inactive)
- Timeout detection
- Frequency monitoring
- Health status reporting

### Error Handling
- Comprehensive error callbacks
- Graceful error handling in callbacks
- Logging of errors and issues

### Configuration Management
- Dynamic parameter updates
- Operational parameter management
- Sensor-specific configuration

### Data Management
- Standardized sensor data creation
- Timestamp management
- Data validation
- Metadata enrichment

## Usage Example

```python
from src.perception.sensor_acquisition.base_sensor import BaseSensorHandler, SensorManager
from src.perception.common.data_types import SensorConfig, SensorType

# Create a sensor manager
sensor_manager = SensorManager(node)

# Create a sensor configuration
config = SensorConfig(
    sensor_id="camera_front",
    sensor_type=SensorType.CAMERA,
    topic_name="/camera/image_raw",
    mounting_position={"frame_id": "front_camera_frame"},
    operational_params={"frequency": 30.0, "timeout": 5.0}
)

# Implement a specific sensor handler
class CameraSensorHandler(BaseSensorHandler):
    def _setup_sensor(self):
        # Set up camera subscription
        pass

    def _teardown_sensor(self):
        # Clean up camera subscription
        pass

    def _process_raw_data(self, raw_data):
        # Process camera image data
        return raw_data

# Create and register the sensor
camera_sensor = CameraSensorHandler(node, config)
sensor_manager.register_sensor(camera_sensor)

# Start all sensors
sensor_manager.start_all_sensors()

# Monitor sensor health
sensor_manager.monitor_sensor_health()
```

## Benefits

1. **Standardization**: Provides a consistent interface for all sensor types
2. **Maintainability**: Centralized management and common functionality
3. **Reliability**: Health monitoring and error handling
4. **Scalability**: Easy to add new sensor types by extending the base class
5. **Safety**: Thread-safe operations and proper resource management