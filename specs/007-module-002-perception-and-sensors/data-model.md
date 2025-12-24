# Data Model: Perception and Sensors Module

## Entities

### SensorData
**Description**: Represents raw data from a specific sensor type
**Fields**:
- `id`: Unique identifier for the sensor data instance
- `sensor_type`: Type of sensor (camera, lidar, imu, etc.)
- `timestamp`: Acquisition time with nanosecond precision
- `data`: Raw sensor data payload (varies by sensor type)
- `frame_id`: Coordinate frame identifier for the sensor
- `metadata`: Additional sensor-specific metadata

### ProcessedData
**Description**: Represents sensor data after initial processing
**Fields**:
- `id`: Unique identifier for the processed data instance
- `source_data_id`: Reference to the original SensorData
- `timestamp`: Processing completion time
- `processed_payload`: Processed data (detected objects, features, etc.)
- `confidence_score`: Confidence level of the processing results
- `processing_method`: Algorithm or method used for processing

### FusedData
**Description**: Represents combined information from multiple sensors
**Fields**:
- `id`: Unique identifier for the fused data instance
- `timestamp`: Fusion completion time
- `fused_payload`: Combined sensor information
- `source_sensor_ids`: List of source sensor data IDs used in fusion
- `confidence_score`: Overall confidence of the fused data
- `coordinate_frame`: Reference frame of the fused data

### SensorConfig
**Description**: Represents configuration parameters for each sensor type
**Fields**:
- `sensor_id`: Unique identifier for the sensor
- `sensor_type`: Type of sensor (camera, lidar, imu, etc.)
- `calibration_data`: Calibration parameters and matrices
- `mounting_position`: Position and orientation relative to robot base
- `operational_params`: Sensor-specific operational parameters
- `last_calibration_time`: Timestamp of last calibration

### DetectedObject
**Description**: Represents an object detected by the computer vision system
**Fields**:
- `id`: Unique identifier for the detected object
- `object_type`: Type of object (human, furniture, door, etc.)
- `position`: 3D position relative to robot
- `bounding_box`: 2D/3D bounding box coordinates
- `confidence_score`: Detection confidence level
- `timestamp`: Time of detection

### EnvironmentalMap
**Description**: Represents the 3D environmental map
**Fields**:
- `id`: Unique identifier for the map
- `map_data`: 3D occupancy grid or point cloud data
- `resolution`: Spatial resolution of the map
- `origin_frame`: Reference frame for the map
- `update_timestamp`: Last update time
- `coverage_area`: Bounding box of mapped area

## Relationships

### SensorData → ProcessedData
- One-to-Many: One SensorData can result in multiple ProcessedData instances (e.g., object detection, feature extraction)

### ProcessedData → FusedData
- Many-to-Many: Multiple ProcessedData instances contribute to one FusedData instance

### SensorConfig → SensorData
- One-to-Many: One SensorConfig applies to multiple SensorData instances

### ProcessedData → DetectedObject
- One-to-Many: One ProcessedData instance can contain multiple DetectedObject instances

### DetectedObject → EnvironmentalMap
- Many-to-Many: Multiple DetectedObjects contribute to one EnvironmentalMap

## Validation Rules

### SensorData Validation
- `timestamp` must be within acceptable range (not in future)
- `sensor_type` must be one of predefined types
- `data` must not be empty
- `frame_id` must follow ROS 2 naming conventions

### ProcessedData Validation
- `confidence_score` must be between 0.0 and 1.0
- `processing_method` must be a valid processing algorithm
- `source_data_id` must reference an existing SensorData

### FusedData Validation
- `confidence_score` must be between 0.0 and 1.0
- `source_sensor_ids` must contain at least 2 valid sensor data IDs
- `coordinate_frame` must be a valid coordinate frame

### DetectedObject Validation
- `object_type` must be one of the humanoid robotics object categories
- `confidence_score` must be between 0.0 and 1.0
- `position` must be a valid 3D coordinate

### EnvironmentalMap Validation
- `resolution` must be positive
- `map_data` must not be empty
- `coverage_area` must define a valid bounding box

## State Transitions

### SensorData States
- `ACQUIRING` → `ACQUIRED` → `PROCESSING` → `PROCESSED`
- `ACQUIRING` → `FAILED` (on sensor error)

### Processing Pipeline States
- `QUEUED` → `PROCESSING` → `COMPLETED` | `FAILED`
- `COMPLETED` → `FUSED` (when included in fusion)

### Sensor Configuration States
- `UNCONFIGURED` → `CALIBRATING` → `CONFIGURED` → `CALIBRATED`
- `CONFIGURED` → `RECALIBRATING` → `CALIBRATED`
- `CONFIGURED` → `ERROR` (on calibration failure)