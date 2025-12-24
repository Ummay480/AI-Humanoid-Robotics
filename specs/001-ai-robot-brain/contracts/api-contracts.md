# AI-Robot Brain API Contracts

## Perception Service API

### Endpoints

#### POST /perception/detect_objects
Detect objects in sensor data

**Request**:
- Content-Type: application/json
- Body:
```json
{
  "sensor_data": {
    "type": "rgb_camera",
    "data": "base64_encoded_image",
    "timestamp": "2025-12-21T10:00:00Z"
  },
  "model_config": {
    "model_name": "object_detection_model",
    "confidence_threshold": 0.7
  }
}
```

**Response**:
- 200: Object detection results
```json
{
  "detection_results": [
    {
      "object_id": "obj_001",
      "class": "person",
      "confidence": 0.95,
      "bbox": {
        "x_min": 100,
        "y_min": 150,
        "x_max": 200,
        "y_max": 300
      },
      "position_3d": {
        "x": 1.5,
        "y": 2.0,
        "z": 0.0
      }
    }
  ],
  "processing_time_ms": 45,
  "timestamp": "2025-12-21T10:00:00Z"
}
```

#### POST /perception/localize_objects
Localize objects in 3D space from sensor data

**Request**:
- Content-Type: application/json
- Body:
```json
{
  "sensor_data": {
    "rgb_data": "base64_encoded_rgb",
    "depth_data": "base64_encoded_depth",
    "camera_info": {
      "intrinsics": [525.0, 525.0, 319.5, 239.5],
      "resolution": [640, 480]
    }
  }
}
```

**Response**:
- 200: 3D localization results
```json
{
  "localized_objects": [
    {
      "object_id": "obj_001",
      "class": "chair",
      "position_3d": {
        "x": 2.1,
        "y": -0.5,
        "z": 0.8
      },
      "orientation": {
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 1.0
      }
    }
  ],
  "timestamp": "2025-12-21T10:00:00Z"
}
```

## VSLAM Service API

### Endpoints

#### POST /vslam/process_frame
Process a visual frame for SLAM

**Request**:
- Content-Type: application/json
- Body:
```json
{
  "frame_data": {
    "image": "base64_encoded_image",
    "timestamp": "2025-12-21T10:00:00Z",
    "camera_id": "camera_001"
  },
  "pose_estimate": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.0,
    "qw": 1.0
  }
}
```

**Response**:
- 200: SLAM processing results
```json
{
  "pose": {
    "position": {
      "x": 1.2,
      "y": 0.8,
      "z": 0.0
    },
    "orientation": {
      "qx": 0.0,
      "qy": 0.0,
      "qz": 0.1,
      "qw": 0.99
    }
  },
  "map_update": {
    "keypoints": [
      {
        "id": "kp_001",
        "position": {"x": 2.1, "y": 1.5, "z": 0.0},
        "descriptor": "feature_descriptor_data"
      }
    ],
    "map_id": "map_001"
  },
  "processing_time_ms": 25,
  "timestamp": "2025-12-21T10:00:00Z"
}
```

#### GET /vslam/map/{map_id}
Retrieve the current map

**Response**:
- 200: Map data
```json
{
  "map_id": "map_001",
  "map_type": "occupancy_grid",
  "resolution": 0.05,
  "origin": {
    "position": {"x": -10.0, "y": -10.0, "z": 0.0},
    "orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
  },
  "data": [0, 0, 0, 100, 100, 0, ...], // 1D array of occupancy values
  "width": 400,
  "height": 400,
  "timestamp": "2025-12-21T10:00:00Z"
}
```

## Path Planning Service API

### Endpoints

#### POST /path_planning/plan_path
Plan a path for bipedal navigation

**Request**:
- Content-Type: application/json
- Body:
```json
{
  "start_pose": {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
  },
  "goal_pose": {
    "position": {"x": 5.0, "y": 3.0, "z": 0.0},
    "orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
  },
  "robot_config": {
    "radius": 0.3,
    "max_step_height": 0.15,
    "foot_size": {"width": 0.2, "length": 0.3}
  },
  "map_id": "map_001"
}
```

**Response**:
- 200: Planned path
```json
{
  "path_id": "path_001",
  "waypoints": [
    {
      "pose": {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
      },
      "time_from_start": 0.0
    },
    {
      "pose": {
        "position": {"x": 1.0, "y": 0.5, "z": 0.0},
        "orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.1, "qw": 0.99}
      },
      "time_from_start": 1.0
    }
  ],
  "planning_time_ms": 120,
  "path_length": 5.8,
  "valid": true,
  "timestamp": "2025-12-21T10:00:00Z"
}
```

#### POST /path_planning/replan_path
Replan path with dynamic obstacles

**Request**:
- Content-Type: application/json
- Body:
```json
{
  "current_path_id": "path_001",
  "dynamic_obstacles": [
    {
      "position": {"x": 2.5, "y": 1.0, "z": 0.0},
      "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
      "size": {"width": 0.5, "length": 0.5, "height": 1.0}
    }
  ]
}
```

**Response**:
- 200: Updated path
```json
{
  "path_id": "path_002",
  "waypoints": [
    {
      "pose": {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
      },
      "time_from_start": 0.0
    }
  ],
  "planning_time_ms": 80,
  "valid": true,
  "timestamp": "2025-12-21T10:00:00Z"
}
```

## Simulation Service API

### Endpoints

#### POST /simulation/create_environment
Create a new simulation environment

**Request**:
- Content-Type: application/json
- Body:
```json
{
  "environment_config": {
    "name": "office_environment",
    "description": "Office environment with desks and chairs",
    "physics_properties": {
      "gravity": [0, 0, -9.81]
    },
    "objects": [
      {
        "name": "desk_001",
        "model": "models/desk.urdf",
        "position": {"x": 2.0, "y": 1.0, "z": 0.0},
        "orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
      }
    ],
    "lighting": {
      "ambient_light": [0.3, 0.3, 0.3],
      "directional_lights": [
        {
          "name": "main_light",
          "direction": {"x": -0.5, "y": -0.5, "z": -1.0},
          "color": [1.0, 1.0, 1.0],
          "intensity": 1.0
        }
      ]
    }
  }
}
```

**Response**:
- 201: Environment created
```json
{
  "environment_id": "env_001",
  "name": "office_environment",
  "status": "created",
  "timestamp": "2025-12-21T10:00:00Z"
}
```

#### POST /simulation/generate_synthetic_data
Generate synthetic training data

**Request**:
- Content-Type: application/json
- Body:
```json
{
  "data_config": {
    "environment_id": "env_001",
    "sensor_config": {
      "camera_resolution": [640, 480],
      "camera_intrinsics": [525.0, 525.0, 319.5, 239.5],
      "sensor_types": ["rgb", "depth", "semantic_segmentation"]
    },
    "domain_randomization": {
      "lighting_range": [0.5, 1.5],
      "texture_randomization": true,
      "object_position_jitter": 0.1
    },
    "sample_count": 1000
  }
}
```

**Response**:
- 200: Data generation started
```json
{
  "job_id": "data_gen_001",
  "estimated_completion_time": 300, // seconds
  "output_path": "/data/synthetic/office_env_001",
  "status": "in_progress",
  "timestamp": "2025-12-21T10:00:00Z"
}
```