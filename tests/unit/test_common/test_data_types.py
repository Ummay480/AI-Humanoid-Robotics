"""
Unit tests for the data_types module.
"""
import pytest
from datetime import datetime
from src.perception.common.data_types import (
    SensorType, ObjectType, SensorConfig, SensorData,
    UltrasonicData, DetectedObject, ProcessedData,
    FusedData, Position, Orientation, Pose,
    BoundingBox, SensorDataState
)


class TestSensorType:
    """Test SensorType enum."""

    def test_sensor_types(self):
        """Test that all expected sensor types exist."""
        assert SensorType.CAMERA.value == "camera"
        assert SensorType.LIDAR.value == "lidar"
        assert SensorType.IMU.value == "imu"
        assert SensorType.ULTRASONIC.value == "ultrasonic"


class TestObjectType:
    """Test ObjectType enum."""

    def test_object_types(self):
        """Test that all expected object types exist."""
        assert ObjectType.HUMAN.value == "human"
        assert ObjectType.CHAIR.value == "chair"
        assert ObjectType.TABLE.value == "table"
        assert ObjectType.DOOR.value == "door"
        assert ObjectType.STAIR.value == "stair"
        assert ObjectType.OBSTACLE.value == "obstacle"


class TestSensorConfig:
    """Test SensorConfig dataclass."""

    def test_sensor_config_creation(self):
        """Test creating a SensorConfig instance."""
        config = SensorConfig(
            sensor_id="camera_front",
            sensor_type=SensorType.CAMERA,
            topic="/camera_front/image_raw",
            calibration_file="calibration/camera_front.yaml"
        )

        assert config.sensor_id == "camera_front"
        assert config.sensor_type == SensorType.CAMERA
        assert config.topic == "/camera_front/image_raw"
        assert config.calibration_file == "calibration/camera_front.yaml"
        assert config.enabled is True
        assert config.processing_frequency == 10.0


class TestSensorData:
    """Test SensorData dataclass."""

    def test_sensor_data_creation(self):
        """Test creating a SensorData instance."""
        data = SensorData(
            id="test_sensor_001",
            sensor_type=SensorType.CAMERA,
            timestamp=1234567890.0,
            data={"image": "data"},
            frame_id="camera_frame",
            metadata={}
        )

        assert data.id == "test_sensor_001"
        assert data.sensor_type == SensorType.CAMERA
        assert data.timestamp == 1234567890.0
        assert data.data == {"image": "data"}
        assert data.frame_id == "camera_frame"


class TestUltrasonicData:
    """Test UltrasonicData dataclass."""

    def test_ultrasonic_data_creation(self):
        """Test creating an UltrasonicData instance."""
        data = UltrasonicData(
            id="ultrasonic_001",
            sensor_type=SensorType.ULTRASONIC,
            timestamp=1234567890.0,
            distance=1.5,
            confidence=0.9
        )

        assert data.id == "ultrasonic_001"
        assert data.sensor_type == SensorType.ULTRASONIC
        assert data.timestamp == 1234567890.0
        assert data.distance == 1.5
        assert data.confidence == 0.9


class TestDetectedObject:
    """Test DetectedObject dataclass."""

    def test_detected_object_creation(self):
        """Test creating a DetectedObject instance."""
        bbox = BoundingBox(x_min=0.0, y_min=0.0, x_max=10.0, y_max=10.0)

        obj = DetectedObject(
            id="obj_001",
            object_type=ObjectType.HUMAN,
            confidence=0.85,
            bounding_box=bbox,
            position_3d=None
        )

        assert obj.id == "obj_001"
        assert obj.object_type == ObjectType.HUMAN
        assert obj.confidence == 0.85
        assert obj.bounding_box.x_min == 0.0
        assert obj.position_3d is None


class TestPosition:
    """Test Position dataclass."""

    def test_position_creation(self):
        """Test creating a Position instance."""
        pos = Position(x=1.0, y=2.0, z=3.0)
        assert pos.x == 1.0
        assert pos.y == 2.0
        assert pos.z == 3.0


class TestOrientation:
    """Test Orientation dataclass."""

    def test_orientation_creation(self):
        """Test creating an Orientation instance."""
        orient = Orientation(x=0.0, y=0.0, z=0.0, w=1.0)
        assert orient.x == 0.0
        assert orient.y == 0.0
        assert orient.z == 0.0
        assert orient.w == 1.0


class TestPose:
    """Test Pose dataclass."""

    def test_pose_creation(self):
        """Test creating a Pose instance."""
        pos = Position(x=1.0, y=2.0, z=3.0)
        orient = Orientation(x=0.0, y=0.0, z=0.0, w=1.0)
        pose = Pose(position=pos, orientation=orient)

        assert pose.position.x == 1.0
        assert pose.orientation.w == 1.0


class TestBoundingBox:
    """Test BoundingBox dataclass and its properties."""

    def test_bounding_box_properties(self):
        """Test BoundingBox properties."""
        bbox = BoundingBox(x_min=0.0, y_min=0.0, x_max=10.0, y_max=8.0)

        assert bbox.width == 10.0
        assert bbox.height == 8.0
        assert bbox.center_x == 5.0
        assert bbox.center_y == 4.0