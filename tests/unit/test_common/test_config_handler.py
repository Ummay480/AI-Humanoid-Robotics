"""
Unit tests for the config_handler module.
"""
import tempfile
import os
from pathlib import Path
from src.perception.common.config_handler import SensorConfigHandler
from src.perception.common.data_types import SensorType, SensorConfig, SensorConfigState


class TestConfigHandler:
    """Test sensor configuration handler."""

    def setup_method(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.handler = SensorConfigHandler(config_dir=self.temp_dir)

    def teardown_method(self):
        """Clean up test fixtures."""
        # Remove all files in the temp directory
        for file_path in Path(self.temp_dir).glob("*"):
            file_path.unlink()
        Path(self.temp_dir).rmdir()

    def test_create_default_config(self):
        """Test creating a default sensor configuration."""
        config = self.handler.create_default_config("test_camera", SensorType.CAMERA)

        assert config.sensor_id == "test_camera"
        assert config.sensor_type == SensorType.CAMERA
        assert config.topic == "/test_camera/data"
        assert config.calibration_file == "calibration/test_camera.yaml"
        assert config.enabled is True
        assert config.processing_frequency == 10.0
        assert config.state == SensorConfigState.UNCONFIGURED

    def test_save_and_load_sensor_config(self):
        """Test saving and loading a sensor configuration."""
        config = SensorConfig(
            sensor_id="test_lidar",
            sensor_type=SensorType.LIDAR,
            topic="/test_lidar/scan",
            calibration_file="calibration/test_lidar.yaml",
            enabled=True,
            processing_frequency=10.0,
            parameters={"range_max": 30.0}
        )

        # Save the config
        result = self.handler.save_sensor_config(config)
        assert result is True

        # Load the config
        loaded_config = self.handler.load_sensor_config("test_lidar")
        assert loaded_config is not None
        assert loaded_config.sensor_id == "test_lidar"
        assert loaded_config.sensor_type == SensorType.LIDAR
        assert loaded_config.topic == "/test_lidar/scan"
        assert loaded_config.calibration_file == "calibration/test_lidar.yaml"
        assert loaded_config.enabled is True
        assert loaded_config.processing_frequency == 10.0
        assert loaded_config.parameters == {"range_max": 30.0}

    def test_get_sensor_config(self):
        """Test getting a sensor configuration."""
        config = SensorConfig(
            sensor_id="test_imu",
            sensor_type=SensorType.IMU,
            topic="/test_imu/data",
            calibration_file="calibration/test_imu.yaml",
            enabled=True,
            processing_frequency=100.0,
            parameters={"frequency": 100.0}
        )

        # Save the config first
        self.handler.save_sensor_config(config)

        # Get the config
        retrieved_config = self.handler.get_sensor_config("test_imu")
        assert retrieved_config is not None
        assert retrieved_config.sensor_id == "test_imu"
        assert retrieved_config.sensor_type == SensorType.IMU

    def test_update_sensor_config(self):
        """Test updating a sensor configuration."""
        config = SensorConfig(
            sensor_id="test_camera",
            sensor_type=SensorType.CAMERA,
            topic="/test_camera/image_raw",
            calibration_file="calibration/test_camera.yaml",
            enabled=True,
            processing_frequency=30.0,
            parameters={"resolution": [640, 480]}
        )

        # Save the initial config
        result = self.handler.update_sensor_config(config)
        assert result is True

        # Update the config
        config.enabled = False
        config.processing_frequency = 15.0
        config.parameters = {"resolution": [1280, 720]}

        result = self.handler.update_sensor_config(config)
        assert result is True

        # Verify the update
        updated_config = self.handler.get_sensor_config("test_camera")
        assert updated_config.enabled is False
        assert updated_config.processing_frequency == 15.0
        assert updated_config.parameters == {"resolution": [1280, 720]}

    def test_validate_sensor_config(self):
        """Test sensor configuration validation."""
        valid_config = SensorConfig(
            sensor_id="valid_sensor",
            sensor_type=SensorType.CAMERA,
            topic="/valid_sensor/data",
            calibration_file="calibration/valid_sensor.yaml",
            enabled=True,
            processing_frequency=10.0,
            parameters={}
        )

        invalid_config = SensorConfig(
            sensor_id="",  # Empty ID
            sensor_type=SensorType.CAMERA,
            topic="/invalid_sensor/data",
            calibration_file="calibration/invalid_sensor.yaml",
            enabled=True,
            processing_frequency=10.0,
            parameters={}
        )

        assert self.handler.validate_sensor_config(valid_config) is True
        assert self.handler.validate_sensor_config(invalid_config) is False

    def test_list_sensor_configs(self):
        """Test listing sensor configurations."""
        # Create and save a few configs
        config1 = SensorConfig(
            sensor_id="camera_front",
            sensor_type=SensorType.CAMERA,
            topic="/camera_front/image_raw",
            calibration_file="calibration/camera_front.yaml",
            enabled=True,
            processing_frequency=30.0,
            parameters={}
        )
        config2 = SensorConfig(
            sensor_id="lidar_3d",
            sensor_type=SensorType.LIDAR,
            topic="/lidar_3d/scan",
            calibration_file="calibration/lidar_3d.yaml",
            enabled=True,
            processing_frequency=10.0,
            parameters={}
        )

        self.handler.save_sensor_config(config1)
        self.handler.save_sensor_config(config2)

        # List configs
        config_ids = self.handler.list_sensor_configs()
        assert len(config_ids) == 2
        assert "camera_front" in config_ids
        assert "lidar_3d" in config_ids

    def test_set_calibration_data(self):
        """Test setting calibration data."""
        result = self.handler.set_calibration_data("test_sensor", "calibration/new_cal.yaml")
        assert result is True

        config = self.handler.get_sensor_config("test_sensor")
        assert config is not None
        assert config.calibration_file == "calibration/new_cal.yaml"
        assert config.state == SensorConfigState.CALIBRATED

    def test_get_calibration_file_path(self):
        """Test getting calibration file path."""
        self.handler.set_calibration_data("test_sensor", "calibration/test.yaml")

        path = self.handler.get_calibration_file_path("test_sensor")
        assert path == "calibration/test.yaml"

    def test_get_default_operational_params(self):
        """Test getting default operational parameters."""
        camera_defaults = self.handler.get_default_operational_params(SensorType.CAMERA)
        assert "frequency" in camera_defaults
        assert camera_defaults["frequency"] == 30.0

        lidar_defaults = self.handler.get_default_operational_params(SensorType.LIDAR)
        assert "range_max" in lidar_defaults
        assert lidar_defaults["range_max"] == 30.0

        ultrasonic_defaults = self.handler.get_default_operational_params(SensorType.ULTRASONIC)
        assert "frequency" in ultrasonic_defaults
        assert ultrasonic_defaults["frequency"] == 20.0