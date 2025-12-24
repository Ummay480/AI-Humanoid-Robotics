"""
Unit tests for BaseSensorHandler.
"""
import pytest
import threading
from unittest.mock import Mock, MagicMock

from src.perception.sensor_acquisition.base_sensor import BaseSensorHandler
from src.perception.common.data_types import SensorConfig, SensorType, SensorConfigState


class ConcreteSensorHandler(BaseSensorHandler):
    """
    Concrete implementation of BaseSensorHandler for testing.
    """

    def _setup_sensor(self):
        """Mock setup sensor implementation."""
        pass

    def _teardown_sensor(self):
        """Mock teardown sensor implementation."""
        pass

    def get_health_status(self) -> dict:
        """Mock health status implementation."""
        return {
            'sensor_id': self.sensor_id,
            'is_active': self.is_active
        }


@pytest.fixture
def mock_node():
    """Create a mock ROS 2 node."""
    node = Mock()
    node.get_logger.return_value = Mock()
    node.get_logger().info = Mock()
    node.get_logger().warning = Mock()
    node.get_logger().error = Mock()
    return node


@pytest.fixture
def sensor_config():
    """Create a test sensor configuration."""
    return SensorConfig(
        sensor_id='test_sensor',
        sensor_type=SensorType.CAMERA,
        topic='/test/camera',
        calibration_file='calibration/test.yaml',
        enabled=True,
        processing_frequency=30.0,
        parameters={
            'timeout': 2.0,
            'processing_frequency': 30.0,
            'resolution': [640, 480]
        }
    )


@pytest.fixture
def sensor_handler(mock_node, sensor_config):
    """Create a test sensor handler."""
    return ConcreteSensorHandler(mock_node, sensor_config)


def test_base_sensor_initialization(sensor_handler, sensor_config):
    """Test that BaseSensorHandler initializes correctly."""
    assert sensor_handler.sensor_id == 'test_sensor'
    assert sensor_handler.sensor_type == SensorType.CAMERA
    assert sensor_handler.operational_params == sensor_config.parameters
    assert sensor_handler.timeout == 2.0
    assert sensor_handler.processing_frequency == 30.0
    assert sensor_handler.is_active is False
    assert sensor_handler.last_data_timestamp is None
    assert isinstance(sensor_handler._lock, type(threading.Lock()))


def test_sensor_start(sensor_handler, mock_node):
    """Test starting a sensor."""
    sensor_handler.start()

    assert sensor_handler.is_active is True
    assert sensor_handler.is_operational() is True
    mock_node.get_logger().info.assert_called()


def test_sensor_stop(sensor_handler, mock_node):
    """Test stopping a sensor."""
    # Start first
    sensor_handler.start()
    assert sensor_handler.is_active is True

    # Then stop
    sensor_handler.stop()

    assert sensor_handler.is_active is False
    assert sensor_handler.is_operational() is False
    mock_node.get_logger().info.assert_called()


def test_sensor_start_when_already_active(sensor_handler, mock_node):
    """Test starting a sensor that's already active."""
    sensor_handler.start()
    assert sensor_handler.is_active is True

    # Try to start again
    sensor_handler.start()

    # Should still be active and warning should be logged
    assert sensor_handler.is_active is True
    mock_node.get_logger().warning.assert_called()


def test_sensor_stop_when_not_active(sensor_handler, mock_node):
    """Test stopping a sensor that's not active."""
    assert sensor_handler.is_active is False

    sensor_handler.stop()

    # Should still be inactive and warning should be logged
    assert sensor_handler.is_active is False
    mock_node.get_logger().warning.assert_called()


def test_set_data_callback(sensor_handler):
    """Test setting data callback."""
    callback = Mock()
    sensor_handler.set_data_callback(callback)

    assert sensor_handler.data_callback == callback


def test_set_error_callback(sensor_handler):
    """Test setting error callback."""
    callback = Mock()
    sensor_handler.set_error_callback(callback)

    assert sensor_handler.error_callback == callback


def test_thread_safety(sensor_handler):
    """Test that sensor operations are thread-safe."""
    def start_stop_sensor():
        sensor_handler.start()
        sensor_handler.stop()

    # Run multiple threads concurrently
    threads = [threading.Thread(target=start_stop_sensor) for _ in range(10)]

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()

    # Sensor should be in a consistent state (not active since we stop at the end)
    assert sensor_handler.is_active is False
