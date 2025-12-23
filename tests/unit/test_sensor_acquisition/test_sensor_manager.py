"""
Unit tests for SensorManager.
"""
import pytest
import threading
from unittest.mock import Mock, MagicMock, patch

from src.perception.sensor_acquisition.sensor_manager import SensorManager
from src.perception.common.data_types import SensorConfig, SensorType, SensorData, SensorConfigState


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
def mock_config_handler():
    """Create a mock sensor configuration handler."""
    handler = Mock()
    handler.get_sensor_config = Mock(return_value=None)
    handler.list_sensor_configs = Mock(return_value=[])
    return handler


@pytest.fixture
def sensor_manager(mock_node, mock_config_handler):
    """Create a test sensor manager."""
    return SensorManager(mock_node, mock_config_handler)


@pytest.fixture
def camera_config():
    """Create a camera sensor configuration."""
    return SensorConfig(
        sensor_id='camera_front',
        sensor_type=SensorType.CAMERA,
        topic='/camera_front/image_raw',
        calibration_file='calibration/camera_front.yaml',
        enabled=True,
        processing_frequency=30.0,
        parameters={'resolution': [640, 480], 'format': 'bgr8'}
    )


@pytest.fixture
def lidar_config():
    """Create a LIDAR sensor configuration."""
    return SensorConfig(
        sensor_id='lidar_3d',
        sensor_type=SensorType.LIDAR,
        topic='/lidar_3d/scan',
        calibration_file='calibration/lidar_3d.yaml',
        enabled=True,
        processing_frequency=10.0,
        parameters={'range_max': 30.0, 'range_min': 0.1}
    )


def test_sensor_manager_initialization(sensor_manager, mock_node):
    """Test that SensorManager initializes correctly."""
    assert sensor_manager.node == mock_node
    assert len(sensor_manager.sensor_handlers) == 0
    assert isinstance(sensor_manager._lock, type(threading.Lock()))
    assert sensor_manager._stats['total_sensors'] == 0
    assert sensor_manager._stats['active_sensors'] == 0


def test_add_sensor_camera(sensor_manager, camera_config):
    """Test adding a camera sensor."""
    result = sensor_manager.add_sensor(camera_config)

    assert result is True
    assert 'camera_front' in sensor_manager.sensor_handlers
    assert sensor_manager._stats['total_sensors'] == 1


def test_add_sensor_lidar(sensor_manager, lidar_config):
    """Test adding a LIDAR sensor."""
    result = sensor_manager.add_sensor(lidar_config)

    assert result is True
    assert 'lidar_3d' in sensor_manager.sensor_handlers
    assert sensor_manager._stats['total_sensors'] == 1


def test_add_duplicate_sensor(sensor_manager, camera_config, mock_node):
    """Test adding a sensor that already exists."""
    # Add sensor first time
    result1 = sensor_manager.add_sensor(camera_config)
    assert result1 is True

    # Try to add again
    result2 = sensor_manager.add_sensor(camera_config)
    assert result2 is False
    mock_node.get_logger().warning.assert_called()


def test_remove_sensor(sensor_manager, camera_config):
    """Test removing a sensor."""
    # Add sensor first
    sensor_manager.add_sensor(camera_config)
    assert 'camera_front' in sensor_manager.sensor_handlers

    # Remove sensor
    result = sensor_manager.remove_sensor('camera_front')

    assert result is True
    assert 'camera_front' not in sensor_manager.sensor_handlers
    assert sensor_manager._stats['total_sensors'] == 0


def test_remove_nonexistent_sensor(sensor_manager, mock_node):
    """Test removing a sensor that doesn't exist."""
    result = sensor_manager.remove_sensor('nonexistent')

    assert result is False
    mock_node.get_logger().warning.assert_called()


def test_get_all_sensor_ids(sensor_manager, camera_config, lidar_config):
    """Test getting all sensor IDs."""
    sensor_manager.add_sensor(camera_config)
    sensor_manager.add_sensor(lidar_config)

    sensor_ids = sensor_manager.get_all_sensor_ids()

    assert len(sensor_ids) == 2
    assert 'camera_front' in sensor_ids
    assert 'lidar_3d' in sensor_ids


def test_get_sensor_handler(sensor_manager, camera_config):
    """Test getting a sensor handler."""
    sensor_manager.add_sensor(camera_config)

    handler = sensor_manager.get_sensor_handler('camera_front')

    assert handler is not None
    assert handler.sensor_id == 'camera_front'


def test_get_nonexistent_sensor_handler(sensor_manager):
    """Test getting a handler for nonexistent sensor."""
    handler = sensor_manager.get_sensor_handler('nonexistent')

    assert handler is None


def test_thread_safety_add_remove(sensor_manager, camera_config):
    """Test thread-safety of add/remove operations."""
    def add_remove_sensor():
        sensor_manager.add_sensor(camera_config)
        sensor_manager.remove_sensor('camera_front')

    # Run multiple threads concurrently
    threads = [threading.Thread(target=add_remove_sensor) for _ in range(5)]

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()

    # Manager should be in consistent state
    assert isinstance(sensor_manager._stats['total_sensors'], int)
    assert sensor_manager._stats['total_sensors'] >= 0


def test_statistics(sensor_manager, camera_config):
    """Test getting sensor statistics."""
    sensor_manager.add_sensor(camera_config)

    stats = sensor_manager.get_statistics()

    assert stats['total_sensors'] == 1
    assert stats['active_sensors'] == 0
    assert stats['total_data_received'] == 0
    assert stats['errors_count'] == 0


def test_data_callback(sensor_manager):
    """Test setting and calling data callback."""
    callback = Mock()
    sensor_manager.set_data_callback(callback)

    # Simulate sensor data
    sensor_data = SensorData(
        id='test_sensor_123',
        sensor_type=SensorType.CAMERA,
        timestamp=1234567890.0,
        data=None,
        frame_id='camera_frame',
        metadata={}
    )

    # Call internal data handler
    sensor_manager._handle_sensor_data(sensor_data)

    assert sensor_manager._stats['total_data_received'] == 1
    callback.assert_called_once()


def test_error_callback(sensor_manager):
    """Test setting and calling error callback."""
    callback = Mock()
    sensor_manager.set_error_callback(callback)

    # Simulate sensor error
    error = Exception('Test error')
    sensor_manager._handle_sensor_error('test_sensor', error)

    assert sensor_manager._stats['errors_count'] == 1
    callback.assert_called_once_with('test_sensor', 'Test error', error)
