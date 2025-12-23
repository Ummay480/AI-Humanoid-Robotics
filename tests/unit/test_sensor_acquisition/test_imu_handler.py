"""
Unit tests for IMUHandler.
"""
import pytest
import numpy as np
from unittest.mock import Mock, MagicMock, patch

from src.perception.common.data_types import SensorConfig, SensorType, SensorData


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
def imu_config():
    """Create an IMU sensor configuration."""
    return SensorConfig(
        sensor_id='imu_main',
        sensor_type=SensorType.IMU,
        topic='/imu_main/data',
        calibration_file='calibration/imu_main.yaml',
        enabled=True,
        processing_frequency=100.0,
        parameters={
            'linear_acceleration_stddev': 0.017,
            'angular_velocity_stddev': 0.001,
            'orientation_stddev': 6.66e-05,
            'frame_id': 'imu_frame'
        }
    )


@pytest.fixture
def imu_handler(mock_node, imu_config):
    """Create a test IMU handler with mocked dependencies."""
    with patch('src.perception.sensor_acquisition.imu_handler.Node'), \
         patch('src.perception.sensor_acquisition.imu_handler.Imu'), \
         patch('src.perception.sensor_acquisition.imu_handler.Quaternion'), \
         patch('src.perception.sensor_acquisition.imu_handler.Vector3'), \
         patch('src.perception.sensor_acquisition.imu_handler.BaseSensorHandler'):
        from src.perception.sensor_acquisition.imu_handler import IMUHandler
        return IMUHandler(mock_node, imu_config)


def test_imu_handler_initialization(imu_handler, imu_config):
    """Test that IMUHandler initializes correctly."""
    assert imu_handler.sensor_id == 'imu_main'
    assert imu_handler.sensor_type == SensorType.IMU
    assert imu_handler.linear_acceleration_stddev == 0.017
    assert imu_handler.angular_velocity_stddev == 0.001
    assert imu_handler.orientation_stddev == 6.66e-05
    assert imu_handler.frame_id == 'imu_frame'
    assert imu_handler.subscription is None
    assert imu_handler._latest_orientation is None
    assert imu_handler._latest_angular_velocity is None
    assert imu_handler._latest_linear_acceleration is None
    assert imu_handler._imu_timestamp is None


def test_imu_setup_sensor(mock_node, imu_config):
    """Test setting up the IMU sensor."""
    with patch('src.perception.sensor_acquisition.imu_handler.Node'), \
         patch('src.perception.sensor_acquisition.imu_handler.Imu') as mock_msg, \
         patch('src.perception.sensor_acquisition.imu_handler.Quaternion'), \
         patch('src.perception.sensor_acquisition.imu_handler.Vector3'), \
         patch('src.perception.sensor_acquisition.imu_handler.BaseSensorHandler'):
        from src.perception.sensor_acquisition.imu_handler import IMUHandler
        handler = IMUHandler(mock_node, imu_config)

        # Mock the subscription creation
        mock_subscription = Mock()
        mock_node.create_subscription.return_value = mock_subscription

        # Call _setup_sensor
        handler._setup_sensor()

        # Verify subscription was created with correct parameters
        mock_node.create_subscription.assert_called_once_with(
            mock_msg,
            '/imu_main/data',
            handler._imu_callback,
            ANY  # This would be the QoS profile
        )


def test_imu_callback(imu_handler):
    """Test the IMU callback function."""
    # Mock Imu message
    mock_imu_msg = Mock()
    mock_imu_msg.orientation.x = 0.0
    mock_imu_msg.orientation.y = 0.0
    mock_imu_msg.orientation.z = 0.0
    mock_imu_msg.orientation.w = 1.0
    mock_imu_msg.angular_velocity.x = 0.1
    mock_imu_msg.angular_velocity.y = 0.2
    mock_imu_msg.angular_velocity.z = 0.3
    mock_imu_msg.linear_acceleration.x = 9.8
    mock_imu_msg.linear_acceleration.y = 0.0
    mock_imu_msg.linear_acceleration.z = 0.0
    mock_imu_msg.header.stamp.sec = 1234
    mock_imu_msg.header.stamp.nanosec = 567890
    mock_imu_msg.header.frame_id = 'imu_frame'
    mock_imu_msg.orientation_covariance = [0.1] * 9
    mock_imu_msg.angular_velocity_covariance = [0.01] * 9
    mock_imu_msg.linear_acceleration_covariance = [0.05] * 9

    # Start the handler to make it active
    imu_handler.start()

    # Call the callback
    imu_handler._imu_callback(mock_imu_msg)

    # Verify that the IMU data was stored
    expected_orientation = np.array([0.0, 0.0, 0.0, 1.0])
    expected_angular_velocity = np.array([0.1, 0.2, 0.3])
    expected_linear_acceleration = np.array([9.8, 0.0, 0.0])

    assert np.array_equal(imu_handler._latest_orientation, expected_orientation)
    assert np.array_equal(imu_handler._latest_angular_velocity, expected_angular_velocity)
    assert np.array_equal(imu_handler._latest_linear_acceleration, expected_linear_acceleration)

    # Verify timestamp was updated
    expected_timestamp = 1234 + 567890 / 1e9
    assert imu_handler._imu_timestamp == expected_timestamp
    assert imu_handler.last_data_timestamp == expected_timestamp


def test_get_latest_orientation(imu_handler):
    """Test getting the latest orientation."""
    # Initially should return None
    assert imu_handler.get_latest_orientation() is None

    # Set an orientation
    test_orientation = np.array([0.0, 0.0, 0.0, 1.0])
    imu_handler._latest_orientation = test_orientation

    # Should return a copy of the orientation
    retrieved_orientation = imu_handler.get_latest_orientation()
    assert np.array_equal(retrieved_orientation, test_orientation)
    assert retrieved_orientation is not test_orientation  # Should be a copy


def test_get_latest_angular_velocity(imu_handler):
    """Test getting the latest angular velocity."""
    # Initially should return None
    assert imu_handler.get_latest_angular_velocity() is None

    # Set angular velocity
    test_angular_velocity = np.array([0.1, 0.2, 0.3])
    imu_handler._latest_angular_velocity = test_angular_velocity

    # Should return a copy
    retrieved_angular_velocity = imu_handler.get_latest_angular_velocity()
    assert np.array_equal(retrieved_angular_velocity, test_angular_velocity)
    assert retrieved_angular_velocity is not test_angular_velocity  # Should be a copy


def test_get_latest_linear_acceleration(imu_handler):
    """Test getting the latest linear acceleration."""
    # Initially should return None
    assert imu_handler.get_latest_linear_acceleration() is None

    # Set linear acceleration
    test_linear_acceleration = np.array([9.8, 0.0, 0.0])
    imu_handler._latest_linear_acceleration = test_linear_acceleration

    # Should return a copy
    retrieved_linear_acceleration = imu_handler.get_latest_linear_acceleration()
    assert np.array_equal(retrieved_linear_acceleration, test_linear_acceleration)
    assert retrieved_linear_acceleration is not test_linear_acceleration  # Should be a copy


def test_get_latest_timestamp(imu_handler):
    """Test getting the latest timestamp."""
    # Initially should return None
    assert imu_handler.get_latest_timestamp() is None

    # Set a timestamp
    imu_handler._imu_timestamp = 1234567890.123
    assert imu_handler.get_latest_timestamp() == 1234567890.123


def test_get_euler_angles(imu_handler):
    """Test converting orientation quaternion to Euler angles."""
    # Set a simple orientation (no rotation)
    imu_handler._latest_orientation = np.array([0.0, 0.0, 0.0, 1.0])

    euler_angles = imu_handler.get_euler_angles()
    assert euler_angles is not None
    assert len(euler_angles) == 3  # roll, pitch, yaw
    assert all(abs(angle) < 1e-6 for angle in euler_angles)  # Should be close to zero

    # Set a 90-degree rotation around Z axis (yaw)
    imu_handler._latest_orientation = np.array([0.0, 0.0, 0.707, 0.707])  # ~[0, 0, sin(π/4), cos(π/4)]
    euler_angles = imu_handler.get_euler_angles()
    assert euler_angles is not None
    roll, pitch, yaw = euler_angles
    assert abs(roll) < 0.01
    assert abs(pitch) < 0.01
    assert abs(yaw - 1.57) < 0.01  # ~π/2


def test_validate_imu_data(imu_handler):
    """Test IMU data validation."""
    # Test with valid quaternion (normalized)
    imu_handler._latest_orientation = np.array([0.0, 0.0, 0.0, 1.0])
    assert imu_handler.validate_imu_data() is True

    # Test with None orientation
    imu_handler._latest_orientation = None
    assert imu_handler.validate_imu_data() is False

    # Test with unnormalized quaternion
    imu_handler._latest_orientation = np.array([1.0, 1.0, 1.0, 1.0])  # Not normalized
    assert imu_handler.validate_imu_data() is False

    # Test with NaN values
    imu_handler._latest_orientation = np.array([np.nan, 0.0, 0.0, 1.0])
    assert imu_handler.validate_imu_data() is False

    # Test with infinite values
    imu_handler._latest_orientation = np.array([float('inf'), 0.0, 0.0, 1.0])
    assert imu_handler.validate_imu_data() is False


def test_get_health_status(imu_handler):
    """Test getting health status."""
    health_status = imu_handler.get_health_status()

    # Check that required fields are present
    assert 'sensor_id' in health_status
    assert 'sensor_type' in health_status
    assert 'is_active' in health_status
    assert 'is_healthy' in health_status
    assert 'has_orientation' in health_status
    assert 'has_angular_velocity' in health_status
    assert 'has_linear_acceleration' in health_status

    # Initially should not be healthy since no data
    assert health_status['sensor_id'] == 'imu_main'
    assert health_status['sensor_type'] == 'imu'
    assert health_status['is_active'] is False
    assert health_status['has_orientation'] is False
    assert health_status['has_angular_velocity'] is False
    assert health_status['has_linear_acceleration'] is False


def test_imu_callback_error_handling(imu_handler):
    """Test error handling in IMU callback."""
    # Mock an Imu message that will cause an error
    mock_imu_msg = Mock()
    mock_imu_msg.orientation.x = 0.0
    mock_imu_msg.orientation.y = 0.0
    mock_imu_msg.orientation.z = 0.0
    mock_imu_msg.orientation.w = 1.0
    mock_imu_msg.angular_velocity.x = 0.1
    mock_imu_msg.angular_velocity.y = 0.2
    mock_imu_msg.angular_velocity.z = 0.3
    mock_imu_msg.linear_acceleration.x = 9.8
    mock_imu_msg.linear_acceleration.y = 0.0
    mock_imu_msg.linear_acceleration.z = 0.0
    mock_imu_msg.header.stamp.sec = 1234
    mock_imu_msg.header.stamp.nanosec = 567890
    mock_imu_msg.header.frame_id = 'imu_frame'

    # Mock an exception in the processing
    with patch.object(imu_handler, '_lock', side_effect=Exception("Processing error")):
        # Start the handler to make it active
        imu_handler.start()

        # Call the callback - should not crash, but log error
        imu_handler._imu_callback(mock_imu_msg)

        # Verify error was logged
        imu_handler.node.get_logger().error.assert_called()


# Need to define ANY for the mock comparison
class ANY:
    def __eq__(self, other):
        return True