"""
Unit tests for LidarHandler.
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
def lidar_config():
    """Create a LIDAR sensor configuration."""
    return SensorConfig(
        sensor_id='lidar_3d',
        sensor_type=SensorType.LIDAR,
        topic='/lidar_3d/scan',
        calibration_file='calibration/lidar_3d.yaml',
        enabled=True,
        processing_frequency=10.0,
        parameters={
            'range_min': 0.1,
            'range_max': 30.0,
            'angle_min': -2.356,  # -135 degrees
            'angle_max': 2.356,   # 135 degrees
            'angle_increment': 0.0043,
            'frame_id': 'lidar_frame',
            'data_format': 'laser_scan'
        }
    )


@pytest.fixture
def lidar_handler(mock_node, lidar_config):
    """Create a test LIDAR handler with mocked dependencies."""
    with patch('src.perception.sensor_acquisition.lidar_handler.Node'), \
         patch('src.perception.sensor_acquisition.lidar_handler.LaserScan'), \
         patch('src.perception.sensor_acquisition.lidar_handler.PointCloud2'), \
         patch('src.perception.sensor_acquisition.lidar_handler.point_cloud2'), \
         patch('src.perception.sensor_acquisition.lidar_handler.BaseSensorHandler'):
        from src.perception.sensor_acquisition.lidar_handler import LidarHandler
        return LidarHandler(mock_node, lidar_config)


def test_lidar_handler_initialization(lidar_handler, lidar_config):
    """Test that LidarHandler initializes correctly."""
    assert lidar_handler.sensor_id == 'lidar_3d'
    assert lidar_handler.sensor_type == SensorType.LIDAR
    assert lidar_handler.range_min == 0.1
    assert lidar_handler.range_max == 30.0
    assert lidar_handler.angle_min == -2.356
    assert lidar_handler.angle_max == 2.356
    assert lidar_handler.angle_increment == 0.0043
    assert lidar_handler.frame_id == 'lidar_frame'
    assert lidar_handler.data_format == 'laser_scan'
    assert lidar_handler.subscription is None
    assert lidar_handler._latest_scan is None
    assert lidar_handler._scan_timestamp is None


def test_lidar_setup_sensor(mock_node, lidar_config):
    """Test setting up the LIDAR sensor."""
    with patch('src.perception.sensor_acquisition.lidar_handler.Node'), \
         patch('src.perception.sensor_acquisition.lidar_handler.LaserScan') as mock_msg, \
         patch('src.perception.sensor_acquisition.lidar_handler.PointCloud2'), \
         patch('src.perception.sensor_acquisition.lidar_handler.point_cloud2'), \
         patch('src.perception.sensor_acquisition.lidar_handler.BaseSensorHandler'):
        from src.perception.sensor_acquisition.lidar_handler import LidarHandler
        handler = LidarHandler(mock_node, lidar_config)

        # Mock the subscription creation
        mock_subscription = Mock()
        mock_node.create_subscription.return_value = mock_subscription

        # Call _setup_sensor
        handler._setup_sensor()

        # Verify subscription was created with correct parameters
        mock_node.create_subscription.assert_called_once_with(
            mock_msg,
            '/lidar_3d/scan',
            handler._laser_scan_callback,
            ANY  # This would be the QoS profile
        )


def test_lidar_laser_scan_callback(lidar_handler):
    """Test the laser scan callback function."""
    # Mock LaserScan message
    mock_scan_msg = Mock()
    mock_scan_msg.ranges = [1.0, 2.0, 3.0, 4.0]
    mock_scan_msg.intensities = [0.5, 0.6, 0.7, 0.8]
    mock_scan_msg.header.stamp.sec = 1234
    mock_scan_msg.header.stamp.nanosec = 567890
    mock_scan_msg.header.frame_id = 'lidar_frame'
    mock_scan_msg.range_min = 0.1
    mock_scan_msg.range_max = 30.0
    mock_scan_msg.angle_min = -2.356
    mock_scan_msg.angle_max = 2.356
    mock_scan_msg.angle_increment = 0.0043
    mock_scan_msg.time_increment = 0.0
    mock_scan_msg.scan_time = 0.1

    # Start the handler to make it active
    lidar_handler.start()

    # Call the callback
    lidar_handler._laser_scan_callback(mock_scan_msg)

    # Verify that the scan was stored
    assert lidar_handler._latest_scan is not None
    assert np.array_equal(lidar_handler._latest_scan, np.array([1.0, 2.0, 3.0, 4.0]))

    # Verify timestamp was updated
    expected_timestamp = 1234 + 567890 / 1e9
    assert lidar_handler._scan_timestamp == expected_timestamp
    assert lidar_handler.last_data_timestamp == expected_timestamp


def test_lidar_point_cloud_callback(lidar_handler):
    """Test the point cloud callback function."""
    # Change the data format to point cloud
    lidar_handler.data_format = 'point_cloud'

    # Mock PointCloud2 message
    mock_pointcloud_msg = Mock()
    mock_pointcloud_msg.header.stamp.sec = 1234
    mock_pointcloud_msg.header.stamp.nanosec = 567890
    mock_pointcloud_msg.header.frame_id = 'lidar_frame'
    mock_pointcloud_msg.height = 1
    mock_pointcloud_msg.width = 100
    mock_pointcloud_msg.fields = [Mock(), Mock(), Mock()]
    for i, field in enumerate(mock_pointcloud_msg.fields):
        field.name = ['x', 'y', 'z'][i]
    mock_pointcloud_msg.is_bigendian = False
    mock_pointcloud_msg.point_step = 12
    mock_pointcloud_msg.row_step = 1200
    mock_pointcloud_msg.is_dense = True

    # Mock the point_cloud2.read_points_numpy function
    with patch('src.perception.sensor_acquisition.lidar_handler.point_cloud2') as mock_pc2:
        mock_pc2.read_points_numpy.return_value = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])

        # Start the handler to make it active
        lidar_handler.start()

        # Call the callback
        lidar_handler._point_cloud_callback(mock_pointcloud_msg)

        # Verify that the point cloud was stored
        assert lidar_handler._latest_scan is not None
        assert np.array_equal(
            lidar_handler._latest_scan,
            np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        )


def test_get_latest_scan(lidar_handler):
    """Test getting the latest scan."""
    # Initially should return None
    assert lidar_handler.get_latest_scan() is None

    # Set a scan
    test_scan = np.array([1.0, 2.0, 3.0])
    lidar_handler._latest_scan = test_scan

    # Should return a copy of the scan
    retrieved_scan = lidar_handler.get_latest_scan()
    assert np.array_equal(retrieved_scan, test_scan)
    assert retrieved_scan is not test_scan  # Should be a copy


def test_get_latest_timestamp(lidar_handler):
    """Test getting the latest timestamp."""
    # Initially should return None
    assert lidar_handler.get_latest_timestamp() is None

    # Set a timestamp
    lidar_handler._scan_timestamp = 1234567890.123
    assert lidar_handler.get_latest_timestamp() == 1234567890.123


def test_get_scan_as_cartesian(lidar_handler):
    """Test converting scan to Cartesian coordinates."""
    # Set up laser scan data
    lidar_handler.data_format = 'laser_scan'
    lidar_handler.angle_min = -1.57  # -90 degrees
    lidar_handler.angle_increment = 0.01  # Small increment
    ranges = np.array([1.0, 1.0, 1.0])  # All ranges are 1.0
    lidar_handler._latest_scan = ranges

    # Get Cartesian coordinates
    cartesian = lidar_handler.get_scan_as_cartesian()

    # Should return coordinates for the three points
    assert cartesian is not None
    assert cartesian.shape[0] == 3  # 3 points
    assert cartesian.shape[1] == 2  # x, y coordinates


def test_filter_scan_by_range(lidar_handler):
    """Test filtering scan by range."""
    scan = np.array([0.05, 0.5, 1.0, 15.0, 35.0])

    # Filter with default ranges (0.1 to 30.0)
    filtered = lidar_handler.filter_scan_by_range(scan)
    expected = np.array([0.5, 1.0, 15.0])  # Only values in range
    assert np.array_equal(filtered, expected)

    # Filter with custom ranges
    filtered = lidar_handler.filter_scan_by_range(scan, min_range=1.0, max_range=20.0)
    expected = np.array([1.0, 15.0])  # Only values in custom range
    assert np.array_equal(filtered, expected)


def test_validate_scan(lidar_handler):
    """Test scan validation."""
    # Test with valid scan
    valid_scan = np.array([1.0, 2.0, 3.0])
    assert lidar_handler.validate_scan(valid_scan) is True

    # Test with None
    assert lidar_handler.validate_scan(None) is False

    # Test with empty array
    empty_scan = np.array([])
    assert lidar_handler.validate_scan(empty_scan) is False

    # Test with scan that has too many invalid readings
    invalid_scan = np.array([0.05, 0.06, 0.07])  # All below range_min
    assert lidar_handler.validate_scan(invalid_scan) is False


def test_get_health_status(lidar_handler):
    """Test getting health status."""
    health_status = lidar_handler.get_health_status()

    # Check that required fields are present
    assert 'sensor_id' in health_status
    assert 'sensor_type' in health_status
    assert 'is_active' in health_status
    assert 'is_healthy' in health_status
    assert 'has_data' in health_status
    assert 'data_format' in health_status

    # Initially should not be healthy since no data
    assert health_status['sensor_id'] == 'lidar_3d'
    assert health_status['sensor_type'] == 'lidar'
    assert health_status['is_active'] is False
    assert health_status['has_data'] is False
    assert health_status['data_format'] == 'laser_scan'


def test_lidar_scan_callback_error_handling(lidar_handler):
    """Test error handling in laser scan callback."""
    # Mock a LaserScan message that will cause an error
    mock_scan_msg = Mock()
    mock_scan_msg.ranges = [1.0, 2.0, 3.0]
    mock_scan_msg.header.stamp.sec = 1234
    mock_scan_msg.header.stamp.nanosec = 567890
    mock_scan_msg.header.frame_id = 'lidar_frame'

    # Mock an exception in the processing
    with patch.object(lidar_handler, '_lock', side_effect=Exception("Processing error")):
        # Start the handler to make it active
        lidar_handler.start()

        # Call the callback - should not crash, but log error
        lidar_handler._laser_scan_callback(mock_scan_msg)

        # Verify error was logged
        lidar_handler.node.get_logger().error.assert_called()


# Need to define ANY for the mock comparison
class ANY:
    def __eq__(self, other):
        return True