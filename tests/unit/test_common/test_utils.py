"""
Unit tests for the utils module.
"""
import time
from src.perception.common.utils import (
    get_current_timestamp,
    timestamp_to_ros_time,
    ros_time_to_timestamp,
    calculate_time_diff,
    is_timestamp_valid,
    normalize_confidence_score,
    validate_coordinate_frame,
    validate_3d_position,
    create_bounding_box,
    calculate_bounding_box_center,
    time_diff_seconds,
    is_timestamp_valid_extended,
    synchronize_timestamps,
    format_timestamp,
    interpolate_timestamps,
    timestamp_to_nanoseconds,
    nanoseconds_to_timestamp
)


class TestUtils:
    """Test utility functions."""

    def test_get_current_timestamp(self):
        """Test getting current timestamp."""
        timestamp = get_current_timestamp()
        current_time = time.time()
        # Allow for small time difference
        assert abs(current_time - timestamp) < 1.0

    def test_timestamp_to_ros_time_and_back(self):
        """Test converting timestamp to ROS time and back."""
        original_timestamp = 1234567890.123456
        ros_time = timestamp_to_ros_time(original_timestamp)
        converted_timestamp = ros_time_to_timestamp(ros_time)
        assert abs(original_timestamp - converted_timestamp) < 1e-6

    def test_calculate_time_diff(self):
        """Test calculating time difference."""
        diff = calculate_time_diff(10.0, 15.0)
        assert diff == 5.0

    def test_is_timestamp_valid(self):
        """Test timestamp validation."""
        current = get_current_timestamp()
        assert is_timestamp_valid(current) is True
        assert is_timestamp_valid(current + 2.0, max_future_offset=1.0) is False  # In future beyond limit

    def test_normalize_confidence_score(self):
        """Test confidence score normalization."""
        assert normalize_confidence_score(0.5) == 0.5
        assert normalize_confidence_score(1.5) == 1.0  # Should cap at 1.0
        assert normalize_confidence_score(-0.5) == 0.0  # Should floor at 0.0

    def test_validate_coordinate_frame(self):
        """Test coordinate frame validation."""
        assert validate_coordinate_frame("camera_frame") is True
        assert validate_coordinate_frame("lidar1") is True
        assert validate_coordinate_frame("base_link") is True
        assert validate_coordinate_frame("") is False  # Empty
        assert validate_coordinate_frame("frame with spaces") is False  # Contains space
        assert validate_coordinate_frame("1frame") is False  # Starts with number

    def test_validate_3d_position(self):
        """Test 3D position validation."""
        assert validate_3d_position([1.0, 2.0, 3.0]) is True
        assert validate_3d_position([1.0, 2.0]) is False  # Only 2 coords
        assert validate_3d_position([1.0, 2.0, float('inf')]) is False  # Infinite coord

    def test_create_bounding_box(self):
        """Test creating a bounding box."""
        bbox = create_bounding_box(10.0, 20.0, 100.0, 50.0)
        assert bbox == [10.0, 20.0, 100.0, 50.0]

    def test_calculate_bounding_box_center(self):
        """Test calculating bounding box center."""
        center = calculate_bounding_box_center([10.0, 20.0, 100.0, 50.0])
        assert center == [60.0, 45.0]  # center_x = 10 + 100/2, center_y = 20 + 50/2

    def test_time_diff_seconds(self):
        """Test time difference calculation."""
        diff = time_diff_seconds(10.0, 15.0)
        assert diff == 5.0

    def test_is_timestamp_valid_extended(self):
        """Test extended timestamp validation."""
        current = get_current_timestamp()
        assert is_timestamp_valid_extended(current) is True
        # Old timestamp (older than 10 seconds)
        old_timestamp = current - 15.0
        assert is_timestamp_valid_extended(old_timestamp) is False

    def test_synchronize_timestamps(self):
        """Test timestamp synchronization."""
        timestamps = [1.0, 2.0, 3.0]
        ref_time = 10.0
        synced = synchronize_timestamps(timestamps, ref_time)
        expected = [10.0, 11.0, 12.0]  # 1.0 becomes 10.0, so offset is +9
        assert synced == expected

    def test_format_timestamp(self):
        """Test timestamp formatting."""
        timestamp = 1234567890.0  # This is 2009-02-13/14 depending on timezone
        formatted = format_timestamp(timestamp, "%Y-%m-%d")
        # Check that it contains the expected year and month
        assert formatted.startswith("2009-02")

    def test_interpolate_timestamps(self):
        """Test timestamp interpolation."""
        interpolated = interpolate_timestamps(0.0, 4.0, 5)
        expected = [0.0, 1.0, 2.0, 3.0, 4.0]
        assert interpolated == expected

    def test_timestamp_to_nanoseconds_and_back(self):
        """Test converting timestamp to nanoseconds and back."""
        original_timestamp = 1234567890.123456
        nanoseconds = timestamp_to_nanoseconds(original_timestamp)
        converted_timestamp = nanoseconds_to_timestamp(nanoseconds)
        assert abs(original_timestamp - converted_timestamp) < 1e-6