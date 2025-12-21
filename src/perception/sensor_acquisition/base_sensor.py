"""
Base sensor handler class providing common functionality for all sensor types.
"""
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Callable
import time
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan, Imu
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from ..common.data_types import SensorType, SensorData, SensorConfig
from ..common.utils import get_current_timestamp, timestamp_to_ros_time


class BaseSensorHandler(ABC):
    """
    Abstract base class for all sensor handlers.
    Provides common functionality and interface for sensor acquisition.
    """

    def __init__(self, node: Node, sensor_config: SensorConfig):
        """
        Initialize the base sensor handler.

        Args:
            node: ROS 2 node to use for sensor operations
            sensor_config: Configuration for this sensor
        """
        self.node = node
        self.sensor_config = sensor_config
        self.sensor_id = sensor_config.sensor_id
        self.sensor_type = sensor_config.sensor_type
        self.is_active = False
        self.last_data_timestamp = None
        self.data_callback = None
        self.error_callback = None

        # Thread lock for thread-safe operations
        self._lock = threading.Lock()

        # Initialize with the config's operational parameters
        self.operational_params = sensor_config.operational_params
        self.frequency = self.operational_params.get('frequency', 10.0)  # Default to 10Hz
        self.timeout = self.operational_params.get('timeout', 5.0)  # Default to 5 seconds

    def set_data_callback(self, callback: Callable[[SensorData], None]):
        """
        Set the callback function to be called when new sensor data is available.

        Args:
            callback: Function to call with new sensor data
        """
        self.data_callback = callback

    def set_error_callback(self, callback: Callable[[str, Exception], None]):
        """
        Set the callback function to be called when sensor errors occur.

        Args:
            callback: Function to call with error information
        """
        self.error_callback = callback

    def start(self):
        """
        Start the sensor acquisition process.
        """
        self.is_active = True
        self._setup_sensor()

    def stop(self):
        """
        Stop the sensor acquisition process.
        """
        self.is_active = False
        self._teardown_sensor()

    def is_operational(self) -> bool:
        """
        Check if the sensor is currently operational.

        Returns:
            bool: True if sensor is operational, False otherwise
        """
        return self.is_active

    @abstractmethod
    def _setup_sensor(self):
        """
        Abstract method to set up the specific sensor type.
        Must be implemented by subclasses.
        """
        pass

    @abstractmethod
    def _teardown_sensor(self):
        """
        Abstract method to tear down the specific sensor type.
        Must be implemented by subclasses.
        """
        pass

    @abstractmethod
    def _process_raw_data(self, raw_data: Any) -> SensorData:
        """
        Abstract method to process raw sensor data into SensorData object.
        Must be implemented by subclasses.

        Args:
            raw_data: Raw data from the sensor

        Returns:
            SensorData: Processed sensor data
        """
        pass

    def _create_sensor_data(self, raw_data: Any, frame_id: str = None) -> SensorData:
        """
        Create a SensorData object from raw data.

        Args:
            raw_data: Raw sensor data
            frame_id: Coordinate frame ID (if None, uses config value)

        Returns:
            SensorData: New SensorData object
        """
        timestamp = get_current_timestamp()

        if frame_id is None:
            # Use the mounting position frame from config or default
            frame_id = self.sensor_config.mounting_position.get('frame_id',
                                                               f"{self.sensor_id}_frame")

        # Process the raw data to extract necessary information
        processed_data = self._process_raw_data(raw_data)

        sensor_data = SensorData(
            id=f"{self.sensor_id}_{int(timestamp * 1e9)}",  # Use nanosecond precision for unique ID
            sensor_type=self.sensor_type,
            timestamp=timestamp,
            data=raw_data,
            frame_id=frame_id,
            metadata={
                'acquisition_time': timestamp,
                'sensor_config_id': self.sensor_config.sensor_id,
                'frequency': self.frequency
            }
        )

        self.last_data_timestamp = timestamp
        return sensor_data

    def _safe_notify_data_callback(self, sensor_data: SensorData):
        """
        Safely notify the data callback with thread locking.

        Args:
            sensor_data: The sensor data to pass to the callback
        """
        with self._lock:
            if self.data_callback:
                try:
                    self.data_callback(sensor_data)
                except Exception as e:
                    self.node.get_logger().error(f"Error in data callback for sensor {self.sensor_id}: {e}")
                    if self.error_callback:
                        self.error_callback(self.sensor_id, e)

    def _safe_notify_error_callback(self, error_msg: str, exception: Exception):
        """
        Safely notify the error callback with thread locking.

        Args:
            error_msg: Error message to pass to the callback
            exception: Exception that occurred
        """
        with self._lock:
            if self.error_callback:
                try:
                    self.error_callback(error_msg, exception)
                except Exception as e:
                    self.node.get_logger().error(f"Error in error callback for sensor {self.sensor_id}: {e}")

    def validate_data(self, sensor_data: SensorData) -> bool:
        """
        Validate sensor data before processing.

        Args:
            sensor_data: Sensor data to validate

        Returns:
            bool: True if data is valid, False otherwise
        """
        # Check if timestamp is valid
        if not sensor_data.timestamp or sensor_data.timestamp <= 0:
            self.node.get_logger().error(f"Invalid timestamp for sensor {self.sensor_id}")
            return False

        # Check if data payload exists
        if sensor_data.data is None:
            self.node.get_logger().error(f"Empty data payload for sensor {self.sensor_id}")
            return False

        # Check if frame ID is valid
        if not sensor_data.frame_id:
            self.node.get_logger().error(f"Missing frame_id for sensor {self.sensor_id}")
            return False

        return True

    def _check_timeout(self) -> bool:
        """
        Check if the sensor has timed out.

        Returns:
            bool: True if sensor has timed out, False otherwise
        """
        if self.last_data_timestamp is None:
            return False

        current_time = get_current_timestamp()
        elapsed_time = current_time - self.last_data_timestamp
        return elapsed_time > self.timeout

    def _get_qos_profile(self, reliability: ReliabilityPolicy = ReliabilityPolicy.BEST_EFFORT):
        """
        Get a QoS profile for sensor data publication.

        Args:
            reliability: Reliability policy for the QoS profile

        Returns:
            QoSProfile: Appropriate QoS profile for sensor type
        """
        # Different sensors might need different QoS profiles
        if self.sensor_type == SensorType.IMU:
            # IMU data often needs to be reliable
            reliability = ReliabilityPolicy.RELIABLE

        return QoSProfile(
            depth=10,
            reliability=reliability
        )

    def update_operational_params(self, new_params: Dict[str, Any]):
        """
        Update operational parameters for this sensor.

        Args:
            new_params: New operational parameters
        """
        self.operational_params.update(new_params)
        self.frequency = self.operational_params.get('frequency', self.frequency)
        self.timeout = self.operational_params.get('timeout', self.timeout)

        # Apply the updated parameters to the sensor
        self._apply_operational_params()

    def _apply_operational_params(self):
        """
        Apply operational parameters to the actual sensor.
        Can be overridden by subclasses to handle specific parameter changes.
        """
        # Default implementation does nothing, can be overridden
        pass

    def get_status(self) -> Dict[str, Any]:
        """
        Get the current status of the sensor.

        Returns:
            Dict[str, Any]: Status information including active state, last data time, etc.
        """
        status = {
            'sensor_id': self.sensor_id,
            'sensor_type': self.sensor_type.value,
            'is_active': self.is_active,
            'last_data_timestamp': self.last_data_timestamp,
            'operational_params': self.operational_params,
            'is_operational': self.is_operational(),
            'health_status': self.get_health_status()
        }

        # Add timeout information
        if self.last_data_timestamp:
            current_time = get_current_timestamp()
            time_since_last_data = current_time - self.last_data_timestamp
            status['time_since_last_data'] = time_since_last_data
            status['is_timed_out'] = time_since_last_data > self.timeout
        else:
            status['time_since_last_data'] = None
            status['is_timed_out'] = False

        return status

    def get_health_status(self) -> str:
        """
        Determine the health status of the sensor.

        Returns:
            str: Health status ('healthy', 'degraded', 'faulty', 'inactive')
        """
        if not self.is_active:
            return 'inactive'

        if self.last_data_timestamp is None:
            return 'faulty'  # No data ever received

        time_since_last_data = get_current_timestamp() - self.last_data_timestamp

        # Check if sensor is timing out
        if time_since_last_data > self.timeout:
            return 'faulty'

        # Check if frequency is lower than expected (degraded)
        expected_interval = 1.0 / self.frequency
        if time_since_last_data > expected_interval * 2:
            return 'degraded'

        return 'healthy'


class SensorManager:
    """
    Manages multiple sensor handlers and coordinates their operation.

    The SensorManager provides centralized control over all sensors in the system,
    including starting/stopping sensors, monitoring their health, and updating
    their operational parameters.
    """

    def __init__(self, node: Node):
        """
        Initialize the sensor manager.

        Args:
            node: ROS 2 node to use for sensor operations
        """
        self.node = node
        self.sensors: Dict[str, BaseSensorHandler] = {}

    def register_sensor(self, sensor_handler: BaseSensorHandler):
        """
        Register a sensor handler with the manager.

        Args:
            sensor_handler: Sensor handler to register
        """
        sensor_id = sensor_handler.sensor_id
        self.sensors[sensor_id] = sensor_handler

    def unregister_sensor(self, sensor_id: str):
        """
        Unregister a sensor handler.

        Args:
            sensor_id: ID of the sensor to unregister
        """
        if sensor_id in self.sensors:
            # Stop the sensor before unregistering
            self.sensors[sensor_id].stop()
            del self.sensors[sensor_id]

    def start_all_sensors(self):
        """
        Start all registered sensors.
        """
        for sensor_id, sensor in self.sensors.items():
            try:
                sensor.start()
                self.node.get_logger().info(f"Started sensor: {sensor_id}")
            except Exception as e:
                self.node.get_logger().error(f"Failed to start sensor {sensor_id}: {e}")
                if self.sensors[sensor_id].error_callback:
                    self.sensors[sensor_id].error_callback(sensor_id, e)

    def stop_all_sensors(self):
        """
        Stop all registered sensors.
        """
        for sensor_id, sensor in self.sensors.items():
            try:
                sensor.stop()
                self.node.get_logger().info(f"Stopped sensor: {sensor_id}")
            except Exception as e:
                self.node.get_logger().error(f"Failed to stop sensor {sensor_id}: {e}")

    def get_sensor_status(self, sensor_id: str) -> Optional[Dict[str, Any]]:
        """
        Get the status of a specific sensor.

        Args:
            sensor_id: ID of the sensor to get status for

        Returns:
            Dict[str, Any]: Status information or None if sensor not found
        """
        if sensor_id in self.sensors:
            return self.sensors[sensor_id].get_status()
        return None

    def get_all_sensor_statuses(self) -> Dict[str, Dict[str, Any]]:
        """
        Get the status of all registered sensors.

        Returns:
            Dict[str, Dict[str, Any]]: Status information for all sensors
        """
        statuses = {}
        for sensor_id, sensor in self.sensors.items():
            try:
                statuses[sensor_id] = sensor.get_status()
            except Exception as e:
                self.node.get_logger().error(f"Error getting status for sensor {sensor_id}: {e}")
                statuses[sensor_id] = {
                    'sensor_id': sensor_id,
                    'error': str(e),
                    'is_operational': False,
                    'health_status': 'faulty'
                }
        return statuses

    def get_healthy_sensors(self) -> Dict[str, BaseSensorHandler]:
        """
        Get only sensors that are currently healthy.

        Returns:
            Dict[str, BaseSensorHandler]: Dictionary of healthy sensors
        """
        healthy_sensors = {}
        for sensor_id, sensor in self.sensors.items():
            try:
                if sensor.get_health_status() == 'healthy':
                    healthy_sensors[sensor_id] = sensor
            except Exception as e:
                self.node.get_logger().error(f"Error checking health for sensor {sensor_id}: {e}")
        return healthy_sensors

    def get_faulty_sensors(self) -> Dict[str, BaseSensorHandler]:
        """
        Get sensors that are currently faulty.

        Returns:
            Dict[str, BaseSensorHandler]: Dictionary of faulty sensors
        """
        faulty_sensors = {}
        for sensor_id, sensor in self.sensors.items():
            try:
                health_status = sensor.get_health_status()
                if health_status in ['faulty', 'degraded']:
                    faulty_sensors[sensor_id] = sensor
            except Exception as e:
                self.node.get_logger().error(f"Error checking health for sensor {sensor_id}: {e}")
                # Consider sensors with health check errors as faulty
                faulty_sensors[sensor_id] = sensor
        return faulty_sensors

    def monitor_sensor_health(self):
        """
        Monitor the health of all sensors and log warnings for unhealthy sensors.
        """
        healthy_sensors = self.get_healthy_sensors()
        faulty_sensors = self.get_faulty_sensors()

        if faulty_sensors:
            self.node.get_logger().warning(f"Found {len(faulty_sensors)} unhealthy sensors: {list(faulty_sensors.keys())}")
            for sensor_id, sensor in faulty_sensors.items():
                try:
                    status = sensor.get_status()
                    self.node.get_logger().warning(f"Sensor {sensor_id} status: {status}")
                except Exception as e:
                    self.node.get_logger().error(f"Error getting status for faulty sensor {sensor_id}: {e}")

    def update_sensor_params(self, sensor_id: str, new_params: Dict[str, Any]):
        """
        Update operational parameters for a specific sensor.

        Args:
            sensor_id: ID of the sensor to update
            new_params: New operational parameters
        """
        if sensor_id in self.sensors:
            self.sensors[sensor_id].update_operational_params(new_params)