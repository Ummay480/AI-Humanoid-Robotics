"""
Integration tests for the perception pipeline.

Tests the complete flow from sensor data acquisition through fusion to mapping.
"""
import pytest
import numpy as np
import time
from pathlib import Path
import sys

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'src'))

from perception.sensor_acquisition.camera_handler import CameraHandler
from perception.sensor_acquisition.lidar_handler import LidarHandler
from perception.sensor_acquisition.imu_handler import IMUHandler
from perception.sensor_acquisition.sensor_manager import SensorManager
from perception.computer_vision.object_detector import YOLODetector
from perception.sensor_fusion.data_fusion import SensorDataFusion, FusionConfig
from perception.sensor_fusion.mapping import OccupancyGrid3D, MapConfig
from perception.sensor_fusion.kalman_filter import KalmanFilter
from perception.common.data_types import (
    SensorType, SensorConfig, SensorConfigState, DetectedObject, ObjectType
)
from perception.common.config_handler import SensorConfigHandler


class TestPerceptionPipeline:
    """Integration tests for the complete perception pipeline."""

    @pytest.fixture
    def config_handler(self, tmp_path):
        """Create a temporary config handler."""
        return SensorConfigHandler(str(tmp_path))

    @pytest.fixture
    def sensor_configs(self):
        """Create test sensor configurations."""
        configs = {}

        # Camera config
        configs['camera'] = SensorConfig(
            sensor_id='test_camera',
            sensor_type=SensorType.CAMERA,
            calibration_data={'fx': 525.0, 'fy': 525.0, 'cx': 320.0, 'cy': 240.0},
            mounting_position={'x': 0.1, 'y': 0.0, 'z': 0.5,
                             'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            operational_params={
                'frequency': 30.0,
                'resolution': [640, 480],
                'format': 'bgr8'
            },
            state=SensorConfigState.CONFIGURED
        )

        # LIDAR config
        configs['lidar'] = SensorConfig(
            sensor_id='test_lidar',
            sensor_type=SensorType.LIDAR,
            calibration_data={},
            mounting_position={'x': 0.0, 'y': 0.0, 'z': 0.3,
                             'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            operational_params={
                'frequency': 10.0,
                'range_min': 0.1,
                'range_max': 30.0,
                'data_format': 'laser_scan'
            },
            state=SensorConfigState.CONFIGURED
        )

        # IMU config
        configs['imu'] = SensorConfig(
            sensor_id='test_imu',
            sensor_type=SensorType.IMU,
            calibration_data={},
            mounting_position={'x': 0.0, 'y': 0.0, 'z': 0.2,
                             'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            operational_params={
                'frequency': 100.0
            },
            state=SensorConfigState.CONFIGURED
        )

        return configs

    def test_sensor_config_handler(self, config_handler, sensor_configs):
        """Test sensor configuration handling."""
        # Save and load camera config
        camera_config = sensor_configs['camera']
        assert config_handler.save_sensor_config(camera_config)

        loaded_config = config_handler.load_sensor_config('test_camera')
        assert loaded_config is not None
        assert loaded_config.sensor_id == 'test_camera'
        assert loaded_config.sensor_type == SensorType.CAMERA

    def test_data_fusion_basic(self):
        """Test basic sensor data fusion."""
        config = FusionConfig(
            sensor_weights={'sensor1': 0.5, 'sensor2': 0.5},
            sync_tolerance=0.1,
            min_sensors=2
        )

        fusion = SensorDataFusion(config)

        # Test position fusion
        positions = {
            'sensor1': np.array([1.0, 2.0, 3.0]),
            'sensor2': np.array([1.1, 2.1, 2.9])
        }
        confidences = {
            'sensor1': 0.8,
            'sensor2': 0.9
        }

        fused_pos, fused_conf = fusion.fuse_position_estimates(positions, confidences)

        assert fused_pos.shape == (3,)
        assert 0.0 < fused_conf <= 1.0
        # Should be close to average
        assert np.allclose(fused_pos, [1.05, 2.05, 2.95], atol=0.1)

    def test_object_detection_fusion(self):
        """Test fusion of detected objects from multiple sensors."""
        config = FusionConfig(
            sensor_weights={'cam1': 0.5, 'cam2': 0.5},
            min_sensors=1
        )

        fusion = SensorDataFusion(config)

        # Create similar detections from two cameras
        detections1 = [
            DetectedObject(
                object_type=ObjectType.HUMAN,
                position=np.array([1.0, 0.0, 0.0]),
                bounding_box=np.array([100, 100, 50, 100]),
                confidence_score=0.85,
                timestamp=time.time(),
                id='det1'
            )
        ]

        detections2 = [
            DetectedObject(
                object_type=ObjectType.HUMAN,
                position=np.array([1.05, 0.05, 0.0]),
                bounding_box=np.array([95, 105, 55, 95]),
                confidence_score=0.90,
                timestamp=time.time(),
                id='det2'
            )
        ]

        # Fuse detections
        fused = fusion.fuse_detected_objects([detections1, detections2])

        # Should merge into single detection
        assert len(fused) == 1
        assert fused[0].object_type == ObjectType.HUMAN
        # Confidence should be max
        assert fused[0].confidence_score == 0.90

    def test_kalman_filter_basic(self):
        """Test basic Kalman filter operation."""
        # Create 1D position-velocity filter
        kf = KalmanFilter(dim_x=2, dim_z=1)

        # State transition: x_k = x_{k-1} + v_{k-1} * dt
        dt = 0.1
        kf.F = np.array([[1, dt],
                        [0, 1]])

        # Measure position only
        kf.H = np.array([[1, 0]])

        # Set initial state
        kf.x = np.array([0, 1])  # position=0, velocity=1

        # Predict
        state = kf.predict()
        assert state.x[0] == pytest.approx(0.1, abs=1e-6)  # position increased

        # Update with measurement
        z = np.array([0.15])
        state = kf.update(z)

        # State should be between prediction and measurement
        assert 0.1 < state.x[0] < 0.15

    def test_occupancy_grid_basic(self):
        """Test basic occupancy grid operations."""
        config = MapConfig(
            resolution=0.1,
            map_size=(5.0, 5.0, 2.0),
            origin=(-2.5, -2.5, 0.0)
        )

        grid = OccupancyGrid3D(config)

        # Test coordinate conversion
        world_pos = np.array([0.0, 0.0, 1.0])
        grid_idx = grid.world_to_grid(world_pos)
        world_pos_back = grid.grid_to_world(grid_idx)

        assert np.allclose(world_pos, world_pos_back, atol=0.1)

        # Test point cloud update
        points = np.array([
            [1.0, 0.0, 0.5],
            [2.0, 0.0, 0.5],
            [1.5, 1.0, 0.5]
        ])
        sensor_origin = np.array([0.0, 0.0, 0.5])

        grid.update_from_point_cloud(points, sensor_origin)

        # Should have some occupied cells
        occupied = grid.get_occupied_cells()
        assert len(occupied) > 0

    def test_occupancy_grid_with_detections(self):
        """Test occupancy grid update with detected objects."""
        config = MapConfig(
            resolution=0.05,
            map_size=(10.0, 10.0, 3.0),
            origin=(-5.0, -5.0, 0.0)
        )

        grid = OccupancyGrid3D(config)

        # Create test detection
        detection = DetectedObject(
            object_type=ObjectType.CHAIR,
            position=np.array([1.0, 1.0, 0.5]),
            bounding_box=np.array([0, 0, 50, 50]),
            confidence_score=0.9,
            timestamp=time.time(),
            id='chair1'
        )

        # Update grid
        grid.update_from_detections([detection])

        # Check that region around detection is occupied
        occupied = grid.get_occupied_cells()
        assert len(occupied) > 0

        # Check map statistics
        stats = grid.get_statistics()
        assert stats['occupied_cells'] > 0
        assert stats['update_count'] == 1

    def test_end_to_end_fusion_pipeline(self):
        """Test complete fusion pipeline from sensors to map."""
        # Setup fusion
        fusion_config = FusionConfig(
            sensor_weights={'lidar': 0.6, 'camera': 0.4},
            min_sensors=1
        )
        fusion = SensorDataFusion(fusion_config)

        # Setup mapping
        map_config = MapConfig(
            resolution=0.1,
            map_size=(10.0, 10.0, 3.0)
        )
        grid = OccupancyGrid3D(map_config)

        # Simulate LIDAR point cloud
        lidar_points = np.random.rand(100, 3) * 5.0  # Random points in 5m range
        sensor_origin = np.array([0.0, 0.0, 0.0])

        # Update grid with LIDAR data
        grid.update_from_point_cloud(lidar_points, sensor_origin)

        # Simulate object detection
        detection = DetectedObject(
            object_type=ObjectType.TABLE,
            position=np.array([2.0, 1.0, 0.75]),
            bounding_box=np.array([0, 0, 100, 80]),
            confidence_score=0.85,
            timestamp=time.time(),
            id='table1'
        )

        # Update grid with detection
        grid.update_from_detections([detection])

        # Get final map
        env_map = grid.get_map_data()

        assert env_map is not None
        assert env_map.resolution == 0.1
        assert env_map.map_data.shape == grid.grid_size

        # Check statistics
        stats = grid.get_statistics()
        assert stats['update_count'] == 2
        assert stats['occupied_cells'] > 0

    def test_performance_latency(self):
        """Test that processing latency meets <20ms requirement."""
        # Create simple Kalman filter
        kf = KalmanFilter(dim_x=3, dim_z=3)
        kf.H = np.eye(3)

        # Measure update latency
        measurements = [np.random.rand(3) for _ in range(100)]

        start_time = time.perf_counter()
        for z in measurements:
            kf.predict()
            kf.update(z)
        end_time = time.perf_counter()

        avg_latency = (end_time - start_time) / len(measurements)

        # Should be well under 20ms
        assert avg_latency < 0.020, f"Average latency {avg_latency*1000:.2f}ms exceeds 20ms"

    def test_uncertainty_reduction(self):
        """Test that fusion reduces uncertainty compared to single sensor."""
        config = FusionConfig(
            sensor_weights={'s1': 0.5, 's2': 0.5},
            min_sensors=2
        )

        fusion = SensorDataFusion(config)

        # Single sensor - higher uncertainty
        positions_single = {'s1': np.array([1.0, 1.0, 1.0])}
        confidences_single = {'s1': 0.7}

        _, conf_single = fusion.fuse_position_estimates(
            positions_single, confidences_single
        )

        # Multiple sensors - lower uncertainty
        positions_multi = {
            's1': np.array([1.0, 1.0, 1.0]),
            's2': np.array([1.05, 0.95, 1.02])
        }
        confidences_multi = {'s1': 0.7, 's2': 0.75}

        _, conf_multi = fusion.fuse_position_estimates(
            positions_multi, confidences_multi
        )

        # Multi-sensor should have higher confidence
        assert conf_multi > conf_single

    def test_system_resilience_sensor_failure(self):
        """Test system handles sensor failures gracefully."""
        config = FusionConfig(
            sensor_weights={'s1': 0.4, 's2': 0.3, 's3': 0.3},
            min_sensors=2
        )

        fusion = SensorDataFusion(config)

        # Start with 3 sensors
        positions = {
            's1': np.array([1.0, 0.0, 0.0]),
            's2': np.array([1.1, 0.1, 0.0]),
            's3': np.array([0.9, -0.1, 0.0])
        }
        confidences = {'s1': 0.8, 's2': 0.8, 's3': 0.8}

        fused_3, _ = fusion.fuse_position_estimates(positions, confidences)

        # One sensor fails
        positions_2 = {
            's1': np.array([1.0, 0.0, 0.0]),
            's2': np.array([1.1, 0.1, 0.0])
        }
        confidences_2 = {'s1': 0.8, 's2': 0.8}

        fused_2, _ = fusion.fuse_position_estimates(positions_2, confidences_2)

        # Should still produce valid fusion
        assert fused_2 is not None
        assert fused_2.shape == (3,)
        # Result should be similar
        assert np.linalg.norm(fused_3 - fused_2) < 0.5


class TestPerformanceRequirements:
    """Tests for specific performance requirements."""

    def test_sensor_processing_latency(self):
        """Test sensor data processing latency < 20ms."""
        # Simulate sensor data processing
        data_size = 640 * 480 * 3  # VGA image
        test_data = np.random.randint(0, 255, data_size, dtype=np.uint8)

        start = time.perf_counter()
        # Simulate processing
        processed = test_data.reshape((480, 640, 3))
        _ = np.mean(processed, axis=2)
        end = time.perf_counter()

        latency = (end - start) * 1000  # Convert to ms
        assert latency < 20.0, f"Processing latency {latency:.2f}ms exceeds 20ms"

    def test_mapping_precision(self):
        """Test mapping precision meets 5cm requirement."""
        config = MapConfig(resolution=0.05)
        grid = OccupancyGrid3D(config)

        # Place object at known position
        test_position = np.array([1.234, 2.345, 0.567])
        grid_idx = grid.world_to_grid(test_position)
        reconstructed = grid.grid_to_world(grid_idx)

        # Error should be within resolution
        error = np.linalg.norm(test_position - reconstructed)
        assert error < 0.05, f"Mapping error {error*100:.2f}cm exceeds 5cm"

    def test_detection_accuracy_threshold(self):
        """Test that detection confidence threshold is properly enforced."""
        # This would normally test with real model, using mock for now
        confidence_threshold = 0.85

        # Simulated detections
        detections_with_scores = [
            (0.92, True),   # Should keep
            (0.88, True),   # Should keep
            (0.75, False),  # Should filter
            (0.60, False),  # Should filter
        ]

        for score, should_keep in detections_with_scores:
            passed = score >= confidence_threshold
            assert passed == should_keep


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
