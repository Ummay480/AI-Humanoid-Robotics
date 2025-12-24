#!/usr/bin/env python3
"""
Sensor Fusion Demonstration

Shows how to use Kalman filtering and sensor fusion for position tracking.
Demonstrates fusing data from multiple sensors with different confidence levels.
"""

import numpy as np
import time
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from src.perception.sensor_fusion.kalman_filter import (
    MultiSensorKalmanFilter,
    create_position_velocity_filter
)
from src.perception.sensor_fusion.data_fusion import SensorDataFusion


def simulate_sensor_measurements(true_position, sensor_noise_levels):
    """
    Simulate noisy sensor measurements.

    Args:
        true_position: True 3D position
        sensor_noise_levels: Dict of sensor ID to noise std dev

    Returns:
        Dict of sensor measurements
    """
    measurements = {}
    for sensor_id, noise_std in sensor_noise_levels.items():
        noise = np.random.normal(0, noise_std, size=3)
        measurements[sensor_id] = true_position + noise

    return measurements


def main():
    """Run sensor fusion demo."""
    print("=" * 60)
    print("Sensor Fusion Demonstration")
    print("=" * 60)

    # Setup
    print("\n1. Setting up Kalman filter and fusion engine...")

    # Create multi-sensor Kalman filter for position tracking
    # State: [x, y, z, vx, vy, vz] - position and velocity
    kf = create_position_velocity_filter(dim=3, dt=0.1)

    # Create fusion engine
    fusion = SensorDataFusion()

    # Initialize with first position
    initial_position = np.array([0.0, 0.0, 0.0])
    kf.initialize(initial_position)
    print(f"   Initialized at position: {initial_position}")

    # Simulate tracking a moving object
    print("\n2. Simulating object motion and sensor measurements...")

    # Define sensor characteristics
    sensor_specs = {
        'camera': {
            'noise_std': 0.15,      # 15cm standard deviation
            'confidence': 0.7,      # 70% confidence
            'update_rate': 30       # 30 Hz
        },
        'lidar': {
            'noise_std': 0.05,      # 5cm standard deviation
            'confidence': 0.9,      # 90% confidence
            'update_rate': 10       # 10 Hz
        },
        'imu_derived': {
            'noise_std': 0.25,      # 25cm standard deviation
            'confidence': 0.5,      # 50% confidence
            'update_rate': 100      # 100 Hz
        }
    }

    print("\n   Sensor Specifications:")
    for sensor_id, specs in sensor_specs.items():
        print(f"   - {sensor_id}: noise={specs['noise_std']*100:.0f}cm, "
              f"confidence={specs['confidence']*100:.0f}%, "
              f"rate={specs['update_rate']}Hz")

    # Simulation parameters
    dt = 0.1  # 100ms timestep
    duration = 5.0  # 5 seconds
    num_steps = int(duration / dt)

    # Object motion: moves in a straight line with constant velocity
    true_velocity = np.array([1.0, 0.5, 0.0])  # 1 m/s in x, 0.5 m/s in y
    true_position = initial_position.copy()

    print(f"\n   Simulating {duration}s of motion at velocity {true_velocity}")
    print("\n3. Running fusion loop...\n")

    # Storage for results
    timestamps = []
    true_positions = []
    fused_positions = []
    filtered_positions = []
    position_errors = []

    # Simulation loop
    for step in range(num_steps):
        current_time = step * dt

        # Update true position
        true_position = initial_position + true_velocity * current_time

        # Simulate sensor measurements with different noise levels
        noise_levels = {sid: specs['noise_std']
                       for sid, specs in sensor_specs.items()}
        measurements = simulate_sensor_measurements(true_position, noise_levels)

        # Fuse measurements
        confidences = {sid: specs['confidence']
                      for sid, specs in sensor_specs.items()}

        fused_position, fused_confidence = fusion.fuse_position_estimates(
            measurements, confidences
        )

        # Update Kalman filter with fused measurement
        kf.update('fused', fused_position, timestamp=current_time)

        # Predict next state
        kf.predict()

        # Get filtered state
        state = kf.get_state()
        filtered_position = state[:3]

        # Calculate error
        error = np.linalg.norm(filtered_position - true_position)

        # Store results
        timestamps.append(current_time)
        true_positions.append(true_position.copy())
        fused_positions.append(fused_position.copy())
        filtered_positions.append(filtered_position.copy())
        position_errors.append(error)

        # Print progress every second
        if step % 10 == 0:
            print(f"   t={current_time:.1f}s: "
                  f"True={true_position}, "
                  f"Filtered={filtered_position}, "
                  f"Error={error:.3f}m")

    # Results
    print("\n4. Results Summary:")
    print("=" * 60)

    avg_error = np.mean(position_errors)
    max_error = np.max(position_errors)
    final_error = position_errors[-1]

    print(f"\n   Position Tracking Accuracy:")
    print(f"   - Average error: {avg_error*100:.2f} cm")
    print(f"   - Maximum error: {max_error*100:.2f} cm")
    print(f"   - Final error:   {final_error*100:.2f} cm")

    # Compare with unfused measurements
    print(f"\n   Sensor Noise Levels (for comparison):")
    for sensor_id, specs in sensor_specs.items():
        print(f"   - {sensor_id}: {specs['noise_std']*100:.0f} cm std dev")

    # Uncertainty reduction
    print(f"\n   Fusion Performance:")
    print(f"   - Fused confidence: {fused_confidence:.2%}")

    # Estimate velocity
    estimated_velocity = state[3:]
    velocity_error = np.linalg.norm(estimated_velocity - true_velocity)
    print(f"\n   Velocity Estimation:")
    print(f"   - True velocity:      {true_velocity}")
    print(f"   - Estimated velocity: {estimated_velocity}")
    print(f"   - Velocity error:     {velocity_error:.3f} m/s")

    print("\n" + "=" * 60)
    print("Demonstration complete!")
    print("=" * 60)

    return {
        'timestamps': timestamps,
        'true_positions': true_positions,
        'filtered_positions': filtered_positions,
        'errors': position_errors,
        'avg_error': avg_error,
        'final_state': state
    }


if __name__ == '__main__':
    try:
        results = main()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
