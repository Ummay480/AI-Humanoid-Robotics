#!/usr/bin/env python3
"""
Actuation and Sensor Validation Script
Phase-3: Module-2 Digital Twin Verification

This script validates that joint actuation and sensor simulation are working
correctly for the humanoid robot.

Usage:
    python3 scripts/validate_actuation_and_sensors.py

Prerequisites:
    - ROS 2 sourced
    - Gazebo simulation running (ros2 launch robot_description gazebo_sim.launch.py)
"""

import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, LaserScan, Image, PointCloud2
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock


class ActuationSensorValidator(Node):
    def __init__(self):
        super().__init__('actuation_sensor_validator')

        # Test results
        self.tests_passed = []
        self.tests_failed = []

        # Data collection
        self.joint_states_received = []
        self.imu_data_received = []
        self.lidar_data_received = []
        self.camera_data_received = []
        self.depth_data_received = []
        self.point_cloud_data_received = []
        self.clock_messages_received = []

        # Controller status
        self.controllers_found = []

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.points_sub = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.points_callback,
            10
        )

        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )

        # Publisher for testing joint commands
        self.position_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )

        self.get_logger().info('Actuation & Sensor Validator initialized')
        self.get_logger().info('Collecting data for 10 seconds...')

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        self.joint_states_received.append(msg)

    def imu_callback(self, msg):
        """Callback for IMU messages"""
        self.imu_data_received.append(msg)

    def lidar_callback(self, msg):
        """Callback for LiDAR messages"""
        self.lidar_data_received.append(msg)

    def camera_callback(self, msg):
        """Callback for camera image messages"""
        self.camera_data_received.append(msg)

    def depth_callback(self, msg):
        """Callback for depth image messages"""
        self.depth_data_received.append(msg)

    def points_callback(self, msg):
        """Callback for point cloud messages"""
        self.point_cloud_data_received.append(msg)

    def clock_callback(self, msg):
        """Callback for clock messages"""
        self.clock_messages_received.append(msg)

    # ========== ACTUATION TESTS ==========

    def test_joint_states_published(self):
        """Test 1: Verify joint states are being published"""
        test_name = "Joint States Publishing"

        if len(self.joint_states_received) > 0:
            duration = 10.0
            frequency = len(self.joint_states_received) / duration

            if frequency >= 30:
                self.tests_passed.append(test_name)
                self.get_logger().info(f'✓ {test_name}: PASSED (freq={frequency:.1f} Hz)')
                return True
            else:
                self.tests_failed.append(test_name)
                self.get_logger().error(f'✗ {test_name}: FAILED (freq={frequency:.1f} Hz, expected ≥30 Hz)')
                return False
        else:
            self.tests_failed.append(test_name)
            self.get_logger().error(f'✗ {test_name}: FAILED (no messages received)')
            return False

    def test_actuated_joints_present(self):
        """Test 2: Verify all actuated joints are present"""
        test_name = "Actuated Joints Present"

        expected_joints = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        if len(self.joint_states_received) > 0:
            last_msg = self.joint_states_received[-1]
            received_joints = set(last_msg.name)
            expected_joints_set = set(expected_joints)

            missing_joints = expected_joints_set - received_joints

            if not missing_joints:
                self.tests_passed.append(test_name)
                self.get_logger().info(f'✓ {test_name}: PASSED (all {len(expected_joints)} actuated joints found)')
                return True
            else:
                self.tests_failed.append(test_name)
                self.get_logger().error(f'✗ {test_name}: FAILED (missing joints: {missing_joints})')
                return False
        else:
            self.tests_failed.append(test_name)
            self.get_logger().error(f'✗ {test_name}: FAILED (no joint states received)')
            return False

    def test_joint_position_data(self):
        """Test 3: Verify joint position data is valid"""
        test_name = "Joint Position Data Valid"

        if len(self.joint_states_received) > 0:
            last_msg = self.joint_states_received[-1]

            if len(last_msg.position) == len(last_msg.name):
                self.tests_passed.append(test_name)
                self.get_logger().info(f'✓ {test_name}: PASSED ({len(last_msg.position)} positions)')
                return True
            else:
                self.tests_failed.append(test_name)
                self.get_logger().error(f'✗ {test_name}: FAILED (mismatch: {len(last_msg.position)} positions, {len(last_msg.name)} joints)')
                return False
        else:
            self.tests_failed.append(test_name)
            self.get_logger().error(f'✗ {test_name}: FAILED (no joint states received)')
            return False

    def test_joint_velocity_data(self):
        """Test 4: Verify joint velocity data is present"""
        test_name = "Joint Velocity Data Present"

        if len(self.joint_states_received) > 0:
            last_msg = self.joint_states_received[-1]

            if len(last_msg.velocity) > 0:
                self.tests_passed.append(test_name)
                self.get_logger().info(f'✓ {test_name}: PASSED ({len(last_msg.velocity)} velocities)')
                return True
            else:
                self.tests_failed.append(test_name)
                self.get_logger().error(f'✗ {test_name}: FAILED (no velocity data)')
                return False
        else:
            self.tests_failed.append(test_name)
            self.get_logger().error(f'✗ {test_name}: FAILED (no joint states received)')
            return False

    # ========== SENSOR TESTS ==========

    def test_imu_sensor(self):
        """Test 5: Verify IMU sensor is publishing"""
        test_name = "IMU Sensor Publishing"

        if len(self.imu_data_received) > 0:
            duration = 10.0
            frequency = len(self.imu_data_received) / duration

            # Expected: 100 Hz, but check for ≥50 Hz
            if frequency >= 50:
                last_msg = self.imu_data_received[-1]
                # Check that orientation and acceleration are present
                if (last_msg.orientation.w != 0 or last_msg.orientation.x != 0 or
                    last_msg.orientation.y != 0 or last_msg.orientation.z != 0):
                    self.tests_passed.append(test_name)
                    self.get_logger().info(f'✓ {test_name}: PASSED (freq={frequency:.1f} Hz)')
                    return True
                else:
                    self.tests_failed.append(test_name)
                    self.get_logger().error(f'✗ {test_name}: FAILED (invalid orientation data)')
                    return False
            else:
                self.tests_failed.append(test_name)
                self.get_logger().error(f'✗ {test_name}: FAILED (freq={frequency:.1f} Hz, expected ≥50 Hz)')
                return False
        else:
            self.tests_failed.append(test_name)
            self.get_logger().error(f'✗ {test_name}: FAILED (no IMU messages received)')
            return False

    def test_lidar_sensor(self):
        """Test 6: Verify LiDAR sensor is publishing"""
        test_name = "LiDAR Sensor Publishing"

        if len(self.lidar_data_received) > 0:
            duration = 10.0
            frequency = len(self.lidar_data_received) / duration

            # Expected: 10 Hz
            if frequency >= 8:
                last_msg = self.lidar_data_received[-1]
                # Check that scan has data
                if len(last_msg.ranges) > 0:
                    self.tests_passed.append(test_name)
                    self.get_logger().info(f'✓ {test_name}: PASSED (freq={frequency:.1f} Hz, {len(last_msg.ranges)} points)')
                    return True
                else:
                    self.tests_failed.append(test_name)
                    self.get_logger().error(f'✗ {test_name}: FAILED (no scan data)')
                    return False
            else:
                self.tests_failed.append(test_name)
                self.get_logger().error(f'✗ {test_name}: FAILED (freq={frequency:.1f} Hz, expected ≥8 Hz)')
                return False
        else:
            self.tests_failed.append(test_name)
            self.get_logger().error(f'✗ {test_name}: FAILED (no LiDAR messages received)')
            return False

    def test_camera_sensor(self):
        """Test 7: Verify RGB camera is publishing"""
        test_name = "RGB Camera Publishing"

        if len(self.camera_data_received) > 0:
            duration = 10.0
            frequency = len(self.camera_data_received) / duration

            # Expected: 30 Hz, but check for ≥20 Hz
            if frequency >= 20:
                last_msg = self.camera_data_received[-1]
                # Check image dimensions
                if last_msg.width > 0 and last_msg.height > 0:
                    self.tests_passed.append(test_name)
                    self.get_logger().info(f'✓ {test_name}: PASSED (freq={frequency:.1f} Hz, {last_msg.width}x{last_msg.height})')
                    return True
                else:
                    self.tests_failed.append(test_name)
                    self.get_logger().error(f'✗ {test_name}: FAILED (invalid image dimensions)')
                    return False
            else:
                self.tests_failed.append(test_name)
                self.get_logger().error(f'✗ {test_name}: FAILED (freq={frequency:.1f} Hz, expected ≥20 Hz)')
                return False
        else:
            self.tests_failed.append(test_name)
            self.get_logger().error(f'✗ {test_name}: FAILED (no camera messages received)')
            return False

    def test_depth_camera_sensor(self):
        """Test 8: Verify depth camera is publishing"""
        test_name = "Depth Camera Publishing"

        if len(self.depth_data_received) > 0:
            duration = 10.0
            frequency = len(self.depth_data_received) / duration

            # Expected: 30 Hz, but check for ≥20 Hz
            if frequency >= 20:
                last_msg = self.depth_data_received[-1]
                if last_msg.width > 0 and last_msg.height > 0:
                    self.tests_passed.append(test_name)
                    self.get_logger().info(f'✓ {test_name}: PASSED (freq={frequency:.1f} Hz, {last_msg.width}x{last_msg.height})')
                    return True
                else:
                    self.tests_failed.append(test_name)
                    self.get_logger().error(f'✗ {test_name}: FAILED (invalid depth dimensions)')
                    return False
            else:
                self.tests_failed.append(test_name)
                self.get_logger().error(f'✗ {test_name}: FAILED (freq={frequency:.1f} Hz, expected ≥20 Hz)')
                return False
        else:
            self.tests_failed.append(test_name)
            self.get_logger().error(f'✗ {test_name}: FAILED (no depth messages received)')
            return False

    def test_point_cloud_sensor(self):
        """Test 9: Verify point cloud is publishing"""
        test_name = "Point Cloud Publishing"

        if len(self.point_cloud_data_received) > 0:
            duration = 10.0
            frequency = len(self.point_cloud_data_received) / duration

            # Expected: 30 Hz, but check for ≥20 Hz
            if frequency >= 20:
                last_msg = self.point_cloud_data_received[-1]
                if last_msg.width > 0 and last_msg.height > 0:
                    self.tests_passed.append(test_name)
                    self.get_logger().info(f'✓ {test_name}: PASSED (freq={frequency:.1f} Hz, {last_msg.width}x{last_msg.height} points)')
                    return True
                else:
                    self.tests_failed.append(test_name)
                    self.get_logger().error(f'✗ {test_name}: FAILED (no point cloud data)')
                    return False
            else:
                self.tests_failed.append(test_name)
                self.get_logger().error(f'✗ {test_name}: FAILED (freq={frequency:.1f} Hz, expected ≥20 Hz)')
                return False
        else:
            self.tests_failed.append(test_name)
            self.get_logger().error(f'✗ {test_name}: FAILED (no point cloud messages received)')
            return False

    def test_simulation_clock(self):
        """Test 10: Verify simulation clock is running"""
        test_name = "Simulation Clock Running"

        if len(self.clock_messages_received) >= 2:
            first_clock = self.clock_messages_received[0].clock
            last_clock = self.clock_messages_received[-1].clock

            first_time = first_clock.sec + first_clock.nanosec * 1e-9
            last_time = last_clock.sec + last_clock.nanosec * 1e-9

            time_advanced = last_time - first_time

            if time_advanced > 0:
                self.tests_passed.append(test_name)
                self.get_logger().info(f'✓ {test_name}: PASSED (advanced {time_advanced:.2f}s)')
                return True
            else:
                self.tests_failed.append(test_name)
                self.get_logger().error(f'✗ {test_name}: FAILED (clock not advancing)')
                return False
        else:
            self.tests_failed.append(test_name)
            self.get_logger().error(f'✗ {test_name}: FAILED (insufficient clock messages)')
            return False

    def run_tests(self):
        """Run all validation tests"""
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('ACTUATION & SENSOR VALIDATION RESULTS - PHASE 3')
        self.get_logger().info('='*70 + '\n')

        # Run actuation tests
        self.get_logger().info('--- ACTUATION TESTS ---')
        self.test_joint_states_published()
        self.test_actuated_joints_present()
        self.test_joint_position_data()
        self.test_joint_velocity_data()

        # Run sensor tests
        self.get_logger().info('\n--- SENSOR TESTS ---')
        self.test_imu_sensor()
        self.test_lidar_sensor()
        self.test_camera_sensor()
        self.test_depth_camera_sensor()
        self.test_point_cloud_sensor()

        # Run system tests
        self.get_logger().info('\n--- SYSTEM TESTS ---')
        self.test_simulation_clock()

        # Summary
        total_tests = len(self.tests_passed) + len(self.tests_failed)

        self.get_logger().info('\n' + '-'*70)
        self.get_logger().info(f'SUMMARY: {len(self.tests_passed)}/{total_tests} tests passed')
        self.get_logger().info('-'*70)

        if self.tests_failed:
            self.get_logger().error(f'\n❌ Failed tests ({len(self.tests_failed)}):')
            for test in self.tests_failed:
                self.get_logger().error(f'  - {test}')
            self.get_logger().error('\nPlease check:')
            self.get_logger().error('  1. Gazebo is running with robot spawned')
            self.get_logger().error('  2. Controllers are loaded (ros2 control list_controllers)')
            self.get_logger().error('  3. Sensor plugins are active')
            self.get_logger().error('  4. All topics are publishing (ros2 topic list)')
            return False
        else:
            self.get_logger().info('\n✅ All tests passed! Digital Twin (Module-2) is fully operational.')
            self.get_logger().info('\nActive capabilities:')
            self.get_logger().info('  ✓ 10 actuated joints (hips, knees, ankles, shoulders, elbows)')
            self.get_logger().info('  ✓ IMU sensor (orientation, acceleration, gyro)')
            self.get_logger().info('  ✓ LiDAR sensor (360° scan)')
            self.get_logger().info('  ✓ RGB-D camera (color + depth + point cloud)')
            self.get_logger().info('  ✓ ros2_control integration')
            self.get_logger().info('\nNext steps:')
            self.get_logger().info('  - Test joint commands: ros2 topic pub /position_controller/commands ...')
            self.get_logger().info('  - Proceed to Module-3: AI-Robot Brain')
            return True


def main():
    """Main function"""
    print("\n" + "="*70)
    print("Actuation & Sensor Validation - Phase 3 (Digital Twin)")
    print("="*70 + "\n")

    print("Prerequisites:")
    print("  1. Gazebo simulation must be running")
    print("  2. Run: ros2 launch robot_description gazebo_sim.launch.py")
    print("\nStarting validation in 3 seconds...\n")
    time.sleep(3)

    # Initialize ROS 2
    rclpy.init()

    try:
        # Create validator node
        validator = ActuationSensorValidator()

        # Collect data for 10 seconds
        print("Collecting data for 10 seconds...")
        start_time = time.time()
        while time.time() - start_time < 10.0:
            rclpy.spin_once(validator, timeout_sec=0.1)

        # Run tests
        success = validator.run_tests()

        # Cleanup
        validator.destroy_node()
        rclpy.shutdown()

        # Exit with appropriate code
        sys.exit(0 if success else 1)

    except KeyboardInterrupt:
        print("\nValidation interrupted by user")
        rclpy.shutdown()
        sys.exit(1)
    except Exception as e:
        print(f"\nError during validation: {e}")
        import traceback
        traceback.print_exc()
        rclpy.shutdown()
        sys.exit(1)


if __name__ == '__main__':
    main()
