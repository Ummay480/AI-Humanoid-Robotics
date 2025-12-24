#!/usr/bin/env python3
"""
Gazebo Physics Validation Script
Phase-2: Module-2 Verification

This script validates that Gazebo physics simulation is working correctly
for the humanoid robot.

Usage:
    python3 scripts/validate_gazebo_physics.py

Prerequisites:
    - ROS 2 sourced
    - Gazebo simulation running (ros2 launch robot_description gazebo_sim.launch.py)
"""

import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock


class GazeboPhysicsValidator(Node):
    def __init__(self):
        super().__init__('gazebo_physics_validator')

        # Test results
        self.tests_passed = []
        self.tests_failed = []

        # Data collection
        self.joint_states_received = []
        self.tf_messages_received = []
        self.clock_messages_received = []

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )

        self.get_logger().info('Gazebo Physics Validator initialized')
        self.get_logger().info('Collecting data for 5 seconds...')

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        self.joint_states_received.append(msg)

    def tf_callback(self, msg):
        """Callback for TF messages"""
        self.tf_messages_received.append(msg)

    def clock_callback(self, msg):
        """Callback for clock messages"""
        self.clock_messages_received.append(msg)

    def test_joint_states_published(self):
        """Test 1: Verify joint states are being published"""
        test_name = "Joint States Publishing"

        if len(self.joint_states_received) > 0:
            # Calculate frequency
            duration = 5.0  # Collection duration in seconds
            frequency = len(self.joint_states_received) / duration

            # Check frequency is reasonable (30-60 Hz expected)
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

    def test_joint_names_present(self):
        """Test 2: Verify all expected joints are present"""
        test_name = "Joint Names Present"

        expected_joints = [
            'left_leg_joint',
            'right_leg_joint',
            'left_foot_joint',
            'right_foot_joint',
            'left_arm_joint',
            'right_arm_joint',
            'head_joint'
        ]

        if len(self.joint_states_received) > 0:
            # Get joint names from last message
            last_msg = self.joint_states_received[-1]
            received_joints = set(last_msg.name)
            expected_joints_set = set(expected_joints)

            missing_joints = expected_joints_set - received_joints

            if not missing_joints:
                self.tests_passed.append(test_name)
                self.get_logger().info(f'✓ {test_name}: PASSED (all {len(expected_joints)} joints found)')
                return True
            else:
                self.tests_failed.append(test_name)
                self.get_logger().error(f'✗ {test_name}: FAILED (missing joints: {missing_joints})')
                return False
        else:
            self.tests_failed.append(test_name)
            self.get_logger().error(f'✗ {test_name}: FAILED (no joint states received)')
            return False

    def test_tf_published(self):
        """Test 3: Verify TF transforms are being published"""
        test_name = "TF Publishing"

        if len(self.tf_messages_received) > 0:
            # Count unique frames
            frames = set()
            for msg in self.tf_messages_received:
                for transform in msg.transforms:
                    frames.add(transform.header.frame_id)
                    frames.add(transform.child_frame_id)

            expected_frames = [
                'base_link', 'left_leg', 'right_leg',
                'left_foot', 'right_foot', 'left_arm',
                'right_arm', 'head'
            ]

            missing_frames = set(expected_frames) - frames

            if not missing_frames:
                self.tests_passed.append(test_name)
                self.get_logger().info(f'✓ {test_name}: PASSED ({len(frames)} frames)')
                return True
            else:
                self.tests_failed.append(test_name)
                self.get_logger().error(f'✗ {test_name}: FAILED (missing frames: {missing_frames})')
                return False
        else:
            self.tests_failed.append(test_name)
            self.get_logger().error(f'✗ {test_name}: FAILED (no TF messages received)')
            return False

    def test_simulation_clock(self):
        """Test 4: Verify simulation clock is running"""
        test_name = "Simulation Clock"

        if len(self.clock_messages_received) >= 2:
            # Check if clock is advancing
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
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('GAZEBO PHYSICS VALIDATION RESULTS')
        self.get_logger().info('='*60)

        # Run tests
        self.test_joint_states_published()
        self.test_joint_names_present()
        self.test_tf_published()
        self.test_simulation_clock()

        # Summary
        total_tests = len(self.tests_passed) + len(self.tests_failed)

        self.get_logger().info('\n' + '-'*60)
        self.get_logger().info(f'SUMMARY: {len(self.tests_passed)}/{total_tests} tests passed')
        self.get_logger().info('-'*60)

        if self.tests_failed:
            self.get_logger().error(f'\nFailed tests:')
            for test in self.tests_failed:
                self.get_logger().error(f'  - {test}')
            self.get_logger().error('\nPlease check:')
            self.get_logger().error('  1. Gazebo is running')
            self.get_logger().error('  2. Robot is spawned')
            self.get_logger().error('  3. All plugins are loaded')
            return False
        else:
            self.get_logger().info('\n✓ All tests passed! Gazebo physics simulation is working correctly.')
            self.get_logger().info('\nNext steps:')
            self.get_logger().info('  - Proceed to Phase-3: Joint Control & Actuation')
            self.get_logger().info('  - Add actuated joints and controllers')
            return True


def main():
    """Main function"""
    print("\n" + "="*60)
    print("Gazebo Physics Validation - Phase 2")
    print("="*60 + "\n")

    # Check if Gazebo is running
    print("Prerequisites:")
    print("  1. Gazebo simulation must be running")
    print("  2. Run: ros2 launch robot_description gazebo_sim.launch.py")
    print("\nStarting validation in 3 seconds...\n")
    time.sleep(3)

    # Initialize ROS 2
    rclpy.init()

    try:
        # Create validator node
        validator = GazeboPhysicsValidator()

        # Collect data for 5 seconds
        start_time = time.time()
        while time.time() - start_time < 5.0:
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
        rclpy.shutdown()
        sys.exit(1)


if __name__ == '__main__':
    main()
