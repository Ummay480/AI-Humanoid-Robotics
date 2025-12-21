#!/usr/bin/env python3

"""
Joint State Publisher - ROS2 Publisher example for robot sensor data
Implements User Story 3: Robot Control Topics and Services - Publisher component
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time


class JointStatePublisher(Node):
    """
    Publisher that simulates joint state data from robot sensors
    Publishes joint positions, velocities, and efforts at a specified rate
    """

    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create publisher for joint states
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Create timer to publish at 50Hz (0.02 seconds)
        self.timer = self.create_timer(0.02, self.publish_joint_state)

        # Initialize joint names and starting positions
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.initial_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.time_start = time.time()

        self.get_logger().info('Joint State Publisher initialized')
        self.get_logger().info(f'Publishing to /joint_states at 50Hz')

    def publish_joint_state(self):
        """
        Publish joint state message with simulated data
        """
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Set joint names
        msg.name = self.joint_names

        # Generate simulated joint positions (oscillating pattern)
        current_time = time.time() - self.time_start
        positions = []
        velocities = []
        efforts = []

        for i, name in enumerate(self.joint_names):
            # Create different oscillating patterns for each joint
            pos = math.sin(current_time * (i + 1) * 0.5) * (i + 1) * 0.2
            vel = math.cos(current_time * (i + 1) * 0.5) * (i + 1) * 0.2 * 0.5  # derivative
            effort = 0.0  # For simulation purposes

            positions.append(pos)
            velocities.append(vel)
            efforts.append(effort)

        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts

        # Publish the message
        self.publisher.publish(msg)

        # Log every 100 messages to avoid spam
        if int(current_time * 50) % 100 == 0:
            self.get_logger().debug(f'Published joint states: positions={positions}')


def main(args=None):
    """
    Main function to run the joint state publisher
    """
    rclpy.init(args=args)

    joint_state_publisher = JointStatePublisher()

    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        joint_state_publisher.get_logger().info('Joint State Publisher stopped by user')
    finally:
        joint_state_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()