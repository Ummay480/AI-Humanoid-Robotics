#!/usr/bin/env python3

"""
Joint Command Subscriber - ROS2 Subscriber example for robot actuator commands
Implements User Story 3: Robot Control Topics and Services - Subscriber component
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time


class JointCommandSubscriber(Node):
    """
    Subscriber that receives joint command messages and simulates actuator responses
    """

    def __init__(self):
        super().__init__('joint_command_subscriber')

        # Create subscriber for joint commands
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.joint_command_callback,
            10
        )

        # Initialize internal state
        self.last_command_time = time.time()
        self.received_commands = []
        self.command_count = 0

        self.get_logger().info('Joint Command Subscriber initialized')
        self.get_logger().info('Subscribed to /joint_commands')

    def joint_command_callback(self, msg):
        """
        Callback function to process incoming joint command messages
        """
        current_time = time.time()
        command_interval = current_time - self.last_command_time

        # Store the received command
        self.received_commands = list(msg.data)
        self.command_count += 1

        # Log the received command (every 10 commands to avoid spam)
        if self.command_count % 10 == 0:
            self.get_logger().info(
                f'Received command #{self.command_count}: {self.received_commands}, '
                f'interval: {command_interval:.3f}s'
            )

        # Check for potential timing issues (should be around 0.02s for 50Hz)
        if command_interval > 0.05:  # More than 2.5x expected interval
            self.get_logger().warn(
                f'Large interval between commands: {command_interval:.3f}s, '
                f'expected ~0.02s for 50Hz operation'
            )

        # Simulate actuator response processing
        self.simulate_actuator_response(msg.data)

        self.last_command_time = current_time

    def simulate_actuator_response(self, commands):
        """
        Simulate the response of actuators to the received commands
        """
        # In a real robot, this would send commands to physical actuators
        # For simulation, we just log the command and simulate a response time
        if len(commands) > 0:
            # Simulate a small delay to represent actuator response time
            # time.sleep(0.001)  # 1ms simulated response time (uncomment in real use)

            # Log actuator simulation
            self.get_logger().debug(f'Actuator simulation: commanded positions {commands}')


def main(args=None):
    """
    Main function to run the joint command subscriber
    """
    rclpy.init(args=args)

    joint_command_subscriber = JointCommandSubscriber()

    try:
        rclpy.spin(joint_command_subscriber)
    except KeyboardInterrupt:
        joint_command_subscriber.get_logger().info(
            f'Joint Command Subscriber stopped by user. '
            f'Received {joint_command_subscriber.command_count} commands.'
        )
    finally:
        joint_command_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()