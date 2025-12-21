#!/usr/bin/env python3

"""
Basic ROS 2 robot control node example
Implements User Story 2 from the specification: Build ROS 2 Nodes for Robot Control using rclpy
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class RobotControlNode(Node):
    """
    A basic robot control node that demonstrates:
    - Node initialization for control systems
    - Lifecycle management
    - Proper logging for debugging control loops
    """

    def __init__(self):
        super().__init__('robot_control_node')

        # Create a QoS profile for control applications
        control_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Create subscriber for joint states (sensor data)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile=control_qos
        )

        # Create publisher for joint commands (actuator commands)
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            qos_profile=control_qos
        )

        # Create timer for control loop (50Hz = 0.02 seconds)
        self.control_timer = self.create_timer(0.02, self.control_loop)

        # Initialize internal state
        self.current_joint_positions = []
        self.target_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # Example: 5 joints

        # Log initialization
        self.get_logger().info('Robot Control Node initialized')
        self.get_logger().info('Subscribed to /joint_states')
        self.get_logger().info('Publishing to /joint_commands')

    def joint_state_callback(self, msg):
        """
        Callback function to process joint state messages
        """
        self.current_joint_positions = list(msg.position)
        self.get_logger().debug(f'Received joint positions: {self.current_joint_positions}')

    def control_loop(self):
        """
        Main control loop function that runs at 50Hz
        """
        # Simple PD controller example
        if len(self.current_joint_positions) > 0:
            commands = Float64MultiArray()

            # Calculate control commands based on target vs current positions
            joint_commands = []
            for i in range(min(len(self.target_joint_positions), len(self.current_joint_positions))):
                # Simple proportional control (P-controller)
                error = self.target_joint_positions[i] - self.current_joint_positions[i]
                command = error * 2.0  # Proportional gain
                joint_commands.append(command)

            # Fill remaining joints with zero if needed
            while len(joint_commands) < len(self.target_joint_positions):
                joint_commands.append(0.0)

            commands.data = joint_commands

            # Publish control commands
            self.joint_command_publisher.publish(commands)

            # Log control loop execution
            self.get_logger().debug(f'Published joint commands: {commands.data}')

    def update_target_positions(self, new_targets):
        """
        Update target joint positions

        Args:
            new_targets: List of new target joint positions
        """
        self.target_joint_positions = new_targets
        self.get_logger().info(f'Updated target positions: {new_targets}')


def main(args=None):
    """
    Main function to run the robot control node
    """
    rclpy.init(args=args)

    robot_control_node = RobotControlNode()

    try:
        # Set some example target positions
        robot_control_node.update_target_positions([0.5, -0.3, 0.2, 0.1, -0.4])

        # Spin the node
        rclpy.spin(robot_control_node)
    except KeyboardInterrupt:
        robot_control_node.get_logger().info('Robot Control Node interrupted by user')
    finally:
        # Clean up
        robot_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()