#!/usr/bin/env python3

"""
AI Robot Agent - Python AI agent that subscribes to robot sensor data and publishes control commands
Implements User Story 4: Bridge Python AI Agents to ROS 2 Robot Controllers using rclpy
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import random
from enum import Enum


class RobotBehavior(Enum):
    """
    Different behaviors the AI agent can exhibit
    """
    IDLE = 1
    TRACKING = 2
    AVOIDING = 3
    HOMING = 4


class AIRobotAgent(Node):
    """
    AI agent that bridges Python-based decision making with ROS 2 robot controllers
    Subscribes to robot sensor data and publishes control commands
    """

    def __init__(self):
        super().__init__('ai_robot_agent')

        # Create QoS profiles for different communication patterns
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        control_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Create subscribers for robot sensor data
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile=sensor_qos
        )

        # Create publishers for robot control commands
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            qos_profile=control_qos
        )

        self.velocity_command_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile=control_qos
        )

        # Initialize internal state
        self.current_joint_positions = []
        self.current_joint_velocities = []
        self.current_behavior = RobotBehavior.IDLE
        self.ai_decision_timer = self.create_timer(0.1, self.ai_decision_loop)  # 10Hz AI decisions
        self.last_ai_decision_time = self.get_clock().now()

        # AI agent parameters
        self.target_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.behavior_switch_interval = 10.0  # seconds
        self.last_behavior_switch = self.get_clock().now().nanoseconds / 1e9

        # Log initialization
        self.get_logger().info('AI Robot Agent initialized')
        self.get_logger().info('Subscribed to /joint_states')
        self.get_logger().info('Publishing to /joint_commands and /cmd_vel')

    def joint_state_callback(self, msg):
        """
        Process incoming joint state messages from robot sensors
        """
        self.current_joint_positions = list(msg.position)
        self.current_joint_velocities = list(msg.velocity)

        # Log first few messages to see data
        if len(self.current_joint_positions) > 0 and sum(abs(p) for p in self.current_joint_positions) < 0.1:
            self.get_logger().debug(f'Received joint states: pos={self.current_joint_positions[:3]}...')

    def ai_decision_loop(self):
        """
        Main AI decision-making loop that runs at 10Hz
        """
        current_time = self.get_clock().now()
        time_elapsed = (current_time - self.last_ai_decision_time).nanoseconds / 1e9

        # Update behavior based on time
        self.update_behavior()

        # Apply AI logic based on current behavior
        control_commands = self.apply_ai_logic()

        # Publish control commands
        if control_commands:
            cmd_msg = Float64MultiArray()
            cmd_msg.data = control_commands
            self.joint_command_publisher.publish(cmd_msg)

            # Log every 10 decisions to avoid spam
            if int(time_elapsed * 10) % 10 == 0:
                self.get_logger().debug(f'Published AI commands: {control_commands[:3]}...')

        # Update timing
        self.last_ai_decision_time = current_time

    def update_behavior(self):
        """
        Update the robot's behavior based on time and current state
        """
        current_time_sec = self.get_clock().now().nanoseconds / 1e9

        if current_time_sec - self.last_behavior_switch > self.behavior_switch_interval:
            # Switch to a random behavior
            behaviors = list(RobotBehavior)
            new_behavior = random.choice(behaviors)

            if new_behavior != self.current_behavior:
                self.current_behavior = new_behavior
                self.last_behavior_switch = current_time_sec
                self.get_logger().info(f'Switched to behavior: {self.current_behavior.name}')

    def apply_ai_logic(self):
        """
        Apply AI decision-making logic based on sensor data and current behavior
        """
        commands = []

        if len(self.current_joint_positions) == 0:
            # No sensor data yet, return zeros
            return [0.0] * 5

        # Apply different logic based on behavior
        if self.current_behavior == RobotBehavior.IDLE:
            # Stay in current position
            commands = [0.0] * len(self.current_joint_positions)
        elif self.current_behavior == RobotBehavior.TRACKING:
            # Follow a simple trajectory pattern
            t = self.get_clock().now().nanoseconds / 1e9
            for i in range(len(self.current_joint_positions)):
                # Create oscillating target positions
                target = math.sin(t * 0.5 + i) * 0.5
                # Simple PD control to reach target
                error = target - self.current_joint_positions[i]
                command = error * 2.0  # P gain
                commands.append(command)
        elif self.current_behavior == RobotBehavior.AVOIDING:
            # Simple obstacle avoidance pattern
            # In a real system, this would use sensor data like laser scans
            for i in range(len(self.current_joint_positions)):
                # Add some random movement to avoid obstacles
                random_offset = random.uniform(-0.1, 0.1)
                command = random_offset
                commands.append(command)
        elif self.current_behavior == RobotBehavior.HOMING:
            # Move back to home position (all zeros)
            for i in range(len(self.current_joint_positions)):
                # PD control to home position
                error = 0.0 - self.current_joint_positions[i]
                command = error * 3.0  # Higher P gain for homing
                commands.append(command)

        # Ensure we have the right number of commands
        while len(commands) < 5:
            commands.append(0.0)

        return commands[:5]  # Limit to 5 joints

    def get_robot_state_summary(self):
        """
        Get a summary of the robot's current state for AI decision making
        """
        state = {
            'joint_positions': self.current_joint_positions,
            'joint_velocities': self.current_joint_velocities,
            'current_behavior': self.current_behavior,
            'time_in_behavior': (self.get_clock().now().nanoseconds / 1e9) - self.last_behavior_switch
        }
        return state


def main(args=None):
    """
    Main function to run the AI robot agent
    """
    rclpy.init(args=args)

    ai_robot_agent = AIRobotAgent()

    try:
        ai_robot_agent.get_logger().info('AI Robot Agent starting...')
        rclpy.spin(ai_robot_agent)
    except KeyboardInterrupt:
        ai_robot_agent.get_logger().info('AI Robot Agent stopped by user')
    finally:
        ai_robot_agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()