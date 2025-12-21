#!/usr/bin/env python3

"""
Robot Control Service Server - ROS2 Service Server example for robot control
Implements User Story 3: Robot Control Topics and Services - Service Server component
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse
from rclpy.action.server import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import math


class RobotControlServiceServer(Node):
    """
    Service server that handles robot control requests
    Implements various control services for robot operations
    """

    def __init__(self):
        super().__init__('robot_control_service_server')

        # Create service to handle emergency stop requests
        self.emergency_stop_service = self.create_service(
            SetBool,
            'emergency_stop',
            self.emergency_stop_callback
        )

        # Create service to handle robot state requests
        self.get_robot_state_service = self.create_service(
            SetBool,
            'get_robot_state',
            self.get_robot_state_callback
        )

        # Initialize robot state
        self.robot_enabled = True
        self.current_trajectory = None
        self.trajectory_execution_active = False

        self.get_logger().info('Robot Control Service Server initialized')
        self.get_logger().info('Services available: /emergency_stop, /get_robot_state')

    def emergency_stop_callback(self, request, response):
        """
        Handle emergency stop service requests
        """
        if request.data:  # If emergency stop is requested
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            self.robot_enabled = False
            response.success = True
            response.message = 'Emergency stop activated'
        else:  # If emergency stop reset is requested
            self.get_logger().info('Emergency stop reset')
            self.robot_enabled = True
            response.success = True
            response.message = 'Emergency stop reset, robot enabled'

        return response

    def get_robot_state_callback(self, request, response):
        """
        Handle robot state query requests
        """
        # Return current robot state
        response.success = True
        state_info = f'Robot enabled: {self.robot_enabled}, ' \
                    f'Trajectory active: {self.trajectory_execution_active}'
        response.message = state_info

        self.get_logger().debug(f'Robot state requested: {state_info}')
        return response


def main(args=None):
    """
    Main function to run the robot control service server
    """
    rclpy.init(args=args)

    robot_control_server = RobotControlServiceServer()

    try:
        rclpy.spin(robot_control_server)
    except KeyboardInterrupt:
        robot_control_server.get_logger().info('Robot Control Service Server stopped by user')
    finally:
        robot_control_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()