#!/usr/bin/env python3

"""
Robot Control Service Client - ROS2 Service Client example for robot control
Implements User Story 3: Robot Control Topics and Services - Service Client component
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time


class RobotControlServiceClient(Node):
    """
    Service client that calls robot control services
    Demonstrates request/response communication with timing constraints
    """

    def __init__(self):
        super().__init__('robot_control_service_client')

        # Create clients for the services
        self.emergency_stop_client = self.create_client(SetBool, 'emergency_stop')
        self.get_robot_state_client = self.create_client(SetBool, 'get_robot_state')

        # Wait for services to be available
        self.get_logger().info('Waiting for services to become available...')
        while not self.emergency_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Emergency stop service not available, waiting again...')

        while not self.get_robot_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get robot state service not available, waiting again...')

        self.get_logger().info('Services are available')

        # Create timer to periodically call services (1Hz)
        self.service_timer = self.create_timer(1.0, self.call_services)

        # Initialize call counters
        self.call_count = 0
        self.state_request_count = 0

        self.get_logger().info('Robot Control Service Client initialized')

    def call_services(self):
        """
        Periodically call robot control services
        """
        self.call_count += 1

        # Alternate between different service calls
        if self.call_count % 3 == 0:
            # Request robot state
            self.request_robot_state()
        elif self.call_count % 6 == 1:
            # Test emergency stop (activate)
            self.activate_emergency_stop()
        elif self.call_count % 6 == 4:
            # Test emergency stop (deactivate)
            self.deactivate_emergency_stop()

    def request_robot_state(self):
        """
        Request current robot state from the service
        """
        self.state_request_count += 1
        request = SetBool.Request()
        request.data = False  # Request state, not change it

        # Call the service asynchronously
        future = self.get_robot_state_client.call_async(request)
        future.add_done_callback(self.robot_state_callback)

        self.get_logger().debug(f'Sent robot state request #{self.state_request_count}')

    def activate_emergency_stop(self):
        """
        Activate emergency stop
        """
        request = SetBool.Request()
        request.data = True  # Activate emergency stop

        # Call the service asynchronously
        future = self.emergency_stop_client.call_async(request)
        future.add_done_callback(self.emergency_stop_callback)

        self.get_logger().info('Sent emergency stop activation request')

    def deactivate_emergency_stop(self):
        """
        Deactivate emergency stop
        """
        request = SetBool.Request()
        request.data = False  # Deactivate emergency stop

        # Call the service asynchronously
        future = self.emergency_stop_client.call_async(request)
        future.add_done_callback(self.emergency_stop_callback)

        self.get_logger().info('Sent emergency stop deactivation request')

    def robot_state_callback(self, future):
        """
        Callback for robot state service response
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Robot state: {response.message}')
            else:
                self.get_logger().error(f'Failed to get robot state: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def emergency_stop_callback(self, future):
        """
        Callback for emergency stop service response
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Emergency stop response: {response.message}')
            else:
                self.get_logger().error(f'Emergency stop failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Emergency stop service call failed: {e}')


def main(args=None):
    """
    Main function to run the robot control service client
    """
    rclpy.init(args=args)

    robot_control_client = RobotControlServiceClient()

    try:
        rclpy.spin(robot_control_client)
    except KeyboardInterrupt:
        robot_control_client.get_logger().info('Robot Control Service Client stopped by user')
    finally:
        robot_control_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()