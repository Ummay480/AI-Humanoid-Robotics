#!/usr/bin/env python3

"""
Launch file for complete ROS2 humanoid robot control pipeline
Implements User Story 6: Run a Complete ROS 2 Humanoid Robot Control Pipeline
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    # Declare launch arguments
    urdf_model_path = LaunchConfiguration('urdf_model', default='humanoid_robot.urdf')

    # Define nodes for the complete pipeline
    # 1. Robot State Publisher to publish TF and robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description':
                # Read URDF from file - in a real implementation this would read the actual URDF
                # For this example, we'll use a parameter placeholder
                '<robot name="simple_humanoid"><link name="base_link"/></robot>'
            }
        ],
        output='screen'
    )

    # 2. Joint State Publisher (for simulation)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # 3. Joint State Publisher GUI (optional, for manual joint control during testing)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # 4. Robot Control Node (from example 01)
    robot_control_node = Node(
        package='your_robot_package',  # This would be replaced with actual package name
        executable='01_robot_control_node',
        name='robot_control_node',
        output='screen',
        parameters=[
            {'use_sim_time': True}  # Use simulation time when running with Gazebo
        ]
    )

    # 5. Joint State Publisher (simulated sensor data - from example 02)
    joint_state_publisher_node = Node(
        package='your_robot_package',
        executable='02_joint_state_publisher',
        name='joint_state_publisher_node',
        output='screen'
    )

    # 6. Joint Command Subscriber (actuator interface - from example 03)
    joint_command_subscriber_node = Node(
        package='your_robot_package',
        executable='03_joint_command_subscriber',
        name='joint_command_subscriber_node',
        output='screen'
    )

    # 7. Robot Control Service Server (from example 04)
    robot_control_service_server = Node(
        package='your_robot_package',
        executable='04_robot_control_service_server',
        name='robot_control_service_server',
        output='screen'
    )

    # 8. Robot Control Service Client (from example 05)
    robot_control_service_client = Node(
        package='your_robot_package',
        executable='05_robot_control_service_client',
        name='robot_control_service_client',
        output='screen'
    )

    # 9. AI Robot Agent (from example 06)
    ai_robot_agent = Node(
        package='your_robot_package',
        executable='06_ai_robot_agent',
        name='ai_robot_agent',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # 10. Gazebo simulation (if available)
    gazebo_sim = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so'
        ],
        output='screen'
    )

    # 11. Spawn robot in Gazebo (using ros2 control)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot'
        ],
        output='screen'
    )

    # 12. Controller Manager (to manage ros2_control controllers)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('your_robot_package'),
                'config',
                'ros2_controllers.yaml'
            ])
        ],
        output='screen'
    )

    # Define the launch description
    ld = LaunchDescription()

    # Add all actions to the launch description
    # Static transforms and robot description
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    # ld.add_action(joint_state_publisher_gui)  # Uncomment if GUI is needed

    # Robot control components
    ld.add_action(robot_control_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_command_subscriber_node)

    # Service components
    ld.add_action(robot_control_service_server)
    ld.add_action(robot_control_service_client)

    # AI agent
    ld.add_action(ai_robot_agent)

    # Simulation components (uncomment if Gazebo is available)
    # ld.add_action(gazebo_sim)
    # ld.add_action(spawn_entity)
    # ld.add_action(controller_manager)

    return ld


# Note: This launch file is structured to work with the example nodes created.
# In a real ROS 2 environment, you would need to:
# 1. Put these Python files in a proper ROS 2 package
# 2. Update the package names to match your actual package
# 3. Create the appropriate configuration files
# 4. Ensure all dependencies are properly declared in package.xml
# 5. Build the package using colcon build