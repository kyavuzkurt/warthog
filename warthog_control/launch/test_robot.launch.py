#!/usr/bin/env python3
# Copyright (c) 2023 Clearpath Robotics
#
# Licensed under the BSD License.

"""
Simple launch file to test the robot without controller_manager.
This publishes joint states and allows manual velocity commands.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Joint state publisher - publishes dummy joint states
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Simple cmd_vel listener that prints velocity commands
    # You can publish to /cmd_vel using:
    # ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
    
    return LaunchDescription([
        use_sim_time_arg,
        joint_state_publisher_node
    ])
