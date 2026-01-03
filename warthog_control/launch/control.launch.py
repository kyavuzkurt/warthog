#!/usr/bin/env python3
# Copyright (c) 2023 Clearpath Robotics
#
# Licensed under the BSD License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package directory
    pkg_warthog_control = get_package_share_directory('warthog_control')
    
    # Declare launch arguments
    enable_ekf_arg = DeclareLaunchArgument(
        'enable_ekf',
        default_value='true',
        description='Enable EKF localization'
    )
    
    config_extras_arg = DeclareLaunchArgument(
        'config_extras',
        default_value=PathJoinSubstitution([
            FindPackageShare('warthog_control'),
            'config',
            'empty.yaml'
        ]),
        description='Path to extra config file'
    )
    
    # Load controller configuration
    control_config = PathJoinSubstitution([
        FindPackageShare('warthog_control'),
        'config',
        'control_ros2.yaml'
    ])
    
    # Controller manager node - spawns the controllers
    controller_manager_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'warthog_joint_publisher',
            'warthog_velocity_controller',
            '--controller-manager-timeout',
            '10'
        ],
        output='screen'
    )
    
    # EKF localization node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization',
        output='screen',
        parameters=[control_config],
        condition=IfCondition(LaunchConfiguration('enable_ekf'))
    )
    
    return LaunchDescription([
        enable_ekf_arg,
        config_extras_arg,
        controller_manager_node,
        ekf_node
    ])
