#!/usr/bin/env python3
# Copyright (c) 2023 Clearpath Robotics
#
# Licensed under the BSD License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package directory
    pkg_warthog_control = get_package_share_directory('warthog_control')
    
    # Declare launch arguments
    enable_joy_arg = DeclareLaunchArgument(
        'enable_joy',
        default_value='false',
        description='Enable joystick teleop'
    )
    
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )
    
    # Load teleop configuration
    teleop_config = PathJoinSubstitution([
        FindPackageShare('warthog_control'),
        'config',
        'teleop.yaml'
    ])
    
    twist_mux_config = PathJoinSubstitution([
        FindPackageShare('warthog_control'),
        'config',
        'twist_mux.yaml'
    ])
    
    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev')
        }],
        condition=IfCondition(LaunchConfiguration('enable_joy'))
    )
    
    # Teleop twist joy node
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[teleop_config],
        condition=IfCondition(LaunchConfiguration('enable_joy'))
    )
    
    # Joy teleop group with namespace
    joy_teleop_group = GroupAction(
        actions=[
            PushRosNamespace('joy_teleop'),
            joy_node,
            teleop_twist_joy_node
        ],
        condition=IfCondition(LaunchConfiguration('enable_joy'))
    )
    
    # Twist mux node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[
            twist_mux_config,
            {'locks': []}
        ],
        remappings=[
            ('cmd_vel_out', 'warthog_velocity_controller/cmd_vel')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        enable_joy_arg,
        joy_dev_arg,
        joy_teleop_group,
        twist_mux_node
    ])
