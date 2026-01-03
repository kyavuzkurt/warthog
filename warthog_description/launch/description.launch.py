#!/usr/bin/env python3
# Copyright (c) 2023 Clearpath Robotics
#
# Licensed under the BSD License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the package directory
    pkg_warthog_description = get_package_share_directory('warthog_description')
    
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='base',
        description='Configuration name (base, arm_mount, bulkhead, empty)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # Build the path to the URDF file
    urdf_file = PathJoinSubstitution([
        FindPackageShare('warthog_description'),
        'urdf',
        'warthog.urdf.xacro'
    ])
    
    config_path = PathJoinSubstitution([
        FindPackageShare('warthog_description'),
        'urdf',
        'configs',
        LaunchConfiguration('config')
    ])
    
    # RViz config file
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('warthog_description'),
        'config',
        'config.rviz'
    ])
    
    # Process the URDF file with xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindPackageShare('warthog_description'), 'scripts', 'env_run']),
        ' ',
        config_path,
        ' ',
        'xacro',
        ' ',
        urdf_file
    ])
    
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        config_arg,
        use_rviz_arg,
        robot_state_publisher_node,
        rviz_node
    ])
