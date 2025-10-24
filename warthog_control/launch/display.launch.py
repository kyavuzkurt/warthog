#!/usr/bin/env python3
# Copyright (c) 2023 Clearpath Robotics
#
# Licensed under the BSD License.

"""
Complete visualization launch - combines robot description, RViz, and joint state publisher.
Use this for testing without hardware or simulation.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    
    # Arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='base',
        description='Robot configuration'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Include the description launch file
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('warthog_description'),
                'launch',
                'description.launch.py'
            ])
        ]),
        launch_arguments={
            'config': LaunchConfiguration('config'),
            'use_rviz': 'true'
        }.items()
    )
    
    # Joint state publisher with GUI to manually control joints
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        config_arg,
        use_sim_time_arg,
        description_launch,
        joint_state_publisher_gui_node
    ])
