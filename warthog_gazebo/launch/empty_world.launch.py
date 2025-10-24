#!/usr/bin/env python3
# Copyright (c) 2023 Clearpath Robotics
#
# Licensed under the BSD License.

"""
Launch Ignition Gazebo with an empty world and spawn a Warthog robot.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get package paths
    pkg_warthog_description = get_package_share_directory('warthog_description')
    pkg_warthog_gazebo = get_package_share_directory('warthog_gazebo')
    
    # Build the resource path - point to models directory
    models_path = os.path.join(pkg_warthog_description, 'models')
    resource_path = models_path + ':' + \
                   pkg_warthog_description + ':' + \
                   pkg_warthog_gazebo + ':' + \
                   os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    # Set Gazebo resource path to find meshes - use both GZ_SIM and IGN_GAZEBO for compatibility
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )
    
    ign_gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path
    )
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    world_name_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World name'
    )
    
    gz_args_arg = DeclareLaunchArgument(
        'gz_args',
        default_value='',
        description='Extra arguments for Gazebo'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    # Get the world file path
    world_file = PathJoinSubstitution([
        FindPackageShare('warthog_gazebo'),
        'worlds',
        'empty.sdf'
    ])
    
    # Launch Ignition Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [world_file, ' -r']
        }.items()
    )
    
    # Include the spawn_warthog launch file
    spawn_warthog = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('warthog_gazebo'),
                'launch',
                'spawn_warthog.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    # Get RViz config file
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('warthog_description'),
        'config',
        'config.rviz'
    ])
    
    # Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # Delay RViz start to ensure robot_state_publisher is ready
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )
    
    return LaunchDescription([
        gz_model_path,
        ign_gazebo_resource_path,
        use_sim_time_arg,
        world_name_arg,
        gz_args_arg,
        rviz_arg,
        gz_sim,
        spawn_warthog,
        delayed_rviz
    ])
