#!/usr/bin/env python3
"""
Comprehensive launch file for Homework 2.
Launches Gazebo with outdoor world, spawns Warthog robot, starts controller node, and RViz.
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
    
    # Set Gazebo resource path to find meshes
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
    
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='5.0',
        description='Initial x position'
    )
    
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Initial y position'
    )
    
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.5',
        description='Initial z position'
    )
    
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw angle'
    )
    
    # Get the outdoor world file path
    world_file = PathJoinSubstitution([
        FindPackageShare('warthog_gazebo'),
        'worlds',
        'outdoor.sdf'
    ])
    
    # Launch Gazebo with outdoor world
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
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    # Warthog controller node (from Homework 1)
    #controller_node = TimerAction(
    #    period=3.0,
    #    actions=[
    #        Node(
    #            package='warthog_controller',
    #            executable='cmd_pub',
    #            name='warthog_controller',
    #            output='screen',
    #            parameters=[{'use_sim_time': False}]
    #        )
    #    ]
    #)
    
    # RViz configuration file path
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('warthog_description'),
        'config',
        'homework_outdoor.rviz'
    ])
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        # Environment variables
        gz_model_path,
        ign_gazebo_resource_path,
        
        # Launch arguments
        use_sim_time_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        
        # Launch Gazebo
        gz_sim,
        
        # Spawn robot (includes robot_state_publisher, bridges, and odom_to_tf)
        spawn_warthog,
        
        # Controller from Homework 1 - delayed to allow robot to spawn
        #controller_node,
        
        # Visualization
        TimerAction(
            period=3.0,
            actions=[rviz_node]
        )
    ])
