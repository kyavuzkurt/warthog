#!/usr/bin/env python3
# Copyright (c) 2023 Clearpath Robotics
#
# Licensed under the BSD License.

"""
Launch file to spawn a Warthog robot in Ignition Gazebo.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='base',
        description='Configuration of Warthog which you would like to simulate'
    )
    
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
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
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
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
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Spawn the robot in Ignition Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_warthog',
        output='screen',
        arguments=[
            '-name', 'warthog',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # ROS-Gazebo bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/lidar_3d/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Odometry to TF broadcaster
    odom_to_tf = Node(
        package='warthog_gazebo',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Camera frame transformer for correct ROS coordinate convention
    camera_frame_transformer = Node(
        package='warthog_controller',
        executable='camera_frame_transformer',
        name='camera_frame_transformer',
        output='screen',
        parameters=[{
            'camera_prefix': 'rgbd_camera',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        config_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        use_sim_time_arg,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
        odom_to_tf,
        camera_frame_transformer
    ])
