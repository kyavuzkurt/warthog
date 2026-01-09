#!/usr/bin/env python3
"""
RTAB-Map launch file configured for Warthog rosbag data
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value='~/.ros/rtabmap.db',
        description='Path to RTAB-Map database'
    )
    
    # RTAB-Map node with full sensor fusion
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'database_path': LaunchConfiguration('database_path'),
            
            # Sensor subscriptions
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan_cloud': True,
            
            # Frame IDs
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            
            # Synchronization
            'approx_sync': True,
            'wait_for_transform': 0.2,
            'queue_size': 30,
            
            # RTAB-Map parameters for high-quality mapping
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '10',
            'RGBD/AngularUpdate': '0.01',
            'RGBD/LinearUpdate': '0.01',
            'RGBD/OptimizeFromGraphEnd': 'false',
            
            # Memory management
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'Mem/ReduceGraph': 'false',
            
            # Loop closure detection (important for drift correction)
            'RGBD/LoopClosureReextractFeatures': 'true',
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
            
            # LiDAR ICP for scan matching
            'Reg/Strategy': '1',  # Use ICP
            'Icp/CorrespondenceRatio': '0.3',
            'Icp/MaxCorrespondenceDistance': '0.1',
            
            # Grid parameters
            'Grid/FromDepth': 'true',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            'Grid/CellSize': '0.05',
            'Grid/RangeMax': '50',
            
            # Optimization
            'Optimizer/Strategy': '1',  # g2o
            'Optimizer/Robust': 'true',
            'Optimizer/Epsilon': '0.0001',
        }],
        remappings=[
            ('rgb/image', '/rgbd_camera/image'),
            ('rgb/camera_info', '/rgbd_camera/camera_info'),
            ('depth/image', '/rgbd_camera/depth_image'),
            ('scan_cloud', '/lidar_3d/points/points'),
            ('odom', '/odom'),
            ('imu', '/imu/data'),
        ],
        arguments=['--delete_db_on_start']
    )
    
    # RTAB-Map visualization with 3D view
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info': True,
            'frame_id': 'base_link',
            'approx_sync': True,
            'wait_for_transform': 0.2,
            'queue_size': 30,
        }],
        remappings=[
            ('rgb/image', '/rgbd_camera/image'),
            ('rgb/camera_info', '/rgbd_camera/camera_info'),
            ('depth/image', '/rgbd_camera/points'),
            ('scan_cloud', '/lidar_3d/points/points'),
            ('odom', '/odom'),
            ('imu', '/imu/data'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        database_path_arg,
        rtabmap_node,
        #rtabmap_viz,  # Real-time 3D visualization
    ])

