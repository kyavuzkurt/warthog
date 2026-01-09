#!/usr/bin/env python3
"""
Camera Frame Transformer Node

This node transforms depth image frame IDs from Gazebo's coordinate system to 
ROS's standard camera optical frame convention for SLAM applications.

This node:
1. Subscribes to depth image topic from Gazebo
2. Updates frame ID to the correct optical frame
3. Republishes with correct frame ID
4. Broadcasts the optical frame transform
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class CameraFrameTransformer(Node):
    """
    Transforms depth image frame IDs for proper SLAM integration.
    """
    
    def __init__(self):
        super().__init__('camera_frame_transformer')
        
        # Declare parameters with default values
        self.declare_parameter('camera_prefix', 'rgbd_camera')
        
        # Get parameters
        self.camera_prefix = self.get_parameter('camera_prefix').value
        
        # Define frame names
        self.camera_link_frame = f'{self.camera_prefix}_link'
        self.optical_frame = f'{self.camera_prefix}_optical_frame'
        
        # Create TF broadcaster for the optical frame transform
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create publisher for transformed depth image
        self.depth_pub = self.create_publisher(
            Image,
            f'/{self.camera_prefix}/depth_image_corrected',
            10
        )
        
        # Create subscriber to raw depth image
        self.depth_sub = self.create_subscription(
            Image,
            f'/{self.camera_prefix}/depth_image',
            self.depth_callback,
            10
        )
        
        # Timer to periodically broadcast the static transform
        self.create_timer(0.1, self.broadcast_optical_frame_transform)
        
        self.get_logger().info(f'Camera Frame Transformer initialized for {self.camera_prefix}')
        self.get_logger().info(f'Subscribing to: /{self.camera_prefix}/depth_image')
        self.get_logger().info(f'Publishing to: /{self.camera_prefix}/depth_image_corrected')
        self.get_logger().info(f'Broadcasting transform: {self.camera_link_frame} -> {self.optical_frame}')
    
    def broadcast_optical_frame_transform(self):
        """
        Broadcast the static transform from camera_link to optical_frame.
        
        Rotation: RPY(-π/2, 0, -π/2)
        This transforms coordinate system to ROS camera convention:
        - x-axis: right
        - y-axis: down  
        - z-axis: forward (optical axis)
        """
        t = TransformStamped()
        
        # Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_link_frame
        t.child_frame_id = self.optical_frame
        
        # Translation (no offset needed)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Rotation: RPY(-π/2, 0, -π/2) as quaternion
        t.transform.rotation.x = -0.5
        t.transform.rotation.y = 0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w = 0.5
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)
    
    def depth_callback(self, msg):
        """
        Transform and republish depth image with corrected frame ID.
        """
        # Create a new message with corrected frame
        corrected_msg = msg
        corrected_msg.header.frame_id = self.optical_frame
        
        # Republish
        self.depth_pub.publish(corrected_msg)


def main(args=None):
    """Main entry point for the camera frame transformer node."""
    rclpy.init(args=args)
    
    node = CameraFrameTransformer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

