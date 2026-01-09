#!/usr/bin/env python3
"""
Publish calibrated LiDAR TF
Overrides the URDF static transform with calibrated values
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import yaml
import sys
import numpy as np
from pathlib import Path


class CalibratedLidarTF(Node):
    def __init__(self, calibration_file):
        super().__init__('calibrated_lidar_tf')
        
        # Load calibration
        with open(calibration_file, 'r') as f:
            calib = yaml.safe_load(f)
        
        # Get corrected pose (original + correction)
        # Original error: xyz="0.0 0.005 0.105" rpy="0.0262 -0.0175 0.0349"
        original_x = 0.0
        original_y = 0.005
        original_z = 0.105
        original_roll = 0.0262
        original_pitch = -0.0175
        original_yaw = 0.0349
        
        # Apply correction (subtract the found error)
        self.x = original_x - calib['calibration']['translation']['x']
        self.y = original_y - calib['calibration']['translation']['y']
        self.z = original_z - calib['calibration']['translation']['z']
        self.roll = original_roll - calib['calibration']['rotation']['roll']
        self.pitch = original_pitch - calib['calibration']['rotation']['pitch']
        self.yaw = original_yaw - calib['calibration']['rotation']['yaw']
        
        self.get_logger().info('Calibrated LiDAR TF Publisher')
        self.get_logger().info(f'Corrected pose:')
        self.get_logger().info(f'  Translation: ({self.x:.6f}, {self.y:.6f}, {self.z:.6f})')
        self.get_logger().info(f'  Rotation: ({self.roll:.6f}, {self.pitch:.6f}, {self.yaw:.6f})')
        
        # Use static broadcaster to override URDF static transform
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish the calibrated transform once
        self.publish_calibrated_tf()
        self.get_logger().info('✓ Calibrated TF published (overriding URDF)')
    
    def publish_calibrated_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'top_chassis_link'
        t.child_frame_id = 'lidar_3d_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        
        # Convert RPY to quaternion (ZYX convention)
        cy = np.cos(self.yaw * 0.5)
        sy = np.sin(self.yaw * 0.5)
        cp = np.cos(self.pitch * 0.5)
        sp = np.sin(self.pitch * 0.5)
        cr = np.cos(self.roll * 0.5)
        sr = np.sin(self.roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)


def main():
    if len(sys.argv) < 2:
        print("Usage: ros2 run warthog_controller calibrated_lidar_tf calibration_result.yaml")
        sys.exit(1)
    
    calib_file = sys.argv[1]
    if not Path(calib_file).exists():
        print(f"ERROR: Calibration file not found: {calib_file}")
        sys.exit(1)
    
    rclpy.init()
    node = CalibratedLidarTF(calib_file)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

