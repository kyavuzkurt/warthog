#! usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdPub(Node):
    def __init__(self):
        super().__init__('cmd_pub')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0  # Set linear velocity
        msg.angular.z = 0.5  # Set angular velocity
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: Linear X: %.2f, Angular Z: %.2f' % (msg.linear.x, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    cmd_pub = CmdPub()
    rclpy.spin(cmd_pub)
    cmd_pub.destroy_node()
    rclpy.shutdown()