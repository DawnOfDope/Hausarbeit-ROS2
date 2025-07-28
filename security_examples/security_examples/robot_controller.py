import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Robot Controller Node has been started and is publishing commands.')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2  # Langsam vorwärts fahren
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "linear.x: %f, angular.z: %f"' % (msg.linear.x, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotControllerNode()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()