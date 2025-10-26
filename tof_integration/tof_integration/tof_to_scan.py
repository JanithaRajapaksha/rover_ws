#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import random

class VirtualScanPublisher(Node):
    def __init__(self):
        super().__init__('virtual_scan_publisher')

        # Enable simulation time if set
        # self.declare_parameter('use_sim_time', True)

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.publish_scan)

        self.get_logger().info('Virtual Scan Publisher started!')

    def publish_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()  # timestamp
        scan_msg.header.frame_id = 'base_link'

        # LaserScan parameters
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2 * math.pi
        scan_msg.angle_increment = 2 * math.pi / 8  # 8 readings
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.0  # corresponds to 20Hz
        scan_msg.range_min = 0.01
        scan_msg.range_max = 2.0

        # Random ranges between 0.5 and 1.5 meters
        scan_msg.ranges = [
    float('inf'),       # 0° (front-left)
    1.5,  # 45°
    1.0,  # 90°
    1.5,  # 135°
    float('inf'),  # 180°
    1.5,  # 225°
    1.0,  # 270°
    1.5       # 315° (front-right)
]
        scan_msg.intensities = [0.0] * 8

        self.publisher.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Virtual Scan Publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
