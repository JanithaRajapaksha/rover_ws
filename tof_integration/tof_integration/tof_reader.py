#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.time import Time

class ProximityBackOneSec(Node):
    def __init__(self):
        super().__init__('proximity_back_one_sec')



        
        # Parameters
        self.declare_parameter('stop_distance', 0.3)
        self.declare_parameter('back_speed', 0.8)  # linear backward speed
        self.declare_parameter('back_duration', 3.0)  # seconds
        
        self.stop_distance = self.get_parameter('stop_distance').value
        self.back_speed = self.get_parameter('back_speed').value
        self.back_duration = self.get_parameter('back_duration').value
        
        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_scaled', 10)
        
        # State variables
        self.forward_distance = float('inf')
        self.backing = False
        self.back_end_time = None
    
    def scan_callback(self, msg: LaserScan):
        # Forward rays: indices 4 and 5
        forward_ranges = [msg.ranges[4], msg.ranges[5]]
        forward_ranges = [r if r == r and r != float('inf') else float('inf') for r in forward_ranges]
        self.forward_distance = min(forward_ranges)
        
        # Start backing if too close and not already backing
        if self.forward_distance < self.stop_distance and not self.backing:
            self.backing = True
            self.back_end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=self.back_duration)
            self.get_logger().info('Obstacle detected! Backing for 1 second.')
    
    def cmd_callback(self, msg: Twist):
        cmd = Twist()
        now = self.get_clock().now()
        
        if self.backing:
            if now < self.back_end_time:
                # Keep backing
                cmd.linear.x = -self.back_speed
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
            else:
                # Stop backing
                self.backing = False
                cmd = msg
        else:
            cmd = msg
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ProximityBackOneSec()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
