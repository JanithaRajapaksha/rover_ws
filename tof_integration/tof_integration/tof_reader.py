#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.time import Time, Duration
from std_msgs.msg import Bool  # Optional for detection topic

class ProximityBackOneSec(Node):
    def __init__(self):
        super().__init__('proximity_back_one_sec')
        
        # Parameters
        self.declare_parameter('stop_distance', 0.3)
        self.declare_parameter('back_speed', 0.2)
        self.declare_parameter('back_duration', 3.0)
        
        self.stop_distance = self.get_parameter('stop_distance').value
        self.back_speed = self.get_parameter('back_speed').value
        self.back_duration = self.get_parameter('back_duration').value
        
        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_scaled', 10)
        self.detect_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        
        # State variables
        self.forward_distance = float('inf')
        self.backing = False
        self.back_end_time = None
        self.obstacle_detected = False  # to avoid log spamming
    
    def scan_callback(self, msg: LaserScan):
        # Check forward-facing sensors: indices 0, 1, and 7
        forward_ranges = [msg.ranges[i] for i in [0, 1, 7]]
        forward_ranges = [r if r == r and r != float('inf') else float('inf') for r in forward_ranges]
        self.forward_distance = min(forward_ranges)
        
        # Detect obstacle
        if self.forward_distance < self.stop_distance:
            if not self.obstacle_detected:
                self.obstacle_detected = True
                self.get_logger().warn(f'⚠️ Obstacle detected at {self.forward_distance:.2f} m!')
                self.detect_pub.publish(Bool(data=True))
            
            # Start backing if not already doing so
            if not self.backing:
                self.backing = True
                self.back_end_time = self.get_clock().now() + Duration(seconds=self.back_duration)
                self.get_logger().info(f'Backing up for {self.back_duration} seconds...')
        else:
            if self.obstacle_detected:
                self.obstacle_detected = False
                self.detect_pub.publish(Bool(data=False))
    
    def cmd_callback(self, msg: Twist):
        cmd = Twist()
        now = self.get_clock().now()
        
        if self.backing:
            if now < self.back_end_time:
                cmd.linear.x = -self.back_speed
                cmd.angular.z = 0.0
            else:
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
