#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
import math
import time

class Rotate90(Node):
    def __init__(self):
        super().__init__('rotate_90_tf')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_scaled', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_yaw = None
        self.target_yaw = None
        self.state = 'WAIT_FOR_YAW'

        # Check every 0.1 s
        self.timer = self.create_timer(0.05, self.timer_callback)

    def get_yaw(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return yaw
        except Exception as e:
            self.get_logger().warn_throttle(5.0, f"Transform not available: {e}")
            return None

    def timer_callback(self):
        yaw = self.get_yaw()
        if yaw is None:
            return

        if self.state == 'WAIT_FOR_YAW':
            self.current_yaw = yaw
            self.target_yaw = self.normalize_angle(yaw + math.pi / 2)
            self.get_logger().info(f"Rotating to {math.degrees(self.target_yaw):.2f}°")
            self.state = 'ROTATING'
            return

        if self.state == 'ROTATING':
            angle_error = self.normalize_angle(self.target_yaw - yaw)
            if abs(angle_error) > math.radians(5):
                twist = Twist()
                twist.angular.z = 0.4 * (1 if angle_error > 0 else -1)
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())  # stop
                self.get_logger().info(f"Reached {math.degrees(yaw):.2f}°")
                self.state = 'WAIT'
                self.wait_start = time.time()

        elif self.state == 'WAIT':
            if time.time() - self.wait_start > 1.0:
                self.state = 'WAIT_FOR_YAW'

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = Rotate90()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
