#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
import math
import time


class MoveAndRotate(Node):
    def __init__(self):
        super().__init__('move_and_rotate_tf')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_scaled', 10)

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # FSM states
        self.state = 'WAIT_FOR_YAW'
        self.current_yaw = None
        self.target_yaw = None

        # Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Motion control
        self.move_state = None
        self.move_start_mag = 0.0
        self.wait_start = 0.0

    # -----------------------------------------------------------
    # Utility: reset TF buffer (flush)
    def reset_tf_buffer(self):
        self.get_logger().info("Resetting TF buffer...")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        time.sleep(0.3)  # short delay to refill TF cache

    # -----------------------------------------------------------
    def get_yaw(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return yaw
        except Exception:
            return None

    # -----------------------------------------------------------
    def get_magnitude(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return math.sqrt(x ** 2 + y ** 2)
        except Exception:
            return None

    # -----------------------------------------------------------
    def move_forward_backward(self):
        mag = self.get_magnitude()
        if mag is None:
            return

        if self.move_state == 'FORWARD':
            twist = Twist()
            twist.linear.x = 0.15
            self.cmd_pub.publish(twist)
            if mag - self.move_start_mag > 0.5:
                self.cmd_pub.publish(Twist())
                self.move_state = 'WAIT_FORWARD'
                self.wait_start = time.time()

        elif self.move_state == 'WAIT_FORWARD':
            if time.time() - self.wait_start > 1.0:
                self.move_state = 'BACKWARD'
                self.move_start_mag = self.get_magnitude() or 0.0

        elif self.move_state == 'BACKWARD':
            twist = Twist()
            twist.linear.x = -0.15
            self.cmd_pub.publish(twist)
            if abs(mag - self.move_start_mag) > 0.5:
                self.cmd_pub.publish(Twist())
                self.move_state = 'WAIT_BACKWARD'
                self.wait_start = time.time()

        elif self.move_state == 'WAIT_BACKWARD':
            if time.time() - self.wait_start > 1.0:
                self.move_state = 'DONE'

    # -----------------------------------------------------------
    def timer_callback(self):
        yaw = self.get_yaw()
        if yaw is None:
            return

        # --- Initial orientation setup ---
        if self.state == 'WAIT_FOR_YAW':
            self.current_yaw = yaw
            self.target_yaw = self.normalize_angle(yaw + math.pi / 2)
            self.get_logger().info(f"Rotating to {math.degrees(self.target_yaw):.2f}°")
            self.state = 'ROTATING'
            return

        # --- Rotation stage ---
        if self.state == 'ROTATING':
            angle_error = self.normalize_angle(self.target_yaw - yaw)
            if abs(angle_error) > math.radians(5):
                twist = Twist()
                twist.angular.z = 0.4 * (1 if angle_error > 0 else -1)
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f"Reached {math.degrees(yaw):.2f}°")
                self.state = 'MOVING'
                self.move_state = 'FORWARD'
                self.move_start_mag = self.get_magnitude() or 0.0

        # --- Forward/backward cycle ---
        elif self.state == 'MOVING':
            self.move_forward_backward()
            if self.move_state == 'DONE':
                self.get_logger().info("Movement cycle complete — waiting...")
                self.state = 'WAIT'
                self.wait_start = time.time()

        # --- Wait before next rotation ---
        elif self.state == 'WAIT':
            if time.time() - self.wait_start > 1.0:
                self.reset_tf_buffer()
                self.state = 'WAIT_FOR_YAW'

    # -----------------------------------------------------------
    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


# ---------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MoveAndRotate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
