#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import time

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_scaled', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.declare_parameter('rotation_speed', 0.3)
        self.declare_parameter('pause_time', 1.0)
        self.running = True

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def rotate_to(self, target_yaw, tolerance=0.05):
        """Rotate robot to a specific yaw (radians)"""
        speed = self.get_parameter('rotation_speed').value
        twist = Twist()

        while abs(self.angle_diff(target_yaw, self.current_yaw)) > tolerance and rclpy.ok():
            diff = self.angle_diff(target_yaw, self.current_yaw)
            direction = 1.0 if diff > 0 else -1.0
            twist.angular.z = speed * direction
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        # stop rotation
        self.cmd_pub.publish(Twist())

    def angle_diff(self, target, current):
        """Compute shortest angular difference"""
        a = target - current
        return math.atan2(math.sin(a), math.cos(a))

    def continuous_rotation(self):
        """Rotate 90 degrees at a time, pause 1 second, then continue"""
        pause_time = self.get_parameter('pause_time').value
        step_angle = math.radians(90)  # 90 degrees

        self.target_yaw = self.current_yaw

        self.get_logger().info("Starting 90-degree step rotation loop...")

        while rclpy.ok() and self.running:
            self.target_yaw += step_angle
            self.target_yaw = math.atan2(math.sin(self.target_yaw), math.cos(self.target_yaw))  # wrap angle
            self.get_logger().info(f"Rotating to yaw: {math.degrees(self.target_yaw):.1f}°")
            self.rotate_to(self.target_yaw)
            self.get_logger().info("Rotation done. Pausing for 1 second...")
            time.sleep(pause_time)


def main(args=None):
    rclpy.init(args=args)
    mover = RobotMover()

    try:
        mover.continuous_rotation()
    except KeyboardInterrupt:
        mover.get_logger().info("Stopping rotation loop.")
    finally:
        mover.running = False
        mover.cmd_pub.publish(Twist())
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import time

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_scaled', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.declare_parameter('rotation_speed', 0.3)
        self.declare_parameter('pause_time', 1.0)
        self.running = True

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def rotate_to(self, target_yaw, tolerance=0.05):
        """Rotate robot to a specific yaw (radians)"""
        speed = self.get_parameter('rotation_speed').value
        twist = Twist()

        while abs(self.angle_diff(target_yaw, self.current_yaw)) > tolerance and rclpy.ok():
            diff = self.angle_diff(target_yaw, self.current_yaw)
            direction = 1.0 if diff > 0 else -1.0
            twist.angular.z = speed * direction
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        # stop rotation
        self.cmd_pub.publish(Twist())

    def angle_diff(self, target, current):
        """Compute shortest angular difference"""
        a = target - current
        return math.atan2(math.sin(a), math.cos(a))

    def continuous_rotation(self):
        """Rotate 90 degrees at a time, pause 1 second, then continue"""
        pause_time = self.get_parameter('pause_time').value
        step_angle = math.radians(90)  # 90 degrees

        self.target_yaw = self.current_yaw

        self.get_logger().info("Starting 90-degree step rotation loop...")

        while rclpy.ok() and self.running:
            self.target_yaw += step_angle
            self.target_yaw = math.atan2(math.sin(self.target_yaw), math.cos(self.target_yaw))  # wrap angle
            self.get_logger().info(f"Rotating to yaw: {math.degrees(self.target_yaw):.1f}°")
            self.rotate_to(self.target_yaw)
            self.get_logger().info("Rotation done. Pausing for 1 second...")
            time.sleep(pause_time)


def main(args=None):
    rclpy.init(args=args)
    mover = RobotMover()

    try:
        mover.continuous_rotation()
    except KeyboardInterrupt:
        mover.get_logger().info("Stopping rotation loop.")
    finally:
        mover.running = False
        mover.cmd_pub.publish(Twist())
        rclpy.shutdown()


if __name__ == '__main__':
    main()
