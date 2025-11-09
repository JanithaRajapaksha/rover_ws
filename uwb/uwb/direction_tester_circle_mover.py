#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
import math
import time

class CircleMovement(Node):
    def __init__(self):
        super().__init__('circle_movement')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_scaled', 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = 'WAIT_BEFORE_FORWARD'
        self.wait_start_time = time.time()

        # State machine
        self.state = 'MOVE_FORWARD'
        self.start_time = time.time()

        # Orientation tracking
        self.start_yaw = None
        self.current_yaw = None
        self.target_yaw = None

        # Position tracking
        self.start_x = None
        self.start_y = None

        self.timer = self.create_timer(0.05, self.timer_callback)

    def get_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return x, y, yaw
        except Exception as e:
            self.get_logger().warn(f"TF not available: {e}")
            return None, None, None

    def timer_callback(self):
        x, y, yaw = self.get_pose()
        if yaw is None:
            return

        # --- Manual yaw loggin  every 0.1s ---
        now = time.time()
        if not hasattr(self, "last_log_time"):
            self.last_log_time = 0
        if now - self.last_log_time > 0.1:
            self.get_logger().info(f"Current yaw: {math.degrees(yaw):.2f}°")
            self.last_log_time = now

        twist = Twist()

        if self.start_x is None:
            self.start_x, self.start_y, self.start_yaw = x, y, yaw

        if self.state == 'WAIT_BEFORE_FORWARD':
            if time.time() - self.wait_start_time < 2.0:  # <-- delay time in seconds
                self.cmd_pub.publish(Twist())  # keep still
                return
            else:
                self.start_time = time.time()
                self.state = 'MOVE_FORWARD'
                self.get_logger().info("Starting to move forward")


        # ---- STATE MACHINE ----
        if self.state == 'MOVE_FORWARD':
            twist.linear.x = 0.15
            self.cmd_pub.publish(twist)

            if time.time() - self.start_time > 2.0:
                self.cmd_pub.publish(Twist())
                self.state = 'TURN_90'
                self.target_yaw = self.normalize_angle(self.start_yaw + math.pi / 2)
                self.get_logger().info(f"Turning +90°, target yaw = {math.degrees(self.target_yaw):.2f}°")

        elif self.state == 'TURN_90':
            angle_error = self.normalize_angle(self.target_yaw - yaw)
            if abs(angle_error) > math.radians(3):
                twist.angular.z = 0.3 * (1 if angle_error > 0 else -1)
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())
                self.circle_start_yaw = yaw
                self.get_logger().info(f"Circle start yaw: {math.degrees(yaw):.2f}°")
                self.state = 'MAKE_CIRCLE'
                self.circle_start_time = time.time()

        elif self.state == 'MAKE_CIRCLE':
            twist.linear.x = 0.1
            twist.angular.z = 0.2
            self.cmd_pub.publish(twist)

            yaw_diff = self.normalize_angle(yaw - self.circle_start_yaw)

            # Target yaw for correction (finish a little beyond the start angle)
            corrected_target_yaw = self.normalize_angle(self.circle_start_yaw + math.radians(20))  # +20° overshoot for correction
            angle_error = self.normalize_angle(corrected_target_yaw - yaw)

            # Detect completion — when yaw reaches corrected angle
            if abs(angle_error) < math.radians(3) and (time.time() - self.circle_start_time) > 1.0:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(
                    f"Circle + correction done. Current yaw: {math.degrees(yaw):.2f}°, target: {math.degrees(corrected_target_yaw):.2f}°"
                )
                self.state = 'TURN_BACK'
                self.target_yaw = self.normalize_angle(yaw - math.pi / 2)
                self.get_logger().info(f"Turning -90° to return, target yaw = {math.degrees(self.target_yaw):.2f}°")


        elif self.state == 'TURN_BACK':
            angle_error = self.normalize_angle(self.target_yaw - yaw)
            if abs(angle_error) > math.radians(3):
                twist.angular.z = 0.3 * (1 if angle_error > 0 else -1)
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f"Returned to original orientation: {math.degrees(yaw):.2f}°")
                self.state = 'MOVE_BACKWARD'
                self.start_back_time = time.time()

        elif self.state == 'MOVE_BACKWARD':
            twist.linear.x = -0.15  # move backward
            self.cmd_pub.publish(twist)
            if time.time() - self.start_back_time > 2.0:
                self.cmd_pub.publish(Twist())
                self.state = 'DONE'
                self.get_logger().info("Moved backward. Stopping ✅")

        elif self.state == 'DONE':
            self.cmd_pub.publish(Twist())  # fully stop

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = CircleMovement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32

# class UWBReader(Node):
#     def __init__(self):
#         super().__init__('uwb_reader')
#         # Subscribe to the topic
#         self.subscription = self.create_subscription(
#             Float32,
#             '/uwb_distance',      # <-- topic name
#             self.uwb_callback,
#             10
#         )
#         self.subscription  # prevent unused variable warning
#         self.get_logger().info("UWB Reader Node started, listening to /uwb_distance")

#     def uwb_callback(self, msg):
#         distance = msg.data
#         self.get_logger().info(f"UWB Distance: {distance:.3f} m")

# def main(args=None):
#     rclpy.init(args=args)
#     node = UWBReader()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

