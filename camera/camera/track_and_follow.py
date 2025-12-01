#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import socket
import json
import threading
import time


class PersonTrackerPIDNode(Node):
    def __init__(self):
        super().__init__('person_tracker_pid_node')

        # --- Parameters ---
        self.declare_parameter('udp_port', 5005)
        self.declare_parameter('udp_ip', '0.0.0.0')

        # Angular PID (for centering)
        self.declare_parameter('kp_ang', 0.3)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.01)
        self.declare_parameter('max_ang_vel', 0.2)

        # Linear PID (for distance control)
        self.declare_parameter('kp_lin', 0.5)
        self.declare_parameter('ki_lin', 0.0)
        self.declare_parameter('kd_lin', 0.02)
        self.declare_parameter('max_lin_vel', 0.1)

        # Target values
        self.declare_parameter('target_x', 0.0)     # center normalized X
        self.declare_parameter('target_width', 0.5) # target bounding box width

        # --- Get parameter values ---
        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value

        self.kp_ang = self.get_parameter('kp_ang').get_parameter_value().double_value
        self.ki_ang = self.get_parameter('ki_ang').get_parameter_value().double_value
        self.kd_ang = self.get_parameter('kd_ang').get_parameter_value().double_value
        self.max_ang_vel = self.get_parameter('max_ang_vel').get_parameter_value().double_value

        self.kp_lin = self.get_parameter('kp_lin').get_parameter_value().double_value
        self.ki_lin = self.get_parameter('ki_lin').get_parameter_value().double_value
        self.kd_lin = self.get_parameter('kd_lin').get_parameter_value().double_value
        self.max_lin_vel = self.get_parameter('max_lin_vel').get_parameter_value().double_value

        self.target_x = self.get_parameter('target_x').get_parameter_value().double_value
        self.target_width = self.get_parameter('target_width').get_parameter_value().double_value

        # --- Publishers ---
        self.point_pub = self.create_publisher(Point, 'person_tracking_data', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_mp', 10)

        # --- UDP Setup ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.get_logger().info(f"Listening for UDP data on {self.udp_ip}:{self.udp_port}")

        # PID Variables (separate for angular & linear)
        self.prev_error_ang = 0.0
        self.integral_ang = 0.0

        self.prev_error_lin = 0.0
        self.integral_lin = 0.0

        self.last_time = time.time()

        # Start background UDP listener
        self.thread = threading.Thread(target=self.listen_udp, daemon=True)
        self.thread.start()

    def listen_udp(self):
        """Continuously listen for UDP packets and apply dual PID control."""
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                msg_str = data.decode().strip()
                json_data = json.loads(msg_str)

                # Extract values
                x = float(json_data.get("x", 0.0))
                width = float(json_data.get("width", 0.0))

                # Publish raw tracking data
                point_msg = Point()
                point_msg.x = x
                point_msg.y = width
                point_msg.z = 0.0
                self.point_pub.publish(point_msg)

                # Time difference for PID
                current_time = time.time()
                dt = current_time - self.last_time if self.last_time else 0.01

                # -------------------
                # Angular PID (goal: x = 0)
                # -------------------
                error_ang = self.target_x - x
                self.integral_ang += error_ang * dt
                derivative_ang = (error_ang - self.prev_error_ang) / dt if dt > 0 else 0.0

                angular_z = (
                    self.kp_ang * error_ang +
                    self.ki_ang * self.integral_ang +
                    self.kd_ang * derivative_ang
                )
                angular_z = max(-self.max_ang_vel, min(self.max_ang_vel, angular_z))

                # -------------------
                # Linear PID (goal: width = 0.5)
                # -------------------
                error_lin = self.target_width - width
                self.integral_lin += error_lin * dt
                derivative_lin = (error_lin - self.prev_error_lin) / dt if dt > 0 else 0.0

                linear_x = (
                    self.kp_lin * error_lin +
                    self.ki_lin * self.integral_lin +
                    self.kd_lin * derivative_lin
                )
                linear_x = max(-self.max_lin_vel, min(self.max_lin_vel, linear_x))

                # -------------------
                # Publish Twist command
                # -------------------
                twist = Twist()
                twist.linear.x = linear_x
                twist.angular.z = angular_z
                self.cmd_vel_pub.publish(twist)

                # Log (optional)
                self.get_logger().info(
                    f"x={x:.3f}, w={width:.3f}, err_x={error_ang:.3f}, err_w={error_lin:.3f}, lin_x={linear_x:.3f}, ang_z={angular_z:.3f}"
                )

                # Update PID states
                self.prev_error_ang = error_ang
                self.prev_error_lin = error_lin
                self.last_time = current_time

            except Exception as e:
                self.get_logger().warn(f"UDP receive error: {e}")

    def destroy_node(self):
        """Clean shutdown."""
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
