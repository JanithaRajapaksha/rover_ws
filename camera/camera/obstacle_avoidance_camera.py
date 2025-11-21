#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import json
import numpy as np
import time


class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, error):
        now = time.perf_counter()  # high-precision clock
        dt = (now - self.last_time) if self.last_time else 0.0
        self.last_time = now

        # Clamp dt (helps with timing jitter)
        if dt <= 0.0001:
            dt = 0.0001

        # PID computation
        p = self.kp * error
        self.integral += error * dt
        i = self.ki * self.integral
        d = self.kd * ((error - self.prev_error) / dt)
        self.prev_error = error

        return p + i + d


class VisionPIDController(Node):
    def __init__(self):
        super().__init__('vision_pid_controller')

        # Parameters (tunable at runtime)
        self.declare_parameter('max_linear_vel', 0.1)
        self.declare_parameter('max_angular_vel', 0.3)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.02)
        self.declare_parameter('kp_angular', 0.5)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.01)
        self.declare_parameter('stop_threshold', 0.25)  # stop if vertical < this

        self.declare_parameter('disable_linear', True)  # default: False
        # deadzone for horizontal error (e.g. small joystick/camera jitter) — values inside
        # [-horizontal_deadzone, horizontal_deadzone] are treated as zero
        self.declare_parameter('horizontal_deadzone', 0.05)


        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_camera', 10)

        # UDP Socket
        UDP_IP, UDP_PORT = "127.0.0.1", 5006
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.setblocking(False)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)

        # PID controllers
        self.linear_pid = PID(
            self.get_parameter('kp_linear').value,
            self.get_parameter('ki_linear').value,
            self.get_parameter('kd_linear').value
        )
        self.angular_pid = PID(
            self.get_parameter('kp_angular').value,
            self.get_parameter('ki_angular').value,
            self.get_parameter('kd_angular').value
        )

        self.last_data = {"vertical": 1.0, "horizontal": 0.0}
        self.last_update_time = time.perf_counter()
        self.last_print_time = self.last_update_time
        self.new_packet = False
        # Throttle interval for status logging (seconds)
        self.declare_parameter('log_throttle', 0.01)
        self.log_throttle = self.get_parameter('log_throttle').value
        # For gradual linear speed changes
        self.current_linear_vel = 0.0
        self.last_linear_update = time.perf_counter()
        # smoothing time constant (seconds) for first-order filter; smaller = faster
        self.declare_parameter('linear_smoothing_tc', 0.2)
        self.linear_smoothing_tc = max(0.001, float(self.get_parameter('linear_smoothing_tc').value))

        # Read deadzone parameter into an attribute for quick access
        try:
            self.horizontal_deadzone = float(self.get_parameter('horizontal_deadzone').value)
        except Exception:
            self.horizontal_deadzone = 0.5

            # Timer: 50 Hz loop (20 ms) — good balance for real-time control
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("🚀 Real-time Vision PID controller started (50 Hz)")

    def read_udp_data(self):
        try:
            data, _ = self.sock.recvfrom(512)
            msg = json.loads(data.decode('utf-8'))
            self.last_data = msg
            self.last_update_time = time.perf_counter()
            # mark that we received a fresh packet so logging can be emitted immediately
            self.new_packet = True
        except BlockingIOError:
            # No new packet — keep last values
            pass
        except Exception as e:
            self.get_logger().warn(f"UDP error: {e}")

    def control_loop(self):
        self.read_udp_data()
        vertical = float(self.last_data.get("vertical", 1.0))
        horizontal = float(self.last_data.get("horizontal", 0.0))

        # Apply horizontal deadzone to avoid reacting to tiny jitter
        if abs(horizontal) < getattr(self, 'horizontal_deadzone', 0.0):
            horizontal = 0.0

        # Compute PID errors for angular control
        angular_error = -horizontal
        angular_cmd = self.angular_pid.compute(angular_error)

        # Limits and thresholds
        max_lin = float(self.get_parameter('max_linear_vel').value)
        max_ang = float(self.get_parameter('max_angular_vel').value)
        stop_threshold = float(self.get_parameter('stop_threshold').value)
        disable_linear = self.get_parameter('disable_linear').value  # new param

        # ----------------- Linear control -----------------
        if not disable_linear:
            # Desired linear speed proportional to vertical free space
            desired_linear = float(np.clip(vertical, 0.0, 1.0)) * max_lin

            # Smooth first-order filter
            now_lin = time.perf_counter()
            dt_lin = now_lin - self.last_linear_update if self.last_linear_update else 0.0
            if dt_lin <= 0.0:
                dt_lin = 0.0001
            alpha = 1.0 - np.exp(-dt_lin / self.linear_smoothing_tc)
            self.current_linear_vel += (desired_linear - self.current_linear_vel) * alpha

            # Safety stop if vertical free space is too low
            if vertical < stop_threshold:
                self.current_linear_vel = 0.0

            self.last_linear_update = now_lin
            linear_vel = float(np.clip(self.current_linear_vel, 0.0, max_lin))
        else:
            linear_vel = 0.1  # linear motion disabled

        # ----------------- Angular control -----------------
        if horizontal == 0.0:
            angular_vel = 0.0
        else:
            angular_vel = float(np.clip(angular_cmd, -max_ang, max_ang))

        # ----------------- Publish Twist -----------------
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_pub.publish(twist)

        # ----------------- Throttled logging -----------------
        now = time.time()
        if self.new_packet:
            self.get_logger().info(
                f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] Vert={vertical:.2f} Horz={horizontal:.2f} | "
                f"v={linear_vel:.2f} w={angular_vel:.2f}"
            )
            self.last_print_time = now
            self.new_packet = False
        elif now - self.last_print_time > float(self.log_throttle):
            self.get_logger().info(
                f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] Vert={vertical:.2f} Horz={horizontal:.2f} | "
                f"v={linear_vel:.2f} w={angular_vel:.2f}"
            )
            self.last_print_time = now



def main(args=None):
    rclpy.init(args=args)
    node = VisionPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
