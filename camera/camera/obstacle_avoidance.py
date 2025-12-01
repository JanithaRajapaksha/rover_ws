#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import serial
import time

class ToFPIDNode(Node):
    def __init__(self):
        super().__init__('tof_pid_node')

        # ----- Serial setup -----
        self.port = '/dev/ttyAMA1'
        self.baudrate = 115200
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"Connected to {self.port} at {self.baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            rclpy.shutdown()
            return

        # ----- Publisher -----
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_tof', 10)
        # Publisher for front 3 ToF distances
        self.tof_dist_pub = self.create_publisher(Float32MultiArray, '/tof_distances', 10)


        # ----- PID parameters -----
        self.kp = 0.01       # proportional gain
        self.ki = 0.0         # integral gain
        self.kd = 0.001       # derivative gain
        self.target_dist = 1000.0  # target distance in mm

        self.max_angular_z = 0.075  # rad/s (maximum angular velocity)

        # PID state
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

        # Loop at 200 Hz
        self.timer = self.create_timer(0.005, self.control_loop)

    def control_loop(self):
        """Read sensor data and perform angular correction with PID."""
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if not line:
                    return

                # Expecting: 8 comma-separated readings
                parts = line.split(',')
                if len(parts) != 8:
                    self.get_logger().warn(f"Invalid data: {line}")
                    return

                readings = [float(x) for x in parts]

                # Extract relevant sensors
                front_right = readings[2]
                front_left = readings[1]
                front_center = readings[0]
                left = readings[3]
                right = readings[4]

                # Publish front 3 distances
                dist_msg = Float32MultiArray()
                dist_msg.data = [front_left, front_center, front_right]
                self.tof_dist_pub.publish(dist_msg)


                # Log readings
                self.get_logger().info(
                    f"FL: {front_left:.0f} | FC: {front_center:.0f} | FR: {front_right:.0f}"
                )

                # Compute PID only if object closer than 1000 mm
                error = (front_left - front_right)
                current_time = time.time()
                dt = current_time - self.last_time if self.last_time else 0.1

                self.integral += error * dt
                derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

                pid_output = (
                    self.kp * error +
                    self.ki * self.integral +
                    self.kd * derivative
                )

                # ---- Limit maximum angular speed ----
                pid_output = max(-self.max_angular_z, min(self.max_angular_z, pid_output))

                # Build and publish a single Twist message per loop.
                twist = Twist()
                twist.angular.z = float(pid_output)
                twist.linear.x = 0.009  # Default forward speed

                # If the front-center sensor reports an obstacle closer than 500 mm,
                # back off by setting a negative linear.x (units: m/s).
                # Sensor readings are in mm, so compare directly and choose a
                # reasonable back-off speed (e.g. -0.15 m/s).
                if front_center < 300.0:
                    backoff_speed = -0.009  # m/s (negative to move backwards)
                    twist.linear.x = backoff_speed
                    self.get_logger().warn(f"Front center {front_center:.0f}mm < 300mm: backing off {backoff_speed} m/s")
                # else:
                #     # No backing off requested; keep forward/backward speed zero.
                #     twist.linear.x = 0.0

                self.cmd_pub.publish(twist)
                # --- Publish front distances ---
                dist_msg = Float32MultiArray()
                dist_msg.data = [front_left, front_center, front_right]
                self.tof_dist_pub.publish(dist_msg)

                self.get_logger().info(
                    f"error={error:.2f} | PID={pid_output:.3f} rad/s | FC={front_center:.0f}mm"
                )

                self.prev_error = error
                self.last_time = current_time

                # Clear integral when no obstacle (keep controller stable)
                self.integral = 0.0

        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ToFPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()