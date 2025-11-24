#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import re

from std_msgs.msg import Float32


class UWBReader(Node):
    def __init__(self):
        super().__init__('pose_pub')

        # --- Parameters ---
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_topic', '/uwb_distance')

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        topic = self.get_parameter('publish_topic').get_parameter_value().string_value

        # --- Serial setup ---
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"✅ Connected to UWB on {port} @ {baud} baud")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to open serial port: {e}")
            
            self.ser = None

        # --- Publisher ---
        self.pub = self.create_publisher(Float32, topic, 10)

        # --- Timer (read every 100 ms) ---
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            line = self.ser.readline().decode(errors='ignore').strip()

            if not line:
                return

            # Extract only float or int numbers (ignoring letters)
            numbers = re.findall(r"[-+]?\d*\.\d+|\d+", line)

            if numbers:
                # Take the first valid number
                value = float(numbers[0])

                msg = Float32()
                msg.data = value
                self.pub.publish(msg)

                self.get_logger().info(f"UWB Distance: {value:.3f} m | Raw: {line}")

        except Exception as e:
            self.get_logger().warn_throttle(5.0, f"Error reading serial: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UWBReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
