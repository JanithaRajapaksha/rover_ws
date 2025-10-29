#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import math

PORT = "/dev/ttyAMA1"
BAUD = 115200
NUM_SENSORS = 8

# --- Sensor configuration ---
SENSOR_ANGLES = [0.0, math.pi/4, math.pi/2, 3*math.pi/4, math.pi, 5*math.pi/4, 3*math.pi/2, 7*math.pi/4]  # radians
SENSOR_OFFSETS = [0.0, -0.02, 0.01, 0.0, 0.0, 0.03, -0.01, 0.0]  # meters, adjust after calibration

class ToFScanPublisher(Node):
    def __init__(self):
        super().__init__('tof_scan_publisher')

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.05, self.publish_scan)  # 20 Hz

        # Open serial port
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
            self.get_logger().info(f'Opened serial port {PORT} at {BAUD} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.ser = None

    def read_tof_values(self):
        if self.ser is None:
            return [float('inf')] * NUM_SENSORS

        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return [float('inf')] * NUM_SENSORS

            # Example format from MCU: "123,456,789,123,456,789,123,456"
            values = [float(x)/1000.0 for x in line.split(',')]  # mm -> m
            if len(values) != NUM_SENSORS:
                return [float('inf')] * NUM_SENSORS

            # Apply offsets
            values = [max(0.0, v + offset) for v, offset in zip(values, SENSOR_OFFSETS)]
            return values

        except Exception as e:
            self.get_logger().warn(f'Error reading serial: {e}')
            return [float('inf')] * NUM_SENSORS

    def publish_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'base_link'

        # LaserScan geometry
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2 * math.pi
        scan_msg.angle_increment = 2 * math.pi / NUM_SENSORS
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.05
        scan_msg.range_min = 0.01
        scan_msg.range_max = 2.0

        # Read actual ToF sensor values
        ranges = self.read_tof_values()

        # Map sensor readings to the LaserScan array based on SENSOR_ANGLES
        scan_array = [float('inf')] * NUM_SENSORS
        for i, angle in enumerate(SENSOR_ANGLES):
            index = int(angle / scan_msg.angle_increment) % NUM_SENSORS
            scan_array[index] = ranges[i]

        scan_msg.ranges = scan_array
        scan_msg.intensities = [0.0] * NUM_SENSORS

        self.publisher.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ToFScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down ToF Scan Publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()






# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# import math
# import random

# class VirtualScanPublisher(Node):
#     def __init__(self):
#         super().__init__('virtual_scan_publisher')

#         # Enable simulation time if set
#         # self.declare_parameter('use_sim_time', True)

#         self.publisher = self.create_publisher(LaserScan, '/scan', 10)
#         timer_period = 0.05  # 20 Hz
#         self.timer = self.create_timer(timer_period, self.publish_scan)

#         self.get_logger().info('Virtual Scan Publisher started!')

#     def publish_scan(self):
#         scan_msg = LaserScan()
#         scan_msg.header.stamp = self.get_clock().now().to_msg()  # timestamp
#         scan_msg.header.frame_id = 'base_link'

#         # LaserScan parameters
#         scan_msg.angle_min = 0.0
#         scan_msg.angle_max = 2 * math.pi
#         scan_msg.angle_increment = 2 * math.pi / 8  # 8 readings
#         scan_msg.time_increment = 0.0
#         scan_msg.scan_time = 0.0  # corresponds to 20Hz
#         scan_msg.range_min = 0.01
#         scan_msg.range_max = 2.0

#         # Random ranges between 0.5 and 1.5 meters
#         scan_msg.ranges = [
#     float('inf'),       # 0° (front-left)
#     1.5,  # 45°
#     1.0,  # 90°
#     1.5,  # 135°
#     float('inf'),  # 180°
#     1.5,  # 225°
#     1.0,  # 270°
#     1.5       # 315° (front-right)
# ]
#         scan_msg.intensities = [0.0] * 8

#         self.publisher.publish(scan_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = VirtualScanPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Shutting down Virtual Scan Publisher...')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
