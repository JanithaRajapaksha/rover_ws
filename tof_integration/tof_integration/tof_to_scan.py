#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import math

PORT = "/dev/ttyAMA1"
BAUD = 115200
NUM_SENSORS = 8

class ToFScanPublisher(Node):
    def __init__(self):
        super().__init__('tof_scan_publisher')

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.000001, self.publish_scan)  # 1 MHz

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

            # Reorder manually: make the 5th sensor (index 4) first
            reordered = [
                values[5]+0.15,  # 6th sensor → first
                values[2]+0.15,  # 5th sensor → second
                values[6],  # 7th → third
                values[7],  # 8th → fourth
                values[3],  # 1st → fifth
                values[1],  # 2nd → sixth
                values[4],  # 3rd → seventh
                values[0]+0.15   # 4th → eighth
            ]

            # mask_indices = [6, 7, 3, 1, 4]
            # for i in mask_indices:
            #     reordered[i] = float('inf')

            return reordered

        except Exception as e:
            self.get_logger().warn(f'Error reading serial: {e}')
            return [float('inf')] * NUM_SENSORS

    def publish_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'

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

        scan_msg.ranges = ranges
        scan_msg.intensities = [0.0] * NUM_SENSORS

        self.get_logger().info(f"LaserScan ranges (m): {['%.2f' % r for r in scan_msg.ranges]}")

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
