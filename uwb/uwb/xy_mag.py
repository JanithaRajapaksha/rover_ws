#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import math

class XYMagnitudeTF(Node):
    def __init__(self):
        super().__init__('xy_magnitude_tf')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.05, self.timer_callback)

    def get_translation(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().warn_throttle(5.0, f"Transform not available: {e}")
            return None, None

    def timer_callback(self):
        x, y = self.get_translation()
        if x is None or y is None:
            return

        magnitude = math.sqrt(x**2 + y**2)
        self.get_logger().info(f"Translation: x={x:.3f}, y={y:.3f} | Magnitude={magnitude:.3f} m")

def main(args=None):
    rclpy.init(args=args)
    node = XYMagnitudeTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
