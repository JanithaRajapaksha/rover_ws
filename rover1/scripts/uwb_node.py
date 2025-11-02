#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import serial
import math


def trilaterate(d1, d2, anchor1, anchor2):
    x1, y1 = anchor1
    x2, y2 = anchor2

    dx = x2 - x1
    dy = y2 - y1
    dist = math.hypot(dx, dy)

    # Check for intersection possibility
    if dist > (d1 + d2) or dist < abs(d1 - d2):
        return None  # No intersection

    a = (d1**2 - d2**2 + dist**2) / (2 * dist)
    h = math.sqrt(max(d1**2 - a**2, 0))

    x0 = x1 + a * dx / dist
    y0 = y1 + a * dy / dist

    inter1 = (x0 + h * dy / dist, y0 - h * dx / dist)
    inter2 = (x0 - h * dy / dist, y0 + h * dx / dist)

    return inter1, inter2


class UWBCirclePublisher(Node):
    def __init__(self):
        super().__init__('uwb_circle_publisher')
        self.pub = self.create_publisher(Marker, '/uwb_circles', 10)

        # Serial ports (update according to your setup)
        self.ser1 = serial.Serial('/dev/ttyUSB1', 115200, timeout=0)
        self.ser2 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)

        # Anchor positions
        self.anchor1 = (0.0, 0.0)
        self.anchor2 = (0.3, 0.0)

        self.timer = self.create_timer(0.1, self.publish_markers)

    def read_distance(self, ser):
        try:
            if ser.in_waiting:
                line = ser.readline().decode().strip()
                return float(line)
        except (ValueError, OSError):
            return None
        return None

    def publish_markers(self):
        d1 = self.read_distance(self.ser1)
        d2 = self.read_distance(self.ser2)

        if d1 is None or d2 is None:
            return

        self.get_logger().info(f"Anchor1 distance: {d1:.3f} m, Anchor2 distance: {d2:.3f} m")

        markers = []

        # --- Circles for anchors ---
        for i, (anchor, radius) in enumerate([(self.anchor1, d1), (self.anchor2, d2)]):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'uwb_circles'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = anchor[0]
            m.pose.position.y = anchor[1]
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = radius * 2
            m.scale.y = radius * 2
            m.scale.z = 0.01
            m.color.r = 0.0 if i == 0 else 1.0
            m.color.g = 1.0 if i == 0 else 0.0
            m.color.b = 0.0
            m.color.a = 0.4
            markers.append(m)

        # --- Anchor Centers (small spheres) ---
        for i, anchor in enumerate([self.anchor1, self.anchor2]):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'anchor_centers'
            m.id = 100 + i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = anchor[0]
            m.pose.position.y = anchor[1]
            m.pose.position.z = 0.02
            m.pose.orientation.w = 1.0
            m.scale.x = 0.02
            m.scale.y = 0.02
            m.scale.z = 0.02
            m.color.r = 0.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.color.a = 1.0
            markers.append(m)

        # --- Perpendicular Arrow at Midpoint ---
        x1, y1 = self.anchor1
        x2, y2 = self.anchor2
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2
        dx = x2 - x1
        dy = y2 - y1
        norm = math.hypot(dx, dy)

        if norm > 0:
            # perpendicular direction (normalized)
            perp_x = -dy / norm
            perp_y = dx / norm

            # Arrow marker
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'perpendicular_arrow'
            arrow.id = 200
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0
            arrow.scale.x = 0.02  # shaft diameter
            arrow.scale.y = 0.04  # head diameter
            arrow.scale.z = 0.04  # head length
            arrow.color.r = 1.0
            arrow.color.g = 1.0
            arrow.color.b = 0.0
            arrow.color.a = 1.0

            # Define start and end points of the arrow
            p_start = [mid_x, mid_y, 0.02]
            p_end = [mid_x + perp_x * 0.15, mid_y + perp_y * 0.15, 0.02]
            arrow.points = []
            from geometry_msgs.msg import Point
            arrow.points.append(Point(x=p_start[0], y=p_start[1], z=p_start[2]))
            arrow.points.append(Point(x=p_end[0], y=p_end[1], z=p_end[2]))

            markers.append(arrow)

        # --- Intersection points ---
        inters = trilaterate(d1, d2, self.anchor1, self.anchor2)
        if inters:
            for idx, (x, y) in enumerate(inters):
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'uwb_points'
                m.id = 10 + idx  # unique IDs
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = x
                m.pose.position.y = y
                m.pose.position.z = 0.05
                m.pose.orientation.w = 1.0
                m.scale.x = 0.02
                m.scale.y = 0.02
                m.scale.z = 0.02
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 1.0
                m.color.a = 1.0
                markers.append(m)

        # Publish all markers
        for m in markers:
            self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = UWBCirclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
