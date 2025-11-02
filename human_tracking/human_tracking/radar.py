#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import serial
import math
from collections import deque

# ==============================================================
# Target object
# ==============================================================
class Target:
    def __init__(self, x, y, speed, pixel_distance):
        self.x = x
        self.y = y
        self.speed = speed
        self.pixel_distance = pixel_distance
        self.distance = math.sqrt(x**2 + y**2)
        self.angle = math.degrees(math.atan2(x, y))
    
    def __str__(self):
        return (f"Target(x={self.x}mm, y={self.y}mm, speed={self.speed}cm/s, "
                f"distance={self.distance:.1f}mm, angle={self.angle:.1f}°)")

# ==============================================================
# RD-03D Radar driver
# ==============================================================
class RD03D:
    SINGLE_TARGET_CMD = bytes([0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01])
    MULTI_TARGET_CMD  = bytes([0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01])
    
    def __init__(self, uart_port='/dev/ttyAMA0', baudrate=256000, multi_mode=True):
        self.uart = serial.Serial(uart_port, baudrate, timeout=0.1)
        self.targets = []
        self.buffer = b''
        time.sleep(0.05)
        self.set_multi_mode(multi_mode)
    
    def set_multi_mode(self, multi_mode=True):
        cmd = self.MULTI_TARGET_CMD if multi_mode else self.SINGLE_TARGET_CMD
        self.uart.write(cmd)
        self.uart.flush()
        time.sleep(0.2)
        self.uart.reset_input_buffer()
        self.buffer = b''
        self.multi_mode = multi_mode
    
    @staticmethod
    def parse_signed16(high, low):
        raw = (high << 8) + low
        sign = 1 if (raw & 0x8000) else -1
        value = raw & 0x7FFF
        return sign * value
    
    def _decode_frame(self, data):
        targets = []
        if len(data) < 30 or data[0] != 0xAA or data[1] != 0xFF or data[-2] != 0x55 or data[-1] != 0xCC:
            return targets
        for i in range(3):
            base = 4 + i*8
            x = self.parse_signed16(data[base+1], data[base])
            y = self.parse_signed16(data[base+3], data[base+2])
            speed = self.parse_signed16(data[base+5], data[base+4])
            pixel_dist = data[base+6] + (data[base+7] << 8)
            targets.append(Target(x, y, speed, pixel_dist))
        return targets
    
    def _find_complete_frame(self, data):
        start_idx = -1
        for i in range(len(data)-1):
            if data[i] == 0xAA and data[i+1] == 0xFF:
                start_idx = i
                break
        if start_idx == -1:
            return None, data
        for i in range(start_idx+2, len(data)-1):
            if data[i] == 0x55 and data[i+1] == 0xCC:
                frame = data[start_idx:i+2]
                remaining = data[i+2:]
                return frame, remaining
        return None, data[start_idx:]
    
    def update(self):
        if self.uart.in_waiting > 0:
            self.buffer += self.uart.read(self.uart.in_waiting)
        if len(self.buffer) > 300:
            self.buffer = self.buffer[-150:]
        latest_frame = None
        temp = self.buffer
        while True:
            frame, temp = self._find_complete_frame(temp)
            if frame:
                latest_frame = frame
            else:
                break
        if latest_frame:
            self.buffer = self.buffer[self.buffer.rfind(latest_frame) + len(latest_frame):]
            decoded = self._decode_frame(latest_frame)
            if decoded:
                self.targets = decoded
                return True
        return False
    
    def get_target(self, n=1):
        if 1 <= n <= len(self.targets):
            return self.targets[n-1]
        return None
    
    def close(self):
        if self.uart.is_open:
            self.uart.close()

# ==============================================================
# Moving Average
# ==============================================================
class MovingAverage:
    def __init__(self, window_size=5):
        from collections import deque
        self.values = deque(maxlen=window_size)
    def update(self, v):
        self.values.append(v)
        return sum(self.values)/len(self.values)

# ==============================================================
# PID Controller
# ==============================================================
class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, dt=0.1, limits=(-1.0,1.0)):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.i = 0
        self.prev = 0
        self.min, self.max = limits
    def update(self, e):
        self.i += e*self.dt
        d = (e - self.prev)/self.dt
        out = self.kp*e + self.ki*self.i + self.kd*d
        out = max(self.min, min(self.max, out))
        self.prev = e
        return out

# ==============================================================
# Radar Tracking Node
# ==============================================================
class RadarTrackingNode(Node):
    def __init__(self):
        super().__init__("radar_pid_tracking_node")

        # Initialize radar
        self.radar = RD03D()
        self.radar.set_multi_mode(True)

        # Filters
        self.dist_avg = MovingAverage(5)
        self.angle_avg = MovingAverage(5)

        # PID controllers
        self.angular_pid = PID(kp=0.02, kd=0.001, dt=0.05, limits=(-0.5,0.5))
        self.linear_pid  = PID(kp=0.02, kd=0.001, dt=0.05, limits=(-0.1,0.1))

        # Goals
        self.angle_goal = 0.0
        self.angle_deadzone = 20.0      # degrees
        self.dist_goal = 800.0          # mm
        self.dist_deadzone = 200.0      # mm

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_tracking", 10)

        # Timer 20 Hz
        self.create_timer(0.05, self.loop)
        self.get_logger().info("Radar PID Tracking Node with linear control started.")

    def loop(self):
        if not self.radar.update():
            return

        # Get valid targets
        targets = [t for t in [self.radar.get_target(1),
                               self.radar.get_target(2),
                               self.radar.get_target(3)] if t and t.distance > 0 and abs(t.angle) <= 50]
        if not targets:
            return

        closest = min(targets, key=lambda t: t.distance)
        filtered_angle = self.angle_avg.update(closest.angle)
        filtered_dist  = self.dist_avg.update(closest.distance)

        # Angular control
        angle_error = self._angle_error_deadzone_excess(filtered_angle)
        angular_z = -self.angular_pid.update(angle_error)

        # Linear control (distance)
        dist_error = self._distance_error_deadzone_excess(filtered_dist)
        linear_x = self.linear_pid.update(dist_error)

        # Publish Twist
        cmd = Twist()
        cmd.linear.x = -linear_x
        cmd.angular.z = -angular_z
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"D={filtered_dist:.0f}mm | E_d={dist_error:+.0f} | "
            f"A={filtered_angle:+.1f}° | E_a={angle_error:+.1f} | "
            f"lin={linear_x:+.3f}, ang={angular_z:+.3f}"
        )

    # ==========================================================
    def _angle_error_deg(self, measured, goal=0.0):
        m, g = (measured+360)%360, (goal+360)%360
        diff = m - g
        if diff >= 180: diff -= 360
        if diff < -180: diff += 360
        return diff

    def _angle_error_deadzone_excess(self, measured):
        e = self._angle_error_deg(measured, self.angle_goal)
        if e > self.angle_deadzone: return e - self.angle_deadzone
        elif e < -self.angle_deadzone: return e + self.angle_deadzone
        return 0.0

    def _distance_error_deadzone_excess(self, measured):
        """Positive error -> move forward, negative -> backward"""
        error = self.dist_goal - measured
        if abs(error) <= self.dist_deadzone:
            return 0.0
        return error - self.dist_deadzone * math.copysign(1, error)

# ==============================================================
def main(args=None):
    rclpy.init(args=args)
    node = RadarTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.radar.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
