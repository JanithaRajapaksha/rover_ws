#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import serial
import math
from collections import deque

class Target:
    def __init__(self, x, y, speed, pixel_distance):
        self.x = x                  # mm
        self.y = y                  # mm
        self.speed = speed          # cm/s
        self.pixel_distance = pixel_distance  # mm
        self.distance = math.sqrt(x**2 + y**2)
        self.angle = math.degrees(math.atan2(x, y))
    
    def __str__(self):
        return ('Target(x={}mm, y={}mm, speed={}cm/s, pixel_dist={}mm, '
                'distance={:.1f}mm, angle={:.1f}°)').format(
                self.x, self.y, self.speed, self.pixel_distance, self.distance, self.angle)

class RD03D:
    SINGLE_TARGET_CMD = bytes([0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01])
    MULTI_TARGET_CMD  = bytes([0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01])
    
    def __init__(self, uart_port='/dev/ttyAMA1', baudrate=256000, multi_mode=True):
        self.uart = serial.Serial(uart_port, baudrate, timeout=0.1)
        self.targets = []  # Stores up to 3 targets
        self.buffer = b''  # Buffer to handle split messages
        time.sleep(0.05)
        self.set_multi_mode(multi_mode)
    
    def set_multi_mode(self, multi_mode=True):
        """Set Radar mode: True=Multi-target, False=Single-target"""
        cmd = self.MULTI_TARGET_CMD if multi_mode else self.SINGLE_TARGET_CMD
        self.uart.write(cmd)
        self.uart.flush()  # Force immediate send
        time.sleep(0.2)
        self.uart.reset_input_buffer()  # Clear buffer after switching
        self.buffer = b''  # Clear internal buffer too
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
            return targets  # invalid frame
        
        for i in range(3):
            base = 4 + i*8
            x = self.parse_signed16(data[base+1], data[base])
            y = self.parse_signed16(data[base+3], data[base+2])
            speed = self.parse_signed16(data[base+5], data[base+4])
            pixel_dist = data[base+6] + (data[base+7] << 8)
            targets.append(Target(x, y, speed, pixel_dist))
        
        return targets
    
    def _find_complete_frame(self, data):
        """Find a complete frame in the data buffer"""
        # Look for frame start (0xAA 0xFF)
        start_idx = -1
        for i in range(len(data) - 1):
            if data[i] == 0xAA and data[i+1] == 0xFF:
                start_idx = i
                break
        
        if start_idx == -1:
            return None, data  # No frame start found, keep all data
        
        # Look for frame end (0x55 0xCC) after the start
        for i in range(start_idx + 2, len(data) - 1):
            if data[i] == 0x55 and data[i+1] == 0xCC:
                # Found complete frame
                frame = data[start_idx:i+2]
                remaining = data[i+2:]
                return frame, remaining
        
        # Frame start found but no end yet, keep data from start
        return None, data[start_idx:]
    
    def update(self):
        """Update internal targets list with latest data from radar."""
        # Read all available data and add to buffer
        if self.uart.in_waiting > 0:
            new_data = self.uart.read(self.uart.in_waiting)
            self.buffer += new_data
        
        # If buffer gets too large, keep only the most recent data
        if len(self.buffer) > 300:  # ~10 frames worth
            self.buffer = self.buffer[-150:]  # Keep last ~5 frames
        
        # Try to extract the MOST RECENT complete frame
        latest_frame = None
        temp_buffer = self.buffer
        
        while True:
            frame, temp_buffer = self._find_complete_frame(temp_buffer)
            if frame:
                latest_frame = frame  # Keep updating to get the latest
            else:
                break
        
        # Update buffer to remaining data after last complete frame
        if latest_frame:
            # Find where the latest frame ends in the original buffer
            frame_end_pos = self.buffer.rfind(latest_frame) + len(latest_frame)
            self.buffer = self.buffer[frame_end_pos:]
            
            decoded = self._decode_frame(latest_frame)
            if decoded:
                self.targets = decoded
                return True  # Successful update
        
        return False  # No valid frame found
    
    def get_target(self, target_number=1):
        """Get a target by number (1-based index)."""
        if 1 <= target_number <= len(self.targets):
            return self.targets[target_number - 1]
        return None  # No such target
    
    def close(self):
        """Close the UART connection"""
        if self.uart.is_open:
            self.uart.close()


# ===============================
# Moving Average Filter
# ===============================
class MovingAverage:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)

    def update(self, value):
        self.values.append(value)
        return sum(self.values) / len(self.values)

# ===============================
# PID Controller
# ===============================
class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, dt=0.1, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.min_output, self.max_output = output_limits

    def update(self, error):
        # Integral term
        self.integral += error * self.dt
        # Derivative term
        derivative = (error - self.prev_error) / self.dt
        # PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        # Clamp output
        output = max(self.min_output, min(self.max_output, output))
        self.prev_error = error
        return output

# ===============================
# Radar Tracking Node
# ===============================
class RadarTrackingNode(Node):
    def __init__(self):
        super().__init__("radar_pid_tracking_node")

        # Initialize radar
        self.radar = RD03D()
        self.radar.set_multi_mode(True)

        # Moving averages
        self.dist_avg = MovingAverage(5)
        self.angle_avg = MovingAverage(5)

        # PID controller for angular.z
        self.angular_pid = PID(kp=0.02, ki=0.0, kd=0.001, dt=0.1, output_limits=(-0.5, 0.5))

        # Deadzone for angular error
        self.deadzone = 20.0  # degrees
        self.angle_goal = 0.0

        # Publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_tracking", 10)

        # Timer for real-time updates (~10 Hz)
        self.create_timer(0.05, self.loop)
        self.get_logger().info("Radar PID Tracking Node Started")

    # ===============================
    def loop(self):
        if not self.radar.update():
            # No data
            return

        # Get valid targets (distance > 0)
        targets = [t for t in [self.radar.get_target(1),
                               self.radar.get_target(2),
                               self.radar.get_target(3)] if t and t.distance > 0]
        if not targets:
            return

        # Closest target
        closest = min(targets, key=lambda t: t.distance)

        # Moving average
        filtered_dist = self.dist_avg.update(closest.distance)
        filtered_angle = self.angle_avg.update(closest.angle)

        # Compute angular error beyond deadzone
        angle_error = self._angular_error_deadzone_excess(filtered_angle)

        # Compute PID output for angular.z
        angular_z = self.angular_pid.update(angle_error)

        # Publish Twist message
        cmd = Twist()
        cmd.linear.x = 0.0  # only angular control
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

        # Optional logging
        self.get_logger().info(
            f"Filtered Angle: {filtered_angle:.1f} deg | Error: {angle_error:+.1f} deg | "
            f"Angular.z: {angular_z:.3f}"
        )

    # ===============================
    def _angle_error_deg(self, measured_deg, goal_deg=0.0):
        """Minimal signed difference [-180,180)"""
        m = (measured_deg + 360.0) % 360.0
        g = (goal_deg + 360.0) % 360.0
        diff = m - g
        if diff >= 180.0:
            diff -= 360.0
        if diff < -180.0:
            diff += 360.0
        return diff

    def _angular_error_deadzone_excess(self, measured_deg):
        """Error = excess beyond deadzone"""
        error = self._angle_error_deg(measured_deg, self.angle_goal)
        if error > self.deadzone:
            return error - self.deadzone
        elif error < -self.deadzone:
            return error + self.deadzone
        else:
            return 0.0

# ===============================
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
