#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import math
import time

import serial
import time
import math

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
    
    def __init__(self, uart_port='/dev/ttyAMA0', baudrate=256000, multi_mode=True):
        self.uart = serial.Serial(uart_port, baudrate, timeout=0.1)
        self.targets = []  # Stores up to 3 targets
        self.buffer = b''  # Buffer to handle split messages
        time.sleep(0.2)
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
# ======================
#  Simple Kalman Filter
# ======================
class KalmanFilter:
    def __init__(self, process_variance=1e-3, measurement_variance=1e-1):
        self.x = 0.0      # estimated angle
        self.P = 1.0      # estimation covariance
        self.Q = process_variance
        self.R = measurement_variance

    def update(self, measurement):
        # Prediction step
        self.P += self.Q

        # Update step
        K = self.P / (self.P + self.R)
        self.x += K * (measurement - self.x)
        self.P *= (1 - K)

        return self.x


# ======================
#        PID Class
# ======================
class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def compute(self, measurement):
        now = time.time()
        dt = now - self.last_time if self.last_time else 0.01
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        self.last_time = now
        return output


# ======================
#   Main Tracking Node
# ======================
class RD03DAngularTracker(Node):
    def __init__(self):
        super().__init__('rd03d_angular_tracker')

        # --- Parameters ---
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 256000)
        self.declare_parameter('target_id', 1)
        self.declare_parameter('angle_setpoint', 0.0)
        self.declare_parameter('kp_ang', 0.01)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.001)

        # --- Radar setup ---
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.radar = RD03D(port, baudrate, multi_mode=True)
        self.target_id = self.get_parameter('target_id').value

        # --- Kalman filter for angle smoothing ---
        self.kalman = KalmanFilter(process_variance=1e-4, measurement_variance=0.5)

        # --- Angular PID controller ---
        self.pid_angle = PID(
            self.get_parameter('kp_ang').value,
            self.get_parameter('ki_ang').value,
            self.get_parameter('kd_ang').value,
            setpoint=self.get_parameter('angle_setpoint').value
        )

        # --- Publisher ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_tracking', 10)

        # --- Timer ---
        self.timer = self.create_timer(0.1, self.loop)  # 10 Hz

        self.get_logger().info("✅ RD03D Angular Tracker with Kalman Filter Started!")

    def loop(self):
        if not self.radar.update():
            return

        target = self.radar.get_target(self.target_id)
        if target is None:
            return

        raw_angle = target.angle
        filtered_angle = self.kalman.update(raw_angle)

        # --- Compute angular correction ---
        angular_z = self.pid_angle.compute(filtered_angle)
        angular_z = max(min(angular_z, 0.2), -0.2)

        twist = Twist()
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f"Target {self.target_id}: raw_angle={raw_angle:.2f}°, "
            f"filtered={filtered_angle:.2f}°, cmd_ang={angular_z:.3f}"
        )

    def destroy_node(self):
        try:
            self.radar.close()
        except Exception as e:
            self.get_logger().warn(f"Error closing radar: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RD03DAngularTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()