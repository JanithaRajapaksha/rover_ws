#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class UDPJoystick(Node):
    def __init__(self):
        super().__init__('udp_joystick_to_cmdvel')

        # --- UDP Setup ---
        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = 4210
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        self.sock.setblocking(False)

        # --- ROS2 Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.mode_pub = self.create_publisher(String, '/rover_mode', 10)

        # --- Timer callback (50 Hz) ---
        self.timer = self.create_timer(0.02, self.receive_data)

        # --- Speed scaling factors (default) ---
        self.speed_level = 1  # 1, 2, or 3
        self.speed_scales = {1: 0.3, 2: 0.6, 3: 1.0}
        self.angular_scale = 0.6

        # --- Mode state ---
        self.current_mode = "manual"

        self.get_logger().info(f"Listening for joystick UDP on {self.UDP_IP}:{self.UDP_PORT}")

    def receive_data(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            decoded = data.decode().strip()
            parts = [p for p in decoded.split(',') if p != '']

            if len(parts) < 6:
                self.get_logger().warn(f"Invalid packet: {decoded}")
                return

            # Parse joystick data
            x = float(parts[0])
            y = float(parts[1])
            speed_btn = int(float(parts[2]))
            cruise_btn = int(float(parts[3]))
            follow_btn = int(float(parts[4]))
            return_btn = int(float(parts[5]))

            # --- Handle speed level ---
            if speed_btn == 1:
                self.speed_level = (self.speed_level % 3) + 1  # cycle 1 → 2 → 3 → 1
                self.get_logger().info(f"Speed level changed to {self.speed_level}")
            
            linear_scale = self.speed_scales[self.speed_level]

            # --- Handle modes ---
            new_mode = self.current_mode
            if cruise_btn == 1:
                new_mode = "cruise"
            elif follow_btn == 1:
                new_mode = "follow"
            elif return_btn == 1:
                new_mode = "return"
            else:
                new_mode = "manual"

            if new_mode != self.current_mode:
                self.current_mode = new_mode
                mode_msg = String()
                mode_msg.data = self.current_mode
                self.mode_pub.publish(mode_msg)
                self.get_logger().info(f"Mode changed to: {self.current_mode}")

            # --- Publish velocity ---
            twist = Twist()
            twist.linear.x = x * linear_scale
            twist.angular.z = y * self.angular_scale
            self.cmd_vel_pub.publish(twist)

        except BlockingIOError:
            # No data yet
            pass
        except Exception as e:
            self.get_logger().error(f"Error parsing packet: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UDPJoystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down UDP joystick node...")
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
