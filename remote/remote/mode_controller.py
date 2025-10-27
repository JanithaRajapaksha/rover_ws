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
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/mode_cmd', 10)

        # --- Timer callback (50 Hz) ---
        self.timer = self.create_timer(0.02, self.receive_data)

        # --- Speed scaling factors ---
        self.linear_scale = 1.0
        self.angular_scale = 0.5

        # --- State memory ---
        self.last_buttons = [0, 0, 0, 0]  # last states of b1–b4

        self.get_logger().info(f"Listening for joystick UDP on {self.UDP_IP}:{self.UDP_PORT}")

    def receive_data(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            decoded = data.decode().strip()
            parts = decoded.split(',')

            # Expecting: x, y, main_btn, b1, b2, b3, b4
            if len(parts) < 7:
                self.get_logger().warn(f"Invalid packet: {decoded}")
                return

            # Parse values
            x = float(parts[0])
            y = float(parts[1])
            main_btn = int(float(parts[2]))
            b1, b2, b3, b4 = [int(float(v)) for v in parts[3:7]]

            # --- Handle motion ---
            twist = Twist()
            if main_btn == 1:
                twist.linear.x = x * self.linear_scale
                twist.angular.z = y * self.angular_scale
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)

            # --- Detect mode button changes ---
            new_buttons = [b1, b2, b3, b4]
            if new_buttons != self.last_buttons:
                self.handle_mode_buttons(new_buttons)
                self.last_buttons = new_buttons

        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().error(f"Error parsing packet: {e}")

    def handle_mode_buttons(self, buttons):
        """Handles button presses and publishes mode commands."""
        b1, b2, b3, b4 = buttons
        msg = String()

        if b1 == 1:
            msg.data = "cruise_control"
        elif b2 == 1:
            msg.data = "speed_mode"
        elif b3 == 1:
            msg.data = "follow_me"
        elif b4 == 1:
            msg.data = "come_back"

        else:
            # No special mode button pressed
            return

        self.mode_pub.publish(msg)
        self.get_logger().info(f"Mode command: {msg.data.upper()}")



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
