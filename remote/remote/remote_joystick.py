#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


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

        # --- ROS2 Publisher ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Timer callback (50 Hz) ---
        self.timer = self.create_timer(0.02, self.receive_data)

        # --- Speed scaling factors (tune these) ---
        self.linear_scale = 1.0     # m/s
        self.angular_scale = 0.5    # rad/s

        self.get_logger().info(f"Listening for joystick UDP on {self.UDP_IP}:{self.UDP_PORT}")

    def receive_data(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            decoded = data.decode().strip()
            parts = decoded.split(',')

            if len(parts) >= 3:
                x = float(parts[0])
                y = float(parts[1])
                button = int(float(parts[2]))  # handle both "1" and "1.0"

                twist = Twist()

                # Example mapping:
                # x-axis → linear.x (forward/backward)
                # y-axis → angular.z (turn left/right)
                if button == 1:  # Enable motion only when button is pressed
                    twist.linear.x = x * self.linear_scale
                    twist.angular.z = y * self.angular_scale
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.cmd_vel_pub.publish(twist)

                self.get_logger().info(
                    f"cmd_vel: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}, button={button}"
                )

            else:
                self.get_logger().warn(f"Invalid packet: {decoded}")

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
