#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import subprocess
import time


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
        self.timer = self.create_timer(0.1, self.receive_data)

        # --- Speed scaling factors (default) ---
        self.speed_level = 1  # 1, 2, 3, or 4
        self.speed_scales = {1: 0.25, 2: 0.5, 3: 0.75, 4: 1.0}
        self.angular_scale = 1.0

        # --- Mode state ---
        self.current_mode = "manual"
        # UDP reset target for follow mode
        self.reset_host = "127.0.0.1"
        self.reset_port = 5007

        self.prev_speed_btn = 0
        self.prev_return_btn = 0

        self.get_logger().info(f"Listening for joystick UDP on {self.UDP_IP}:{self.UDP_PORT}")

    def send_reset_udp(self, host: str, port: int):
        """Send a UDP datagram with the ASCII payload 'reset' to (host, port)."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.sendto(b"reset", (host, port))
            self.get_logger().info(f"Sent UDP reset to {host}:{port}")
        except Exception as e:
            self.get_logger().error(f"send_reset_udp error: {e}")

    def receive_data(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            decoded = data.decode().strip()
            parts = [p for p in decoded.split(',') if p != '']

            if len(parts) < 6:
                self.get_logger().warn(f"Invalid packet: {decoded}")
                return

            # Parse joystick data
            x = float(parts[1])
            y = float(parts[0])
            speed_btn = int(float(parts[2]))
            cruise_btn = int(float(parts[3]))
            follow_btn = int(float(parts[5]))
            return_btn = int(float(parts[4]))

            # --- Handle speed level ---
            if speed_btn == 1 and self.prev_speed_btn == 0:
                self.speed_level = (self.speed_level % 3) + 1
                self.get_logger().info(f"Speed level changed to {self.speed_level}")
            self.prev_speed_btn = speed_btn
            # --- Handle return button edge (press) ---
            if return_btn == 1 and self.prev_return_btn == 0:
                # Trigger restart of main launch nodes and direction_tester
                try:
                    self.get_logger().info("Return pressed: restarting main launch nodes and direction_tester")
                    self._restart_main_launch_nodes()
                    self._restart_direction_tester()
                except Exception as e:
                    self.get_logger().error(f"Error restarting nodes: {e}")
            self.prev_return_btn = return_btn

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
                # If switching to follow mode, send a UDP "reset" message
                if new_mode == "follow":
                    try:
                        self.send_reset_udp(self.reset_host, self.reset_port)
                    except Exception as e:
                        self.get_logger().error(f"Failed sending reset UDP: {e}")
                    # small pause to allow the camera/reset listener to process
                    try:
                        time.sleep(0.1)
                    except Exception:
                        pass

                    # Launch the camera-based track-and-follow stack (non-blocking)
                    try:
                        self.get_logger().info("Launching camera/track_and_follow_all.launch.py for follow mode")
                        self._run_cmd(["ros2", "launch", "camera", "track_and_follow_all.launch.py"])
                    except Exception as e:
                        self.get_logger().error(f"Failed launching track_and_follow_all: {e}")

                self.current_mode = new_mode
                mode_msg = String()
                mode_msg.data = self.current_mode
                self.mode_pub.publish(mode_msg)
                self.get_logger().info(f"Mode changed to: {self.current_mode}")

            # --- Publish velocity only in manual or cruise mode ---
            if self.current_mode in ["manual", "cruise"]:
                twist = Twist()
                twist.linear.x = x * linear_scale
                twist.angular.z = y * -self.angular_scale
                self.cmd_vel_pub.publish(twist)
        except BlockingIOError:
            # No data yet
            pass
        except Exception as e:
            self.get_logger().error(f"Error parsing packet: {e}")
    def _run_cmd(self, cmd_list):
        """Run a command as subprocess without blocking the main thread and swallow output."""
        try:
            subprocess.Popen(cmd_list, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().error(f"Failed to run {' '.join(cmd_list)}: {e}")

    def _pkill_patterns(self, patterns):
        for p in patterns:
            try:
                subprocess.run(["pkill", "-f", p], check=False)
                self.get_logger().info(f"pkill -f {p} executed")
            except Exception as e:
                self.get_logger().error(f"pkill for {p} failed: {e}")

    def _restart_main_launch_nodes(self):
        """Kill a set of main launch-related processes and relaunch the main robot launch."""
        # Patterns taken from existing restart script
        patterns = [
            "twist_mux",
            "controller_manager",
            "robot_state_publisher",
        ]
        # Kill processes
        self._pkill_patterns(patterns)
        # small pause to allow processes to terminate
        time.sleep(0.5)

        # Relaunch main robot launch (non-blocking)
        try:
            self.get_logger().info("Relaunching main robot launch (rover1 launch_robot.launch.py)")
            self._run_cmd(["ros2", "launch", "rover1", "launch_robot.launch.py"])
        except Exception as e:
            self.get_logger().error(f"Failed to relaunch main launch: {e}")

    def _restart_direction_tester(self):
        """Kill direction_tester and come_to_me_mux, then restart direction_tester."""
        
        # ---- Kill both processes ----
        processes_to_kill = [
            "direction_tester",
            "come_to_me_mux",
            "pose_pub",
            "vision_pid_controller"
        ]

        for proc in processes_to_kill:
            try:
                subprocess.run(["pkill", "-f", proc], check=False)
                self.get_logger().info(f"pkill -f {proc} executed")
            except Exception as e:
                self.get_logger().error(f"Failed to pkill {proc}: {e}")

        # Small delay
        time.sleep(0.3)

        # Additionally ensure any person-tracking / follow nodes are stopped when
        # entering come-to-me mode (these are launched by camera/track_and_follow_all.launch.py)
        tracking_patterns = [
            "track_and_follow.py",
            # "obstacle_avoidance.py",
            "obstacle_avoidance_camera.py",
            "track_and_follow_with_obs_avoidance.py",
            # also try matching node/process name fragments
            "person_tracker_pid_node",
            # "tof_pid_node",
            # "camera_publisher_node",
            "vision_pid_controller",
            "cmd_mux_node_follow",
        ]

        for p in tracking_patterns:
            try:
                subprocess.run(["pkill", "-f", p], check=False)
                self.get_logger().info(f"pkill -f {p} executed")
            except Exception as e:
                self.get_logger().error(f"Failed to pkill tracking pattern {p}: {e}")

        # # ---- Restart nodes ----
        # try:
        #     self.get_logger().info("Relaunching pose_pub...")
        #     self._run_cmd(["ros2", "run", "uwb", "pose_pub.py"])
        # except Exception as e:
        #     self.get_logger().error(f"Failed to relaunch direction_tester: {e}")

        # try:
        #     self.get_logger().info("Relaunching come_to_me_mux...")
        #     self._run_cmd(["ros2", "run", "uwb", "come_to_me_mux.py"])
        # except Exception as e:
        #     self.get_logger().error(f"Failed to relaunch come_to_me_mux: {e}")

        try:
            self.get_logger().info("Relaunching direction_tester...")
            self._run_cmd(["ros2", "launch", "uwb", "come_to_me_all.launch.py"])
        except Exception as e:
            self.get_logger().error(f"Failed to relaunch come_to_me: {e}")

        # try:
        #     self.get_logger().info("Relaunching come_to_me_mux...")
        #     self._run_cmd(["ros2", "run", "uwb", "come_to_me_mux.py"])
        # except Exception as e:
        #     self.get_logger().error(f"Failed to relaunch come_to_me_mux: {e}")
       


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
