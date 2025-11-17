#!/usr/bin/env python3
"""Command multiplexer for track-and-follow with obstacle override.

Subscribes to:
 - /tof_distances (std_msgs/Float32MultiArray) : array of ToF readings
 - cmd_vel_tof (geometry_msgs/Twist) : velocity commands from obstacle avoidance
 - cmd_vel_scaled (geometry_msgs/Twist) : velocity commands from come-to-marker or motion planner

Publishes to:
 - cmd_vel_tracking (geometry_msgs/Twist) : the selected command forwarded to the tracker

Behavior:
 - If any ToF reading is below the threshold (default 500 mm), forward `cmd_vel_tof`.
 - Otherwise, forward `cmd_vel_scaled`.
 - If a selected source hasn't published yet, the node waits and logs a warning.
 - The obstacle decision times out if no ToF message is received for a short period.
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class CmdMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_mux_node')

        # Parameters
        self.declare_parameter('obstacle_threshold_mm', 500.0)
        self.declare_parameter('max_angular_z', 0.2)
        self.declare_parameter('scaled_timeout', 2.0)
 
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_return', 10)

        # Subscribers
        self.tof_sub = self.create_subscription(Float32MultiArray, '/tof_distances', self.tof_callback, 10)
        self.cmd_tof_sub = self.create_subscription(Twist, 'cmd_vel_tof', self.cmd_tof_callback, 10)
        self.cmd_scaled_sub = self.create_subscription(Twist, 'cmd_vel_nav', self.cmd_scaled_callback, 10)

        # State
        self.latest_tof_cmd = None
        self.latest_scaled_cmd = None
        self.latest_tof_readings = []
        self.obstacle = False
        self.last_tof_time = 0.0
        self.last_scaled_time = 0.0
        self.tof_timeout = 1.5  # seconds

        self.mode_sub = self.create_subscription(
            String, '/rover_mode', self.mode_callback, 10
        )
        self.current_mode = "manual"

        # Timer for periodic checks
        self.timer = self.create_timer(0.05, self._periodic_check)

        self.get_logger().info(f'cmd_mux_node started, obstacle_threshold_mm={self.threshold}')

    # === CALLBACKS ===
    def tof_callback(self, msg: Float32MultiArray):
        try:
            self.latest_tof_readings = list(msg.data)
            self.last_tof_time = time.time()

            prev_obstacle = self.obstacle
            # Consider only valid readings (>0)
            self.obstacle = any((d > 0 and d < self.threshold) for d in self.latest_tof_readings)

            if self.obstacle != prev_obstacle:
                state = 'OBSTACLE' if self.obstacle else 'CLEAR'
                self.get_logger().info(f'ToF state changed -> {state} (readings={self.latest_tof_readings})')

            self._publish_selected()
        except Exception as e:
            self.get_logger().error(f'Error in tof_callback: {e}')

    def mode_callback(self, msg: String):
        self.current_mode = msg.data.strip().lower()
        self.get_logger().info(f"Mode updated: {self.current_mode}")

    def cmd_tof_callback(self, msg: Twist):
        self.latest_tof_cmd = msg
        if self.obstacle:
            self.cmd_pub.publish(msg)

    def cmd_scaled_callback(self, msg: Twist):
        self.latest_scaled_cmd = msg
        self.last_scaled_time = time.time()
        if not self.obstacle:
            self.cmd_pub.publish(msg)

    # === SELECTION LOGIC ===
    def _publish_selected(self):
        # only operate when in follow mode
        if self.current_mode != "return":
            # Optionally stop robot when not following
            return
        now = time.time()

        # ToF timeout handling
        if (now - self.last_tof_time) > self.tof_timeout:
            if self.obstacle:
                self.get_logger().warn('ToF data stale → treating as CLEAR')
            self.obstacle = False

        # Select appropriate command
        if self.obstacle:
            if self.latest_tof_cmd is not None:
                self.cmd_pub.publish(self.latest_tof_cmd)
            else:
                self.get_logger().warn('Obstacle detected but no `cmd_vel_tof` yet — stopping robot')
                self._publish_zero()
        else:
            scaled_fresh = (self.latest_scaled_cmd is not None) and ((now - self.last_scaled_time) <= self.scaled_timeout)
            if scaled_fresh:
                self.cmd_pub.publish(self.latest_scaled_cmd)
                self.get_logger().debug('Publishing latest `cmd_vel_scaled`')
            else:
                self.get_logger().warn('No valid `cmd_vel_scaled` — publishing zero Twist')
                self._publish_zero()

    def _publish_zero(self):
        zero = Twist()
        zero.linear.x = 0.0
        zero.angular.z = 0.0
        self.cmd_pub.publish(zero)

    def _periodic_check(self):
        try:
            self._publish_selected()
        except Exception as e:
            self.get_logger().error(f'Error in periodic check: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
