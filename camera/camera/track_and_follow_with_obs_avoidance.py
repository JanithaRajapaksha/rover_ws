#!/usr/bin/env python3
"""Command multiplexer for track-and-follow with obstacle override.

Subscribes to:
 - /tof_distances (std_msgs/Float32MultiArray) : array of ToF readings
 - cmd_vel_tof (geometry_msgs/Twist) : velocity commands produced by ToF obstacle avoidance
 - cmd_vel_mp (geometry_msgs/Twist)  : velocity commands produced by motion planner / tracker

Publishes to:
 - cmd_vel_tracking (geometry_msgs/Twist) : the selected command forwarded to the tracker

Behavior:
 - If any ToF reading is below the threshold (default 500 mm) the node forwards
   the latest `cmd_vel_tof` commands to `cmd_vel_tracking`.
 - Otherwise it forwards the latest `cmd_vel_mp` commands.
 - If a selected source hasn't published yet the node will wait and log a warning.
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

        # parameter: obstacle threshold in millimetres
        self.declare_parameter('obstacle_threshold_mm', 500.0)
        self.declare_parameter('max_angular_z', 0.2)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)
        self.threshold = float(self.get_parameter('obstacle_threshold_mm').value)

        # publisher: forward selected Twist to the tracking controller
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_tracking', 10)

        # subscribers
        self.tof_sub = self.create_subscription(
            Float32MultiArray, '/tof_distances', self.tof_callback, 10
        )
        self.cmd_tof_sub = self.create_subscription(
            Twist, 'cmd_vel_tof', self.cmd_tof_callback, 10
        )
        self.cmd_mp_sub = self.create_subscription(
            Twist, 'cmd_vel_mp', self.cmd_mp_callback, 10
        )

        # state
        self.latest_tof_cmd = None
        self.latest_mp_cmd = None
        self.latest_tof_readings = []
        self.obstacle = False
        self.last_tof_time = 0.0
        # timeouts (seconds) before inputs are considered stale
        self.tof_timeout = 1.5  # seconds before ToF is considered stale
        # mp (motion planner) timeout: consider mp command stale after this
        self.declare_parameter('mp_timeout', 2.0)
        self.mp_timeout = float(self.get_parameter('mp_timeout').value)
        self.last_mp_time = 0.0

        # periodic check: ensure we re-publish when needed and handle stale ToF
        self.timer = self.create_timer(0.05, self._periodic_check)

        self.get_logger().info(f'cmd_mux_node started, obstacle_threshold_mm={self.threshold}')

    def tof_callback(self, msg: Float32MultiArray):
        try:
            self.latest_tof_readings = list(msg.data)
            self.last_tof_time = time.time()

            prev_obstacle = self.obstacle
            # ignore non-positive readings (sensor error) when deciding
            self.obstacle = any((d > 0 and d < self.threshold) for d in self.latest_tof_readings)

            if self.obstacle != prev_obstacle:
                state = 'OBSTACLE' if self.obstacle else 'CLEAR'
                self.get_logger().info(f'ToF state changed -> {state} (readings={self.latest_tof_readings})')

            # whenever a ToF update arrives, forward the most recent command from the
            # currently-active source so the tracking actor gets an up-to-date message.
            self._publish_selected()

        except Exception as e:
            self.get_logger().error(f'Error in tof_callback: {e}')

    def cmd_tof_callback(self, msg: Twist):
        # store latest ToF-driven command and publish it immediately when obstacle active
        self.latest_tof_cmd = msg
        if self.obstacle:
            self.cmd_pub.publish(msg)

    def cmd_mp_callback(self, msg: Twist):
        # store latest motion-planner/tracker command and publish it immediately when no obstacle
        self.latest_mp_cmd = msg
        self.last_mp_time = time.time()
        if not self.obstacle:
            self.cmd_pub.publish(msg)

    def _publish_selected(self):
        # choose which command to forward based on obstacle flag and freshness
        now = time.time()
        if (now - self.last_tof_time) > self.tof_timeout:
            # ToF is stale; treat as clear
            if self.obstacle:
                self.get_logger().warn('ToF data stale -> treating as CLEAR')
            self.obstacle = False

        if self.obstacle:
            if self.latest_tof_cmd is not None:
                self.cmd_pub.publish(self.latest_tof_cmd)
            else:
                # Safe fallback: publish zero velocities so the robot stops
                self.get_logger().warn('Obstacle detected but no `cmd_vel_tof` received yet — publishing zero Twist')
                zero = Twist()
                zero.linear.x = 0.0
                zero.angular.z = 0.0
                self.cmd_pub.publish(zero)
        else:
            mp_fresh = (self.latest_mp_cmd is not None) and ((now - self.last_mp_time) <= self.mp_timeout)
            # if mp_fresh:
            self.cmd_pub.publish(self.latest_mp_cmd)
            self.get_logger().debug('Publishing latest `cmd_vel_mp`')
            # else:
            #     # Safe fallback: stop robot
            #     self.get_logger().warn('No valid `cmd_vel_mp` — publishing zero Twist')
            #     zero = Twist()
            #     zero.linear.x = 0.0
            #     zero.angular.z = 0.2   # ✅ Stop turning
            #     self.cmd_pub.publish(zero)





    def _periodic_check(self):
        # periodically re-evaluate and re-publish last-chosen command so downstream
        # controllers see a steady stream (helps timeouts on controllers).
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



