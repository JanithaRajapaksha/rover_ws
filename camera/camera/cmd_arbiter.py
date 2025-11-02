#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import time


class CmdArbiterNode(Node):
    """Arbiter that listens to cmd_vel_mp, cmd_vel_tof and /tof_distances.

    Behavior:
    - If any ToF distance < tof_threshold (mm): prefer cmd_vel_tof and log it.
    - Else: prefer cmd_vel_mp and log it.
    - If no cmd_vel_mp received within mp_timeout seconds: publish a fallback
      rotation with angular.z = fallback_rot (rad/s) and log the event.

    The selected command is forwarded to `output_topic` (default: `cmd_vel`).
    """

    def __init__(self):
        super().__init__('cmd_arbiter')

        # parameters
        self.declare_parameter('mp_timeout', 1.0)
        self.declare_parameter('tof_threshold', 500.0)  # mm
        self.declare_parameter('fallback_rot', 0.2)     # rad/s
        self.declare_parameter('output_topic', 'cmd_vel')

        self.mp_timeout = self.get_parameter('mp_timeout').get_parameter_value().double_value
        self.tof_threshold = self.get_parameter('tof_threshold').get_parameter_value().double_value
        self.fallback_rot = self.get_parameter('fallback_rot').get_parameter_value().double_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # subscriptions
        self.sub_mp = self.create_subscription(Twist, 'cmd_vel_mp', self.mp_cb, 10)
        self.sub_tof_cmd = self.create_subscription(Twist, 'cmd_vel_tof', self.tof_cmd_cb, 10)
        self.sub_tof = self.create_subscription(Float32MultiArray, '/tof_distances', self.tof_cb, 10)

        # publisher (forwarded/arbited command)
        self.pub = self.create_publisher(Twist, self.output_topic, 10)

        # state
        self.last_mp_msg = None
        self.last_mp_time = None

        self.last_tof_cmd = None
        self.last_tof_cmd_time = None

        self.last_tof_dist = []
        self.last_tof_time = None

        # timer for decision loop (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info(f'CmdArbiter started: output -> "{self.output_topic}"')

    # callbacks -----------------------------------------------------------------
    def mp_cb(self, msg: Twist):
        self.last_mp_msg = msg
        self.last_mp_time = time.time()

    def tof_cmd_cb(self, msg: Twist):
        self.last_tof_cmd = msg
        self.last_tof_cmd_time = time.time()

    def tof_cb(self, msg: Float32MultiArray):
        # store distances list (expected mm). Keep timestamp.
        try:
            self.last_tof_dist = list(msg.data)
            self.last_tof_time = time.time()
        except Exception:
            self.get_logger().warn('Received invalid /tof_distances message')

    # decision loop -------------------------------------------------------------
    def timer_cb(self):
        now = time.time()

        # check for close obstacle
        obstacle_close = False
        if self.last_tof_dist:
            # ignore zero or negative readings
            for d in self.last_tof_dist:
                try:
                    if d > 0 and d < self.tof_threshold:
                        obstacle_close = True
                        break
                except Exception:
                    continue

        # if obstacle close, prefer cmd_vel_tof
        if obstacle_close:
            if self.last_tof_cmd is not None:
                log_msg = (
                    f"Obstacle detected (<{self.tof_threshold}mm). Using cmd_vel_tof -> "
                    f"linear.x={self.last_tof_cmd.linear.x:.3f}, angular.z={self.last_tof_cmd.angular.z:.3f}"
                )
                self.get_logger().info(log_msg)
                # forward
                self.pub.publish(self.last_tof_cmd)
            else:
                self.get_logger().warn('Obstacle detected but no cmd_vel_tof received yet')
        else:
            # no obstacle: prefer cmd_vel_mp
            if self.last_mp_time is not None and (now - self.last_mp_time) <= self.mp_timeout:
                if self.last_mp_msg is not None:
                    self.get_logger().info(
                        f'No close obstacle. Using cmd_vel_mp -> '
                        f'linear.x={self.last_mp_msg.linear.x:.3f}, angular.z={self.last_mp_msg.angular.z:.3f}'
                    )
                    self.pub.publish(self.last_mp_msg)
                else:
                    # unlikely: time recorded but no msg
                    self.get_logger().warn('mp timestamp present but message missing')
            else:
                # mp commands stale or never received -> fallback rotation
                twist = Twist()
                twist.angular.z = float(self.fallback_rot)
                twist.linear.x = 0.0
                self.pub.publish(twist)
                self.get_logger().warn(
                    f'No recent cmd_vel_mp within {self.mp_timeout}s -> publishing fallback rotation {self.fallback_rot} rad/s'
                )


def main(args=None):
    rclpy.init(args=args)
    node = CmdArbiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('CmdArbiter stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
