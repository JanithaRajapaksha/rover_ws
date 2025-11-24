#!/usr/bin/env python3
"""Command multiplexer for track-and-follow with obstacle override.

Subscribes to:
 - /tof_distances (std_msgs/Float32MultiArray) : array of ToF readings
 - cmd_vel_tof (geometry_msgs/Twist) : velocity commands from obstacle avoidance
 - cmd_vel_nav (geometry_msgs/Twist) : velocity commands from come-to-marker or motion planner

Publishes to:
 - cmd_vel_tracking (geometry_msgs/Twist) : the selected command forwarded to the tracker

Behavior:
 - If any ToF reading is below the threshold (default 500 mm), forward `cmd_vel_tof`.
 - Otherwise, forward `cmd_vel_nav`.
 - If a selected source hasn't published yet, the node waits and logs a warning.
 - The obstacle decision times out if no ToF message is received for a short period.
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String, Float32


class CmdMuxNode(Node):
    def __init__(self):
        super().__init__('come_to_me_mux')

        # Parameters
        self.declare_parameter('obstacle_threshold_mm', 500.0)
        self.declare_parameter('max_angular_z', 0.2)
        self.declare_parameter('nav_timeout', 2.0)
        self.declare_parameter('camera_timeout', 2.0)
        self.declare_parameter('uwb_stop_distance', 1.5)
 
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_return', 10)

        # Subscribers
        self.tof_sub = self.create_subscription(Float32MultiArray, '/tof_distances', self.tof_callback, 10)
        self.cmd_tof_sub = self.create_subscription(Twist, 'cmd_vel_tof', self.cmd_tof_callback, 10)
        self.cmd_nav_sub = self.create_subscription(Twist, 'cmd_vel_nav', self.cmd_nav_callback, 10)
        # Camera velocity commands (prefer these over nav when fresh, but after ToF)
        self.cmd_camera_sub = self.create_subscription(Twist, 'cmd_vel_camera', self.cmd_camera_callback, 10)
        # Subscribe to the state machine status so we ignore ToF until state machine DONE
        self.status_sub = self.create_subscription(String, '/state_machine_status', self.state_status_callback, 10)
        # Subscribe to UWB distance to stop when close
        self.uwb_sub = self.create_subscription(Float32, '/uwb_distance', self.uwb_callback, 10)

        # State
        self.latest_tof_cmd = None
        self.latest_nav_cmd = None
        self.latest_camera_cmd = None
        self.latest_tof_readings = []
        self.latest_uwb_distance = None
        self.obstacle = False
        self.last_tof_time = 0.0
        self.last_nav_time = 0.0
        self.last_camera_time = 0.0
        self.tof_timeout = 1.5  # seconds
        # Only start considering ToF readings after state machine publishes DONE
        self.state_machine_done = False
        # Avoid spamming logs while ToF is being ignored
        self._tof_ignored_warned = False
        # Avoid spamming logs while camera cmds are ignored
        self._camera_ignored_warned = False

        # Read parameter values into attributes so they exist when referenced
        try:
            self.threshold = float(self.get_parameter('obstacle_threshold_mm').value)
        except Exception:
            # fallback default
            self.threshold = 500.0

        try:
            self.max_angular_z = float(self.get_parameter('max_angular_z').value)
        except Exception:
            self.max_angular_z = 0.2

        try:
            self.nav_timeout = float(self.get_parameter('nav_timeout').value)
        except Exception:
            self.nav_timeout = 2.0

        try:
            self.camera_timeout = float(self.get_parameter('camera_timeout').value)
        except Exception:
            self.camera_timeout = 2.0

        try:
            self.uwb_stop_distance = float(self.get_parameter('uwb_stop_distance').value)
        except Exception:
            self.uwb_stop_distance = 1.5

        # Flag when UWB indicates we're close enough to stop
        self.uwb_close = False
        self._uwb_warned = False

        # self.mode_sub = self.create_subscription(
        #     String, '/rover_mode', self.mode_callback, 10
        # )
        # self.current_mode = "manual"

        # Timer for periodic checks
        self.timer = self.create_timer(0.05, self._periodic_check)

        self.get_logger().info(f'cmd_mux_node started, obstacle_threshold_mm={self.threshold}')

    # === CALLBACKS ===
    def tof_callback(self, msg: Float32MultiArray):
        # Ignore ToF readings until the external state machine signals DONE
        if not self.state_machine_done:
            if not self._tof_ignored_warned:
                self.get_logger().info('Ignoring ToF readings until state machine status is DONE')
                self._tof_ignored_warned = True
            return

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

    def uwb_callback(self, msg: Float32):
        try:
            self.latest_uwb_distance = float(msg.data)
            # set close flag
            prev_close = self.uwb_close
            if self.latest_uwb_distance is not None and self.latest_uwb_distance > 0 and self.latest_uwb_distance < self.uwb_stop_distance:
                self.uwb_close = True
            else:
                self.uwb_close = False

            if self.uwb_close and not prev_close:
                self.get_logger().info(f'UWB distance {self.latest_uwb_distance:.2f} m < {self.uwb_stop_distance} m → stopping robot')
            elif not self.uwb_close and prev_close:
                self.get_logger().info(f'UWB distance {self.latest_uwb_distance:.2f} m ≥ {self.uwb_stop_distance} m → resuming')

            # enforce stop immediately if close
            if self.uwb_close:
                self._publish_zero()
            else:
                # clear warned flag so next stop logs once
                self._uwb_warned = False
        except Exception as e:
            self.get_logger().error(f'Error in uwb_callback: {e}')

    def state_status_callback(self, msg: String):
        try:
            val = msg.data.strip()
            if val == 'DONE' and not self.state_machine_done:
                self.state_machine_done = True
                # clear suppressed-warn flags so next ignored log shows once if needed
                self._tof_ignored_warned = False
                self._camera_ignored_warned = False
                self.get_logger().info('State machine DONE received — enabling ToF and camera processing')
            # Optionally support resetting behavior if state machine restarts
            elif val != 'DONE' and self.state_machine_done:
                self.state_machine_done = False
                # reset suppressed-warn flags so user sees informative logs again
                self._tof_ignored_warned = False
                self._camera_ignored_warned = False
                self.get_logger().info('State machine left DONE — disabling ToF and camera processing')
        except Exception as e:
            self.get_logger().error(f'Error in state_status_callback: {e}')

    # def mode_callback(self, msg: String):
    #     self.current_mode = msg.data.strip().lower()
    #     self.get_logger().info(f"Mode updated: {self.current_mode}")

    def cmd_tof_callback(self, msg: Twist):
        self.latest_tof_cmd = msg
        # If UWB indicates we're too close, enforce stop instead of forwarding
        if self.uwb_close:
            if not self._uwb_warned:
                self.get_logger().info(f'UWB close ({self.latest_uwb_distance:.2f} m) — suppressing ToF cmd and stopping')
                self._uwb_warned = True
            self._publish_zero()
            return
        if self.obstacle:
            self.cmd_pub.publish(msg)

    def cmd_camera_callback(self, msg: Twist):
        # Ignore camera commands until state machine DONE
        if not self.state_machine_done:
            if not self._camera_ignored_warned:
                self.get_logger().info('Ignoring camera cmd_vel until state machine status is DONE')
                self._camera_ignored_warned = True
            return

        # store latest camera cmd and timestamp
        self.latest_camera_cmd = msg
        self.last_camera_time = time.time()
        # If UWB is close, suppress camera commands and stop
        if self.uwb_close:
            if not self._uwb_warned:
                self.get_logger().info(f'UWB close ({self.latest_uwb_distance:.2f} m) — suppressing camera cmd and stopping')
                self._uwb_warned = True
            self._publish_zero()
            return
        # if no obstacle currently, allow immediate pass-through (to reduce latency)
        if not self.obstacle:
            self.cmd_pub.publish(msg)

    def cmd_nav_callback(self, msg: Twist):
        self.latest_nav_cmd = msg
        self.last_nav_time = time.time()
        # If UWB is close, suppress nav commands and stop
        if self.uwb_close:
            if not self._uwb_warned:
                self.get_logger().info(f'UWB close ({self.latest_uwb_distance:.2f} m) — suppressing nav cmd and stopping')
                self._uwb_warned = True
            self._publish_zero()
            return

        if not self.obstacle:
            self.cmd_pub.publish(msg)

    # === SELECTION LOGIC ===
    def _publish_selected(self):
        # only operate when in follow mode
        # if self.current_mode != "return":
        #     # Optionally stop robot when not following
        #     return
        now = time.time()

        # UWB close check: if UWB reports close, always publish zero and return
        if self.uwb_close:
            if not self._uwb_warned:
                try:
                    dist = self.latest_uwb_distance if self.latest_uwb_distance is not None else float('nan')
                except Exception:
                    dist = float('nan')
                self.get_logger().info(f'UWB close ({dist:.2f} m) — overriding and publishing zero')
                self._uwb_warned = True
            self._publish_zero()
            return

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
            # Priority: camera (if fresh) -> nav (if fresh) -> zero
            camera_fresh = self.state_machine_done and (self.latest_camera_cmd is not None) and ((now - self.last_camera_time) <= self.camera_timeout)
            nav_fresh = (self.latest_nav_cmd is not None) and ((now - self.last_nav_time) <= self.nav_timeout)

            if camera_fresh:
                self.cmd_pub.publish(self.latest_camera_cmd)
                self.get_logger().debug('Publishing latest `cmd_vel_camera`')
            elif nav_fresh:
                self.cmd_pub.publish(self.latest_nav_cmd)
                self.get_logger().debug('Publishing latest `cmd_vel_nav`')
            else:
                self.get_logger().warn('No valid `cmd_vel_nav` — publishing zero Twist')
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
    