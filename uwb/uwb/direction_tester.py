#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32, String
import math
import time


class CircleMovement(Node):
    def __init__(self):
        super().__init__('circle_movement')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.marker_pub = self.create_publisher(Marker, '/target_marker', 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to UWB distance
        self.subscription = self.create_subscription(
            Float32,
            '/uwb_distance',
            self.uwb_callback,
            10
        )
        self.subscription
        self.uwb_distance = None
        self.min_uwb_distance = float('inf')

        # State machine
        self.state = 'WAIT_BEFORE_FORWARD'
        self.wait_start_time = time.time()

        # Orientation tracking
        self.start_yaw = None
        self.current_yaw = None
        self.target_yaw = None

        # Position tracking
        self.start_x = None
        self.start_y = None

        self.prev_angle_error = 0.0
        self.integral_error = 0.0

        # Publish an initial marker at origin (0,0,0)
        initial_marker = Marker()
        initial_marker.header.frame_id = "base_link"
        initial_marker.header.stamp = self.get_clock().now().to_msg()
        initial_marker.ns = "target_marker"
        initial_marker.id = 0
        initial_marker.type = Marker.SPHERE
        initial_marker.action = Marker.ADD
        initial_marker.pose.position = Point(x=0.0, y=0.0, z=0.0)
        initial_marker.pose.orientation.w = 1.0
        initial_marker.scale.x = 0.25
        initial_marker.scale.y = 0.25
        initial_marker.scale.z = 0.25
        initial_marker.color.a = 1.0
        initial_marker.color.r = 0.0
        initial_marker.color.g = 1.0
        initial_marker.color.b = 0.0
        self.marker_pub.publish(initial_marker)
        self.get_logger().info("📍 Initial marker published at (0,0,0)")

        self.last_marker_distance = 1.0  # default
        self.marker_published = False
        self.marker_timer = self.create_timer(0.1, self.keep_publishing_marker)

        self.marker_world_x = 0.0
        self.marker_world_y = 0.0
        self.marker_published = False
        self.marker_timer = self.create_timer(0.1, self.keep_publishing_marker)
        # Publisher to notify other nodes when state machine is DONE
        self.status_pub = self.create_publisher(String, '/state_machine_status', 10)
        self.done_msg_published = False
        self.done_state_time = None

        self.timer = self.create_timer(0.05, self.timer_callback)

    def keep_publishing_marker(self):
        """Continuously republish the marker so it stays visible in RViz."""
        if self.marker_published:
            marker = Marker()
            marker.header.frame_id = "odom"  # FIXED in world frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "target_marker"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position = Point(x=self.marker_world_x, y=self.marker_world_y, z=0.0)
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            self.marker_pub.publish(marker)

    def publish_done(self):
        """Schedule a one-shot DONE message to be published after 3s of stable DONE state.

        This method does not immediately publish — it records the time when the node
        first entered DONE and the periodic `timer_callback` will publish the message
        only if the state remains `DONE` for at least 3 seconds. If the state leaves
        `DONE` before 3s the scheduled publish is cancelled.
        """
        try:
            # If already published, nothing to do
            if getattr(self, 'done_msg_published', False):
                return

            # Record the time when DONE was first requested/scheduled
            if getattr(self, 'done_state_time', None) is None:
                self.done_state_time = time.time()
                self.get_logger().info('Scheduled state_machine_status DONE to be published after 3s of stable DONE')
        except Exception as e:
            self.get_logger().error(f"Failed to schedule DONE message: {e}")

    def get_heading_error_to_marker(self):
        """
        Returns angular error (radians) between robot's heading and marker direction.
        Positive = marker is to left, Negative = marker to right.
        """
        if not self.marker_published:
            return None

        x_r, y_r, yaw_r = self.get_pose()
        if x_r is None:
            return None

        dx = self.marker_world_x - x_r
        dy = self.marker_world_y - y_r
        desired_yaw = math.atan2(dy, dx)
        return self.normalize_angle(desired_yaw - yaw_r)



    def pid_control(self, error, dt, Kp=0.015, Ki=0.0, Kd=0.002):
        self.integral_error += error * dt
        derivative = (error - self.prev_angle_error) / dt if dt > 0 else 0.0
        self.prev_angle_error = error
        output = Kp * error + Ki * self.integral_error + Kd * derivative
        return output

    def publish_target_marker(self, distance):
        # Get current robot pose in odom frame
        x_r, y_r, yaw_r = self.get_pose()
        if x_r is None:
            self.get_logger().warn("Cannot publish marker: robot pose not available")
            return

        # Compute marker world coordinates
        self.marker_world_x = x_r + distance * math.cos(yaw_r)
        self.marker_world_y = y_r + distance * math.sin(yaw_r)

        # Publish marker once
        marker = Marker()
        marker.header.frame_id = "odom"  # FIXED in world frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = Point(x=self.marker_world_x, y=self.marker_world_y, z=0.0)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)
        self.get_logger().info(f"📍 Marker published at world position ({self.marker_world_x:.2f}, {self.marker_world_y:.2f})")

        self.marker_published = True


    def check_relative_heading(self):
        try:
            # Current robot pose
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            x_r = trans.transform.translation.x
            y_r = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, yaw_r = euler_from_quaternion([q.x, q.y, q.z, q.w])

            # Marker world coordinates (in odom frame) based on last recorded pose and yaw
            x_m = x_r + self.target_distance * math.cos(yaw_r)
            y_m = y_r + self.target_distance * math.sin(yaw_r)

            # Re-get robot pose to compute heading difference
            trans2 = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            x_r2 = trans2.transform.translation.x
            y_r2 = trans2.transform.translation.y
            q2 = trans2.transform.rotation
            _, _, yaw_r2 = euler_from_quaternion([q2.x, q2.y, q2.z, q2.w])

            # Compute vector from robot → marker
            dx = x_m - x_r2
            dy = y_m - y_r2
            marker_angle = math.atan2(dy, dx)

            # Relative heading difference
            heading_diff = self.normalize_angle(marker_angle - yaw_r2)
            self.get_logger().info(f"🧭 Relative heading to marker: {math.degrees(heading_diff):.2f}°")

        except Exception as e:
            self.get_logger().warn(f"Failed to compute relative heading: {e}")



    # --------------------------
    #  UWB Subscriber Callback
    # --------------------------
    def uwb_callback(self, msg):
        self.uwb_distance = msg.data
        # Optional: throttle log
        # self.get_logger().info_throttle(1.0, f"UWB Distance: {self.uwb_distance:.3f} m")

    # --------------------------
    #  TF Pose Reader
    # --------------------------
    def get_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return x, y, yaw
        except Exception as e:
            now = time.time()
            if not hasattr(self, 'last_tf_warn_time'):
                self.last_tf_warn_time = 0.0
            if now - self.last_tf_warn_time > 5.0:
                self.get_logger().warn(f"TF not available: {e}")
                self.last_tf_warn_time = now
            return None, None, None


    # --------------------------
    #  Main Control Loop
    # --------------------------
    def timer_callback(self):
        x, y, yaw = self.get_pose()
        if yaw is None:
            return

        # If a DONE publish was scheduled, check whether 3s of stable DONE elapsed
        if getattr(self, 'done_state_time', None) is not None and not getattr(self, 'done_msg_published', False):
            # Only publish if we are still in the DONE state
            if self.state == 'DONE':
                if time.time() - self.done_state_time >= 3.0:
                    try:
                        msg = String()
                        msg.data = 'DONE'
                        self.status_pub.publish(msg)
                        self.done_msg_published = True
                        self.get_logger().info('🔔 Published state_machine_status: DONE (after 3s stable)')
                    except Exception as e:
                        self.get_logger().error(f'Failed to publish scheduled DONE message: {e}')
            else:
                # State left DONE before timeout; cancel scheduled publish
                self.get_logger().info('State left DONE before 3s — cancelling scheduled DONE publish')
                self.done_state_time = None

        # --- Manual yaw logging every 0.1s ---
        now = time.time()
        if not hasattr(self, "last_log_time"):
            self.last_log_time = 0
        if now - self.last_log_time > 0.1:
            self.get_logger().info(f"Current yaw: {math.degrees(yaw):.2f}°")
            self.last_log_time = now

        twist = Twist()

        if self.start_x is None:
            self.start_x, self.start_y, self.start_yaw = x, y, yaw

        # ---- STATE MACHINE ----
        if self.state == 'WAIT_BEFORE_FORWARD':
            if time.time() - self.wait_start_time < 5.0:
                self.cmd_pub.publish(Twist())  # stay still
                return
            else:
                self.start_time = time.time()
                self.state = 'MOVE_FORWARD'
                self.get_logger().info("Starting to move forward")

        elif self.state == 'MOVE_FORWARD':
            twist.linear.x = 0.15
            self.cmd_pub.publish(twist)

            if time.time() - self.start_time > 2.0:
                self.cmd_pub.publish(Twist())
                self.state = 'TURN_90'
                self.target_yaw = self.normalize_angle(self.start_yaw + math.pi / 2)
                self.get_logger().info(f"Turning +90°, target yaw = {math.degrees(self.target_yaw):.2f}°")

        elif self.state == 'TURN_90':
            angle_error = self.normalize_angle(self.target_yaw - yaw)
            if abs(angle_error) > math.radians(3):
                twist.angular.z = 0.3 * (1 if angle_error > 0 else -1)
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())
                self.circle_start_yaw = yaw
                self.get_logger().info(f"Circle start yaw: {math.degrees(yaw):.2f}°")
                self.min_uwb_distance = float('inf')  # reset before circle
                self.state = 'MAKE_CIRCLE'
                self.circle_start_time = time.time()

        elif self.state == 'MAKE_CIRCLE':
            twist.linear.x = 0.1
            twist.angular.z = 0.11
            self.cmd_pub.publish(twist)

            yaw_diff = self.normalize_angle(yaw - self.circle_start_yaw)

            # Update minimum UWB distance while moving
            if self.uwb_distance is not None:
                if self.uwb_distance < self.min_uwb_distance:
                    self.min_uwb_distance = self.uwb_distance
                    self.min_uwb_yaw = yaw  # store yaw of min distance
                    self.get_logger().info(
                        f"📉 New closest UWB distance: {self.min_uwb_distance:.3f} m at yaw {math.degrees(yaw):.2f}°"
                    )


            # Target yaw for correction (slight overshoot)
            corrected_target_yaw = self.normalize_angle(self.circle_start_yaw + math.radians(20))
            angle_error = self.normalize_angle(corrected_target_yaw - yaw)

            # Detect circle completion
            if abs(angle_error) < math.radians(3) and (time.time() - self.circle_start_time) > 1.0:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(
                    f"Circle + correction done. Min UWB distance: {self.min_uwb_distance:.3f} m"
                )
                self.state = 'TURN_BACK'
                self.target_yaw = self.normalize_angle(yaw - math.pi / 2)
                self.get_logger().info(f"Turning -90° to return, target yaw = {math.degrees(self.target_yaw):.2f}°")

        elif self.state == 'TURN_BACK':
            angle_error = self.normalize_angle(self.target_yaw - yaw)
            if abs(angle_error) > math.radians(3):
                twist.angular.z = 0.3 * (1 if angle_error > 0 else -1)
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f"Returned to original orientation: {math.degrees(yaw):.2f}°")
                self.state = 'MOVE_BACKWARD'
                self.start_back_time = time.time()

        elif self.state == 'MOVE_BACKWARD':
            twist.linear.x = -0.15
            self.cmd_pub.publish(twist)
            if time.time() - self.start_back_time > 2.0:
                self.cmd_pub.publish(Twist())

                if hasattr(self, "min_uwb_yaw"):
                    self.state = 'TURN_TO_PERP'
                    # +90 degrees from min UWB direction
                    self.target_yaw = self.normalize_angle(self.min_uwb_yaw + math.pi / 2)
                    self.get_logger().info(
                        f"🧭 Turning to perpendicular direction (min UWB yaw + 90°): "
                        f"{math.degrees(self.target_yaw):.2f}°"
                    )
                else:
                    self.state = 'DONE'
                    self.get_logger().info("No UWB data recorded. Stopping ✅")
                    self.publish_done()

        elif self.state == 'TURN_TO_PERP':
            # --- Compute Error ---
            angle_error = self.normalize_angle(self.target_yaw - yaw)
            angle_deg = math.degrees(angle_error)

            # --- Proportional Gain ---
            Kp = 1.0
            pid_output = Kp * angle_error

            # --- Apply angular velocity ---
            twist.angular.z = -pid_output
            twist.angular.z = max(min(twist.angular.z, 0.4), -0.4)

            # --- Dead zone ---
            dead_zone = math.radians(5)

            # --- Track oscillations ---
            prev_sign = getattr(self, "prev_turn_sign", 0)
            current_sign = 1 if twist.angular.z > 0 else -1

            # Initialize history
            if not hasattr(self, "yaw_history"):
                self.yaw_history = []
                self.prev_turn_sign = current_sign
                self.oscillation_detected_time = None

            # Detect sign flip
            if prev_sign != current_sign:
                self.yaw_history.append(yaw)
                if len(self.yaw_history) > 2:
                    self.yaw_history.pop(0)

                # Record detection time of oscillation
                if len(self.yaw_history) == 2 and self.oscillation_detected_time is None:
                    self.oscillation_detected_time = (
                        self.get_clock().now().seconds_nanoseconds()[0]
                        + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
                    )
                    self.get_logger().info("Oscillation detected, waiting 1 sec before stopping...")

            # --- Wait for 1 second after oscillation detection ---
            if getattr(self, "oscillation_detected_time", None) is not None:
                current_time = (
                    self.get_clock().now().seconds_nanoseconds()[0]
                    + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
                )
                if current_time - self.oscillation_detected_time >= 1.0:
                    # Stop at midpoint yaw
                    mid_yaw = (self.yaw_history[0] + self.yaw_history[1]) / 2.0
                    self.target_yaw = mid_yaw
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info(
                        f"Stopping after 1s. Midpoint yaw: {math.degrees(mid_yaw):.2f}°"
                    )
                    self.state = "TURN_OFFSET"
                    return

            # --- Normal turning ---
            if abs(angle_error) > dead_zone:
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())
                self.state = "TURN_OFFSET"
                self.get_logger().info(f"Reached perpendicular (final yaw: {math.degrees(yaw):.2f}°)")

            self.prev_turn_sign = current_sign

        elif self.state == 'TURN_OFFSET':
            angle_error = self.normalize_angle(self.target_yaw - yaw)
            if abs(angle_error) > math.radians(3):
                twist.angular.z = 0.15 * (1 if angle_error > 0 else -1)
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())
                self.get_logger().info("✅ Offset correction done.")

                # --- Publish marker ahead of robot (only once) ---
                if not self.marker_published:
                    if self.uwb_distance is not None:
                        self.publish_target_marker(self.uwb_distance)
                    else:
                        self.publish_target_marker(1.0)
                    # self.marker_published = True


                self.state = 'DONE'
                self.publish_done()

        elif self.state == 'DONE' and self.marker_published:
            # Keep facing the marker using PD control
            angle_error = self.get_heading_error_to_marker()
            if angle_error is not None:
                # --- Compute distance to marker ---
                x_r, y_r, _ = self.get_pose()
                if x_r is not None:
                    dx = self.marker_world_x - x_r
                    dy = self.marker_world_y - y_r
                    distance_to_marker = math.sqrt(dx**2 + dy**2)

                    # Log distance
                    self.get_logger().info(f"📏 Distance to marker: {distance_to_marker:.2f} m")

                    # --- Stop if closer than 1m ---
                    if distance_to_marker < 1.0:
                        self.cmd_pub.publish(Twist())  # stop robot
                        self.get_logger().info("🛑 Reached marker (<1 m). Stopping robot.")
                        return

                # Dead zone threshold (radians)
                dead_zone = math.radians(10)  # e.g., 10 degrees

                # Log the error
                self.get_logger().info(f"🧭 Heading error to marker: {math.degrees(angle_error):.2f}°")

                if abs(angle_error) > dead_zone:
                    Kp = 0.8
                    Kd = 0.1
                    dt = 1.0 # timer period

                    # Initialize previous error if not present
                    if not hasattr(self, 'prev_marker_error'):
                        self.prev_marker_error = 0.0

                    # PD computation
                    derivative = (angle_error - self.prev_marker_error) / dt
                    twist = Twist()
                    twist.angular.z = max(min(Kp * angle_error + Kd * derivative, 0.4), -0.4)
                    self.cmd_pub.publish(twist)

                    # Save previous error
                    self.prev_marker_error = angle_error
                else:
                    # Within dead zone, stop rotation but go forward
                    twist_forward = Twist()
                    twist_forward.linear.x = 0.1  # Move forward slowly
                    self.cmd_pub.publish(twist_forward)
                    self.get_logger().info("✅ Heading error within dead zone, stopping rotation.")

            # Keep republishing marker
            self.keep_publishing_marker()







    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = CircleMovement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
