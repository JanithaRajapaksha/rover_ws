#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math, time, serial

# =====================================================
#  RD-03D Radar Interface  (same parser you used before)
# =====================================================
class Target:
    def __init__(self, x, y, speed, pixel_distance):
        self.x = x
        self.y = y
        self.speed = speed
        self.pixel_distance = pixel_distance
        self.distance = math.sqrt(x**2 + y**2)
        self.angle = math.degrees(math.atan2(x, y))


class RD03D:
    SINGLE_TARGET_CMD = bytes([0xFD,0xFC,0xFB,0xFA,0x02,0x00,0x80,0x00,0x04,0x03,0x02,0x01])
    MULTI_TARGET_CMD  = bytes([0xFD,0xFC,0xFB,0xFA,0x02,0x00,0x90,0x00,0x04,0x03,0x02,0x01])

    def __init__(self, port="/dev/ttyAMA0", baudrate=256000, multi_mode=True):
        self.uart = serial.Serial(port, baudrate, timeout=0.1)
        self.targets = []
        self.buffer = b''
        time.sleep(0.2)
        self.set_multi_mode(multi_mode)

    def set_multi_mode(self, multi_mode=True):
        cmd = self.MULTI_TARGET_CMD if multi_mode else self.SINGLE_TARGET_CMD
        self.uart.write(cmd)
        self.uart.flush()
        time.sleep(0.2)
        self.uart.reset_input_buffer()
        self.buffer = b''
        self.multi_mode = multi_mode

    @staticmethod
    def parse_signed16(high, low):
        raw = (high << 8) + low
        sign = -1 if (raw & 0x8000) else 1
        value = raw & 0x7FFF
        return sign * value

    def _decode_frame(self, data):
        targets = []
        if len(data) < 30 or data[0]!=0xAA or data[1]!=0xFF or data[-2]!=0x55 or data[-1]!=0xCC:
            return targets
        for i in range(3):
            base = 4 + i*8
            x = self.parse_signed16(data[base+1], data[base])
            y = self.parse_signed16(data[base+3], data[base+2])
            speed = self.parse_signed16(data[base+5], data[base+4])
            pix = data[base+6] + (data[base+7]<<8)
            targets.append(Target(x,y,speed,pix))
        return targets

    def _find_complete_frame(self, data):
        start = -1
        for i in range(len(data)-1):
            if data[i]==0xAA and data[i+1]==0xFF:
                start=i; break
        if start==-1: return None, data
        for j in range(start+2, len(data)-1):
            if data[j]==0x55 and data[j+1]==0xCC:
                frame=data[start:j+2]; remain=data[j+2:]; return frame,remain
        return None,data[start:]

    def update(self):
        if self.uart.in_waiting>0:
            self.buffer += self.uart.read(self.uart.in_waiting)
        if len(self.buffer)>300: self.buffer=self.buffer[-150:]
        latest=None; tmp=self.buffer
        while True:
            frame,tmp=self._find_complete_frame(tmp)
            if frame: latest=frame
            else: break
        if latest:
            end=self.buffer.rfind(latest)+len(latest)
            self.buffer=self.buffer[end:]
            decoded=self._decode_frame(latest)
            if decoded:
                self.targets=decoded
                return True
        return False

    def get_target(self, n=1):
        if 1<=n<=len(self.targets): return self.targets[n-1]
        return None

    def close(self):
        if self.uart.is_open: self.uart.close()


# =====================================================
#               Simple Kalman Filter
# =====================================================
class KalmanFilter:
    def __init__(self, process_var=1e-3, meas_var=1e-1):
        self.x = 0.0
        self.P = 1.0
        self.Q = process_var
        self.R = meas_var
    def update(self, measurement):
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.x += K*(measurement - self.x)
        self.P *= (1-K)
        return self.x


# =====================================================
#             Human Following Node (Full Logic)
# =====================================================
class HumanFollower(Node):
    def __init__(self):
        super().__init__("human_follower_radar")

        # ---- parameters ----
        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baudrate", 256000)
        self.declare_parameter("target_id", 1)

        self.FOLLOW_DIST = 500      # mm
        self.STOP_DIST   = 300      # mm
        self.ANGLE_TURN_THRESHOLD = 25
        self.ANGLE_MATCH_TOLERANCE = 50
        self.DIST_MATCH_TOLERANCE = 1500

        # ---- radar + filters ----
        port = self.get_parameter("port").value
        baud = self.get_parameter("baudrate").value
        self.radar = RD03D(port, baud, multi_mode=True)
        self.kf_dist  = KalmanFilter(10, 300)
        self.kf_angle = KalmanFilter(1, 100)

        # ---- publisher ----
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_tracking", 10)

        # ---- internal state ----
        self.target_locked = False
        self.locked_angle = 0.0
        self.locked_dist = 0.0
        self.last_seen = time.time()

        # ---- timer ----
        self.create_timer(0.1, self.loop)
        self.get_logger().info("✅ Human Follower Radar Node Started (Kalman Enabled)")

    # =====================================================
    def loop(self):
        if not self.radar.update():
            # check target lost timeout
            if self.target_locked and (time.time() - self.last_seen > 5):
                self.get_logger().info("Target Lost! Unlocking...")
                self.target_locked = False
                self.publish_stop()
            return

        target = self.radar.get_target(self.get_parameter("target_id").value)
        if target is None:
            return

        self.last_seen = time.time()

        # apply kalman filters
        f_dist = self.kf_dist.update(target.distance)
        f_angle = self.kf_angle.update(target.angle)

        # target lock
        if not self.target_locked:
            self.locked_angle = f_angle
            self.locked_dist = f_dist
            self.target_locked = True
            self.get_logger().info("Target Locked!")

        angle_diff = abs(f_angle - self.locked_angle)
        dist_diff  = abs(f_dist - self.locked_dist)

        if angle_diff < self.ANGLE_MATCH_TOLERANCE and dist_diff < self.DIST_MATCH_TOLERANCE:
            # main control
            cmd = Twist()
            if abs(f_angle) > self.ANGLE_TURN_THRESHOLD:
                cmd.angular.z = 0.2 if f_angle > 0 else -0.2
                self.get_logger().info(f"Turning {'Right' if f_angle>0 else 'Left'} | Angle={f_angle:.1f}")
            elif f_dist > self.FOLLOW_DIST:
                cmd.linear.x = 0.2
                self.get_logger().info(f"Moving Forward | Dist={f_dist:.1f}")
            elif f_dist < self.STOP_DIST:
                cmd.linear.x = -0.2
                self.get_logger().info(f"Moving Backward | Dist={f_dist:.1f}")
            else:
                self.publish_stop()
                return

            self.cmd_pub.publish(cmd)
        else:
            self.get_logger().info("Ignoring secondary object...")
            self.publish_stop()

    # =====================================================
    def publish_stop(self):
        stop = Twist()
        self.cmd_pub.publish(stop)

    def destroy_node(self):
        try:
            self.radar.close()
        except Exception as e:
            self.get_logger().warn(f"Radar close error: {e}")
        super().destroy_node()


# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = HumanFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
