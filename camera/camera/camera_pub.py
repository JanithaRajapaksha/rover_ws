#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp
import numpy as np
import json
import time


def get_bounding_box(landmarks, frame_width, frame_height):
    x_coords = [lm.x * frame_width for lm in landmarks]
    y_coords = [lm.y * frame_height for lm in landmarks]

    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)

    padding = 20
    x_min = max(0, int(x_min) - padding)
    y_min = max(0, int(y_min) - padding)
    x_max = min(frame_width, int(x_max) + padding)
    y_max = min(frame_height, int(y_max) + padding)

    return (x_min, y_min, x_max - x_min, y_max - y_min)


class MPTrackingNode(Node):
    def __init__(self):
        super().__init__("mp_tracking_node")

        # ROS2 Publishers
        self.pose_pub = self.create_publisher(String, "pose_data", 10)
        self.obs_pub = self.create_publisher(String, "obstacle_data", 10)

        # Mediapipe pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.8,
            model_complexity=1,
        )

        # Camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Camera could not be opened")
            return

        self.cap.set(cv2.CAP_PROP_FPS, 15)

        self.person_detected = False

        # Timer callback at ~15 Hz
        self.create_timer(0.066, self.update)

        self.get_logger().info("MediaPipe Tracking Node Started")

    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Frame capture failed")
            return

        frame_height, frame_width = frame.shape[:2]

        # --- Pose Processing ---
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_rgb.flags.writeable = False
        results = self.pose.process(frame_rgb)
        frame_rgb.flags.writeable = True

        if results.pose_landmarks:
            if not self.person_detected:
                self.get_logger().info("Person detected")
                self.person_detected = True

            bbox = get_bounding_box(results.pose_landmarks.landmark, frame_width, frame_height)
            (x, y, w, h) = bbox

            bbox_center = (x + w//2, y + h//2)
            bbox_center_norm = (
                (bbox_center[0] - frame_width//2) / (frame_width//2),
                (bbox_center[1] - frame_height//2) / (frame_height//2),
            )
            bbox_width_norm = w / frame_width

            msg = {
                "x": round(bbox_center_norm[0], 3),
                "width": round(bbox_width_norm, 3),
            }
            self.pose_pub.publish(String(data=json.dumps(msg)))

        else:
            if self.person_detected:
                self.get_logger().info("Person lost")
                self.person_detected = False

        # --- Obstacle Detection ---
        pts = np.array([
            [int(frame_width * 0.7), int(frame_height * 0.66)],
            [int(frame_width * 0.3), int(frame_height * 0.66)],
            [0, frame_height],
            [frame_width, frame_height],
        ], np.int32)
        pts = pts.reshape((-1, 1, 2))

        trapezoid_mask = np.zeros((frame_height, frame_width), np.uint8)
        cv2.fillPoly(trapezoid_mask, [pts], 255)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0
        free_space = None

        for cnt in contours:
            contour_mask = np.zeros((frame_height, frame_width), np.uint8)
            cv2.drawContours(contour_mask, [cnt], -1, 255, -1)
            if cv2.countNonZero(cv2.bitwise_and(contour_mask, trapezoid_mask)) > 0:
                area = cv2.contourArea(cnt)
                if area > max_area:
                    max_area = area
                    free_space = cnt

        free_mask = np.zeros((frame_height, frame_width), np.uint8)
        if free_space is not None:
            cv2.drawContours(free_mask, [free_space], -1, 255, -1)
            free_mask = cv2.bitwise_and(free_mask, trapezoid_mask)

        # Vertical free space
        num_zones = 3
        free_ratios = []

        ys = np.where(trapezoid_mask > 0)[0]
        if ys.size > 0:
            y_min = int(ys.min())
            y_max = int(ys.max()) + 1
            span = y_max - y_min
            zone_height = max(1, span // num_zones)
        else:
            y_min, y_max = 0, frame_height
            zone_height = (y_max - y_min) // num_zones

        for i in range(num_zones):
            y_start = y_min + i * zone_height
            y_end = y_max if i == num_zones-1 else y_min + (i+1) * zone_height

            zone_mask = np.zeros((frame_height, frame_width), np.uint8)
            zone_mask[y_start:y_end, :] = 255
            zone_mask = cv2.bitwise_and(zone_mask, trapezoid_mask)

            zone_free = cv2.bitwise_and(free_mask, zone_mask)
            ratio = cv2.countNonZero(zone_free) / max(1, cv2.countNonZero(zone_mask))
            free_ratios.append(ratio)

        weights = np.array([0.1, 0.3, 0.6])
        weights /= weights.sum()

        vertical_free_space = float(np.dot(free_ratios, weights))
        vertical_free_space = float(np.clip(vertical_free_space, 0, 1))

        # Horizontal free path
        path_rows = [int(frame_height * r) for r in [0.75, 0.8, 0.85, 0.9]]
        centers = []

        for ry in path_rows:
            row_pixels = np.where(free_mask[ry, :] > 0)[0]
            if len(row_pixels) > 0:
                centers.append(np.mean(row_pixels))

        if centers:
            path_center = int(np.mean(centers))
        else:
            path_center = frame_width // 2

        denom = frame_width/2
        horizontal_error = (path_center - (frame_width/2)) / denom
        horizontal_error = float(np.clip(horizontal_error, -1, 1))

        obs_msg = {
            "vertical": round(vertical_free_space, 3),
            "horizontal": round(horizontal_error, 3),
        }

        self.obs_pub.publish(String(data=json.dumps(obs_msg)))

    def destroy_node(self):
        self.cap.release()
        self.pose.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = MPTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
