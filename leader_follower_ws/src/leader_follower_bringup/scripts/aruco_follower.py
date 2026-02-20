#!/usr/bin/env python3
"""
Combined ArUco detector + follower.
OpenCV DICT_4X4_50 detection + pixel-based proportional control with EMA smoothing.
No pose estimation needed - uses marker center position and apparent size.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class ArucoFollower(Node):
    def __init__(self):
        super().__init__('aruco_follower')
        self.bridge = CvBridge()

        # ArUco
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())
        self.target_id = 42

        # Subs / pubs
        self.create_subscription(Image, '/TB3_2/camera/image_raw', self.image_cb, 1)
        self.cmd_pub = self.create_publisher(Twist, '/TB3_2/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/aruco_follower/debug', 10)

        # Image info
        self.img_w = 0

        # Desired marker width in pixels (tune this: bigger = follow closer)
        # At 1920px wide image, ~150px marker width ≈ 0.5m distance
        self.desired_w = 150.0

        # Speed limits
        self.max_lin = 0.20
        self.max_ang = 1.0

        # EMA smoothing (0 = no smoothing, 1 = no update)
        self.alpha = 0.4
        self.smooth_err = 0.0
        self.smooth_size = 0.0
        self.first = True

        # Lost tracking
        self.last_seen = 0.0
        self.last_dir = 1.0
        self.create_timer(0.15, self.watchdog)

        self.get_logger().info('ArUco Follower started (DICT_4X4_50, id=42)')

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]
        self.img_w = w
        cx = w / 2.0

        corners, ids, _ = self.detector.detectMarkers(frame)

        debug = frame.copy()
        cv2.aruco.drawDetectedMarkers(debug, corners, ids)
        cv2.line(debug, (int(cx), 0), (int(cx), h), (0, 0, 255), 1)

        if ids is not None and self.target_id in ids.flatten():
            self.last_seen = time.time()
            idx = list(ids.flatten()).index(self.target_id)
            c = corners[idx][0]

            marker_cx = float(np.mean(c[:, 0]))
            marker_w = float((np.linalg.norm(c[1] - c[0]) + np.linalg.norm(c[2] - c[3])) / 2.0)

            # Normalized error: -1 (far left) to +1 (far right)
            raw_err = (marker_cx - cx) / (w / 2.0)

            # EMA smoothing
            if self.first:
                self.smooth_err = raw_err
                self.smooth_size = marker_w
                self.first = False
            else:
                self.smooth_err = self.alpha * self.smooth_err + (1.0 - self.alpha) * raw_err
                self.smooth_size = self.alpha * self.smooth_size + (1.0 - self.alpha) * marker_w

            # Remember last direction for search
            if abs(self.smooth_err) > 0.05:
                self.last_dir = 1.0 if self.smooth_err < 0 else -1.0

            # --- ANGULAR ---
            cmd = Twist()
            if abs(self.smooth_err) < 0.03:
                cmd.angular.z = 0.0  # dead zone
            else:
                cmd.angular.z = -self.max_ang * self.smooth_err

            # --- LINEAR ---
            size_ratio = self.smooth_size / self.desired_w
            if size_ratio > 2.0:
                cmd.linear.x = -0.05  # way too close, back up
            elif size_ratio > 1.2:
                cmd.linear.x = 0.0    # close enough, stop
            elif size_ratio > 0.5:
                # Proportional approach
                cmd.linear.x = self.max_lin * (1.2 - size_ratio) / 0.7
            else:
                cmd.linear.x = self.max_lin  # far, full speed

            # Slow down while turning
            cmd.linear.x *= (1.0 - 0.5 * abs(self.smooth_err))

            # Clamp
            cmd.linear.x = max(-0.05, min(self.max_lin, cmd.linear.x))
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, cmd.angular.z))

            self.cmd_pub.publish(cmd)

            # Debug overlay
            cv2.putText(debug, f'w={self.smooth_size:.0f}px err={self.smooth_err:.2f}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(debug, f'lin={cmd.linear.x:.2f} ang={cmd.angular.z:.2f}',
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(debug, f'ratio={size_ratio:.2f}',
                        (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            self.get_logger().info(
                f'TRACK w={self.smooth_size:.0f} err={self.smooth_err:.2f} '
                f'lin={cmd.linear.x:.2f} ang={cmd.angular.z:.2f}',
                throttle_duration_sec=0.5)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))

    def watchdog(self):
        if self.last_seen > 0 and (time.time() - self.last_seen) > 1.0:
            cmd = Twist()
            cmd.angular.z = 0.4 * self.last_dir
            self.cmd_pub.publish(cmd)
            self.get_logger().info('SEARCH...', throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = ArucoFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
