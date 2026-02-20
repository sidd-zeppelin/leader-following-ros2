#!/usr/bin/env python3
"""
ArUco Marker Detection Test Node
═════════════════════════════════
Subscribes to a camera image topic, detects ArUco marker ID 42,
estimates its 6-DOF pose, and publishes a debug image.

Usage (with your multi-robot setup):
    # Detect from TB3_2's camera (the follower) looking at TB3_1 (the leader):
    python3 aruco_detector_test.py --ros-args \
        -r image_raw:=/TB3_2/camera/image_raw \
        -r camera_info:=/TB3_2/camera/camera_info

    # View the debug output:
    ros2 run rqt_image_view rqt_image_view /aruco_debug_image
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2


class ArucoDetectorTest(Node):

    ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50
    TARGET_MARKER_ID = 42
    MARKER_REAL_SIZE = 0.10  # meters — must match the SDF marker size

    def __init__(self):
        super().__init__("aruco_detector_test")

        # ArUco detector
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT_TYPE)
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshConstant = 7
        params.minMarkerPerimeterRate = 0.03
        params.maxMarkerPerimeterRate = 4.0
        params.polygonalApproxAccuracyRate = 0.05
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.got_camera_info = False

        # Subscribers
        self.create_subscription(CameraInfo, "camera_info", self.camera_info_cb, 10)
        self.create_subscription(Image, "image_raw", self.image_cb, 10)

        # Publishers
        self.debug_img_pub = self.create_publisher(Image, "/aruco_debug_image", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/detected_aruco_pose", 10)

        self.get_logger().info(
            f"ArUco detector started — looking for marker ID {self.TARGET_MARKER_ID}"
        )
        self.get_logger().info("Waiting for camera_info and image_raw topics...")

    def camera_info_cb(self, msg: CameraInfo):
        if not self.got_camera_info:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.got_camera_info = True
            self.get_logger().info(
                f"Camera intrinsics received: "
                f"fx={self.camera_matrix[0,0]:.1f}, fy={self.camera_matrix[1,1]:.1f}, "
                f"cx={self.camera_matrix[0,2]:.1f}, cy={self.camera_matrix[1,2]:.1f}"
            )

    def image_cb(self, msg: Image):
        if not self.got_camera_info:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        debug_image = cv_image.copy()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == self.TARGET_MARKER_ID:
                    self._process_marker(corners[i], debug_image, msg.header)
                    break
            else:
                self._draw_text(debug_image,
                    f"Found IDs {ids.flatten().tolist()} — not ID {self.TARGET_MARKER_ID}",
                    (0, 165, 255))
        else:
            self._draw_text(debug_image, "No ArUco markers detected", (0, 0, 255))

        if rejected:
            cv2.putText(debug_image, f"Rejected candidates: {len(rejected)}",
                (10, debug_image.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_img_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Debug image publish error: {e}")

    def _process_marker(self, corners, debug_image, header):
        half = self.MARKER_REAL_SIZE / 2.0
        obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float64)

        success, rvec, tvec = cv2.solvePnP(
            obj_points, corners.reshape(4, 2),
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not success:
            self._draw_text(debug_image, "solvePnP FAILED", (0, 0, 255))
            return

        tvec = tvec.flatten()
        rvec = rvec.flatten()

        # Draw 3D axes on marker
        cv2.drawFrameAxes(debug_image, self.camera_matrix, self.dist_coeffs,
            rvec.reshape(3, 1), tvec.reshape(3, 1), self.MARKER_REAL_SIZE * 0.5)

        distance = np.linalg.norm(tvec)
        bearing_deg = math.degrees(math.atan2(tvec[0], tvec[2]))

        # Draw info overlay
        lines = [
            f"ID {self.TARGET_MARKER_ID} DETECTED",
            f"Distance: {distance:.3f} m",
            f"Bearing:  {bearing_deg:.1f} deg",
            f"tvec: [{tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}]",
        ]
        for i, line in enumerate(lines):
            cv2.putText(debug_image, line, (10, 25 + i * 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        # Log
        self.get_logger().info(
            f"Marker {self.TARGET_MARKER_ID}: "
            f"dist={distance:.3f}m, bearing={bearing_deg:.1f}deg, "
            f"tvec=[{tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}]"
        )

        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = "camera_link"
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])

        rot_matrix, _ = cv2.Rodrigues(rvec)
        q = self._rot_to_quat(rot_matrix)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.pose_pub.publish(pose_msg)

    def _draw_text(self, img, text, color):
        cv2.putText(img, text, (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    @staticmethod
    def _rot_to_quat(R):
        trace = R[0,0] + R[1,1] + R[2,2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            return [(R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s, 0.25/s]
        elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
            return [0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s]
        elif R[1,1] > R[2,2]:
            s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            return [(R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s]
        else:
            s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            return [(R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s]


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
