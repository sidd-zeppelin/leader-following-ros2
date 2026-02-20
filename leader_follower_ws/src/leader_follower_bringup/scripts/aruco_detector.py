#!/usr/bin/env python3
"""
Custom ArUco detector using OpenCV (DICT_4X4_50).
Publishes detected marker pose to /aruco_single/pose.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = 0.15  # meters
        self.target_id = 42

        # ArUco setup
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # Subscribers
        self.create_subscription(CameraInfo, '/TB3_2/camera/camera_info', self.info_cb, 1)
        self.create_subscription(Image, '/TB3_2/camera/image_raw', self.image_cb, 1)

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_single/pose', 10)
        self.debug_pub = self.create_publisher(Image, '/aruco_detector/debug', 10)

        self.get_logger().info('ArUco Detector started (DICT_4X4_50, marker_id=42)')

    def info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info(f'Camera calibration received')

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        corners, ids, rejected = self.detector.detectMarkers(frame)

        # Publish debug image
        debug = frame.copy()
        cv2.aruco.drawDetectedMarkers(debug, corners, ids)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == self.target_id:
                    # Estimate pose
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[i]], self.marker_size,
                        self.camera_matrix, self.dist_coeffs
                    )
                    rvec = rvecs[0][0]
                    tvec = tvecs[0][0]

                    # Draw axis on debug
                    cv2.drawFrameAxes(debug, self.camera_matrix, self.dist_coeffs,
                                      rvec, tvec, 0.1)

                    # Convert to PoseStamped
                    pose = PoseStamped()
                    pose.header = msg.header
                    pose.header.frame_id = 'TB3_2/camera_rgb_optical_frame'

                    # Translation
                    pose.pose.position.x = float(tvec[0])
                    pose.pose.position.y = float(tvec[1])
                    pose.pose.position.z = float(tvec[2])

                    # Rotation (rodrigues to quaternion)
                    rmat, _ = cv2.Rodrigues(rvec)
                    q = self.rotation_matrix_to_quaternion(rmat)
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]

                    self.pose_pub.publish(pose)
                    break

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))

    def rotation_matrix_to_quaternion(self, R):
        t = np.trace(R)
        if t > 0:
            s = 0.5 / np.sqrt(t + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return [x, y, z, w]

def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
