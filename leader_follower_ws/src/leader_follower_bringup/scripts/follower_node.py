#!/usr/bin/env python3
"""
Follower Node — Full Sensor-Only Leader Following
══════════════════════════════════════════════════
Detects ArUco marker on leader using camera, tracks with Kalman filter,
follows with pursuit controller, avoids obstacles with LiDAR.

NO access to leader's odometry or position — purely sensor-based.

Topics subscribed:
    /TB3_2/camera/image_raw     - follower's camera
    /TB3_2/camera/camera_info   - camera intrinsics
    /TB3_2/scan                 - follower's LiDAR

Topics published:
    /TB3_2/cmd_vel              - follower velocity commands
    /aruco_debug_image          - debug visualization
    /detected_aruco_pose        - detected leader pose

Usage:
    python3 follower_node.py
"""

import math
import numpy as np
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
import cv2


# ═══════════════════════════════════════════════════════════════
#  KALMAN FILTER
# ═══════════════════════════════════════════════════════════════

class KalmanTracker:
    """
    Tracks leader position (x, y) and velocity (vx, vy)
    in the follower's camera frame.
    Smooths noisy detections and coasts through brief occlusions.
    """

    def __init__(self, dt=0.1):
        self.dt = dt
        self.x = np.zeros(4)        # [x, y, vx, vy]
        self.P = np.eye(4) * 1.0    # state covariance

        # Process noise — how much we expect the leader to accelerate
        q = 0.8
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        self.Q = np.array([
            [dt4/4, 0, dt3/2, 0],
            [0, dt4/4, 0, dt3/2],
            [dt3/2, 0, dt2, 0],
            [0, dt3/2, 0, dt2]
        ]) * q * q

        # Measurement noise — ArUco is fairly accurate
        self.R = np.diag([0.03**2, 0.03**2])

        # State transition (constant velocity model)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Measurement matrix (we observe x, y)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        self.initialized = False
        self.frames_lost = 0

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement):
        """
        measurement: [x, z] from ArUco detection (camera frame)
                     or None if not detected
        """
        if measurement is not None:
            z = np.array(measurement)

            if not self.initialized:
                self.x[0] = z[0]
                self.x[1] = z[1]
                self.initialized = True
                self.frames_lost = 0
                return

            self.predict()

            # Kalman update
            y = z - self.H @ self.x
            S = self.H @ self.P @ self.H.T + self.R
            K = self.P @ self.H.T @ np.linalg.inv(S)
            self.x = self.x + K @ y
            self.P = (np.eye(4) - K @ self.H) @ self.P
            self.frames_lost = 0
        else:
            if self.initialized:
                self.predict()
                self.frames_lost += 1

    @property
    def position(self):
        return self.x[0], self.x[1]

    @property
    def velocity(self):
        return self.x[2], self.x[3]

    @property
    def is_lost(self):
        return self.frames_lost > 30  # ~3 seconds at 10Hz

    @property
    def is_tracking(self):
        return self.initialized and not self.is_lost


# ═══════════════════════════════════════════════════════════════
#  STATES
# ═══════════════════════════════════════════════════════════════

class FollowerState(Enum):
    SEARCH = 'SEARCH'
    FOLLOW = 'FOLLOW'
    TOO_CLOSE = 'TOO_CLOSE'


# ═══════════════════════════════════════════════════════════════
#  FOLLOWER NODE
# ═══════════════════════════════════════════════════════════════

class FollowerNode(Node):

    # ── ArUco config ──
    ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50
    TARGET_MARKER_ID = 42
    MARKER_REAL_SIZE = 0.15   # meters (must match your SDF marker size)

    # ── Following config ──
    DESIRED_FOLLOW_DIST = 0.60   # meters behind leader
    MIN_FOLLOW_DIST = 0.35       # too close threshold
    MAX_LINEAR_VEL = 0.20        # m/s
    MAX_ANGULAR_VEL = 1.5        # rad/s
    K_LINEAR = 0.5               # proportional gain for distance
    K_ANGULAR = 1.2              # proportional gain for bearing

    # ── Obstacle avoidance config ──
    OBSTACLE_THRESHOLD = 0.40    # react to obstacles within this range (meters)
    OBSTACLE_STOP_DIST = 0.20    # hard stop if anything this close ahead
    K_REPULSE = 0.5              # repulsive force gain

    def __init__(self):
        super().__init__('follower_node')

        # ── ArUco detector ──
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT_TYPE)
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshConstant = 7
        params.minMarkerPerimeterRate = 0.03
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.got_camera_info = False

        # ── Tracker ──
        self.tracker = KalmanTracker(dt=0.1)
        self.state = FollowerState.SEARCH

        # ── Latest sensor data ──
        self.latest_scan = None
        self.latest_detection = None   # (bearing, distance) or None

        # ── Subscribers ──
        self.create_subscription(
            CameraInfo, '/TB3_2/camera/camera_info', self.camera_info_cb, 10)
        self.create_subscription(
            Image, '/TB3_2/camera/image_raw', self.image_cb, 10)
        self.create_subscription(
            LaserScan, '/TB3_2/scan', self.scan_cb, 10)

        # ── Publishers ──
        self.cmd_pub = self.create_publisher(Twist, '/TB3_2/cmd_vel', 10)
        self.debug_img_pub = self.create_publisher(Image, '/aruco_debug_image', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_aruco_pose', 10)

        # ── Control loop at 10 Hz ──
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Follower node started.')
        self.get_logger().info(f'  ArUco marker ID: {self.TARGET_MARKER_ID}')
        self.get_logger().info(f'  Follow distance: {self.DESIRED_FOLLOW_DIST}m')
        self.get_logger().info(f'  State: SEARCH')
        self.get_logger().info('Waiting for camera and LiDAR...')

    # ─────────────────────────────────────────────────────────
    #  CAMERA CALLBACKS
    # ─────────────────────────────────────────────────────────

    def camera_info_cb(self, msg):
        if not self.got_camera_info:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.got_camera_info = True
            self.get_logger().info('Camera intrinsics received.')

    def image_cb(self, msg):
        if not self.got_camera_info:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        debug_image = cv_image.copy()
        detection = None

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)

            for i, mid in enumerate(ids.flatten()):
                if mid == self.TARGET_MARKER_ID:
                    detection = self._estimate_pose(corners[i], debug_image)
                    break

        if detection is None:
            color = (0, 0, 255) if self.state == FollowerState.SEARCH else (0, 165, 255)
            cv2.putText(debug_image, f'[{self.state.value}] No marker detected',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Update tracker
        if detection is not None:
            self.tracker.update([detection['tx'], detection['tz']])
            self.latest_detection = detection
        else:
            self.tracker.update(None)
            self.latest_detection = None

        # Draw state on debug image
        cv2.putText(debug_image, f'State: {self.state.value}',
                    (10, debug_image.shape[0] - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if self.tracker.is_tracking:
            tx, tz = self.tracker.position
            dist = math.sqrt(tx*tx + tz*tz)
            cv2.putText(debug_image, f'Track: dist={dist:.2f}m lost={self.tracker.frames_lost}',
                        (10, debug_image.shape[0] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        try:
            self.debug_img_pub.publish(
                self.bridge.cv2_to_imgmsg(debug_image, 'bgr8'))
        except:
            pass

    def _estimate_pose(self, corners, debug_image):
        """Estimate 3D pose of ArUco marker using solvePnP."""
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
            return None

        tvec = tvec.flatten()
        rvec = rvec.flatten()

        distance = math.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
        bearing = math.atan2(tvec[0], tvec[2])

        # Draw on debug image
        cv2.drawFrameAxes(debug_image, self.camera_matrix, self.dist_coeffs,
                          rvec.reshape(3,1), tvec.reshape(3,1),
                          self.MARKER_REAL_SIZE * 0.5)

        info = [
            f'ID {self.TARGET_MARKER_ID} DETECTED',
            f'Distance: {distance:.2f}m',
            f'Bearing:  {math.degrees(bearing):.1f}deg',
        ]
        for i, line in enumerate(info):
            cv2.putText(debug_image, line, (10, 25 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'camera_link'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])
        self.pose_pub.publish(pose_msg)

        return {
            'tx': tvec[0],    # horizontal offset (+ = right)
            'ty': tvec[1],    # vertical offset
            'tz': tvec[2],    # depth (distance forward)
            'distance': distance,
            'bearing': bearing,
        }

    # ─────────────────────────────────────────────────────────
    #  LIDAR CALLBACK
    # ─────────────────────────────────────────────────────────

    def scan_cb(self, msg):
        self.latest_scan = msg

    # ─────────────────────────────────────────────────────────
    #  OBSTACLE AVOIDANCE (from LiDAR)
    # ─────────────────────────────────────────────────────────

    def get_obstacle_avoidance(self):
        """
        Compute repulsive velocity adjustments from LiDAR scan.
        Returns (linear_adj, angular_adj, front_blocked).
        """
        if self.latest_scan is None:
            return 0.0, 0.0, False

        scan = self.latest_scan
        lin_adj = 0.0
        ang_adj = 0.0
        front_blocked = False

        angle = scan.angle_min
        front_min = float('inf')

        for r in scan.ranges:
            if not (scan.range_min < r < scan.range_max):
                angle += scan.angle_increment
                continue

            # Check front arc (±15 degrees) for hard stop
            if abs(angle) < math.radians(15):
                front_min = min(front_min, r)

            # Compute repulsive force for close obstacles
            if r < self.OBSTACLE_THRESHOLD:
                force = self.K_REPULSE * (1.0/r - 1.0/self.OBSTACLE_THRESHOLD) / (r * r)
                lin_adj -= force * math.cos(angle)
                ang_adj -= force * math.sin(angle)

            angle += scan.angle_increment

        if front_min < self.OBSTACLE_STOP_DIST:
            front_blocked = True

        # Clamp adjustments
        lin_adj = np.clip(lin_adj, -0.10, 0.10)
        ang_adj = np.clip(ang_adj, -0.8, 0.8)

        return lin_adj, ang_adj, front_blocked

    # ─────────────────────────────────────────────────────────
    #  MAIN CONTROL LOOP
    # ─────────────────────────────────────────────────────────

    def control_loop(self):
        cmd = Twist()

        # ── Update state machine ──
        if self.tracker.is_tracking:
            tx, tz = self.tracker.position
            dist = math.sqrt(tx*tx + tz*tz)

            if dist < self.MIN_FOLLOW_DIST:
                new_state = FollowerState.TOO_CLOSE
            else:
                new_state = FollowerState.FOLLOW
        else:
            new_state = FollowerState.SEARCH

        # Log state changes
        if new_state != self.state:
            self.get_logger().info(f'State: {self.state.value} → {new_state.value}')
            self.state = new_state

        # ── Execute behavior ──
        if self.state == FollowerState.SEARCH:
            # Rotate slowly to find the leader
            cmd.angular.z = 0.4
            cmd.linear.x = 0.0

        elif self.state == FollowerState.FOLLOW:
            tx, tz = self.tracker.position
            dist = math.sqrt(tx*tx + tz*tz)
            bearing = math.atan2(tx, tz)  # tx=horizontal, tz=depth

            # ── Pursuit controller ──
            dist_error = dist - self.DESIRED_FOLLOW_DIST
            speed = self.K_LINEAR * dist_error
            speed = np.clip(speed, 0.0, self.MAX_LINEAR_VEL)

            # Slow down if not facing the leader (large bearing)
            speed *= max(0.1, math.cos(bearing))

            steer = self.K_ANGULAR * bearing
            steer = np.clip(steer, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)

            cmd.linear.x = speed
            cmd.angular.z = steer

            # ── Apply obstacle avoidance ──
            lin_adj, ang_adj, front_blocked = self.get_obstacle_avoidance()

            if front_blocked:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5 if ang_adj >= 0 else -0.5  # turn away
                self.get_logger().warn('Obstacle ahead! Stopping forward motion.')
            else:
                cmd.linear.x = np.clip(cmd.linear.x + lin_adj,
                                       0.0, self.MAX_LINEAR_VEL)
                cmd.angular.z = np.clip(cmd.angular.z + ang_adj,
                                        -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)

        elif self.state == FollowerState.TOO_CLOSE:
            # Back up slowly
            cmd.linear.x = -0.05
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
