#!/usr/bin/env python3
"""
Leader Driver Node
══════════════════
Drives TB3_1 in a repeating waypoint pattern using simple go-to-goal control.
Uses odometry to navigate between waypoints.

Usage:
    python3 leader_driver.py
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class LeaderDriver(Node):
    def __init__(self):
        super().__init__('leader_driver')

        # ── Topics (TB3_1 = leader) ──
        self.cmd_pub = self.create_publisher(Twist, '/TB3_1/cmd_vel', 10)
        self.create_subscription(Odometry, '/TB3_1/odom', self.odom_cb, 10)

        # ── Waypoints (loop through these) ──
        self.waypoints = [
            (-1.0, -0.5),
            (-1.0,  0.5),
            ( 0.0,  0.5),
            ( 0.0, -0.5),
        ]
        self.current_wp = 0

        # ── Robot state ──
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.got_odom = False

        # ── Control params ──
        self.linear_speed = 0.15       # m/s (gentle pace so follower can keep up)
        self.angular_speed = 0.8       # rad/s max
        self.goal_tolerance = 0.15     # meters — close enough to waypoint

        # ── Control loop at 10 Hz ──
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Leader driver started. Waypoints: ' + str(self.waypoints))
        self.get_logger().info('Waiting for /TB3_1/odom ...')

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        if not self.got_odom:
            self.got_odom = True
            self.get_logger().info(f'Got odom. Starting at ({self.x:.2f}, {self.y:.2f})')

    def control_loop(self):
        if not self.got_odom:
            return

        # Current target
        gx, gy = self.waypoints[self.current_wp]
        dx = gx - self.x
        dy = gy - self.y
        dist = math.sqrt(dx * dx + dy * dy)

        cmd = Twist()

        if dist < self.goal_tolerance:
            # Reached waypoint — move to next
            self.current_wp = (self.current_wp + 1) % len(self.waypoints)
            gx, gy = self.waypoints[self.current_wp]
            self.get_logger().info(
                f'Reached waypoint! Next: ({gx:.1f}, {gy:.1f}) '
                f'[{self.current_wp}/{len(self.waypoints)}]'
            )
            return

        # Angle to goal
        angle_to_goal = math.atan2(dy, dx)
        angle_error = normalize_angle(angle_to_goal - self.yaw)

        if abs(angle_error) > 0.3:
            # Turn first, then drive
            cmd.angular.z = max(-self.angular_speed,
                               min(self.angular_speed, 1.5 * angle_error))
            cmd.linear.x = 0.05  # creep forward while turning
        else:
            # Drive toward goal
            cmd.linear.x = min(self.linear_speed, 0.5 * dist)
            cmd.angular.z = max(-self.angular_speed,
                               min(self.angular_speed, 1.0 * angle_error))

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
