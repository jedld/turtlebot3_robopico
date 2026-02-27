#!/usr/bin/env python3
"""
Quick TurtleBot3 Pico turn-direction self-check.

Purpose
-------
Verifies that:
  - positive angular.z produces a LEFT turn (CCW)
  - negative angular.z produces a RIGHT turn (CW)

Safety
------
Run with wheels off the ground (on a stand).

Usage
-----
  python3 test_turn_direction.py
  python3 test_turn_direction.py --speed 0.4 --duration 0.8
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_to_yaw(orientation) -> float:
    siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny, cosy)


def angle_diff(a: float, b: float) -> float:
    delta = a - b
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


class TurnDirectionTester(Node):
    def __init__(self):
        super().__init__("turn_direction_tester")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom = None
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)

    def _odom_cb(self, msg: Odometry):
        self.odom = msg

    def send_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def wait_for_odom(self, timeout_sec: float = 8.0) -> bool:
        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None:
                return True
        return False

    def stop(self, settle_sec: float = 0.3):
        self.send_cmd(0.0, 0.0)
        stop_start = time.time()
        while time.time() - stop_start < settle_sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_rotation_test(self, angular_z: float, duration_sec: float):
        if self.odom is None:
            return None

        yaw_before = quat_to_yaw(self.odom.pose.pose.orientation)

        start = time.time()
        while time.time() - start < duration_sec:
            self.send_cmd(0.0, angular_z)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop()

        if self.odom is None:
            return None

        yaw_after = quat_to_yaw(self.odom.pose.pose.orientation)
        return angle_diff(yaw_after, yaw_before)


def classify(delta_yaw: float) -> str:
    if delta_yaw > 0.02:
        return "LEFT (CCW)"
    if delta_yaw < -0.02:
        return "RIGHT (CW)"
    return "NO CLEAR ROTATION"


def main():
    parser = argparse.ArgumentParser(description="TurtleBot3 Pico turn-direction self-check")
    parser.add_argument("--speed", type=float, default=0.5, help="|angular.z| in rad/s (default: 0.5)")
    parser.add_argument("--duration", type=float, default=1.0, help="command duration in seconds (default: 1.0)")
    args = parser.parse_args()

    print("\n=== Turn Direction Self-Check ===")
    print("Expected mapping:")
    print("  angular.z > 0  -> LEFT (CCW)")
    print("  angular.z < 0  -> RIGHT (CW)")
    print("\nSafety: keep wheels off the ground while running this check.\n")

    rclpy.init()
    node = TurnDirectionTester()

    if not node.wait_for_odom():
        print("ERROR: /odom not available. Start bringup first.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # Positive angular.z test
    delta_pos = node.run_rotation_test(abs(args.speed), args.duration)
    if delta_pos is None:
        print("ERROR: Failed during +angular.z test.")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print(f"+angular.z ({abs(args.speed):.2f}) observed: {classify(delta_pos)}  (Δyaw={math.degrees(delta_pos):+.2f}°)")

    time.sleep(0.4)

    # Negative angular.z test
    delta_neg = node.run_rotation_test(-abs(args.speed), args.duration)
    if delta_neg is None:
        print("ERROR: Failed during -angular.z test.")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print(f"-angular.z ({-abs(args.speed):.2f}) observed: {classify(delta_neg)}  (Δyaw={math.degrees(delta_neg):+.2f}°)")

    ok_pos = delta_pos > 0.02
    ok_neg = delta_neg < -0.02

    print("\nResult:")
    if ok_pos and ok_neg:
        print("PASS: turn directions match ROS convention.")
        code = 0
    else:
        print("FAIL: turn direction mapping still looks inverted or unclear.")
        code = 2

    node.stop(0.2)
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
