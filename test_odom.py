#!/usr/bin/env python3
"""Test rotation and odometry through ROS 2 pipeline.

Requires:  bringup already running
Usage:     python3 test_odom.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import time, math, sys


class OdomTester(Node):
    def __init__(self):
        super().__init__('odom_tester')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.latest_js = None
        self.latest_odom = None

        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

    def _js_cb(self, msg):
        self.latest_js = msg

    def _odom_cb(self, msg):
        self.latest_odom = msg

    def wait_for_data(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_js is not None and self.latest_odom is not None:
                return True
        return False

    def print_state(self, label):
        if self.latest_js:
            pos = list(self.latest_js.position)
            vel = list(self.latest_js.velocity)
            print(f"  joint_states pos={[round(p,4) for p in pos]}  vel={[round(v,4) for v in vel]}")
        if self.latest_odom:
            p = self.latest_odom.pose.pose.position
            o = self.latest_odom.pose.pose.orientation
            # Extract yaw from quaternion
            siny = 2.0 * (o.w * o.z + o.x * o.y)
            cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
            yaw = math.atan2(siny, cosy)
            tw = self.latest_odom.twist.twist
            print(f"  odom pos=({p.x:.4f}, {p.y:.4f})  yaw={math.degrees(yaw):.2f}deg")
            print(f"  odom twist lin={tw.linear.x:.4f}  ang={tw.angular.z:.4f}")

    def send_vel(self, lin_x=0.0, ang_z=0.0):
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        self.pub.publish(msg)

    def drive(self, lin_x, ang_z, duration, rate_hz=20):
        """Publish velocity at rate_hz for duration seconds, spinning to get callbacks."""
        dt = 1.0 / rate_hz
        t0 = time.time()
        while time.time() - t0 < duration:
            self.send_vel(lin_x, ang_z)
            rclpy.spin_once(self, timeout_sec=dt)


def main():
    rclpy.init()
    node = OdomTester()

    print("Waiting for data...")
    if not node.wait_for_data(8.0):
        print("ERROR: no joint_states / odom received. Is bringup running?")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # ---- Baseline ----
    print("\n=== BASELINE ===")
    node.print_state("baseline")

    # ---- Test 1: Pure rotation (3 seconds, ang_z = 0.5) ----
    print("\n=== ROTATION: ang_z=0.5 for 3s ===")
    node.drive(0.0, 0.5, 3.0)
    node.send_vel(0.0, 0.0)
    # Let it settle
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.print_state("after rotation")

    # ---- Test 2: Forward (2 seconds, lin_x = 0.05) ----
    print("\n=== FORWARD: lin_x=0.05 for 2s ===")
    node.drive(0.05, 0.0, 2.0)
    node.send_vel(0.0, 0.0)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.print_state("after forward")

    # ---- Stop ----
    node.send_vel(0.0, 0.0)
    print("\nDone.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
