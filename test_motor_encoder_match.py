#!/usr/bin/env python3
"""
Quick diagnostic: verify that the left PID drives the left encoder and
the right PID drives the right encoder.

Sends a brief forward cmd_vel and records /joint_states at high rate,
printing a time-series of left/right wheel positions and velocities.
Also reads the last motor commands and calibration flags from the firmware
to determine if the motor swap / reversal settings are correct.
"""
import math
import os
import sys
import time
from collections import deque

os.environ.setdefault("ROS_DOMAIN_ID", "42")

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

class MotorEncoderTest(Node):
    def __init__(self):
        super().__init__("motor_encoder_test")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._js_sub = self.create_subscription(
            JointState, "/joint_states", self._js_cb, BEST_EFFORT_QOS)
        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, BEST_EFFORT_QOS)
        self._js_data = deque(maxlen=500)  # (time, pos_l, pos_r, vel_l, vel_r)
        self._t0 = None
        self._odom = None

    def _js_cb(self, msg):
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0
        # TurtleBot3 joint_states: [left_wheel, right_wheel]
        if len(msg.position) >= 2 and len(msg.velocity) >= 2:
            self._js_data.append((
                t,
                msg.position[0], msg.position[1],
                msg.velocity[0], msg.velocity[1],
            ))

    def _odom_cb(self, msg):
        self._odom = msg

    def send(self, lin_x, ang_z=0.0):
        t = Twist()
        t.linear.x = float(lin_x)
        t.angular.z = float(ang_z)
        self._pub.publish(t)

    def stop(self):
        self.send(0.0, 0.0)

    def spin_ms(self, ms):
        end = time.monotonic() + ms / 1000.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)


def main():
    rclpy.init()
    node = MotorEncoderTest()

    print("Waiting for /joint_states …", end="", flush=True)
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node._js_data:
            break
    if not node._js_data:
        print(" TIMEOUT – no data received.")
        return 1
    print(" OK\n")

    # ── Phase 1: Baseline (stopped, 0.5 s) ────────────────────────────────
    node._js_data.clear()
    node._t0 = None
    node.stop()
    node.spin_ms(500)
    print(f"Baseline samples: {len(node._js_data)}")
    if node._js_data:
        last = node._js_data[-1]
        base_pos_l, base_pos_r = last[1], last[2]
        print(f"  pos_L={base_pos_l:.4f} rad  pos_R={base_pos_r:.4f} rad\n")
    else:
        base_pos_l = base_pos_r = 0.0

    # ── Phase 2: Forward drive (2 s at 0.08 m/s) ─────────────────────────
    node._js_data.clear()
    node._t0 = None
    print("=== FORWARD  cmd_vel.linear.x = +0.08 m/s ===")
    t_start = time.monotonic()
    while time.monotonic() - t_start < 2.0:
        node.send(0.08)
        node.spin_ms(20)
    node.stop()
    node.spin_ms(200)  # let final samples arrive

    print(f"\n{'time_s':>7}  {'pos_L':>10}  {'pos_R':>10}  {'vel_L':>10}  {'vel_R':>10}  {'Δpos_L':>10}  {'Δpos_R':>10}")
    print("-" * 77)
    first_pos_l = first_pos_r = None
    for t, pl, pr, vl, vr in node._js_data:
        if first_pos_l is None:
            first_pos_l, first_pos_r = pl, pr
        dpl = pl - first_pos_l
        dpr = pr - first_pos_r
        print(f"{t:7.3f}  {pl:10.4f}  {pr:10.4f}  {vl*1000:10.2f}  {vr*1000:10.2f}  {dpl:10.4f}  {dpr:10.4f}")

    if node._js_data:
        last = node._js_data[-1]
        first = node._js_data[0]
        total_dl = last[1] - first[1]
        total_dr = last[2] - first[2]
        dur = last[0] - first[0]
        avg_vl = total_dl / dur if dur > 0 else 0
        avg_vr = total_dr / dur if dur > 0 else 0
        WHEEL_R = 0.033
        avg_vl_ms = avg_vl * WHEEL_R
        avg_vr_ms = avg_vr * WHEEL_R

        print(f"\n  Total Δpos_L = {total_dl:+.4f} rad   Δpos_R = {total_dr:+.4f} rad")
        print(f"  Avg vel:  L = {avg_vl_ms*1000:+.1f} mm/s   R = {avg_vr_ms*1000:+.1f} mm/s")
        if abs(avg_vr_ms) > 0.001:
            ratio = avg_vl_ms / avg_vr_ms
            print(f"  L/R ratio = {ratio:.3f}")
            if abs(ratio) < 0.7:
                print(f"  ⚠ SEVERE L/R asymmetry — left wheel much slower than right")
            elif ratio < 0:
                print(f"  ⚠ WHEELS RUNNING OPPOSITE DIRECTIONS — motor swap/reversal wrong!")
        if abs(total_dl) < 0.01 and abs(total_dr) > 0.1:
            print(f"  ⚠ LEFT WHEEL NOT MOVING — check motor wiring or swap/reversal config")
        elif abs(total_dr) < 0.01 and abs(total_dl) > 0.1:
            print(f"  ⚠ RIGHT WHEEL NOT MOVING — check motor wiring or swap/reversal config")

    # ── Phase 3: Brief wait then test individual sides ────────────────────
    print("\n=== LEFT-ONLY: cmd_vel angular_z = +1.0 (CCW pivot — left backward, right forward) ===")
    node._js_data.clear()
    node._t0 = None
    t_start = time.monotonic()
    while time.monotonic() - t_start < 1.0:
        node.send(0.0, 1.0)
        node.spin_ms(20)
    node.stop()
    node.spin_ms(200)

    if node._js_data:
        first = node._js_data[0]
        last = node._js_data[-1]
        dl = last[1] - first[1]
        dr = last[2] - first[2]
        print(f"  Δpos_L = {dl:+.4f} rad (expect negative — backward)")
        print(f"  Δpos_R = {dr:+.4f} rad (expect positive — forward)")
        if dl > 0 and dr > 0:
            print(f"  ⚠ BOTH POSITIVE — motors may be swapped or reversed incorrectly")
        elif dl < 0 and dr < 0:
            print(f"  ⚠ BOTH NEGATIVE — motors may be swapped or reversed incorrectly")
        elif dl > 0 and dr < 0:
            print(f"  ⚠ INVERTED — left goes forward, right goes backward (opposite of expected)")

    print("\nDone.")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
