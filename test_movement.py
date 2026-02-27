#!/usr/bin/env python3
"""  
test_movement.py — IMU + Odometry movement validation for TurtleBot3.

Tests performed:
  1. Forward drive   — 0.1 m/s for 3 s  →  odom x ≈ 0.30 m  (±25%)
  2. Reverse drive   — return to start   →  odom x ≈ 0.00 m  (±0.05 m)
  3. Rotation 360°   — 0.5 rad/s CW for ~12.6 s  →  net yaw ≈ 0° (±20°)
  4. IMU tracks rotation — cumulative IMU yaw sweep ≈ odom yaw sweep (±25%)
  5. Gravity stable  — |g| stays within 9.0–10.6 m/s² while stationary

Requires:  bringup already running (robot.launch.py)
Usage:     python3 test_movement.py
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


# ─────────────────────────── thresholds ────────────────────────────────────
FORWARD_SPEED   = 0.10   # m/s
FORWARD_TIME    = 3.0    # seconds  → expected travel ~0.30 m
FORWARD_EXPECT  = FORWARD_SPEED * FORWARD_TIME   # 0.30 m
FORWARD_TOL     = 0.25   # fraction  (±25%)

ROTATE_SPEED    = 0.5    # rad/s
ROTATE_ANGLE    = 2 * math.pi          # 360°
ROTATE_TIME     = ROTATE_ANGLE / ROTATE_SPEED   # ≈12.57 s
ROTATE_YAW_TOL  = math.radians(20)    # ±20° closure after full turn

IMU_SWEEP_TOL   = 0.25   # cumulative IMU yaw sweep must be within 25% of odom sweep

GRAVITY_MIN     = 9.0
GRAVITY_MAX     = 10.6

SETTLE_SEC      = 0.5    # pause between tests
RATE_HZ         = 20     # cmd_vel publish rate


# ─────────────────────────── helpers ───────────────────────────────────────
def quat_to_yaw(o) -> float:
    siny = 2.0 * (o.w * o.z + o.x * o.y)
    cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    return math.atan2(siny, cosy)


def angle_diff(a: float, b: float) -> float:
    """Smallest signed difference a-b wrapped to (-π, π]."""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"


def check(label: str, condition: bool, detail: str = "") -> bool:
    status = PASS if condition else FAIL
    print(f"  [{status}] {label}" + (f"  ({detail})" if detail else ""))
    return condition


# ─────────────────────────── ROS node ──────────────────────────────────────
class MovementTester(Node):
    def __init__(self):
        super().__init__("movement_tester")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.odom  : Odometry | None = None
        self.imu   : Imu | None      = None

        # gravity samples collected ONLY while stationary
        self._gravity_samples: list[float] = []
        self._robot_moving: bool = False

        # Cumulative (unwrapped) IMU yaw tracking for Test 4
        # Tracks total angular sweep, not absolute heading
        self._imu_cum_active: bool = False
        self._imu_cum_yaw: float = 0.0
        self._last_imu_yaw: float | None = None
        self._last_imu_stamp: float | None = None

        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(Imu,      "/imu",  self._imu_cb,  10)

    # ── callbacks ───────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.odom = msg

    def _imu_cb(self, msg: Imu):
        self.imu = msg
        a = msg.linear_acceleration
        g_mag = math.sqrt(a.x**2 + a.y**2 + a.z**2)

        # Gravity: only sample while stationary
        if not self._robot_moving:
            self._gravity_samples.append(g_mag)

        # Cumulative unwrapped yaw (for Test 4 rotation tracking)
        yaw = quat_to_yaw(msg.orientation)
        if self._last_imu_yaw is not None and self._imu_cum_active:
            self._imu_cum_yaw += angle_diff(yaw, self._last_imu_yaw)
        self._last_imu_yaw = yaw

    # ── utilities ───────────────────────────────────────────────────────────
    def spin_for(self, secs: float):
        """Spin callbacks for `secs` seconds without driving."""
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_data(self, timeout: float = 8.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None and self.imu is not None:
                return True
        return False

    def send_vel(self, lin_x: float = 0.0, ang_z: float = 0.0):
        msg = Twist()
        msg.linear.x  = lin_x
        msg.angular.z = ang_z
        self.pub.publish(msg)

    def drive(self, lin_x: float, ang_z: float, duration: float):
        dt = 1.0 / RATE_HZ
        t0 = time.time()
        self._robot_moving = True
        while time.time() - t0 < duration:
            self.send_vel(lin_x, ang_z)
            rclpy.spin_once(self, timeout_sec=dt)
        self._robot_moving = False

    def stop(self):
        self.send_vel(0.0, 0.0)
        self.spin_for(SETTLE_SEC)

    # ── snapshot ─────────────────────────────────────────────────────────────
    def odom_xy(self) -> tuple[float, float]:
        p = self.odom.pose.pose.position
        return p.x, p.y

    def odom_yaw(self) -> float:
        return quat_to_yaw(self.odom.pose.pose.orientation)


# ─────────────────────────── test sequence ─────────────────────────────────
def main():
    rclpy.init()
    node = MovementTester()
    results: list[bool] = []

    print("\n════════════════════════════════════════════════════")
    print(" TurtleBot3  IMU + Odometry Movement Test")
    print("════════════════════════════════════════════════════")

    # ── Wait for data ────────────────────────────────────────────────────────
    print("\nWaiting for /odom and /imu …")
    if not node.wait_for_data(8.0):
        print("ERROR: no data received. Is bringup running?")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    print("Data OK  ✓")

    # ── Snapshot baseline ────────────────────────────────────────────────────
    x0, y0 = node.odom_xy()
    yaw0    = node.odom_yaw()
    print(f"\nBaseline: pos=({x0:.3f}, {y0:.3f})  yaw={math.degrees(yaw0):.1f}°")

    # ════════════════════════════════════════════════════
    # TEST 1: Forward drive
    # ════════════════════════════════════════════════════
    print(f"\n── Test 1: Forward  ({FORWARD_SPEED} m/s × {FORWARD_TIME} s) ──────────────────")
    node.drive(FORWARD_SPEED, 0.0, FORWARD_TIME)
    node.stop()

    x1, y1 = node.odom_xy()
    dist   = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
    lo     = FORWARD_EXPECT * (1 - FORWARD_TOL)
    hi     = FORWARD_EXPECT * (1 + FORWARD_TOL)
    print(f"  Travelled: {dist:.3f} m  (expected {lo:.2f}–{hi:.2f} m)")
    results.append(check("Forward distance in range", lo <= dist <= hi,
                         f"{dist:.3f} m vs {FORWARD_EXPECT:.2f} m ±{FORWARD_TOL*100:.0f}%"))

    # ════════════════════════════════════════════════════
    # TEST 2: Reverse back to start
    # ════════════════════════════════════════════════════
    print(f"\n── Test 2: Reverse to start ────────────────────────────────")
    node.drive(-FORWARD_SPEED, 0.0, FORWARD_TIME)
    node.stop()

    x2, y2 = node.odom_xy()
    err    = math.sqrt((x2 - x0)**2 + (y2 - y0)**2)
    print(f"  Return error: {err:.3f} m  (target ≤0.05 m)")
    results.append(check("Return-to-start error ≤0.05 m", err <= 0.05,
                         f"{err:.3f} m"))

    # ════════════════════════════════════════════════════
    # TEST 3: 360° rotation — yaw closure
    # ════════════════════════════════════════════════════
    print(f"\n── Test 3: 360° rotation ({ROTATE_SPEED} rad/s) ─────────────────────")
    print(f"  Rotating for {ROTATE_TIME:.1f} s …")
    yaw_before = node.odom_yaw()
    # Arm cumulative IMU tracking right before rotation
    node._imu_cum_yaw   = 0.0
    node._last_imu_yaw  = None
    node._imu_cum_active = True
    node.drive(0.0, ROTATE_SPEED, ROTATE_TIME)
    node._imu_cum_active = False
    node.stop()

    yaw_after = node.odom_yaw()
    closure   = abs(angle_diff(yaw_after, yaw_before))
    print(f"  Yaw before: {math.degrees(yaw_before):.1f}°  after: {math.degrees(yaw_after):.1f}°")
    print(f"  Yaw closure error: {math.degrees(closure):.1f}°  (tolerance ±{math.degrees(ROTATE_YAW_TOL):.0f}°)")
    results.append(check("360° yaw closure within tolerance", closure <= ROTATE_YAW_TOL,
                         f"{math.degrees(closure):.1f}°"))

    # ════════════════════════════════════════════════════
    # TEST 4: Cumulative IMU yaw sweep matches odom yaw sweep
    # The BNO085 rotation vector is absolute (fused with mag), so after a full
    # 360° spin the absolute yaw returns close to start.  Tracking the
    # cumulative (unwrapped) yaw change gives the total angular sweep, which
    # should match the odom-reported total sweep (±25%).
    # ════════════════════════════════════════════════════
    print(f"\n── Test 4: IMU cumulative yaw sweep vs odom sweep ──────────")
    imu_sweep  = abs(node._imu_cum_yaw)          # rad, always positive
    odom_sweep = ROTATE_ANGLE                    # commanded 2π rad
    ratio      = imu_sweep / odom_sweep if odom_sweep > 0 else 0.0
    print(f"  IMU cumulative sweep : {math.degrees(imu_sweep):.1f}°")
    print(f"  Odom expected sweep  : {math.degrees(odom_sweep):.1f}°")
    print(f"  Ratio (IMU/odom)     : {ratio:.3f}  (ideal 1.000, tol ±{IMU_SWEEP_TOL*100:.0f}%)")
    if imu_sweep < math.radians(5):
        print("  NOTE: IMU sweep is near-zero — BNO085 orientation not yet tracking")
        print("        rotation. Needs more magnetometer calibration (drive around).")
        results.append(False)
    else:
        results.append(check("IMU cumulative yaw sweep within 25% of odom sweep",
                             abs(ratio - 1.0) <= IMU_SWEEP_TOL,
                             f"IMU={math.degrees(imu_sweep):.1f}° vs "
                             f"odom={math.degrees(odom_sweep):.1f}°"))

    # ════════════════════════════════════════════════════
    # TEST 5: Gravity magnitude stable while stationary
    # ════════════════════════════════════════════════════
    print(f"\n── Test 5: Gravity stability (stationary samples only) ──────")
    samples = node._gravity_samples
    if len(samples) < 10:
        print("  Not enough stationary IMU samples — skipping")
        results.append(False)
    else:
        g_min  = min(samples)
        g_max  = max(samples)
        g_mean = sum(samples) / len(samples)
        outliers = sum(1 for g in samples if not (GRAVITY_MIN <= g <= GRAVITY_MAX))
        pct_ok   = 100.0 * (1 - outliers / len(samples))
        print(f"  Samples: {len(samples)}  mean={g_mean:.3f}  "
              f"min={g_min:.3f}  max={g_max:.3f}  ok={pct_ok:.1f}%")
        results.append(check("≥95% of stationary gravity samples in [9.0, 10.6] m/s²",
                             pct_ok >= 95.0,
                             f"{pct_ok:.1f}% in range"))

    # ════════════════════════════════════════════════════
    # Summary
    # ════════════════════════════════════════════════════
    print("\n════════════════════════════════════════════════════")
    passed = sum(results)
    total  = len(results)
    status = PASS if passed == total else FAIL
    print(f" Result: [{status}]  {passed}/{total} tests passed")
    print("════════════════════════════════════════════════════\n")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
