#!/usr/bin/env python3
"""
diagnose_encoder.py — Encoder asymmetry diagnosis and heading-hold verification.

Drives the robot forward at a fixed speed and measures:
  • Left / right wheel velocities from /joint_states
  • Heading drift from /imu and /odom
  • Encoder velocity ratio (asymmetry metric)

Usage:
  python3 diagnose_encoder.py [--speed 0.10] [--duration 5.0] [--passes 3]

Requires: bringup already running (robot.launch.py)
"""

import argparse
import math
import sys
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan, Range

# ── QoS ──────────────────────────────────────────────────────────────────────
BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── Colour codes ─────────────────────────────────────────────────────────────
RED = "\033[0;31m"
GRN = "\033[0;32m"
YLW = "\033[1;33m"
CYN = "\033[0;36m"
BLD = "\033[1m"
NC  = "\033[0m"

WHEEL_RADIUS = 0.033  # m

# ── Safety thresholds ─────────────────────────────────────────────────────────
MIN_CLEARANCE_M  = 0.20   # minimum forward clearance before starting a pass
WALL_STOP_M      = 0.10   # emergency stop if obstacle closer than this
LIDAR_SECTOR_DEG = 15.0   # forward cone half-angle for LiDAR obstacle check (narrow to avoid furniture)
USS_MAX_RANGE_M  = 3.0    # discard ultrasonic readings above this


def quat_to_yaw(o) -> float:
    siny = 2.0 * (o.w * o.z + o.x * o.y)
    cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    return math.atan2(siny, cosy)


def angle_diff(a: float, b: float) -> float:
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


@dataclass
class PassResult:
    """Results from a single forward drive pass."""
    duration_s: float = 0.0
    odom_dx: float = 0.0
    odom_dy: float = 0.0
    heading_start_deg: float = 0.0
    heading_end_deg: float = 0.0
    heading_drift_deg: float = 0.0
    imu_yaw_start_deg: float = 0.0
    imu_yaw_end_deg: float = 0.0
    imu_drift_deg: float = 0.0
    vel_samples: int = 0
    mean_vel_l: float = 0.0
    mean_vel_r: float = 0.0
    ratio_l_r: float = 1.0
    mean_gyro_z: float = 0.0  # mean IMU angular_velocity.z during pass (rad/s)


class DiagNode(Node):
    def __init__(self):
        super().__init__("diagnose_encoder")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # State
        self._odom: Odometry | None = None
        self._imu: Imu | None = None
        self._joint: JointState | None = None

        # Wheel velocity samples during a pass
        self._vel_left: list[float] = []
        self._vel_right: list[float] = []
        self._gyro_z: list[float] = []   # IMU angular_velocity.z samples
        self._sampling = False

        # Encoder ticks from joint_states (radians)
        self._prev_pos_l: float | None = None
        self._prev_pos_r: float | None = None
        self._prev_stamp: float | None = None

        self.create_subscription(Odometry, "/odom", self._odom_cb, BEST_EFFORT_QOS)
        self.create_subscription(Imu, "/imu", self._imu_cb, BEST_EFFORT_QOS)
        self.create_subscription(JointState, "/joint_states",
                                 self._joint_cb, BEST_EFFORT_QOS)

        # Ultrasonic sensors (optional)
        self._us: dict[str, float | None] = {"left": None, "right": None}
        self._us_ok: dict[str, bool] = {"left": False, "right": False}
        self.create_subscription(
            Range, "/ultrasonic/left",
            lambda m: self._us_cb(m, "left"), BEST_EFFORT_QOS)
        self.create_subscription(
            Range, "/ultrasonic/right",
            lambda m: self._us_cb(m, "right"), BEST_EFFORT_QOS)

        # LiDAR (optional fallback for obstacle detection)
        self._scan: LaserScan | None = None
        self.create_subscription(LaserScan, "/scan", self._scan_cb, BEST_EFFORT_QOS)

    def _odom_cb(self, msg: Odometry):
        self._odom = msg

    def _us_cb(self, msg: Range, side: str):
        self._us_ok[side] = True
        r = msg.range
        if math.isfinite(r) and msg.min_range <= r <= min(msg.max_range, USS_MAX_RANGE_M):
            self._us[side] = r
        else:
            self._us[side] = None

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    def _imu_cb(self, msg: Imu):
        self._imu = msg
        if self._sampling:
            self._gyro_z.append(msg.angular_velocity.z)

    def _joint_cb(self, msg: JointState):
        self._joint = msg
        if not self._sampling or len(msg.position) < 2:
            return

        # TurtleBot3 node publishes position (radians) and velocity (rad/s).
        # Try velocity field first; if zero, compute from position diff.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        vl_rad = msg.velocity[0] if len(msg.velocity) > 0 else 0.0
        vr_rad = msg.velocity[1] if len(msg.velocity) > 1 else 0.0

        # If velocity field is zero, compute from position differencing
        if abs(vl_rad) < 1e-6 and abs(vr_rad) < 1e-6:
            pl = msg.position[0]
            pr = msg.position[1]
            if self._prev_pos_l is not None and self._prev_stamp is not None:
                dt = stamp - self._prev_stamp
                if dt > 0.001:
                    vl_rad = (pl - self._prev_pos_l) / dt
                    vr_rad = (pr - self._prev_pos_r) / dt
            self._prev_pos_l = pl
            self._prev_pos_r = pr
            self._prev_stamp = stamp

        vl_ms = vl_rad * WHEEL_RADIUS
        vr_ms = vr_rad * WHEEL_RADIUS
        if abs(vl_ms) > 0.001 or abs(vr_ms) > 0.001:
            self._vel_left.append(vl_ms)
            self._vel_right.append(vr_ms)

    def spin_for(self, secs: float):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send_vel(self, lin_x: float, ang_z: float = 0.0):
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        self.pub.publish(msg)

    def stop(self):
        self.send_vel(0.0)
        self.spin_for(0.3)
        self.send_vel(0.0)

    def wait_for_topics(self, timeout: float = 10.0):
        print(f"  Waiting for /odom, /imu, /joint_states ... ", end="", flush=True)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._odom and self._imu and self._joint:
                print(f"{GRN}OK{NC}")
                # Report sensor availability
                has_us = self._us_ok["left"] or self._us_ok["right"]
                has_scan = self._scan is not None
                sensors = []
                if self._us_ok["left"]:  sensors.append("USS-L")
                if self._us_ok["right"]: sensors.append("USS-R")
                if has_scan: sensors.append("LiDAR")
                if sensors:
                    print(f"  Obstacle sensors: {GRN}{', '.join(sensors)}{NC}")
                else:
                    print(f"  Obstacle sensors: {YLW}none detected yet (will keep checking){NC}")
                return True
        missing = []
        if not self._odom:  missing.append("/odom")
        if not self._imu:   missing.append("/imu")
        if not self._joint: missing.append("/joint_states")
        print(f"{RED}TIMEOUT{NC} (missing: {', '.join(missing)})")
        return False

    def get_forward_clearance(self) -> float | None:
        """Return the minimum forward distance (m) from ultrasonics or LiDAR."""
        distances: list[float] = []
        # Ultrasonic sensors
        for side in ("left", "right"):
            if self._us_ok[side] and self._us[side] is not None:
                distances.append(self._us[side])
        # LiDAR forward sector as fallback / complement
        scan = self._scan
        if scan is not None and len(scan.ranges) > 0:
            half_angle = math.radians(LIDAR_SECTOR_DEG)
            for i, r in enumerate(scan.ranges):
                angle = scan.angle_min + i * scan.angle_increment
                if abs(angle) <= half_angle:
                    if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                        distances.append(r)
        return min(distances) if distances else None

    def run_pass(self, speed: float, duration: float) -> PassResult:
        res = PassResult()
        res.duration_s = duration

        # Clear samples
        self._vel_left.clear()
        self._vel_right.clear()
        self._gyro_z.clear()
        self._prev_pos_l = None
        self._prev_pos_r = None
        self._prev_stamp = None

        # Settle and record start
        self.spin_for(0.3)

        # ── Pre-drive clearance check ────────────────────────────────────────
        clearance = self.get_forward_clearance()
        if clearance is not None:
            print(f"  Forward clearance: {clearance*100:.0f} cm", end="")
            if clearance < MIN_CLEARANCE_M:
                print(f"  {RED}< {MIN_CLEARANCE_M*100:.0f} cm — TOO CLOSE, skipping pass{NC}")
                return res
            print(f"  {GRN}OK{NC}")
        else:
            print(f"  {YLW}No ultrasonic/LiDAR data — driving cautiously{NC}")

        odom_start = self._odom
        imu_start  = self._imu
        if not odom_start or not imu_start:
            return res

        start_x = odom_start.pose.pose.position.x
        start_y = odom_start.pose.pose.position.y
        res.heading_start_deg = math.degrees(quat_to_yaw(odom_start.pose.pose.orientation))
        res.imu_yaw_start_deg = math.degrees(quat_to_yaw(imu_start.orientation))

        # Drive forward — with live obstacle detection
        self._sampling = True
        t0 = time.monotonic()
        rate_hz = 20
        aborted = False
        while time.monotonic() - t0 < duration:
            # Check for obstacles every cycle
            fwd = self.get_forward_clearance()
            if fwd is not None and fwd < WALL_STOP_M:
                print(f"  {RED}⚠ OBSTACLE at {fwd*100:.0f} cm — emergency stop!{NC}")
                aborted = True
                break
            self.send_vel(speed)
            rclpy.spin_once(self, timeout_sec=1.0 / rate_hz)

        self.stop()
        self._sampling = False
        self.spin_for(0.5)

        # Record end
        odom_end = self._odom
        imu_end  = self._imu
        if odom_end:
            res.odom_dx = odom_end.pose.pose.position.x - start_x
            res.odom_dy = odom_end.pose.pose.position.y - start_y
            end_yaw = quat_to_yaw(odom_end.pose.pose.orientation)
            res.heading_end_deg = math.degrees(end_yaw)
            res.heading_drift_deg = math.degrees(
                angle_diff(end_yaw,
                           math.radians(res.heading_start_deg)))
        if imu_end:
            end_imu_yaw = quat_to_yaw(imu_end.orientation)
            res.imu_yaw_end_deg = math.degrees(end_imu_yaw)
            res.imu_drift_deg = math.degrees(
                angle_diff(end_imu_yaw,
                           math.radians(res.imu_yaw_start_deg)))

        # Velocity stats
        res.vel_samples = min(len(self._vel_left), len(self._vel_right))
        if res.vel_samples > 2:
            # Drop first and last 10% (transient)
            trim = max(1, res.vel_samples // 10)
            vl = self._vel_left[trim:-trim] if trim < res.vel_samples // 2 else self._vel_left
            vr = self._vel_right[trim:-trim] if trim < res.vel_samples // 2 else self._vel_right
            res.mean_vel_l = sum(vl) / len(vl) if vl else 0.0
            res.mean_vel_r = sum(vr) / len(vr) if vr else 0.0
            if abs(res.mean_vel_r) > 0.001:
                res.ratio_l_r = res.mean_vel_l / res.mean_vel_r
            else:
                res.ratio_l_r = float('inf')

        # Gyro stats
        if self._gyro_z:
            trim = max(1, len(self._gyro_z) // 10)
            gz = self._gyro_z[trim:-trim] if trim < len(self._gyro_z) // 2 else self._gyro_z
            res.mean_gyro_z = sum(gz) / len(gz) if gz else 0.0

        return res


def main():
    parser = argparse.ArgumentParser(description="Encoder asymmetry diagnosis")
    parser.add_argument("--speed", type=float, default=0.10,
                        help="Forward speed in m/s (default 0.10)")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Drive duration per pass in seconds (default 5.0)")
    parser.add_argument("--passes", type=int, default=3,
                        help="Number of forward passes (default 3)")
    args = parser.parse_args()

    rclpy.init()
    node = DiagNode()

    try:
        if not node.wait_for_topics():
            print(f"{RED}Could not connect to robot topics.{NC}")
            return 1

        print()
        print(f"{BLD}═══════════════════════════════════════════════════════════{NC}")
        print(f"{BLD} Encoder Asymmetry Diagnosis{NC}")
        print(f"{BLD} Speed: {args.speed:.2f} m/s   Duration: {args.duration:.1f} s   Passes: {args.passes}{NC}")
        print(f"{BLD}═══════════════════════════════════════════════════════════{NC}")

        all_results: list[PassResult] = []

        for i in range(args.passes):
            print(f"\n{CYN}──── Pass {i+1}/{args.passes} ────{NC}")
            res = node.run_pass(args.speed, args.duration)
            all_results.append(res)

            # Print pass results
            dist = math.sqrt(res.odom_dx**2 + res.odom_dy**2)
            lat  = abs(res.odom_dy)
            print(f"  Odom distance:    {dist*100:6.1f} cm  (dx={res.odom_dx*100:.1f}, dy={res.odom_dy*100:.1f} cm)")
            print(f"  Lateral drift:    {lat*100:6.2f} cm")
            print(f"  Heading drift:    {res.heading_drift_deg:+6.2f}°  (odom)")
            print(f"  IMU yaw drift:    {res.imu_drift_deg:+6.2f}°  (IMU)")
            print(f"  Mean vel L:       {res.mean_vel_l*1000:6.1f} mm/s")
            print(f"  Mean vel R:       {res.mean_vel_r*1000:6.1f} mm/s")
            pct_diff = (res.ratio_l_r - 1.0) * 100
            colour = GRN if abs(pct_diff) < 2.0 else (YLW if abs(pct_diff) < 5.0 else RED)
            print(f"  Vel ratio L/R:    {res.ratio_l_r:.4f}  ({colour}{pct_diff:+.2f}%{NC})")
            print(f"  Mean gyro-Z:      {res.mean_gyro_z:+.4f} rad/s  ({math.degrees(res.mean_gyro_z):+.2f}°/s)")
            print(f"  Samples:          {res.vel_samples} vel, {len(node._gyro_z)} gyro")

            # Pause between passes
            if i < args.passes - 1:
                print(f"  {YLW}Pausing 2 s ...{NC}")
                node.spin_for(2.0)

        # ── Aggregate summary ────────────────────────────────────────────────
        print(f"\n{BLD}═══════════════════════════════════════════════════════════{NC}")
        print(f"{BLD} Summary across {len(all_results)} passes{NC}")
        print(f"{BLD}═══════════════════════════════════════════════════════════{NC}")

        mean_ratio = sum(r.ratio_l_r for r in all_results) / len(all_results)
        mean_drift = sum(r.heading_drift_deg for r in all_results) / len(all_results)
        mean_imu   = sum(r.imu_drift_deg for r in all_results) / len(all_results)
        mean_lat   = sum(abs(r.odom_dy) for r in all_results) / len(all_results)
        mean_vl    = sum(r.mean_vel_l for r in all_results) / len(all_results)
        mean_vr    = sum(r.mean_vel_r for r in all_results) / len(all_results)

        pct = (mean_ratio - 1.0) * 100
        colour = GRN if abs(pct) < 2.0 else (YLW if abs(pct) < 5.0 else RED)

        print(f"  Mean vel L:       {mean_vl*1000:6.1f} mm/s")
        print(f"  Mean vel R:       {mean_vr*1000:6.1f} mm/s")
        print(f"  Mean ratio L/R:   {mean_ratio:.4f}  ({colour}{pct:+.2f}%{NC})")
        print(f"  Mean heading drift: {mean_drift:+.2f}° per {args.duration:.0f}s pass")
        print(f"  Mean IMU drift:     {mean_imu:+.2f}° per {args.duration:.0f}s pass")
        print(f"  Mean lateral drift: {mean_lat*100:.2f} cm per pass")

        # Interpret
        print()
        if abs(pct) < 2.0 and abs(mean_drift) < 2.0:
            print(f"  {GRN}✓ Encoder asymmetry is minimal ({pct:+.2f}%), heading hold is working well.{NC}")
        elif abs(pct) < 5.0:
            print(f"  {YLW}⚠ Moderate encoder asymmetry ({pct:+.2f}%).{NC}")
            if pct > 0:
                print(f"    Left wheel is ~{abs(pct):.1f}% faster than right → robot drifts right.")
            else:
                print(f"    Right wheel is ~{abs(pct):.1f}% faster than left → robot drifts left.")
            print(f"    The heading-hold + encoder trim should be compensating for this.")
            print(f"    If drift persists, try increasing ENC_TRIM_KP (current default 0.5).")
        else:
            print(f"  {RED}✗ Significant encoder asymmetry ({pct:+.2f}%).{NC}")
            if pct > 0:
                print(f"    Left wheel is ~{abs(pct):.1f}% faster than right → robot drifts right.")
            else:
                print(f"    Right wheel is ~{abs(pct):.1f}% faster than left → robot drifts left.")
            print(f"    Possible causes:")
            print(f"      • Different motor friction / internal resistance")
            print(f"      • Wheel diameter difference (check tire inflation/damage)")
            print(f"      • Encoder wiring / resolution difference")
            print(f"    The firmware heading-hold + encoder trim should correct this.")
            print(f"    Consider tuning: HEADING_HOLD_KP, HEADING_HOLD_KI, ENC_TRIM_KP")

        if abs(mean_drift) > 3.0:
            print()
            drift_dir = "right (CW)" if mean_drift < 0 else "left (CCW)"
            print(f"  {YLW}⚠ Significant heading drift: {mean_drift:+.2f}°/pass → drifting {drift_dir}{NC}")
            print(f"    The heading-hold PI controller should be counteracting this.")
            print(f"    If persistent, try increasing HEADING_HOLD_KP (default 2.0)")
            print(f"    or HEADING_HOLD_KI (default 0.3).")

        # Per-pass detail table
        print(f"\n{BLD}  Pass | Dist cm | Lat cm | Hdg° | IMU° | L mm/s | R mm/s | Ratio{NC}")
        print(f"  ─────┼─────────┼────────┼──────┼──────┼────────┼────────┼──────")
        for i, r in enumerate(all_results):
            dist = math.sqrt(r.odom_dx**2 + r.odom_dy**2)
            print(f"  {i+1:4d} │ {dist*100:6.1f}  │ {abs(r.odom_dy)*100:5.2f}  │{r.heading_drift_deg:+5.1f} │"
                  f"{r.imu_drift_deg:+5.1f} │ {r.mean_vel_l*1000:5.1f}  │ {r.mean_vel_r*1000:5.1f}  │ {r.ratio_l_r:.4f}")

        print()
        return 0

    except KeyboardInterrupt:
        print(f"\n{YLW}Interrupted — stopping robot.{NC}")
        node.stop()
        return 1
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
