#!/usr/bin/env python3
"""
calibrate_linear.py — Linear distance accuracy calibration for TurtleBot3 Pico.

Correction target: MAX_WHEEL_SPEED_MS  (same as original)
  duty = commanded_velocity / MAX_WHEEL_SPEED_MS
  Over-travel  → increase MAX  (lowers duty → slower physical speed)
  Under-travel → decrease MAX  (raises  duty → faster physical speed)

Enhancement: two-stage wall alignment + dual Pi-side ultrasonic ground truth
             + wheel-encoder cross-check.

Workflow
--------
1.  Sense: wait for /scan, /ultrasonic/left, /ultrasonic/right, /joint_states.
2.  Align — Stage 1 (LiDAR, coarse):
      - Scan the forward sector (±ALIGN_SECTOR °) for the nearest wall surface.
      - Rotate until the 1/r²-weighted wall bearing is < ALIGN_TOL_DEG (8°).
      - Repeat up to ALIGN_MAX_ITER times.
    Align — Stage 2 (ultrasonic, fine):
      - Sample both L/R sensors repeatedly.
      - If d_left ≠ d_right the robot is angled; rotate by asin(Δ/sep) to equalise.
      - Repeat until |d_left − d_right| < USS_FINE_TOL_M (1 cm) or max iters.
3.  Space check:
      - Read current ultrasonic distance (both sensors averaged) = uss_start.
      - Need at least MIN_SPACE_M (1.0 m) in front.
      - If too close: back up and re-align.
      - Auto-compute test_dist = uss_start − WALL_CLEARANCE_M, clamped to
        [MIN_DRIVE_M … MAX_DRIVE_M].
4.  Per-pass measurement (--passes times):
      a. Sample ultrasonics (avg of USS_SAMPLES readings) → d_before (m).
      b. Reset encoder accumulators.
      c. Drive forward test_dist m at --speed m/s.
         Real-time safety stop if any US reading < WALL_STOP_M.
      d. Settle, sample ultrasonics → d_after (m).
      e. uss_delta  = d_before − d_after      (ground truth, m)
      f. enc_delta  = wheel_radius × (|ΔL| + |ΔR|) / 2   (independent check, m)
      g. correction = uss_delta / commanded_dist
5.  Average corrections → new MAX_WHEEL_SPEED_MS.

Sensor priority
  1. Dual ultrasonic average   (primary ground truth)
  2. Single ultrasonic          (active sensor fallback)
  3. Wheel encoder arc          (cross-check, printed but not used to correct)

Requirements
  • turtlebot3-bringup.service running (/odom, /joint_states, /scan active)
  • pi_ultrasonic_dual_ros2.py running (publishes /ultrasonic/left + /right)
  • At least 100 cm clear space in front of a flat wall

Usage
-----
  python3 calibrate_linear.py                   # dry-run
  python3 calibrate_linear.py --apply           # write firmware/main.c
  python3 calibrate_linear.py --apply --flash   # apply + rebuild + flash
  python3 calibrate_linear.py --no-align        # skip LiDAR wall alignment
  python3 calibrate_linear.py --passes 5 --apply
  python3 calibrate_linear.py --speed 0.06 --passes 4 --apply
"""

import argparse
import math
import re
import subprocess
import time
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import JointState, LaserScan, Range

# ── paths ─────────────────────────────────────────────────────────────────────

SCRIPT_DIR   = Path(__file__).resolve().parent
MAIN_C       = SCRIPT_DIR / "firmware" / "main.c"
FIRMWARE_DIR = SCRIPT_DIR / "firmware"
BUILD_SCRIPT = FIRMWARE_DIR / "build.sh"

# ── tunable constants ─────────────────────────────────────────────────────────

# LiDAR coarse alignment
ALIGN_SECTOR_DEG  = 60.0    # look ±60° around robot forward
ALIGN_TOL_DEG     = 8.0     # LiDAR coarse tolerance (fine-tuned by USS next)
ALIGN_SPEED       = 0.35    # rad/s for in-place rotation during coarse alignment
ALIGN_MAX_ITER    = 5       # max closed-loop LiDAR alignment iterations

# Ultrasonic fine alignment (left/right differential)
USS_FINE_TOL_M    = 0.010   # accept if |d_left − d_right| < 1 cm
USS_SENSOR_SEP_M  = 0.15    # lateral distance between the two US transducers (m)
USS_FINE_SPEED    = 0.20    # rad/s for in-place rotation during fine alignment
USS_FINE_MAX_ITER = 8       # max fine-alignment iterations

# Space / distance
MIN_SPACE_M       = 1.00    # minimum allowed forward clearance to start
WALL_STOP_M       = 0.22    # emergency stop if ultrasonic < this
WALL_CLEARANCE_M  = 0.25    # leave this much space between robot and wall
MIN_DRIVE_M       = 0.15    # minimum per-pass test distance
MAX_DRIVE_M       = 0.60    # maximum per-pass test distance
MIN_BACKUP_M      = 0.30    # minimum extra reverse when space is too small

# Ultrasonic
USS_SAMPLES       = 10      # readings to average before/after
USS_SETTLE_S      = 0.10    # seconds between successive USS readings
USS_MAX_RANGE_M   = 3.0     # discard readings above this

# Encoder
WHEEL_RADIUS_DEFAULT = 0.033    # m — used if not found in main.c

# Correction sanity clamp
MAX_CORRECTION_PCT = 0.30   # warn if |correction − 1| > 30 %

# Backup (closed-loop reverse)
BACKUP_SPEED_FALLBACK = 0.22  # m/s — fallback probe speed when min-duty unknown
BACKUP_SPEED_MARGIN   = 0.03  # m/s — added above firmware deadzone minimum
BACKUP_PROBE_S     = 0.50    # s  — probe window to confirm encoder movement
BACKUP_MOVE_ENC    = 0.003   # m  — encoder threshold to confirm movement
BACKUP_MAX_M       = 1.50    # m  — absolute max distance to reverse
BACKUP_POLL_S      = 0.05    # s  — cmd_vel re-publish / spin interval
# (BACKUP_SPEED_RAMP constants removed — speed is now derived from firmware
#  MOTOR_MIN_DUTY_LEFT/RIGHT_DEFAULT × MAX_WHEEL_SPEED_MS_DEFAULT so the
#  probe always starts above the actual hardware deadzone)
MOTOR_MIN_DUTY_DEFAULT = 0.90  # fallback if #defines not found in main.c

# QoS compatible with TurtleBot3 bringup topics
BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Colours
RED = "\033[0;31m"
GRN = "\033[0;32m"
YLW = "\033[1;33m"
CYN = "\033[0;36m"
BLD = "\033[1m"
NC  = "\033[0m"


# ── firmware source helpers ───────────────────────────────────────────────────

def read_define_float(path: Path, name: str) -> Optional[float]:
    pat = re.compile(rf"^\s*#define\s+{re.escape(name)}\s+([\d.+\-eEfF]+)")
    for line in path.read_text().splitlines():
        m = pat.match(line)
        if m:
            return float(m.group(1).rstrip("fF"))
    return None


def write_define_float(path: Path, name: str, value: float,
                       comment: str = "") -> None:
    content = path.read_text()
    suffix  = f"  // {comment}" if comment else ""
    pat     = re.compile(
        rf"(#define\s+{re.escape(name)}\s+)[\d.+\-eEfF]+f?([ \t]*(?://[^\n]*)?)"
    )
    new_content, count = pat.subn(
        lambda m: f"{m.group(1)}{value:.6f}f{suffix}",
        content, count=1,
    )
    if count == 0:
        raise RuntimeError(f"Could not find #define {name} in {path}")
    path.write_text(new_content)


# ── LiDAR helpers ─────────────────────────────────────────────────────────────

def find_wall_bearing(scan: LaserScan,
                      sector_deg: float = ALIGN_SECTOR_DEG,
                      max_range_m: float = 4.0) -> Optional[float]:
    """
    Return the bearing (rad) to the nearest wall in the forward sector ±sector_deg.

    Uses a 1/r² weighted centroid of valid scan points so a flat wall
    contributes many samples and beats isolated narrow obstacles.
    Returns None if no valid data in the sector.
    """
    sector_rad = math.radians(sector_deg)
    n = len(scan.ranges)
    ang_inc = (scan.angle_max - scan.angle_min) / max(n - 1, 1)

    w_sin = 0.0
    w_cos = 0.0
    total_w = 0.0

    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > min(scan.range_max, max_range_m):
            continue
        angle = scan.angle_min + i * ang_inc
        if abs(angle) > sector_rad:
            continue
        w = 1.0 / (r * r + 1e-6)
        w_sin += w * math.sin(angle)
        w_cos += w * math.cos(angle)
        total_w += w

    if total_w < 1e-9:
        return None
    return math.atan2(w_sin, w_cos)


def find_wall_distance_lidar(scan: LaserScan,
                              sector_deg: float = ALIGN_SECTOR_DEG
                              ) -> Optional[float]:
    """Return the minimum range (m) in the forward sector."""
    sector_rad = math.radians(sector_deg)
    n = len(scan.ranges)
    ang_inc = (scan.angle_max - scan.angle_min) / max(n - 1, 1)
    best: Optional[float] = None
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        angle = scan.angle_min + i * ang_inc
        if abs(angle) > sector_rad:
            continue
        if best is None or r < best:
            best = r
    return best


# ── ROS node ──────────────────────────────────────────────────────────────────

class LinearCalibNode(Node):
    """
    Subscribes to:  /scan, /ultrasonic/left, /ultrasonic/right,
                    /joint_states, /odom
    Publishes:      /cmd_vel
    """

    def __init__(self,
                 us_left_topic:  str = "/ultrasonic/left",
                 us_right_topic: str = "/ultrasonic/right",
                 scan_topic:     str = "/scan"):
        super().__init__("linear_calib")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # cmd_vel echo — subscribe back to our own publisher to confirm
        # messages make it into the ROS middleware
        self._cmdvel_echo_count = 0
        self._cmdvel_last_lin   = 0.0
        self.create_subscription(Twist, "/cmd_vel",
                                 self._cmdvel_echo_cb, 10)

        # Ultrasonic (Pi-side Range messages, metres)
        self._us: dict[str, Optional[float]] = {"left": None, "right": None}
        self._us_ok: dict[str, bool] = {"left": False, "right": False}
        self.create_subscription(Range, us_left_topic,
                                 lambda m: self._us_cb(m, "left"), BEST_EFFORT_QOS)
        self.create_subscription(Range, us_right_topic,
                                 lambda m: self._us_cb(m, "right"), BEST_EFFORT_QOS)

        # LiDAR
        self._scan: Optional[LaserScan] = None
        self._scan_ok = False
        self.create_subscription(LaserScan, scan_topic,
                                 self._scan_cb, BEST_EFFORT_QOS)

        # Wheel encoders
        self._joint_ok = False
        self._joint_msg_count = 0
        self._enc_collecting = False
        self._enc_pos: dict[str, Optional[float]] = {"left": None, "right": None}
        self._enc_delta: dict[str, float] = {"left": 0.0, "right": 0.0}
        self.create_subscription(JointState, "/joint_states",
                                 self._joint_cb, BEST_EFFORT_QOS)

        # Odometry (cross-check)
        self._odom_ok = False
        self._odom_x  = 0.0
        self._odom_y  = 0.0
        self.create_subscription(Odometry, "/odom",
                                 self._odom_cb, BEST_EFFORT_QOS)

    # ── callbacks ────────────────────────────────────────────────────────────

    def _cmdvel_echo_cb(self, msg: Twist) -> None:
        self._cmdvel_echo_count += 1
        self._cmdvel_last_lin    = msg.linear.x

    def _us_cb(self, msg: Range, side: str) -> None:
        self._us_ok[side] = True
        r = msg.range
        if math.isfinite(r) and msg.min_range <= r <= min(msg.max_range, USS_MAX_RANGE_M):
            self._us[side] = r
        else:
            self._us[side] = None   # out-of-range reading

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_ok = True

    def _joint_cb(self, msg: JointState) -> None:
        self._joint_ok = True
        self._joint_msg_count += 1
        if not self._enc_collecting:
            return
        try:
            li = msg.name.index("wheel_left_joint")
            ri = msg.name.index("wheel_right_joint")
        except ValueError:
            return
        l_pos = msg.position[li]
        r_pos = msg.position[ri]
        for side, pos in (("left", l_pos), ("right", r_pos)):
            prev = self._enc_pos[side]
            if prev is not None:
                delta = pos - prev
                # Wrap per-tick delta to ±π to survive position roll-over
                while delta >  math.pi: delta -= 2.0 * math.pi
                while delta < -math.pi: delta += 2.0 * math.pi
                self._enc_delta[side] += delta
            self._enc_pos[side] = pos

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_ok = True
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y

    # ── spin helpers ──────────────────────────────────────────────────────────

    def spin_for(self, secs: float) -> None:
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, lin: float, ang: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self._pub.publish(msg)

    def drive_for(self, lin: float, ang: float, secs: float) -> None:
        """Continuously publish cmd_vel for `secs` seconds, then stop.
        Unlike send()+spin_for(), this keeps re-publishing so the robot
        doesn't time out and coast to a halt mid-manoeuvre."""
        end = time.time() + secs
        while time.time() < end:
            self.send(lin, ang)
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self, settle: float = 0.5) -> None:
        self.send(0.0)
        self.spin_for(settle)

    # ── wait helpers ──────────────────────────────────────────────────────────

    def wait_for_scan(self, timeout: float = 8.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._scan_ok:
                return True
        return False

    def wait_for_ultrasonic(self, timeout: float = 8.0,
                             required: str = "either") -> bool:
        """required = 'both' | 'left' | 'right' | 'either'"""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if required == "both" and all(self._us_ok.values()):
                return True
            if required in ("left", "right") and self._us_ok[required]:
                return True
            if required == "either" and any(self._us_ok.values()):
                return True
        return False

    def wait_for_odom(self, timeout: float = 8.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._odom_ok:
                return True
        return False

    # ── encoder helpers ───────────────────────────────────────────────────────

    def enc_reset(self) -> None:
        """Reset encoder accumulators for a fresh measurement window."""
        self._enc_delta      = {"left": 0.0, "right": 0.0}
        self._enc_pos        = {"left": None, "right": None}
        self._enc_collecting = True

    def enc_stop(self) -> None:
        self._enc_collecting = False

    def enc_distance(self, wheel_radius: float) -> float:
        """Mean forward arc (m) from left+right accumulators."""
        return wheel_radius * (self._enc_delta["left"] + self._enc_delta["right"]) / 2.0

    # ── ultrasonic helpers ────────────────────────────────────────────────────

    def us_sample(self, sides: list[str],
                  n: int = USS_SAMPLES) -> dict[str, Optional[float]]:
        """
        Collect n readings from each side; discard outliers; return means.
        Returns None for a side with insufficient valid readings.
        """
        buckets: dict[str, list[float]] = {s: [] for s in sides}
        for _ in range(n):
            rclpy.spin_once(self, timeout_sec=USS_SETTLE_S + 0.02)
            time.sleep(USS_SETTLE_S)
            for s in sides:
                v = self._us[s]
                if v is not None:
                    buckets[s].append(v)

        result: dict[str, Optional[float]] = {}
        for s in sides:
            vals = buckets[s]
            if len(vals) < max(2, n // 3):
                result[s] = None
                continue
            vals_s = sorted(vals)
            med = vals_s[len(vals_s) // 2]
            filtered = [v for v in vals_s if abs(v - med) < max(0.02, med * 0.10)]
            result[s] = sum(filtered) / len(filtered) if filtered else med
        return result

    def us_average(self, sides: list[str], n: int = USS_SAMPLES) -> Optional[float]:
        """Return the mean of valid sensors, or None if none valid."""
        samples = self.us_sample(sides, n)
        vals = [v for v in samples.values() if v is not None]
        return sum(vals) / len(vals) if vals else None

    def us_current(self, sides: list[str]) -> Optional[float]:
        """Latest cached value averaged across valid sides (no new spin)."""
        vals = [self._us[s] for s in sides if self._us[s] is not None]
        return sum(vals) / len(vals) if vals else None


# ── pre-flight linear motion diagnostic ──────────────────────────────────────

def preflight_linear_check(node: LinearCalibNode,
                           probe_speed: float,
                           wheel_radius: float,
                           safe_forward_s: float = 0.18) -> bool:
    """
    Verify that linear cmd_vel commands are actually reaching the firmware
    and that the wheel encoders reflect motion.

    Sends a brief FORWARD burst (safe_forward_s seconds) at probe_speed,
    then stops.  Reports:
      • cmd_vel echo count  (confirms ROS middleware delivery)
      • joint_state msgs received during the burst
      • raw encoder arc and odom delta

    Returns True if motion was detected, False otherwise.
    """
    print(f"\n{BLD}[Pre-flight]{NC} Linear motion check at "
          f"{probe_speed*100:.1f} cm/s for {safe_forward_s*1000:.0f} ms …")

    # Snapshot baselines
    echo_before   = node._cmdvel_echo_count
    jmsg_before   = node._joint_msg_count
    odom_x0, odom_y0 = node._odom_x, node._odom_y
    node.enc_reset()

    burst_end = time.time() + safe_forward_s
    while time.time() < burst_end:
        node.send(probe_speed)          # FORWARD — robot is close to wall,
        rclpy.spin_once(node, timeout_sec=0.02)  # so keep burst very short

    node.stop(0.4)

    enc_arc   = node.enc_distance(wheel_radius)
    node.enc_stop()
    odom_d    = math.hypot(node._odom_x - odom_x0, node._odom_y - odom_y0)
    echo_rcvd = node._cmdvel_echo_count - echo_before
    jmsg_rcvd = node._joint_msg_count   - jmsg_before

    moved = abs(enc_arc) >= 0.002 or odom_d >= 0.002

    status = GRN + "PASS" + NC if moved else RED + "FAIL" + NC
    print(f"  cmd_vel echo msgs : {echo_rcvd}  "
          f"({'received' if echo_rcvd else RED + 'NONE — publisher not working?' + NC})")
    print(f"  /joint_states msgs: {jmsg_rcvd}  "
          f"({'active' if jmsg_rcvd else RED + 'NONE — bringup not publishing?' + NC})")
    print(f"  Encoder arc       : {enc_arc*10:+.1f} mm  "
          f"Odom delta: {odom_d*10:.1f} mm")
    print(f"  Motion detected   : {status}")

    if not moved:
        if echo_rcvd == 0:
            print(f"  {RED}cmd_vel not echoed — check ROS publisher / QoS.{NC}")
        elif jmsg_rcvd == 0:
            print(f"  {RED}/joint_states silent during burst — "
                  f"turtlebot3-bringup may be down.{NC}")
        else:
            print(f"  {RED}Encoder/odom showed no movement despite "
                  f"{jmsg_rcvd} joint_state msgs.{NC}")
            print(f"  {YLW}Possible causes: motor power off, "
                  f"firmware not forwarding cmd_vel, low battery, "
                  f"or MOTOR_MIN_DUTY miscalibrated.{NC}")
    return moved


# ── alignment ─────────────────────────────────────────────────────────────────

def align_to_wall(node: LinearCalibNode,
                  sector_deg: float = ALIGN_SECTOR_DEG,
                  tol_deg: float    = ALIGN_TOL_DEG,
                  speed: float      = ALIGN_SPEED,
                  max_iter: int     = ALIGN_MAX_ITER) -> bool:
    """
    Closed-loop LiDAR wall-alignment.  Rotates in place until the weighted
    centroid of the nearest wall features is within tol_deg of bearing 0.

    Returns True on convergence, False if no scan or max iterations exhausted.
    """
    print(f"\n{BLD}[Align]{NC} Finding wall in ±{sector_deg:.0f}° sector …")

    for iteration in range(max_iter):
        scan = node._scan
        if scan is None:
            node.spin_for(0.3)
            scan = node._scan
        if scan is None:
            print(f"  {RED}No LiDAR scan available.{NC}")
            return False

        bearing = find_wall_bearing(scan, sector_deg)
        if bearing is None:
            print(f"  {YLW}No valid scan points in sector — cannot align.{NC}")
            return False

        err_deg   = math.degrees(bearing)
        wall_dist = find_wall_distance_lidar(scan, sector_deg)
        dist_str  = f"{wall_dist*100:.0f} cm" if wall_dist is not None else "?"
        print(f"  Iter {iteration+1}/{max_iter}: wall ≈ {dist_str}  "
              f"bearing error = {err_deg:+.1f}°")

        if abs(err_deg) < tol_deg:
            print(f"  {GRN}Aligned! (error = {err_deg:+.1f}° < {tol_deg:.1f}°){NC}")
            node.stop(0.4)
            return True

        # Positive bearing → wall is to the left → rotate left (positive ang_z)
        rotate_time = abs(bearing) / speed
        node.drive_for(0.0, math.copysign(speed, bearing), rotate_time)
        node.stop(0.35)

    # Final check after all iterations
    scan = node._scan
    if scan:
        bearing = find_wall_bearing(scan, sector_deg)
        if bearing is not None and abs(math.degrees(bearing)) < tol_deg * 2:
            print(f"  {YLW}Partial alignment ({math.degrees(bearing):+.1f}°) — proceeding.{NC}")
            return True

    print(f"  {YLW}Alignment did not fully converge — proceeding anyway.{NC}")
    return False


def align_fine_uss(node: LinearCalibNode,
                  active_sides: list[str],
                  tol_m: float      = USS_FINE_TOL_M,
                  sensor_sep: float = USS_SENSOR_SEP_M,
                  speed: float      = USS_FINE_SPEED,
                  max_iter: int     = USS_FINE_MAX_ITER) -> bool:
    """
    Fine-tune wall perpendicularity using the left/right ultrasonic differential.

    When d_left == d_right the robot is square to the wall.
    If d_left > d_right the robot is yawed clockwise → rotate left (positive ang_z).
    Skipped if both sensors are not active.
    """
    if "left" not in active_sides or "right" not in active_sides \
            or not node._us_ok.get("left") or not node._us_ok.get("right"):
        print(f"\n{YLW}[Fine-align]{NC} Needs both sensors — skipped.")
        return False

    print(f"\n{BLD}[Fine-align]{NC} Equalising L/R ultrasonic readings "
          f"(sep={sensor_sep*100:.0f} cm, tol={tol_m*10:.0f} mm) …")

    for iteration in range(max_iter):
        samples = node.us_sample(["left", "right"], n=5)
        d_left  = samples.get("left")
        d_right = samples.get("right")

        if d_left is None or d_right is None:
            print(f"  {YLW}USS reading unavailable — aborting fine-align.{NC}")
            return False

        delta = d_left - d_right   # +ve → left farther → robot yawed CW → rotate CCW
        print(f"  Iter {iteration+1}/{max_iter}:  "
              f"L={d_left*100:.1f} cm  R={d_right*100:.1f} cm  "
              f"Δ={delta*100:+.1f} cm")

        if abs(delta) <= tol_m:
            print(f"  {GRN}Fine-aligned! "
                  f"(Δ={delta*1000:+.0f} mm < {tol_m*1000:.0f} mm){NC}")
            return True

        # Geometric correction: θ = asin(Δ / sep), clamped to avoid domain error
        clamped    = max(-sensor_sep * 0.98, min(sensor_sep * 0.98, delta))
        rotate_rad = math.asin(clamped / sensor_sep)
        rotate_time = abs(rotate_rad) / speed
        node.drive_for(0.0, math.copysign(speed, rotate_rad), rotate_time)
        node.stop(0.30)

    # Final check after exhausting iterations
    samples = node.us_sample(["left", "right"], n=5)
    d_left  = samples.get("left")
    d_right = samples.get("right")
    if d_left is not None and d_right is not None:
        delta = d_left - d_right
        print(f"  {YLW}Fine-align ended: Δ={delta*1000:+.0f} mm — proceeding.{NC}")
        return abs(delta) < tol_m * 3

    print(f"  {YLW}Fine-align did not converge — proceeding anyway.{NC}")
    return False


# ── space check + auto-distance ───────────────────────────────────────────────

def ensure_space(node: LinearCalibNode,
                 active_sides: list[str],
                 wheel_radius: float,
                 min_space_m: float       = MIN_SPACE_M,
                 wall_clearance_m: float  = WALL_CLEARANCE_M,
                 max_backup_m: float      = BACKUP_MAX_M,
                 min_moving_speed: Optional[float] = None) -> Optional[float]:
    """
    Measure ultrasonic distance to wall; if closer than min_space_m reverse.

    The probe speed is derived from the firmware's MOTOR_MIN_DUTY × MAX_WHEEL_SPEED
    (passed in as min_moving_speed) so the very first attempt already exceeds the
    hardware deadzone.  Falls back to BACKUP_SPEED_FALLBACK when unknown.

    Returns the computed test_dist (m) clamped to [MIN_DRIVE_M … MAX_DRIVE_M],
    or None if sensors give no valid reading or deadzone cannot be overcome.
    """
    print(f"\n{BLD}[Space]{NC} Measuring distance to wall …")
    uss_dist = node.us_average(active_sides, n=USS_SAMPLES)
    if uss_dist is None:
        print(f"  {RED}Ultrasonic sensors returned no valid reading.{NC}")
        return None

    print(f"  Ultrasonic distance to wall: {BLD}{uss_dist*100:.1f} cm{NC}")

    if uss_dist < min_space_m:
        target_m = min_space_m + 0.10
        print(f"  {YLW}Too close ({uss_dist*100:.0f} cm < {min_space_m*100:.0f} cm).  "
              f"Reversing to ≥{target_m*100:.0f} cm …{NC}")

        node.enc_reset()

        # ── Probe at firmware-derived minimum moving speed ────────────────────
        # The firmware maps any non-zero command linearly into
        # [MOTOR_MIN_DUTY, 1.0], so there is no point ramping from low speeds —
        # the robot either moves above the deadzone or not at all.
        if min_moving_speed is not None:
            probe_speed = min_moving_speed + BACKUP_SPEED_MARGIN
            probe_label = (f"firmware deadzone "
                           f"({min_moving_speed*100:.1f} cm/s) + "
                           f"{BACKUP_SPEED_MARGIN*100:.0f} cm/s margin")
        else:
            probe_speed = BACKUP_SPEED_FALLBACK
            probe_label = f"fallback ({probe_speed*100:.0f} cm/s)"

        moving = False
        for attempt in range(1, 4):   # up to 3 probe attempts
            node.enc_reset()
            print(f"  Probe {attempt}/3 at {probe_speed*100:.1f} cm/s "
                  f"({probe_label}) …", end="", flush=True)
            probe_end = time.time() + BACKUP_PROBE_S
            while time.time() < probe_end:
                node.send(-probe_speed)
                rclpy.spin_once(node, timeout_sec=BACKUP_POLL_S)
            node.stop(0.3)
            enc_now = abs(node.enc_distance(wheel_radius))
            print(f" encoder {enc_now*10:.1f} mm")
            if enc_now >= BACKUP_MOVE_ENC:
                moving = True
                current_speed = probe_speed
                break
            # Each retry bumps speed by 10 cm/s in case of stiction
            probe_speed = round(probe_speed + 0.10, 3)
            probe_label = f"retry at {probe_speed*100:.0f} cm/s"

        if not moving:
            # ── LiDAR fallback: derive test_dist from scan if possible ────────
            node.enc_stop()
            scan = node._scan
            lidar_dist: Optional[float] = None
            if scan is not None:
                lidar_dist = find_wall_distance_lidar(
                    scan, sector_deg=ALIGN_SECTOR_DEG)
            if lidar_dist is not None:
                print(f"  {YLW}Deadzone not overcome — "
                      f"LiDAR reports {lidar_dist*100:.1f} cm to wall.{NC}")
                if lidar_dist >= min_space_m:
                    available  = lidar_dist - wall_clearance_m
                    test_dist  = max(MIN_DRIVE_M, min(MAX_DRIVE_M, available))
                    print(f"  {YLW}Sufficient space via LiDAR — "
                          f"using as fallback (cannot reverse).{NC}")
                    print(f"  Available travel = {lidar_dist*100:.1f} cm − "
                          f"{wall_clearance_m*100:.0f} cm clearance = "
                          f"{available*100:.1f} cm")
                    print(f"  Test distance    = {BLD}{test_dist*100:.1f} cm{NC}  "
                          f"(clamped to [{MIN_DRIVE_M*100:.0f}…{MAX_DRIVE_M*100:.0f}] cm)")
                    return test_dist
                print(f"  {RED}LiDAR: {lidar_dist*100:.1f} cm < "
                      f"{min_space_m*100:.0f} cm minimum — insufficient space.{NC}")
            else:
                print(f"  {RED}No LiDAR scan available for fallback.{NC}")
            print(f"  {RED}Could not overcome deadzone — cannot reverse.{NC}")
            return None

        print(f"  {GRN}Movement confirmed at {current_speed*100:.0f} cm/s{NC}  "
              f"(encoder moved {abs(node.enc_distance(wheel_radius))*10:.1f} mm)")
        backup_speed = current_speed
        node.enc_reset()   # start fresh for the actual measured distance

        # ── Phase 2: closed-loop reverse at confirmed speed ───────────────────
        last_print = 0.0
        timeout    = max_backup_m / backup_speed + 5.0
        deadline   = time.time() + timeout

        while time.time() < deadline:
            node.send(-backup_speed)
            rclpy.spin_once(node, timeout_sec=BACKUP_POLL_S)

            enc_dist = abs(node.enc_distance(wheel_radius))
            live     = node.us_current(active_sides)
            now      = time.time()

            if now - last_print >= 0.5:
                dist_str = f"{live*100:.1f} cm" if live is not None else "---"
                print(f"  …reversed {enc_dist*100:.1f} cm  USS={dist_str}",
                      end="\r", flush=True)
                last_print = now

            if live is not None and live >= target_m:
                node.stop(0.6)
                print()
                node.enc_stop()
                uss_dist = node.us_average(active_sides, n=USS_SAMPLES)
                if uss_dist is None:
                    print(f"  {RED}No ultrasonic reading after backup.{NC}")
                    return None
                print(f"  {GRN}Target reached.{NC}  "
                      f"Reversed {enc_dist*100:.1f} cm  distance={uss_dist*100:.1f} cm")
                break

            if enc_dist >= max_backup_m:
                node.stop(0.6)
                print()
                node.enc_stop()
                print(f"  {YLW}Max backup ({max_backup_m*100:.0f} cm) reached.{NC}")
                uss_dist = node.us_average(active_sides, n=USS_SAMPLES) or live or uss_dist
                break
        else:
            node.stop(0.6)
            print()
            node.enc_stop()
            print(f"  {YLW}Backup timed out.{NC}")
            uss_dist = node.us_average(active_sides, n=USS_SAMPLES) or uss_dist

        if uss_dist is None:
            print(f"  {RED}No ultrasonic reading after backup.{NC}")
            return None

    available = uss_dist - wall_clearance_m
    test_dist = max(MIN_DRIVE_M, min(MAX_DRIVE_M, available))
    print(f"  Available travel = {uss_dist*100:.1f} cm − {wall_clearance_m*100:.0f} cm "
          f"clearance = {available*100:.1f} cm")
    print(f"  Test distance    = {BLD}{test_dist*100:.1f} cm{NC}  "
          f"(clamped to [{MIN_DRIVE_M*100:.0f}…{MAX_DRIVE_M*100:.0f}] cm)")
    return test_dist


# ── calibration pass ──────────────────────────────────────────────────────────

def run_pass(node: LinearCalibNode,
             pass_num: int,
             n_passes: int,
             test_dist_m: float,
             speed: float,
             active_sides: list[str],
             wheel_radius: float) -> Optional[float]:
    """
    Execute one calibration pass.

    Returns the correction ratio  uss_delta / commanded_dist,
    or None if the pass should be discarded.
    """
    print(f"\n{BLD}── Pass {pass_num}/{n_passes} ──{NC}  "
          f"target = {test_dist_m*100:.1f} cm  speed = {speed:.3f} m/s")

    # 1. Read ultrasonic BEFORE
    print(f"  Sampling ultrasonic before … ", end="", flush=True)
    uss_before_map = node.us_sample(active_sides, n=USS_SAMPLES)
    parts = [f"{s}={v*100:.1f}cm" if v is not None else f"{s}=---"
             for s, v in uss_before_map.items()]
    print("  ".join(parts))

    valid_before = {s: v for s, v in uss_before_map.items() if v is not None}
    if not valid_before:
        print(f"  {RED}No valid ultrasonic reading before drive — skip.{NC}")
        return None
    d_before = sum(valid_before.values()) / len(valid_before)
    print(f"  USS mean before: {BLD}{d_before*100:.2f} cm{NC}")

    if d_before < test_dist_m + WALL_STOP_M + 0.05:
        print(f"  {YLW}Wall already too close ({d_before*100:.0f} cm) — skip pass.{NC}")
        return None

    # 2. Reset encoder accumulators
    node.enc_reset()
    odom_x0 = node._odom_x
    odom_y0 = node._odom_y

    # 3. Drive forward
    duration      = test_dist_m / speed
    drive_start   = time.time()
    wall_stop_hit = False
    print(f"  Driving {test_dist_m*100:.1f} cm …", end="", flush=True)

    while time.time() - drive_start < duration:
        node.send(speed)
        rclpy.spin_once(node, timeout_sec=0.02)
        usv = node.us_current(active_sides)
        if usv is not None and usv < WALL_STOP_M:
            node.stop(0.4)
            print(f"\n  {RED}[WALL STOP]{NC} ultrasonic = {usv*100:.0f} cm "
                  f"< {WALL_STOP_M*100:.0f} cm — stopped early")
            wall_stop_hit = True
            break

    elapsed = time.time() - drive_start
    if not wall_stop_hit:
        node.stop(0.7)
    print(f" done  ({elapsed:.2f} s)")

    # Commanded distance = time × speed (open-loop ground reference)
    commanded_dist = speed * min(elapsed, duration)

    # 4. Stop encoder
    node.enc_stop()

    # 5. Encoder cross-check
    enc_dist = node.enc_distance(wheel_radius)
    dL_deg   = math.degrees(node._enc_delta["left"])
    dR_deg   = math.degrees(node._enc_delta["right"])
    print(f"  Encoder:  {enc_dist*100:.2f} cm  "
          f"(ΔL={dL_deg:+.1f}°  ΔR={dR_deg:+.1f}°)")

    # Odometry cross-check (informational)
    if node._odom_ok:
        odom_delta = math.hypot(node._odom_x - odom_x0, node._odom_y - odom_y0)
        print(f"  Odometry: {odom_delta*100:.2f} cm")

    # 6. Read ultrasonic AFTER
    print(f"  Sampling ultrasonic after  … ", end="", flush=True)
    uss_after_map = node.us_sample(active_sides, n=USS_SAMPLES)
    parts = [f"{s}={v*100:.1f}cm" if v is not None else f"{s}=---"
             for s, v in uss_after_map.items()]
    print("  ".join(parts))

    valid_after = {s: v for s, v in uss_after_map.items()
                   if v is not None and s in valid_before}
    if not valid_after:
        print(f"  {RED}No valid ultrasonic reading after drive — skip.{NC}")
        return None
    d_after = sum(valid_after.values()) / len(valid_after)
    print(f"  USS mean after:  {BLD}{d_after*100:.2f} cm{NC}")

    uss_delta = d_before - d_after
    print(f"\n  Ground truth (USS Δ) : {BLD}{uss_delta*100:.2f} cm{NC}")
    print(f"  Commanded distance  : {commanded_dist*100:.2f} cm")
    print(f"  Encoder distance    : {enc_dist*100:.2f} cm")

    if uss_delta < 0.010:
        print(f"  {YLW}USS delta too small ({uss_delta*100:.1f} mm) — skip.{NC}")
        return None

    if wall_stop_hit and commanded_dist < test_dist_m * 0.50:
        print(f"  {YLW}Stopped far too early — skip.{NC}")
        return None

    correction = uss_delta / commanded_dist
    pct_err    = (correction - 1.0) * 100.0
    tendency   = "over-travel" if correction > 1.0 else "under-travel"
    print(f"  Correction ratio    : {BLD}{correction:.4f}{NC}  "
          f"({pct_err:+.1f}%  {tendency})")

    if enc_dist > 0.001:
        enc_vs_uss = uss_delta / enc_dist
        print(f"  USS / encoder       : {enc_vs_uss:.4f}  "
              f"(≈1.00 = wheel-radius well calibrated)")

    return correction


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Calibrate MAX_WHEEL_SPEED_MS using LiDAR alignment "
                    "+ dual Pi-side ultrasonic ground truth + encoder cross-check"
    )
    parser.add_argument("--speed", type=float, default=0.07,
                        help="forward drive speed in m/s (default 0.07)")
    parser.add_argument("--passes", type=int, default=3,
                        help="calibration passes (default 3)")
    parser.add_argument("--apply", action="store_true",
                        help="write new MAX_WHEEL_SPEED_MS to firmware/main.c")
    parser.add_argument("--flash", action="store_true",
                        help="after --apply: rebuild + flash + restart service")
    parser.add_argument("--no-align", action="store_true",
                        help="skip LiDAR wall-alignment step")
    parser.add_argument("--scan-topic", default="/scan",
                        help="LiDAR scan topic (default: /scan)")
    parser.add_argument("--us-left", default="/ultrasonic/left",
                        help="left ultrasonic topic (default: /ultrasonic/left)")
    parser.add_argument("--us-right", default="/ultrasonic/right",
                        help="right ultrasonic topic (default: /ultrasonic/right)")
    parser.add_argument("--sensor", choices=["both", "left", "right"],
                        default="both",
                        help="which ultrasonic(s) to use as source of truth (default: both)")
    parser.add_argument("--align-sector", type=float, default=ALIGN_SECTOR_DEG,
                        help=f"LiDAR sector ±° for wall search (default {ALIGN_SECTOR_DEG})")
    parser.add_argument("--min-space", type=float, default=MIN_SPACE_M,
                        help=f"minimum wall clearance in m to begin (default {MIN_SPACE_M})")
    parser.add_argument("--sensor-sep", type=float, default=USS_SENSOR_SEP_M,
                        help=f"lateral distance between USS transducers in m "
                             f"(default {USS_SENSOR_SEP_M})")
    args = parser.parse_args()

    if args.flash and not args.apply:
        print("ERROR: --flash requires --apply")
        return 4

    active_sides = ["left", "right"] if args.sensor == "both" else [args.sensor]

    # Read firmware constants
    old_max = read_define_float(MAIN_C, "MAX_WHEEL_SPEED_MS_DEFAULT")
    if old_max is None:
        print(f"ERROR: MAX_WHEEL_SPEED_MS_DEFAULT not found in {MAIN_C}")
        return 2
    wheel_radius = (read_define_float(MAIN_C, "WHEEL_RADIUS_DEFAULT")
                    or WHEEL_RADIUS_DEFAULT)

    # Read per-motor deadzone so the backup probe starts at the right speed
    min_duty_l = (read_define_float(MAIN_C, "MOTOR_MIN_DUTY_LEFT_DEFAULT")
                  or read_define_float(MAIN_C, "MOTOR_MIN_DUTY_DEFAULT")
                  or MOTOR_MIN_DUTY_DEFAULT)
    min_duty_r = (read_define_float(MAIN_C, "MOTOR_MIN_DUTY_RIGHT_DEFAULT")
                  or min_duty_l)
    # Minimum speed at which the firmware will actually drive the motors
    min_moving_speed = max(min_duty_l, min_duty_r) * old_max

    print(f"\n{BLD}=== Linear Distance Calibration (ultrasonic + encoder) ==={NC}")
    print(f"  Firmware         : {MAIN_C}")
    print(f"  MAX_WHEEL_SPEED_MS_DEFAULT : {old_max:.6f} m/s  (current)")
    print(f"  Wheel radius     : {wheel_radius*1000:.1f} mm")
    print(f"  Drive speed      : {args.speed:.3f} m/s")
    print(f"  Passes           : {args.passes}")
    print(f"  Ultrasonic       : {', '.join(active_sides)}")
    print(f"  Min space needed : {args.min_space*100:.0f} cm  "
          f"(wall clearance {WALL_CLEARANCE_M*100:.0f} cm, "
          f"stop at {WALL_STOP_M*100:.0f} cm)")
    print(f"  Motor min duty   : L={min_duty_l:.2f}  R={min_duty_r:.2f}  "
          f"→ min backup speed ≈ {min_moving_speed*100:.1f} cm/s")
    print()

    rclpy.init()
    node = LinearCalibNode(
        us_left_topic=args.us_left,
        us_right_topic=args.us_right,
        scan_topic=args.scan_topic,
    )

    try:
        # ── Wait for sensors ──────────────────────────────────────────────────
        print(f"Waiting for /odom …", end=" ", flush=True)
        if node.wait_for_odom(8.0):
            print(f"{GRN}OK{NC}")
        else:
            print(f"{YLW}not available (odom cross-check disabled){NC}")

        # Ultrasonic
        print(f"Waiting for ultrasonic ({', '.join(active_sides)}) …",
              end=" ", flush=True)
        req = "both" if len(active_sides) == 2 else active_sides[0]
        if node.wait_for_ultrasonic(timeout=8.0, required=req):
            avail = [s for s in active_sides if node._us_ok[s]]
            print(f"{GRN}OK{NC}  ({', '.join(avail)} responding)")
            if len(avail) < len(active_sides):
                missing = [s for s in active_sides if not node._us_ok[s]]
                print(f"  {YLW}Warning: {', '.join(missing)} not responding — "
                      f"using only {', '.join(avail)}{NC}")
                active_sides = avail
        else:
            print(f"\n{RED}ERROR:{NC} No ultrasonic sensors found on "
                  f"{args.us_left} / {args.us_right}")
            print("  Ensure pi_ultrasonic_dual_ros2.py is running.")
            return 1

        # Encoder status
        print(f"Waiting for /joint_states …", end=" ", flush=True)
        end_t = time.time() + 5.0
        while time.time() < end_t:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node._joint_ok:
                break
        print(f"{GRN if node._joint_ok else YLW}"
              f"{'OK  (encoder cross-check enabled)' if node._joint_ok else 'not available (encoder cross-check disabled)'}"
              f"{NC}")

        # LiDAR
        if not args.no_align:
            print(f"Waiting for /scan …", end=" ", flush=True)
            if node.wait_for_scan(6.0):
                scan  = node._scan
                n_pts = len(scan.ranges) if scan else 0
                print(f"{GRN}OK{NC}  ({n_pts} pts)")
            else:
                print(f"{YLW}not available — skipping LiDAR alignment{NC}")
                args.no_align = True
        print()

        # ── Stage 1: LiDAR coarse alignment ──────────────────────────────────
        if not args.no_align:
            align_to_wall(node, sector_deg=args.align_sector)
        else:
            print(f"{YLW}[Align]{NC} Skipped.")

        # ── Stage 2: USS fine alignment ───────────────────────────────────────
        if not args.no_align:
            align_fine_uss(node, active_sides, sensor_sep=args.sensor_sep)

        # ── Space check + auto-compute test distance ──────────────────────────
        test_dist = ensure_space(
            node, active_sides, wheel_radius,
            min_space_m=args.min_space,
            wall_clearance_m=WALL_CLEARANCE_M,
            min_moving_speed=min_moving_speed,
        )
        if test_dist is None:
            print(f"\n{RED}ERROR:{NC} Cannot establish wall distance from ultrasonics.")
            return 1

        # ── Calibration passes ────────────────────────────────────────────────
        corrections: list[float] = []

        for p in range(1, args.passes + 1):
            corr = run_pass(node, p, args.passes, test_dist, args.speed,
                            active_sides, wheel_radius)
            if corr is not None:
                corrections.append(corr)
            else:
                print(f"  {YLW}Pass {p} discarded.{NC}")

            if p < args.passes:
                print(f"\n  {CYN}Drive robot back to start position.{NC}")
                input(f"  Press Enter when ready for pass {p+1} … ")
                # Re-align and re-measure space for next pass
                if not args.no_align:
                    align_to_wall(node, sector_deg=args.align_sector)
                    align_fine_uss(node, active_sides, sensor_sep=args.sensor_sep)
                new_t = ensure_space(node, active_sides, wheel_radius,
                                     min_space_m=args.min_space,
                                     wall_clearance_m=WALL_CLEARANCE_M,
                                     min_moving_speed=min_moving_speed)
                if new_t is not None:
                    test_dist = new_t

        # ── Aggregate ─────────────────────────────────────────────────────────
        if not corrections:
            print(f"\n{RED}ERROR: no valid correction factors collected.{NC}")
            return 3

        if len(corrections) > 3:
            corrections = sorted(corrections)[1:-1]   # trim outliers

        avg_corr = sum(corrections) / len(corrections)
        std_corr = (
            math.sqrt(sum((c - avg_corr) ** 2 for c in corrections) / len(corrections))
            if len(corrections) > 1 else float("nan")
        )

        new_max = max(0.05, min(2.0, old_max * avg_corr))

        print(f"\n{BLD}───── Result ─────────────────────────────────────────────{NC}")
        for i, c in enumerate(corrections):
            print(f"  Pass {i+1}: correction = {c:.4f}  ({(c-1)*100:+.1f}%)")
        print(f"  Passes used           : {len(corrections)}")
        print(f"  Avg correction        : {BLD}{avg_corr:.4f}{NC}  (σ = {std_corr:.4f})")
        print(f"  Formula               : {old_max:.6f} × {avg_corr:.4f} = {new_max:.6f}")
        print(f"  {BLD}old MAX_WHEEL_SPEED_MS_DEFAULT : {old_max:.6f} m/s{NC}")
        print(f"  {BLD}new MAX_WHEEL_SPEED_MS_DEFAULT : {new_max:.6f} m/s{NC}")

        pct = abs(avg_corr - 1.0)
        if pct > MAX_CORRECTION_PCT:
            print(f"\n  {YLW}Warning: correction {pct*100:.1f}% > "
                  f"{MAX_CORRECTION_PCT*100:.0f}% safety limit.{NC}")
            print("  Verify: robot was facing a flat wall squarely, "
                  "sensors were responding.")

        if args.apply:
            comment = (
                f"calibrated {time.strftime('%Y-%m-%d')}: "
                f"USS ground truth, correction={avg_corr:.4f} "
                f"({(avg_corr-1)*100:+.1f}%)"
            )
            write_define_float(MAIN_C, "MAX_WHEEL_SPEED_MS_DEFAULT", new_max, comment)
            print(f"\n  {GRN}Applied to:{NC} {MAIN_C.relative_to(SCRIPT_DIR)}")

            if args.flash:
                print(f"\n{BLD}── Flash ──{NC}")
                if not BUILD_SCRIPT.exists():
                    print(f"  {RED}ERROR:{NC} {BUILD_SCRIPT} not found")
                    return 5
                print("  Building and flashing firmware …")
                r = subprocess.run(["bash", str(BUILD_SCRIPT), "flash"],
                                   cwd=str(FIRMWARE_DIR))
                if r.returncode != 0:
                    print(f"  {RED}ERROR:{NC} flash failed (exit {r.returncode})")
                    return r.returncode
                print("  Restarting turtlebot3-bringup.service …")
                r = subprocess.run(
                    ["sudo", "systemctl", "restart", "turtlebot3-bringup.service"])
                if r.returncode != 0:
                    print(f"  {RED}ERROR:{NC} service restart failed")
                    return r.returncode
                print(f"  {GRN}Done — firmware flashed and service restarted.{NC}")
            else:
                print("\n  To rebuild + flash after applying:")
                print("    cd firmware && IMU_TYPE=2 ./build.sh flash")
                print("    sudo systemctl restart turtlebot3-bringup.service")
        else:
            print(f"\n  Dry-run: no files changed.  Use --apply to write changes.")

        return 0

    except KeyboardInterrupt:
        print(f"\n{YLW}Interrupted.{NC}")
        node.stop(0.3)
        return 0
    finally:
        try:
            node.stop(0.2)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
