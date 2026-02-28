#!/usr/bin/env python3
"""
straight_calibrate.py — Iterative straight-line calibration for TurtleBot3.

Uses the front-facing LiDAR to measure actual travel distance and angular
drift while driving perpendicular to a wall.  Computes corrections for:

  1. MAX_WHEEL_SPEED_MS — forward distance accuracy (LiDAR ground-truth vs odom)
     Only applied when wall metrics are trusted (robot ≤15° off-perpendicular).
  2. MOTOR_TRIM_LEFT / MOTOR_TRIM_RIGHT — per-motor duty to cancel lateral drift
     (measured from IMU yaw + LiDAR wall-plane angle change)

The corrections are written back to:
  • firmware/main.c  (MAX_WHEEL_SPEED_MS, MOTOR_TRIM_LEFT, MOTOR_TRIM_RIGHT)

After updating, the script rebuilds and reflashes the firmware automatically,
then runs one more verification pass to confirm calibration.

Safety:
  • Robot stops immediately if front LiDAR < WALL_STOP_M (0.35 m default)
  • Each pass uses a conservative travel distance (≤ 20 cm)

Usage:
  python3 straight_calibrate.py [OPTIONS]

Options:
  --distance M    travel per pass in metres      (default 0.20)
  --speed    M/S  forward speed                  (default 0.05)
  --passes   N    calibration passes to average  (default 3)
    --manual-start  fully manual start position for every pass (no auto-return)
    --imu-only      calibrate straightness from IMU/odometry only (ignore LiDAR metrics)
  --no-flash      update files but skip rebuild/flash
  --verify-only   run one measurement pass, print results, exit (no changes)

Requires: ROS 2 bringup already running with /scan, /imu, /odom topics.
"""

import argparse
import math
import os
import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, LaserScan

# Sensor topics (scan, imu) are published as BEST_EFFORT / VOLATILE by TurtleBot3
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ─────────────────────────── paths ─────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
FIRMWARE_DIR = os.path.join(SCRIPT_DIR, "firmware")
MAIN_C       = os.path.join(FIRMWARE_DIR, "main.c")
YAML_PATH    = os.path.join(SCRIPT_DIR, "config", "burger_pico.yaml")

# ─────────────────────────── safety / test constants ───────────────────────
WALL_STOP_M     = 0.35   # emergency stop if front range drops below this
WALL_WARN_M     = 0.50   # warning banner threshold
SCAN_WINDOW_DEG = 40.0   # degrees either side of front used for wall-plane fit
MIN_WALL_PTS    = 8      # minimum valid scan points for a reliable fit
SETTLE_SEC      = 1.5    # pause after stopping before taking measurements
RATE_HZ         = 20     # cmd_vel publish rate
# Maximum |pre_wall_angle| for which LiDAR wall-plane metrics are trusted.
# Beyond this the scan window is unevenly clipped, making the SVD normal vector
# noisy: a few mm of forward motion produces tens of degrees of apparent drift
# and the perpendicular-distance reading becomes unreliable.
WALL_ANGLE_TRUST_DEG = 15.0

# ─────────────────────────── correction damping ────────────────────────────
# Apply only this fraction of the computed correction each pass to avoid
# overshooting.  Increase toward 1.0 for faster convergence.
CORRECTION_DAMP = 0.75    # 75% of computed correction applied

# ─────────────────────────── return-to-start control ───────────────────────
# Closed-loop auto-return between passes (IMU yaw + LiDAR wall distance).
RETURN_YAW_KP       = 1.8          # P-gain for in-place yaw correction
RETURN_YAW_KD       = 1.2          # D-gain — uses IMU angular_velocity.z directly
RETURN_YAW_MAX      = 0.28         # rad/s cap on angular command
RETURN_YAW_TOL      = math.radians(2.0)   # convergence tolerance
RETURN_YAW_SETTLED  = 6            # number of consecutive on-target cycles to declare done
RETURN_LAT_KP       = 0.30         # P-gain for heading hold while reversing
RETURN_LAT_KD       = 0.20         # D-gain for heading hold while reversing
RETURN_LAT_MAX      = 0.12         # rad/s cap on lateral correction while reversing
RETURN_WALL_TOL     = 0.015        # stop reversing when wall dist within 1.5 cm of target
RETURN_SETTLE_SEC   = 0.8          # settle pause between return sub-steps

# Wall-relative localisation ("limited SLAM").
# Instead of trusting IMU yaw alone, we continuously fit wall_angle from
# the LiDAR scan plane.  wall_angle==0 means robot is exactly perpendicular.
RETURN_WALL_SCAN_N    = 6          # scans to median-filter for robust wall-pose estimate
RETURN_WALL_ANG_TOL   = math.radians(1.8)  # alignment tolerance for wall-angle controller
RETURN_WALL_ANG_KP    = 1.0        # P-gain: wall_angle → 0
RETURN_WALL_ANG_KD    = 1.4        # D-gain: damps IMU omega while squaring to wall
RETURN_WALL_ANG_MAX   = 0.15       # rad/s cap
RETURN_WALL_ANG_SETTLED = 6        # consecutive on-target cycles required

# ─────────────────────────── duty-cycle calibration ──────────────────────────
# Slow-speed sweep to detect motor stall and jerkiness, then calibrate
# MOTOR_MIN_DUTY and MOTOR_KICK_DUTY in firmware/main.c.
DUTY_SWEEP_SPEEDS   = [0.025, 0.040, 0.060]   # m/s commanded during sweep steps
DUTY_SWEEP_SECS     = 2.0    # seconds of driving per sweep step
STALL_VEL_THRESH    = 0.006  # m/s — odom velocity below this = effectively stalled
STUTTER_COV_THRESH  = 0.35   # coefficient-of-variation threshold → stutter
JERK_SAMPLE_SECS    = 0.45   # window for startup angular-jerk capture
JERK_ALPHA_THRESH   = 8.0    # rad/s² — peak |dω/dt| above this = excessive kick
DUTY_CHANGE_MIN     = 0.01   # only write new MIN_DUTY if it differs by at least this
DUTY_MIN_BOUND      = 0.30   # hard lower bound for MOTOR_MIN_DUTY search
DUTY_MAX_BOUND      = 0.80   # hard upper bound


# ═══════════════════════════════════════════════════════════════════════════
# LIDAR wall-plane fitting
# ═══════════════════════════════════════════════════════════════════════════

def fit_wall(scan: LaserScan, window_deg: float = SCAN_WINDOW_DEG):
    """Return (wall_dist_m, wall_angle_rad) from a LaserScan.

    wall_dist_m  — perpendicular distance from robot origin to the wall.
    wall_angle_rad — signed angle by which the wall's inward normal deviates
                     from the robot's x-axis (+ve = normal tilts left = robot
                     has yawed right relative to wall = drifted right).
    Returns (None, None) if not enough valid points.
    """
    n = len(scan.ranges)
    if n == 0:
        return None, None

    win_rad = math.radians(window_deg)
    pts = []
    for i in range(n):
        angle = scan.angle_min + i * scan.angle_increment
        # Normalise to (-π, π)
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(angle) > win_rad:
            continue
        r = scan.ranges[i]
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        pts.append((r * math.cos(angle), r * math.sin(angle)))  # (x_fwd, y_left)

    if len(pts) < MIN_WALL_PTS:
        return None, None

    # Total-least-squares (SVD) line fit in robot frame.
    # The wall is a near-vertical line in (x_fwd, y_left) space.
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    n_pts = len(xs)
    mx = sum(xs) / n_pts
    my = sum(ys) / n_pts

    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    sxy = sum((xs[i] - mx) * (ys[i] - my) for i in range(n_pts))

    # Eigenvalues of the 2×2 covariance matrix
    tr   = sxx + syy
    det  = sxx * syy - sxy * sxy
    disc = math.sqrt(max(0.0, (tr / 2) ** 2 - det))
    lam_small = tr / 2 - disc  # eigenvalue for the wall-normal direction

    # Corresponding eigenvector (wall inward normal)
    nx = sxx - lam_small
    ny = sxy
    norm = math.sqrt(nx * nx + ny * ny)
    if norm < 1e-9:
        nx, ny = 1.0, 0.0
    else:
        nx /= norm
        ny /= norm
    if nx < 0:          # normal should point forward toward wall
        nx, ny = -nx, -ny

    wall_dist  = mx * nx + my * ny           # perpendicular distance
    wall_angle = math.atan2(ny, nx)          # deviation from +x axis

    return wall_dist, wall_angle


def median_wall(scans, window_deg=SCAN_WINDOW_DEG):
    """Return (dist, angle) as median over a list of LaserScan messages."""
    results = [fit_wall(s, window_deg) for s in scans]
    results = [(d, a) for d, a in results if d is not None]
    if not results:
        return None, None
    dists  = sorted(r[0] for r in results)
    angles = sorted(r[1] for r in results)
    mid = len(results) // 2
    return dists[mid], angles[mid]


# ═══════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════

def quat_to_yaw(o) -> float:
    siny = 2.0 * (o.w * o.z + o.x * o.y)
    cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    return math.atan2(siny, cosy)


def angle_diff(a, b):
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


def fmt(v, unit="", prec=4):
    return f"{v:+.{prec}f}{unit}"


RED  = "\033[0;31m"
GRN  = "\033[0;32m"
YLW  = "\033[1;33m"
CYN  = "\033[0;36m"
BLD  = "\033[1m"
NC   = "\033[0m"

def ok(s):   print(f"{GRN}[PASS]{NC}  {s}")
def fail(s): print(f"{RED}[FAIL]{NC}  {s}")
def info(s): print(f"{CYN}[INFO]{NC}  {s}")
def warn(s): print(f"{YLW}[WARN]{NC}  {s}")
def hdr(s):  print(f"\n{BLD}{'─'*60}\n  {s}\n{'─'*60}{NC}")


# ═══════════════════════════════════════════════════════════════════════════
# Firmware / YAML parameter I/O
# ═══════════════════════════════════════════════════════════════════════════

def read_firmware_float(path: str, name: str) -> float | None:
    """Read a #define float constant from a C file."""
    with open(path) as f:
        for line in f:
            m = re.match(rf'\s*#define\s+{re.escape(name)}\s+([\d.+\-eEfF]+)', line)
            if m:
                return float(m.group(1).rstrip('fF'))
    return None


def write_firmware_float(path: str, name: str, value: float, comment: str = ""):
    """Replace a #define float constant in a C file (preserves surrounding text)."""
    with open(path) as f:
        content = f.read()

    # Match the define line, preserving any trailing comment
    # Use [ \t]* (not \s*) to avoid consuming the newline and merging adjacent defines
    pattern = rf'(#define\s+{re.escape(name)}\s+)[\d.+\-eEfF]+f([ \t]*(?://[^\n]*)?)'  
    suffix  = f"  // {comment}" if comment else ""

    def replacer(m):
        return f"{m.group(1)}{value:.6f}f{suffix}"

    new_content, n = re.subn(pattern, replacer, content)
    if n == 0:
        raise ValueError(f"Could not find #define {name} in {path}")
    with open(path, 'w') as f:
        f.write(new_content)


def write_yaml_radius(path: str, value: float):
    """Update wheels.radius in burger_pico.yaml."""
    with open(path) as f:
        content = f.read()
    new_content, n = re.subn(
        r'(radius:\s*)[\d.]+(\s*#[^\n]*)',
        lambda m: f"{m.group(1)}{value:.5f}{m.group(2)}",
        content
    )
    if n == 0:
        # fallback: no inline comment
        new_content, n = re.subn(r'(radius:\s*)[\d.]+', f"radius: {value:.5f}", content)
    if n == 0:
        raise ValueError(f"Could not find 'radius:' in {path}")
    with open(path, 'w') as f:
        f.write(new_content)


# ═══════════════════════════════════════════════════════════════════════════
# Correction maths
# ═══════════════════════════════════════════════════════════════════════════

def compute_radius_correction(odom_dist_m: float, lidar_dist_m: float,
                               current_radius: float) -> float:
    """
    If the robot travels lidar_dist_m in reality but odom reports odom_dist_m,
    the wheel radius in firmware is off by the same ratio.

    corrected = current * lidar_dist / odom_dist
    """
    if odom_dist_m < 0.01:
        return current_radius
    return current_radius * (lidar_dist_m / odom_dist_m)


def compute_trim_correction(yaw_drift_rad: float, travel_m: float,
                             wheel_sep_m: float,
                             trim_left: float, trim_right: float,
                             damp: float = CORRECTION_DAMP):
    """
    Compute corrected MOTOR_TRIM_LEFT / MOTOR_TRIM_RIGHT.

    yaw_drift_rad positive → robot turned CCW (left) → right motor ran faster.
    yaw_drift_rad negative → robot turned CW  (right) → left motor ran faster.

    Differential arc geometry:
        r_turn = travel_m / yaw_drift_rad   (signed turn radius)
        v_ratio = (r_turn - sep/2) / (r_turn + sep/2)
    where v_ratio = v_right / v_left for a leftward drift.

    We slow the faster motor.  One trim stays at 1.0; only the other changes.
    """
    if abs(yaw_drift_rad) < math.radians(0.3):
        return trim_left, trim_right     # negligible drift, no change

    r_turn   = travel_m / yaw_drift_rad if abs(yaw_drift_rad) > 1e-6 else 1e6
    half_sep = wheel_sep_m / 2.0

    # ratio by which the inner wheel is slower than the outer
    v_ratio = (r_turn - half_sep) / (r_turn + half_sep)   # v_slower / v_faster

    # Apply damping to correction factor
    # full_correction brings ratio to 1.0 in one step;
    # damped version applies CORRECTION_DAMP fraction of that.
    full_correction = 1.0 - v_ratio          # how much to scale down the faster motor
    damped_correction = full_correction * damp

    if yaw_drift_rad > 0:
        # Robot drifted LEFT → right is faster → reduce right trim
        new_trim_right = trim_right * (1.0 - damped_correction)
        new_trim_left  = trim_left
    else:
        # Robot drifted RIGHT → left is faster → reduce left trim
        new_trim_left  = trim_left * (1.0 - damped_correction)
        new_trim_right = trim_right

    # Normalise so max trim = 1.0 (avoid both trims < 1, which wastes resolution)
    max_trim = max(new_trim_left, new_trim_right)
    if max_trim > 0:
        new_trim_left  /= max_trim
        new_trim_right /= max_trim

    # Sanity clamp
    new_trim_left  = max(0.70, min(1.20, new_trim_left))
    new_trim_right = max(0.70, min(1.20, new_trim_right))

    return new_trim_left, new_trim_right


# ═══════════════════════════════════════════════════════════════════════════
# ROS node
# ═══════════════════════════════════════════════════════════════════════════

class Calibrator(Node):
    def __init__(self):
        super().__init__("straight_calibrator")
        self._pub   = self.create_publisher(Twist, "/cmd_vel", 10)
        self._odom  : Odometry | None  = None
        self._imu   : Imu | None       = None
        self._scan  : LaserScan | None = None
        self._scans_buf: list[LaserScan] = []
        self._emergency_stop = False
        self.scan_available  = False   # set True once a scan message is received
        self._suppress_approach_warn = False  # silences WALL_WARN_M spam during in-place spin
        self._suppress_safety       = False  # silences safety hard-stop during intentional in-place spin
        self._scan_diag_done        = False  # one-shot scan geometry diagnostic

        self.create_subscription(Odometry,  "/odom", self._odom_cb,  10)
        self.create_subscription(Imu,       "/imu",  self._imu_cb,   SENSOR_QOS)
        self.create_subscription(LaserScan, "/scan", self._scan_cb,  SENSOR_QOS)

        # 10 Hz safety monitor
        self.create_timer(0.10, self._safety_cb)

    # ── callbacks ──────────────────────────────────────────────────────────
    def _odom_cb(self, msg): self._odom = msg
    def _imu_cb (self, msg): self._imu  = msg

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg
        self._scan_available = True
        self.scan_available  = True
        self._scans_buf.append(msg)

    def _safety_cb(self):
        if self._scan is None:
            return
        n = len(self._scan.ranges)
        if n == 0:
            return

        # ── one-shot scan geometry diagnostic ──────────────────────────────
        if not self._scan_diag_done:
            self._scan_diag_done = True
            inc  = self._scan.angle_increment
            amin = self._scan.angle_min
            # Index of forward-facing beam (angle = 0)
            fwd_idx = round(-amin / inc) % n if abs(inc) > 1e-9 else 0
            info(f"[SCAN DIAG] n={n}  angle_min={math.degrees(amin):.1f} deg  "
                 f"angle_max={math.degrees(self._scan.angle_max):.1f} deg  "
                 f"increment={math.degrees(inc):.3f} deg  "
                 f"forward_idx={fwd_idx}  "
                 f"range[0]={self._scan.ranges[0]:.3f} m  "
                 f"range[fwd]={self._scan.ranges[fwd_idx]:.3f} m")

        # ── forward-facing window (centred on angle=0, not index 0) ─────────
        # angle_min is typically -pi, so index 0 points BACKWARD.
        # We compute the index corresponding to angle=0 (forward) and sample
        # a +-10 deg window around it.
        inc = self._scan.angle_increment
        if abs(inc) < 1e-9:
            return
        fwd_idx = round(-self._scan.angle_min / inc) % n
        win = max(1, int(math.radians(10) / abs(inc)))
        window_ranges = []
        for di in range(-win, win + 1):
            idx = (fwd_idx + di) % n
            r = self._scan.ranges[idx]
            if math.isfinite(r) and self._scan.range_min <= r <= self._scan.range_max:
                window_ranges.append(r)
        if not window_ranges:
            return
        min_front = min(window_ranges)

        # During intentional in-place rotation the robot cannot drive into
        # an obstacle, so suppress the hard stop (but still log if very close).
        if self._suppress_safety:
            return

        if min_front < WALL_STOP_M:
            if not self._emergency_stop:
                self._emergency_stop = True
                warn(f"SAFETY STOP — front obstacle at {min_front:.3f} m "
                     f"(limit {WALL_STOP_M:.2f} m).  Stopping motors.")
            self._send_vel(0.0, 0.0)
        elif min_front < WALL_WARN_M and not self._emergency_stop and not self._suppress_approach_warn:
            warn(f"Wall at {min_front:.3f} m -- approaching limit {WALL_STOP_M:.2f} m.")

    # ── utilities ──────────────────────────────────────────────────────────
    def _send_vel(self, lin, ang):
        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self._pub.publish(msg)

    def spin_for(self, secs: float):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_data(self, timeout=10.0) -> bool:
        """Wait for odom + imu (required).  Scan is optional but waited for
        up to `timeout`; after that we proceed without it."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._odom is not None and self._imu is not None:
                # Required topics are up.  Wait a couple more seconds for scan.
                if not self.scan_available:
                    remaining = timeout - (time.time() - t0)
                    scan_wait = min(remaining, 3.0)
                    t1 = time.time()
                    while time.time() - t1 < scan_wait:
                        rclpy.spin_once(self, timeout_sec=0.05)
                        if self.scan_available:
                            break
                if not self.scan_available:
                    warn("LiDAR /scan not available — wall-distance ground-truth "
                         "disabled.  Drift measured via IMU only.")
                return True
        return False

    @property
    def odom_xy(self):
        p = self._odom.pose.pose.position
        return p.x, p.y

    @property
    def odom_yaw(self):
        return quat_to_yaw(self._odom.pose.pose.orientation)

    @property
    def imu_yaw(self):
        return quat_to_yaw(self._imu.orientation)

    def snapshot_scans(self, duration=1.0) -> list[LaserScan]:
        """Collect scan messages for `duration` seconds."""
        self._scans_buf.clear()
        self.spin_for(duration)
        return list(self._scans_buf)

    # ── duty-cycle sweep helpers ─────────────────────────────────────────────
    def measure_motion_quality(self, cmd_speed: float,
                               duration: float = DUTY_SWEEP_SECS) -> dict | None:
        """Drive at cmd_speed m/s for duration s and return motion-quality metrics.

        Returned dict keys:
          cmd_speed   — commanded speed (m/s)
          mean_v      — mean odom linear.x (m/s)
          std_v       — standard deviation of odom linear.x
          cov_v       — coefficient of variation (std / |mean|); high = stuttery
          stall_frac  — fraction of samples where |v| < STALL_VEL_THRESH
          peak_jerk   — peak |dω_z/dt| rad/s² in first JERK_SAMPLE_SECS (startup kick)
          label       — 'STALL', 'STUTTER', 'JERK', or 'OK'
        """
        self._send_vel(0.0, 0.0)
        self.spin_for(0.30)          # let robot settle
        self._emergency_stop = False

        vel_samples: list[float] = []
        imu_window:  list[tuple[float, float]] = []  # (omega_z, timestamp)
        jerk_phase   = True
        t0           = time.time()
        last_sample  = t0

        while time.time() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.04)
            if self._emergency_stop:
                self._send_vel(0.0, 0.0)
                return None
            self._send_vel(cmd_speed, 0.0)
            now = time.time()
            if now - last_sample >= 0.05:
                if self._odom is not None:
                    vel_samples.append(self._odom.twist.twist.linear.x)
                if jerk_phase and self._imu is not None:
                    imu_window.append((self._imu.angular_velocity.z, now))
                if now - t0 > JERK_SAMPLE_SECS:
                    jerk_phase = False
                last_sample = now

        self._send_vel(0.0, 0.0)
        self.spin_for(0.30)

        if not vel_samples:
            return None

        mean_v = sum(vel_samples) / len(vel_samples)
        if len(vel_samples) > 1:
            std_v = math.sqrt(sum((v - mean_v) ** 2 for v in vel_samples)
                              / (len(vel_samples) - 1))
        else:
            std_v = 0.0
        cov_v      = std_v / max(abs(mean_v), 1e-6)
        stall_frac = sum(1 for v in vel_samples
                         if abs(v) < STALL_VEL_THRESH) / len(vel_samples)

        # Peak startup angular jerk  |dω/dt|
        peak_jerk = 0.0
        for i in range(1, len(imu_window)):
            dw = imu_window[i][0] - imu_window[i - 1][0]
            dt = imu_window[i][1] - imu_window[i - 1][1]
            if dt > 1e-4:
                peak_jerk = max(peak_jerk, abs(dw / dt))

        # Classify
        if stall_frac > 0.40 or abs(mean_v) < cmd_speed * 0.30:
            label = "STALL"
        elif cov_v > STUTTER_COV_THRESH:
            label = "STUTTER"
        elif peak_jerk > JERK_ALPHA_THRESH:
            label = "JERK"
        else:
            label = "OK"

        return {
            "cmd_speed":  cmd_speed,
            "mean_v":     mean_v,
            "std_v":      std_v,
            "cov_v":      cov_v,
            "stall_frac": stall_frac,
            "peak_jerk":  peak_jerk,
            "label":      label,
        }

    def run_duty_sweep(self, speeds: list | None = None) -> list[dict]:
        """Run measure_motion_quality at each speed.  Returns list of quality dicts."""
        if speeds is None:
            speeds = DUTY_SWEEP_SPEEDS
        results = []
        for v in speeds:
            info(f"  ▶ sweeping {v * 100:.0f} cm/s…")
            q = self.measure_motion_quality(v)
            if q is None:
                warn(f"  Speed {v * 100:.0f} cm/s aborted (safety stop).")
                continue
            col = GRN if q["label"] == "OK" else (YLW if q["label"] == "JERK" else RED)
            info(f"    mean={q['mean_v']*100:+.1f} cm/s  CoV={q['cov_v']:.2f}  "
                 f"stall={q['stall_frac']:.0%}  jerk={q['peak_jerk']:.1f} rad/s²  "
                 f"{col}[{q['label']}]{NC}")
            results.append(q)
        return results

    # ── yaw / return helpers ────────────────────────────────────────────────
    def _wall_pose(self, n_scans: int = RETURN_WALL_SCAN_N) -> tuple:
        """Collect n_scans LiDAR scans and return a robust median wall pose.

        Returns (wall_dist_m, wall_angle_rad), or (None, None) if the wall
        cannot be seen reliably (too few valid fits).

        This is the core of the limited scan-based localisation: instead of
        relying on odometry or IMU alone we fit the wall plane each time and
        average several independent measurements to reduce noise.
        """
        self._scans_buf.clear()
        deadline = time.time() + n_scans * 0.15   # ~10 Hz scans, budget 1.5× time
        while len(self._scans_buf) < n_scans and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        scans = list(self._scans_buf)
        if not scans:
            return None, None
        return median_wall(scans)

    def _wall_align(self, timeout: float = 18.0,
                    fallback_yaw: float | None = None) -> tuple:
        """Rotate in-place until the robot is perpendicular to the wall
        (wall_angle -> 0).

        Key sign convention (confirmed from run logs):
          angular.z POSITIVE = CCW (left) on this robot -- standard ROS.
          wall_angle NEGATIVE = wall normal tilted CW from +x = robot yawed
            left relative to wall = needs to turn RIGHT (negative az).
          BUT empirically the robot spins the WRONG direction for az=-0.15,
          so we negate the proportional term.

        Safety guards:
          - initial_dist gate  : discard fits > 40% shorter than first reading.
          - IMU rotation clamp : abort after +-65° total rotation.
          - Stale scan guard   : skip fit if scan stamp has not advanced.

        Returns (residual_wall_angle_rad, wall_dist_m).
        """
        t0            = time.time()
        settled_cnt   = 0
        wall_miss_t0  = None
        initial_dist  = None
        start_imu_yaw = self.imu_yaw
        IMU_ROT_CLAMP = math.radians(65)

        last_dist   = None
        last_angle  = None
        last_stamp  = None   # scan stamp freshness guard
        last_az     = 0.0    # hold last command across stale cycles
        scan_was_fresh = False  # set True each cycle a new stamp is seen
        cycle       = 0
        filtered_count = 0

        self._suppress_approach_warn = True
        self._suppress_safety        = True
        self._emergency_stop         = False

        def _imu_fallback_target():
            """Compute IMU target heading = current_yaw + wall_angle_error.
            wall_angle is the angle the robot must rotate (in its own frame)
            to become perpendicular, so target = imu_yaw + last_angle."""
            if last_angle is not None:
                # wall_angle = rotation needed in robot frame to face wall
                # target IMU yaw = current + that rotation
                target = self.imu_yaw + last_angle
                # normalise to (-pi, pi)
                target = math.atan2(math.sin(target), math.cos(target))
                info(f"  [align] IMU fallback: imu_yaw={math.degrees(self.imu_yaw):.1f} deg  "
                     f"last_wall_ang={math.degrees(last_angle):.1f} deg  "
                     f"-> target={math.degrees(target):.1f} deg")
                return target
            info(f"  [align] IMU fallback: no wall reading, "
                 f"fallback_yaw={math.degrees(fallback_yaw):.1f} deg"
                 if fallback_yaw is not None
                 else "  [align] IMU fallback: no reading and no fallback_yaw")
            return fallback_yaw

        try:
            while time.time() - t0 < timeout:
                rclpy.spin_once(self, timeout_sec=0.05)
                omega = self._imu.angular_velocity.z if self._imu is not None else 0.0
                cycle += 1

                # IMU rotation clamp
                yaw_delta = abs(angle_diff(self.imu_yaw, start_imu_yaw))
                if yaw_delta > IMU_ROT_CLAMP:
                    warn(f"  Wall align: IMU clamp ({math.degrees(yaw_delta):.1f} deg "
                         f"> {math.degrees(IMU_ROT_CLAMP):.0f} deg)"
                         " -- finishing with IMU fallback.")
                    imu_target = _imu_fallback_target()
                    if imu_target is not None:
                        self._send_vel(0.0, 0.0)
                        self.spin_for(0.2)
                        residual_imu = self._yaw_align(imu_target)
                        return residual_imu, last_dist
                    break

                # Fresh wall-plane estimate -- skip if scan hasn't updated
                raw_dist, raw_ang = None, None
                scan_was_fresh = False
                if self._scan is not None:
                    cur_stamp = (self._scan.header.stamp.sec,
                                 self._scan.header.stamp.nanosec)
                    if cur_stamp != last_stamp:
                        last_stamp = cur_stamp
                        scan_was_fresh = True
                        raw_dist, raw_ang = fit_wall(self._scan)
                    # else: stale scan, treat as no reading this cycle

                # initial_dist gate
                w_dist, w_ang = raw_dist, raw_ang
                gate_msg = ""
                if w_dist is not None:
                    if initial_dist is None:
                        initial_dist = w_dist
                        gate_msg = f"(initial_dist={w_dist:.3f} m)"
                    elif w_dist < initial_dist * 0.40:
                        filtered_count += 1
                        gate_msg = f"(FILTERED {w_dist:.3f}<40%of{initial_dist:.3f})"
                        w_dist, w_ang = None, None

                # Per-cycle diagnostic (every 4th cycle)
                if cycle % 4 == 1:
                    pts_in_window = 0
                    if self._scan is not None:
                        sc = self._scan
                        win_r = math.radians(SCAN_WINDOW_DEG)
                        for i in range(len(sc.ranges)):
                            a = sc.angle_min + i * sc.angle_increment
                            a = math.atan2(math.sin(a), math.cos(a))
                            if abs(a) <= win_r and math.isfinite(sc.ranges[i]):
                                pts_in_window += 1
                    az_raw  = (-RETURN_WALL_ANG_KP * (w_ang or 0.0)
                               + RETURN_WALL_ANG_KD * omega)
                    az_clip = max(-RETURN_WALL_ANG_MAX,
                                  min(RETURN_WALL_ANG_MAX, az_raw))
                    info(f"  [align c{cycle:03d}] "
                         f"raw=({raw_dist and f'{raw_dist:.3f}m' or 'None'}, "
                         f"{raw_ang and f'{math.degrees(raw_ang):+.1f}d' or 'None'})  "
                         f"filt={filtered_count}  pts={pts_in_window}  "
                         f"omega={omega:+.3f}  "
                         f"yaw={math.degrees(self.imu_yaw):+.1f}d  "
                         f"delta={math.degrees(yaw_delta):.1f}d  "
                         f"AZ_raw={az_raw:.3f} clip={az_clip:.3f}  held={last_az:.3f}  "
                         f"{'fresh' if scan_was_fresh else 'STALE'}  {gate_msg}")

                if w_ang is None:
                    if wall_miss_t0 is None:
                        wall_miss_t0 = time.time()
                    elif time.time() - wall_miss_t0 > 1.0:
                        warn("  Wall lost >1 s -- falling back to IMU yaw align.")
                        imu_target = _imu_fallback_target()
                        if imu_target is not None:
                            self._send_vel(0.0, 0.0)
                            self.spin_for(0.2)
                            residual_imu = self._yaw_align(imu_target)
                            return residual_imu, last_dist
                        break
                    # Hold last commanded az rather than stopping; stopping
                    # causes the robot to never build enough torque to overcome
                    # static friction when the wall is only visible ~1 in 12
                    # cycles (LiDAR 10 Hz vs control loop ~100 Hz).
                    self._send_vel(0.0, last_az)
                    continue

                wall_miss_t0 = None
                last_dist    = w_dist
                last_angle   = w_ang

                # PD on wall_angle (target = 0 = perpendicular).
                # Sign: wall_angle < 0 means robot yawed left; rotate LEFT
                # (positive az / CCW) to reduce the error.  Empirically the
                # direction that reduces |wall_angle| is opposite to the naive
                # KP*w_ang sign so we negate the proportional term.
                az = -RETURN_WALL_ANG_KP * w_ang + RETURN_WALL_ANG_KD * omega
                az = max(-RETURN_WALL_ANG_MAX, min(RETURN_WALL_ANG_MAX, az))
                last_az = az   # hold for stale/None cycles

                if abs(w_ang) < RETURN_WALL_ANG_TOL and abs(omega) < 0.03:
                    settled_cnt += 1
                    if settled_cnt >= RETURN_WALL_ANG_SETTLED:
                        break
                    self._send_vel(0.0, 0.0)
                else:
                    settled_cnt = 0
                    self._send_vel(0.0, az)
        finally:
            self._suppress_approach_warn = False
            self._suppress_safety        = False

        self._send_vel(0.0, 0.0)
        self.spin_for(RETURN_SETTLE_SEC)
        residual = last_angle if last_angle is not None else 0.0
        return residual, last_dist


    def _yaw_align(self, target_yaw: float, timeout: float = 10.0) -> float:
        """Rotate in-place to target_yaw using a PD controller.

        The derivative term uses IMU angular_velocity.z **directly** so there
        is no numerical differentiation lag.  The controller exits only after
        RETURN_YAW_SETTLED consecutive 50 ms cycles that are all within
        RETURN_YAW_TOL, preventing premature exit on transient noise.

        Returns residual yaw error (rad) after settling.
        """
        t0          = time.time()
        settled_cnt = 0

        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)

            err   = angle_diff(target_yaw, self.imu_yaw)
            omega = self._imu.angular_velocity.z if self._imu is not None else 0.0

            # PD: proportional on yaw error, derivative damps current spin rate
            az = RETURN_YAW_KP * err - RETURN_YAW_KD * omega
            az = max(-RETURN_YAW_MAX, min(RETURN_YAW_MAX, az))

            if abs(err) < RETURN_YAW_TOL and abs(omega) < 0.04:
                settled_cnt += 1
                if settled_cnt >= RETURN_YAW_SETTLED:
                    break
                # Keep sending zero so motors actually stop
                self._send_vel(0.0, 0.0)
            else:
                settled_cnt = 0
                self._send_vel(0.0, az)

        self._send_vel(0.0, 0.0)
        self.spin_for(RETURN_SETTLE_SEC)
        return angle_diff(target_yaw, self.imu_yaw)

    def return_to_start(self, target_dist: float, speed: float,
                        pre_wall_dist: float | None, start_yaw: float,
                        use_lidar: bool = True) -> bool:
        """
        Return the robot to its calibration start position using closed-loop
        wall-relative localisation at every step.

        Strategy (limited SLAM):
          The wall in front of the robot is the only stationary, planar
          feature in the scene.  We use it as a 2-DOF absolute reference:
            • wall_angle = 0  ⇒ robot is perpendicular (heading reference)
            • wall_dist = pre_wall_dist  ⇒ robot is at start distance (range ref)
          Both quantities are re-estimated from raw LiDAR scans each control
          cycle, so drift in IMU heading integration cannot accumulate.
          IMU yaw is kept as a fallback when the wall is temporarily invisible.

        Steps:
          1. Wall squaring  — PD controller drives wall_angle → 0.
          2. Reverse drive  — PD heading hold on wall_angle while backing up;
                              stop when wall_dist ≥ pre_wall_dist (LiDAR) or
                              odom distance covered (fallback).
          3. Final squaring — second _wall_align pass to remove residual angle
                              error accumulated during the reverse.
        """
        if not use_lidar:
            info("Returning to start (IMU + odometry, LiDAR ignored)…")
            self._emergency_stop = False

            # 1) Align heading back to the run-start yaw.
            info("  [1/3] IMU yaw alignment to start heading…")
            self._yaw_align(start_yaw)

            # 2) Reverse by odometry distance while holding heading from IMU.
            info(f"  [2/3] Reversing {target_dist*100:.0f} cm (odom target)…")
            x0, y0 = self.odom_xy
            t0 = time.time()
            max_t = target_dist / speed * 3.5 + 3.0
            stop_reason = "timeout"

            while time.time() - t0 < max_t:
                rclpy.spin_once(self, timeout_sec=0.05)
                omega = self._imu.angular_velocity.z if self._imu is not None else 0.0
                yaw_err = angle_diff(start_yaw, self.imu_yaw)
                az = RETURN_LAT_KP * yaw_err - RETURN_LAT_KD * omega
                az = max(-RETURN_LAT_MAX, min(RETURN_LAT_MAX, az))

                dx = self._odom.pose.pose.position.x - x0
                dy = self._odom.pose.pose.position.y - y0
                if math.sqrt(dx*dx + dy*dy) >= target_dist:
                    stop_reason = "odom distance"
                    break

                self._send_vel(-speed, az)

            self._send_vel(0.0, 0.0)
            info(f"        Stopped: {stop_reason}")
            self.spin_for(RETURN_SETTLE_SEC)

            # 3) Final heading touch-up.
            info("  [3/3] Final IMU yaw alignment…")
            residual = self._yaw_align(start_yaw)
            ok(f"  Returned to start. Residual angle={math.degrees(residual):+.1f}°")
            return True

        info("Returning to start (wall-relative localisation)…")
        self._emergency_stop = False

        # ── 1. Wall squaring ──────────────────────────────────────────────
        info("  [1/3] Wall squaring (LiDAR-based heading localisation)…")
        res_ang, cur_dist = self._wall_align(fallback_yaw=start_yaw)
        if cur_dist is not None:
            info(f"        Residual wall angle : {math.degrees(res_ang):+.1f}°  "
                 f"wall dist : {cur_dist:.3f} m")
        else:
            info(f"        Residual wall angle : {math.degrees(res_ang):+.1f}°")

        # Sanity-check: if the squaring step produced a large residual the fit
        # was unreliable (e.g. side-wall grab before the clamp fired).  Fall
        # back to IMU alignment so the reverse leg is at least roughly correct.
        if abs(res_ang) > math.radians(20):
            warn(f"  Wall squaring residual {math.degrees(res_ang):.1f}° > 20° — "
                 "re-aligning with IMU fallback.")
            self._yaw_align(start_yaw)

        # ── 2. Reverse drive ──────────────────────────────────────────────
        if pre_wall_dist is not None:
            info(f"  [2/3] Reversing toward start "
                 f"(target wall dist {pre_wall_dist:.3f} m)…")
        else:
            info(f"  [2/3] Reversing {target_dist*100:.0f} cm (odom fallback)…")

        x0, y0      = self.odom_xy
        t0          = time.time()
        max_t       = target_dist / speed * 3.5 + 3.0
        stop_reason = "timeout"

        while time.time() - t0 < max_t:
            rclpy.spin_once(self, timeout_sec=0.05)
            omega = self._imu.angular_velocity.z if self._imu is not None else 0.0

            # Primary distance stop: LiDAR wall distance
            if pre_wall_dist is not None and self._scan is not None:
                w_dist, w_ang = fit_wall(self._scan)
                if w_dist is not None:
                    if w_dist >= pre_wall_dist - RETURN_WALL_TOL:
                        stop_reason = (f"wall dist {w_dist:.3f} m ≥ "
                                       f"target {pre_wall_dist:.3f} m")
                        break
                    # Heading hold: PD on wall_angle (absolute, no IMU drift)
                    az = RETURN_LAT_KP * w_ang - RETURN_LAT_KD * omega
                    az = max(-RETURN_LAT_MAX, min(RETURN_LAT_MAX, az))
                else:
                    # Wall lost — fall back to IMU heading hold
                    yaw_err = angle_diff(start_yaw, self.imu_yaw)
                    az = RETURN_LAT_KP * yaw_err - RETURN_LAT_KD * omega
                    az = max(-RETURN_LAT_MAX, min(RETURN_LAT_MAX, az))
            else:
                yaw_err = angle_diff(start_yaw, self.imu_yaw)
                az = RETURN_LAT_KP * yaw_err - RETURN_LAT_KD * omega
                az = max(-RETURN_LAT_MAX, min(RETURN_LAT_MAX, az))

            # Odometry fallback stop
            dx = self._odom.pose.pose.position.x - x0
            dy = self._odom.pose.pose.position.y - y0
            if math.sqrt(dx*dx + dy*dy) >= target_dist:
                stop_reason = "odom distance"
                break

            self._send_vel(-speed, az)

        self._send_vel(0.0, 0.0)
        info(f"        Stopped: {stop_reason}")
        self.spin_for(RETURN_SETTLE_SEC)

        # ── 3. Final wall squaring ────────────────────────────────────────
        info("  [3/3] Final wall squaring…")
        res_ang, final_dist = self._wall_align(fallback_yaw=start_yaw)

        # ── Report ────────────────────────────────────────────────────────
        if final_dist is not None and pre_wall_dist is not None:
            d_err = (final_dist - pre_wall_dist) * 1000
            ok(f"  Returned: wall={final_dist:.3f} m (Δ{d_err:+.0f} mm), "
               f"angle={math.degrees(res_ang):+.1f}°")
        else:
            ok(f"  Returned to start. Residual angle={math.degrees(res_ang):+.1f}°")
        return True

    # ── core test pass ──────────────────────────────────────────────────────
    def run_pass(self, target_dist: float, speed: float,
                 use_lidar: bool = True) -> dict | None:
        """
        Drive forward `target_dist` metres at `speed` m/s while monitoring
        safety.  Returns a measurement dict or None on emergency stop.
        """
        self._emergency_stop = False

        # --- Pre-run wall snapshot (optional) ---
        if use_lidar:
            info("Measuring wall state before run…")
            pre_scans = self.snapshot_scans(SETTLE_SEC)
            pre_wall_dist, pre_wall_angle = median_wall(pre_scans)

            if pre_wall_dist is None:
                warn("Could not fit wall plane before run (not enough scan pts).")
            else:
                info(f"  Wall: dist={pre_wall_dist:.4f} m  "
                     f"angle={math.degrees(pre_wall_angle):+.2f}°")
                if pre_wall_dist < WALL_STOP_M + target_dist + 0.15:
                    warn(f"Wall too close ({pre_wall_dist:.3f} m) to safely drive "
                         f"{target_dist*100:.0f} cm.")
                    warn("  >>> Please move the robot further from the wall, "
                         "then press Enter to retry, or Ctrl-C to abort. <<<")
                    try:
                        input()
                    except EOFError:
                        pass
                    # Re-measure after user repositions
                    pre_scans = self.snapshot_scans(SETTLE_SEC)
                    pre_wall_dist, pre_wall_angle = median_wall(pre_scans)
                    if pre_wall_dist is None:
                        fail("Still cannot fit wall after repositioning.")
                        return None
                    info(f"  Wall after reposition: dist={pre_wall_dist:.4f} m  "
                         f"angle={math.degrees(pre_wall_angle):+.2f}°")
                    if pre_wall_dist < WALL_STOP_M + target_dist + 0.15:
                        fail(f"Wall still too close ({pre_wall_dist:.3f} m).  Aborting.")
                        return None
            # Warn if robot is too far off-perpendicular: the SVD fit will be
            # unreliable, producing bogus distance and drift readings.
            if pre_wall_angle is not None and \
                    abs(pre_wall_angle) > math.radians(WALL_ANGLE_TRUST_DEG):
                warn(f"Robot is {math.degrees(pre_wall_angle):+.1f}\u00b0 off-perpendicular "
                     f"(trust limit \u00b1{WALL_ANGLE_TRUST_DEG:.0f}\u00b0). "
                     "LiDAR distance and yaw-drift will be UNRELIABLE for this pass. "
                     "Square the robot to face the wall for accurate results.")
        else:
            info("IMU-only mode: skipping LiDAR wall snapshots and distance fitting.")
            pre_wall_dist, pre_wall_angle = None, None

        # Wall metrics are reliable only when robot started near-perpendicular
        wall_metrics_valid = (
            pre_wall_angle is not None and
            abs(pre_wall_angle) <= math.radians(WALL_ANGLE_TRUST_DEG)
        )

        # --- Baseline pose ---
        x0, y0  = self.odom_xy
        yaw0_o  = self.odom_yaw
        yaw0_i  = self.imu_yaw
        self._start_yaw = yaw0_i    # stored so return_to_start() can reference it

        # --- Drive ---
        info(f"Driving {target_dist*100:.0f} cm at {speed*100:.1f} cm/s…")
        dt = 1.0 / RATE_HZ
        t0 = time.time()
        max_t = target_dist / speed * 2.5 + 3.0

        while True:
            if self._emergency_stop:
                return None                               # safety stop
            dx = self._odom.pose.pose.position.x - x0
            dy = self._odom.pose.pose.position.y - y0
            dist_done = math.sqrt(dx * dx + dy * dy)
            if dist_done >= target_dist:
                break
            if time.time() - t0 > max_t:
                warn(f"Timed out — only {dist_done*100:.1f} cm covered.")
                break
            self._send_vel(speed, 0.0)
            rclpy.spin_once(self, timeout_sec=dt)

        # --- Stop ---
        self._send_vel(0.0, 0.0)
        info("Settling…")
        self.spin_for(SETTLE_SEC)

        # --- Post-run wall snapshot (optional) ---
        if use_lidar:
            post_scans = self.snapshot_scans(SETTLE_SEC)
            post_wall_dist, post_wall_angle = median_wall(post_scans)
        else:
            post_wall_dist, post_wall_angle = None, None

        # --- Final pose ---
        x1, y1  = self.odom_xy
        yaw1_o  = self.odom_yaw
        yaw1_i  = self.imu_yaw

        # Odometry distance (in robot initial-heading frame)
        dx_o = x1 - x0
        dy_o = y1 - y0
        cos_h = math.cos(-yaw0_o)
        sin_h = math.sin(-yaw0_o)
        odom_fwd  = dx_o * cos_h - dy_o * sin_h
        odom_lat  = dx_o * sin_h + dy_o * cos_h
        odom_dist = math.sqrt(dx_o**2 + dy_o**2)

        # LiDAR ground-truth distance (change in perpendicular wall distance)
        lidar_dist = None
        if pre_wall_dist is not None and post_wall_dist is not None:
            lidar_dist = pre_wall_dist - post_wall_dist  # positive = moved toward wall

        # Yaw drift (use average of IMU and odom, prefer IMU)
        imu_drift  = angle_diff(yaw1_i, yaw0_i)
        odom_drift = angle_diff(yaw1_o, yaw0_o)
        # wall-angle change is the most direct measurement
        wall_drift = None
        if pre_wall_angle is not None and post_wall_angle is not None:
            wall_drift = angle_diff(post_wall_angle, pre_wall_angle)
            # +wall_drift means normal rotated CCW = robot rotated CW = drifted right
            # flip sign to get robot yaw drift convention
            wall_drift = -wall_drift

        return {
            "odom_dist":  odom_dist,
            "odom_fwd":   odom_fwd,
            "odom_lat":   odom_lat,
            "odom_drift": odom_drift,
            "imu_drift":  imu_drift,
            "wall_drift": wall_drift,
            "lidar_dist": lidar_dist,
            "pre_wall_dist":   pre_wall_dist,
            "post_wall_dist":  post_wall_dist,
            "pre_wall_angle":  pre_wall_angle,
            "post_wall_angle": post_wall_angle,
            "wall_metrics_valid": wall_metrics_valid,
            "start_yaw":       yaw0_i,   # IMU heading at run start (for auto-return)
        }


# ═══════════════════════════════════════════════════════════════════════════
# Analysis & reporting
# ═══════════════════════════════════════════════════════════════════════════

def print_pass(i, r, target_dist):
    hdr(f"Pass {i + 1} results")
    info(f"Odometry fwd      : {r['odom_fwd']*100:+.2f} cm  "
         f"(target {target_dist*100:.0f} cm, "
         f"error {(r['odom_fwd']-target_dist)*100:+.2f} cm)")
    info(f"Odometry lateral  : {r['odom_lat']*100:+.2f} cm "
         f"({'LEFT' if r['odom_lat'] > 0 else 'RIGHT'})")
    if r["lidar_dist"] is not None:
        info(f"LiDAR actual dist : {r['lidar_dist']*100:+.2f} cm  "
             f"(Δ odom-lidar: {(r['odom_dist']-r['lidar_dist'])*100:+.2f} cm)")
    info(f"IMU yaw drift     : {math.degrees(r['imu_drift']):+.2f}°")
    info(f"Odom yaw drift    : {math.degrees(r['odom_drift']):+.2f}°")
    if r["wall_drift"] is not None:
        info(f"LiDAR wall drift  : {math.degrees(r['wall_drift']):+.2f}° "
             "(most accurate)")


def aggregate(results):
    """Return averaged measurements from a list of pass result dicts."""
    def avg(key):
        vals = [r[key] for r in results if r.get(key) is not None]
        return (sum(vals) / len(vals)) if vals else None

    agg = {k: avg(k) for k in results[0].keys() if k != "wall_metrics_valid"}
    # Wall metrics are only trusted when ALL passes had a near-perpendicular start
    agg["wall_metrics_valid"] = all(r.get("wall_metrics_valid", False) for r in results)
    return agg


def best_yaw_drift(r):
    """Pick the most reliable yaw-drift estimate (LiDAR > IMU > odom).

    LiDAR wall drift is only used when the robot was near-perpendicular
    at the start of the pass (wall_metrics_valid=True).  Off-axis, the SVD
    fit is ill-conditioned and produces spuriously large drift angles.
    """
    if r["wall_drift"] is not None and r.get("wall_metrics_valid", False):
        return r["wall_drift"], "LiDAR wall"
    if r["imu_drift"] is not None:
        return r["imu_drift"], "IMU"
    return r["odom_drift"], "odometry"

def analyse_duty_sweep(sweep: list[dict],
                       cur_min_duty: float,
                       cur_kick_duty: float) -> tuple[float, float, str]:
    """Analyse a motion-quality sweep and recommend new MOTOR_MIN_DUTY /
    MOTOR_KICK_DUTY values.

    Returns (new_min_duty, new_kick_duty, diagnosis_string).
    new_* values equal the current values if no change is needed.
    """
    if not sweep:
        return cur_min_duty, cur_kick_duty, "no sweep data"

    stall_qs   = [q for q in sweep if q["label"] == "STALL"]
    stutter_qs = [q for q in sweep if q["label"] == "STUTTER"]
    jerk_qs    = [q for q in sweep if q["label"] == "JERK"]
    ok_qs      = [q for q in sweep if q["label"] == "OK"]
    max_jerk   = max(q["peak_jerk"] for q in sweep)
    diagnoses: list[str] = []

    new_min_duty  = cur_min_duty
    new_kick_duty = cur_kick_duty

    # ── Min-duty diagnosis ────────────────────────────────────────────────
    if stall_qs:
        # Hard stall: motor not moving at all → MIN_DUTY far below friction threshold.
        # Bump by 0.05 per stall speed + 0.03 safety margin.
        bump = 0.05 * len(stall_qs) + 0.03
        new_min_duty = min(DUTY_MAX_BOUND, cur_min_duty + bump)
        stall_speeds = [f"{q['cmd_speed']*100:.0f}cm/s" for q in stall_qs]
        diagnoses.append(
            f"STALL at {stall_speeds} → "
            f"MIN_DUTY {cur_min_duty:.2f}→{new_min_duty:.2f}")
    elif stutter_qs:
        # Motor barely clearing friction: intermittent movement.
        new_min_duty = min(DUTY_MAX_BOUND, cur_min_duty + 0.03)
        stutter_speeds = [f"{q['cmd_speed']*100:.0f}cm/s" for q in stutter_qs]
        diagnoses.append(
            f"STUTTER at {stutter_speeds} → "
            f"MIN_DUTY {cur_min_duty:.2f}→{new_min_duty:.2f}")
    elif len(ok_qs) == len(sweep) and max_jerk < JERK_ALPHA_THRESH * 0.5:
        # All speeds are smooth and low-jerk → MIN_DUTY may be higher than needed.
        # Reduce cautiously (single step); user can iterate.
        candidate = max(DUTY_MIN_BOUND, cur_min_duty - 0.03)
        if cur_min_duty - candidate >= DUTY_CHANGE_MIN:
            new_min_duty = candidate
            diagnoses.append(
                f"All OK + low jerk → trying MIN_DUTY reduction "
                f"{cur_min_duty:.2f}→{new_min_duty:.2f}")
        else:
            diagnoses.append("Motion quality optimal — no MIN_DUTY change needed")
    else:
        diagnoses.append("Motion quality acceptable")

    # ── Kick-duty diagnosis ────────────────────────────────────────────────
    if max_jerk > JERK_ALPHA_THRESH:
        # Excessive angular jerk at startup: kick pulse is too aggressive.
        new_kick_duty = max(new_min_duty + 0.02,
                            min(DUTY_MAX_BOUND, cur_kick_duty - 0.05))
        diagnoses.append(
            f"JERK {max_jerk:.1f} rad/s² > {JERK_ALPHA_THRESH:.1f} → "
            f"KICK_DUTY {cur_kick_duty:.2f}→{new_kick_duty:.2f}")
    elif stall_qs and cur_kick_duty < new_min_duty + 0.05:
        # If we just raised MIN_DUTY, make sure KICK still leads MIN.
        new_kick_duty = min(DUTY_MAX_BOUND, new_min_duty + 0.05)
        diagnoses.append(
            f"Raising KICK_DUTY to stay above new MIN_DUTY: "
            f"{cur_kick_duty:.2f}→{new_kick_duty:.2f}")

    return new_min_duty, new_kick_duty, " | ".join(diagnoses)

# ═══════════════════════════════════════════════════════════════════════════
# File updates
# ═══════════════════════════════════════════════════════════════════════════

def apply_corrections(new_max_speed, new_trim_l, new_trim_r,
                      old_max_speed, old_trim_l, old_trim_r,
                      new_min_duty=None, old_min_duty=None,
                      new_kick_duty=None, old_kick_duty=None):
    hdr("Applying corrections to source files")

    info(f"MAX_WHEEL_SPEED_MS : {old_max_speed:.6f} → {new_max_speed:.6f} m/s")
    write_firmware_float(MAIN_C, "MAX_WHEEL_SPEED_MS", new_max_speed,
                         f"calibrated — straight_calibrate.py")
    ok("MAX_WHEEL_SPEED_MS updated in firmware/main.c")

    info(f"MOTOR_TRIM_LEFT  : {old_trim_l:.6f} → {new_trim_l:.6f}")
    write_firmware_float(MAIN_C, "MOTOR_TRIM_LEFT", new_trim_l,
                         f"calibrated — straight_calibrate.py")
    ok("MOTOR_TRIM_LEFT updated")

    info(f"MOTOR_TRIM_RIGHT : {old_trim_r:.6f} → {new_trim_r:.6f}")
    write_firmware_float(MAIN_C, "MOTOR_TRIM_RIGHT", new_trim_r,
                         f"calibrated — straight_calibrate.py")
    ok("MOTOR_TRIM_RIGHT updated")

    if new_min_duty is not None and old_min_duty is not None and \
            abs(new_min_duty - old_min_duty) >= DUTY_CHANGE_MIN:
        info(f"MOTOR_MIN_DUTY   : {old_min_duty:.4f} → {new_min_duty:.4f}")
        write_firmware_float(MAIN_C, "MOTOR_MIN_DUTY", new_min_duty,
                             f"calibrated — straight_calibrate.py")
        ok("MOTOR_MIN_DUTY updated")

    if new_kick_duty is not None and old_kick_duty is not None and \
            abs(new_kick_duty - old_kick_duty) >= DUTY_CHANGE_MIN:
        info(f"MOTOR_KICK_DUTY  : {old_kick_duty:.4f} → {new_kick_duty:.4f}")
        write_firmware_float(MAIN_C, "MOTOR_KICK_DUTY", new_kick_duty,
                             f"calibrated — straight_calibrate.py")
        ok("MOTOR_KICK_DUTY updated")


def rebuild_flash():
    hdr("Rebuilding and flashing firmware")
    build_script = os.path.join(FIRMWARE_DIR, "build.sh")
    result = subprocess.run(["bash", build_script, "flash"],
                            cwd=FIRMWARE_DIR, timeout=300)
    if result.returncode != 0:
        fail(f"Build/flash failed (exit {result.returncode}).")
        return False
    ok("Firmware rebuilt and flashed successfully.")
    return True


def restart_service(settle_secs: float = 8.0):
    """Restart the turtlebot3 systemd service and wait for it to stabilise.
    Falls back gracefully if systemctl is unavailable or the service is unknown."""
    info("Restarting turtlebot3 service…")
    result = subprocess.run(
        ["sudo", "systemctl", "restart", "turtlebot3"],
        timeout=30, capture_output=True, text=True)
    if result.returncode == 0:
        info(f"Service restarted — waiting {settle_secs:.0f} s for topics to come up…")
        time.sleep(settle_secs)
        ok("Service restart complete.")
    else:
        warn(f"systemctl restart failed (rc={result.returncode}): {result.stderr.strip()}")
        warn("Continuing without service restart — topics may be stale.")


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description="Straight-line motor calibration")
    ap.add_argument("--distance",    type=float, default=0.20,
                    help="Travel per pass in metres (default 0.20)")
    ap.add_argument("--speed",       type=float, default=0.05,
                    help="Forward speed m/s (default 0.05)")
    ap.add_argument("--passes",      type=int,   default=3,
                    help="Calibration passes to average (default 3)")
    ap.add_argument("--manual-start", action="store_true",
                    help="Manually place robot before every pass (disable auto-return)")
    ap.add_argument("--imu-only", action="store_true",
                    help="Calibrate straightness using IMU/odometry only (ignore LiDAR metrics)")
    ap.add_argument("--no-flash",    action="store_true",
                    help="Update files but skip rebuild/flash")
    ap.add_argument("--verify-only", action="store_true",
                    help="One measurement pass, print results, no changes")
    ap.add_argument("--duty-cal",    action="store_true",
                    help="Run stutter/stall sweep to calibrate MOTOR_MIN_DUTY and "
                         "MOTOR_KICK_DUTY before the main calibration passes")
    args = ap.parse_args()

    print(f"\n{BLD}{'═'*60}")
    print("  TurtleBot3 Pico — Straight-Line Calibration")
    print(f"{'═'*60}{NC}")
    print(f"  Travel per pass : {args.distance*100:.0f} cm  at  "
          f"{args.speed*100:.1f} cm/s")
    print(f"  Averaging       : {args.passes} passes")
    print(f"  Start mode      : {'manual every pass' if args.manual_start else 'manual first + auto-return'}")
    print(f"  Drift source    : {'IMU only (LiDAR ignored)' if args.imu_only else 'LiDAR+IMU'}")
    print(f"  Safety stop     : front < {WALL_STOP_M} m")
    if args.imu_only:
        print(f"\n{YLW}IMU-only mode: LiDAR is not used for calibration math.{NC}")
        print(f"{YLW}Place the robot in open space with clear forward safety margin and press Enter to start.{NC}")
    else:
        print(f"\n{YLW}Place the robot perpendicular to a flat wall, at")
        print(f"least {WALL_STOP_M + args.distance + 0.20:.2f} m away, and press Enter to start.{NC}")
    try:
        input()
    except EOFError:
        pass

    rclpy.init()
    node = Calibrator()

    info("Waiting for /odom, /imu, /scan …")
    if not node.wait_for_data(12.0):
        fail("No data received.  Is bringup running?")
        rclpy.shutdown(); sys.exit(1)
    ok("Sensors ready.")

    # Read current firmware values
    cur_max_speed = read_firmware_float(MAIN_C, "MAX_WHEEL_SPEED_MS")
    cur_trim_l    = read_firmware_float(MAIN_C, "MOTOR_TRIM_LEFT")
    cur_trim_r    = read_firmware_float(MAIN_C, "MOTOR_TRIM_RIGHT")
    wheel_sep     = read_firmware_float(MAIN_C, "WHEEL_SEPARATION")
    cur_min_duty  = read_firmware_float(MAIN_C, "MOTOR_MIN_DUTY")
    cur_kick_duty = read_firmware_float(MAIN_C, "MOTOR_KICK_DUTY")

    if None in (cur_max_speed, cur_trim_l, cur_trim_r, wheel_sep):
        fail("Could not read current firmware parameters from main.c")
        rclpy.shutdown(); sys.exit(1)

    info(f"Current MAX_WHEEL_SPEED_MS = {cur_max_speed:.6f} m/s")
    info(f"Current MOTOR_TRIM_LEFT    = {cur_trim_l:.6f}")
    info(f"Current MOTOR_TRIM_RIGHT   = {cur_trim_r:.6f}")
    info(f"Current WHEEL_SEPARATION   = {wheel_sep:.6f} m")
    if cur_min_duty is not None:
        info(f"Current MOTOR_MIN_DUTY     = {cur_min_duty:.4f}")
    if cur_kick_duty is not None:
        info(f"Current MOTOR_KICK_DUTY  = {cur_kick_duty:.4f}")

    # ── Optional duty-cycle calibration phase ───────────────────────────────────
    if args.duty_cal and cur_min_duty is not None and cur_kick_duty is not None:
        hdr("Duty-cycle calibration — stutter / stall sweep")
        info("Driving at slow speeds to detect stall and jerkiness.")
        info("(The robot will drive forward a short distance at each step —"
             " make sure there is ≥30 cm of clearance ahead.)")

        sweep = node.run_duty_sweep()
        if sweep:
            new_min, new_kick, diag = analyse_duty_sweep(
                sweep, cur_min_duty, cur_kick_duty)
            hdr("Duty-cycle analysis")
            info(f"Diagnosis : {diag}")
            info(f"MOTOR_MIN_DUTY  : {cur_min_duty:.4f} → {new_min:.4f}")
            info(f"MOTOR_KICK_DUTY : {cur_kick_duty:.4f} → {new_kick:.4f}")

            duty_changed = (abs(new_min  - cur_min_duty)  >= DUTY_CHANGE_MIN or
                            abs(new_kick - cur_kick_duty) >= DUTY_CHANGE_MIN)
            if duty_changed and not args.no_flash:
                # Write and reflash so subsequent passes use the corrected duty.
                if abs(new_min - cur_min_duty) >= DUTY_CHANGE_MIN:
                    write_firmware_float(MAIN_C, "MOTOR_MIN_DUTY", new_min,
                                         "calibrated — straight_calibrate.py")
                if abs(new_kick - cur_kick_duty) >= DUTY_CHANGE_MIN:
                    write_firmware_float(MAIN_C, "MOTOR_KICK_DUTY", new_kick,
                                         "calibrated — straight_calibrate.py")
                ok("Duty-cycle values written to firmware/main.c")
                if not rebuild_flash():
                    fail("Duty reflash failed — continuing with old firmware.")
                else:
                    info("Waiting 5 s for device to re-enumerate…")
                    time.sleep(5.0)
                    restart_service()
                    node.destroy_node()
                    node = Calibrator()
                    info("Waiting for topics after duty reflash…")
                    if not node.wait_for_data(20.0):
                        fail("Sensors not available after reflash.")
                        rclpy.shutdown(); sys.exit(1)
                    # Refresh duty values from what was just written
                    cur_min_duty  = new_min
                    cur_kick_duty = new_kick
                    ok("Ready — duty-cycle calibration applied.")
            elif duty_changed and args.no_flash:
                warn("Duty changes computed but --no-flash set; values NOT written.")
            else:
                ok("No duty-cycle changes required.")
        else:
            warn("Duty sweep returned no data; skipping duty calibration.")

    n_passes = 1 if args.verify_only else args.passes

    hdr(f"Running {'verification' if args.verify_only else 'calibration'} "
        f"({'1' if args.verify_only else str(n_passes)} pass{'es' if n_passes>1 else ''})")

    results = []
    for i in range(n_passes):
        if i == 0 or args.manual_start:
            # Manual mode: operator positions before every pass.
            # Default mode: only first pass is manual; subsequent passes auto-return.
            info(f"\nPass {i+1}/{n_passes} — press Enter when robot is in start position.")
            try:
                input()
            except EOFError:
                pass
        else:
            # Subsequent passes: auto-return using IMU + LiDAR closed-loop
            prev = results[-1]
            ok(f"\nPass {i+1}/{n_passes} — auto-returning to start position…")
            node.return_to_start(
                target_dist    = args.distance,
                speed          = args.speed,
                pre_wall_dist  = prev["pre_wall_dist"],
                start_yaw      = prev["start_yaw"],
                use_lidar      = not args.imu_only,
            )
        r = node.run_pass(args.distance, args.speed, use_lidar=not args.imu_only)
        if r is None:
            fail("Pass aborted (safety stop or no movement).")
            rclpy.shutdown(); sys.exit(1)
        results.append(r)
        print_pass(i, r, args.distance)

    # ── Aggregate ───
    avg = aggregate(results)
    hdr("Aggregated measurements")
    info(f"Avg odom_fwd      : {avg['odom_fwd']*100:+.2f} cm")
    info(f"Avg odom lateral  : {avg['odom_lat']*100:+.2f} cm")
    lidar_available = avg["lidar_dist"] is not None
    if lidar_available:
        info(f"Avg LiDAR dist    : {avg['lidar_dist']*100:+.2f} cm  (ground truth)")
    drift, drift_src = best_yaw_drift(avg)
    info(f"Avg yaw drift     : {math.degrees(drift):+.3f}°  (source: {drift_src})")

    wall_trusted = avg.get("wall_metrics_valid", False)

    if args.verify_only:
        hdr("Verification summary (no changes applied)")
        lat_mm  = abs(avg['odom_lat']) * 1000
        if wall_trusted and avg["lidar_dist"] is not None:
            dist_err = (avg["lidar_dist"] - args.distance) / args.distance * 100
            print(f"  Distance error : {dist_err:+.1f}%  "
                  f"(LiDAR {avg['lidar_dist']*100:.2f} cm vs commanded {args.distance*100:.0f} cm)")
        else:
            fwd_err = (avg['odom_fwd'] - args.distance) / args.distance * 100
            print(f"  Forward error  : {fwd_err:+.1f}%  (odom only — LiDAR not trusted)")
        print(f"  Lateral drift  : {lat_mm:.1f} mm "
              f"({'LEFT' if avg['odom_lat'] > 0 else 'RIGHT'})")
        print(f"  Yaw drift      : {math.degrees(drift):+.2f}°  (source: {drift_src})")
        if not wall_trusted:
            warn("LiDAR wall metrics not trusted (robot off-perpendicular).")
        if abs(math.degrees(drift)) < 1:
            ok("Robot is already well-calibrated!")
        else:
            warn("Calibration recommended.  Rerun without --verify-only.")
        node.destroy_node(); rclpy.shutdown(); return

    # ── Compute new parameters ──
    hdr("Computing corrections")

    # 1. MAX_WHEEL_SPEED_MS from LiDAR ground-truth distance (only when wall trusted)
    if wall_trusted and avg["lidar_dist"] is not None and avg["lidar_dist"] > 0.01 \
            and avg["odom_dist"] is not None and avg["odom_dist"] > 0.01:
        ratio = avg["lidar_dist"] / avg["odom_dist"]
        new_max_speed = max(0.05, min(2.0, cur_max_speed * ratio))
        info(f"MAX_WHEEL_SPEED_MS: {cur_max_speed:.6f} → {new_max_speed:.6f} m/s  "
             f"(LiDAR {avg['lidar_dist']*100:.2f} cm vs odom {avg['odom_dist']*100:.2f} cm, "
             f"ratio {ratio:.4f})")
    else:
        new_max_speed = cur_max_speed
        if not wall_trusted:
            warn("LiDAR not trusted (robot off-perpendicular) — MAX_WHEEL_SPEED_MS unchanged.")
        else:
            warn("No reliable LiDAR distance — MAX_WHEEL_SPEED_MS unchanged.")

    # 2. Motor trim from yaw drift
    # Use LiDAR distance for travel if trusted, else fall back to odom
    travel = avg["lidar_dist"] if (wall_trusted and avg["lidar_dist"] is not None) \
             else avg["odom_fwd"]
    new_trim_l, new_trim_r = compute_trim_correction(
        drift, travel, wheel_sep, cur_trim_l, cur_trim_r)
    info(f"Trim correction: L {cur_trim_l:.6f} → {new_trim_l:.6f}  "
         f"R {cur_trim_r:.6f} → {new_trim_r:.6f}")
    info(f"  (yaw drift {math.degrees(drift):+.2f}° over "
         f"{travel*100:.1f} cm @ sep={wheel_sep*100:.1f} cm, "
         f"damp={CORRECTION_DAMP:.0%}, source: {drift_src})")

    # Sanity-check: if drift is negligible, keep existing trims
    if abs(math.degrees(drift)) < 0.5:
        info("Yaw drift < 0.5° — motor trims unchanged (already balanced).")
        new_trim_l, new_trim_r = cur_trim_l, cur_trim_r

    # ── Apply to files ──
    apply_corrections(new_max_speed, new_trim_l, new_trim_r,
                      cur_max_speed, cur_trim_l, cur_trim_r)

    if args.no_flash:
        warn("--no-flash: files updated but firmware NOT rebuilt.")
        warn("Run:  cd firmware && ./build.sh flash")
        node.destroy_node(); rclpy.shutdown(); return

    # ── Rebuild & flash ──
    if not rebuild_flash():
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    # Brief pause for the device to re-enumerate, then restart the service
    info("Waiting 5 s for device to re-enumerate…")
    time.sleep(5.0)
    restart_service()

    # Wait for ROS topics to come back with a fresh node
    node.destroy_node()
    node = Calibrator()
    info("Waiting for topics after reflash…")
    if not node.wait_for_data(20.0):
        warn("Topics not yet available — skipping verification pass.")
        node.destroy_node(); rclpy.shutdown(); return

    # ── Verification pass ──
    hdr("Verification pass (with new firmware)")
    print(f"{YLW}Return robot to start position and press Enter.{NC}")
    try:
        input()
    except EOFError:
        pass

    vr = node.run_pass(args.distance, args.speed)
    if vr is None:
        warn("Verification aborted.")
    else:
        print_pass(0, vr, args.distance)
        v_drift, v_src = best_yaw_drift(vr)
        lat_mm = abs(vr["odom_lat"]) * 1000
        hdr("Final calibration quality")
        if abs(math.degrees(v_drift)) < 1.5 and lat_mm < 5.0:
            ok(f"Straight-line calibration COMPLETE — "
               f"drift={math.degrees(v_drift):+.2f}°, lateral={lat_mm:.1f} mm")
        else:
            warn(f"Residual drift: {math.degrees(v_drift):+.2f}° / "
                 f"{lat_mm:.1f} mm lateral.  "
                 f"Rerun to iterate further.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
