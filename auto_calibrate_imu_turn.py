#!/usr/bin/env python3
"""
Auto-calibrate TurtleBot3 Pico angular accuracy via WHEEL_SEPARATION.

Why WHEEL_SEPARATION, not a command scale?
-----------------------------------------
In diff-drive firmware:
    v_left  = lin_x - ang_z * (WHEEL_SEPARATION / 2)
    v_right = lin_x + ang_z * (WHEEL_SEPARATION / 2)

This also drives the tick accumulation that becomes /odom.  A pure scale on
ang_z would corrupt reported odometry — nav-stack feedback would be wrong
even if the robot physically rotated correctly.  The right calibration is
WHEEL_SEPARATION: if the robot over-rotates, decrease it; under-rotates,
increase it.

    new_separation = old_separation * (desired_angle / measured_angle)

The same value must be set in BOTH:
  - firmware/main.c          (#define WHEEL_SEPARATION)
  - config/burger_pico.yaml  (wheels.separation)

IMU usage
---------
The firmware publishes a REAL BNO085 IMU on /imu if the sensor is wired.
If the sensor is absent, /imu carries a SIMULATED signal derived from
commanded velocity — measuring it would just reflect commands, not actual
motion.

This script detects real vs simulated IMU at startup by checking angular
velocity variance during a brief rotation.  If the IMU is simulated, it
falls back to a manual measurement prompt.

LiDAR fusion
------------
When a /scan topic is available the script also uses the LiDAR point cloud
to independently measure the rotation angle.  Two algorithms are applied and
combined:

  1. FFT polar cross-correlation
     The LDS-01 produces a 360° range array sampled at uniform angular steps.
     Circular cross-correlation in the frequency domain (R · S*) gives a
     correlation peak whose index is the rotation shift.  Sub-pixel accuracy
     is recovered with parabolic peak interpolation.  This is O(N log N) and
     robust against feature-poor environments (walls, corners all contribute).

  2. SVD/Procrustes 2D scan refinement
     Valid range samples are converted to (x, y) Cartesian points.  The
     optimal rotation that best aligns the two point sets is found via the
     standard SVD rotation step (Arun et al., 1987).  This gives an
     independent angular estimate that is used to validate the FFT result and
     to further refine the final answer.

  3. Kalman-style weighted fusion
     Each sensor contributes with a weight proportional to 1/σ²:
       σ_lidar_fft ≈ 0.3°  (sub-pixel interpolated)
       σ_lidar_svd ≈ 0.4°
       σ_imu       ≈ 1.5°  (gyro drift over the measurement window)
     Fused estimate = Σ(w_i · θ_i) / Σ(w_i)

Usage
-----
  # Dry-run (measure and report only, both IMU + LiDAR):
  python3 auto_calibrate_imu_turn.py

  # Apply to files:
  python3 auto_calibrate_imu_turn.py --apply

  # Apply + rebuild + flash + restart service:
  python3 auto_calibrate_imu_turn.py --apply --flash

  # Custom test parameters:
  python3 auto_calibrate_imu_turn.py --speed 0.4 --angle 180 --runs 3 --apply

  # IMU-only (no LiDAR):
  python3 auto_calibrate_imu_turn.py --no-lidar

  # LiDAR-only (skip IMU weighting):
  python3 auto_calibrate_imu_turn.py --fusion lidar
"""

import argparse
import math
import re
import subprocess
import time
from pathlib import Path
from typing import NamedTuple, Optional

import numpy as np

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu, LaserScan


SCRIPT_DIR   = Path(__file__).resolve().parent
MAIN_C       = SCRIPT_DIR / "firmware" / "main.c"
YAML_PATH    = SCRIPT_DIR / "config" / "burger_pico.yaml"
FIRMWARE_DIR = SCRIPT_DIR / "firmware"
BUILD_SCRIPT = FIRMWARE_DIR / "build.sh"

# Angular velocity variance threshold to distinguish real BNO085 from simulated
REAL_IMU_GYRO_VAR_THRESHOLD = 1e-5   # rad^2/s^2

# Sensor noise standard deviations (degrees) used for weighted fusion
SIGMA_IMU       = 1.5   # gyro drift + integration error
SIGMA_LIDAR_FFT = 0.30  # FFT + sub-pixel interpolation
SIGMA_LIDAR_SVD = 0.40  # SVD Procrustes alignment



# ─── file I/O helpers ────────────────────────────────────────────────────────

def read_define_float(path: Path, name: str) -> Optional[float]:
    pat = re.compile(rf"^\s*#define\s+{re.escape(name)}\s+([\d.+\-eEfF]+)")
    for line in path.read_text().splitlines():
        m = pat.match(line)
        if m:
            return float(m.group(1).rstrip("fF"))
    return None


def write_define_float(path: Path, name: str, value: float, comment: str = "") -> None:
    content = path.read_text()
    suffix  = f"  // {comment}" if comment else ""
    pat     = re.compile(
        rf"(#define\s+{re.escape(name)}\s+)[\d.+\-eEfF]+f?([ \t]*(?://[^\n]*)?)"
    )
    def repl(m):
        return f"{m.group(1)}{value:.6f}f{suffix}"
    new_content, count = pat.subn(repl, content, count=1)
    if count == 0:
        raise RuntimeError(f"Could not find #define {name} in {path}")
    path.write_text(new_content)


def write_yaml_separation(path: Path, value: float) -> None:
    content = path.read_text()
    pat = re.compile(r"(separation:\s*)[\d.]+(\s*(?:#[^\n]*)?)")
    new_content, count = pat.subn(
        lambda m: f"{m.group(1)}{value:.6f}{m.group(2)}", content, count=1
    )
    if count == 0:
        raise RuntimeError(f"Could not find 'separation:' in {path}")
    path.write_text(new_content)


# ─── LiDAR scan-matching helpers ─────────────────────────────────────────────

def _scan_to_array(scan: LaserScan) -> np.ndarray:
    """Return a 1-D range array with inf/nan/max-range replaced by 0."""
    r = np.array(scan.ranges, dtype=np.float64)
    r = np.where(np.isfinite(r), r, 0.0)
    if scan.range_max > 0:
        r = np.where(r >= scan.range_max * 0.98, 0.0, r)
    return r


def _parabolic_peak(arr: np.ndarray, idx: int) -> float:
    """Sub-sample peak location via parabolic interpolation."""
    n = len(arr)
    il = (idx - 1) % n
    ir = (idx + 1) % n
    denom = arr[il] - 2.0 * arr[idx] + arr[ir]
    if abs(denom) < 1e-12:
        return float(idx)
    return idx - 0.5 * (arr[ir] - arr[il]) / denom


def fft_scan_rotation(scan_before: LaserScan, scan_after: LaserScan) -> tuple[float, float]:
    """
    Estimate the rotation angle between two LaserScans using FFT
    circular cross-correlation.  Returns (angle_rad, quality).

    Algorithm (Hara & Ioka 1994 / Röfer 1998):
      1. Interpolate both scans onto a common uniform angular grid.
      2. Z-score normalise each array so quality is amplitude-independent.
      3. R̂ = FFT(before) · conj(FFT(after)); corr = IFFT(R̂).
      4. Sub-pixel peak via parabolic interpolation.
      5. quality = peak / second_peak  (higher → more reliable).
    """
    n_ref = len(scan_before.ranges)
    n_cur = len(scan_after.ranges)
    N = min(n_ref, n_cur)

    r_before = np.interp(
        np.linspace(0, 1, N), np.linspace(0, 1, n_ref), _scan_to_array(scan_before)
    )
    r_after = np.interp(
        np.linspace(0, 1, N), np.linspace(0, 1, n_cur), _scan_to_array(scan_after)
    )

    std_b = r_before.std()
    std_a = r_after.std()
    if std_b < 1e-6 or std_a < 1e-6:
        return 0.0, 0.0  # featureless scan

    r_before = (r_before - r_before.mean()) / std_b
    r_after  = (r_after  - r_after.mean())  / std_a

    F_b  = np.fft.rfft(r_before)
    F_a  = np.fft.rfft(r_after)
    corr = np.fft.irfft(F_b * np.conj(F_a), n=N)

    peak_idx = int(np.argmax(corr))
    sub_idx  = _parabolic_peak(corr, peak_idx)

    # Second-highest peak quality metric (exclude ±3 bins around peak)
    mask = np.ones(N, dtype=bool)
    for k in range(-3, 4):
        mask[(peak_idx + k) % N] = False
    second_peak = corr[mask].max() if mask.any() else 0.0
    quality = corr[peak_idx] / (second_peak + 1e-9)

    fov_rad     = scan_before.angle_max - scan_before.angle_min
    rad_per_bin = fov_rad / N
    shift       = sub_idx
    if shift > N / 2:
        shift -= N
    return shift * rad_per_bin, quality


def svd_scan_rotation(
    scan_before: LaserScan,
    scan_after: LaserScan,
    seed_angle_rad: float = 0.0,
    max_match_dist: float = 0.10,
    min_points: int = 20,
) -> tuple[Optional[float], float]:
    """
    Estimate rotation via 2-D SVD/Procrustes (Arun et al. 1987).

    For pure-rotation motion with seed from FFT:
      1. Convert scans to (x, y) Cartesian.
      2. Pre-rotate scan_after by –seed to align roughly.
      3. Nearest-neighbour data association; reject outliers > max_match_dist.
      4. SVD of H = Pᵀ Q  →  R = V Uᵀ  →  θ = atan2(R[1,0], R[0,0]).
      5. Final angle = seed + refinement.

    Returns (angle_rad, inlier_ratio).
    """
    def to_xy(scan: LaserScan) -> np.ndarray:
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        r = np.array(scan.ranges, dtype=np.float64)
        valid = np.isfinite(r) & (r > scan.range_min + 1e-4) & (r < scan.range_max * 0.98)
        return np.column_stack([r[valid] * np.cos(angles[valid]),
                                r[valid] * np.sin(angles[valid])])

    pts_b = to_xy(scan_before)
    pts_a = to_xy(scan_after)

    if len(pts_b) < min_points or len(pts_a) < min_points:
        return None, 0.0

    # Pre-align with FFT seed
    c, s = math.cos(-seed_angle_rad), math.sin(-seed_angle_rad)
    R_seed = np.array([[c, -s], [s, c]])
    pts_a_aligned = (R_seed @ pts_a.T).T

    # Nearest-neighbour correspondences
    if len(pts_b) <= 500:
        diff   = pts_b[:, np.newaxis, :] - pts_a_aligned[np.newaxis, :, :]
        dists  = np.linalg.norm(diff, axis=2)
        nn_idx = np.argmin(dists, axis=1)
        nn_dist = dists[np.arange(len(pts_b)), nn_idx]
    else:
        from scipy.spatial import cKDTree
        tree = cKDTree(pts_a_aligned)
        nn_dist, nn_idx = tree.query(pts_b, k=1)

    inlier       = nn_dist < max_match_dist
    inlier_ratio = inlier.sum() / max(len(pts_b), 1)

    if inlier.sum() < min_points:
        return None, inlier_ratio

    P = pts_b[inlier] - pts_b[inlier].mean(axis=0)
    Q = pts_a_aligned[nn_idx[inlier]] - pts_a_aligned[nn_idx[inlier]].mean(axis=0)

    H = P.T @ Q
    U, _, Vt = np.linalg.svd(H)
    R_svd = Vt.T @ U.T
    if np.linalg.det(R_svd) < 0:
        Vt[-1, :] *= -1
        R_svd = Vt.T @ U.T

    refinement  = math.atan2(R_svd[1, 0], R_svd[0, 0])
    total_angle = seed_angle_rad + refinement
    return total_angle, inlier_ratio


def fuse_estimates(
    imu_rad: Optional[float],
    fft_rad: Optional[float],
    fft_quality: float,
    svd_rad: Optional[float],
    svd_inlier: float,
    mode: str,
) -> tuple[float, str]:
    """
    Kalman-style weighted fusion: fused = Σ(w_i θ_i) / Σ(w_i)
    where w_i = 1 / σ_i²

    Returns (fused_angle_rad, explanation_string).
    """
    estimates = []  # list of (angle_rad, label, sigma_deg)

    if imu_rad is not None and mode in ("imu", "fused"):
        estimates.append((imu_rad, "IMU", SIGMA_IMU))

    if fft_rad is not None and mode in ("lidar", "fused"):
        sigma_fft = SIGMA_LIDAR_FFT / max(fft_quality / 5.0, 1.0)
        sigma_fft = max(sigma_fft, 0.15)
        estimates.append((fft_rad, f"LiDAR-FFT(q={fft_quality:.1f})", sigma_fft))

    if svd_rad is not None and mode in ("lidar", "fused"):
        sigma_svd = SIGMA_LIDAR_SVD / max(svd_inlier, 0.3)
        sigma_svd = max(sigma_svd, 0.15)
        estimates.append((svd_rad, f"LiDAR-SVD(inlier={svd_inlier:.0%})", sigma_svd))

    if not estimates:
        raise ValueError("No estimates available for fusion")

    weights = [1.0 / (math.radians(s) ** 2) for _, _, s in estimates]
    total_w  = sum(weights)
    fused    = sum(w * a for w, (a, _, _) in zip(weights, estimates)) / total_w

    parts = [
        f"{lbl}={math.degrees(ang):.2f}°(w={w:.0f})"
        for w, (ang, lbl, _) in zip(weights, estimates)
    ]
    return fused, " | ".join(parts)


# ─── ROS node ─────────────────────────────────────────────────────────────────

def angle_diff(a: float, b: float) -> float:
    d = a - b
    while d >  math.pi: d -= 2.0 * math.pi
    while d < -math.pi: d += 2.0 * math.pi
    return d


def quat_to_yaw(msg: Imu) -> float:
    o = msg.orientation
    siny = 2.0 * (o.w * o.z + o.x * o.y)
    cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    return math.atan2(siny, cosy)


class TurnCalibrator(Node):
    def __init__(self, scan_topic: str = "/scan"):
        super().__init__("turn_calibrator")
        self.pub           = self.create_publisher(Twist, "/cmd_vel", 10)
        self.last_yaw      = None   # kept only for wait_for_imu() gate
        self.cum_yaw       = 0.0
        self.collecting    = False
        self._last_imu_stamp: Optional[float] = None
        self._gyro_samples: list[float] = []
        self._gyro_collecting = False

        self.create_subscription(Imu, "/imu", self._imu_cb, 30)

        # LiDAR publishes with BEST_EFFORT (YDLidar driver default).
        # A RELIABLE subscription silently receives nothing — match the publisher.
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._latest_scan: Optional[LaserScan] = None
        self._scan_received = False
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, scan_qos)
        self._scan_topic = scan_topic

    # ── callbacks ──────────────────────────────────────────────────────────

    def _imu_cb(self, msg: Imu):
        # Keep last_yaw for wait_for_imu() gate only (not used for measurement)
        self.last_yaw = quat_to_yaw(msg)

        if self._gyro_collecting:
            self._gyro_samples.append(msg.angular_velocity.z)

        if self.collecting:
            # Integrate gyro-Z × dt instead of summing quaternion yaw diffs.
            #
            # Why: the BNO085 orientation quaternion fires at high rate (50-200 Hz).
            # Summing angle_diff(quat_yaw_new, quat_yaw_old) at each tick
            # accumulates quantization noise and FIFO-ordering artefacts that
            # produce systematic over-counting (observed: 3-4× actual rotation).
            # Integrating msg.angular_velocity.z × dt is a direct physical
            # measurement of the angular displacement with no such artefacts.
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self._last_imu_stamp is not None:
                dt = stamp - self._last_imu_stamp
                if 0.0 < dt < 0.5:   # ignore stale or out-of-order stamps
                    self.cum_yaw += msg.angular_velocity.z * dt
            self._last_imu_stamp = stamp

    def _scan_cb(self, msg: LaserScan):
        self._latest_scan = msg
        self._scan_received = True

    # ── spin helpers ───────────────────────────────────────────────────────

    def spin_for(self, secs: float) -> None:
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, lin: float, ang: float) -> None:
        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self.pub.publish(msg)

    def stop(self, settle: float = 0.5) -> None:
        self.send(0.0, 0.0)
        self.spin_for(settle)

    def wait_for_imu(self, timeout: float = 8.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_yaw is not None:
                return True
        return False

    def wait_for_lidar(self, timeout: float = 6.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._scan_received:
                return True
        return False

    def capture_scan(self, settle: float = 0.15) -> Optional[LaserScan]:
        """Spin briefly and return the most recent scan snapshot."""
        self.spin_for(settle)
        return self._latest_scan

    # ── sensor detection ───────────────────────────────────────────────────

    def detect_real_imu(self, speed: float = 0.4, duration: float = 1.5) -> bool:
        """Return True if /imu appears to be a real BNO085 (non-simulated)."""
        print("Detecting IMU type (real BNO085 vs simulated)...")
        self._gyro_samples.clear()
        self._gyro_collecting = True
        end = time.time() + duration
        while time.time() < end:
            self.send(0.0, speed)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop(0.3)
        self._gyro_collecting = False

        samples = self._gyro_samples
        if len(samples) < 5:
            return False
        mean = sum(samples) / len(samples)
        var  = sum((s - mean) ** 2 for s in samples) / len(samples)
        print(f"  Gyro-Z  samples={len(samples)}  mean={mean:.4f} rad/s  var={var:.2e}")
        is_real = var > REAL_IMU_GYRO_VAR_THRESHOLD
        print(f"  IMU type: {'REAL BNO085' if is_real else 'SIMULATED (no BNO085 sensor)'}")
        return is_real

    # ── measurement ────────────────────────────────────────────────────────

    def measure_rotation_imu(self, speed: float, target_rad: float) -> float:
        """Rotate at `speed` for the time matching `target_rad`, return IMU yaw change."""
        duration = abs(target_rad / speed)
        self.cum_yaw  = 0.0
        self.collecting = True
        end = time.time() + duration
        while time.time() < end:
            self.send(0.0, speed)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.collecting = False
        self.stop(0.6)
        return self.cum_yaw

    def measure_rotation_fused(
        self,
        speed: float,
        target_rad: float,
        use_lidar: bool,
        real_imu: bool,
        fusion_mode: str,
    ) -> tuple[float, str]:
        """
        Execute one rotation and return (measured_angle_rad, detail_string).

        Pipeline:
          1. Capture pre-rotation scan (if LiDAR enabled).
          2. Rotate while accumulating IMU yaw.
          3. Capture post-rotation scan.
          4. FFT polar cross-correlation → coarse angle.
          5. SVD Procrustes refinement with FFT seed → fine angle.
          6. Kalman-style weighted fusion with IMU.
        """
        imu_angle: Optional[float] = None
        fft_angle: Optional[float] = None
        fft_quality = 0.0
        svd_angle: Optional[float] = None
        svd_inlier = 0.0

        # Capture reference scan before rotation
        scan_before: Optional[LaserScan] = None
        if use_lidar:
            scan_before = self.capture_scan(settle=0.20)
            if scan_before is None:
                print("  Warning: no LiDAR scan available — falling back to IMU only.")
                use_lidar = False

        # Execute rotation + collect IMU
        duration = abs(target_rad / speed)
        self.cum_yaw  = 0.0
        self._last_imu_stamp = None   # reset so first dt is skipped, not garbage
        self.collecting = real_imu
        end = time.time() + duration
        while time.time() < end:
            self.send(0.0, speed)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.collecting = False
        self.stop(settle=0.6)

        if real_imu:
            imu_angle = self.cum_yaw  # signed
            # Sanity check: gyro integration drifting severely indicates a
            # measurement problem (e.g. IMU flooding with stale stamps).
            # Discard rather than letting a garbage reading pollute the fusion.
            if abs(imu_angle) > 3.0 * abs(target_rad):
                print(f"  Warning: IMU measured {math.degrees(abs(imu_angle)):.1f}° "
                      f"which is >3× target {math.degrees(abs(target_rad)):.1f}° "
                      "— discarding IMU for this run.")
                imu_angle = None

        # Capture post-rotation scan
        if use_lidar:
            scan_after = self.capture_scan(settle=0.20)

            if scan_after is not None:
                # 1. FFT polar cross-correlation
                fft_raw, fft_quality = fft_scan_rotation(scan_before, scan_after)

                # Enforce consistent sign using command direction or IMU
                sign = math.copysign(1.0, imu_angle if imu_angle is not None else speed)
                fft_angle = math.copysign(abs(fft_raw), sign)

                # 2. SVD/Procrustes refinement
                svd_raw, svd_inlier = svd_scan_rotation(
                    scan_before, scan_after, seed_angle_rad=fft_angle
                )
                if svd_raw is not None:
                    svd_angle = math.copysign(abs(svd_raw), sign)

                # Sanity: discard LiDAR if FFT quality too low or estimates diverge
                if fft_quality < 1.2:
                    print(f"  Warning: FFT quality={fft_quality:.2f} too low — "
                          "discarding LiDAR estimates.")
                    fft_angle = None
                    svd_angle = None
                elif svd_angle is not None:
                    diff_deg = abs(math.degrees(abs(fft_angle) - abs(svd_angle)))
                    if diff_deg > max(5.0, 0.10 * math.degrees(abs(target_rad))):
                        print(f"  Warning: FFT ({math.degrees(fft_angle):.1f}°) and "
                              f"SVD ({math.degrees(svd_angle):.1f}°) diverge by "
                              f"{diff_deg:.1f}° — dropping SVD for this run.")
                        svd_angle = None

        # Weighted fusion
        fused, detail = fuse_estimates(
            imu_angle, fft_angle, fft_quality, svd_angle, svd_inlier, mode=fusion_mode
        )
        return fused, detail


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Auto-calibrate WHEEL_SEPARATION for angular accuracy"
    )
    parser.add_argument("--speed",  type=float, default=0.5,
                        help="angular.z command in rad/s (default 0.5)")
    parser.add_argument("--angle",  type=float, default=180.0,
                        help="target rotation in degrees (default 180)")
    parser.add_argument("--runs",   type=int,   default=3,
                        help="calibration runs to average (default 3)")
    parser.add_argument("--apply",  action="store_true",
                        help="write updated WHEEL_SEPARATION to firmware and yaml")
    parser.add_argument("--flash",  action="store_true",
                        help="after --apply, rebuild/flash firmware and restart service")
    parser.add_argument("--no-lidar", action="store_true",
                        help="skip LiDAR scan-matching; use IMU only")
    parser.add_argument("--scan-topic", default="/scan", metavar="TOPIC",
                        help="LiDAR scan topic (default: /scan)")
    parser.add_argument("--fusion", choices=["fused", "imu", "lidar"], default="fused",
                        help="sensor weighting mode (default: fused)")
    parser.add_argument("--damping", type=float, default=0.85,
                        help="correction step damping 0-1 (default 0.85; lower = "
                             "more conservative, prevents divergence on bad runs)")
    parser.add_argument("--reset", type=float, default=None, metavar="CM",
                        help="reset WHEEL_SEPARATION to CM without running a test "
                             "(e.g. --reset 16.0 for the stock Burger default); "
                             "combine with --apply to write immediately")
    args = parser.parse_args()

    if args.flash and not args.apply:
        print("ERROR: --flash requires --apply")
        return 4

    # ── Hard reset: bypass measurement entirely ───────────────────────────────
    if args.reset is not None:
        new_sep = args.reset / 100.0
        new_sep = max(0.01, min(0.50, new_sep))
        print(f"\nResetting WHEEL_SEPARATION to {new_sep*100:.4f} cm ({new_sep:.6f} m)")
        if args.apply:
            write_define_float(
                MAIN_C, "WHEEL_SEPARATION", new_sep,
                "metres — reset by auto_calibrate_imu_turn.py --reset"
            )
            write_yaml_separation(YAML_PATH, new_sep)
            print(f"Applied to      : {MAIN_C.relative_to(SCRIPT_DIR)}")
            print(f"Applied to      : {YAML_PATH.relative_to(SCRIPT_DIR)}")
            print("\nNext steps: rebuild/flash firmware, then restart service:")
            print(f"  cd {SCRIPT_DIR.relative_to(SCRIPT_DIR.parent)}/firmware && ./build.sh flash")
        else:
            print("Dry-run: use --apply to write.")
        return 0

    use_lidar   = not args.no_lidar
    fusion_mode = args.fusion
    if args.no_lidar and fusion_mode == "lidar":
        print("ERROR: --no-lidar and --fusion lidar are contradictory.")
        return 4

    target_rad = math.radians(args.angle)

    old_sep = read_define_float(MAIN_C, "WHEEL_SEPARATION")
    if old_sep is None:
        print(f"ERROR: WHEEL_SEPARATION not found in {MAIN_C}")
        return 2

    print("\n=== Angular Calibration via WHEEL_SEPARATION ===")
    print(f"Firmware        : {MAIN_C}")
    print(f"YAML            : {YAML_PATH}")
    print(f"Current sep     : {old_sep*100:.4f} cm")
    print(f"Test rotation   : {args.angle:.0f} deg at {args.speed:.2f} rad/s  ({args.runs} run(s))")
    print(f"Fusion mode     : {fusion_mode}")
    print(f"LiDAR topic     : {args.scan_topic if use_lidar else 'disabled'}")
    print("Safety          : robot should be able to spin freely\n")

    rclpy.init()
    node = TurnCalibrator(scan_topic=args.scan_topic)

    if not node.wait_for_imu():
        print("ERROR: /imu not available. Start bringup first.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    real_imu = node.detect_real_imu(speed=min(args.speed, 0.5), duration=1.5)
    print()

    # Probe LiDAR availability
    if use_lidar:
        print(f"Waiting for LiDAR on {args.scan_topic} ...", end=" ", flush=True)
        if node.wait_for_lidar(timeout=6.0):
            scan = node._latest_scan
            n_pts  = len(scan.ranges) if scan else 0
            fov_deg = math.degrees(scan.angle_max - scan.angle_min) if scan else 0
            print(f"OK  ({n_pts} pts, FOV={fov_deg:.0f}°)")
        else:
            print("not available — proceeding with IMU only.")
            use_lidar = False
            if fusion_mode == "lidar":
                print("ERROR: --fusion lidar requested but no LiDAR found.")
                node.destroy_node()
                rclpy.shutdown()
                return 1
    print()

    if not use_lidar and fusion_mode == "fused":
        fusion_mode = "imu"
        print("LiDAR unavailable — switching fusion mode to 'imu'.\n")

    measurements: list[float] = []

    if real_imu or use_lidar:
        # ── Automatic measurement (IMU and/or LiDAR) ─────────────────────
        for i in range(args.runs):
            print(f"Run {i+1}/{args.runs} — rotating {args.angle:.0f} deg ...")
            measured, detail = node.measure_rotation_fused(
                speed=args.speed,
                target_rad=target_rad,
                use_lidar=use_lidar,
                real_imu=real_imu,
                fusion_mode=fusion_mode,
            )
            measurements.append(abs(measured))
            print(f"  Measured : {math.degrees(abs(measured)):.2f}°  "
                  f"desired={args.angle:.1f}°  ratio={target_rad/abs(measured):.4f}")
            print(f"  Detail   : {detail}")
            if i < args.runs - 1:
                time.sleep(0.8)
    else:
        # ── Manual measurement fallback ───────────────────────────────────
        print("WARNING: Simulated IMU and no LiDAR — using manual measurement mode.")
        print("   Mark the robot heading with tape/marker before each run.\n")
        for i in range(args.runs):
            input(f"Run {i+1}/{args.runs}: Mark robot heading, then press Enter to rotate... ")
            duration = abs(target_rad / args.speed)
            print(f"  Rotating {args.angle:.0f} deg ({duration:.1f}s)...")
            end = time.time() + duration
            while time.time() < end:
                node.send(0.0, args.speed)
                rclpy.spin_once(node, timeout_sec=0.05)
            node.stop(0.6)
            raw = input("  Measure actual rotation in degrees (use tape mark as reference): ")
            try:
                measured_deg = float(raw.strip())
            except ValueError:
                print("  Invalid input — skipping this run.")
                continue
            measured = math.radians(abs(measured_deg))
            measurements.append(measured)
            print(f"  Recorded : {measured_deg:.1f}°  ratio={target_rad/measured:.4f}")
            if i < args.runs - 1:
                time.sleep(0.8)

    node.stop(0.2)
    node.destroy_node()
    rclpy.shutdown()

    if not measurements:
        print("ERROR: no valid measurements collected.")
        return 3

    avg_measured = sum(measurements) / len(measurements)
    if avg_measured < math.radians(5):
        print("ERROR: measured angle too small to calibrate reliably.")
        return 3

    std_deg = (
        math.degrees(math.sqrt(
            sum((m - avg_measured) ** 2 for m in measurements) / len(measurements)
        ))
        if len(measurements) > 1 else float("nan")
    )

    # new_sep = old_sep * (1 + damping*(correction - 1))
    # where correction = target/measured
    # This damps the step: at damping=1.0 it applies the full correction;
    # at damping=0.85 it applies 85% of the step, preventing divergence
    # caused by noisy single-session measurements.
    correction = target_rad / avg_measured
    correction_damped = 1.0 + args.damping * (correction - 1.0)
    new_sep = old_sep * correction_damped
    new_sep = max(0.01, min(0.50, new_sep))   # sanity clamp 1-50 cm

    # Warn if run-to-run variance is high enough that the result may be
    # unreliable (σ > 20% of mean)
    if len(measurements) > 1 and std_deg > 0.20 * math.degrees(avg_measured):
        print(f"\nWARNING: σ={std_deg:.1f}° is >20% of avg={math.degrees(avg_measured):.1f}°."
              " Results may be unreliable — consider more runs (--runs 5) or check environment.")

    print("\n--- Result ---------------------------------------------------")
    for idx, m in enumerate(measurements):
        print(f"  Run {idx+1}: {math.degrees(m):.2f}°")
    print(f"Avg measured    : {math.degrees(avg_measured):.2f}°  (σ={std_deg:.2f}°)")
    print(f"Desired         : {args.angle:.1f}°")
    print(f"old WHEEL_SEP   : {old_sep*100:.4f} cm  ({old_sep:.6f} m)")
    print(f"new WHEEL_SEP   : {new_sep*100:.4f} cm  ({new_sep:.6f} m)")
    print(f"Formula         : new = {old_sep:.6f} × (1 + {args.damping:.2f}×({args.angle:.1f}°/{math.degrees(avg_measured):.2f}°-1))")

    if args.apply:
        write_define_float(
            MAIN_C, "WHEEL_SEPARATION", new_sep,
            "metres — effective turning base; calibrated by auto_calibrate_imu_turn.py"
        )
        write_yaml_separation(YAML_PATH, new_sep)
        print(f"\nApplied to      : {MAIN_C.relative_to(SCRIPT_DIR)}")
        print(f"Applied to      : {YAML_PATH.relative_to(SCRIPT_DIR)}")

        if args.flash:
            print("\n--- Flash ----------------------------------------------------")
            if not BUILD_SCRIPT.exists():
                print(f"ERROR: {BUILD_SCRIPT} not found")
                return 5
            print("Building and flashing firmware...")
            r = subprocess.run(["bash", str(BUILD_SCRIPT), "flash"],
                               cwd=str(FIRMWARE_DIR))
            if r.returncode != 0:
                print(f"ERROR: flash failed (exit {r.returncode})")
                return r.returncode
            print("Restarting turtlebot3-bringup service...")
            r = subprocess.run(["sudo", "systemctl", "restart", "turtlebot3-bringup"])
            if r.returncode != 0:
                print(f"ERROR: service restart failed (exit {r.returncode})")
                return r.returncode
            print("Done — firmware flashed and service restarted.")
        else:
            print("\nNext steps: rebuild/flash firmware, then restart service:")
            print(f"  cd firmware && ./build.sh flash")
    else:
        print("\nDry-run: no files changed. Use --apply to write changes.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

