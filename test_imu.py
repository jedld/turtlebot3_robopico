#!/usr/bin/env python3
"""
test_imu.py — BNO085 IMU verification test (static, no motor movement)

Subscribes to /imu and /magnetic_field, collects samples for ~5 seconds, then
reports:
  - Whether the data looks like the real BNO085 or the simulated fallback
  - Orientation (roll / pitch / yaw in degrees)
  - Linear acceleration vector and magnitude (should be ~9.81 m/s²)
  - Angular velocity (should be near-zero but with small sensor noise)
  - Magnetometer field (if published)
  - Covariance diagonals

Detection heuristics for real vs. simulated IMU:
  Simulated fallback produces:
    orientation  = identity quaternion  (w=1, x=y=z=0, constant)
    linear_accel = (0, 0, 9.81)        (constant)
    angular_vel  = (0, 0, 0)           (constant)
  Real BNO085 will show small sensor noise on all axes and the gravity
  vector may deviate slightly from pure +Z depending on how the board
  is mounted.

Usage:
    python3 test_imu.py

Requires: ROS 2 bringup already running (turtlebot3_node publishing /imu).
"""

import math
import time
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

# ── colours ──────────────────────────────────────────────────────────────────
RED  = "\033[0;31m"
GRN  = "\033[0;32m"
YLW  = "\033[1;33m"
CYN  = "\033[0;36m"
BLD  = "\033[1m"
NC   = "\033[0m"

def ok(msg):   print(f"{GRN}[PASS]{NC}  {msg}")
def fail(msg): print(f"{RED}[FAIL]{NC}  {msg}")
def info(msg): print(f"{CYN}[INFO]{NC}  {msg}")
def warn(msg): print(f"{YLW}[WARN]{NC}  {msg}")

COLLECT_SECS = 5.0          # how long to collect samples
MIN_SAMPLES  = 20           # minimum samples to consider a valid test

# ── helpers ───────────────────────────────────────────────────────────────────

def quat_to_euler(x, y, z, w):
    """Return (roll, pitch, yaw) in degrees from a unit quaternion."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def stdev(values):
    if len(values) < 2:
        return 0.0
    mean = sum(values) / len(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))

def fmt_vec3(label, x, y, z, unit=""):
    return f"  {label:<20s}  x={x:+8.4f}  y={y:+8.4f}  z={z:+8.4f}  {unit}"

# ── ROS node ──────────────────────────────────────────────────────────────────

class ImuTester(Node):
    def __init__(self):
        super().__init__("imu_tester")
        self.imu_samples = []
        self.mag_samples = []

        self.create_subscription(Imu, "/imu", self._imu_cb, 20)
        self.create_subscription(MagneticField, "/magnetic_field", self._mag_cb, 20)

    def _imu_cb(self, msg):
        self.imu_samples.append(msg)

    def _mag_cb(self, msg):
        self.mag_samples.append(msg)

    def collect(self, duration=COLLECT_SECS):
        info(f"Collecting IMU samples for {duration:.0f} s …")
        t0 = time.time()
        while time.time() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.05)
        info(f"Collected {len(self.imu_samples)} IMU samples"
             f" and {len(self.mag_samples)} mag samples.")


# ── analysis ──────────────────────────────────────────────────────────────────

def analyse(imu_samples, mag_samples):
    results_ok = True

    if len(imu_samples) < MIN_SAMPLES:
        fail(f"Only {len(imu_samples)} samples — is bringup running?  "
             f"Need ≥ {MIN_SAMPLES}.")
        return False

    # ── Latest sample snapshot ────────────────────────────────────────────────
    last = imu_samples[-1]
    ox, oy, oz, ow = (last.orientation.x, last.orientation.y,
                      last.orientation.z, last.orientation.w)
    ax, ay, az = (last.linear_acceleration.x,
                  last.linear_acceleration.y,
                  last.linear_acceleration.z)
    gx, gy, gz = (last.angular_velocity.x,
                  last.angular_velocity.y,
                  last.angular_velocity.z)

    roll, pitch, yaw = quat_to_euler(ox, oy, oz, ow)
    accel_mag = math.sqrt(ax**2 + ay**2 + az**2)

    print()
    print(f"{BLD}── Latest IMU reading ──────────────────────────────────────{NC}")
    print(fmt_vec3("Orientation (quat)", ox, oy, oz, f"  w={ow:+.4f}"))
    print(f"  {'Euler (deg)':<20s}  roll={roll:+7.2f}°  pitch={pitch:+7.2f}°  yaw={yaw:+7.2f}°")
    print(fmt_vec3("Linear accel", ax, ay, az, "m/s²"))
    print(f"  {'|accel| magnitude':<20s}  {accel_mag:.4f} m/s²  (expected ~9.81)")
    print(fmt_vec3("Angular velocity", gx, gy, gz, "rad/s"))

    # ── Magnetometer ─────────────────────────────────────────────────────────
    print()
    print(f"{BLD}── Magnetometer ────────────────────────────────────────────{NC}")
    if mag_samples:
        m = mag_samples[-1]
        mx, my, mz = m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z
        mag_field_mag = math.sqrt(mx**2 + my**2 + mz**2)
        mag_ut = mag_field_mag * 1e6
        print(f"  {'Mag field (T)':<20s}  x={mx:+.2e}  y={my:+.2e}  z={mz:+.2e}  T")
        print(f"  {'|field| magnitude':<20s}  {mag_ut:.2f} µT  (expected 25–65 µT on Earth)")
        if mag_ut < 1.0:
            fail(f"Mag field is ~0 µT — BNO085 magnetometer not reporting data.  "
                 f"Check report ID / firmware.")
            results_ok = False
        elif 25.0 <= mag_ut <= 65.0:
            ok(f"|mag| = {mag_ut:.2f} µT is within Earth's typical field range (25–65 µT).")
        else:
            warn(f"|mag| = {mag_ut:.2f} µT is outside typical Earth range (25–65 µT) — "
                 f"check for magnetic interference or mount location.")
    else:
        fail("/magnetic_field not published — BNO085 mag may not be initialised.")
        results_ok = False

    # ── Variance analysis (real vs simulated) ─────────────────────────────────
    print()
    print(f"{BLD}── Variance analysis (real vs simulated) ───────────────────{NC}")

    qs_w   = [s.orientation.w        for s in imu_samples]
    qs_x   = [s.orientation.x        for s in imu_samples]
    qs_y   = [s.orientation.y        for s in imu_samples]
    qs_z   = [s.orientation.z        for s in imu_samples]
    acc_x  = [s.linear_acceleration.x for s in imu_samples]
    acc_y  = [s.linear_acceleration.y for s in imu_samples]
    acc_z  = [s.linear_acceleration.z for s in imu_samples]
    gyro_x = [s.angular_velocity.x    for s in imu_samples]
    gyro_y = [s.angular_velocity.y    for s in imu_samples]
    gyro_z = [s.angular_velocity.z    for s in imu_samples]

    sd_qw    = stdev(qs_w)
    sd_gyrox = stdev(gyro_x)
    sd_ax    = stdev(acc_x)

    # The simulated IMU is perfectly constant — std-dev will be exactly 0.
    # A real BNO085 will always show tiny noise even when stationary.
    NOISE_THRESHOLD = 1e-6

    is_real = (sd_qw > NOISE_THRESHOLD or
               sd_gyrox > NOISE_THRESHOLD or
               sd_ax > NOISE_THRESHOLD)

    print(f"  Quaternion W    stdev = {sd_qw:.2e}")
    print(f"  Gyro X          stdev = {sd_gyrox:.2e}")
    print(f"  Accel X         stdev = {sd_ax:.2e}")
    print()

    if is_real:
        ok("IMU data shows sensor noise — BNO085 hardware is ACTIVE (not simulated).")
    else:
        fail("IMU data is perfectly constant — firmware is using the SIMULATED fallback."
             "  Check I2C wiring and I2C address (SA0 → GND for 0x4A).")
        results_ok = False

    # Extra checks: gyro and rotation vector frozen suggests cargo-parser bug.
    # NOTE: a stationary robot with good bias calibration legitimately produces
    # gyro = 0 (Q9 resolution = 0.002 rad/s; any rate below that rounds to 0).
    # Only flag as broken if mag is ALSO zero — that indicates the cargo parser
    # is still breaking early on an unrecognised report ID before gyro is parsed.
    mag_working = bool(mag_samples and
                       math.sqrt(mag_samples[-1].magnetic_field.x**2 +
                                 mag_samples[-1].magnetic_field.y**2 +
                                 mag_samples[-1].magnetic_field.z**2) * 1e6 > 1.0)

    if sd_gyrox < NOISE_THRESHOLD and sd_ax > NOISE_THRESHOLD:
        if not mag_working:
            fail("Gyro is frozen at zero AND mag is zero — cargo parser is "
                 "likely stopping early (wrong report ID before gyro in packet).")
            results_ok = False
        else:
            info("Gyro output is zero — expected for a stationary calibrated sensor "
                 "(Q9 resolution = 0.002 rad/s; real output below threshold rounds to 0).")
    elif sd_gyrox > NOISE_THRESHOLD:
        ok("Gyro shows sensor noise — gyroscope is ACTIVE.")

    qw_constant = (stdev(qs_w) < NOISE_THRESHOLD and
                   stdev(qs_x) < NOISE_THRESHOLD and
                   stdev(qs_y) < NOISE_THRESHOLD and
                   stdev(qs_z) < NOISE_THRESHOLD)
    if qw_constant and is_real:
        if not mag_working:
            fail("Rotation vector is frozen at identity AND mag is zero — "
                 "rotation vector report not being parsed (check firmware).")
            results_ok = False
        else:
            info("Rotation vector is identity — expected for a freshly-booted stationary "
                 "sensor (9DOF fusion needs mag calibration to converge; tilt robot to verify).")
    elif not qw_constant:
        ok("Rotation vector is updating — orientation fusion is ACTIVE.")

    # ── Gravity magnitude check ───────────────────────────────────────────────
    print()
    print(f"{BLD}── Gravity check ───────────────────────────────────────────{NC}")
    accel_mags = [math.sqrt(s.linear_acceleration.x**2 +
                            s.linear_acceleration.y**2 +
                            s.linear_acceleration.z**2) for s in imu_samples]
    mean_mag = sum(accel_mags) / len(accel_mags)
    print(f"  Mean |accel| over {len(imu_samples)} samples = {mean_mag:.4f} m/s²")

    if 9.0 < mean_mag < 10.6:
        ok(f"|accel| = {mean_mag:.4f} m/s²  is within expected gravity range (9.0–10.6).")
    else:
        fail(f"|accel| = {mean_mag:.4f} m/s²  is OUTSIDE expected gravity range (9.0–10.6).")
        results_ok = False

    # ── Covariance check ─────────────────────────────────────────────────────
    print()
    print(f"{BLD}── Covariance matrices ─────────────────────────────────────{NC}")
    cov_o  = list(last.orientation_covariance)
    cov_av = list(last.angular_velocity_covariance)
    cov_la = list(last.linear_acceleration_covariance)
    print(f"  orientation_covariance[0,4,8]     = "
          f"{cov_o[0]:.3e}, {cov_o[4]:.3e}, {cov_o[8]:.3e}")
    print(f"  angular_velocity_covariance[0,4,8]= "
          f"{cov_av[0]:.3e}, {cov_av[4]:.3e}, {cov_av[8]:.3e}")
    print(f"  linear_accel_covariance[0,4,8]    = "
          f"{cov_la[0]:.3e}, {cov_la[4]:.3e}, {cov_la[8]:.3e}")

    # ── Rate check ────────────────────────────────────────────────────────────
    if len(imu_samples) >= 2:
        t_first = imu_samples[0].header.stamp.sec  + imu_samples[0].header.stamp.nanosec * 1e-9
        t_last  = imu_samples[-1].header.stamp.sec + imu_samples[-1].header.stamp.nanosec * 1e-9
        span = t_last - t_first
        if span > 0:
            rate = (len(imu_samples) - 1) / span
            print()
            print(f"{BLD}── Publish rate ────────────────────────────────────────────{NC}")
            print(f"  Effective /imu publish rate: {rate:.1f} Hz")
            if rate < 10.0:
                warn(f"Rate is low ({rate:.1f} Hz).  Expected ≈ 50 Hz.")
            else:
                ok(f"Rate {rate:.1f} Hz looks good (expected ≈ 50 Hz).")

    return results_ok


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    print()
    print(f"{BLD}{'='*60}{NC}")
    print(f"{BLD}  BNO085 IMU Test  (Grove 1 / I2C0 / GP0=SDA / GP1=SCL){NC}")
    print(f"{BLD}{'='*60}{NC}")

    rclpy.init()
    node = ImuTester()

    node.collect(COLLECT_SECS)

    passed = analyse(node.imu_samples, node.mag_samples)

    node.destroy_node()
    rclpy.shutdown()

    print()
    if passed:
        print(f"{GRN}{BLD}All checks passed — BNO085 is connected and working.{NC}")
        sys.exit(0)
    else:
        print(f"{RED}{BLD}One or more checks FAILED — see output above.{NC}")
        sys.exit(1)


if __name__ == "__main__":
    main()
