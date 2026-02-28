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

Usage
-----
  # Dry-run (measure and report only):
  python3 auto_calibrate_imu_turn.py

  # Apply to files:
  python3 auto_calibrate_imu_turn.py --apply

  # Apply + rebuild + flash + restart service:
  python3 auto_calibrate_imu_turn.py --apply --flash

  # Custom test parameters:
  python3 auto_calibrate_imu_turn.py --speed 0.4 --angle 180 --runs 3 --apply
"""

import argparse
import math
import re
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu


SCRIPT_DIR   = Path(__file__).resolve().parent
MAIN_C       = SCRIPT_DIR / "firmware" / "main.c"
YAML_PATH    = SCRIPT_DIR / "config" / "burger_pico.yaml"
FIRMWARE_DIR = SCRIPT_DIR / "firmware"
BUILD_SCRIPT = FIRMWARE_DIR / "build.sh"

# Angular velocity variance threshold to distinguish real BNO085 from simulated
REAL_IMU_GYRO_VAR_THRESHOLD = 1e-5  # rad^2/s^2


# ─── file I/O helpers ────────────────────────────────────────────────────────

def read_define_float(path: Path, name: str) -> float | None:
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
    def __init__(self):
        super().__init__("turn_calibrator")
        self.pub          = self.create_publisher(Twist, "/cmd_vel", 10)
        self.last_yaw     = None
        self.cum_yaw      = 0.0
        self.collecting   = False
        self._gyro_samples: list[float] = []
        self._gyro_collecting = False
        self.create_subscription(Imu, "/imu", self._imu_cb, 30)

    def _imu_cb(self, msg: Imu):
        yaw = quat_to_yaw(msg)
        if self.last_yaw is not None and self.collecting:
            self.cum_yaw += angle_diff(yaw, self.last_yaw)
        self.last_yaw = yaw
        if self._gyro_collecting:
            self._gyro_samples.append(msg.angular_velocity.z)

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
    args = parser.parse_args()

    if args.flash and not args.apply:
        print("ERROR: --flash requires --apply")
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
    print("Safety          : robot should be able to spin freely\n")

    rclpy.init()
    node = TurnCalibrator()

    if not node.wait_for_imu():
        print("ERROR: /imu not available. Start bringup first.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # Detect whether IMU is giving real sensor data
    real_imu = node.detect_real_imu(speed=min(args.speed, 0.5), duration=1.5)
    print()

    measurements = []

    if real_imu:
        # ── IMU-based automatic measurement ──────────────────────────────────
        for i in range(args.runs):
            print(f"Run {i+1}/{args.runs} — rotating {args.angle:.0f} deg (IMU measurement)...")
            measured = abs(node.measure_rotation_imu(args.speed, target_rad))
            measurements.append(measured)
            print(f"  IMU measured : {math.degrees(measured):.1f} deg  "
                  f"(desired {args.angle:.0f} deg, ratio {target_rad/measured:.4f})")
            if i < args.runs - 1:
                time.sleep(0.8)
    else:
        # ── Manual measurement fallback ───────────────────────────────────────
        print("WARNING: Simulated IMU detected — using manual measurement mode.")
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
            print(f"  Recorded: {measured_deg:.1f} deg  ratio={target_rad/measured:.4f}")
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

    # new_sep = old_sep * (desired / measured)
    # Over-rotation  (measured > desired) -> decrease sep
    # Under-rotation (measured < desired) -> increase sep
    new_sep = old_sep * (target_rad / avg_measured)
    new_sep = max(0.05, min(0.50, new_sep))   # sanity clamp 5-50 cm

    print("\n--- Result ---------------------------------------------------")
    print(f"Avg measured    : {math.degrees(avg_measured):.1f} deg")
    print(f"Desired         : {args.angle:.1f} deg")
    print(f"old WHEEL_SEP   : {old_sep*100:.4f} cm  ({old_sep:.6f} m)")
    print(f"new WHEEL_SEP   : {new_sep*100:.4f} cm  ({new_sep:.6f} m)")
    print(f"Formula         : new = {old_sep:.6f} * ({args.angle:.1f} / {math.degrees(avg_measured):.1f})")

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
            print("Restarting turtlebot3 service...")
            r = subprocess.run(["sudo", "systemctl", "restart", "turtlebot3"])
            if r.returncode != 0:
                print(f"ERROR: service restart failed (exit {r.returncode})")
                return r.returncode
            print("Done — firmware flashed and service restarted.")
        else:
            print("\nNext steps: rebuild/flash firmware, then restart service:")
            print("  cd turtlebot3_pico/firmware && ./build.sh flash")
            print("  sudo systemctl restart turtlebot3")
    else:
        print("\nDry-run: no files changed. Use --apply to write changes.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
