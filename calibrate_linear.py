#!/usr/bin/env python3
"""
Calibrate TurtleBot3 Pico linear distance accuracy via MAX_WHEEL_SPEED_MS.

How it works
------------
In the open-loop firmware:

    duty = commanded_velocity / MAX_WHEEL_SPEED_MS

If MAX_WHEEL_SPEED_MS is too LOW, the duty is too HIGH and the robot
physically travels further than commanded.  The correction is:

    new_MAX_WHEEL_SPEED_MS = old × (measured / desired)

Example: commanded 10 cm, robot moved 18 cm
    new = 0.117 × (18 / 10) = 0.2106

NOTE: odometry is derived from commanded velocity (open-loop, no encoders),
so it is always accurate to the command.  This calibration corrects the
PHYSICAL motion, making it match the command.

Usage
-----
  # Dry-run: robot moves, you measure, script prints the correction
  python3 calibrate_linear.py

  # Apply directly to firmware/main.c:
  python3 calibrate_linear.py --apply

  # Apply + rebuild/flash + restart service:
  python3 calibrate_linear.py --apply --flash

  # Provide your own measurement (no motion):
  python3 calibrate_linear.py --desired 0.10 --measured 0.18 --apply

  # Override test distance or speed:
  python3 calibrate_linear.py --distance 0.30 --speed 0.10
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


SCRIPT_DIR   = Path(__file__).resolve().parent
MAIN_C       = SCRIPT_DIR / "firmware" / "main.c"
FIRMWARE_DIR = SCRIPT_DIR / "firmware"
BUILD_SCRIPT = FIRMWARE_DIR / "build.sh"


# ─── file helpers ─────────────────────────────────────────────────────────────

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


# ─── ROS node ─────────────────────────────────────────────────────────────────

class LinearCalibrator(Node):
    def __init__(self):
        super().__init__("linear_calibrator")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def send(self, lin: float, ang: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self.pub.publish(msg)

    def stop(self, settle: float = 0.5) -> None:
        self.send(0.0)
        end = time.time() + settle
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive_distance(self, speed: float, distance: float) -> None:
        """Drive forward at `speed` m/s for distance/speed seconds."""
        duration = abs(distance / speed)
        end = time.time() + duration
        while time.time() < end:
            self.send(speed)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.stop(0.6)


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Calibrate MAX_WHEEL_SPEED_MS for linear distance accuracy"
    )
    parser.add_argument("--distance", type=float, default=0.30,
                        help="test drive distance in metres (default 0.30)")
    parser.add_argument("--speed",    type=float, default=0.10,
                        help="drive speed in m/s (default 0.10)")
    parser.add_argument("--runs",     type=int,   default=3,
                        help="number of runs to average (default 3)")
    parser.add_argument("--desired",  type=float, default=None,
                        help="skip driving; provide commanded distance in m")
    parser.add_argument("--measured", type=float, default=None,
                        help="skip driving; provide physically-measured distance in m")
    parser.add_argument("--apply",    action="store_true",
                        help="write new MAX_WHEEL_SPEED_MS to firmware/main.c")
    parser.add_argument("--flash",    action="store_true",
                        help="after --apply, rebuild/flash firmware and restart service")
    args = parser.parse_args()

    if args.flash and not args.apply:
        print("ERROR: --flash requires --apply")
        return 4

    old_max = read_define_float(MAIN_C, "MAX_WHEEL_SPEED_MS")
    if old_max is None:
        print(f"ERROR: MAX_WHEEL_SPEED_MS not found in {MAIN_C}")
        return 2

    print("\n=== Linear Distance Calibration (MAX_WHEEL_SPEED_MS) ===")
    print(f"Firmware          : {MAIN_C}")
    print(f"Current MAX_SPEED : {old_max:.6f} m/s")

    # ── offline / manual path ─────────────────────────────────────────────────
    if args.desired is not None and args.measured is not None:
        desired_m  = args.desired
        measured_m = args.measured
        print(f"Offline mode      : desired={desired_m*100:.1f} cm, "
              f"measured={measured_m*100:.1f} cm\n")
    else:
        # ── live drive path ──────────────────────────────────────────────────
        print(f"Test distance     : {args.distance*100:.0f} cm at {args.speed:.2f} m/s  "
              f"({args.runs} run(s))")
        print("Safety            : clear space in front of robot\n")

        rclpy.init()
        node = LinearCalibrator()
        measurements = []

        for i in range(args.runs):
            input(f"Run {i+1}/{args.runs}: mark robot START position, then press Enter... ")
            print(f"  Driving {args.distance*100:.0f} cm...")
            node.drive_distance(args.speed, args.distance)
            raw = input("  Measure actual distance travelled in cm: ")
            try:
                m_cm = float(raw.strip())
            except ValueError:
                print("  Invalid input — skipping this run.")
                continue
            measurements.append(m_cm / 100.0)
            print(f"  Recorded: {m_cm:.1f} cm  ratio={m_cm/100.0/args.distance:.4f}")
            if i < args.runs - 1:
                time.sleep(0.5)

        node.destroy_node()
        rclpy.shutdown()

        if not measurements:
            print("ERROR: no valid measurements collected.")
            return 3

        desired_m  = args.distance
        measured_m = sum(measurements) / len(measurements)
        print(f"\nAvg measured      : {measured_m*100:.2f} cm")
        print(f"Desired           : {desired_m*100:.1f} cm")

    if measured_m < 0.005:
        print("ERROR: measured distance too small to calibrate reliably.")
        return 3

    # new = old × (measured / desired)
    # Over-travel  (measured > desired) → increase MAX_WHEEL_SPEED_MS (lower duty → slower)
    # Under-travel (measured < desired) → decrease MAX_WHEEL_SPEED_MS (higher duty → faster)
    new_max = old_max * (measured_m / desired_m)
    new_max = max(0.05, min(2.0, new_max))  # sanity clamp

    print("\n--- Result ---------------------------------------------------")
    print(f"old MAX_WHEEL_SPEED_MS : {old_max:.6f} m/s")
    print(f"new MAX_WHEEL_SPEED_MS : {new_max:.6f} m/s")
    print(f"Formula               : new = {old_max:.6f} * "
          f"({measured_m*100:.2f} / {desired_m*100:.1f})")

    if args.apply:
        comment = (
            f"calibrated: robot moved {measured_m*100:.1f} cm "
            f"when commanded {desired_m*100:.1f} cm"
        )
        write_define_float(MAIN_C, "MAX_WHEEL_SPEED_MS", new_max, comment)
        print(f"\nApplied to        : {MAIN_C.relative_to(SCRIPT_DIR)}")

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
