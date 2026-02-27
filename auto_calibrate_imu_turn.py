#!/usr/bin/env python3
"""
Auto-calibrate TurtleBot3 Pico angular command scale using IMU yaw.

What it does
------------
1) Sends an in-place rotation command (/cmd_vel angular.z)
2) Integrates IMU yaw change during the test
3) Computes corrected ANGULAR_CMD_SCALE:
     new_scale = old_scale * (desired_yaw / measured_yaw)
4) Optionally writes it into firmware/main.c

Usage
-----
  python3 auto_calibrate_imu_turn.py
  python3 auto_calibrate_imu_turn.py --speed 0.5 --duration 4.0 --runs 3 --apply

Notes
-----
- Best done with wheels off the floor (safe bench calibration), then verify on ground.
- Requires bringup running so /imu and /cmd_vel are available.
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


SCRIPT_DIR = Path(__file__).resolve().parent
MAIN_C = SCRIPT_DIR / "firmware" / "main.c"
FIRMWARE_DIR = SCRIPT_DIR / "firmware"
BUILD_SCRIPT = FIRMWARE_DIR / "build.sh"


def angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def quat_to_yaw(msg: Imu) -> float:
    o = msg.orientation
    siny = 2.0 * (o.w * o.z + o.x * o.y)
    cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    return math.atan2(siny, cosy)


def read_define_float(path: Path, name: str) -> float | None:
    pattern = re.compile(rf"^\s*#define\s+{re.escape(name)}\s+([\d.+\-eEfF]+)")
    for line in path.read_text().splitlines():
        match = pattern.match(line)
        if match:
            return float(match.group(1).rstrip("fF"))
    return None


def write_define_float(path: Path, name: str, value: float) -> None:
    content = path.read_text()
    pattern = re.compile(rf"(#define\s+{re.escape(name)}\s+)[\d.+\-eEfF]+f?([ \t]*(?://[^\n]*)?)")

    def repl(match):
        return f"{match.group(1)}{value:.6f}f{match.group(2)}"

    new_content, count = pattern.subn(repl, content, count=1)
    if count == 0:
        raise RuntimeError(f"Could not find #define {name} in {path}")
    path.write_text(new_content)


class ImuTurnCalibrator(Node):
    def __init__(self):
        super().__init__("imu_turn_calibrator")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.last_yaw = None
        self.cum_yaw = 0.0
        self.collecting = False
        self.create_subscription(Imu, "/imu", self._imu_cb, 30)

    def _imu_cb(self, msg: Imu):
        yaw = quat_to_yaw(msg)
        if self.last_yaw is not None and self.collecting:
            self.cum_yaw += angle_diff(yaw, self.last_yaw)
        self.last_yaw = yaw

    def send(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)

    def stop(self, settle_sec: float = 0.4):
        self.send(0.0, 0.0)
        end = time.time() + settle_sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_imu(self, timeout: float = 8.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_yaw is not None:
                return True
        return False

    def measure_run(self, speed: float, duration: float) -> float:
        self.cum_yaw = 0.0
        self.collecting = True
        end = time.time() + duration
        while time.time() < end:
            self.send(0.0, speed)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.collecting = False
        self.stop(0.5)
        return self.cum_yaw


def main():
    parser = argparse.ArgumentParser(description="IMU-based ANGULAR_CMD_SCALE auto-calibration")
    parser.add_argument("--speed", type=float, default=0.5, help="angular.z command (rad/s)")
    parser.add_argument("--duration", type=float, default=4.0, help="seconds per run")
    parser.add_argument("--runs", type=int, default=3, help="number of calibration runs")
    parser.add_argument("--apply", action="store_true", help="write updated ANGULAR_CMD_SCALE to firmware/main.c")
    parser.add_argument("--flash", action="store_true", help="after --apply, rebuild/flash firmware and restart turtlebot3 service")
    args = parser.parse_args()

    if args.flash and not args.apply:
        print("ERROR: --flash requires --apply")
        return 4

    old_scale = read_define_float(MAIN_C, "ANGULAR_CMD_SCALE")
    if old_scale is None:
        print(f"ERROR: ANGULAR_CMD_SCALE not found in {MAIN_C}")
        return 2

    desired = abs(args.speed) * args.duration

    print("\n=== IMU Turn Auto-Calibration ===")
    print(f"Firmware file : {MAIN_C}")
    print(f"Current scale : {old_scale:.6f}")
    print(f"Command       : angular.z={args.speed:.3f} rad/s, duration={args.duration:.2f}s")
    print(f"Desired yaw   : {math.degrees(desired):.1f} deg per run")
    print("Safety        : keep robot lifted/off floor for calibration run\n")

    rclpy.init()
    node = ImuTurnCalibrator()

    if not node.wait_for_imu():
        print("ERROR: /imu not available. Start bringup first.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    measurements = []
    for i in range(args.runs):
        print(f"Run {i+1}/{args.runs}...")
        measured = abs(node.measure_run(args.speed, args.duration))
        measurements.append(measured)
        ratio = (desired / measured) if measured > 1e-6 else float("inf")
        print(f"  measured yaw: {math.degrees(measured):.1f} deg, ratio desired/measured: {ratio:.4f}")
        time.sleep(0.4)

    node.stop(0.2)
    node.destroy_node()
    rclpy.shutdown()

    avg_measured = sum(measurements) / len(measurements)
    if avg_measured < 1e-6:
        print("ERROR: measured yaw too small; cannot calibrate.")
        return 3

    new_scale = old_scale * (desired / avg_measured)
    new_scale = max(0.05, min(5.0, new_scale))

    print("\n--- Result ---")
    print(f"Avg measured yaw : {math.degrees(avg_measured):.1f} deg")
    print(f"Suggested scale  : {new_scale:.6f}")
    print("Formula          : new = old * (desired/measured)")

    if args.apply:
        write_define_float(MAIN_C, "ANGULAR_CMD_SCALE", new_scale)
        print(f"Applied          : ANGULAR_CMD_SCALE={new_scale:.6f} in {MAIN_C}")
        print("Next             : rebuild/flash firmware, then retest")

        if args.flash:
            print("\n--- Flash ---")
            if not BUILD_SCRIPT.exists():
                print(f"ERROR: build script not found: {BUILD_SCRIPT}")
                return 5

            print(f"Running          : {BUILD_SCRIPT} flash")
            flash = subprocess.run(["bash", str(BUILD_SCRIPT), "flash"], cwd=str(FIRMWARE_DIR))
            if flash.returncode != 0:
                print(f"ERROR: flash failed with exit code {flash.returncode}")
                return flash.returncode

            print("Running          : sudo systemctl restart turtlebot3")
            restart = subprocess.run(["sudo", "systemctl", "restart", "turtlebot3"])
            if restart.returncode != 0:
                print(f"ERROR: service restart failed with exit code {restart.returncode}")
                return restart.returncode

            print("Done             : flashed firmware and restarted turtlebot3 service")
    else:
        print("Dry-run          : no file changes (use --apply to write)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
