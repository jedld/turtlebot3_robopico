#!/usr/bin/env python3
"""
Auto-calibrate MOTOR_MIN_DUTY (dead-zone compensation) and MOTOR_KICK_DUTY
using the BNO085 IMU to detect when wheels are actually moving.

Strategy
--------
1. Disable kick-start (KICK_CYCLES=0) so we only test the steady-state floor.
2. Set MIN_DUTY to a low value and command a slow rotation.
3. If the IMU reads near-zero angular velocity → motors stalled.
4. Binary-search upward until both motors reliably move.
5. Add a configurable safety margin.
6. Optionally set KICK_DUTY = discovered MIN_DUTY + small boost.

The test deliberately commands small angular velocities (pivot turns)
because that's where stall is most likely — one wheel goes forward, the
other goes backward, each at a tiny fraction of max speed.

Usage
-----
  # Dry-run (discover and report, don't persist):
  python3 calibrate_deadzone.py

  # Apply result to runtime + flash + source files:
  python3 calibrate_deadzone.py --apply

  # Custom parameters:
  python3 calibrate_deadzone.py --margin 0.06 --speed 0.3 --apply
"""

import argparse
import math
import re
import struct
import time
from pathlib import Path
from typing import Optional

import serial

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu


# ── paths & protocol constants ────────────────────────────────────────────────

SCRIPT_DIR = Path(__file__).resolve().parent
MAIN_C     = SCRIPT_DIR / "firmware" / "main.c"

DEV_ID = 200
DXL_BAUD = 1_000_000
INST_CALIBRATION = 0x90

CALIB_CMD_SET  = 0x01
CALIB_CMD_SAVE = 0x04

CALIB_KEY_MOTOR_MIN_DUTY    = 0x04
CALIB_KEY_MOTOR_KICK_DUTY   = 0x05
CALIB_KEY_MOTOR_KICK_CYCLES = 0x06


# ── Dynamixel helpers ─────────────────────────────────────────────────────────

def _make_crc_table() -> list[int]:
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8005) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        table.append(crc)
    return table


_CRC_TABLE = _make_crc_table()


def _crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


def _build_instruction(dev_id: int, inst: int, params: bytes) -> bytes:
    pkt_len = len(params) + 3
    hdr = bytes([
        0xFF, 0xFF, 0xFD, 0x00,
        dev_id,
        pkt_len & 0xFF, (pkt_len >> 8) & 0xFF,
        inst,
    ])
    pkt = hdr + params
    crc = _crc16(pkt)
    return pkt + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def send_calib(port: str, subcmd: int, key: int | None = None,
               value: float | None = None) -> None:
    params = bytes([subcmd])
    if subcmd == CALIB_CMD_SET:
        if key is None or value is None:
            raise ValueError("SET requires key and value")
        params += bytes([key]) + struct.pack("<f", float(value))
    pkt = _build_instruction(0xFE, INST_CALIBRATION, params)
    with serial.Serial(port, DXL_BAUD, timeout=0.02, write_timeout=0.2) as ser:
        ser.write(pkt)
        ser.flush()
    time.sleep(0.02)


# ── source-file helpers ───────────────────────────────────────────────────────

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
    def repl(m):
        return f"{m.group(1)}{value:.2f}f{suffix}"
    new_content, count = pat.subn(repl, content, count=1)
    if count == 0:
        raise RuntimeError(f"Could not find #define {name} in {path}")
    path.write_text(new_content)


# ── ROS node ──────────────────────────────────────────────────────────────────

class DeadzoneProber(Node):
    """Publishes /cmd_vel and collects IMU angular velocity samples."""

    def __init__(self):
        super().__init__("deadzone_prober")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._gyro_z_samples: list[float] = []
        self._collecting = False
        self._imu_ok = False
        self.create_subscription(Imu, "/imu", self._imu_cb, 30)

    def _imu_cb(self, msg: Imu):
        self._imu_ok = True
        if self._collecting:
            self._gyro_z_samples.append(msg.angular_velocity.z)

    def wait_for_imu(self, timeout: float = 8.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._imu_ok:
                return True
        return False

    def send(self, ang_z: float) -> None:
        msg = Twist()
        msg.angular.z = ang_z
        self.pub.publish(msg)

    def stop(self, settle: float = 0.3) -> None:
        self.send(0.0)
        end = time.time() + settle
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def probe(self, ang_speed: float, duration: float = 1.5,
              settle_before: float = 0.3) -> float:
        """
        Command a rotation at `ang_speed` for `duration` seconds and return
        the mean absolute gyro-Z observed by the IMU (rad/s).

        A return value near 0 means the motors stalled.
        """
        # Settle first so previous motion doesn't contaminate
        self.stop(settle_before)

        self._gyro_z_samples.clear()
        self._collecting = True

        end = time.time() + duration
        while time.time() < end:
            self.send(ang_speed)
            rclpy.spin_once(self, timeout_sec=0.03)

        self._collecting = False
        self.stop(0.4)

        if not self._gyro_z_samples:
            return 0.0

        # Discard the first few samples (transition period)
        samples = self._gyro_z_samples
        if len(samples) > 6:
            samples = samples[3:]

        return sum(abs(s) for s in samples) / len(samples)


# ── calibration logic ─────────────────────────────────────────────────────────

def find_min_duty(
    node: DeadzoneProber,
    port: str,
    ang_speed: float,
    search_lo: float = 0.10,
    search_hi: float = 0.80,
    stall_threshold: float = 0.05,
    resolution: float = 0.02,
    probe_duration: float = 1.5,
    verbose: bool = True,
) -> float:
    """
    Binary-search for the lowest MOTOR_MIN_DUTY where the robot reliably
    rotates when commanded at `ang_speed` rad/s.

    Returns the duty value at which both wheels move (gyro > stall_threshold).
    """
    # First, disable kick so we only test the steady-state floor
    send_calib(port, CALIB_CMD_SET, CALIB_KEY_MOTOR_KICK_CYCLES, value=0.0)
    if verbose:
        print(f"  Kick-start disabled (KICK_CYCLES=0)")

    lo, hi = search_lo, search_hi

    # Phase 1: quick sweep to find a coarse bracket
    if verbose:
        print(f"\n  Phase 1: coarse sweep [{lo:.2f} … {hi:.2f}]")

    # Verify hi works at all
    send_calib(port, CALIB_CMD_SET, CALIB_KEY_MOTOR_MIN_DUTY, value=hi)
    time.sleep(0.05)
    rate = node.probe(ang_speed, duration=probe_duration)
    if verbose:
        print(f"    MIN_DUTY={hi:.2f}  →  gyro={rate:.3f} rad/s  "
              f"{'MOVING' if rate > stall_threshold else 'STALL'}")
    if rate <= stall_threshold:
        print(f"\n  ERROR: motors don't move even at MIN_DUTY={hi:.2f}.")
        print(f"  Check wiring, battery voltage, or increase --search-hi.")
        return hi

    # Verify lo stalls (otherwise the stall point is below our range)
    send_calib(port, CALIB_CMD_SET, CALIB_KEY_MOTOR_MIN_DUTY, value=lo)
    time.sleep(0.05)
    rate = node.probe(ang_speed, duration=probe_duration)
    if verbose:
        print(f"    MIN_DUTY={lo:.2f}  →  gyro={rate:.3f} rad/s  "
              f"{'MOVING' if rate > stall_threshold else 'STALL'}")
    if rate > stall_threshold:
        if verbose:
            print(f"  Motors move even at {lo:.2f} — stall point is below range.")
        return lo

    # Phase 2: binary search
    if verbose:
        print(f"\n  Phase 2: binary search (resolution={resolution:.2f})")

    iterations = 0
    while (hi - lo) > resolution and iterations < 20:
        mid = (lo + hi) / 2.0
        send_calib(port, CALIB_CMD_SET, CALIB_KEY_MOTOR_MIN_DUTY, value=mid)
        time.sleep(0.05)

        rate = node.probe(ang_speed, duration=probe_duration)
        moving = rate > stall_threshold
        if verbose:
            print(f"    MIN_DUTY={mid:.2f}  →  gyro={rate:.3f} rad/s  "
                  f"{'MOVING' if moving else 'STALL'}")

        if moving:
            hi = mid
        else:
            lo = mid
        iterations += 1

    # The stall threshold is at `hi` (lowest value that reliably moves)
    if verbose:
        print(f"\n  Stall threshold: {hi:.2f}")

    return hi


def verify_duty(node: DeadzoneProber, port: str, duty: float,
                ang_speed: float, trials: int = 3,
                stall_threshold: float = 0.05,
                verbose: bool = True) -> bool:
    """Run multiple trials to confirm the motor reliably starts at `duty`."""
    send_calib(port, CALIB_CMD_SET, CALIB_KEY_MOTOR_MIN_DUTY, value=duty)
    time.sleep(0.05)
    successes = 0
    for t in range(trials):
        # Alternate rotation direction to test both motors in both roles
        direction = 1.0 if t % 2 == 0 else -1.0
        rate = node.probe(direction * ang_speed, duration=1.2)
        ok = rate > stall_threshold
        if verbose:
            print(f"    Trial {t+1}/{trials}  dir={'CCW' if direction > 0 else 'CW '}  "
                  f"gyro={rate:.3f}  {'OK' if ok else 'STALL'}")
        if ok:
            successes += 1
    return successes == trials


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Auto-calibrate MOTOR_MIN_DUTY (dead-zone compensation) "
                    "and MOTOR_KICK_DUTY using the BNO085 IMU."
    )
    parser.add_argument("--speed", type=float, default=0.3,
                        help="angular.z test speed in rad/s (default 0.3)")
    parser.add_argument("--margin", type=float, default=0.05,
                        help="safety margin added above the stall threshold "
                             "(default 0.05 = 5%%)")
    parser.add_argument("--kick-margin", type=float, default=0.05,
                        help="KICK_DUTY = MIN_DUTY + this margin (default 0.05)")
    parser.add_argument("--kick-cycles", type=int, default=3,
                        help="KICK_CYCLES to set (default 3; 0 to disable kick)")
    parser.add_argument("--stall-threshold", type=float, default=0.05,
                        help="IMU gyro-Z below this (rad/s) = stall (default 0.05)")
    parser.add_argument("--search-lo", type=float, default=0.10,
                        help="lowest MIN_DUTY to try (default 0.10)")
    parser.add_argument("--search-hi", type=float, default=0.80,
                        help="highest MIN_DUTY to try (default 0.80)")
    parser.add_argument("--resolution", type=float, default=0.02,
                        help="binary search step size (default 0.02)")
    parser.add_argument("--apply", action="store_true",
                        help="persist to flash + source files")
    parser.add_argument("--dxl-port", default="/dev/ttyTB3", metavar="PORT",
                        help="Dynamixel serial port (default: /dev/ttyTB3)")
    args = parser.parse_args()

    # Read current values from source
    cur_min_duty  = read_define_float(MAIN_C, "MOTOR_MIN_DUTY_DEFAULT")
    cur_kick_duty = read_define_float(MAIN_C, "MOTOR_KICK_DUTY_DEFAULT")
    cur_kick_cyc  = read_define_float(MAIN_C, "MOTOR_KICK_CYCLES_DEFAULT")

    print("\n=== Dead-Zone Compensation Calibration ===")
    print(f"Firmware        : {MAIN_C}")
    print(f"Current         : MIN_DUTY={cur_min_duty}  KICK_DUTY={cur_kick_duty}  "
          f"KICK_CYCLES={cur_kick_cyc}")
    print(f"Test speed      : {args.speed:.2f} rad/s angular")
    print(f"Stall threshold : {args.stall_threshold:.3f} rad/s gyro-Z")
    print(f"Search range    : [{args.search_lo:.2f} … {args.search_hi:.2f}]  "
          f"resolution={args.resolution:.2f}")
    print(f"Safety margin   : {args.margin:.2f}")
    print()

    rclpy.init()
    node = DeadzoneProber()

    if not node.wait_for_imu():
        print("ERROR: /imu not available. Start turtlebot3 bringup first.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print("IMU detected — starting calibration.\n")
    print("─" * 56)
    print("Step 1: Find stall threshold via binary search")
    print("─" * 56)

    stall_duty = find_min_duty(
        node, args.dxl_port,
        ang_speed=args.speed,
        search_lo=args.search_lo,
        search_hi=args.search_hi,
        stall_threshold=args.stall_threshold,
        resolution=args.resolution,
    )

    new_min_duty = min(stall_duty + args.margin, 0.95)
    new_kick_duty = min(new_min_duty + args.kick_margin, 0.95)
    new_kick_cycles = args.kick_cycles

    print()
    print("─" * 56)
    print(f"Step 2: Verify MIN_DUTY={new_min_duty:.2f} across multiple trials")
    print("─" * 56)

    # Apply the candidate and verify
    send_calib(args.dxl_port, CALIB_CMD_SET,
               CALIB_KEY_MOTOR_KICK_CYCLES, value=float(new_kick_cycles))
    send_calib(args.dxl_port, CALIB_CMD_SET,
               CALIB_KEY_MOTOR_KICK_DUTY, value=new_kick_duty)

    ok = verify_duty(node, args.dxl_port, new_min_duty,
                     ang_speed=args.speed, trials=4,
                     stall_threshold=args.stall_threshold)

    if not ok:
        print(f"\n  Verification failed — bumping MIN_DUTY by extra {args.margin:.2f}")
        new_min_duty = min(new_min_duty + args.margin, 0.95)
        new_kick_duty = min(new_min_duty + args.kick_margin, 0.95)
        ok = verify_duty(node, args.dxl_port, new_min_duty,
                         ang_speed=args.speed, trials=4,
                         stall_threshold=args.stall_threshold)
        if not ok:
            print("\n  WARNING: Still failing verification. "
                  "Applying anyway — you may need to tune manually.")

    node.stop(0.3)
    node.destroy_node()
    rclpy.shutdown()

    # ── results ───────────────────────────────────────────────────────────
    print()
    print("─" * 56)
    print("Result")
    print("─" * 56)
    print(f"  Stall threshold  : {stall_duty:.2f}")
    print(f"  Safety margin    : {args.margin:.2f}")
    print(f"  MOTOR_MIN_DUTY   : {cur_min_duty:.2f} → {new_min_duty:.2f}")
    print(f"  MOTOR_KICK_DUTY  : {cur_kick_duty:.2f} → {new_kick_duty:.2f}")
    print(f"  MOTOR_KICK_CYCLES: {cur_kick_cyc} → {new_kick_cycles}")
    print()

    if args.apply:
        # Runtime already has the values from the verification step.
        # Persist to flash.
        try:
            send_calib(args.dxl_port, CALIB_CMD_SAVE)
            print("Persisted       : saved to firmware flash")
        except Exception as e:
            print(f"Warning         : flash save failed ({e})")

        # Write to source files
        write_define_float(MAIN_C, "MOTOR_MIN_DUTY_DEFAULT", new_min_duty,
                           "calibrated — calibrate_deadzone.py")
        write_define_float(MAIN_C, "MOTOR_KICK_DUTY_DEFAULT", new_kick_duty,
                           "calibrated — calibrate_deadzone.py")
        # KICK_CYCLES is an integer define with 'u' suffix — handle specially
        content = MAIN_C.read_text()
        pat = re.compile(
            r"(#define\s+MOTOR_KICK_CYCLES_DEFAULT\s+)\d+u?([ \t]*(?://[^\n]*)?)")
        new_content, count = pat.subn(
            lambda m: f"{m.group(1)}{new_kick_cycles}u"
                      f"  // calibrated — calibrate_deadzone.py",
            content, count=1
        )
        if count:
            MAIN_C.write_text(new_content)

        print(f"Applied to      : {MAIN_C.relative_to(SCRIPT_DIR)}")
        print("\nDone — calibration applied live and persisted.\n")
    else:
        # Restore old values so we don't leave the robot in a weird state
        try:
            if cur_min_duty is not None:
                send_calib(args.dxl_port, CALIB_CMD_SET,
                           CALIB_KEY_MOTOR_MIN_DUTY, value=cur_min_duty)
            if cur_kick_duty is not None:
                send_calib(args.dxl_port, CALIB_CMD_SET,
                           CALIB_KEY_MOTOR_KICK_DUTY, value=cur_kick_duty)
            if cur_kick_cyc is not None:
                send_calib(args.dxl_port, CALIB_CMD_SET,
                           CALIB_KEY_MOTOR_KICK_CYCLES, value=cur_kick_cyc)
            print("Restored        : original motor values (dry-run)")
        except Exception as e:
            print(f"Warning         : restore failed ({e})")
        print("\nDry-run: use --apply to persist. No files changed.\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
