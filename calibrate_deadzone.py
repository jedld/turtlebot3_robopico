#!/usr/bin/env python3
"""
Auto-calibrate per-motor MOTOR_MIN_DUTY (dead-zone compensation) and
MOTOR_KICK_DUTY using the BNO085 IMU to detect when wheels are actually
moving.  Detects motor asymmetry by testing each motor individually.

Strategy
--------
1. Disable kick-start (KICK_CYCLES=0) so we only test the steady-state floor.
2. Test each motor individually (left and right) by commanding velocities
   that zero one wheel while driving the other.
3. Binary-search MOTOR_MIN_DUTY for each motor until the IMU detects motion.
4. Test both forward and backward directions per motor; take the worst case.
5. Report any asymmetry between left and right motors.
6. Add a configurable safety margin per motor.
7. Optionally set KICK_DUTY = max(MIN_DUTY_LEFT, MIN_DUTY_RIGHT) + boost.

Per-motor testing uses the IMU gyro-Z: when only one wheel is driven,
the robot rotates recognizably, providing a clear stall/moving signal.

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

CALIB_KEY_MOTOR_MIN_DUTY       = 0x04   # sets BOTH left and right
CALIB_KEY_MOTOR_KICK_DUTY      = 0x05
CALIB_KEY_MOTOR_KICK_CYCLES    = 0x06
CALIB_KEY_MOTOR_MIN_DUTY_LEFT  = 0x0B
CALIB_KEY_MOTOR_MIN_DUTY_RIGHT = 0x0C


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

    def send(self, lin_x: float = 0.0, ang_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        self.pub.publish(msg)

    def stop(self, settle: float = 0.3) -> None:
        self.send(0.0, 0.0)
        end = time.time() + settle
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def probe(self, ang_z: float = 0.0, lin_x: float = 0.0,
              duration: float = 1.5, settle_before: float = 0.3) -> float:
        """
        Command motion at (lin_x, ang_z) for `duration` seconds and return
        the mean absolute gyro-Z observed by the IMU (rad/s).

        A return value near 0 means the motors stalled.
        """
        # Settle first so previous motion doesn't contaminate
        self.stop(settle_before)

        self._gyro_z_samples.clear()
        self._collecting = True

        end = time.time() + duration
        while time.time() < end:
            self.send(lin_x, ang_z)
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

def find_single_motor_stall(
    node: "DeadzoneProber",
    port: str,
    motor: str,
    direction: str,
    ang_speed: float,
    search_lo: float = 0.10,
    search_hi: float = 0.80,
    stall_threshold: float = 0.05,
    resolution: float = 0.02,
    probe_duration: float = 1.5,
    verbose: bool = True,
) -> float:
    """
    Binary-search for the lowest MOTOR_MIN_DUTY where the specified motor
    (left or right) reliably turns.

    Strategy: use per-motor MIN_DUTY keys to isolate each motor.  Set the
    tested motor's MIN_DUTY to the candidate value and the OTHER motor's
    MIN_DUTY to 0 (disabling dead-zone compensation).  Then command a
    pivot turn: at low angular speed the raw throttle per wheel is tiny
    (~1-2%), so without dead-zone boost the "off" motor won't move.  Only
    the tested motor gets boosted, so any detected rotation is caused
    exclusively by it.

    The `direction` parameter selects the pivot direction:
      "forward"  → positive angular.z (tested motor drives forward)
      "backward" → negative angular.z (tested motor drives backward)
    """
    # Pick pivot sign so the tested motor drives in the requested direction:
    # Pivot CCW (ang_z > 0): left goes backward, right goes forward
    # Pivot CW  (ang_z < 0): left goes forward,  right goes backward
    if motor == "left":
        ang_z = -ang_speed if direction == "forward" else ang_speed
    else:
        ang_z = ang_speed if direction == "forward" else -ang_speed

    # Keys for the tested vs untested motor
    key_test  = (CALIB_KEY_MOTOR_MIN_DUTY_LEFT if motor == "left"
                 else CALIB_KEY_MOTOR_MIN_DUTY_RIGHT)
    key_other = (CALIB_KEY_MOTOR_MIN_DUTY_RIGHT if motor == "left"
                 else CALIB_KEY_MOTOR_MIN_DUTY_LEFT)

    label = f"{motor}-{direction}"

    lo, hi = search_lo, search_hi

    def _probe_at(duty: float) -> float:
        """Set the tested motor's min duty and probe."""
        send_calib(port, CALIB_CMD_SET, key_test, value=duty)
        send_calib(port, CALIB_CMD_SET, key_other, value=0.0)
        time.sleep(0.05)
        return node.probe(ang_z=ang_z, duration=probe_duration)

    # Verify hi works at all
    rate = _probe_at(hi)
    if verbose:
        print(f"    [{label}] MIN_DUTY={hi:.2f}  →  gyro={rate:.3f} rad/s  "
              f"{'MOVING' if rate > stall_threshold else 'STALL'}")
    if rate <= stall_threshold:
        if verbose:
            print(f"    [{label}] Motor doesn't move even at {hi:.2f}!")
        return hi

    # Verify lo stalls
    rate = _probe_at(lo)
    if verbose:
        print(f"    [{label}] MIN_DUTY={lo:.2f}  →  gyro={rate:.3f} rad/s  "
              f"{'MOVING' if rate > stall_threshold else 'STALL'}")
    if rate > stall_threshold:
        if verbose:
            print(f"    [{label}] Motor moves even at {lo:.2f} — stall below range.")
        return lo

    # Binary search
    iterations = 0
    while (hi - lo) > resolution and iterations < 20:
        mid = (lo + hi) / 2.0
        rate = _probe_at(mid)
        moving = rate > stall_threshold
        if verbose:
            print(f"    [{label}] MIN_DUTY={mid:.2f}  →  gyro={rate:.3f} rad/s  "
                  f"{'MOVING' if moving else 'STALL'}")

        if moving:
            hi = mid
        else:
            lo = mid
        iterations += 1

    if verbose:
        print(f"    [{label}] Stall threshold: {hi:.2f}")

    return hi


def verify_duty(node: "DeadzoneProber", port: str,
                duty_left: float, duty_right: float,
                ang_speed: float, trials: int = 3,
                stall_threshold: float = 0.05,
                verbose: bool = True) -> bool:
    """Run multiple pivot trials to confirm motors reliably start."""
    send_calib(port, CALIB_CMD_SET, CALIB_KEY_MOTOR_MIN_DUTY_LEFT,
               value=duty_left)
    send_calib(port, CALIB_CMD_SET, CALIB_KEY_MOTOR_MIN_DUTY_RIGHT,
               value=duty_right)
    time.sleep(0.05)
    successes = 0
    for t in range(trials):
        # Alternate rotation direction to test both motors in both roles
        direction = 1.0 if t % 2 == 0 else -1.0
        rate = node.probe(ang_z=direction * ang_speed, duration=1.2)
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
        description="Auto-calibrate per-motor MOTOR_MIN_DUTY (dead-zone "
                    "compensation) and MOTOR_KICK_DUTY using the BNO085 IMU.  "
                    "Detects motor asymmetry."
    )
    parser.add_argument("--speed", type=float, default=0.3,
                        help="angular.z test speed in rad/s (default 0.3)")
    parser.add_argument("--margin", type=float, default=0.05,
                        help="safety margin added above each stall threshold "
                             "(default 0.05 = 5%%)")
    parser.add_argument("--kick-margin", type=float, default=0.05,
                        help="KICK_DUTY = max(MIN_DUTY_L,R) + this (default 0.05)")
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
    cur_min_duty_left  = read_define_float(MAIN_C, "MOTOR_MIN_DUTY_LEFT_DEFAULT")
    cur_min_duty_right = read_define_float(MAIN_C, "MOTOR_MIN_DUTY_RIGHT_DEFAULT")
    cur_kick_duty = read_define_float(MAIN_C, "MOTOR_KICK_DUTY_DEFAULT")
    cur_kick_cyc  = read_define_float(MAIN_C, "MOTOR_KICK_CYCLES_DEFAULT")

    print("\n=== Dead-Zone Compensation Calibration (per-motor) ===")
    print(f"Firmware        : {MAIN_C}")
    print(f"Current         : MIN_DUTY_L={cur_min_duty_left}  "
          f"MIN_DUTY_R={cur_min_duty_right}  "
          f"KICK_DUTY={cur_kick_duty}  KICK_CYCLES={cur_kick_cyc}")
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

    # Disable kick-start so we only test the steady-state floor
    send_calib(args.dxl_port, CALIB_CMD_SET,
               CALIB_KEY_MOTOR_KICK_CYCLES, value=0.0)
    print("Kick-start disabled (KICK_CYCLES=0)\n")

    # ── Step 1a: Left motor stall detection ───────────────────────────────
    print("─" * 60)
    print("Step 1a: Left motor dead-zone (forward + backward)")
    print("─" * 60)

    left_fwd_stall = find_single_motor_stall(
        node, args.dxl_port, "left", "forward",
        ang_speed=args.speed,
        search_lo=args.search_lo, search_hi=args.search_hi,
        stall_threshold=args.stall_threshold,
        resolution=args.resolution,
    )
    left_bwd_stall = find_single_motor_stall(
        node, args.dxl_port, "left", "backward",
        ang_speed=args.speed,
        search_lo=args.search_lo, search_hi=args.search_hi,
        stall_threshold=args.stall_threshold,
        resolution=args.resolution,
    )
    left_stall = max(left_fwd_stall, left_bwd_stall)
    print(f"\n  Left motor:  fwd={left_fwd_stall:.2f}  bwd={left_bwd_stall:.2f}"
          f"  → worst-case={left_stall:.2f}\n")

    # ── Step 1b: Right motor stall detection ──────────────────────────────
    print("─" * 60)
    print("Step 1b: Right motor dead-zone (forward + backward)")
    print("─" * 60)

    right_fwd_stall = find_single_motor_stall(
        node, args.dxl_port, "right", "forward",
        ang_speed=args.speed,
        search_lo=args.search_lo, search_hi=args.search_hi,
        stall_threshold=args.stall_threshold,
        resolution=args.resolution,
    )
    right_bwd_stall = find_single_motor_stall(
        node, args.dxl_port, "right", "backward",
        ang_speed=args.speed,
        search_lo=args.search_lo, search_hi=args.search_hi,
        stall_threshold=args.stall_threshold,
        resolution=args.resolution,
    )
    right_stall = max(right_fwd_stall, right_bwd_stall)
    print(f"\n  Right motor: fwd={right_fwd_stall:.2f}  bwd={right_bwd_stall:.2f}"
          f"  → worst-case={right_stall:.2f}\n")

    # ── Asymmetry report ──────────────────────────────────────────────────
    asymmetry = abs(left_stall - right_stall)
    print("─" * 60)
    print("Motor asymmetry analysis")
    print("─" * 60)
    print(f"  Left  motor stall duty : {left_stall:.2f}")
    print(f"  Right motor stall duty : {right_stall:.2f}")
    print(f"  Asymmetry              : {asymmetry:.2f}")
    if asymmetry > 0.02:
        weaker = "RIGHT" if right_stall > left_stall else "LEFT"
        print(f"  NOTE: {weaker} motor has higher dead zone "
              f"— per-motor compensation applied.")
    else:
        print(f"  Motors are well matched.")
    print()

    # ── Compute per-motor min duty values ─────────────────────────────────
    new_min_duty_left  = min(left_stall + args.margin, 0.95)
    new_min_duty_right = min(right_stall + args.margin, 0.95)
    new_kick_duty      = min(max(new_min_duty_left, new_min_duty_right)
                             + args.kick_margin, 0.95)
    new_kick_cycles    = args.kick_cycles

    # ── Step 2: Verify with pivot test ────────────────────────────────────
    print("─" * 60)
    print(f"Step 2: Verify MIN_DUTY_L={new_min_duty_left:.2f}  "
          f"MIN_DUTY_R={new_min_duty_right:.2f} (pivot trials)")
    print("─" * 60)

    send_calib(args.dxl_port, CALIB_CMD_SET,
               CALIB_KEY_MOTOR_KICK_CYCLES, value=float(new_kick_cycles))
    send_calib(args.dxl_port, CALIB_CMD_SET,
               CALIB_KEY_MOTOR_KICK_DUTY, value=new_kick_duty)

    ok = verify_duty(node, args.dxl_port,
                     duty_left=new_min_duty_left,
                     duty_right=new_min_duty_right,
                     ang_speed=args.speed, trials=4,
                     stall_threshold=args.stall_threshold)

    if not ok:
        print(f"\n  Verification failed — bumping both MIN_DUTY by "
              f"extra {args.margin:.2f}")
        new_min_duty_left  = min(new_min_duty_left + args.margin, 0.95)
        new_min_duty_right = min(new_min_duty_right + args.margin, 0.95)
        new_kick_duty = min(max(new_min_duty_left, new_min_duty_right)
                            + args.kick_margin, 0.95)
        ok = verify_duty(node, args.dxl_port,
                         duty_left=new_min_duty_left,
                         duty_right=new_min_duty_right,
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
    print("─" * 60)
    print("Result")
    print("─" * 60)
    print(f"  Left  stall threshold : {left_stall:.2f}")
    print(f"  Right stall threshold : {right_stall:.2f}")
    print(f"  Safety margin         : {args.margin:.2f}")
    print(f"  MOTOR_MIN_DUTY_LEFT   : {cur_min_duty_left} → {new_min_duty_left:.2f}")
    print(f"  MOTOR_MIN_DUTY_RIGHT  : {cur_min_duty_right} → {new_min_duty_right:.2f}")
    print(f"  MOTOR_KICK_DUTY       : {cur_kick_duty} → {new_kick_duty:.2f}")
    print(f"  MOTOR_KICK_CYCLES     : {cur_kick_cyc} → {new_kick_cycles}")
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
        write_define_float(MAIN_C, "MOTOR_MIN_DUTY_LEFT_DEFAULT",
                           new_min_duty_left,
                           "calibrated — calibrate_deadzone.py")
        write_define_float(MAIN_C, "MOTOR_MIN_DUTY_RIGHT_DEFAULT",
                           new_min_duty_right,
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
            if cur_min_duty_left is not None:
                send_calib(args.dxl_port, CALIB_CMD_SET,
                           CALIB_KEY_MOTOR_MIN_DUTY_LEFT,
                           value=cur_min_duty_left)
            if cur_min_duty_right is not None:
                send_calib(args.dxl_port, CALIB_CMD_SET,
                           CALIB_KEY_MOTOR_MIN_DUTY_RIGHT,
                           value=cur_min_duty_right)
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
