#!/usr/bin/env python3
"""
Guided per-motor encoder test for the TurtleBot3 Pico firmware.

Purpose
-------
Tests each wheel motor individually while reading the firmware's encoder
registers in real time.  Intended for bench testing with the robot lifted so
both wheels spin freely.

What it checks
--------------
For each side (left, then right):
  1. Command only that wheel forward.
  2. Show live encoder counts / delta counts / measured wheel velocities.
  3. Ask you to confirm the expected wheel physically moved.
  4. Command only that wheel reverse.
  5. Verify the encoder sign flips between forward and reverse.

The non-target wheel may move slightly because this is a differential-drive
robot running through the normal firmware path; the script reports whether the
intended wheel dominates the encoder motion.

Usage
-----
  Stop bringup first so the serial port is free:
    sudo systemctl stop turtlebot3-bringup.service turtlebot3-camera.service

  python3 test_motor_encoder_guided.py
  python3 test_motor_encoder_guided.py --port /dev/ttyTB3
  python3 test_motor_encoder_guided.py --speed 0.08 --time 1.2
"""

from __future__ import annotations

import argparse
import os
import re
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial


# ── Defaults ────────────────────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parent
CONFIG_YAML = ROOT / "config" / "burger_pico.yaml"
DEFAULT_PORT_CANDIDATES = ("/dev/ttyTB3", "/dev/ttyACM0", "/dev/ttyACM1")
BAUD = 1_000_000
DEV_ID = 200

ADDR_MOTOR_TORQUE_EN = 149
ADDR_CMD_LINEAR_X = 150
ADDR_CMD_ANGULAR_Z = 170
ADDR_ENC_L_COUNT = 184
ADDR_ENC_R_COUNT = 188
ADDR_ENC_RESET = 192
ADDR_DBG_VEL_L = 212
ADDR_DBG_VEL_R = 216

CMD_REFRESH_S = 0.05
DISPLAY_REFRESH_S = 0.10
ACTIVE_MIN_COUNTS = 40
DOMINANCE_RATIO = 1.5
IO_RETRIES = 5


# ── Dynamixel Protocol 2.0 helpers ─────────────────────────────────────────
_CRC_TABLE: list[int] = []
for _i in range(256):
    _c = _i << 8
    for _ in range(8):
        _c = ((_c << 1) ^ 0x8005) & 0xFFFF if (_c & 0x8000) else (_c << 1) & 0xFFFF
    _CRC_TABLE.append(_c)


def _crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


def _packet(instr: int, params: bytes) -> bytes:
    length = len(params) + 3
    hdr = bytes([
        0xFF, 0xFF, 0xFD, 0x00,
        DEV_ID,
        length & 0xFF, (length >> 8) & 0xFF,
        instr,
    ]) + params
    crc = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])


def _read_resp(ser: serial.Serial, timeout: float = 0.25) -> bytes | None:
    t0 = time.time()
    buf = b""
    while time.time() - t0 < timeout:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
        idx = buf.find(b"\xFF\xFF\xFD\x00")
        if idx >= 0:
            buf = buf[idx:]
            if len(buf) >= 7:
                plen = buf[5] | (buf[6] << 8)
                total = 7 + plen
                if len(buf) >= total:
                    return buf[:total]
    return None


def dxl_write(ser: serial.Serial, addr: int, data: bytes) -> bool:
    ser.reset_input_buffer()
    ser.write(_packet(0x03, struct.pack("<H", addr) + data))
    return _read_resp(ser) is not None


def dxl_read(ser: serial.Serial, addr: int, length: int) -> bytes | None:
    ser.reset_input_buffer()
    ser.write(_packet(0x02, struct.pack("<HH", addr, length)))
    resp = _read_resp(ser)
    if resp and len(resp) >= 9 + length:
        return resp[9: 9 + length]
    return None


# ── Config / helpers ───────────────────────────────────────────────────────

def detect_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    for candidate in DEFAULT_PORT_CANDIDATES:
        if os.path.exists(candidate):
            return candidate
    return DEFAULT_PORT_CANDIDATES[0]


def read_wheel_separation_m() -> float:
    if CONFIG_YAML.exists():
        text = CONFIG_YAML.read_text(encoding="utf-8")
        m = re.search(r"^\s*separation:\s*([0-9.]+)", text, re.MULTILINE)
        if m:
            try:
                return float(m.group(1))
            except ValueError:
                pass
    return 0.5


def open_serial(port: str) -> serial.Serial:
    ser = serial.Serial(port, baudrate=BAUD, timeout=0.1)
    time.sleep(0.15)
    ser.reset_input_buffer()
    return ser


def read_counts_and_vel_once(ser: serial.Serial) -> tuple[int, int, float, float] | None:
    counts = dxl_read(ser, ADDR_ENC_L_COUNT, 8)
    if counts is None or len(counts) < 8:
        return None
    vels = dxl_read(ser, ADDR_DBG_VEL_L, 8)
    if vels is None or len(vels) < 8:
        return None
    left = struct.unpack_from("<i", counts, 0)[0]
    right = struct.unpack_from("<i", counts, 4)[0]
    vel_l = struct.unpack_from("<f", vels, 0)[0]
    vel_r = struct.unpack_from("<f", vels, 4)[0]
    return left, right, vel_l, vel_r


def read_counts_and_vel(ser: serial.Serial) -> tuple[int, int, float, float] | None:
    for _ in range(IO_RETRIES):
        sample = read_counts_and_vel_once(ser)
        if sample is not None:
            return sample
        time.sleep(0.05)
    return None


def recover_serial(port: str, ser: serial.Serial | None) -> serial.Serial:
    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass
    time.sleep(0.3)
    return open_serial(port)


def reset_encoders(ser: serial.Serial) -> bool:
    for _ in range(IO_RETRIES):
        ok = dxl_write(ser, ADDR_ENC_RESET, bytes([1]))
        time.sleep(0.08)
        if ok:
            return True
    return False


def set_cmd_vel(ser: serial.Serial, linear_x_ms: float, angular_z_rads: float) -> None:
    lin_raw = int(round(linear_x_ms * 100.0))
    ang_raw = int(round(angular_z_rads * 100.0))
    payload = struct.pack("<Bi", 1, lin_raw)
    dxl_write(ser, ADDR_MOTOR_TORQUE_EN, payload)
    dxl_write(ser, ADDR_CMD_ANGULAR_Z, struct.pack("<i", ang_raw))


def stop_robot(ser: serial.Serial) -> None:
    try:
        set_cmd_vel(ser, 0.0, 0.0)
    except Exception:
        pass
    time.sleep(0.2)


def isolated_wheel_command(side: str, wheel_speed_ms: float, wheel_separation_m: float) -> tuple[float, float]:
    # Differential-drive relation:
    #   v_l = lin - ang * sep / 2
    #   v_r = lin + ang * sep / 2
    # Solve for one stationary wheel and one commanded wheel.
    if side == "left":
        lin = wheel_speed_ms / 2.0
        ang = -wheel_speed_ms / wheel_separation_m
    else:
        lin = wheel_speed_ms / 2.0
        ang = wheel_speed_ms / wheel_separation_m
    return lin, ang


def ask_yes_no(prompt: str, default: bool = True) -> bool:
    suffix = "[Y/n]" if default else "[y/N]"
    reply = input(f"{prompt} {suffix} ").strip().lower()
    if not reply:
        return default
    return reply in ("y", "yes")


@dataclass
class PhaseResult:
    delta_left: int
    delta_right: int
    peak_vel_left: float
    peak_vel_right: float
    final_vel_left: float
    final_vel_right: float


@dataclass
class SideSummary:
    side: str
    forward: PhaseResult
    reverse: PhaseResult
    visually_ok: bool
    active_counts_ok: bool
    dominance_ok: bool
    sign_flip_ok: bool


# ── Test phases ────────────────────────────────────────────────────────────

def run_phase(
    ser: serial.Serial,
    *,
    side: str,
    label: str,
    wheel_speed_ms: float,
    duration_s: float,
    wheel_separation_m: float,
) -> PhaseResult:
    lin, ang = isolated_wheel_command(side, wheel_speed_ms, wheel_separation_m)

    baseline = read_counts_and_vel(ser)
    if baseline is None:
        raise RuntimeError("Failed to read encoder baseline")
    l0, r0, _, _ = baseline

    t_end = time.time() + duration_s
    next_cmd = 0.0
    peak_l = 0.0
    peak_r = 0.0

    print(f"\n[{side.upper()} wheel] {label}: wheel target {wheel_speed_ms:+.3f} m/s")
    print(f"  cmd_vel => linear.x={lin:+.3f} m/s  angular.z={ang:+.3f} rad/s")
    print(f"  {'elapsed':>7s}  {'ΔL':>8s}  {'ΔR':>8s}  {'velL':>8s}  {'velR':>8s}")
    print("  " + "-" * 50)

    while time.time() < t_end:
        now = time.time()
        if now >= next_cmd:
            set_cmd_vel(ser, lin, ang)
            next_cmd = now + CMD_REFRESH_S

        sample = read_counts_and_vel(ser)
        if sample is not None:
            l, r, vl, vr = sample
            dl = l - l0
            dr = r - r0
            if abs(vl) >= abs(peak_l):
                peak_l = vl
            if abs(vr) >= abs(peak_r):
                peak_r = vr
            elapsed = duration_s - (t_end - now)
            print(
                f"\r  {elapsed:7.2f}  {dl:+8d}  {dr:+8d}  {vl:+8.3f}  {vr:+8.3f}",
                end="",
                flush=True,
            )
        time.sleep(DISPLAY_REFRESH_S)

    stop_robot(ser)
    print()

    final_sample = read_counts_and_vel(ser)
    if final_sample is None:
        raise RuntimeError("Failed to read final encoder values")
    lf, rf, vlf, vrf = final_sample
    return PhaseResult(
        delta_left=lf - l0,
        delta_right=rf - r0,
        peak_vel_left=peak_l,
        peak_vel_right=peak_r,
        final_vel_left=vlf,
        final_vel_right=vrf,
    )


def run_phase_with_recovery(
    ser: serial.Serial,
    *,
    port: str,
    side: str,
    label: str,
    wheel_speed_ms: float,
    duration_s: float,
    wheel_separation_m: float,
) -> tuple[serial.Serial, PhaseResult]:
    try:
        return ser, run_phase(
            ser,
            side=side,
            label=label,
            wheel_speed_ms=wheel_speed_ms,
            duration_s=duration_s,
            wheel_separation_m=wheel_separation_m,
        )
    except (RuntimeError, serial.SerialException) as exc:
        print(f"\n  Warning: {exc}")
        print("  Attempting serial recovery and one retry...")
        ser = recover_serial(port, ser)
        stop_robot(ser)
        reset_encoders(ser)
        return ser, run_phase(
            ser,
            side=side,
            label=label,
            wheel_speed_ms=wheel_speed_ms,
            duration_s=duration_s,
            wheel_separation_m=wheel_separation_m,
        )


def summarize_side(side: str, forward: PhaseResult, reverse: PhaseResult) -> tuple[bool, bool, bool]:
    if side == "left":
        active_fwd = forward.delta_left
        active_rev = reverse.delta_left
        passive_fwd = forward.delta_right
        passive_rev = reverse.delta_right
        active_peak_fwd = forward.peak_vel_left
        active_peak_rev = reverse.peak_vel_left
    else:
        active_fwd = forward.delta_right
        active_rev = reverse.delta_right
        passive_fwd = forward.delta_left
        passive_rev = reverse.delta_left
        active_peak_fwd = forward.peak_vel_right
        active_peak_rev = reverse.peak_vel_right

    active_counts_ok = max(abs(active_fwd), abs(active_rev)) >= ACTIVE_MIN_COUNTS
    dominance_ok = (
        abs(active_fwd) >= max(ACTIVE_MIN_COUNTS, int(abs(passive_fwd) * DOMINANCE_RATIO))
        and abs(active_rev) >= max(ACTIVE_MIN_COUNTS, int(abs(passive_rev) * DOMINANCE_RATIO))
    )
    # NOTE: raw encoder count sign is hardware-specific and is not the best
    # signal for this test. Use the firmware's signed measured wheel velocity
    # peaks instead, because that is what odometry/control actually uses.
    sign_flip_ok = (
        abs(active_peak_fwd) > 0.005
        and abs(active_peak_rev) > 0.005
        and ((active_peak_fwd > 0.0) != (active_peak_rev > 0.0))
    )
    return active_counts_ok, dominance_ok, sign_flip_ok


# ── Main ───────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(description="Guided per-motor encoder test")
    ap.add_argument("--port", default=None, help="Serial port (default: auto-detect /dev/ttyTB3, /dev/ttyACM*)")
    ap.add_argument("--speed", type=float, default=0.08, help="Target wheel speed in m/s (default: 0.08)")
    ap.add_argument("--time", type=float, default=1.2, help="Duration of each forward/reverse phase in seconds")
    ap.add_argument("--no-prompt", action="store_true", help="Do not wait for Enter / yes-no prompts")
    args = ap.parse_args()

    port = detect_port(args.port)
    sep = read_wheel_separation_m()

    print("\n=== Guided Motor + Encoder Test ===")
    print(f"Port             : {port}")
    print(f"Wheel separation : {sep:.3f} m")
    print(f"Wheel speed      : {args.speed:.3f} m/s")
    print(f"Phase time       : {args.time:.2f} s")
    print("\nSafety:")
    print("  • Lift the robot so BOTH wheels spin freely.")
    print("  • Stop bringup first so the serial port is not busy.")
    print("  • Keep fingers / cables clear of the wheels.")

    if not args.no_prompt:
        input("\nPress Enter when ready to begin... ")

    try:
        ser = open_serial(port)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}")
        print("Tip: sudo systemctl stop turtlebot3-bringup.service turtlebot3-camera.service")
        return 1

    try:
        sample = read_counts_and_vel(ser)
        if sample is None:
            print("ERROR: No response from firmware.")
            return 1

        print("\nFirmware communication OK.")
        print(f"Initial encoder counts: L={sample[0]:+d}  R={sample[1]:+d}")

        summaries: list[SideSummary] = []

        for side in ("left", "right"):
            print("\n" + "=" * 72)
            print(f"Testing {side.upper()} wheel")
            print("=" * 72)
            print(f"Expected: the {side} wheel should spin first forward, then reverse.")
            print("Watch both wheels and the encoder deltas below.")

            if not args.no_prompt:
                input(f"Press Enter to start {side} wheel FORWARD phase... ")

            if not reset_encoders(ser):
                print("  Warning: encoder reset did not acknowledge; attempting recovery...")
                ser = recover_serial(port, ser)
                reset_encoders(ser)
            ser, forward = run_phase_with_recovery(
                ser,
                port=port,
                side=side,
                label="FORWARD",
                wheel_speed_ms=abs(args.speed),
                duration_s=args.time,
                wheel_separation_m=sep,
            )

            if not args.no_prompt:
                input(f"Press Enter to start {side} wheel REVERSE phase... ")

            if not reset_encoders(ser):
                print("  Warning: encoder reset did not acknowledge; attempting recovery...")
                ser = recover_serial(port, ser)
                reset_encoders(ser)
            ser, reverse = run_phase_with_recovery(
                ser,
                port=port,
                side=side,
                label="REVERSE",
                wheel_speed_ms=-abs(args.speed),
                duration_s=args.time,
                wheel_separation_m=sep,
            )

            active_counts_ok, dominance_ok, sign_flip_ok = summarize_side(side, forward, reverse)

            print("\nAutomatic checks:")
            print(f"  Forward ΔL / ΔR : {forward.delta_left:+d} / {forward.delta_right:+d}")
            print(f"  Reverse ΔL / ΔR : {reverse.delta_left:+d} / {reverse.delta_right:+d}")
            print(f"  Active encoder movement : {'OK' if active_counts_ok else 'LOW'}")
            print(f"  Intended wheel dominates: {'OK' if dominance_ok else 'MARGINAL'}")
            print(f"  Encoder sign flips F/R : {'OK' if sign_flip_ok else 'FAIL'}")

            if args.no_prompt:
                visually_ok = True
            else:
                visually_ok = ask_yes_no(
                    f"Did the {side} wheel visibly move as expected in both directions?",
                    default=True,
                )

            summaries.append(
                SideSummary(
                    side=side,
                    forward=forward,
                    reverse=reverse,
                    visually_ok=visually_ok,
                    active_counts_ok=active_counts_ok,
                    dominance_ok=dominance_ok,
                    sign_flip_ok=sign_flip_ok,
                )
            )

        print("\n" + "=" * 72)
        print("Summary")
        print("=" * 72)
        overall_ok = True
        for s in summaries:
            side_ok = s.visually_ok and s.active_counts_ok and s.sign_flip_ok
            overall_ok &= side_ok
            print(
                f"{s.side.capitalize():>5s}: "
                f"visual={'OK' if s.visually_ok else 'FAIL'}  "
                f"counts={'OK' if s.active_counts_ok else 'LOW'}  "
                f"dominance={'OK' if s.dominance_ok else 'MARGINAL'}  "
                f"sign={'OK' if s.sign_flip_ok else 'FAIL'}  "
                f"=> {'PASS' if side_ok else 'CHECK'}"
            )

        print()
        if overall_ok:
            print("PASS: both wheel motors and encoder directions look correct.")
            return 0

        print("CHECK: at least one side needs attention.")
        print("Inspect the wheel that failed visual motion, low counts, or sign reversal.")
        return 2

    finally:
        try:
            stop_robot(ser)
        except Exception:
            pass
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
