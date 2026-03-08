#!/usr/bin/env python3
"""
Guided on-ground motion check for TurtleBot3 Pico firmware.

Purpose
-------
Run a short sequence of safe on-ground motion checks and ask the user to
confirm that the robot behaved correctly:
  1. Forward
  2. Reverse
  3. In-place left turn
  4. In-place right turn

This talks directly to the Pico over the Dynamixel serial interface, so ROS is
not required. It is intended as the next step after bench motor/encoder checks.
"""

from __future__ import annotations

import argparse
import os
import struct
import time
from dataclasses import dataclass

import serial

PORT_CANDIDATES = ("/dev/ttyTB3", "/dev/ttyACM0", "/dev/ttyACM1")
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
SETTLE_S = 0.50

CRC_TABLE: list[int] = []
for i in range(256):
    c = i << 8
    for _ in range(8):
        c = (((c << 1) ^ 0x8005) & 0xFFFF) if (c & 0x8000) else ((c << 1) & 0xFFFF)
    CRC_TABLE.append(c)


@dataclass
class StepResult:
    name: str
    delta_left: int
    delta_right: int
    peak_vel_left: float
    peak_vel_right: float
    final_vel_left: float
    final_vel_right: float
    observed_ok: bool


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


def packet(instr: int, params: bytes) -> bytes:
    length = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, DEV_ID, length & 0xFF, (length >> 8) & 0xFF, instr]) + params
    crc = crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])


def read_resp(ser: serial.Serial, timeout: float = 0.25):
    t0 = time.time()
    buf = b""
    while time.time() - t0 < timeout:
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except serial.SerialException:
            return None
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
    try:
        ser.reset_input_buffer()
        ser.write(packet(0x03, struct.pack("<H", addr) + data))
    except serial.SerialException:
        return False
    return read_resp(ser) is not None


def dxl_read(ser: serial.Serial, addr: int, length: int):
    try:
        ser.reset_input_buffer()
        ser.write(packet(0x02, struct.pack("<HH", addr, length)))
    except serial.SerialException:
        return None
    resp = read_resp(ser)
    if resp and len(resp) >= 9 + length:
        return resp[9:9 + length]
    return None


def detect_port(explicit: str | None):
    if explicit:
        return explicit
    for p in PORT_CANDIDATES:
        if os.path.exists(p):
            return p
    return PORT_CANDIDATES[0]


def open_serial(port: str) -> serial.Serial:
    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.15)
    ser.reset_input_buffer()
    return ser


def try_reconnect(port: str, max_attempts: int = 8, delay: float = 0.5) -> serial.Serial | None:
    """Attempt to reopen a dropped serial port.  Returns the new Serial or None."""
    print(f"\n  [serial] Port dropped — reconnecting to {port} ", end="", flush=True)
    for _ in range(max_attempts):
        time.sleep(delay)
        print(".", end="", flush=True)
        try:
            ser = open_serial(port)
            print(" OK")
            return ser
        except serial.SerialException:
            pass
    print(" FAILED")
    return None


def read_counts_and_vel(ser: serial.Serial):
    counts = dxl_read(ser, ADDR_ENC_L_COUNT, 8)
    vels = dxl_read(ser, ADDR_DBG_VEL_L, 8)
    if not counts or not vels or len(counts) < 8 or len(vels) < 8:
        return None
    left = struct.unpack_from("<i", counts, 0)[0]
    right = struct.unpack_from("<i", counts, 4)[0]
    vel_l = struct.unpack_from("<f", vels, 0)[0]
    vel_r = struct.unpack_from("<f", vels, 4)[0]
    return left, right, vel_l, vel_r


def reset_encoders(ser: serial.Serial) -> bool:
    ok = dxl_write(ser, ADDR_ENC_RESET, bytes([1]))
    time.sleep(0.08)
    return ok


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
    time.sleep(SETTLE_S)


def ask_yes_no(prompt: str, default: bool = True) -> bool:
    suffix = "[Y/n]" if default else "[y/N]"
    reply = input(f"{prompt} {suffix} ").strip().lower()
    if not reply:
        return default
    return reply in ("y", "yes")


def run_step(
    ser_holder: list[serial.Serial],
    port: str,
    *,
    name: str,
    linear_x_ms: float,
    angular_z_rads: float,
    duration_s: float,
    expectation: str,
    no_prompt: bool,
) -> StepResult:
    """Execute one motion step.  ser_holder is a one-element list so reconnect
    logic can replace the Serial object in-place without returning it."""
    if not no_prompt:
        input(f"\nPress Enter to start: {name} ... ")

    ser = ser_holder[0]
    if not reset_encoders(ser):
        print("Warning: encoder reset did not acknowledge.")

    # Retry baseline read through transient glitches
    baseline = None
    for _attempt in range(3):
        baseline = read_counts_and_vel(ser)
        if baseline is not None:
            break
        new_ser = try_reconnect(port)
        if new_ser is None:
            raise RuntimeError("Failed to read encoder baseline — cannot reconnect")
        ser_holder[0] = ser = new_ser
    if baseline is None:
        raise RuntimeError("Failed to read encoder baseline after retries")
    l0, r0, _, _ = baseline

    t_end = time.time() + duration_s
    next_cmd = 0.0
    peak_l = 0.0
    peak_r = 0.0
    glitch_count = 0

    print(f"\n{name}")
    print(f"  Expectation: {expectation}")
    print(f"  cmd_vel => linear.x={linear_x_ms:+.3f} m/s  angular.z={angular_z_rads:+.3f} rad/s")
    print(f"  {'elapsed':>7s}  {'ΔL':>8s}  {'ΔR':>8s}  {'velL':>8s}  {'velR':>8s}")
    print("  " + "-" * 50)

    while time.time() < t_end:
        now = time.time()
        ser = ser_holder[0]
        if now >= next_cmd:
            if not set_cmd_vel(ser, linear_x_ms, angular_z_rads):
                # write failed — attempt reconnect
                new_ser = try_reconnect(port)
                if new_ser is None:
                    print("\n  [serial] Reconnect failed — aborting step.")
                    break
                ser_holder[0] = ser = new_ser
                glitch_count += 1
                set_cmd_vel(ser, linear_x_ms, angular_z_rads)
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
            print(f"\r  {elapsed:7.2f}  {dl:+8d}  {dr:+8d}  {vl:+8.3f}  {vr:+8.3f}", end="", flush=True)
        else:
            glitch_count += 1
        time.sleep(DISPLAY_REFRESH_S)
    print()
    if glitch_count:
        print(f"  [serial] {glitch_count} read glitch(es) during step.")

    ser = ser_holder[0]
    try:
        stop_robot(ser)
    except serial.SerialException:
        new_ser = try_reconnect(port)
        if new_ser is not None:
            ser_holder[0] = ser = new_ser
            stop_robot(ser)

    # Retry final read
    final_sample = None
    for _attempt in range(3):
        final_sample = read_counts_and_vel(ser)
        if final_sample is not None:
            break
        new_ser = try_reconnect(port)
        if new_ser is None:
            raise RuntimeError("Failed to read final encoder values — cannot reconnect")
        ser_holder[0] = ser = new_ser
    if final_sample is None:
        raise RuntimeError("Failed to read final encoder values after retries")
    lf, rf, vlf, vrf = final_sample

    if no_prompt:
        observed_ok = True
    else:
        observed_ok = ask_yes_no(f"Did the robot {expectation.lower()}?", default=True)

    return StepResult(
        name=name,
        delta_left=lf - l0,
        delta_right=rf - r0,
        peak_vel_left=peak_l,
        peak_vel_right=peak_r,
        final_vel_left=vlf,
        final_vel_right=vrf,
        observed_ok=observed_ok,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="Guided on-ground motion check")
    ap.add_argument("--port", default=None, help="Serial port (default: auto-detect /dev/ttyTB3, /dev/ttyACM*)")
    ap.add_argument("--linear", type=float, default=0.06, help="Linear speed in m/s for forward/reverse (default: 0.06)")
    ap.add_argument("--angular", type=float, default=0.50, help="Angular speed in rad/s for turns (default: 0.50)")
    ap.add_argument("--time", type=float, default=1.5, help="Duration of each motion step in seconds (default: 1.5)")
    ap.add_argument("--no-prompt", action="store_true", help="Do not wait for Enter or yes/no confirmations")
    args = ap.parse_args()

    port = detect_port(args.port)

    print("\n=== Guided On-Ground Motion Check ===")
    print(f"Port         : {port}")
    print(f"Linear speed : {args.linear:.3f} m/s")
    print(f"Angular speed: {args.angular:.3f} rad/s")
    print(f"Step time    : {args.time:.2f} s")
    print("\nSafety:")
    print("  • Place the robot on open floor space.")
    print("  • Keep a hand near the stop point or lift handle.")
    print("  • Stop bringup first so the serial port is free.")
    print("  • Be ready to interrupt if motion is unsafe.")
    print("\nChecks:")
    print("  1. Forward: robot should move mostly straight ahead")
    print("  2. Reverse: robot should move mostly straight backward")
    print("  3. Left turn: robot should rotate CCW in place")
    print("  4. Right turn: robot should rotate CW in place")

    if not args.no_prompt:
        input("\nPress Enter when ready to begin... ")

    ser_holder: list[serial.Serial] = []   # filled after port opens; used by finally
    try:
        ser = open_serial(port)
    except serial.SerialException as exc:
        print(f"ERROR: Cannot open {port}: {exc}")
        print("Tip: sudo systemctl stop turtlebot3-bringup.service turtlebot3-camera.service")
        return 1

    ser_holder = [ser]   # mutable container so run_step can reconnect in place
    try:
        sample = read_counts_and_vel(ser)
        if sample is None:
            print("ERROR: No response from firmware.")
            return 1

        print(f"\nFirmware communication OK. Initial counts: L={sample[0]:+d}  R={sample[1]:+d}")

        steps = [
            ("Step 1: FORWARD", +args.linear, 0.0, "move forward in a mostly straight line"),
            ("Step 2: REVERSE", -args.linear, 0.0, "move backward in a mostly straight line"),
            ("Step 3: LEFT TURN", 0.0, +args.angular, "rotate left (counter-clockwise) in place"),
            ("Step 4: RIGHT TURN", 0.0, -args.angular, "rotate right (clockwise) in place"),
        ]
        results: list[StepResult] = []
        for name, lin, ang, expectation in steps:
            result = run_step(
                ser_holder,
                port,
                name=name,
                linear_x_ms=lin,
                angular_z_rads=ang,
                duration_s=args.time,
                expectation=expectation,
                no_prompt=args.no_prompt,
            )
            results.append(result)
            print("\nStep result:")
            print(f"  ΔL / ΔR           : {result.delta_left:+d} / {result.delta_right:+d}")
            print(f"  Peak vel L / R    : {result.peak_vel_left:+.3f} / {result.peak_vel_right:+.3f}")
            print(f"  Final vel L / R   : {result.final_vel_left:+.3f} / {result.final_vel_right:+.3f}")
            print(f"  User observation  : {'OK' if result.observed_ok else 'CHECK'}")

        print("\n" + "=" * 72)
        print("Summary")
        print("=" * 72)
        overall_ok = True
        for result in results:
            overall_ok &= result.observed_ok
            print(f"{result.name:<20s} observation={'OK' if result.observed_ok else 'CHECK'}")

        print()
        if overall_ok:
            print("PASS: on-ground motion checks looked correct.")
            return 0

        print("CHECK: at least one on-ground motion step looked wrong.")
        print("Use the step name above to describe what the robot actually did.")
        return 2
    finally:
        final_ser = ser_holder[0] if ser_holder else None
        if final_ser is not None:
            try:
                stop_robot(final_ser)
            except Exception:
                pass
            try:
                final_ser.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
