#!/usr/bin/env python3
"""
verify_encoder.py — Encoder count-per-revolution verification for motor swap.

Reads raw encoder tick counts from the Pico firmware via Dynamixel Protocol 2.0.
Used to verify the encoder PPR × gear ratio product when swapping motors.

Usage:
  1. Lift the robot so wheels spin freely (or remove wheels and mark the shaft)
  2. Run:  python3 verify_encoder.py --stop-bringup
  3. Slowly rotate each wheel by hand EXACTLY one full revolution
  4. The script reports the actual counts per revolution

The expected value for JGA25-371 (11 PPR, 30:1 gear, X4 quad) = 1320 counts/rev.
"""

import argparse
import math
import struct
import subprocess
import sys
import time
from typing import Optional

import serial

# ── Dynamixel Protocol 2.0 ───────────────────────────────────────────────────
DXL_ID   = 200
BAUDRATE = 1_000_000
PORT     = "/dev/ttyTB3"

# Register addresses (must match firmware main.c)
REG_MILLIS       = 36   # uint32 — uptime ms
REG_ENC_L_COUNT  = 184  # int32 — left  encoder X4 count
REG_ENC_R_COUNT  = 188  # int32 — right encoder X4 count
REG_ENC_RESET    = 192  # uint8 — write non-zero to reset
REG_MOTOR_TORQUE = 256  # uint8 — motor torque enable
REG_DBG_VEL_L    = 212  # float — left  velocity (m/s)
REG_DBG_VEL_R    = 216  # float — right velocity (m/s)

# CRC
_CRC_TABLE = []
for _i in range(256):
    _c = _i << 8
    for _ in range(8):
        _c = ((_c << 1) ^ 0x8005) & 0xFFFF if (_c & 0x8000) else ((_c << 1) & 0xFFFF)
    _CRC_TABLE.append(_c)

def _crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc

def _make_read_pkt(addr: int, length: int) -> bytes:
    params = struct.pack("<HH", addr, length)
    plen = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, DXL_ID,
                 plen & 0xFF, (plen >> 8) & 0xFF, 0x02]) + params
    crc = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])

def _make_write_u8_pkt(addr: int, value: int) -> bytes:
    params = struct.pack("<H", addr) + bytes([value & 0xFF])
    plen = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, DXL_ID,
                 plen & 0xFF, (plen >> 8) & 0xFF, 0x03]) + params
    crc = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])

def _read_response(ser, expected: int, timeout: float = 0.25) -> Optional[bytes]:
    HEADER = b"\xFF\xFF\xFD\x00"
    t0 = time.monotonic()
    buf = b""
    while time.monotonic() - t0 < timeout:
        waiting = ser.in_waiting
        chunk = ser.read(waiting if waiting > 0 else 1)
        if chunk:
            buf += chunk
        while True:
            idx = buf.find(HEADER)
            if idx < 0:
                buf = buf[-3:] if len(buf) >= 3 else buf
                break
            buf = buf[idx:]
            if len(buf) < 9:
                break
            if buf[4] != DXL_ID or buf[7] != 0x55:
                buf = buf[4:]
                continue
            plen = buf[5] | (buf[6] << 8)
            num_params = plen - 4
            total = 7 + plen
            if len(buf) < total:
                break
            if num_params != expected:
                buf = buf[4:]
                continue
            return buf[9:9 + expected]
    return None

def dxl_write_and_drain(ser, pkt, timeout=0.05):
    ser.write(pkt)
    _read_response(ser, 0, timeout)

def dxl_read_i32(ser, addr):
    ser.write(_make_read_pkt(addr, 4))
    data = _read_response(ser, 4)
    if data and len(data) >= 4:
        return struct.unpack_from("<i", data, 0)[0]
    return None

def dxl_read_f32(ser, addr):
    ser.write(_make_read_pkt(addr, 4))
    data = _read_response(ser, 4)
    if data and len(data) >= 4:
        return struct.unpack_from("<f", data, 0)[0]
    return None

# ── Service management ────────────────────────────────────────────────────────
BRINGUP = "turtlebot3-bringup.service"

def bringup_active():
    try:
        r = subprocess.run(["systemctl", "is-active", BRINGUP],
                           capture_output=True, text=True)
        return r.stdout.strip() == "active"
    except Exception:
        return False

def bringup_stop():
    subprocess.run(["sudo", "systemctl", "stop", BRINGUP], capture_output=True)
    time.sleep(1.0)

def bringup_start():
    subprocess.run(["sudo", "systemctl", "start", BRINGUP], capture_output=True)
    time.sleep(2.0)

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Verify encoder counts per wheel revolution")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--stop-bringup", action="store_true",
                    help="Stop bringup service to take serial port")
    ap.add_argument("--motorised", action="store_true",
                    help="Use motor power instead of hand rotation (drives at low duty)")
    args = ap.parse_args()

    restart_bringup = False
    if bringup_active():
        if not args.stop_bringup:
            print("ERROR: turtlebot3-bringup is running — cannot take serial port.")
            print("  Run with --stop-bringup to stop it automatically.")
            sys.exit(1)
        print("Stopping bringup service ...")
        bringup_stop()
        restart_bringup = True
        print("  stopped\n")

    try:
        ser = serial.Serial(args.port, BAUDRATE, timeout=0.1)
        time.sleep(0.5)
        ser.reset_input_buffer()

        # Verify connection
        ms = dxl_read_i32(ser, REG_MILLIS)
        if ms is None:
            print("ERROR: No response from Pico. Check port and connections.")
            sys.exit(1)
        print(f"Pico online (uptime: {ms} ms)\n")

        # Disable motor torque for safety during hand rotation
        dxl_write_and_drain(ser, _make_write_u8_pkt(REG_MOTOR_TORQUE, 0))

        # Reset encoder counts
        dxl_write_and_drain(ser, _make_write_u8_pkt(REG_ENC_RESET, 1))
        time.sleep(0.1)

        # Verify reset
        l0 = dxl_read_i32(ser, REG_ENC_L_COUNT)
        r0 = dxl_read_i32(ser, REG_ENC_R_COUNT)
        print(f"Encoders reset: L={l0}  R={r0}")

        print("\n" + "=" * 60)
        print("ENCODER COUNT VERIFICATION")
        print("=" * 60)
        print()
        print("Instructions:")
        print("  1. Lift the robot so wheels can spin freely")
        print("  2. Put a mark on the wheel and the chassis at the same position")
        print("  3. Slowly rotate the LEFT wheel forward exactly ONE full turn")
        print("     (mark returns to starting position)")
        print("  4. Then rotate the RIGHT wheel forward exactly ONE full turn")
        print("  5. Press Enter after EACH wheel to record the count")
        print()
        print("Expected counts for JGA25-371 (11 PPR × 4 × 30:1) = 1320")
        print()

        # Live display of counts
        print("Live encoder counts (updating — press Ctrl+C to stop, Enter to record):")
        print("-" * 60)

        def show_counts():
            l = dxl_read_i32(ser, REG_ENC_L_COUNT)
            r = dxl_read_i32(ser, REG_ENC_R_COUNT)
            return l, r

        # --- LEFT wheel ---
        dxl_write_and_drain(ser, _make_write_u8_pkt(REG_ENC_RESET, 1))
        time.sleep(0.1)
        print("\n>>> Rotate LEFT wheel ONE full revolution forward, then press Enter")
        
        import select, termios, tty
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                l, r = show_counts()
                sys.stdout.write(f"\r  LEFT: {l:+6d}   RIGHT: {r:+6d}   (press Enter when done)")
                sys.stdout.flush()
                # Check for Enter key (non-blocking)
                if select.select([sys.stdin], [], [], 0.15)[0]:
                    ch = sys.stdin.read(1)
                    if ch in ('\n', '\r'):
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        left_count, _ = show_counts()
        print(f"\n  LEFT wheel: {left_count} counts per revolution")

        # --- RIGHT wheel ---
        dxl_write_and_drain(ser, _make_write_u8_pkt(REG_ENC_RESET, 1))
        time.sleep(0.1)
        print("\n>>> Rotate RIGHT wheel ONE full revolution forward, then press Enter")

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                l, r = show_counts()
                sys.stdout.write(f"\r  LEFT: {l:+6d}   RIGHT: {r:+6d}   (press Enter when done)")
                sys.stdout.flush()
                if select.select([sys.stdin], [], [], 0.15)[0]:
                    ch = sys.stdin.read(1)
                    if ch in ('\n', '\r'):
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        _, right_count = show_counts()
        print(f"\n  RIGHT wheel: {right_count} counts per revolution")

        # --- Analysis ---
        print("\n" + "=" * 60)
        print("RESULTS")
        print("=" * 60)
        expected = 1320  # 11 PPR × 4 × 30:1
        print(f"  Expected (11 PPR × 4 × 30:1):  {expected}")
        print(f"  Left  wheel actual:             {left_count:+d}  ({abs(left_count)} counts)")
        print(f"  Right wheel actual:             {right_count:+d}  ({abs(right_count)} counts)")

        # Check sign (direction)
        print()
        if left_count > 0:
            print("  Left  direction: ✓ FORWARD = positive (correct)")
        elif left_count < 0:
            print("  Left  direction: ✗ FORWARD = negative (REVERSED — flip LEFT_MOTOR_REVERSED)")
        else:
            print("  Left  direction: ? No counts detected — check wiring!")

        if right_count > 0:
            print("  Right direction: ✓ FORWARD = positive (correct)")
        elif right_count < 0:
            print("  Right direction: ✗ FORWARD = negative (REVERSED — flip RIGHT_MOTOR_REVERSED)")
        else:
            print("  Right direction: ? No counts detected — check wiring!")

        # Check magnitude
        print()
        for label, count in [("Left", abs(left_count)), ("Right", abs(right_count))]:
            if count == 0:
                print(f"  {label}: NO COUNTS — encoder not working or wiring issue")
            elif abs(count - expected) <= expected * 0.05:
                print(f"  {label}: ✓ Within 5% of expected ({count} vs {expected})")
            else:
                # Try to figure out PPR
                for ppr in [7, 11, 12, 13, 14, 16]:
                    for gear in [10, 20, 21, 30, 34, 45, 50, 56, 75, 100, 120, 131, 150, 226]:
                        if abs(count - ppr * 4 * gear) <= 2:
                            print(f"  {label}: Count {count} matches {ppr} PPR × 4 × {gear}:1 gear ratio")
                            break
                    else:
                        continue
                    break
                else:
                    print(f"  {label}: Count {count} does NOT match expected {expected}")
                    print(f"         Possible CPR values to try: {count}")
                    if count > 0:
                        print(f"         If gear=30:  PPR = {count / (4 * 30):.1f}")
                        print(f"         If PPR=11:   gear = {count / (4 * 11):.1f}")

        # Next steps
        print()
        print("=" * 60)
        print("NEXT STEPS")
        print("=" * 60)
        avg_count = (abs(left_count) + abs(right_count)) / 2
        if abs(avg_count - expected) <= expected * 0.05:
            print("  Encoder counts match expected values.")
            print("  Proceed to: python3 calibrate_deadzone.py --stop-bringup")
        else:
            print(f"  Encoder counts ({int(avg_count)}) differ from expected ({expected}).")
            print(f"  Update ENC_PPR_MOTOR and ENC_GEAR_RATIO in firmware/main.c")
            print(f"  to match actual counts, then rebuild and re-test.")
        
        if left_count < 0 or right_count < 0:
            print()
            print("  ⚠ Direction reversal needed — see encoder direction notes above.")
            print("  You may need to swap encoder A/B wires or flip *_MOTOR_REVERSED in firmware.")

        ser.close()

    finally:
        if restart_bringup:
            print("\nRestarting bringup service ...")
            bringup_start()
            print("  bringup restarted")

if __name__ == "__main__":
    main()
