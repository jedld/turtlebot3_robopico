#!/usr/bin/env python3
"""Interactive motor test for TurtleBot3 Pico + Waveshare I2C motor driver.

Sends velocity commands directly via Dynamixel Protocol 2.0 and prompts
the user to confirm physical motor movement.

Usage:
    python3 test_motor_driver.py [--port /dev/ttyACM0]
"""

import argparse
import math
import struct
import time
import os
import sys

import serial
from serial import SerialException

# ── Dynamixel constants ────────────────────────────────────────────────────────
PORTS   = ("/dev/ttyTB3", "/dev/ttyACM0", "/dev/ttyACM1")
DEV_ID  = 200
BAUD    = 1_000_000

# Control-table addresses (Dynamixel-protocol 2 register map in firmware)
ADDR_MOTOR_TORQUE_EN    = 149   # uint8: 1=on, 0=off
ADDR_CMD_LINEAR_X       = 150   # int32, 0.01 m/s units  (e.g. 10 → 0.10 m/s)
ADDR_CMD_ANGULAR_Z      = 170   # int32, 0.01 rad/s units (e.g. 100 → 1.00 rad/s)
ADDR_DBG_VEL_L          = 212   # float32, measured left  wheel velocity (m/s)
ADDR_DBG_VEL_R          = 216   # float32, measured right wheel velocity (m/s)

ADDR_MOTOR_I2C_NDEV     = 344   # uint8: I2C1 device count
ADDR_MOTOR_I2C_ERR_CNT  = 352   # uint32: cumulative failed I2C writes
ADDR_MOTOR_LAST_CMD_M1  = 356   # int8: last raw speed sent to Motor1
ADDR_MOTOR_LAST_CMD_M2  = 357   # int8: last raw speed sent to Motor2
ADDR_MOTOR_DIRECT_M1    = 358   # int8: write to drive Motor1 directly (bypass PID)
ADDR_MOTOR_DIRECT_M2    = 359   # int8: write to drive Motor2 directly (bypass PID)
ADDR_MOTOR_WHO_AM_I     = 360   # uint8: WHO_AM_I from device; 0xA4 = genuine Waveshare

# ── CRC ───────────────────────────────────────────────────────────────────────
CRC_TABLE = []
for _i in range(256):
    _c = _i << 8
    for _ in range(8):
        _c = ((_c << 1) ^ 0x8005) & 0xFFFF if (_c & 0x8000) else ((_c << 1) & 0xFFFF)
    CRC_TABLE.append(_c)

def _crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc

def _pkt(instr: int, params: bytes) -> bytes:
    length = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00,
                 DEV_ID, length & 0xFF, (length >> 8) & 0xFF,
                 instr]) + params
    crc = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])

def _read_resp(ser: serial.Serial, timeout: float = 0.4):
    t0 = time.time()
    buf = b""
    while time.time() - t0 < timeout:
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except SerialException:
            return None
        if not chunk:
            continue
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
    params = struct.pack("<H", addr) + data
    ser.reset_input_buffer()
    ser.write(_pkt(0x03, params))
    resp = _read_resp(ser)
    return resp is not None

def dxl_read(ser: serial.Serial, addr: int, length: int):
    ser.reset_input_buffer()
    ser.write(_pkt(0x02, struct.pack("<HH", addr, length)))
    resp = _read_resp(ser)
    if resp and len(resp) >= 9 + length:
        return resp[9:9 + length]
    return None

def detect_port(explicit):
    if explicit:
        return explicit
    for p in PORTS:
        if os.path.exists(p):
            return p
    return PORTS[0]

# ── High-level helpers ─────────────────────────────────────────────────────────

def torque_on(ser):
    dxl_write(ser, ADDR_MOTOR_TORQUE_EN, bytes([1]))

def torque_off(ser):
    dxl_write(ser, ADDR_MOTOR_TORQUE_EN, bytes([0]))

def set_velocity(ser, lin_x_m_s: float, ang_z_rad_s: float = 0.0):
    """Write goal velocity: lin_x in m/s, ang_z in rad/s (0.01-unit integers)."""
    dxl_write(ser, ADDR_CMD_LINEAR_X,  struct.pack("<i", int(lin_x_m_s * 100.0)))
    time.sleep(0.003)
    dxl_write(ser, ADDR_CMD_ANGULAR_Z, struct.pack("<i", int(ang_z_rad_s * 100.0)))
    time.sleep(0.003)

def get_velocity(ser):
    """Return (left_m_s, right_m_s) measured wheel velocities."""
    for _ in range(3):
        data = dxl_read(ser, ADDR_DBG_VEL_L, 8)
        if data and len(data) >= 8:
            left = struct.unpack_from("<f", data, 0)[0]
            right = struct.unpack_from("<f", data, 4)[0]
            if all(math.isfinite(v) and abs(v) < 5.0 for v in (left, right)):
                return left, right
        time.sleep(0.01)
    return None, None

def get_motor_error_count(ser):
    for _ in range(3):
        data = dxl_read(ser, ADDR_MOTOR_I2C_ERR_CNT, 4)
        if data and len(data) >= 4:
            return struct.unpack_from("<I", data, 0)[0]
        time.sleep(0.01)
    return None

def get_motor_i2c_ndev(ser):
    data = dxl_read(ser, ADDR_MOTOR_I2C_NDEV, 12)
    if data and len(data) >= 12:
        ndev   = data[0]
        devs   = [data[i] for i in range(1, 4) if data[i] != 0xFF]
        status = data[4]
        err_cnt = struct.unpack_from("<I", data, 8)[0]
        return ndev, devs, status, err_cnt
    return None, None, None, None

def get_last_cmds(ser):
    """Return (last_m1_speed, last_m2_speed) as signed int8 values."""
    data = dxl_read(ser, ADDR_MOTOR_LAST_CMD_M1, 2)
    if data and len(data) >= 2:
        m1 = struct.unpack_from("<b", data, 0)[0]
        m2 = struct.unpack_from("<b", data, 1)[0]
        return m1, m2
    return None, None

def get_who_am_i(ser):
    data = dxl_read(ser, ADDR_MOTOR_WHO_AM_I, 1)
    if data and len(data) >= 1:
        return data[0]
    return None

def direct_motor(ser, m1_speed: int, m2_speed: int):
    """Bypass PID and send raw int8 speed [-100,+100] directly to both motors."""
    m1 = max(-100, min(100, m1_speed))
    m2 = max(-100, min(100, m2_speed))
    dxl_write(ser, ADDR_MOTOR_DIRECT_M1, struct.pack("<bb", m1, m2))

# ── Test cases (lin_x m/s, ang_z rad/s) ──────────────────────────────────────

TESTS = [
    ("Forward  slow",  0.10,  0.0),
    ("Forward  fast",  0.30,  0.0),
    ("Backward slow", -0.10,  0.0),
    ("Backward fast", -0.30,  0.0),
    ("Spin left    ",  0.0,   1.5),
    ("Spin right   ",  0.0,  -1.5),
    ("Curve left   ",  0.15,  0.8),
    ("Curve right  ",  0.15, -0.8),
]

def _ask(prompt: str) -> str:
    try:
        return input(prompt).strip().lower()
    except (EOFError, KeyboardInterrupt):
        return "q"

# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Interactive motor test")
    ap.add_argument("--port", default=None)
    ap.add_argument("--duration", type=float, default=2.0,
                    help="How long each test command runs (seconds)")
    args = ap.parse_args()

    port = detect_port(args.port)
    print(f"Opening {port} at {BAUD} baud …")
    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.15)

    # ── Print motor I2C bus diagnostics first ──────────────────────────────────
    print("\n=== Motor I2C1 boot diagnostics ===")
    ndev, devs, status, err_cnt = get_motor_i2c_ndev(ser)
    who = get_who_am_i(ser)
    if ndev is None:
        print("  (no response — check /dev link or reflash firmware)")
    else:
        dev_str = " ".join(f"0x{d:02X}" for d in devs) if devs else "(none)"
        stat_str = {0: "OK – 0x55 responded", 1: "FAIL – 0x55 not found",
                    0xFF: "not yet init"}.get(status, f"unknown (0x{status:02X})")
        if who is not None:
            who_str = f"0x{who:02X}" + (" (OK)" if who == 0xA4 else " (unexpected — expected 0xA4)")
        else:
            who_str = "N/A (old firmware)"
        print(f"  devices found : {ndev}  ({dev_str})")
        print(f"  status        : {stat_str}")
        print(f"  who_am_i      : {who_str}")
        print(f"  write errors  : {err_cnt}")
        if status != 0:
            print("\n  WARNING: Motor I2C slave 0x55 not responding!")
            print("  Check Grove 2 cable, motor board power, and firmware.")
            if _ask("\n  Continue anyway? [y/N] ") != "y":
                ser.close()
                return 1

    # ── Direct bypass test (bypasses PID entirely) ────────────────────────────
    print("\n=== Direct motor bypass test (PID-free I2C write) ===")
    torque_off(ser)
    time.sleep(0.05)
    print("  Sending M1=+50, M2=+50 for 1 second via direct bypass register …")
    direct_motor(ser, 50, 50)
    time.sleep(0.05)
    m1_chk, m2_chk = get_last_cmds(ser)
    if m1_chk is not None:
        print(f"  last_cmd: M1={m1_chk:+d}  M2={m2_chk:+d}")
        if m1_chk == 50 and m2_chk == 50:
            print("  Firmware confirmed it sent the command.")
        else:
            print("  last_cmd differs from +50/+50 — sampled value may have been updated by a later command.")
    time.sleep(0.95)
    direct_motor(ser, 0, 0)  # stop
    ans_direct = _ask("  Did BOTH motors spin? [y/n/s=skip]: ")
    print()

    if ans_direct == "n":
        print("  Direct bypass failed. This means the Waveshare board is not responding")
        print("  to I2C writes, even though 0x55 was found. Check:")
        print("    1. Motor power connector (VM/GND) on Waveshare board")
        print("    2. Correct motor channel (M1/M2, not M3/M4)")
        print("    3. Grove 2 cable firmly seated on BOTH boards")
        if _ask("  Continue with PID tests anyway? [y/N] ") != "y":
            ser.close()
            return 1

    print(f"\n=== Running {len(TESTS)} PID velocity tests ({args.duration}s each) ===")
    print("Enabling torque …")
    torque_on(ser)
    time.sleep(0.1)
    print("Press ENTER to confirm motion observed, 'n' = no motion, 's' = skip, 'q' = quit.\n")

    passed = 0
    failed = 0
    skipped = 0

    for label, lin, ang in TESTS:
        print(f"  TEST: {label}  (lin_x={lin:+.2f} m/s  ang_z={ang:+.2f} rad/s)")
        err_before = get_motor_error_count(ser)
        set_velocity(ser, lin, ang)
        time.sleep(args.duration)

        # Sample present velocity and last command
        lv, rv = get_velocity(ser)
        m1, m2 = get_last_cmds(ser)
        err_after = get_motor_error_count(ser)
        if err_before is not None and err_after is not None and err_after >= err_before:
            new_errors = str(err_after - err_before)
        else:
            new_errors = "unavailable"
        if lv is not None:
            print(f"    measured vel : L={lv:+.3f} m/s  R={rv:+.3f} m/s")
        if m1 is not None:
            print(f"    last I2C cmd : M1={m1:+d}  M2={m2:+d}  (non-zero → firmware drove the motors)")
        print(f"    i2c errors   : +{new_errors} during this test")

        # Stop before asking
        set_velocity(ser, 0.0, 0.0)
        time.sleep(0.1)

        ans = _ask("    Did wheel(s) move? [y/n/s=skip/q=quit]: ")
        if ans == "q":
            print("  (aborted by user)")
            break
        elif ans == "s":
            skipped += 1
            print("  → SKIPPED")
        elif ans == "y":
            passed += 1
            print("  → PASS")
        else:
            failed += 1
            print("  → FAIL")
        print()

    print("=== Re-enabling torque ===")
    torque_on(ser)

    total = passed + failed + skipped
    print(f"\n=== Results: {passed} pass / {failed} fail / {skipped} skip out of {total} tests ===")
    if failed == 0 and skipped == 0:
        print("All tests passed!")
    elif failed > 0:
        print("Some tests failed — check I2C wiring and motor board power.")

    ser.close()
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
