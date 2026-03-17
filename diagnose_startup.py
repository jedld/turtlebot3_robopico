#!/usr/bin/env python3
"""
diagnose_startup.py — Motor startup asymmetry diagnostic.

Drives the robot forward at low speed, sampling encoders and motor commands
at high speed (~300-500 Hz) for a short burst to measure:
  - Time until each encoder first moves
  - Per-wheel velocity ramp-up profile
  - Raw motor commands sent to Motor1/Motor2
  - Calibration state (swap, reversed flags)

Requires bringup to be stopped (takes exclusive serial access).

Usage:
    sudo systemctl stop turtlebot3-bringup.service
    python3 diagnose_startup.py [--speed 0.08] [--burst 1.5] [--direct 30]
    sudo systemctl start turtlebot3-bringup.service
"""

import argparse
import math
import os
import struct
import sys
import time

import serial
from serial import SerialException

# ── Dynamixel constants ────────────────────────────────────────────────────────
PORTS   = ("/dev/ttyTB3", "/dev/ttyACM0", "/dev/ttyACM1")
DEV_ID  = 200
BAUD    = 1_000_000

# Register addresses (from firmware/main.c)
ADDR_MOTOR_TORQUE_EN    = 149
ADDR_CMD_LINEAR_X       = 150
ADDR_CMD_ANGULAR_Z      = 170
ADDR_ENC_L_COUNT        = 184
ADDR_ENC_R_COUNT        = 188
ADDR_DBG_VEL_L          = 212
ADDR_DBG_VEL_R          = 216
ADDR_MOTOR_I2C_NDEV     = 344
ADDR_MOTOR_I2C_STATUS   = 348
ADDR_MOTOR_I2C_ERR_CNT  = 352
ADDR_MOTOR_LAST_CMD_M1  = 356
ADDR_MOTOR_LAST_CMD_M2  = 357
ADDR_MOTOR_DIRECT_M1    = 358
ADDR_MOTOR_DIRECT_M2    = 359
ADDR_MOTOR_WHO_AM_I     = 360

# Calibration instruction (0xA0) — firmware custom Dynamixel instruction
INST_CALIBRATION = 0xA0
CALIB_CMD_GET    = 0x02

# Encoder constants
ENC_PPR_MOTOR    = 11
ENC_GEAR_RATIO   = 44
ENC_COUNTS_PER_WHEEL_REV = ENC_PPR_MOTOR * 4 * ENC_GEAR_RATIO  # 1936
WHEEL_RADIUS     = 0.033  # m
ENC_RAD_PER_COUNT = 2.0 * math.pi / ENC_COUNTS_PER_WHEEL_REV

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

def _read_resp(ser, timeout=0.1):
    t0 = time.time()
    buf = b""
    while time.time() - t0 < timeout:
        chunk = ser.read(ser.in_waiting or 1)
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

def dxl_write(ser, addr, data):
    params = struct.pack("<H", addr) + data
    ser.reset_input_buffer()
    ser.write(_pkt(0x03, params))
    return _read_resp(ser) is not None

def dxl_read(ser, addr, length):
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

# ── Calibration read ──────────────────────────────────────────────────────────
def read_calibration(ser):
    """Read calibration via firmware's custom 0xA0 instruction."""
    ser.reset_input_buffer()
    ser.write(_pkt(INST_CALIBRATION, bytes([CALIB_CMD_GET])))
    resp = _read_resp(ser, timeout=0.3)
    if not resp or len(resp) < 9 + 37:
        return None
    d = resp[9:]
    return {
        "wheel_radius":     struct.unpack_from("<f", d, 1)[0],
        "wheel_separation": struct.unpack_from("<f", d, 5)[0],
        "max_wheel_speed":  struct.unpack_from("<f", d, 9)[0],
        "right_reversed":   d[13],
        "swap_motors":      d[14],
        "min_duty_left":    struct.unpack_from("<f", d, 15)[0],
        "kick_duty":        struct.unpack_from("<f", d, 19)[0],
        "kick_cycles":      d[23],
        "trim_left":        struct.unpack_from("<f", d, 24)[0],
        "trim_right":       struct.unpack_from("<f", d, 28)[0],
        "min_duty_right":   struct.unpack_from("<f", d, 32)[0],
        "left_reversed":    d[36],
    }

# ── High-speed sampling helpers ───────────────────────────────────────────────
# Bulk read: enc_l(184,4) + enc_r(188,4) + vel_l(212,4) + vel_r(216,4)
#            + last_cmd_m1(356,1) + last_cmd_m2(357,1)
# For efficiency, read 184..220 (36 bytes) and 356..358 (2 bytes) separately.

def read_enc_vel(ser):
    """Fast read: encoder counts + debug velocities (184..219 = 36 bytes)."""
    data = dxl_read(ser, ADDR_ENC_L_COUNT, 36)
    if not data or len(data) < 36:
        return None
    enc_l = struct.unpack_from("<i", data, 0)[0]   # 184
    enc_r = struct.unpack_from("<i", data, 4)[0]   # 188
    vel_l = struct.unpack_from("<f", data, 28)[0]   # 212
    vel_r = struct.unpack_from("<f", data, 32)[0]   # 216
    return enc_l, enc_r, vel_l, vel_r

def read_last_cmds(ser):
    """Read last raw motor commands (M1, M2) as signed int8."""
    data = dxl_read(ser, ADDR_MOTOR_LAST_CMD_M1, 2)
    if not data or len(data) < 2:
        return None, None
    m1 = struct.unpack_from("<b", data, 0)[0]
    m2 = struct.unpack_from("<b", data, 1)[0]
    return m1, m2

def set_velocity(ser, lin_x, ang_z=0.0):
    dxl_write(ser, ADDR_CMD_LINEAR_X,  struct.pack("<i", int(lin_x * 100.0)))
    time.sleep(0.002)
    dxl_write(ser, ADDR_CMD_ANGULAR_Z, struct.pack("<i", int(ang_z * 100.0)))

def direct_motor(ser, m1, m2):
    """Bypass PID: write raw int8 speed to both motors."""
    m1 = max(-100, min(100, m1))
    m2 = max(-100, min(100, m2))
    dxl_write(ser, ADDR_MOTOR_DIRECT_M1, struct.pack("<bb", m1, m2))

# ── Colors ────────────────────────────────────────────────────────────────────
RED = "\033[0;31m"; GRN = "\033[0;32m"; YLW = "\033[1;33m"
CYN = "\033[0;36m"; BLD = "\033[1m";    NC  = "\033[0m"

# ══════════════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="Motor startup asymmetry diagnostic")
    ap.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    ap.add_argument("--speed", type=float, default=0.08,
                    help="Forward speed in m/s (default 0.08)")
    ap.add_argument("--burst", type=float, default=2.0,
                    help="Burst duration in seconds (default 2.0)")
    ap.add_argument("--direct", type=int, default=0,
                    help="If >0, also run a direct bypass test at this duty (0-100)")
    args = ap.parse_args()

    port = detect_port(args.port)
    print(f"{BLD}diagnose_startup.py{NC} — Motor Startup Asymmetry Diagnostic")
    print(f"  port={port}  speed={args.speed:.3f} m/s  burst={args.burst:.1f} s\n")

    ser = serial.Serial(port, BAUD, timeout=0.05)
    time.sleep(0.15)

    # ── 1. Read calibration state ─────────────────────────────────────────────
    print(f"{BLD}=== Calibration State ==={NC}")
    cal = read_calibration(ser)
    if cal:
        print(f"  swap_motors      = {cal['swap_motors']}")
        print(f"  left_reversed    = {cal['left_reversed']}")
        print(f"  right_reversed   = {cal['right_reversed']}")
        print(f"  min_duty_left    = {cal['min_duty_left']:.3f}")
        print(f"  min_duty_right   = {cal['min_duty_right']:.3f}")
        print(f"  kick_duty        = {cal['kick_duty']:.3f}")
        print(f"  kick_cycles      = {cal['kick_cycles']}")
        print(f"  trim_left        = {cal['trim_left']:.4f}")
        print(f"  trim_right       = {cal['trim_right']:.4f}")
        print(f"  wheel_radius     = {cal['wheel_radius']:.4f}")
        print(f"  wheel_separation = {cal['wheel_separation']:.4f}")
        print(f"  max_wheel_speed  = {cal['max_wheel_speed']:.6f}")
    else:
        print(f"  {RED}Could not read calibration (old firmware?){NC}")
    print()

    # ── 2. Motor I2C diagnostics ──────────────────────────────────────────────
    print(f"{BLD}=== Motor I2C Status ==={NC}")
    data = dxl_read(ser, ADDR_MOTOR_I2C_NDEV, 17)
    if data and len(data) >= 17:
        ndev = data[0]
        devs = [data[i] for i in range(1, 4) if data[i] != 0xFF]
        status = struct.unpack_from("<I", data, 4)[0]
        err_cnt = struct.unpack_from("<I", data, 8)[0]
        who = data[16]
        dev_str = " ".join(f"0x{d:02X}" for d in devs) if devs else "(none)"
        print(f"  devices: {ndev} ({dev_str})  status={status}  errors={err_cnt}  who=0x{who:02X}")
    else:
        print(f"  {RED}Could not read I2C diagnostics{NC}")
    print()

    # ── 3. Enable torque ──────────────────────────────────────────────────────
    dxl_write(ser, ADDR_MOTOR_TORQUE_EN, bytes([1]))
    time.sleep(0.05)

    # ── 4. PID-based forward burst with high-speed encoder sampling ───────────
    print(f"{BLD}=== PID Forward Burst (speed={args.speed:.3f} m/s, {args.burst:.1f}s) ==={NC}")

    # Baseline encoder reading
    baseline = read_enc_vel(ser)
    if not baseline:
        print(f"  {RED}Cannot read encoders — aborting.{NC}")
        ser.close()
        return 1
    enc_l_base, enc_r_base = baseline[0], baseline[1]
    print(f"  Baseline: enc_l={enc_l_base}  enc_r={enc_r_base}")

    # Start driving
    set_velocity(ser, args.speed)
    t0 = time.monotonic()
    samples = []

    while time.monotonic() - t0 < args.burst:
        ev = read_enc_vel(ser)
        m1, m2 = read_last_cmds(ser)
        if ev:
            t_ms = (time.monotonic() - t0) * 1000.0
            samples.append((t_ms, ev[0], ev[1], ev[2], ev[3], m1, m2))
        # Refresh velocity command periodically
        if len(samples) % 20 == 0:
            set_velocity(ser, args.speed)

    # Stop
    set_velocity(ser, 0.0)
    time.sleep(0.1)

    print(f"  Collected {len(samples)} samples in {args.burst:.1f}s "
          f"({len(samples)/args.burst:.0f} Hz)\n")

    # ── 5. Analyze encoder startup ────────────────────────────────────────────
    ENC_MOVE_THRESH = 3
    first_l_ms = None
    first_r_ms = None
    for t_ms, enc_l, enc_r, *_ in samples:
        if first_l_ms is None and abs(enc_l - enc_l_base) >= ENC_MOVE_THRESH:
            first_l_ms = t_ms
        if first_r_ms is None and abs(enc_r - enc_r_base) >= ENC_MOVE_THRESH:
            first_r_ms = t_ms
        if first_l_ms is not None and first_r_ms is not None:
            break

    if first_l_ms is None: first_l_ms = args.burst * 1000
    if first_r_ms is None: first_r_ms = args.burst * 1000

    lag_ms = first_r_ms - first_l_ms
    leader = "L leads" if lag_ms > 0 else ("R leads" if lag_ms < 0 else "even")

    print(f"{BLD}=== Startup Timing ==={NC}")
    col_l = GRN if first_l_ms < 100 else (YLW if first_l_ms < 300 else RED)
    col_r = GRN if first_r_ms < 100 else (YLW if first_r_ms < 300 else RED)
    col_lag = GRN if abs(lag_ms) < 30 else (YLW if abs(lag_ms) < 80 else RED)
    print(f"  First encoder move:  L={col_l}{first_l_ms:.0f} ms{NC}  R={col_r}{first_r_ms:.0f} ms{NC}")
    print(f"  Lag: {col_lag}{abs(lag_ms):.0f} ms ({leader}){NC}")
    print()

    # ── 6. Steady-state analysis (last 50% of samples) ───────────────────────
    if len(samples) > 6:
        ss_start = len(samples) // 2
        ss = samples[ss_start:]
        mean_vl = sum(s[3] for s in ss) / len(ss)
        mean_vr = sum(s[4] for s in ss) / len(ss)
        mean_m1 = sum(s[5] for s in ss if s[5] is not None) / max(1, sum(1 for s in ss if s[5] is not None))
        mean_m2 = sum(s[6] for s in ss if s[6] is not None) / max(1, sum(1 for s in ss if s[6] is not None))

        ratio = abs(mean_vl / mean_vr) if abs(mean_vr) > 0.0001 else float('nan')
        asym_pct = (1.0 - ratio) * 100.0 if math.isfinite(ratio) else float('nan')

        print(f"{BLD}=== Steady-State (last {len(ss)} samples) ==={NC}")
        print(f"  Measured velocity:  L={mean_vl*1000:+6.1f} mm/s   R={mean_vr*1000:+6.1f} mm/s")
        if math.isfinite(ratio):
            col_r2 = GRN if abs(asym_pct) < 10 else (YLW if abs(asym_pct) < 25 else RED)
            print(f"  L/R ratio: {col_r2}{ratio:.3f}  ({asym_pct:+.1f}% asymmetry){NC}")
        print(f"  Mean motor cmds:   M1={mean_m1:+.1f}  M2={mean_m2:+.1f}")

        # Encoder travel
        last = samples[-1]
        dl = (last[1] - enc_l_base) * ENC_RAD_PER_COUNT * WHEEL_RADIUS
        dr = (last[2] - enc_r_base) * ENC_RAD_PER_COUNT * WHEEL_RADIUS
        print(f"  Encoder distance:  L={dl*100:.2f} cm   R={dr*100:.2f} cm")
        if cal:
            ws = cal['wheel_separation']
        else:
            ws = 0.048
        drift_rad = (dl - dr) / ws
        print(f"  Heading drift:     {math.degrees(drift_rad):+.1f}°")
    print()

    # ── 7. Print first 20 samples (transient detail) ─────────────────────────
    print(f"{BLD}=== Transient Detail (first 20 samples) ==={NC}")
    print(f"  {'t_ms':>7s}  {'enc_l':>8s}  {'enc_r':>8s}  {'Δenc_l':>7s}  {'Δenc_r':>7s}  "
          f"{'vel_l':>8s}  {'vel_r':>8s}  {'cmd_m1':>6s}  {'cmd_m2':>6s}")
    for i, (t_ms, enc_l, enc_r, vel_l, vel_r, m1, m2) in enumerate(samples[:20]):
        dl = enc_l - enc_l_base
        dr = enc_r - enc_r_base
        m1s = f"{m1:+d}" if m1 is not None else "?"
        m2s = f"{m2:+d}" if m2 is not None else "?"
        print(f"  {t_ms:7.1f}  {enc_l:8d}  {enc_r:8d}  {dl:+7d}  {dr:+7d}  "
              f"{vel_l*1000:+8.2f}  {vel_r*1000:+8.2f}  {m1s:>6s}  {m2s:>6s}")
    print()

    # ── 8. Optional direct bypass test ────────────────────────────────────────
    if args.direct > 0:
        duty = args.direct
        print(f"{BLD}=== Direct Bypass Test (M1={duty}, M2={duty}, 1.5s) ==={NC}")
        print("  This bypasses PID — raw I2C command to motor driver.\n")

        # Disable torque so PID doesn't interfere
        dxl_write(ser, ADDR_MOTOR_TORQUE_EN, bytes([0]))
        time.sleep(0.05)

        baseline2 = read_enc_vel(ser)
        if baseline2:
            enc_l_base2, enc_r_base2 = baseline2[0], baseline2[1]
        else:
            enc_l_base2, enc_r_base2 = 0, 0

        direct_motor(ser, duty, duty)
        t0d = time.monotonic()
        direct_samples = []
        while time.monotonic() - t0d < 1.5:
            ev = read_enc_vel(ser)
            m1, m2 = read_last_cmds(ser)
            if ev:
                t_ms = (time.monotonic() - t0d) * 1000.0
                direct_samples.append((t_ms, ev[0], ev[1], ev[2], ev[3], m1, m2))

        direct_motor(ser, 0, 0)
        time.sleep(0.1)

        print(f"  Collected {len(direct_samples)} samples\n")
        print(f"  {'t_ms':>7s}  {'Δenc_l':>7s}  {'Δenc_r':>7s}  "
              f"{'vel_l':>8s}  {'vel_r':>8s}  {'cmd_m1':>6s}  {'cmd_m2':>6s}")
        for i, (t_ms, enc_l, enc_r, vel_l, vel_r, m1, m2) in enumerate(direct_samples[:20]):
            dl = enc_l - enc_l_base2
            dr = enc_r - enc_r_base2
            m1s = f"{m1:+d}" if m1 is not None else "?"
            m2s = f"{m2:+d}" if m2 is not None else "?"
            print(f"  {t_ms:7.1f}  {dl:+7d}  {dr:+7d}  "
                  f"{vel_l*1000:+8.2f}  {vel_r*1000:+8.2f}  {m1s:>6s}  {m2s:>6s}")

        if len(direct_samples) > 6:
            ss = direct_samples[len(direct_samples)//2:]
            mean_vl = sum(s[3] for s in ss) / len(ss)
            mean_vr = sum(s[4] for s in ss) / len(ss)
            last = direct_samples[-1]
            dl = (last[1] - enc_l_base2) * ENC_RAD_PER_COUNT * WHEEL_RADIUS
            dr = (last[2] - enc_r_base2) * ENC_RAD_PER_COUNT * WHEEL_RADIUS
            print(f"\n  Direct steady-state:  L={mean_vl*1000:+6.1f} mm/s  R={mean_vr*1000:+6.1f} mm/s")
            print(f"  Direct distance:      L={dl*100:.2f} cm  R={dr*100:.2f} cm")
        print()

    # ── Cleanup ───────────────────────────────────────────────────────────────
    set_velocity(ser, 0.0)
    dxl_write(ser, ADDR_MOTOR_TORQUE_EN, bytes([1]))  # leave torque on for bringup
    ser.close()

    print(f"{BLD}Done.{NC}  Remember to restart bringup:")
    print(f"  sudo systemctl start turtlebot3-bringup.service")
    return 0

if __name__ == "__main__":
    sys.exit(main())
