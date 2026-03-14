#!/usr/bin/env python3
"""
test_imu_heading_fusion.py — Comprehensive test of IMU-encoder complementary
filter heading correction added to the TurtleBot3 Pico firmware.

Runs paired forward/reverse passes with three configurations:
  1) Encoder-only heading hold  (fusion disabled)
  2) IMU-fused heading hold     (fusion enabled, α = default 0.98)
  3) Comparison summary + CSV

For each pass it samples at ~10 Hz:
  - Fused heading error (ADDR_DBG_HEADING_FUSED)
  - Gyro-only integrated heading (ADDR_DBG_HEADING_GYRO)
  - Encoder-only heading error (ADDR_DBG_HEADING_ENC)
  - Heading hold correction (ADDR_HEADING_HOLD_CORR)
  - IMU raw gyro-Z (ADDR_IMU_ANG_VEL_Z)
  - Measured L/R wheel velocities (ADDR_DBG_VEL_L/R)
  - Heading hold integral term (ADDR_HEADING_HOLD_ITERM)
  - Encoder diff (ADDR_DBG_ENC_DIFF)
  - Complementary filter alpha (ADDR_IMU_HEADING_ALPHA)

Produces:
  • Live console output with colour-coded pass summaries
  • Timestamped CSV file for post-analysis
  • Final comparison table: encoder-only vs fused

Must be run with turtlebot3-bringup STOPPED (takes exclusive serial access).

Usage:
  python3 test_imu_heading_fusion.py
  python3 test_imu_heading_fusion.py --speed 0.10 --duration 4 --pairs 3
  python3 test_imu_heading_fusion.py --stop-bringup
  python3 test_imu_heading_fusion.py --alpha 0.95   # test a different alpha
"""

import argparse
import csv
import math
import os
import struct
import subprocess
import sys
import time
from dataclasses import dataclass, fields
from datetime import datetime
from pathlib import Path
from typing import Optional, List

# ── Colours ───────────────────────────────────────────────────────────────────
RED = "\033[0;31m"; GRN = "\033[0;32m"; YLW = "\033[1;33m"
CYN = "\033[0;36m"; BLD = "\033[1m";    DIM = "\033[2m"; NC  = "\033[0m"

# ── Dynamixel constants ───────────────────────────────────────────────────────
DXL_PORTS   = ("/dev/ttyTB3", "/dev/ttyACM0", "/dev/ttyACM1")
DXL_ID      = 200
DXL_BAUD    = 1_000_000

# ── Firmware register addresses (must match firmware/main.c) ─────────────────
REG_MILLIS           = 10
REG_IMU_ANG_VEL_Z    = 68    # float — gyro Z rad/s
REG_PRESENT_VEL_L    = 128   # int32 — Dynamixel RPM units
REG_PRESENT_VEL_R    = 132
REG_PRESENT_POS_L    = 136   # int32 — Dynamixel ticks
REG_PRESENT_POS_R    = 140
REG_MOTOR_TORQUE     = 149   # uint8
REG_CMD_LINEAR_X     = 150   # int32, 0.01 m/s
REG_CMD_ANGULAR_Z    = 170   # int32, 0.01 rad/s
REG_DBG_IMU_SOURCE   = 174   # uint8: 0=sim, 1=BNO085, 2=BNO055
REG_ENC_L_COUNT      = 184   # int32 — raw quadrature
REG_ENC_R_COUNT      = 188
REG_DBG_VEL_L        = 212   # float — measured m/s
REG_DBG_VEL_R        = 216
REG_HDG_I_SEED_FWD   = 256   # float
REG_HDG_I_SEED_REV   = 260   # float
REG_HDG_ITERM        = 264   # float — current integral accumulator
REG_VEL_TRIM_FWD     = 268   # float
REG_VEL_TRIM_REV     = 272   # float
REG_HEADING_HOLD_KP  = 280   # float
REG_DBG_HEADING_ERR  = 284   # float
REG_HEADING_HOLD_EN  = 288   # uint8
REG_HEADING_HOLD_KI  = 292   # float
REG_HEADING_CORR     = 296   # float
REG_ENC_TRIM_KP      = 300   # float
REG_DBG_ENC_TRIM     = 304   # float
REG_DBG_ENC_DIFF     = 308   # float
REG_HEADING_HOLD_KD  = 312   # float

# New IMU fusion registers
REG_IMU_HEADING_ALPHA = 316  # float — complementary filter α
REG_IMU_HEADING_EN    = 320  # uint8 — 1=enabled, 0=disabled
REG_DBG_HEADING_FUSED = 324  # float — fused heading error
REG_DBG_HEADING_GYRO  = 328  # float — gyro-only integrated heading
REG_DBG_HEADING_ENC   = 332  # float — encoder-only heading error
REG_IMU_HEADING_BIAS_BETA = 336  # float — gyro bias learning rate
REG_DBG_GYRO_BIAS     = 340  # float — estimated gyro-Z bias (rad/s)

# Bulk read: 68..343 = 276 bytes (includes bias registers)
BULK_ADDR = REG_IMU_ANG_VEL_Z   # 68
BULK_LEN  = (REG_DBG_GYRO_BIAS + 4) - BULK_ADDR  # 276 bytes

WHEEL_RADIUS = 0.033
WHEEL_BASE   = 0.121642
DXL_VEL_UNIT = 0.229
RPM_TO_RADS  = 2.0 * math.pi / 60.0

# ── CRC ───────────────────────────────────────────────────────────────────────
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

# ── Packet builders ───────────────────────────────────────────────────────────
def _make_read_pkt(addr: int, length: int) -> bytes:
    params = struct.pack("<HH", addr, length)
    plen = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, DXL_ID,
                 plen & 0xFF, (plen >> 8) & 0xFF, 0x02]) + params
    crc = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])

def _make_write_i32_pkt(addr: int, value: int) -> bytes:
    params = struct.pack("<H", addr) + struct.pack("<i", value)
    plen = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, DXL_ID,
                 plen & 0xFF, (plen >> 8) & 0xFF, 0x03]) + params
    crc = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])

def _make_write_f32_pkt(addr: int, value: float) -> bytes:
    params = struct.pack("<H", addr) + struct.pack("<f", value)
    plen = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, DXL_ID,
                 plen & 0xFF, (plen >> 8) & 0xFF, 0x03]) + params
    crc = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])

def _make_write_u8_pkt(addr: int, value: int) -> bytes:
    params = struct.pack("<H", addr) + bytes([value & 0xFF])
    plen = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, DXL_ID,
                 plen & 0xFF, (plen >> 8) & 0xFF, 0x03]) + params
    crc = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])

# ── Response reader ───────────────────────────────────────────────────────────
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

# ── High-level serial helpers ─────────────────────────────────────────────────
def dxl_write_and_drain(ser, pkt: bytes, timeout: float = 0.05):
    ser.write(pkt)
    # Drain the status response (write returns a short status packet)
    _read_response(ser, expected=0, timeout=timeout)

def dxl_read_i32(ser, addr: int) -> Optional[int]:
    ser.write(_make_read_pkt(addr, 4))
    data = _read_response(ser, 4)
    if data and len(data) >= 4:
        return struct.unpack_from("<i", data, 0)[0]
    return None

def dxl_read_f32(ser, addr: int) -> Optional[float]:
    ser.write(_make_read_pkt(addr, 4))
    data = _read_response(ser, 4)
    if data and len(data) >= 4:
        return struct.unpack_from("<f", data, 0)[0]
    return None

def dxl_read_u8(ser, addr: int) -> Optional[int]:
    ser.write(_make_read_pkt(addr, 1))
    data = _read_response(ser, 1)
    if data and len(data) >= 1:
        return data[0]
    return None

def keep_alive(ser):
    dxl_read_i32(ser, REG_MILLIS)

# ── Firmware register snapshot ────────────────────────────────────────────────
@dataclass
class FusionSample:
    """One row of telemetry data, captured at ~10 Hz."""
    ts:              float = 0.0   # wall clock offset from pass start (s)
    phase:           str   = ""    # "fwd" or "rev"
    mode:            str   = ""    # "enc_only" or "fused"
    pair:            int   = 0
    # Heading signals
    heading_fused:   float = 0.0   # complementary filter output (rad)
    heading_gyro:    float = 0.0   # gyro-only integrated (rad)
    heading_enc:     float = 0.0   # encoder-only (rad)
    heading_err:     float = 0.0   # active heading error used by PID (rad)
    heading_corr:    float = 0.0   # angular correction applied (rad/s)
    heading_iterm:   float = 0.0   # integral accumulator
    # IMU raw
    gyro_z:          float = 0.0   # raw IMU angular velocity Z (rad/s)
    # Wheels
    vel_l:           float = 0.0   # measured left velocity (m/s)
    vel_r:           float = 0.0   # measured right velocity (m/s)
    enc_diff:        float = 0.0   # accumulated L−R distance (m)
    # Config snapshot
    alpha:           float = 0.0   # complementary filter weight
    fusion_en:       int   = 0     # fusion enable flag
    hh_en:           int   = 0     # heading hold enable flag
    gyro_bias:       float = 0.0   # estimated gyro-Z bias (rad/s)


def do_bulk_read(ser) -> Optional[FusionSample]:
    """Single bulk read of all relevant registers."""
    ser.write(_make_read_pkt(BULK_ADDR, BULK_LEN))
    data = _read_response(ser, BULK_LEN, timeout=0.15)
    if data is None or len(data) < BULK_LEN:
        return None

    def f32(addr): return struct.unpack_from("<f", data, addr - BULK_ADDR)[0]
    def i32(addr): return struct.unpack_from("<i", data, addr - BULK_ADDR)[0]
    def u8(addr):  return data[addr - BULK_ADDR]

    s = FusionSample()
    s.ts = time.monotonic()
    s.gyro_z        = f32(REG_IMU_ANG_VEL_Z)
    s.vel_l         = f32(REG_DBG_VEL_L)
    s.vel_r         = f32(REG_DBG_VEL_R)
    s.heading_err   = f32(REG_DBG_HEADING_ERR)
    s.heading_corr  = f32(REG_HEADING_CORR)
    s.heading_iterm = f32(REG_HDG_ITERM)
    s.enc_diff      = f32(REG_DBG_ENC_DIFF)
    s.hh_en         = u8(REG_HEADING_HOLD_EN)
    s.heading_fused = f32(REG_DBG_HEADING_FUSED)
    s.heading_gyro  = f32(REG_DBG_HEADING_GYRO)
    s.heading_enc   = f32(REG_DBG_HEADING_ENC)
    s.alpha         = f32(REG_IMU_HEADING_ALPHA)
    s.fusion_en     = u8(REG_IMU_HEADING_EN)
    s.gyro_bias     = f32(REG_DBG_GYRO_BIAS)
    return s


# ── Service management ────────────────────────────────────────────────────────
BRINGUP_SERVICE = "turtlebot3-bringup.service"

def bringup_active():
    try:
        r = subprocess.run(["systemctl", "is-active", BRINGUP_SERVICE],
                           capture_output=True, text=True)
        return r.stdout.strip() == "active"
    except Exception:
        return False

def bringup_stop():
    subprocess.run(["sudo", "systemctl", "stop", BRINGUP_SERVICE],
                   capture_output=True)
    time.sleep(1.0)
    return not bringup_active()

def bringup_start():
    subprocess.run(["sudo", "systemctl", "start", BRINGUP_SERVICE],
                   capture_output=True)
    time.sleep(2.0)

# ── Analysis helpers ──────────────────────────────────────────────────────────

def analyse_pass(samples: List[FusionSample]) -> dict:
    """Compute summary statistics for one driving pass."""
    if len(samples) < 3:
        return {}
    duration = samples[-1].ts - samples[0].ts

    # Heading signals at end of pass
    final = samples[-1]
    hdg_fused_final = final.heading_fused
    hdg_gyro_final  = final.heading_gyro
    hdg_enc_final   = final.heading_enc
    hdg_err_final   = final.heading_err

    # Max absolute heading error during pass
    max_hdg_err = max(abs(s.heading_err) for s in samples)

    # Heading error RMS
    hdg_rms = math.sqrt(sum(s.heading_err**2 for s in samples) / len(samples))

    # Gyro-Z stats (oscillation / bias)
    gyro_zs = [s.gyro_z for s in samples]
    gyro_mean = sum(gyro_zs) / len(gyro_zs)
    gyro_std = math.sqrt(sum((g - gyro_mean)**2 for g in gyro_zs) / max(len(gyro_zs)-1, 1))

    # Enc diff at end
    enc_diff_final = final.enc_diff

    # Velocity asymmetry
    vel_diffs = [s.vel_l - s.vel_r for s in samples]
    vel_diff_mean = sum(vel_diffs) / len(vel_diffs)

    # Heading correction stats
    corrs = [s.heading_corr for s in samples]
    corr_mean = sum(corrs) / len(corrs)
    corr_max = max(abs(c) for c in corrs)

    # Integral term at end
    iterm_final = final.heading_iterm

    return {
        "duration": duration,
        "n_samples": len(samples),
        "hdg_fused_deg": math.degrees(hdg_fused_final),
        "hdg_gyro_deg": math.degrees(hdg_gyro_final),
        "hdg_enc_deg": math.degrees(hdg_enc_final),
        "hdg_err_deg": math.degrees(hdg_err_final),
        "max_hdg_err_deg": math.degrees(max_hdg_err),
        "hdg_rms_deg": math.degrees(hdg_rms),
        "gyro_mean_dps": math.degrees(gyro_mean),
        "gyro_std_dps": math.degrees(gyro_std),
        "enc_diff_mm": enc_diff_final * 1000.0,
        "vel_diff_mean_mms": vel_diff_mean * 1000.0,
        "corr_mean_dps": math.degrees(corr_mean),
        "corr_max_dps": math.degrees(corr_max),
        "iterm": iterm_final,
        "alpha": final.alpha,
        "fusion_en": final.fusion_en,
    }


def print_pass_summary(label: str, stats: dict):
    """Pretty-print one pass result."""
    d = stats.get("hdg_err_deg", 0)
    mx = stats.get("max_hdg_err_deg", 0)
    rms = stats.get("hdg_rms_deg", 0)
    # Colour: green if < 1°, yellow < 3°, red >= 3°
    clr = GRN if abs(d) < 1.0 else (YLW if abs(d) < 3.0 else RED)

    print(f"\n  {BLD}{label}{NC}")
    print(f"    Heading error (active):  {clr}{d:+.2f}°{NC}  "
          f"(max {mx:.2f}°, RMS {rms:.2f}°)")
    print(f"    Fused heading:  {stats.get('hdg_fused_deg',0):+.2f}°   "
          f"Gyro-only: {stats.get('hdg_gyro_deg',0):+.2f}°   "
          f"Enc-only: {stats.get('hdg_enc_deg',0):+.2f}°")
    print(f"    Gyro-Z:  mean {stats.get('gyro_mean_dps',0):+.2f}°/s  "
          f"std {stats.get('gyro_std_dps',0):.2f}°/s")
    print(f"    Enc diff: {stats.get('enc_diff_mm',0):+.2f} mm   "
          f"Vel L-R: {stats.get('vel_diff_mean_mms',0):+.1f} mm/s")
    print(f"    Correction: mean {stats.get('corr_mean_dps',0):+.2f}°/s  "
          f"max {stats.get('corr_max_dps',0):.2f}°/s   "
          f"iterm={stats.get('iterm',0):+.4f}")
    print(f"    α={stats.get('alpha',0):.2f}  fusion_en={stats.get('fusion_en',0)}  "
          f"({stats.get('n_samples',0)} samples, {stats.get('duration',0):.1f}s)")


# ── Main test runner ──────────────────────────────────────────────────────────

def run_pass(ser, speed_mps: float, duration_s: float, phase: str,
             mode: str, pair: int) -> List[FusionSample]:
    """Drive forward or reverse for `duration_s` seconds, collecting samples."""
    import serial as pyserial  # deferred import since it's already open

    lin_val = int(round(speed_mps * 100.0))
    if phase == "rev":
        lin_val = -lin_val

    lin_pkt = _make_write_i32_pkt(REG_CMD_LINEAR_X, lin_val)
    ang_pkt = _make_write_i32_pkt(REG_CMD_ANGULAR_Z, 0)

    samples: List[FusionSample] = []
    t0 = time.monotonic()

    while time.monotonic() - t0 < duration_s:
        # Command velocity (also keeps host timeout alive)
        dxl_write_and_drain(ser, lin_pkt)
        dxl_write_and_drain(ser, ang_pkt)

        # Bulk read telemetry
        s = do_bulk_read(ser)
        if s is not None:
            s.ts = s.ts - t0  # relative time
            s.phase = phase
            s.mode = mode
            s.pair = pair
            samples.append(s)

        time.sleep(0.08)  # ~10-12 Hz sampling (limited by serial round-trips)

    # Stop
    dxl_write_and_drain(ser, _make_write_i32_pkt(REG_CMD_LINEAR_X, 0))
    dxl_write_and_drain(ser, _make_write_i32_pkt(REG_CMD_ANGULAR_Z, 0))
    keep_alive(ser)

    return samples


def run_settle(ser, settle_s: float = 1.5):
    """Stop motors and wait for the robot to settle."""
    dxl_write_and_drain(ser, _make_write_i32_pkt(REG_CMD_LINEAR_X, 0))
    dxl_write_and_drain(ser, _make_write_i32_pkt(REG_CMD_ANGULAR_Z, 0))
    t0 = time.monotonic()
    while time.monotonic() - t0 < settle_s:
        keep_alive(ser)
        time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(
        description="Test IMU-encoder complementary filter heading fusion")
    parser.add_argument("--speed", type=float, default=0.10,
                        help="Drive speed m/s (default 0.10)")
    parser.add_argument("--duration", type=float, default=4.0,
                        help="Per-pass drive duration seconds (default 4)")
    parser.add_argument("--pairs", type=int, default=2,
                        help="Number of fwd+rev pairs per mode (default 2)")
    parser.add_argument("--alpha", type=float, default=None,
                        help="Override complementary filter α (default: use firmware default)")
    parser.add_argument("--port", default=None,
                        help="Serial port (auto-detected if omitted)")
    parser.add_argument("--stop-bringup", action="store_true",
                        help="Stop turtlebot3-bringup before test")
    parser.add_argument("--settle", type=float, default=2.0,
                        help="Settle time between passes (default 2.0s)")
    args = parser.parse_args()

    # ── Bringup management ───────────────────────────────────
    stopped_bringup = False
    if args.stop_bringup and bringup_active():
        print(f"{YLW}Stopping bringup service ...{NC}", flush=True)
        bringup_stop()
        stopped_bringup = True
        print(f"  {GRN}stopped{NC}")
    elif bringup_active():
        print(f"{RED}ERROR: turtlebot3-bringup is running — cannot take serial port.{NC}")
        print(f"  Run with --stop-bringup to stop it automatically.")
        return 1

    # ── Serial port ──────────────────────────────────────────
    import serial as pyserial
    port = args.port
    ser = None
    if port is None:
        for candidate in DXL_PORTS:
            if os.path.exists(candidate):
                port = candidate
                break
    if port is None:
        print(f"{RED}ERROR: No serial port found.{NC}")
        return 1

    try:
        ser = pyserial.Serial(port, DXL_BAUD, timeout=0.05)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.5)
        ser.reset_input_buffer()

        # ── Ping ─────────────────────────────────────────────
        print(f"\n{BLD}=== IMU Heading Fusion Test ==={NC}")
        print(f"Port: {port}")
        millis = None
        for _ in range(20):
            millis = dxl_read_i32(ser, REG_MILLIS)
            if millis is not None:
                break
            time.sleep(0.3)
        if millis is None:
            print(f"{RED}FAILED to ping Pico — check connection{NC}")
            return 1
        print(f"Pico online (uptime: {millis} ms)")

        # ── Read firmware state ──────────────────────────────
        imu_src = dxl_read_u8(ser, REG_DBG_IMU_SOURCE)
        imu_names = {0: "simulated", 1: "BNO085", 2: "BNO055"}
        imu_name = imu_names.get(imu_src, f"unknown({imu_src})")

        hh_en = dxl_read_u8(ser, REG_HEADING_HOLD_EN)
        hh_kp = dxl_read_f32(ser, REG_HEADING_HOLD_KP)
        hh_ki = dxl_read_f32(ser, REG_HEADING_HOLD_KI)
        hh_kd = dxl_read_f32(ser, REG_HEADING_HOLD_KD)
        fusion_en = dxl_read_u8(ser, REG_IMU_HEADING_EN)
        alpha_hw = dxl_read_f32(ser, REG_IMU_HEADING_ALPHA)
        gyro_z_rest = dxl_read_f32(ser, REG_IMU_ANG_VEL_Z)
        i_seed_fwd = dxl_read_f32(ser, REG_HDG_I_SEED_FWD)
        i_seed_rev = dxl_read_f32(ser, REG_HDG_I_SEED_REV)
        vel_trim_fwd = dxl_read_f32(ser, REG_VEL_TRIM_FWD)
        vel_trim_rev = dxl_read_f32(ser, REG_VEL_TRIM_REV)

        print(f"\n{BLD}Firmware configuration:{NC}")
        print(f"  IMU source:       {imu_name}")
        print(f"  Heading hold:     EN={hh_en}  Kp={hh_kp}  Ki={hh_ki}  Kd={hh_kd}")
        print(f"  IMU fusion:       EN={fusion_en}  α={alpha_hw}")
        print(f"  Gyro-Z at rest:   {gyro_z_rest:+.6f} rad/s  "
              f"({math.degrees(gyro_z_rest or 0):+.3f}°/s)")
        print(f"  I-seeds:          fwd={i_seed_fwd}  rev={i_seed_rev}")
        print(f"  Vel trims:        fwd={vel_trim_fwd}  rev={vel_trim_rev}")

        if imu_src == 0:
            print(f"\n  {YLW}WARNING: IMU is simulated — fusion will have no effect!{NC}")
            print(f"  {YLW}Test will still run to verify register plumbing.{NC}")

        desired_alpha = args.alpha if args.alpha is not None else (alpha_hw or 0.98)

        print(f"\n{BLD}Test plan:{NC}")
        print(f"  Speed:    {args.speed:.2f} m/s")
        print(f"  Duration: {args.duration:.1f}s per pass")
        print(f"  Pairs:    {args.pairs} (fwd+rev each)")
        print(f"  Modes:    1) encoder-only (fusion OFF)")
        print(f"            2) fused (fusion ON, α={desired_alpha:.3f})")
        print(f"  Settle:   {args.settle:.1f}s between passes")
        total_time = args.pairs * 2 * (args.duration + args.settle) * 2
        print(f"  Est. time: ~{total_time:.0f}s")

        # Prepare CSV
        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = Path(f"test_imu_fusion_{ts_str}.csv")
        csv_fields = [f.name for f in fields(FusionSample)]

        all_samples: List[FusionSample] = []
        all_stats: List[dict] = []

        # Enable torque
        dxl_write_and_drain(ser, _make_write_u8_pkt(REG_MOTOR_TORQUE, 1))

        # ── Phase 1: Encoder-only ────────────────────────────
        print(f"\n{'='*60}")
        print(f"{BLD}{CYN}Phase 1: ENCODER-ONLY (fusion disabled){NC}")
        print(f"{'='*60}")
        # Disable fusion
        dxl_write_and_drain(ser, _make_write_u8_pkt(REG_IMU_HEADING_EN, 0))
        time.sleep(0.1)
        verify_en = dxl_read_u8(ser, REG_IMU_HEADING_EN)
        print(f"  IMU fusion confirmed OFF (reg={verify_en})")
        run_settle(ser, 1.0)

        for p in range(1, args.pairs + 1):
            for phase in ("fwd", "rev"):
                lbl = f"enc-only pair {p} {phase.upper()}"
                print(f"\n  {BLD}▶ {lbl}:{NC} driving ...", end="", flush=True)
                samples = run_pass(ser, args.speed, args.duration,
                                   phase, "enc_only", p)
                all_samples.extend(samples)
                stats = analyse_pass(samples)
                stats["label"] = lbl
                stats["mode"] = "enc_only"
                stats["phase"] = phase
                stats["pair"] = p
                all_stats.append(stats)
                d = stats.get("hdg_err_deg", 0)
                clr = GRN if abs(d) < 1 else (YLW if abs(d) < 3 else RED)
                print(f" done — heading err: {clr}{d:+.2f}°{NC}  "
                      f"(fused={stats.get('hdg_fused_deg',0):+.2f}°  "
                      f"gyro={stats.get('hdg_gyro_deg',0):+.2f}°  "
                      f"enc={stats.get('hdg_enc_deg',0):+.2f}°)")
                run_settle(ser, args.settle)

        # ── Phase 2: Fused ───────────────────────────────────
        print(f"\n{'='*60}")
        print(f"{BLD}{CYN}Phase 2: IMU-FUSED (α={desired_alpha:.3f}){NC}")
        print(f"{'='*60}")
        # Enable fusion and set alpha
        dxl_write_and_drain(ser, _make_write_f32_pkt(REG_IMU_HEADING_ALPHA, desired_alpha))
        dxl_write_and_drain(ser, _make_write_u8_pkt(REG_IMU_HEADING_EN, 1))
        time.sleep(0.1)
        verify_en = dxl_read_u8(ser, REG_IMU_HEADING_EN)
        verify_a = dxl_read_f32(ser, REG_IMU_HEADING_ALPHA)
        print(f"  IMU fusion confirmed ON (reg={verify_en}, α={verify_a:.3f})")
        run_settle(ser, 1.0)

        for p in range(1, args.pairs + 1):
            for phase in ("fwd", "rev"):
                lbl = f"fused pair {p} {phase.upper()}"
                print(f"\n  {BLD}▶ {lbl}:{NC} driving ...", end="", flush=True)
                samples = run_pass(ser, args.speed, args.duration,
                                   phase, "fused", p)
                all_samples.extend(samples)
                stats = analyse_pass(samples)
                stats["label"] = lbl
                stats["mode"] = "fused"
                stats["phase"] = phase
                stats["pair"] = p
                all_stats.append(stats)
                d = stats.get("hdg_err_deg", 0)
                clr = GRN if abs(d) < 1 else (YLW if abs(d) < 3 else RED)
                print(f" done — heading err: {clr}{d:+.2f}°{NC}  "
                      f"(fused={stats.get('hdg_fused_deg',0):+.2f}°  "
                      f"gyro={stats.get('hdg_gyro_deg',0):+.2f}°  "
                      f"enc={stats.get('hdg_enc_deg',0):+.2f}°)")
                run_settle(ser, args.settle)

        # Stop motors
        dxl_write_and_drain(ser, _make_write_i32_pkt(REG_CMD_LINEAR_X, 0))
        dxl_write_and_drain(ser, _make_write_i32_pkt(REG_CMD_ANGULAR_Z, 0))

        # Restore fusion to its original state
        dxl_write_and_drain(ser, _make_write_u8_pkt(REG_IMU_HEADING_EN,
                                                     fusion_en if fusion_en is not None else 1))
        if alpha_hw is not None:
            dxl_write_and_drain(ser, _make_write_f32_pkt(REG_IMU_HEADING_ALPHA, alpha_hw))

        # ── Write CSV ────────────────────────────────────────
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=csv_fields)
            writer.writeheader()
            for s in all_samples:
                writer.writerow({name: getattr(s, name) for name in csv_fields})
        print(f"\n{DIM}CSV saved: {csv_path}  ({len(all_samples)} samples){NC}")

        # ── Comparison table ─────────────────────────────────
        print(f"\n{'='*72}")
        print(f"{BLD}                   COMPARISON: ENCODER-ONLY   vs   IMU-FUSED{NC}")
        print(f"{'='*72}")

        # Aggregate by mode
        for mode_label, mode_key in [("ENCODER-ONLY", "enc_only"),
                                      ("IMU-FUSED", "fused")]:
            mode_stats = [s for s in all_stats if s.get("mode") == mode_key]
            if not mode_stats:
                continue
            print(f"\n  {BLD}{mode_label}{NC}")
            for direction in ("fwd", "rev"):
                dir_stats = [s for s in mode_stats if s.get("phase") == direction]
                if not dir_stats:
                    continue
                avg_err = sum(s["hdg_err_deg"] for s in dir_stats) / len(dir_stats)
                avg_rms = sum(s["hdg_rms_deg"] for s in dir_stats) / len(dir_stats)
                avg_max = sum(s["max_hdg_err_deg"] for s in dir_stats) / len(dir_stats)
                avg_gyro_std = sum(s["gyro_std_dps"] for s in dir_stats) / len(dir_stats)
                avg_enc_diff = sum(s["enc_diff_mm"] for s in dir_stats) / len(dir_stats)
                clr = GRN if abs(avg_err) < 1 else (YLW if abs(avg_err) < 3 else RED)
                dir_up = direction.upper()
                print(f"    {dir_up:>3s}: err={clr}{avg_err:+.2f}°{NC}  "
                      f"RMS={avg_rms:.2f}°  max={avg_max:.2f}°  "
                      f"gyro_std={avg_gyro_std:.2f}°/s  "
                      f"enc_diff={avg_enc_diff:+.1f}mm")

        # Per-pass detail
        print(f"\n  {DIM}{'Pass':<25s} {'ErrEnd':>7s} {'ErrMax':>7s} "
              f"{'RMS':>6s} {'Fused':>7s} {'Gyro':>7s} {'Enc':>7s} "
              f"{'EncDif':>7s} {'Corr':>7s} {'α':>5s}{NC}")
        print(f"  {DIM}{'-'*90}{NC}")
        for s in all_stats:
            label = s.get("label", "?")
            err = s.get("hdg_err_deg", 0)
            clr = GRN if abs(err) < 1 else (YLW if abs(err) < 3 else RED)
            print(f"  {label:<25s} {clr}{err:+7.2f}{NC} "
                  f"{s.get('max_hdg_err_deg',0):7.2f} "
                  f"{s.get('hdg_rms_deg',0):6.2f} "
                  f"{s.get('hdg_fused_deg',0):+7.2f} "
                  f"{s.get('hdg_gyro_deg',0):+7.2f} "
                  f"{s.get('hdg_enc_deg',0):+7.2f} "
                  f"{s.get('enc_diff_mm',0):+7.1f} "
                  f"{s.get('corr_mean_dps',0):+7.2f} "
                  f"{s.get('alpha',0):5.2f}")

        # ── Improvement summary ──────────────────────────────
        enc_stats = [s for s in all_stats if s.get("mode") == "enc_only"]
        fused_stats = [s for s in all_stats if s.get("mode") == "fused"]

        if enc_stats and fused_stats:
            enc_avg_abs = sum(abs(s["hdg_err_deg"]) for s in enc_stats) / len(enc_stats)
            fused_avg_abs = sum(abs(s["hdg_err_deg"]) for s in fused_stats) / len(fused_stats)

            enc_avg_rms = sum(s["hdg_rms_deg"] for s in enc_stats) / len(enc_stats)
            fused_avg_rms = sum(s["hdg_rms_deg"] for s in fused_stats) / len(fused_stats)

            print(f"\n{'='*72}")
            print(f"{BLD}RESULT SUMMARY{NC}")
            print(f"{'='*72}")
            print(f"  Avg |heading error| — encoder-only: {enc_avg_abs:.2f}°   fused: {fused_avg_abs:.2f}°")
            print(f"  Avg heading RMS    — encoder-only: {enc_avg_rms:.2f}°   fused: {fused_avg_rms:.2f}°")

            if enc_avg_abs > 0.01:
                pct = (1.0 - fused_avg_abs / enc_avg_abs) * 100.0
                if pct > 0:
                    print(f"  {GRN}► Fusion IMPROVED heading by {pct:.0f}%{NC}")
                elif pct < -10:
                    print(f"  {RED}► Fusion DEGRADED heading by {abs(pct):.0f}%{NC}")
                else:
                    print(f"  {YLW}► Similar performance (Δ {pct:+.0f}%){NC}")
            else:
                print(f"  {DIM}(encoder-only error too small to compare){NC}")

            if imu_src == 0:
                print(f"\n  {YLW}NOTE: IMU is simulated — gyro reads are always zero.{NC}")
                print(f"  {YLW}With a real IMU (BNO055/BNO085), expect larger differences.{NC}")

        print(f"\n  CSV: {csv_path}")
        print()

        ser.close()
        return 0

    except KeyboardInterrupt:
        print(f"\n{YLW}Interrupted — stopping motors{NC}")
        if ser and ser.is_open:
            try:
                dxl_write_and_drain(ser, _make_write_i32_pkt(REG_CMD_LINEAR_X, 0))
                dxl_write_and_drain(ser, _make_write_i32_pkt(REG_CMD_ANGULAR_Z, 0))
            except Exception:
                pass
            ser.close()
        return 130

    except Exception as e:
        print(f"\n{RED}ERROR: {e}{NC}")
        import traceback
        traceback.print_exc()
        if ser and ser.is_open:
            try:
                dxl_write_and_drain(ser, _make_write_i32_pkt(REG_CMD_LINEAR_X, 0))
                dxl_write_and_drain(ser, _make_write_i32_pkt(REG_CMD_ANGULAR_Z, 0))
            except Exception:
                pass
            ser.close()
        return 1

    finally:
        if stopped_bringup:
            print(f"\n{YLW}Restarting bringup service ...{NC}")
            bringup_start()
            print(f"  {GRN}bringup restarted{NC}")


if __name__ == "__main__":
    raise SystemExit(main())
