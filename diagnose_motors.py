#!/usr/bin/env python3
"""
diagnose_motors.py  —  Raw motor asymmetry diagnostic

Drives both motors at identical commanded velocities with heading-hold
DISABLED so no steering correction masks the hardware asymmetry.
Tests multiple speeds in both directions, reports per-motor velocity,
startup lag, steady-state asymmetry, and gives a clear verdict on
whether a motor swap is likely to help.

Usage:
    ./diagnose_motors.py                 # full test
    ./diagnose_motors.py --speeds 0.05 0.10 0.15   # custom speeds
    ./diagnose_motors.py --duration 8    # longer per-pass duration
    ./diagnose_motors.py --stop-bringup  # stop ros bringup first (recommended)
    ./diagnose_motors.py --csv out.csv   # save raw samples to CSV

Requires: pyserial (serial port to Robo Pico firmware via Dynamixel v2 protocol)
"""
from __future__ import annotations
import argparse
import math
import os
import struct
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional

# ── Dynamixel v2 constants ───────────────────────────────────────────────────
DXL_PORTS   = ("/dev/ttyTB3", "/dev/ttyACM0", "/dev/ttyACM1")
DXL_ID      = 200
DXL_BAUD    = 1_000_000

# Register addresses (matching firmware/main.c)
REG_CMD_LINEAR_X    = 150   # int32, units of 0.01 m/s
REG_CMD_ANGULAR_Z   = 170   # int32, units of 0.01 rad/s
REG_MOTOR_TORQUE_EN = 149   # uint8

REG_ENC_L_COUNT     = 184   # int32 — raw quadrature counts
REG_ENC_R_COUNT     = 188   # int32
REG_ENC_RESET       = 192   # uint8 — write non-zero to reset

REG_DBG_VEL_L       = 212   # float — measured left  wheel velocity (m/s)
REG_DBG_VEL_R       = 216   # float — measured right wheel velocity (m/s)
REG_HEADING_HOLD_EN = 288   # uint8 — 1=enabled, 0=disabled
REG_HEADING_HOLD_KP = 280   # float
REG_HEADING_HOLD_KI = 292   # float
REG_HEADING_HOLD_KD = 312   # float
REG_HEADING_CORR    = 296   # float — ang_z correction (rad/s)
REG_IMU_ANG_VEL_Z   = 68    # float — gyro Z (rad/s)

REG_VEL_TRIM_FWD    = 268   # float — velocity trim forward
REG_VEL_TRIM_REV    = 272   # float — velocity trim reverse
REG_HDG_I_SEED_FWD  = 256   # float
REG_HDG_I_SEED_REV  = 260   # float

# Bulk read range: 68..315 = 248 bytes (same as diagnose_reverse.py)
BULK_ADDR = REG_IMU_ANG_VEL_Z   # 68
BULK_LEN  = (REG_HEADING_HOLD_KD - BULK_ADDR) + 4  # 248 bytes

WHEEL_RADIUS    = 0.033   # m
WHEEL_BASE      = 0.121642  # m
DXL_VEL_UNIT    = 0.229   # RPM per unit
RPM_TO_RADS     = 2.0 * math.pi / 60.0
ENC_COUNTS_PER_REV = 3840  # quadrature counts per wheel revolution
ENC_M_PER_COUNT = (2.0 * math.pi * WHEEL_RADIUS) / ENC_COUNTS_PER_REV

# Colours
RED = "\033[91m"
GRN = "\033[92m"
YLW = "\033[93m"
CYN = "\033[96m"
BLD = "\033[1m"
NC  = "\033[0m"


# ── CRC-16 (Dynamixel v2) ────────────────────────────────────────────────────
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


# ── Packet builders ──────────────────────────────────────────────────────────
def _make_write_i32_pkt(addr: int, value: int) -> bytes:
    params = struct.pack("<H", addr) + struct.pack("<i", value)
    plen   = len(params) + 3
    hdr    = bytes([0xFF, 0xFF, 0xFD, 0x00, DXL_ID,
                    plen & 0xFF, (plen >> 8) & 0xFF, 0x03]) + params
    crc    = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])

def _make_write_f32_pkt(addr: int, value: float) -> bytes:
    params = struct.pack("<H", addr) + struct.pack("<f", value)
    plen   = len(params) + 3
    hdr    = bytes([0xFF, 0xFF, 0xFD, 0x00, DXL_ID,
                    plen & 0xFF, (plen >> 8) & 0xFF, 0x03]) + params
    crc    = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])

def _make_write_u8_pkt(addr: int, value: int) -> bytes:
    params = struct.pack("<H", addr) + bytes([value & 0xFF])
    plen   = len(params) + 3
    hdr    = bytes([0xFF, 0xFF, 0xFD, 0x00, DXL_ID,
                    plen & 0xFF, (plen >> 8) & 0xFF, 0x03]) + params
    crc    = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])

def _make_read_pkt(addr: int, length: int) -> bytes:
    params = struct.pack("<HH", addr, length)
    plen   = len(params) + 3
    hdr    = bytes([0xFF, 0xFF, 0xFD, 0x00, DXL_ID,
                    plen & 0xFF, (plen >> 8) & 0xFF, 0x02]) + params
    crc    = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])


# ── Serial helpers ───────────────────────────────────────────────────────────
def _read_response(ser, expected: int, timeout: float = 0.25) -> Optional[bytes]:
    HEADER = b"\xFF\xFF\xFD\x00"
    t0  = time.monotonic()
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
            plen       = buf[5] | (buf[6] << 8)
            num_params = plen - 4
            total      = 7 + plen
            if len(buf) < total:
                break
            if num_params != expected:
                buf = buf[4:]
                continue
            return buf[9:9 + expected]
    return None


def _send_vel_cmd(ser, v: float, a: float = 0.0) -> None:
    lin_val = int(round(v * 100.0))
    ang_val = int(round(a * 100.0))
    ser.write(_make_write_i32_pkt(REG_CMD_LINEAR_X, lin_val))
    time.sleep(0.003)
    ser.read(max(ser.in_waiting, 1))
    ser.write(_make_write_i32_pkt(REG_CMD_ANGULAR_Z, ang_val))
    time.sleep(0.003)
    ser.read(max(ser.in_waiting, 1))


def _write_f32(ser, addr: int, value: float) -> None:
    ser.write(_make_write_f32_pkt(addr, value))
    time.sleep(0.003)
    ser.read(max(ser.in_waiting, 1))


def _write_u8(ser, addr: int, value: int) -> None:
    ser.write(_make_write_u8_pkt(addr, value))
    time.sleep(0.003)
    ser.read(max(ser.in_waiting, 1))


def _read_regs(ser) -> Optional[dict]:
    ser.reset_input_buffer()
    ser.write(_make_read_pkt(BULK_ADDR, BULK_LEN))
    raw = _read_response(ser, BULK_LEN, timeout=0.15)
    if raw is None or len(raw) < BULK_LEN:
        return None
    def f32(off): return struct.unpack_from("<f", raw, off - BULK_ADDR)[0]
    def i32(off): return struct.unpack_from("<i", raw, off - BULK_ADDR)[0]
    def u8(off):  return raw[off - BULK_ADDR]
    return {
        "vel_l":    f32(REG_DBG_VEL_L),
        "vel_r":    f32(REG_DBG_VEL_R),
        "enc_l":    i32(REG_ENC_L_COUNT),
        "enc_r":    i32(REG_ENC_R_COUNT),
        "gyro_z":   f32(REG_IMU_ANG_VEL_Z),
        "hdg_corr": f32(REG_HEADING_CORR),
        "hdg_en":   u8(REG_HEADING_HOLD_EN),
    }


# ── Data structures ──────────────────────────────────────────────────────────
@dataclass
class Sample:
    t: float
    vel_l: float
    vel_r: float
    enc_l: int
    enc_r: int
    gyro_z: float

@dataclass
class PassResult:
    speed_cmd: float
    direction: str
    samples: List[Sample] = field(default_factory=list)

    # Computed after collection
    mean_vel_l: float = 0.0
    mean_vel_r: float = 0.0
    std_vel_l:  float = 0.0
    std_vel_r:  float = 0.0
    distance_l: float = 0.0
    distance_r: float = 0.0
    asym_pct:   float = 0.0  # (|L| - |R|) / max(|L|,|R|) * 100
    drift_deg:  float = 0.0
    startup_lag_ms: float = 0.0  # time diff for each motor to start moving
    first_move_l_ms: float = 0.0
    first_move_r_ms: float = 0.0
    ramp_50_l_ms: float = 0.0   # time to reach 50% of target speed
    ramp_50_r_ms: float = 0.0
    mean_gyro_z: float = 0.0
    gyro_drift: float = 0.0


# ── Drive pass (heading-hold disabled) ───────────────────────────────────────
def run_pass(ser, speed: float, duration: float) -> PassResult:
    direction = "forward" if speed > 0 else "reverse"
    pr = PassResult(speed_cmd=speed, direction=direction)

    # Reset encoder counts
    _write_u8(ser, REG_ENC_RESET, 1)
    time.sleep(0.05)

    t0 = time.monotonic()
    while time.monotonic() - t0 < duration:
        _send_vel_cmd(ser, speed)
        regs = _read_regs(ser)
        if regs:
            pr.samples.append(Sample(
                t=time.monotonic() - t0,
                vel_l=regs["vel_l"],
                vel_r=regs["vel_r"],
                enc_l=regs["enc_l"],
                enc_r=regs["enc_r"],
                gyro_z=regs["gyro_z"],
            ))
        time.sleep(0.08)

    # Stop
    _send_vel_cmd(ser, 0.0)
    time.sleep(0.3)

    _analyse_pass(pr, speed)
    return pr


def _analyse_pass(pr: PassResult, target_speed: float):
    if len(pr.samples) < 3:
        return

    # ── Steady-state velocity (second half of samples) ──
    mid = len(pr.samples) // 2
    ss = pr.samples[mid:]
    vls = [s.vel_l for s in ss]
    vrs = [s.vel_r for s in ss]
    pr.mean_vel_l = sum(vls) / len(vls)
    pr.mean_vel_r = sum(vrs) / len(vrs)
    pr.std_vel_l = math.sqrt(sum((v - pr.mean_vel_l)**2 for v in vls) / max(len(vls)-1, 1))
    pr.std_vel_r = math.sqrt(sum((v - pr.mean_vel_r)**2 for v in vrs) / max(len(vrs)-1, 1))

    # ── Total distance (from encoder counts) ──
    pr.distance_l = (pr.samples[-1].enc_l - pr.samples[0].enc_l) * ENC_M_PER_COUNT
    pr.distance_r = (pr.samples[-1].enc_r - pr.samples[0].enc_r) * ENC_M_PER_COUNT

    # ── Asymmetry ──
    abs_l = abs(pr.mean_vel_l)
    abs_r = abs(pr.mean_vel_r)
    denom = max(abs_l, abs_r, 1e-9)
    pr.asym_pct = (abs_l - abs_r) / denom * 100.0

    # ── Heading drift (encoder-based) ──
    dl = pr.distance_l
    dr = pr.distance_r
    if abs(dl + dr) > 1e-6:
        pr.drift_deg = math.degrees((dl - dr) / WHEEL_BASE)

    # ── Gyro ──
    gzs = [s.gyro_z for s in ss]
    pr.mean_gyro_z = math.degrees(sum(gzs) / len(gzs))
    # Integrated gyro drift
    all_gz = [s.gyro_z for s in pr.samples]
    all_ts = [s.t for s in pr.samples]
    gyro_int = 0.0
    for i in range(1, len(all_gz)):
        dt = all_ts[i] - all_ts[i-1]
        gyro_int += all_gz[i] * dt
    pr.gyro_drift = math.degrees(gyro_int)

    # ── Startup timing ──
    target_half = abs(target_speed) * 0.1  # 10% of target as "first move" threshold
    target_50   = abs(target_speed) * 0.5

    first_l = first_r = ramp_l = ramp_r = None
    for s in pr.samples:
        if first_l is None and abs(s.vel_l) > target_half:
            first_l = s.t
        if first_r is None and abs(s.vel_r) > target_half:
            first_r = s.t
        if ramp_l is None and abs(s.vel_l) > target_50:
            ramp_l = s.t
        if ramp_r is None and abs(s.vel_r) > target_50:
            ramp_r = s.t

    if first_l is not None:
        pr.first_move_l_ms = first_l * 1000
    if first_r is not None:
        pr.first_move_r_ms = first_r * 1000
    if first_l is not None and first_r is not None:
        pr.startup_lag_ms = (first_l - first_r) * 1000  # positive = L lags R
    if ramp_l is not None:
        pr.ramp_50_l_ms = ramp_l * 1000
    if ramp_r is not None:
        pr.ramp_50_r_ms = ramp_r * 1000


# ── Printing ─────────────────────────────────────────────────────────────────
def print_pass(pr: PassResult):
    d = pr.direction.upper()
    s_cmd = pr.speed_cmd * 1000
    print(f"\n  {BLD}[ {d} @ {s_cmd:+.0f} mm/s ]{NC}")
    print(f"    Steady-state velocity:  L={pr.mean_vel_l*1000:+7.1f} mm/s  "
          f"R={pr.mean_vel_r*1000:+7.1f} mm/s")
    print(f"    Velocity std:           L={pr.std_vel_l*1000:6.2f} mm/s     "
          f"R={pr.std_vel_r*1000:6.2f} mm/s")
    asym_col = RED if abs(pr.asym_pct) > 3.0 else YLW if abs(pr.asym_pct) > 1.5 else GRN
    print(f"    L−R asymmetry:          {asym_col}{pr.asym_pct:+.2f}%{NC}")
    print(f"    Distance travelled:     L={pr.distance_l*100:+7.2f} cm   "
          f"R={pr.distance_r*100:+7.2f} cm")
    drift_col = RED if abs(pr.drift_deg) > 3.0 else YLW if abs(pr.drift_deg) > 1.5 else GRN
    print(f"    Heading drift (enc):    {drift_col}{pr.drift_deg:+.2f}°{NC}")
    print(f"    Gyro-Z drift:           {pr.gyro_drift:+.2f}°  "
          f"(mean ω={pr.mean_gyro_z:+.1f}°/s)")
    print(f"    Startup: first move     L={pr.first_move_l_ms:.0f} ms  "
          f"R={pr.first_move_r_ms:.0f} ms  "
          f"(lag={pr.startup_lag_ms:+.0f} ms)")
    print(f"    Ramp to 50%:            L={pr.ramp_50_l_ms:.0f} ms  "
          f"R={pr.ramp_50_r_ms:.0f} ms  "
          f"(Δ={pr.ramp_50_l_ms - pr.ramp_50_r_ms:+.0f} ms)")


def print_comparison_table(results: List[PassResult]):
    print(f"\n{'='*72}")
    print(f"  {BLD}MOTOR ASYMMETRY SUMMARY{NC}")
    print(f"{'='*72}")
    print(f"  {'Speed':>10s}  {'Dir':>7s}  {'|Vel L|':>8s}  {'|Vel R|':>8s}  "
          f"{'Asym%':>7s}  {'Drift°':>7s}  {'Lag ms':>7s}")
    print(f"  {'─'*10}  {'─'*7}  {'─'*8}  {'─'*8}  {'─'*7}  {'─'*7}  {'─'*7}")

    for pr in results:
        asym_col = RED if abs(pr.asym_pct) > 3.0 else YLW if abs(pr.asym_pct) > 1.5 else GRN
        drift_col = RED if abs(pr.drift_deg) > 3.0 else YLW if abs(pr.drift_deg) > 1.5 else GRN
        print(f"  {pr.speed_cmd*1000:+10.0f}  {pr.direction:>7s}  "
              f"{abs(pr.mean_vel_l)*1000:7.1f}   {abs(pr.mean_vel_r)*1000:7.1f}   "
              f"{asym_col}{pr.asym_pct:+6.2f}%{NC}  "
              f"{drift_col}{pr.drift_deg:+6.2f}°{NC}  "
              f"{pr.startup_lag_ms:+6.0f}")

    # ── Aggregate analysis ──
    fwd = [p for p in results if p.speed_cmd > 0]
    rev = [p for p in results if p.speed_cmd < 0]

    if fwd and rev:
        avg_fwd_asym = sum(p.asym_pct for p in fwd) / len(fwd)
        avg_rev_asym = sum(p.asym_pct for p in rev) / len(rev)
        avg_fwd_drift = sum(p.drift_deg for p in fwd) / len(fwd)
        avg_rev_drift = sum(p.drift_deg for p in rev) / len(rev)

        print(f"\n  {BLD}Average asymmetry:{NC}")
        print(f"    Forward:  {avg_fwd_asym:+.2f}%  (drift {avg_fwd_drift:+.2f}°)")
        print(f"    Reverse:  {avg_rev_asym:+.2f}%  (drift {avg_rev_drift:+.2f}°)")

        # ── Verdict ──
        print(f"\n  {BLD}ANALYSIS:{NC}")

        # Check if one motor is consistently slower
        l_slower_fwd = sum(1 for p in fwd if p.asym_pct < -1.0)
        l_slower_rev = sum(1 for p in rev if p.asym_pct < -1.0)
        r_slower_fwd = sum(1 for p in fwd if p.asym_pct > 1.0)
        r_slower_rev = sum(1 for p in rev if p.asym_pct > 1.0)

        always_same_side = False
        slow_motor = None

        if l_slower_fwd + l_slower_rev == len(results) and len(results) > 1:
            always_same_side = True
            slow_motor = "LEFT"
        elif r_slower_fwd + r_slower_rev == len(results) and len(results) > 1:
            always_same_side = True
            slow_motor = "RIGHT"

        # Direction-dependent asymmetry (different motor slower in fwd vs rev)
        direction_dep = ((avg_fwd_asym > 1.0 and avg_rev_asym < -1.0) or
                         (avg_fwd_asym < -1.0 and avg_rev_asym > 1.0))

        rev_only = (abs(avg_rev_asym) > 3.0 and abs(avg_fwd_asym) < 2.0)
        both_bad = (abs(avg_rev_asym) > 3.0 and abs(avg_fwd_asym) > 3.0)

        if both_bad and always_same_side:
            print(f"    {RED}⚠  {slow_motor} motor is consistently slower in BOTH directions.{NC}")
            print(f"    {RED}   → Likely a bad motor, worn gearbox, or loose tire on the {slow_motor.lower()} side.{NC}")
            print(f"    {RED}   → Replacing the {slow_motor.lower()} motor should fix this.{NC}")
        elif rev_only:
            # Which motor is slower in reverse?
            if avg_rev_asym < -2.0:
                culprit = "LEFT"
            elif avg_rev_asym > 2.0:
                culprit = "RIGHT"
            else:
                culprit = "UNKNOWN"
            print(f"    {YLW}⚠  Asymmetry is primarily in REVERSE — {culprit} motor is slower.{NC}")
            print(f"       This suggests:")
            print(f"       1. {culprit.lower()} motor has higher reverse friction/backlash")
            print(f"       2. Gearbox on {culprit.lower()} side may have different play in reverse")
            print(f"       3. Try swapping left and right motors to confirm:")
            print(f"          - If the slow side follows the motor → bad motor/gearbox, replace it")
            print(f"          - If the slow side stays on {culprit.lower()} → mechanical (tire, axle, frame)")
        elif direction_dep:
            print(f"    {YLW}ℹ  Asymmetry flips between forward and reverse.{NC}")
            print(f"       This is characteristic of backlash/gear play, not a defective motor.")
            print(f"       Motor replacement is unlikely to help. Try:")
            print(f"       1. Velocity-level feedforward trim (auto-tune Phase 4)")
            print(f"       2. Reduce gear play if accessible")
        elif always_same_side:
            print(f"    {RED}⚠  {slow_motor} motor is consistently slower.{NC}")
            print(f"    {RED}   → Swapping or replacing the {slow_motor.lower()} motor should help.{NC}")
        elif abs(avg_rev_asym) < 2.0 and abs(avg_fwd_asym) < 2.0:
            print(f"    {GRN}✓  Motor asymmetry is within normal range (<2%).{NC}")
            print(f"       Heading drift is likely due to mechanical alignment or tire wear,")
            print(f"       not individual motor performance. Software compensation should suffice.")
        else:
            print(f"    {YLW}ℹ  Moderate asymmetry detected. Try motor swap test to isolate cause.{NC}")

        # Startup analysis
        fwd_lags = [p.startup_lag_ms for p in fwd]
        rev_lags = [p.startup_lag_ms for p in rev]
        avg_fwd_lag = sum(fwd_lags) / len(fwd_lags) if fwd_lags else 0
        avg_rev_lag = sum(rev_lags) / len(rev_lags) if rev_lags else 0

        if abs(avg_fwd_lag) > 20 or abs(avg_rev_lag) > 20:
            print(f"\n    {YLW}⚠  Startup lag detected:{NC}")
            if abs(avg_fwd_lag) > 20:
                lagging = "LEFT" if avg_fwd_lag > 0 else "RIGHT"
                print(f"       Forward: {lagging} motor starts {abs(avg_fwd_lag):.0f} ms later")
            if abs(avg_rev_lag) > 20:
                lagging = "LEFT" if avg_rev_lag > 0 else "RIGHT"
                print(f"       Reverse: {lagging} motor starts {abs(avg_rev_lag):.0f} ms later")
            print(f"       This causes heading transients at the start of each motion.")
            print(f"       → Different static friction / min-duty thresholds between motors.")
            print(f"       → A motor swap test will reveal if this follows the motor.")

        # Raw motor data for swap test reference
        print(f"\n  {BLD}MOTOR SWAP TEST PROCEDURE:{NC}")
        print(f"    1. Physically swap left and right motors (swap the motor connectors)")
        print(f"    2. Run this script again:  ./diagnose_motors.py --stop-bringup")
        print(f"    3. Compare: if the slow/laggy side moved from L→R (or R→L),")
        print(f"       the problem follows the motor → replace that motor")
        print(f"    4. If the slow side stays on the same side despite swapping,")
        print(f"       the problem is frame/wheel/tire → not a motor issue")


def save_csv(results: List[PassResult], path: str):
    with open(path, "w") as f:
        f.write("pass,direction,speed_cmd_mms,t_s,vel_l_mms,vel_r_mms,enc_l,enc_r,gyro_z_dps\n")
        for pi, pr in enumerate(results):
            for s in pr.samples:
                f.write(f"{pi},{pr.direction},{pr.speed_cmd*1000:.0f},"
                        f"{s.t:.4f},{s.vel_l*1000:.2f},{s.vel_r*1000:.2f},"
                        f"{s.enc_l},{s.enc_r},{math.degrees(s.gyro_z):.2f}\n")
    print(f"\n  CSV saved: {path}  ({sum(len(p.samples) for p in results)} rows)")


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Raw motor asymmetry diagnostic")
    parser.add_argument("--speeds", nargs="+", type=float, default=[0.05, 0.08, 0.10, 0.13],
                        help="Speeds to test (m/s), each tested fwd and rev")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Duration per pass in seconds")
    parser.add_argument("--settle", type=float, default=1.5,
                        help="Settle time between passes")
    parser.add_argument("--csv", type=str, default=None,
                        help="Save raw samples to CSV file")
    parser.add_argument("--stop-bringup", action="store_true",
                        help="Stop turtlebot3-bringup.service before testing")
    parser.add_argument("--port", type=str, default=None,
                        help="Override serial port")
    parser.add_argument("--repeats", type=int, default=1,
                        help="Number of repeats per speed/direction")
    args = parser.parse_args()

    print(f"\n{'='*60}")
    print(f" {BLD}diagnose_motors.py — Raw Motor Asymmetry Diagnostic{NC}")
    print(f" speeds={args.speeds}  duration={args.duration}s  repeats={args.repeats}")
    print(f"{'='*60}\n")

    # ── Stop bringup ──
    bringup_was_running = False
    if args.stop_bringup:
        print("  Stopping turtlebot3-bringup.service …")
        try:
            rc = subprocess.run(["systemctl", "is-active", "--quiet",
                                "turtlebot3-bringup.service"]).returncode
            if rc == 0:
                bringup_was_running = True
            subprocess.run(["sudo", "systemctl", "stop",
                           "turtlebot3-bringup.service"],
                          timeout=10, capture_output=True)
            time.sleep(1.0)
        except Exception as e:
            print(f"  {YLW}Warning: could not stop bringup: {e}{NC}")

    # ── Open serial ──
    import serial as _serial
    port = args.port
    ser = None
    if port:
        ports_to_try = [port]
    else:
        ports_to_try = DXL_PORTS
    for p in ports_to_try:
        try:
            ser = _serial.Serial(p, DXL_BAUD, timeout=0.05)
            port = p
            break
        except Exception:
            continue
    if ser is None:
        print(f"  {RED}ERROR: Could not open any serial port{NC}")
        sys.exit(1)
    print(f"  Serial port: {port}  ({DXL_BAUD} baud)")

    # Probe firmware
    print("  Waiting for firmware response … ", end="", flush=True)
    for _ in range(20):
        regs = _read_regs(ser)
        if regs:
            break
        time.sleep(0.1)
    if regs is None:
        print(f"{RED}FAILED{NC}")
        sys.exit(1)
    print(f"OK  heading_hold_en={regs['hdg_en']}")

    # ── DISABLE heading-hold and zero all feedforward ──
    print(f"  {YLW}Disabling heading-hold and zeroing feedforward …{NC}")
    _write_u8(ser, REG_HEADING_HOLD_EN, 0)
    _write_f32(ser, REG_VEL_TRIM_FWD, 0.0)
    _write_f32(ser, REG_VEL_TRIM_REV, 0.0)
    _write_f32(ser, REG_HDG_I_SEED_FWD, 0.0)
    _write_f32(ser, REG_HDG_I_SEED_REV, 0.0)
    time.sleep(0.1)

    # Verify
    regs = _read_regs(ser)
    if regs and regs["hdg_en"] != 0:
        print(f"  {RED}WARNING: heading-hold still enabled!{NC}")
    else:
        print(f"  {GRN}Heading-hold: DISABLED  (raw motor behavior){NC}")

    print(f"\n  {CYN}Running motor tests — robot will move!{NC}")
    print(f"  Each speed tested forward then reverse, {args.duration:.0f}s per pass.")

    results: List[PassResult] = []

    for speed in args.speeds:
        for rep in range(args.repeats):
            rep_str = f" (rep {rep+1}/{args.repeats})" if args.repeats > 1 else ""

            # Forward
            print(f"\n  ── Forward @ {speed*1000:.0f} mm/s{rep_str} ──")
            sys.stdout.write("    Driving … ")
            sys.stdout.flush()
            pr_fwd = run_pass(ser, +speed, args.duration)
            print(f"done ({len(pr_fwd.samples)} samples)")
            print_pass(pr_fwd)
            results.append(pr_fwd)

            # Settle
            time.sleep(args.settle)

            # Reverse
            print(f"\n  ── Reverse @ {speed*1000:.0f} mm/s{rep_str} ──")
            sys.stdout.write("    Driving … ")
            sys.stdout.flush()
            pr_rev = run_pass(ser, -speed, args.duration)
            print(f"done ({len(pr_rev.samples)} samples)")
            print_pass(pr_rev)
            results.append(pr_rev)

            # Settle
            time.sleep(args.settle)

    # ── Re-enable heading-hold ──
    print(f"\n  Re-enabling heading-hold …")
    _write_u8(ser, REG_HEADING_HOLD_EN, 1)

    # ── Summary ──
    print_comparison_table(results)

    # ── CSV ──
    if args.csv:
        save_csv(results, args.csv)
    else:
        ts = time.strftime("%Y%m%d_%H%M%S")
        default_csv = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    f"motor_diag_{ts}.csv")
        save_csv(results, default_csv)

    ser.close()

    # Restart bringup if it was running
    if bringup_was_running:
        print(f"\n  Restarting turtlebot3-bringup.service …")
        try:
            subprocess.run(["sudo", "systemctl", "start",
                           "turtlebot3-bringup.service"],
                          timeout=10, capture_output=True)
            print("  turtlebot3-bringup.service restarted.")
        except Exception as e:
            print(f"  {YLW}Warning: could not restart bringup: {e}{NC}")

    print()


if __name__ == "__main__":
    main()
