#!/usr/bin/env python3
"""
Rotation wobble diagnostic — records IMU gyro, encoder velocities, PID output,
motor commands, and heading-hold state during a controlled pivot turn.

Outputs a timestamped CSV + summary stats to help identify the cause of
uneven rotation (PID oscillation, encoder noise, IMU drift, I2C errors, etc.).

Usage:
    sudo systemctl stop turtlebot3-bringup.service
    python3 diagnose_rotation.py          # default: 1.0 rad/s CW, 4 s
    python3 diagnose_rotation.py --speed -2.0 --duration 6
"""

import argparse
import csv
import math
import os
import struct
import sys
import time
from pathlib import Path

import serial
from serial import SerialException

# ── Dynamixel constants ─────────────────────────────────────────────────────
PORTS = ("/dev/ttyTB3", "/dev/ttyACM0", "/dev/ttyACM1")
DEV_ID = 200
BAUD = 1_000_000

# Register addresses
ADDR_MILLIS            = 10
ADDR_IMU_ANG_VEL_Z     = 68
ADDR_IMU_LIN_ACC_X     = 72
ADDR_IMU_LIN_ACC_Y     = 76
ADDR_PRESENT_VEL_L     = 128
ADDR_PRESENT_VEL_R     = 132
ADDR_MOTOR_TORQUE_EN   = 149
ADDR_CMD_LINEAR_X      = 150
ADDR_CMD_ANGULAR_Z     = 170
ADDR_PID_KP            = 200
ADDR_PID_KI            = 204
ADDR_DBG_VEL_L         = 212   # filtered encoder vel L (m/s)
ADDR_DBG_VEL_R         = 216   # filtered encoder vel R (m/s)
ADDR_VEL_RAMP          = 220
ADDR_HEADING_HOLD_KP   = 280
ADDR_HEADING_HOLD_EN   = 288
ADDR_HEADING_HOLD_KI   = 292
ADDR_HEADING_HOLD_CORR = 296
ADDR_DBG_ENC_TRIM      = 304
ADDR_DBG_ENC_DIFF      = 308
ADDR_HEADING_HOLD_KD   = 312
ADDR_DBG_HEADING_FUSED = 324
ADDR_DBG_HEADING_GYRO  = 328
ADDR_DBG_HEADING_ENC   = 332
ADDR_DBG_GYRO_BIAS     = 340
ADDR_MOTOR_I2C_ERR_CNT = 352
ADDR_MOTOR_LAST_CMD_M1 = 356
ADDR_MOTOR_LAST_CMD_M2 = 357

# ── CRC ──────────────────────────────────────────────────────────────────────
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


def _read_resp(ser, timeout=0.15):
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


def read_f32(ser, addr):
    raw = dxl_read(ser, addr, 4)
    if raw and len(raw) >= 4:
        v = struct.unpack_from("<f", raw)[0]
        if math.isfinite(v) and abs(v) < 1e6:
            return v
    return None


def read_i32(ser, addr):
    raw = dxl_read(ser, addr, 4)
    if raw and len(raw) >= 4:
        return struct.unpack_from("<i", raw)[0]
    return None


def read_u32(ser, addr):
    raw = dxl_read(ser, addr, 4)
    if raw and len(raw) >= 4:
        return struct.unpack_from("<I", raw)[0]
    return None


def read_i8(ser, addr):
    raw = dxl_read(ser, addr, 1)
    if raw and len(raw) >= 1:
        return struct.unpack_from("<b", raw)[0]
    return None


def detect_port(explicit):
    if explicit:
        return explicit
    for p in PORTS:
        if os.path.exists(p):
            return p
    return PORTS[0]


# ── Sampling ─────────────────────────────────────────────────────────────────

def sample_all(ser):
    """Read a comprehensive snapshot of sensor + control state."""
    row = {}

    # Bulk read: encoder velocities (filtered) + IMU gyro Z  in one shot
    # Read a block from ADDR_IMU_ANG_VEL_Z(68) through ADDR_IMU_LIN_ACC_Y(76)+4 = 12 bytes
    imu_blk = dxl_read(ser, ADDR_IMU_ANG_VEL_Z, 16)
    if imu_blk and len(imu_blk) >= 16:
        row["gyro_z"] = struct.unpack_from("<f", imu_blk, 0)[0]
        row["acc_x"]  = struct.unpack_from("<f", imu_blk, 4)[0]
        row["acc_y"]  = struct.unpack_from("<f", imu_blk, 8)[0]
        row["acc_z"]  = struct.unpack_from("<f", imu_blk, 12)[0]
    else:
        row["gyro_z"] = row["acc_x"] = row["acc_y"] = row["acc_z"] = None

    # Filtered encoder velocities
    vel_blk = dxl_read(ser, ADDR_DBG_VEL_L, 8)
    if vel_blk and len(vel_blk) >= 8:
        row["vel_l"] = struct.unpack_from("<f", vel_blk, 0)[0]
        row["vel_r"] = struct.unpack_from("<f", vel_blk, 4)[0]
    else:
        row["vel_l"] = row["vel_r"] = None

    # Motor last commands
    cmd_blk = dxl_read(ser, ADDR_MOTOR_LAST_CMD_M1, 2)
    if cmd_blk and len(cmd_blk) >= 2:
        row["cmd_m1"] = struct.unpack_from("<b", cmd_blk, 0)[0]
        row["cmd_m2"] = struct.unpack_from("<b", cmd_blk, 1)[0]
    else:
        row["cmd_m1"] = row["cmd_m2"] = None

    # Heading hold correction
    hh_corr = read_f32(ser, ADDR_HEADING_HOLD_CORR)
    row["hh_corr"] = hh_corr

    # Heading hold error
    hh_err_blk = dxl_read(ser, ADDR_DBG_HEADING_FUSED, 12)
    if hh_err_blk and len(hh_err_blk) >= 12:
        row["heading_fused"] = struct.unpack_from("<f", hh_err_blk, 0)[0]
        row["heading_gyro"]  = struct.unpack_from("<f", hh_err_blk, 4)[0]
        row["heading_enc"]   = struct.unpack_from("<f", hh_err_blk, 8)[0]
    else:
        row["heading_fused"] = row["heading_gyro"] = row["heading_enc"] = None

    # I2C error count
    row["i2c_err"] = read_u32(ser, ADDR_MOTOR_I2C_ERR_CNT)

    return row


FIELDS = [
    "t_s", "gyro_z", "acc_x", "acc_y", "acc_z",
    "vel_l", "vel_r", "cmd_m1", "cmd_m2",
    "hh_corr", "heading_fused", "heading_gyro", "heading_enc",
    "i2c_err",
]


def keep_alive(ser):
    dxl_read(ser, ADDR_MILLIS, 4)


def main():
    parser = argparse.ArgumentParser(description="Rotation wobble diagnostic")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="angular velocity in rad/s (positive=CCW, negative=CW; default 1.0)")
    parser.add_argument("--duration", type=float, default=4.0,
                        help="rotation duration in seconds (default 4.0)")
    parser.add_argument("--settle", type=float, default=1.0,
                        help="settle time before recording (default 1.0 s)")
    parser.add_argument("--port", default=None)
    parser.add_argument("--csv", default="rotation_diag.csv",
                        help="output CSV filename (default rotation_diag.csv)")
    args = parser.parse_args()

    port = detect_port(args.port)
    print(f"Port            : {port}")
    print(f"Angular speed   : {args.speed:.2f} rad/s")
    print(f"Duration        : {args.duration:.1f} s  (+ {args.settle:.1f} s settle)")
    print(f"Output CSV      : {args.csv}")
    print()

    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.3)

    # Ping
    millis = read_i32(ser, ADDR_MILLIS)
    if millis is None:
        print("ERROR: cannot communicate with Pico. Is bringup stopped?")
        ser.close()
        sys.exit(1)
    print(f"Pico uptime     : {millis} ms")

    # Read config
    pid_kp = read_f32(ser, ADDR_PID_KP)
    pid_ki = read_f32(ser, ADDR_PID_KI)
    vel_ramp = read_f32(ser, ADDR_VEL_RAMP)
    hh_en = dxl_read(ser, ADDR_HEADING_HOLD_EN, 1)
    hh_en_val = hh_en[0] if hh_en else None
    hh_kp = read_f32(ser, ADDR_HEADING_HOLD_KP)
    hh_ki = read_f32(ser, ADDR_HEADING_HOLD_KI)
    hh_kd = read_f32(ser, ADDR_HEADING_HOLD_KD)
    gyro_bias = read_f32(ser, ADDR_DBG_GYRO_BIAS)
    i2c_err_start = read_u32(ser, ADDR_MOTOR_I2C_ERR_CNT) or 0

    print(f"PID             : Kp={pid_kp}  Ki={pid_ki}")
    print(f"Vel ramp        : {vel_ramp} m/s²")
    print(f"Heading hold    : {'ON' if hh_en_val else 'OFF'}  Kp={hh_kp}  Ki={hh_ki}  Kd={hh_kd}")
    print(f"Gyro bias       : {gyro_bias}")
    print(f"I2C errors      : {i2c_err_start}")
    print()

    # Enable torque
    dxl_write(ser, ADDR_MOTOR_TORQUE_EN, bytes([1]))

    # ── Phase 1: Settle (spin up, don't record) ────────────────────────────
    print(f"Spinning up ({args.settle:.1f} s settle)...", flush=True)
    ang_raw = struct.pack("<i", int(round(args.speed * 100)))
    lin_raw = struct.pack("<i", 0)

    dxl_write(ser, ADDR_CMD_LINEAR_X, lin_raw)
    dxl_write(ser, ADDR_CMD_ANGULAR_Z, ang_raw)

    t_settle_end = time.time() + args.settle
    while time.time() < t_settle_end:
        keep_alive(ser)
        time.sleep(0.04)

    # ── Phase 2: Record ────────────────────────────────────────────────────
    print(f"Recording ({args.duration:.1f} s)...", flush=True)
    rows = []
    t0 = time.time()
    sample_count = 0
    read_errors = 0

    while True:
        elapsed = time.time() - t0
        if elapsed >= args.duration:
            break

        # Keep command alive
        dxl_write(ser, ADDR_CMD_LINEAR_X, lin_raw)
        dxl_write(ser, ADDR_CMD_ANGULAR_Z, ang_raw)

        row = sample_all(ser)
        row["t_s"] = round(elapsed, 4)
        rows.append(row)
        sample_count += 1

        if row["vel_l"] is None:
            read_errors += 1

    # ── Stop ───────────────────────────────────────────────────────────────
    print("Stopping motors...", flush=True)
    dxl_write(ser, ADDR_CMD_LINEAR_X, struct.pack("<i", 0))
    dxl_write(ser, ADDR_CMD_ANGULAR_Z, struct.pack("<i", 0))
    time.sleep(0.5)

    i2c_err_end = read_u32(ser, ADDR_MOTOR_I2C_ERR_CNT) or 0
    # Leave torque enabled so bringup node can drive motors after this script
    ser.close()

    # ── Write CSV ──────────────────────────────────────────────────────────
    csv_path = Path(args.csv)
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        for r in rows:
            writer.writerow({k: r.get(k, "") for k in FIELDS})
    print(f"\nWrote {sample_count} samples to {csv_path}")

    # ── Analysis ──────────────────────────────────────────────────────────
    print("\n" + "=" * 70)
    print("ROTATION DIAGNOSTIC SUMMARY")
    print("=" * 70)

    def stats(values, name):
        vals = [v for v in values if v is not None and math.isfinite(v)]
        if not vals:
            print(f"  {name:24s}: NO DATA")
            return None, None, None
        mn, mx = min(vals), max(vals)
        avg = sum(vals) / len(vals)
        std = (sum((v - avg) ** 2 for v in vals) / len(vals)) ** 0.5
        pk2pk = mx - mn
        print(f"  {name:24s}: avg={avg:+.4f}  std={std:.4f}  "
              f"min={mn:+.4f}  max={mx:+.4f}  pk2pk={pk2pk:.4f}")
        return avg, std, pk2pk

    # Gyro
    print("\nIMU gyro-Z (rad/s):")
    gyro_avg, gyro_std, gyro_pk2pk = stats([r["gyro_z"] for r in rows], "gyro_z")

    # Encoder velocities
    print("\nEncoder velocities (m/s):")
    vl_avg, vl_std, vl_pk2pk = stats([r["vel_l"] for r in rows], "vel_L (filtered)")
    vr_avg, vr_std, vr_pk2pk = stats([r["vel_r"] for r in rows], "vel_R (filtered)")

    # Velocity difference (should be ~constant for pure rotation)
    diffs = []
    for r in rows:
        if r["vel_l"] is not None and r["vel_r"] is not None:
            diffs.append(r["vel_l"] - r["vel_r"])
    print("\nVelocity L-R difference (m/s):")
    diff_avg, diff_std, diff_pk2pk = stats(diffs, "vel_L - vel_R")

    # Motor commands
    print("\nMotor raw commands (I2C int8):")
    stats([r["cmd_m1"] for r in rows], "cmd_M1 (left)")
    stats([r["cmd_m2"] for r in rows], "cmd_M2 (right)")

    # Command changes (oscillation detection)
    m1_vals = [r["cmd_m1"] for r in rows if r["cmd_m1"] is not None]
    m2_vals = [r["cmd_m2"] for r in rows if r["cmd_m2"] is not None]
    m1_changes = sum(1 for i in range(1, len(m1_vals)) if m1_vals[i] != m1_vals[i-1])
    m2_changes = sum(1 for i in range(1, len(m2_vals)) if m2_vals[i] != m2_vals[i-1])
    m1_sign_flips = sum(1 for i in range(1, len(m1_vals))
                        if (m1_vals[i] > 0) != (m1_vals[i-1] > 0) and m1_vals[i] != 0 and m1_vals[i-1] != 0)
    m2_sign_flips = sum(1 for i in range(1, len(m2_vals))
                        if (m2_vals[i] > 0) != (m2_vals[i-1] > 0) and m2_vals[i] != 0 and m2_vals[i-1] != 0)
    print(f"\n  M1 command changes: {m1_changes}/{len(m1_vals)}  sign flips: {m1_sign_flips}")
    print(f"  M2 command changes: {m2_changes}/{len(m2_vals)}  sign flips: {m2_sign_flips}")

    # Heading hold
    print("\nHeading-hold correction (rad/s):")
    stats([r["hh_corr"] for r in rows], "heading_hold_corr")

    print("\nHeading tracking:")
    stats([r["heading_fused"] for r in rows], "heading_fused (rad)")
    stats([r["heading_gyro"] for r in rows], "heading_gyro (rad)")
    stats([r["heading_enc"] for r in rows], "heading_enc (rad)")

    # Accelerometer (lateral wobble)
    print("\nAccelerometer (m/s²):")
    stats([r["acc_x"] for r in rows], "acc_x (forward)")
    stats([r["acc_y"] for r in rows], "acc_y (lateral)")

    # I2C errors
    new_errs = i2c_err_end - i2c_err_start
    print(f"\nI2C errors during test: {new_errs}  (total: {i2c_err_end})")
    print(f"Read errors (missed Dxl replies): {read_errors}/{sample_count}")

    # ── Diagnosis ──────────────────────────────────────────────────────────
    print("\n" + "-" * 70)
    print("DIAGNOSIS")
    print("-" * 70)
    issues = []

    if gyro_std and gyro_std > 0.3:
        issues.append(f"HIGH gyro-Z jitter (std={gyro_std:.3f} rad/s) — IMU noise or vibration")

    if vl_std and vl_std > 0.03:
        issues.append(f"HIGH left encoder variance (std={vl_std:.4f} m/s) — PID oscillation or encoder noise")
    if vr_std and vr_std > 0.03:
        issues.append(f"HIGH right encoder variance (std={vr_std:.4f} m/s) — PID oscillation or encoder noise")

    if diff_std and diff_std > 0.02:
        issues.append(f"HIGH L-R velocity jitter (std={diff_std:.4f}) — asymmetric PID response")

    if m1_sign_flips > 3:
        issues.append(f"M1 command sign flips: {m1_sign_flips} — PID hunting / dead-zone issue on left motor")
    if m2_sign_flips > 3:
        issues.append(f"M2 command sign flips: {m2_sign_flips} — PID hunting / dead-zone issue on right motor")

    if m1_changes > 0.8 * len(m1_vals):
        issues.append(f"M1 command unstable ({m1_changes}/{len(m1_vals)} changes) — PID output oscillating")
    if m2_changes > 0.8 * len(m2_vals):
        issues.append(f"M2 command unstable ({m2_changes}/{len(m2_vals)} changes) — PID output oscillating")

    hh_corrs = [r["hh_corr"] for r in rows if r["hh_corr"] is not None]
    if hh_corrs:
        hh_pk2pk = max(hh_corrs) - min(hh_corrs)
        if hh_pk2pk > 0.5:
            issues.append(f"Heading-hold correction swinging (pk2pk={hh_pk2pk:.3f} rad/s) — HH fighting the rotation command")

    if new_errs > 5:
        issues.append(f"I2C write errors: {new_errs} — wiring or bus contention")

    if not issues:
        print("  No obvious issues detected. Rotation looks clean.")
    else:
        for i, issue in enumerate(issues, 1):
            print(f"  {i}. {issue}")

    print()


if __name__ == "__main__":
    main()
