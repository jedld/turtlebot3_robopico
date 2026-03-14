#!/usr/bin/env python3
"""
collect_forward_drift.py — Long-run forward drift collector and troubleshooter.

Runs one or more straight-line forward passes through the Pico firmware's
Dynamixel interface, records heading-hold telemetry, saves CSV, and prints a
root-cause summary.

Default behaviour compares:
  1. current firmware settings
  2. encoder-only heading hold (IMU fusion forced off)

This is useful for separating:
  • IMU-induced drift (especially BNO055 vibration bias)
  • residual left/right motor asymmetry
  • heading-hold saturation

The script needs exclusive serial access, so it can stop/restart
`turtlebot3-bringup.service` automatically.
"""
from __future__ import annotations

import argparse
import csv
import math
import struct
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Optional

import serial

# ── serial / Dynamixel constants ────────────────────────────────────────────
DXL_PORT_CANDIDATES = ("/dev/ttyTB3", "/dev/ttyACM0", "/dev/ttyACM1")
DXL_BAUD = 1_000_000
DXL_ID = 200
INST_READ = 0x02
INST_WRITE = 0x03

# ── firmware register addresses (must match firmware/main.c) ───────────────
REG_MILLIS = 10
REG_IMU_ANG_VEL_Z = 68
REG_MOTOR_TORQUE = 149
REG_CMD_LINEAR_X = 150
REG_CMD_ANGULAR_Z = 170
REG_DBG_IMU_SOURCE = 174
REG_ENC_L_COUNT = 184
REG_ENC_R_COUNT = 188
REG_ENC_RESET = 192
REG_DBG_VEL_L = 212
REG_DBG_VEL_R = 216
REG_HDG_I_SEED_FWD = 256
REG_HDG_ITERM = 264
REG_VEL_TRIM_FWD = 268
REG_HEADING_HOLD_KP = 280
REG_DBG_HEADING_ERR = 284
REG_HEADING_HOLD_EN = 288
REG_HEADING_HOLD_KI = 292
REG_HEADING_CORR = 296
REG_ENC_TRIM_KP = 300
REG_DBG_ENC_TRIM = 304
REG_DBG_ENC_DIFF = 308
REG_HEADING_HOLD_KD = 312
REG_IMU_HEADING_ALPHA = 316
REG_IMU_HEADING_EN = 320
REG_DBG_HEADING_FUSED = 324
REG_DBG_HEADING_GYRO = 328
REG_DBG_HEADING_ENC = 332
REG_IMU_HEADING_BIAS_BETA = 336
REG_DBG_GYRO_BIAS = 340

BULK_ADDR = REG_IMU_ANG_VEL_Z
BULK_LEN = (REG_DBG_GYRO_BIAS + 4) - BULK_ADDR

WHEEL_RADIUS_M = 0.03405
WHEEL_BASE_M = 0.121642
ENC_COUNTS_PER_REV = 1936.0
ENC_M_PER_COUNT = (2.0 * math.pi * WHEEL_RADIUS_M) / ENC_COUNTS_PER_REV

BRINGUP_SERVICE = "turtlebot3-bringup.service"
IMU_SOURCE_NAMES = {0: "sim", 1: "BNO085", 2: "BNO055", 0xFF: "unknown"}

RED = "\033[0;31m"
GRN = "\033[0;32m"
YLW = "\033[1;33m"
CYN = "\033[0;36m"
BLD = "\033[1m"
NC = "\033[0m"


@dataclass
class Sample:
    mode: str
    ts: float
    imu_source: int
    fusion_en: int
    alpha: float
    gyro_z_rads: float
    gyro_bias_rads: float
    heading_err_rad: float
    heading_fused_rad: float
    heading_enc_rad: float
    heading_gyro_rad: float
    heading_corr_rads: float
    heading_iterm: float
    vel_trim_fwd: float
    enc_trim_rads: float
    enc_diff_m: float
    vel_l_ms: float
    vel_r_ms: float
    enc_l: int
    enc_r: int
    hh_en: int
    hh_kp: float
    hh_ki: float
    hh_kd: float
    hdg_i_seed_fwd: float
    imu_bias_beta: float


@dataclass
class Summary:
    mode: str
    imu_source: str
    fusion_en: int
    alpha: float
    distance_l_m: float
    distance_r_m: float
    distance_avg_m: float
    mean_vel_l_ms: float
    mean_vel_r_ms: float
    asym_pct: float
    enc_drift_deg: float
    gyro_drift_deg: float
    mean_gyro_bias_dps: float
    mean_heading_corr_rads: float
    peak_heading_corr_rads: float
    peak_heading_err_deg: float


# ── CRC / packet helpers ────────────────────────────────────────────────────
_CRC_TABLE: list[int] = []
for _i in range(256):
    crc = _i << 8
    for _ in range(8):
        if crc & 0x8000:
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF
        else:
            crc = (crc << 1) & 0xFFFF
    _CRC_TABLE.append(crc)


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


def make_read_pkt(addr: int, length: int) -> bytes:
    params = struct.pack("<HH", addr, length)
    plen = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, DXL_ID,
                 plen & 0xFF, (plen >> 8) & 0xFF, INST_READ]) + params
    crc = crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])


def make_write_pkt(addr: int, payload: bytes) -> bytes:
    params = struct.pack("<H", addr) + payload
    plen = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, DXL_ID,
                 plen & 0xFF, (plen >> 8) & 0xFF, INST_WRITE]) + params
    crc = crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])


def read_response(ser: serial.Serial, expected: int, timeout: float = 0.25) -> Optional[bytes]:
    header = b"\xFF\xFF\xFD\x00"
    t0 = time.monotonic()
    buf = b""
    while time.monotonic() - t0 < timeout:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
        while True:
            idx = buf.find(header)
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


def write_u8(ser: serial.Serial, addr: int, value: int) -> None:
    ser.write(make_write_pkt(addr, bytes([value & 0xFF])))
    read_response(ser, expected=0, timeout=0.05)


def write_i32(ser: serial.Serial, addr: int, value: int) -> None:
    ser.write(make_write_pkt(addr, struct.pack("<i", value)))
    read_response(ser, expected=0, timeout=0.05)


def write_f32(ser: serial.Serial, addr: int, value: float) -> None:
    ser.write(make_write_pkt(addr, struct.pack("<f", value)))
    read_response(ser, expected=0, timeout=0.05)


def read_u8(ser: serial.Serial, addr: int) -> int:
    ser.write(make_read_pkt(addr, 1))
    data = read_response(ser, expected=1)
    if not data:
        raise RuntimeError(f"read_u8 failed at {addr}")
    return data[0]


def read_f32(ser: serial.Serial, addr: int) -> float:
    ser.write(make_read_pkt(addr, 4))
    data = read_response(ser, expected=4)
    if not data or len(data) < 4:
        raise RuntimeError(f"read_f32 failed at {addr}")
    return struct.unpack_from("<f", data, 0)[0]


def read_i32(ser: serial.Serial, addr: int) -> int:
    ser.write(make_read_pkt(addr, 4))
    data = read_response(ser, expected=4)
    if not data or len(data) < 4:
        raise RuntimeError(f"read_i32 failed at {addr}")
    return struct.unpack_from("<i", data, 0)[0]


def bulk_read(ser: serial.Serial) -> Optional[Sample]:
    ser.write(make_read_pkt(BULK_ADDR, BULK_LEN))
    data = read_response(ser, expected=BULK_LEN, timeout=0.15)
    if data is None or len(data) < BULK_LEN:
        return None

    def f32(addr: int) -> float:
        return struct.unpack_from("<f", data, addr - BULK_ADDR)[0]

    def i32(addr: int) -> int:
        return struct.unpack_from("<i", data, addr - BULK_ADDR)[0]

    def u8(addr: int) -> int:
        return data[addr - BULK_ADDR]

    return Sample(
        mode="",
        ts=0.0,
        imu_source=u8(REG_DBG_IMU_SOURCE),
        fusion_en=u8(REG_IMU_HEADING_EN),
        alpha=f32(REG_IMU_HEADING_ALPHA),
        gyro_z_rads=f32(REG_IMU_ANG_VEL_Z),
        gyro_bias_rads=f32(REG_DBG_GYRO_BIAS),
        heading_err_rad=f32(REG_DBG_HEADING_ERR),
        heading_fused_rad=f32(REG_DBG_HEADING_FUSED),
        heading_enc_rad=f32(REG_DBG_HEADING_ENC),
        heading_gyro_rad=f32(REG_DBG_HEADING_GYRO),
        heading_corr_rads=f32(REG_HEADING_CORR),
        heading_iterm=f32(REG_HDG_ITERM),
        vel_trim_fwd=f32(REG_VEL_TRIM_FWD),
        enc_trim_rads=f32(REG_DBG_ENC_TRIM),
        enc_diff_m=f32(REG_DBG_ENC_DIFF),
        vel_l_ms=f32(REG_DBG_VEL_L),
        vel_r_ms=f32(REG_DBG_VEL_R),
        enc_l=i32(REG_ENC_L_COUNT),
        enc_r=i32(REG_ENC_R_COUNT),
        hh_en=u8(REG_HEADING_HOLD_EN),
        hh_kp=f32(REG_HEADING_HOLD_KP),
        hh_ki=f32(REG_HEADING_HOLD_KI),
        hh_kd=f32(REG_HEADING_HOLD_KD),
        hdg_i_seed_fwd=f32(REG_HDG_I_SEED_FWD),
        imu_bias_beta=f32(REG_IMU_HEADING_BIAS_BETA),
    )


# ── service helpers ─────────────────────────────────────────────────────────
def bringup_is_active() -> bool:
    try:
        proc = subprocess.run(
            ["systemctl", "is-active", BRINGUP_SERVICE],
            capture_output=True, text=True, check=False,
        )
        return proc.stdout.strip() == "active"
    except Exception:
        return False


def stop_bringup() -> bool:
    subprocess.run(["sudo", "systemctl", "stop", BRINGUP_SERVICE], check=False)
    time.sleep(1.5)
    return not bringup_is_active()


def start_bringup() -> None:
    subprocess.run(["sudo", "systemctl", "start", BRINGUP_SERVICE], check=False)
    time.sleep(2.0)


# ── higher-level helpers ────────────────────────────────────────────────────
def detect_port(explicit: Optional[str]) -> str:
    if explicit:
        return explicit
    for port in DXL_PORT_CANDIDATES:
        if Path(port).exists():
            return port
    raise FileNotFoundError("No TurtleBot3 serial port found")


def send_cmd_vel(ser: serial.Serial, lin_x: float, ang_z: float = 0.0) -> None:
    write_i32(ser, REG_CMD_LINEAR_X, int(round(lin_x * 100.0)))
    write_i32(ser, REG_CMD_ANGULAR_Z, int(round(ang_z * 100.0)))


def wait_for_device(ser: serial.Serial, timeout_s: float = 5.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            _ = read_i32(ser, REG_MILLIS)
            return
        except Exception:
            time.sleep(0.2)
    raise RuntimeError("Pico did not respond to REG_MILLIS")


def run_pass(
    ser: serial.Serial,
    mode: str,
    speed: float,
    duration: float,
    sample_period: float,
    force_fusion_en: Optional[int],
) -> list[Sample]:
    samples: list[Sample] = []

    if force_fusion_en is not None:
        write_u8(ser, REG_IMU_HEADING_EN, int(force_fusion_en))
        time.sleep(0.1)

    write_u8(ser, REG_ENC_RESET, 1)
    write_u8(ser, REG_MOTOR_TORQUE, 1)
    send_cmd_vel(ser, 0.0, 0.0)
    time.sleep(0.2)

    t0 = time.monotonic()
    next_sample = t0
    while time.monotonic() - t0 < duration:
        send_cmd_vel(ser, speed, 0.0)
        now = time.monotonic()
        if now >= next_sample:
            sample = bulk_read(ser)
            if sample is not None:
                sample.mode = mode
                sample.ts = now - t0
                samples.append(sample)
            next_sample = now + sample_period
        time.sleep(0.01)

    send_cmd_vel(ser, 0.0, 0.0)
    time.sleep(0.3)
    tail = bulk_read(ser)
    if tail is not None:
        tail.mode = mode
        tail.ts = time.monotonic() - t0
        samples.append(tail)
    return samples


def analyse(samples: list[Sample]) -> Summary:
    if len(samples) < 3:
        raise RuntimeError("Not enough samples collected")

    moving = [s for s in samples if abs(s.vel_l_ms) > 0.02 or abs(s.vel_r_ms) > 0.02]
    if len(moving) < 3:
        raise RuntimeError("Not enough moving samples collected")

    half = moving[len(moving) // 2 :]
    mean_vel_l = sum(s.vel_l_ms for s in half) / len(half)
    mean_vel_r = sum(s.vel_r_ms for s in half) / len(half)
    abs_l = abs(mean_vel_l)
    abs_r = abs(mean_vel_r)
    asym_pct = ((abs_l - abs_r) / max(abs_l, abs_r, 1e-9)) * 100.0

    start = moving[0]
    end = moving[-1]
    enc_l0, enc_r0 = start.enc_l, start.enc_r
    enc_l1, enc_r1 = end.enc_l, end.enc_r
    dist_l = (enc_l1 - enc_l0) * ENC_M_PER_COUNT
    dist_r = (enc_r1 - enc_r0) * ENC_M_PER_COUNT
    dist_avg = 0.5 * (abs(dist_l) + abs(dist_r))

    enc_drift_deg = math.degrees((dist_l - dist_r) / WHEEL_BASE_M)

    gyro_drift = 0.0
    for prev, cur in zip(moving, moving[1:]):
        gyro_drift += cur.gyro_z_rads * max(cur.ts - prev.ts, 0.0)

    mean_bias = sum(s.gyro_bias_rads for s in half) / len(half)
    mean_corr = sum(s.heading_corr_rads for s in half) / len(half)
    peak_corr = max(abs(s.heading_corr_rads) for s in moving)
    peak_err_deg = max(abs(math.degrees(s.heading_err_rad)) for s in moving)
    imu_source = IMU_SOURCE_NAMES.get(end.imu_source, str(end.imu_source))

    return Summary(
        mode=end.mode,
        imu_source=imu_source,
        fusion_en=end.fusion_en,
        alpha=end.alpha,
        distance_l_m=dist_l,
        distance_r_m=dist_r,
        distance_avg_m=dist_avg,
        mean_vel_l_ms=mean_vel_l,
        mean_vel_r_ms=mean_vel_r,
        asym_pct=asym_pct,
        enc_drift_deg=enc_drift_deg,
        gyro_drift_deg=math.degrees(gyro_drift),
        mean_gyro_bias_dps=math.degrees(mean_bias),
        mean_heading_corr_rads=mean_corr,
        peak_heading_corr_rads=peak_corr,
        peak_heading_err_deg=peak_err_deg,
    )


def write_csv(path: Path, samples: Iterable[Sample]) -> None:
    rows = [asdict(s) for s in samples]
    if not rows:
        return
    with path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def print_summary(summary: Summary) -> None:
    drift_col = GRN if abs(summary.enc_drift_deg) < 1.0 else YLW if abs(summary.enc_drift_deg) < 3.0 else RED
    asym_col = GRN if abs(summary.asym_pct) < 2.0 else YLW if abs(summary.asym_pct) < 4.0 else RED
    bias_col = GRN if abs(summary.mean_gyro_bias_dps) < 0.5 else YLW if abs(summary.mean_gyro_bias_dps) < 1.5 else RED
    print(f"\n{BLD}[ {summary.mode} ]{NC}")
    print(f"  IMU source:              {summary.imu_source}  (fusion_en={summary.fusion_en}, alpha={summary.alpha:.2f})")
    print(f"  Distance:                L={summary.distance_l_m:.3f} m  R={summary.distance_r_m:.3f} m  avg={summary.distance_avg_m:.3f} m")
    print(f"  Steady velocity:         L={summary.mean_vel_l_ms*1000:+6.1f} mm/s  R={summary.mean_vel_r_ms*1000:+6.1f} mm/s")
    print(f"  Velocity asymmetry:      {asym_col}{summary.asym_pct:+.2f}%{NC}")
    print(f"  Encoder heading drift:   {drift_col}{summary.enc_drift_deg:+.2f}°{NC}")
    print(f"  Integrated gyro drift:   {summary.gyro_drift_deg:+.2f}°")
    print(f"  Mean gyro bias:          {bias_col}{summary.mean_gyro_bias_dps:+.2f}°/s{NC}")
    print(f"  Mean heading correction: {summary.mean_heading_corr_rads:+.3f} rad/s")
    print(f"  Peak heading correction: {summary.peak_heading_corr_rads:+.3f} rad/s")
    print(f"  Peak heading error:      {summary.peak_heading_err_deg:.2f}°")


def print_diagnosis(summaries: list[Summary]) -> None:
    by_mode = {s.mode: s for s in summaries}
    current = by_mode.get("current")
    enc_only = by_mode.get("encoder_only")

    print(f"\n{BLD}Diagnosis{NC}")
    notes: list[str] = []

    if current and current.imu_source == "BNO055" and abs(current.mean_gyro_bias_dps) > 1.0:
        notes.append("BNO055 gyro bias is significant under drive. Long straight runs will drift if that gyro is fused into heading control or odometry.")

    if current and enc_only:
        cur = abs(current.enc_drift_deg)
        enc = abs(enc_only.enc_drift_deg)
        if current.imu_source == "BNO055" and enc < cur * 0.6:
            notes.append("Encoder-only mode is materially straighter than the current mode. IMU fusion is the primary drift source.")
        elif abs(enc - cur) < 0.5:
            notes.append("Encoder-only mode is similar to the current mode. Residual drift is mostly mechanical/asymmetry, not IMU fusion.")

    if any(abs(s.enc_drift_deg) > 4.0 for s in summaries):
        notes.append("Encoder drift is large even though steady-state wheel speeds are close. Most of the error is a startup transient; preload heading-hold feedforward or integral seed.")

    worst_asym = max((abs(s.asym_pct) for s in summaries), default=0.0)
    if worst_asym > 4.0:
        notes.append("Left/right wheel velocity asymmetry is still high. Re-run drift calibration and re-check motor dead-zone tuning.")

    worst_corr = max((abs(s.peak_heading_corr_rads) for s in summaries), default=0.0)
    worst_drift = max((abs(s.enc_drift_deg) for s in summaries), default=0.0)
    if worst_corr > 0.24 and worst_drift > 2.0:
        notes.append("Heading correction is nearing saturation while drift remains high. The controller is fighting a large bias source rather than a small trim error.")

    if not notes:
        notes.append("No single dominant fault stood out. Use the CSV to inspect whether drift grows with gyro bias, wheel asymmetry, or correction saturation.")

    for idx, note in enumerate(notes, start=1):
        print(f"  {idx}. {note}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Collect long-run forward drift telemetry")
    ap.add_argument("--port", help="serial port (default: auto-detect)")
    ap.add_argument("--speed", type=float, default=0.10, help="forward speed in m/s")
    ap.add_argument("--duration", type=float, default=8.0, help="pass duration in seconds")
    ap.add_argument("--sample-period", type=float, default=0.10, help="sample interval in seconds")
    ap.add_argument("--modes", nargs="+", choices=["current", "encoder_only"],
                    default=["current", "encoder_only"], help="passes to run")
    ap.add_argument("--csv", help="output CSV path")
    ap.add_argument("--no-stop-bringup", action="store_true", help="do not stop bringup automatically")
    args = ap.parse_args()

    port = detect_port(args.port)
    csv_path = Path(args.csv) if args.csv else Path(__file__).resolve().parent / (
        f"forward_drift_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )

    stopped_bringup = False
    if bringup_is_active() and not args.no_stop_bringup:
        print(f"{CYN}Stopping {BRINGUP_SERVICE} for exclusive serial access...{NC}")
        if not stop_bringup():
            print(f"{RED}Failed to stop {BRINGUP_SERVICE}{NC}")
            return 1
        stopped_bringup = True

    try:
        with serial.Serial(port, DXL_BAUD, timeout=0.05) as ser:
            time.sleep(0.2)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            wait_for_device(ser)

            orig_fusion_en = read_u8(ser, REG_IMU_HEADING_EN)
            orig_alpha = read_f32(ser, REG_IMU_HEADING_ALPHA)
            orig_hh_en = read_u8(ser, REG_HEADING_HOLD_EN)

            print(f"{BLD}Forward drift collection{NC}")
            print(f"  Port:       {port}")
            print(f"  Speed:      {args.speed:.3f} m/s")
            print(f"  Duration:   {args.duration:.1f} s per pass")
            print(f"  CSV:        {csv_path}")
            print(f"  HH enable:  {orig_hh_en}")
            print(f"  IMU fusion: {orig_fusion_en}  alpha={orig_alpha:.2f}")

            all_samples: list[Sample] = []
            summaries: list[Summary] = []

            for mode in args.modes:
                force_fusion = None if mode == "current" else 0
                label = "current" if mode == "current" else "encoder_only"
                print(f"\n{CYN}Running {label} pass...{NC}")
                samples = run_pass(
                    ser,
                    mode=label,
                    speed=args.speed,
                    duration=args.duration,
                    sample_period=args.sample_period,
                    force_fusion_en=force_fusion,
                )
                summary = analyse(samples)
                all_samples.extend(samples)
                summaries.append(summary)
                print_summary(summary)
                time.sleep(0.5)

            write_csv(csv_path, all_samples)
            print_diagnosis(summaries)
            print(f"\n{GRN}Saved CSV to {csv_path}{NC}")

            send_cmd_vel(ser, 0.0, 0.0)
            write_u8(ser, REG_IMU_HEADING_EN, orig_fusion_en)
            write_f32(ser, REG_IMU_HEADING_ALPHA, orig_alpha)
            return 0
    finally:
        if stopped_bringup:
            print(f"\n{CYN}Restarting {BRINGUP_SERVICE}...{NC}")
            start_bringup()


if __name__ == "__main__":
    raise SystemExit(main())
