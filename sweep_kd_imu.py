#!/usr/bin/env python3
"""
sweep_kd_imu.py — Sweep heading-hold KD and IMU fusion parameters at runtime.

Writes KD, alpha, beta directly to firmware registers (no reflashing needed),
runs FWD+REV legs, and records encoder drift, IMU yaw drift, peak error.

Needs exclusive serial access (bringup must be stopped beforehand).
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass

import serial

from dxl_utils import *  # noqa: F401,F403


def quat_yaw_deg(ser) -> float:
    """Read BNO085 game rotation vector (mag-free) yaw."""
    qw = dxl_read_f32(ser, REG_GAME_ROT_W) or 1.0
    qx = dxl_read_f32(ser, REG_GAME_ROT_X) or 0.0
    qy = dxl_read_f32(ser, REG_GAME_ROT_Y) or 0.0
    qz = dxl_read_f32(ser, REG_GAME_ROT_Z) or 0.0
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def dxl_write_f32(ser, addr, val, dev_id=DEV_ID):
    dxl_write(ser, addr, struct.pack("<f", val), dev_id)


def set_params(ser, kd, alpha, beta, d_filter_hz=None):
    """Write KD, alpha, beta to firmware registers."""
    dxl_write_f32(ser, REG_HEADING_HOLD_KD, kd)
    dxl_write_f32(ser, REG_IMU_HEADING_ALPHA, alpha)
    dxl_write_f32(ser, REG_IMU_HEADING_BIAS_BETA, beta)
    # Verify
    time.sleep(0.02)
    kd_r = dxl_read_f32(ser, REG_HEADING_HOLD_KD)
    al_r = dxl_read_f32(ser, REG_IMU_HEADING_ALPHA)
    be_r = dxl_read_f32(ser, REG_IMU_HEADING_BIAS_BETA)
    return (kd_r if kd_r is not None else -1,
            al_r if al_r is not None else -1,
            be_r if be_r is not None else -1)


@dataclass
class LegResult:
    direction: str
    enc_drift_deg: float
    imu_yaw_deg: float
    mean_err_deg: float
    peak_err_deg: float
    gyro_bias_dps: float
    vel_asym_pct: float
    duration_s: float


def run_leg(ser, speed, dist_m) -> LegResult:
    direction = "FWD" if speed > 0 else "REV"
    timeout_s = (dist_m / abs(speed)) * 2.0 + 5.0

    dxl_write(ser, ADDR_TORQUE_EN, bytes([1]))
    time.sleep(0.05)

    enc_l0 = dxl_read_i32(ser, ADDR_ENC_L) or 0
    enc_r0 = dxl_read_i32(ser, ADDR_ENC_R) or 0
    imu_yaw0 = quat_yaw_deg(ser)

    set_velocity(ser, speed)
    time.sleep(0.1)

    samples = []
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        hd_err = dxl_read_f32(ser, ADDR_DBG_HEADING_ERR) or 0.0
        vel_l = dxl_read_f32(ser, ADDR_DBG_VEL_L) or 0.0
        vel_r = dxl_read_f32(ser, ADDR_DBG_VEL_R) or 0.0
        enc_l = dxl_read_i32(ser, ADDR_ENC_L) or enc_l0
        enc_r = dxl_read_i32(ser, ADDR_ENC_R) or enc_r0

        dl = abs(enc_l - enc_l0) * ENC_M_PER_COUNT
        dr = abs(enc_r - enc_r0) * ENC_M_PER_COUNT
        avg_dist = (dl + dr) / 2.0

        samples.append(dict(t=time.time()-t0, hd_err=hd_err, vel_l=vel_l, vel_r=vel_r))

        if avg_dist >= dist_m:
            break
        time.sleep(0.05)

    set_velocity(ser, 0.0)
    time.sleep(0.3)
    dxl_write(ser, ADDR_TORQUE_EN, bytes([0]))
    time.sleep(1.5)

    imu_yaw1 = quat_yaw_deg(ser)
    enc_l1 = dxl_read_i32(ser, ADDR_ENC_L) or enc_l0
    enc_r1 = dxl_read_i32(ser, ADDR_ENC_R) or enc_r0
    gyro_bias = dxl_read_f32(ser, REG_DBG_GYRO_BIAS) or 0.0

    dl_m = (enc_l1 - enc_l0) * ENC_M_PER_COUNT
    dr_m = (enc_r1 - enc_r0) * ENC_M_PER_COUNT
    enc_drift = math.degrees((dl_m - dr_m) / 0.185)

    imu_delta = imu_yaw1 - imu_yaw0
    if imu_delta > 180: imu_delta -= 360
    elif imu_delta < -180: imu_delta += 360

    n = len(samples)
    lo = max(1, n // 5)
    hi = n - lo
    mid = samples[lo:hi] if n > 5 else samples[1:]
    if mid:
        mean_err = math.degrees(sum(s["hd_err"] for s in mid) / len(mid))
        peak_err = math.degrees(max(abs(s["hd_err"]) for s in mid))
        vl = [abs(s["vel_l"]) for s in mid]
        vr = [abs(s["vel_r"]) for s in mid]
        mvl = sum(vl)/len(vl)
        mvr = sum(vr)/len(vr)
        asym = (mvl - mvr) / max((mvl+mvr)/2, 1e-6) * 100
    else:
        mean_err = peak_err = asym = 0.0

    return LegResult(
        direction=direction, enc_drift_deg=enc_drift, imu_yaw_deg=imu_delta,
        mean_err_deg=mean_err, peak_err_deg=peak_err,
        gyro_bias_dps=math.degrees(gyro_bias),
        vel_asym_pct=asym, duration_s=time.time()-t0,
    )


def main():
    ap = argparse.ArgumentParser(description="Sweep KD and IMU fusion params")
    ap.add_argument("--port", default="/dev/ttyTB3")
    ap.add_argument("--speed", type=float, default=0.10)
    ap.add_argument("--distance", type=float, default=0.50)
    ap.add_argument("--repeats", type=int, default=2)
    args = ap.parse_args()

    ser = serial.Serial(args.port, baudrate=1000000, timeout=0.2)
    time.sleep(0.5)

    keep_alive(ser)
    millis = dxl_read_i32(ser, ADDR_MILLIS)
    print(f"Pico uptime: {millis} ms")

    # Read current defaults
    cur_kd = dxl_read_f32(ser, REG_HEADING_HOLD_KD) or 0.0
    cur_alpha = dxl_read_f32(ser, REG_IMU_HEADING_ALPHA) or 0.0
    cur_beta = dxl_read_f32(ser, REG_IMU_HEADING_BIAS_BETA) or 0.0
    print(f"Current: KD={cur_kd:.4f}  alpha={cur_alpha:.4f}  beta={cur_beta:.4f}")

    # Parameter combinations to sweep — beta always 0.25 (never read from register)
    BETA = 0.25
    configs = [
        # (label, KD, alpha, beta)
        ("KD=0.01 a=0.05",                0.01, 0.05, BETA),
        ("KD=0.01 a=0.10",                0.01, 0.10, BETA),
        ("KD=0.01 a=0.15",                0.01, 0.15, BETA),
        ("KD=0.01 a=0.20",                0.01, 0.20, BETA),
        ("KD=0.02 a=0.10",                0.02, 0.10, BETA),
        ("KD=0.02 a=0.15",                0.02, 0.15, BETA),
        ("KD=0.02 a=0.20",                0.02, 0.20, BETA),
        ("KD=0.05 a=0.10",                0.05, 0.10, BETA),
        ("KD=0.05 a=0.15",                0.05, 0.15, BETA),
    ]

    results = []

    hdr = (f"{'Config':<22} {'Dir':>3} {'EncDrift':>9} {'IMU_Yaw':>9} "
           f"{'MeanErr':>8} {'PeakErr':>8} {'GyroBias':>9} {'VelAsym':>8}")
    print(f"\n{hdr}")
    print("─" * len(hdr))

    for label, kd, alpha, beta in configs:
        kd_r, al_r, be_r = set_params(ser, kd, alpha, beta)
        print(f"\n  {BLD}{label}{NC}  (KD={kd_r:.4f} α={al_r:.4f} β={be_r:.4f})")

        for rep in range(args.repeats):
            for spd_sign in [+1, -1]:
                spd = spd_sign * args.speed
                try:
                    leg = run_leg(ser, spd, args.distance)
                except Exception as exc:
                    print(f"  {RED}ERROR: {exc}{NC} — skipping")
                    try:
                        set_velocity(ser, 0.0)
                        dxl_write(ser, ADDR_TORQUE_EN, bytes([0]))
                    except Exception:
                        pass
                    time.sleep(2.0)
                    continue
                d_clr = GRN if abs(leg.imu_yaw_deg) < 3 else (YLW if abs(leg.imu_yaw_deg) < 8 else RED)
                p_clr = GRN if leg.peak_err_deg < 2 else (YLW if leg.peak_err_deg < 5 else RED)
                print(f"  {label:<22} {leg.direction:>3} "
                      f"{leg.enc_drift_deg:+8.2f}° "
                      f"{d_clr}{leg.imu_yaw_deg:+8.2f}°{NC} "
                      f"{leg.mean_err_deg:+7.2f}° "
                      f"{p_clr}{leg.peak_err_deg:7.2f}°{NC} "
                      f"{leg.gyro_bias_dps:+8.2f}°/s "
                      f"{leg.vel_asym_pct:+7.2f}%")
                results.append((label, leg))
                time.sleep(0.8)

    # Restore original params
    set_params(ser, cur_kd, cur_alpha, BETA)
    print(f"\n  Restored: KD={cur_kd:.4f}  alpha={cur_alpha:.4f}  beta={BETA:.4f}")

    # Summary: average |IMU yaw| per config
    print(f"\n{'═'*70}")
    print(f"  {'Config':<22} {'Avg|IMU|':>9} {'Avg|Enc|':>9} {'AvgPeak':>8}")
    print(f"  {'─'*52}")
    seen = []
    for label, _, _, _ in configs:
        if label in seen:
            continue
        seen.append(label)
        legs = [r for l, r in results if l == label]
        avg_imu = sum(abs(r.imu_yaw_deg) for r in legs) / len(legs)
        avg_enc = sum(abs(r.enc_drift_deg) for r in legs) / len(legs)
        avg_peak = sum(r.peak_err_deg for r in legs) / len(legs)
        clr = GRN if avg_imu < 3 else (YLW if avg_imu < 8 else RED)
        print(f"  {label:<22} {clr}{avg_imu:8.2f}°{NC} {avg_enc:8.2f}° {avg_peak:7.2f}°")

    ser.close()
    print("\nDone.")


if __name__ == "__main__":
    main()
