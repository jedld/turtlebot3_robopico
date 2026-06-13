#!/usr/bin/env python3
"""
test_fwd_rev_drift.py — Alternating forward/reverse drift test at multiple speeds.

Drives 1 m forward then 1 m backward at each requested speed, recording
heading drift per leg. Repeats for every speed in the list.

Needs exclusive serial access (stops/restarts turtlebot3-bringup).
"""
from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
import time
from dataclasses import dataclass, fields
from datetime import datetime
from pathlib import Path

import serial

from dxl_utils import *  # noqa: F401,F403 — protocol, registers, colours


def quat_yaw_deg(ser) -> float:
    """Read BNO085 game rotation vector quaternion (mag-free) and return yaw in degrees."""
    qw = dxl_read_f32(ser, REG_GAME_ROT_W) or 1.0
    qx = dxl_read_f32(ser, REG_GAME_ROT_X) or 0.0
    qy = dxl_read_f32(ser, REG_GAME_ROT_Y) or 0.0
    qz = dxl_read_f32(ser, REG_GAME_ROT_Z) or 0.0
    # yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def colour_drift(deg):
    a = abs(deg)
    return GRN if a < 0.5 else (YLW if a < 1.0 else RED)


# ── Data ───────────────────────────────────────────────────────────────────
@dataclass
class LegResult:
    speed_mps: float
    direction: str  # "FWD" or "REV"
    distance_l_m: float
    distance_r_m: float
    distance_avg_m: float
    drift_deg: float
    gyro_drift_deg: float
    enc_drift_deg: float
    imu_yaw_drift_deg: float
    mean_err_deg: float
    peak_err_deg: float
    mean_corr_dps: float
    vel_asym_pct: float
    mean_vel_l: float
    mean_vel_r: float
    n_samples: int
    duration_s: float


# ── Core logic ─────────────────────────────────────────────────────────────
def run_leg(ser, speed: float, target_dist_m: float) -> LegResult:
    """
    Drive at `speed` m/s (positive = fwd, negative = rev) until
    `target_dist_m` of travel is reached or a timeout expires.
    Returns a LegResult with drift stats.
    """
    direction = "FWD" if speed > 0 else "REV"
    abs_speed = abs(speed)
    timeout_s = (target_dist_m / abs_speed) * 2.0 + 5.0  # generous timeout

    # Enable torque and drive
    dxl_write(ser, ADDR_TORQUE_EN, bytes([1]))
    time.sleep(0.05)

    # Read initial encoder positions
    enc_l0 = dxl_read_i32(ser, ADDR_ENC_L) or 0
    enc_r0 = dxl_read_i32(ser, ADDR_ENC_R) or 0

    # IMU yaw baseline BEFORE motors start (no vibration/magnetic interference)
    imu_yaw0 = quat_yaw_deg(ser)

    set_velocity(ser, speed)
    time.sleep(0.1)  # let firmware process velocity cmd and reset heading accumulators

    # Read gyro/enc heading baselines AFTER velocity cmd (firmware resets them)
    hdg_gyro0 = dxl_read_f32(ser, REG_DBG_HEADING_GYRO) or 0.0
    hdg_enc0 = dxl_read_f32(ser, REG_DBG_HEADING_ENC) or 0.0

    samples = []
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        hd_err = dxl_read_f32(ser, ADDR_DBG_HEADING_ERR) or 0.0
        hd_corr = dxl_read_f32(ser, ADDR_HEADING_HOLD_CORR) or 0.0
        vel_l = dxl_read_f32(ser, ADDR_DBG_VEL_L) or 0.0
        vel_r = dxl_read_f32(ser, ADDR_DBG_VEL_R) or 0.0
        enc_l = dxl_read_i32(ser, ADDR_ENC_L) or enc_l0
        enc_r = dxl_read_i32(ser, ADDR_ENC_R) or enc_r0
        hdg_gyro = dxl_read_f32(ser, REG_DBG_HEADING_GYRO) or 0.0
        hdg_enc = dxl_read_f32(ser, REG_DBG_HEADING_ENC) or 0.0

        dl = (enc_l - enc_l0) * ENC_M_PER_COUNT
        dr = (enc_r - enc_r0) * ENC_M_PER_COUNT
        avg_dist = (abs(dl) + abs(dr)) / 2.0

        samples.append(dict(
            t=time.time() - t0,
            hd_err=hd_err, hd_corr=hd_corr,
            vel_l=vel_l, vel_r=vel_r,
            enc_l=enc_l, enc_r=enc_r,
            dl=dl, dr=dr, avg_dist=avg_dist,
            hdg_gyro=hdg_gyro - hdg_gyro0,
            hdg_enc=hdg_enc - hdg_enc0,
        ))

        if avg_dist >= target_dist_m:
            break

        time.sleep(0.06)

    # Stop
    set_velocity(ser, 0.0)
    time.sleep(0.3)
    dxl_write(ser, ADDR_TORQUE_EN, bytes([0]))
    time.sleep(1.5)

    # IMU yaw AFTER motors stop and settle (no vibration/magnetic interference)
    imu_yaw1 = quat_yaw_deg(ser)

    if not samples:
        return LegResult(speed_mps=speed, direction=direction,
                         distance_l_m=0, distance_r_m=0, distance_avg_m=0,
                         drift_deg=0, gyro_drift_deg=0, enc_drift_deg=0,
                         imu_yaw_drift_deg=0,
                         mean_err_deg=0, peak_err_deg=0,
                         mean_corr_dps=0, vel_asym_pct=0,
                         mean_vel_l=0, mean_vel_r=0, n_samples=0, duration_s=0)

    # Compute results
    last = samples[-1]
    dist_l = abs(last["dl"])
    dist_r = abs(last["dr"])
    dist_avg = (dist_l + dist_r) / 2.0

    # Drift from encoder difference → heading
    enc_diff_m = last["dl"] - last["dr"]
    drift_deg = math.degrees(enc_diff_m / WHEEL_BASE_M)

    # IMU gyro-integrated heading and encoder-only heading from firmware
    gyro_drift_deg = math.degrees(last["hdg_gyro"])
    enc_drift_deg = math.degrees(last["hdg_enc"])

    # BNO085 rotation-vector yaw delta (sensor-fused, bias-compensated)
    # Read before motors start and after motors stop to avoid magnetic interference
    imu_yaw_delta = imu_yaw1 - imu_yaw0
    # Handle wraparound (±180°)
    if imu_yaw_delta > 180.0:
        imu_yaw_delta -= 360.0
    elif imu_yaw_delta < -180.0:
        imu_yaw_delta += 360.0

    # Steady-state stats from middle 60%
    n = len(samples)
    lo = max(1, n // 5)
    hi = n - lo
    mid = samples[lo:hi] if n > 5 else samples[1:]

    if mid:
        mean_err = sum(s["hd_err"] for s in mid) / len(mid)
        peak_err = max(abs(s["hd_err"]) for s in mid)
        mean_corr = sum(s["hd_corr"] for s in mid) / len(mid)
        vl_vals = [abs(s["vel_l"]) for s in mid]
        vr_vals = [abs(s["vel_r"]) for s in mid]
        mean_vl = sum(vl_vals) / len(vl_vals)
        mean_vr = sum(vr_vals) / len(vr_vals)
        asym = (mean_vl - mean_vr) / max((mean_vl + mean_vr) / 2.0, 1e-6) * 100.0
    else:
        mean_err = peak_err = mean_corr = 0.0
        mean_vl = mean_vr = asym = 0.0

    return LegResult(
        speed_mps=speed, direction=direction,
        distance_l_m=dist_l, distance_r_m=dist_r, distance_avg_m=dist_avg,
        drift_deg=drift_deg,
        gyro_drift_deg=gyro_drift_deg,
        enc_drift_deg=enc_drift_deg,
        imu_yaw_drift_deg=imu_yaw_delta,
        mean_err_deg=math.degrees(mean_err),
        peak_err_deg=math.degrees(peak_err),
        mean_corr_dps=math.degrees(mean_corr),
        vel_asym_pct=asym,
        mean_vel_l=mean_vl * 1000,
        mean_vel_r=mean_vr * 1000,
        n_samples=n,
        duration_s=last["t"],
    )


def main():
    ap = argparse.ArgumentParser(
        description="Alternating fwd/rev drift test at multiple speeds (1 m per leg)")
    ap.add_argument("--port", default="/dev/ttyTB3")
    ap.add_argument("--distance", type=float, default=1.0,
                    help="Distance per leg in metres (default: 1.0)")
    ap.add_argument("--speeds", nargs="+", type=float,
                    default=[0.05, 0.08, 0.10, 0.15, 0.20],
                    help="Speeds to test (m/s). Both fwd and rev tested at each.")
    ap.add_argument("--settle", type=float, default=2.0,
                    help="Settle time between legs (s)")
    args = ap.parse_args()

    csv_path = Path(f"fwd_rev_drift_{datetime.now():%Y%m%d_%H%M%S}.csv")

    print(f"\n{BLD}test_fwd_rev_drift.py{NC} — Alternating forward/reverse drift test")
    print(f"  port={args.port}  distance={args.distance:.1f}m  speeds={args.speeds}")
    print(f"  CSV: {csv_path}\n")

    # Stop bringup (only if running)
    _bringup_was_active = False
    try:
        _bringup_was_active = subprocess.run(
            ["systemctl", "is-active", "--quiet", BRINGUP_SERVICE],
            timeout=5).returncode == 0
    except Exception:
        pass
    if _bringup_was_active:
        print("Stopping turtlebot3-bringup.service ...")
        subprocess.run(["sudo", "systemctl", "stop", BRINGUP_SERVICE],
                       capture_output=True, timeout=10)
        time.sleep(1.0)
    else:
        print("turtlebot3-bringup.service already stopped.")

    ser = serial.Serial(args.port, BAUD, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    millis = dxl_read_i32(ser, ADDR_MILLIS)
    if millis is None:
        print(f"{RED}Cannot reach Pico — check port.{NC}")
        ser.close()
        return 1
    print(f"Pico uptime: {millis} ms")

    # Query wheel separation from firmware
    global WHEEL_BASE_M, ENC_M_PER_COUNT
    ws = query_wheel_separation(ser)
    if ws and ws > 0.01:
        WHEEL_BASE_M = ws
    else:
        WHEEL_BASE_M = 0.185  # fallback to firmware default
        print(f"{YLW}Warning: could not read wheel separation, using default {WHEEL_BASE_M:.4f} m{NC}")
    print(f"  Wheel separation: {WHEEL_BASE_M:.6f} m")

    # Query wheel radius from firmware and recompute ENC_M_PER_COUNT
    wr = query_wheel_radius(ser)
    if wr and wr > 0.01:
        ENC_M_PER_COUNT = (2.0 * math.pi * wr) / ENC_COUNTS_PER_REV
        print(f"  Wheel radius:     {wr:.6f} m  (ENC_M_PER_COUNT={ENC_M_PER_COUNT:.8f})")
    else:
        print(f"{YLW}Warning: could not read wheel radius, using default {WHEEL_RADIUS_M:.5f} m{NC}")

    # Read current config
    kp = dxl_read_f32(ser, ADDR_HEADING_HOLD_KP) or 0.0
    ki = dxl_read_f32(ser, ADDR_HEADING_HOLD_KI) or 0.0
    iseed_fwd = dxl_read_f32(ser, ADDR_I_SEED_FWD) or 0.0
    iseed_rev = dxl_read_f32(ser, ADDR_I_SEED_REV) or 0.0
    vtrim_fwd = dxl_read_f32(ser, ADDR_VEL_TRIM_FWD) or 0.0
    vtrim_rev = dxl_read_f32(ser, ADDR_VEL_TRIM_REV) or 0.0
    hh_en = dxl_read_i32(ser, ADDR_HEADING_HOLD_EN)

    print(f"  HH: en={hh_en}  KP={kp:.2f}  KI={ki:.4f}")
    print(f"  I_SEED: fwd={iseed_fwd:.6f} rev={iseed_rev:.6f}")
    print(f"  VEL_TRIM: fwd={vtrim_fwd:.6f} rev={vtrim_rev:.6f}\n")

    results: list[LegResult] = []

    for speed in args.speeds:
        print(f"{BLD}{'─'*60}")
        print(f"  Speed: {speed:.3f} m/s   (distance: {args.distance:.1f} m per leg)")
        print(f"{'─'*60}{NC}")

        # Forward leg
        print(f"  [{CYN}FWD{NC}] driving at +{speed:.3f} m/s ... ", end="", flush=True)
        r_fwd = run_leg(ser, +speed, args.distance)
        c = colour_drift(r_fwd.drift_deg)
        ci = colour_drift(r_fwd.imu_yaw_drift_deg)
        print(f"dist={r_fwd.distance_avg_m:.3f}m  enc_drift={c}{r_fwd.drift_deg:+.2f}°{NC}  "
              f"imu_yaw={ci}{r_fwd.imu_yaw_drift_deg:+.2f}°{NC}  "
              f"err={r_fwd.mean_err_deg:+.2f}°  peak={r_fwd.peak_err_deg:.2f}°  "
              f"vL={r_fwd.mean_vel_l:.1f} vR={r_fwd.mean_vel_r:.1f} mm/s  "
              f"t={r_fwd.duration_s:.1f}s")
        results.append(r_fwd)

        try:
            input(f"  {YLW}Reposition robot and press ENTER for REV leg …{NC} ")
        except EOFError:
            time.sleep(args.settle)

        # Reverse leg
        print(f"  [{CYN}REV{NC}] driving at -{speed:.3f} m/s ... ", end="", flush=True)
        r_rev = run_leg(ser, -speed, args.distance)
        c = colour_drift(r_rev.drift_deg)
        ci = colour_drift(r_rev.imu_yaw_drift_deg)
        print(f"dist={r_rev.distance_avg_m:.3f}m  enc_drift={c}{r_rev.drift_deg:+.2f}°{NC}  "
              f"imu_yaw={ci}{r_rev.imu_yaw_drift_deg:+.2f}°{NC}  "
              f"err={r_rev.mean_err_deg:+.2f}°  peak={r_rev.peak_err_deg:.2f}°  "
              f"vL={r_rev.mean_vel_l:.1f} vR={r_rev.mean_vel_r:.1f} mm/s  "
              f"t={r_rev.duration_s:.1f}s")
        results.append(r_rev)

        # Pause to reposition unless this is the last speed
        if speed != args.speeds[-1]:
            try:
                input(f"  {YLW}Reposition robot and press ENTER for next speed …{NC} ")
            except EOFError:
                time.sleep(args.settle)

    ser.close()

    # ── Save CSV ──────────────────────────────────────────────────────────
    fieldnames = [f.name for f in fields(LegResult)]
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in results:
            row = {f.name: getattr(r, f.name) for f in fields(LegResult)}
            w.writerow(row)
    print(f"\nCSV saved: {csv_path}")

    # ── Summary table ─────────────────────────────────────────────────────
    print(f"\n{BLD}{'='*80}")
    print(f"  SUMMARY")
    print(f"{'='*80}{NC}")
    print(f"  {'Speed':>6s}  {'Dir':>3s}  {'Dist':>6s}  {'EncDrift':>9s}  "
          f"{'IMU Yaw':>9s}  "
          f"{'MeanErr':>8s}  {'PeakErr':>8s}  {'VelAsym':>8s}  {'vL':>6s}  {'vR':>6s}")
    print(f"  {'-'*82}")

    for r in results:
        c = colour_drift(r.drift_deg)
        ci = colour_drift(r.imu_yaw_drift_deg)
        print(f"  {abs(r.speed_mps):6.3f}  {r.direction:>3s}  "
              f"{r.distance_avg_m:5.3f}m  {c}{r.drift_deg:+8.2f}°{NC}  "
              f"{ci}{r.imu_yaw_drift_deg:+8.2f}°{NC}  "
              f"{r.mean_err_deg:+7.2f}°  {r.peak_err_deg:7.2f}°  "
              f"{r.vel_asym_pct:+7.2f}%  "
              f"{r.mean_vel_l:5.1f}  {r.mean_vel_r:5.1f}")

    # ── Per-speed fwd vs rev comparison ───────────────────────────────────
    print(f"\n{BLD}  FWD vs REV comparison (encoder / IMU yaw):{NC}")
    print(f"  {'Speed':>6s}  {'FWD enc':>9s}  {'FWD imu':>9s}  "
          f"{'REV enc':>9s}  {'REV imu':>9s}  "
          f"{'Net enc':>9s}  {'Net imu':>9s}")
    print(f"  {'-'*68}")

    for i in range(0, len(results), 2):
        fwd = results[i]
        rev = results[i + 1]
        net_enc = fwd.drift_deg + rev.drift_deg
        net_imu = fwd.imu_yaw_drift_deg + rev.imu_yaw_drift_deg
        print(f"  {abs(fwd.speed_mps):6.3f}  "
              f"{colour_drift(fwd.drift_deg)}{fwd.drift_deg:+8.2f}°{NC}  "
              f"{colour_drift(fwd.imu_yaw_drift_deg)}{fwd.imu_yaw_drift_deg:+8.2f}°{NC}  "
              f"{colour_drift(rev.drift_deg)}{rev.drift_deg:+8.2f}°{NC}  "
              f"{colour_drift(rev.imu_yaw_drift_deg)}{rev.imu_yaw_drift_deg:+8.2f}°{NC}  "
              f"{colour_drift(net_enc)}{net_enc:+8.2f}°{NC}  "
              f"{colour_drift(net_imu)}{net_imu:+8.2f}°{NC}")

    # Restart bringup (only if we stopped it)
    if _bringup_was_active:
        print(f"\nRestarting {BRINGUP_SERVICE} ...")
        try:
            subprocess.run(["sudo", "systemctl", "start", BRINGUP_SERVICE],
                           capture_output=True, timeout=10)
        except subprocess.TimeoutExpired:
            print(f"{YLW}Timeout restarting service — run manually: sudo systemctl start {BRINGUP_SERVICE}{NC}")
        print("Done.\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
