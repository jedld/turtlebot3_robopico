#!/usr/bin/env python3
"""
test_encoder_ticks.py — Verify encoder tick count accuracy at various speeds.

Drives the robot a fixed distance at each speed and compares the measured
encoder counts against the expected count for that distance.  Detects tick
loss that would indicate the PIO quadrature decoder or firmware read path
is dropping counts at higher speeds.

Expected counts = distance / ENC_M_PER_COUNT

Needs exclusive serial access (stops/restarts turtlebot3-bringup).
"""
from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from dataclasses import dataclass, fields
from datetime import datetime
from pathlib import Path

import serial

from dxl_utils import *  # noqa: F401,F403

# ── defaults ───────────────────────────────────────────────────────────────
DEFAULT_SPEEDS = [0.03, 0.05, 0.08, 0.10, 0.15, 0.20]
DEFAULT_DISTANCE = 0.50  # metres per leg
DEFAULT_REPEATS = 2

_bringup_was_active = False


# ── bringup helpers ────────────────────────────────────────────────────────
def stop_bringup():
    global _bringup_was_active
    try:
        result = subprocess.run(
            ["systemctl", "is-active", "--quiet", BRINGUP_SERVICE],
            timeout=5,
        )
        _bringup_was_active = result.returncode == 0
    except Exception:
        _bringup_was_active = False

    if _bringup_was_active:
        print("Stopping turtlebot3-bringup …")
        subprocess.run(
            ["sudo", "systemctl", "stop", BRINGUP_SERVICE],
            timeout=15,
        )
        time.sleep(2.0)


def restart_bringup():
    if _bringup_was_active:
        print("Restarting turtlebot3-bringup …")
        try:
            subprocess.run(
                ["sudo", "systemctl", "start", BRINGUP_SERVICE],
                timeout=15,
            )
        except Exception as exc:
            print(f"  warning: restart failed: {exc}")


# ── data ───────────────────────────────────────────────────────────────────
@dataclass
class LegResult:
    direction: str
    speed_mps: float
    repeat: int
    enc_l: int
    enc_r: int
    expected_counts: float
    err_l_pct: float  # (measured - expected) / expected * 100
    err_r_pct: float
    lr_diff_pct: float  # (L - R) / avg * 100
    dist_l_m: float
    dist_r_m: float
    duration_s: float
    mean_vel_l: float
    mean_vel_r: float


# ── core ───────────────────────────────────────────────────────────────────
def run_leg(ser, speed: float, distance_m: float, enc_m_per_count: float) -> LegResult:
    direction = "FWD" if speed > 0 else "REV"
    abs_speed = abs(speed)
    timeout_s = (distance_m / abs_speed) * 2.5 + 5.0

    expected_counts = distance_m / enc_m_per_count

    # Enable torque
    dxl_write(ser, ADDR_TORQUE_EN, bytes([1]))
    time.sleep(0.05)

    # Baseline encoder counts
    enc_l0 = dxl_read_i32(ser, REG_ENC_L) or 0
    enc_r0 = dxl_read_i32(ser, REG_ENC_R) or 0

    # Drive
    set_velocity(ser, speed)
    t0 = time.time()

    vel_l_samples = []
    vel_r_samples = []

    while time.time() - t0 < timeout_s:
        enc_l = dxl_read_i32(ser, REG_ENC_L) or enc_l0
        enc_r = dxl_read_i32(ser, REG_ENC_R) or enc_r0
        vel_l = dxl_read_f32(ser, REG_DBG_VEL_L) or 0.0
        vel_r = dxl_read_f32(ser, REG_DBG_VEL_R) or 0.0

        vel_l_samples.append(abs(vel_l))
        vel_r_samples.append(abs(vel_r))

        dl = abs(enc_l - enc_l0) * enc_m_per_count
        dr = abs(enc_r - enc_r0) * enc_m_per_count
        avg_dist = (dl + dr) / 2.0
        if avg_dist >= distance_m:
            break
        time.sleep(0.05)

    # Stop
    set_velocity(ser, 0.0)
    time.sleep(0.3)
    dxl_write(ser, ADDR_TORQUE_EN, bytes([0]))
    time.sleep(1.0)

    duration = time.time() - t0

    # Final encoder counts
    enc_l1 = dxl_read_i32(ser, REG_ENC_L) or enc_l0
    enc_r1 = dxl_read_i32(ser, REG_ENC_R) or enc_r0

    delta_l = abs(enc_l1 - enc_l0)
    delta_r = abs(enc_r1 - enc_r0)
    dist_l = delta_l * enc_m_per_count
    dist_r = delta_r * enc_m_per_count

    err_l_pct = (delta_l - expected_counts) / expected_counts * 100.0
    err_r_pct = (delta_r - expected_counts) / expected_counts * 100.0
    avg_counts = (delta_l + delta_r) / 2.0
    lr_diff_pct = ((delta_l - delta_r) / avg_counts * 100.0) if avg_counts > 0 else 0.0

    mean_vl = sum(vel_l_samples) / len(vel_l_samples) if vel_l_samples else 0.0
    mean_vr = sum(vel_r_samples) / len(vel_r_samples) if vel_r_samples else 0.0

    return LegResult(
        direction=direction,
        speed_mps=speed,
        repeat=0,
        enc_l=delta_l,
        enc_r=delta_r,
        expected_counts=expected_counts,
        err_l_pct=err_l_pct,
        err_r_pct=err_r_pct,
        lr_diff_pct=lr_diff_pct,
        dist_l_m=dist_l,
        dist_r_m=dist_r,
        duration_s=duration,
        mean_vel_l=mean_vl * 1000,
        mean_vel_r=mean_vr * 1000,
    )


# ── display ────────────────────────────────────────────────────────────────
def colour_err(pct):
    a = abs(pct)
    return GRN if a < 1.0 else (YLW if a < 3.0 else RED)


def print_results(results: list[LegResult]):
    hdr = (f"  {'Dir':>3} {'Spd':>5} {'Rep':>3}  "
           f"{'EncL':>7} {'EncR':>7} {'Expect':>7}  "
           f"{'ErrL%':>7} {'ErrR%':>7} {'L-R%':>6}  "
           f"{'DistL':>7} {'DistR':>7}  "
           f"{'vL':>6} {'vR':>6}  {'Time':>5}")
    print(f"\n{hdr}")
    print(f"  {'─' * (len(hdr) - 2)}")
    for r in results:
        cl = colour_err(r.err_l_pct)
        cr = colour_err(r.err_r_pct)
        cd = colour_err(r.lr_diff_pct)
        print(f"  {r.direction:>3} {abs(r.speed_mps):5.3f} {r.repeat:3d}  "
              f"{r.enc_l:7d} {r.enc_r:7d} {r.expected_counts:7.0f}  "
              f"{cl}{r.err_l_pct:+6.2f}%{NC} {cr}{r.err_r_pct:+6.2f}%{NC} "
              f"{cd}{r.lr_diff_pct:+5.2f}%{NC}  "
              f"{r.dist_l_m:7.4f} {r.dist_r_m:7.4f}  "
              f"{r.mean_vel_l:5.1f} {r.mean_vel_r:5.1f}  "
              f"{r.duration_s:5.1f}s")


def print_summary(results: list[LegResult]):
    print(f"\n{BLD}  Speed-grouped summary (tick accuracy vs expected):{NC}")
    print(f"  {'Speed':>6} {'Dir':>3}  {'AvgErrL%':>9} {'AvgErrR%':>9}  {'AvgL-R%':>8}  {'N':>2}")
    print(f"  {'─' * 50}")

    speeds = sorted(set(abs(r.speed_mps) for r in results))
    for spd in speeds:
        for d in ("FWD", "REV"):
            legs = [r for r in results if abs(r.speed_mps) == spd and r.direction == d]
            if not legs:
                continue
            avg_el = sum(r.err_l_pct for r in legs) / len(legs)
            avg_er = sum(r.err_r_pct for r in legs) / len(legs)
            avg_lr = sum(r.lr_diff_pct for r in legs) / len(legs)
            cl = colour_err(avg_el)
            cr = colour_err(avg_er)
            cd = colour_err(avg_lr)
            print(f"  {spd:6.3f} {d:>3}  "
                  f"{cl}{avg_el:+8.2f}%{NC} {cr}{avg_er:+8.2f}%{NC}  "
                  f"{cd}{avg_lr:+7.2f}%{NC}  {len(legs):2d}")

    # Check for speed-dependent tick loss trend
    print(f"\n{BLD}  Tick loss trend (err% vs speed):{NC}")
    for d in ("FWD", "REV"):
        errs = []
        for spd in speeds:
            legs = [r for r in results if abs(r.speed_mps) == spd and r.direction == d]
            if legs:
                avg_err = (sum(r.err_l_pct for r in legs) + sum(r.err_r_pct for r in legs)) / (2 * len(legs))
                errs.append((spd, avg_err))
        if len(errs) >= 2:
            # Simple linear trend: if error gets more negative with speed → tick loss
            slope = (errs[-1][1] - errs[0][1]) / (errs[-1][0] - errs[0][0])
            trend_clr = GRN if abs(slope) < 5 else (YLW if abs(slope) < 15 else RED)
            print(f"    {d}: slope = {trend_clr}{slope:+.1f} %/( m/s){NC}  "
                  f"({'OK — no tick loss trend' if abs(slope) < 5 else 'WARNING — possible tick loss at speed'})")


# ── main ───────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Encoder tick accuracy test at multiple speeds")
    ap.add_argument("--port", default="/dev/ttyTB3")
    ap.add_argument("--speeds", type=float, nargs="+", default=DEFAULT_SPEEDS,
                    help=f"Speed(s) in m/s (default: {DEFAULT_SPEEDS})")
    ap.add_argument("--distance", type=float, default=DEFAULT_DISTANCE,
                    help=f"Distance per leg in metres (default: {DEFAULT_DISTANCE})")
    ap.add_argument("--repeats", type=int, default=DEFAULT_REPEATS,
                    help=f"Repeats per speed/direction (default: {DEFAULT_REPEATS})")
    args = ap.parse_args()

    print(f"\n{BLD}test_encoder_ticks.py{NC} — Encoder tick accuracy at various speeds")
    print(f"  port={args.port}  distance={args.distance:.2f}m  "
          f"speeds={args.speeds}  repeats={args.repeats}")

    stop_bringup()
    results: list[LegResult] = []

    try:
        ser = serial.Serial(args.port, baudrate=BAUD, timeout=0.2)
        time.sleep(0.5)

        keep_alive(ser)
        millis = dxl_read_i32(ser, REG_MILLIS)
        if millis is None:
            print(f"{RED}Cannot reach Pico — check port.{NC}")
            return 1
        print(f"  Pico uptime: {millis} ms")

        # Query wheel radius from firmware for accurate expected counts
        wr = query_wheel_radius(ser)
        if wr and wr > 0.01:
            enc_m_per_count = (2.0 * math.pi * wr) / ENC_COUNTS_PER_REV
            print(f"  Wheel radius (firmware): {wr:.6f} m")
        else:
            enc_m_per_count = ENC_M_PER_COUNT
            wr = WHEEL_RADIUS_M
            print(f"{YLW}  Warning: using default wheel radius {wr:.6f} m{NC}")
        print(f"  ENC_COUNTS_PER_REV = {ENC_COUNTS_PER_REV:.0f}")
        print(f"  ENC_M_PER_COUNT    = {enc_m_per_count:.8f} m")
        expected_per_leg = args.distance / enc_m_per_count
        print(f"  Expected counts for {args.distance:.2f} m = {expected_per_leg:.0f}")

        for spd in args.speeds:
            print(f"\n{'═' * 60}")
            print(f"  Speed: {spd:.3f} m/s   Distance: {args.distance:.2f} m   Repeats: {args.repeats}")
            print(f"{'═' * 60}")

            for rep in range(1, args.repeats + 1):
                # FWD
                print(f"  Rep {rep}/{args.repeats}  FWD … ", end="", flush=True)
                leg = run_leg(ser, +spd, args.distance, enc_m_per_count)
                leg.repeat = rep
                results.append(leg)
                cl = colour_err(leg.err_l_pct)
                cr = colour_err(leg.err_r_pct)
                print(f"L={leg.enc_l} R={leg.enc_r} exp={leg.expected_counts:.0f}  "
                      f"errL={cl}{leg.err_l_pct:+.2f}%{NC} errR={cr}{leg.err_r_pct:+.2f}%{NC}")

                # REV
                print(f"  Rep {rep}/{args.repeats}  REV … ", end="", flush=True)
                leg = run_leg(ser, -spd, args.distance, enc_m_per_count)
                leg.repeat = rep
                results.append(leg)
                cl = colour_err(leg.err_l_pct)
                cr = colour_err(leg.err_r_pct)
                print(f"L={leg.enc_l} R={leg.enc_r} exp={leg.expected_counts:.0f}  "
                      f"errL={cl}{leg.err_l_pct:+.2f}%{NC} errR={cr}{leg.err_r_pct:+.2f}%{NC}")

                # Pause to reposition unless this is the very last leg
                is_last = (spd == args.speeds[-1] and rep == args.repeats)
                if not is_last:
                    try:
                        input(f"  {YLW}Reposition robot and press ENTER for next leg …{NC} ")
                    except EOFError:
                        pass

        ser.close()

    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as exc:
        print(f"{RED}Error: {exc}{NC}")
        import traceback; traceback.print_exc()
    finally:
        print_results(results)
        print_summary(results)
        restart_bringup()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
