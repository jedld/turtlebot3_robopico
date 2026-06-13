#!/usr/bin/env python3
"""
test_encoder_symmetry.py — Check left/right encoder symmetry during forward
and reverse driving.

Drives the robot forward and backward at one or more speeds, recording raw
left and right encoder counts.  Prints a table comparing L vs R counts and
distance for each leg so asymmetry between encoders is immediately visible.

Needs exclusive serial access (stops/restarts turtlebot3-bringup).
"""
from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime

import serial

from dxl_utils import *  # noqa: F401,F403

# ── defaults ───────────────────────────────────────────────────────────────
DEFAULT_SPEEDS = [0.10]
DEFAULT_DISTANCE = 0.50   # metres per leg
DEFAULT_REPEATS = 3

_bringup_was_active = False


# ── bringup helpers ────────────────────────────────────────────────────────
def stop_bringup():
    global _bringup_was_active
    try:
        result = subprocess.run(
            ["systemctl", "is-active", "--quiet", "turtlebot3-bringup"],
            timeout=5,
        )
        _bringup_was_active = result.returncode == 0
    except Exception:
        _bringup_was_active = False

    if _bringup_was_active:
        print("Stopping turtlebot3-bringup …")
        subprocess.run(
            ["sudo", "systemctl", "stop", "turtlebot3-bringup"],
            timeout=15,
        )
        time.sleep(2.0)


def restart_bringup():
    if _bringup_was_active:
        print("Restarting turtlebot3-bringup …")
        try:
            subprocess.run(
                ["sudo", "systemctl", "start", "turtlebot3-bringup"],
                timeout=15,
            )
        except Exception as exc:
            print(f"  warning: restart failed: {exc}")


# ── data ───────────────────────────────────────────────────────────────────
@dataclass
class LegData:
    direction: str     # FWD / REV
    speed_mps: float
    repeat: int
    enc_l_counts: int
    enc_r_counts: int
    dist_l_m: float
    dist_r_m: float
    diff_counts: int   # L − R
    diff_m: float      # L − R in metres
    diff_pct: float    # (L − R) / avg * 100
    duration_s: float


# ── core ───────────────────────────────────────────────────────────────────
def run_leg(ser, speed: float, distance_m: float) -> LegData:
    direction = "FWD" if speed > 0 else "REV"
    abs_speed = abs(speed)
    timeout_s = (distance_m / abs_speed) * 2.0 + 5.0

    # Enable torque
    dxl_write(ser, ADDR_TORQUE_EN, bytes([1]))
    time.sleep(0.05)

    # Baseline encoder counts
    enc_l0 = dxl_read_i32(ser, REG_ENC_L) or 0
    enc_r0 = dxl_read_i32(ser, REG_ENC_R) or 0

    # Drive
    set_velocity(ser, speed)
    t0 = time.time()

    while time.time() - t0 < timeout_s:
        enc_l = dxl_read_i32(ser, REG_ENC_L) or enc_l0
        enc_r = dxl_read_i32(ser, REG_ENC_R) or enc_r0
        dl = abs(enc_l - enc_l0) * ENC_M_PER_COUNT
        dr = abs(enc_r - enc_r0) * ENC_M_PER_COUNT
        avg_dist = (dl + dr) / 2.0
        if avg_dist >= distance_m:
            break
        time.sleep(0.05)

    # Stop
    set_velocity(ser, 0.0)
    time.sleep(0.3)
    dxl_write(ser, ADDR_TORQUE_EN, bytes([0]))
    time.sleep(1.0)

    # Final encoder counts
    enc_l1 = dxl_read_i32(ser, REG_ENC_L) or enc_l0
    enc_r1 = dxl_read_i32(ser, REG_ENC_R) or enc_r0

    delta_l = enc_l1 - enc_l0
    delta_r = enc_r1 - enc_r0
    dist_l = delta_l * ENC_M_PER_COUNT
    dist_r = delta_r * ENC_M_PER_COUNT
    diff_counts = delta_l - delta_r
    diff_m = dist_l - dist_r
    avg_abs = (abs(dist_l) + abs(dist_r)) / 2.0
    diff_pct = (diff_m / avg_abs * 100.0) if avg_abs > 1e-6 else 0.0
    duration = time.time() - t0

    return LegData(
        direction=direction,
        speed_mps=speed,
        repeat=0,
        enc_l_counts=delta_l,
        enc_r_counts=delta_r,
        dist_l_m=dist_l,
        dist_r_m=dist_r,
        diff_counts=diff_counts,
        diff_m=diff_m,
        diff_pct=diff_pct,
        duration_s=duration,
    )


# ── display ────────────────────────────────────────────────────────────────
def print_table(results: list[LegData]):
    hdr = (
        f"{'Dir':>4} {'Spd':>5} {'Rep':>3} "
        f"{'EncL':>8} {'EncR':>8} {'L-R':>7} "
        f"{'DistL_m':>9} {'DistR_m':>9} {'Diff_m':>9} {'Diff%':>7} "
        f"{'Time_s':>6}"
    )
    print()
    print(hdr)
    print("─" * len(hdr))

    for r in results:
        diff_clr = GRN if abs(r.diff_pct) < 0.5 else (YLW if abs(r.diff_pct) < 1.5 else RED)
        print(
            f"{r.direction:>4} {abs(r.speed_mps):5.2f} {r.repeat:3d} "
            f"{r.enc_l_counts:8d} {r.enc_r_counts:8d} {r.diff_counts:7d} "
            f"{r.dist_l_m:9.5f} {r.dist_r_m:9.5f} "
            f"{diff_clr}{r.diff_m:9.5f}{NC} {diff_clr}{r.diff_pct:+6.2f}%{NC} "
            f"{r.duration_s:6.2f}"
        )


def print_summary(results: list[LegData]):
    """Print aggregated per-direction statistics."""
    for direction in ("FWD", "REV"):
        legs = [r for r in results if r.direction == direction]
        if not legs:
            continue
        avg_diff_counts = sum(r.diff_counts for r in legs) / len(legs)
        avg_diff_m = sum(r.diff_m for r in legs) / len(legs)
        avg_diff_pct = sum(r.diff_pct for r in legs) / len(legs)
        max_diff_pct = max(abs(r.diff_pct) for r in legs)
        avg_dist_l = sum(abs(r.dist_l_m) for r in legs) / len(legs)
        avg_dist_r = sum(abs(r.dist_r_m) for r in legs) / len(legs)

        clr = GRN if abs(avg_diff_pct) < 0.5 else (YLW if abs(avg_diff_pct) < 1.5 else RED)
        print(f"\n  {direction} ({len(legs)} legs):")
        print(f"    Avg enc L: {avg_dist_l:.5f} m   Avg enc R: {avg_dist_r:.5f} m")
        print(f"    Avg L−R : {clr}{avg_diff_m:+.5f} m  ({avg_diff_pct:+.2f}%){NC}")
        print(f"    Avg L−R counts: {avg_diff_counts:+.1f}")
        print(f"    Max |diff|: {max_diff_pct:.2f}%")

    # Overall
    if results:
        overall_pct = sum(r.diff_pct for r in results) / len(results)
        clr = GRN if abs(overall_pct) < 0.5 else (YLW if abs(overall_pct) < 1.5 else RED)
        print(f"\n  Overall avg L−R: {clr}{overall_pct:+.2f}%{NC}")


# ── main ───────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Encoder L/R symmetry test")
    ap.add_argument("--port", default="/dev/ttyTB3")
    ap.add_argument("--speeds", type=float, nargs="+", default=DEFAULT_SPEEDS,
                     help="Speed(s) in m/s (default: 0.10)")
    ap.add_argument("--distance", type=float, default=DEFAULT_DISTANCE,
                     help="Distance per leg in metres (default: 0.50)")
    ap.add_argument("--repeats", type=int, default=DEFAULT_REPEATS,
                     help="Repeats per speed/direction (default: 3)")
    args = ap.parse_args()

    stop_bringup()
    results: list[LegData] = []

    try:
        ser = serial.Serial(args.port, baudrate=1000000, timeout=0.2)
        time.sleep(0.5)

        # Ping
        keep_alive(ser)
        print(f"{GRN}Ping OK{NC}")

        for spd in args.speeds:
            print(f"\n{'═'*60}")
            print(f"  Speed: {spd:.2f} m/s,  Distance: {args.distance:.2f} m,  Repeats: {args.repeats}")
            print(f"{'═'*60}")

            for rep in range(1, args.repeats + 1):
                print(f"\n── Rep {rep}/{args.repeats}  FWD …")
                leg_fwd = run_leg(ser, +spd, args.distance)
                leg_fwd.repeat = rep
                results.append(leg_fwd)
                print(f"   L={leg_fwd.enc_l_counts:+d}  R={leg_fwd.enc_r_counts:+d}  "
                      f"diff={leg_fwd.diff_counts:+d} ({leg_fwd.diff_pct:+.2f}%)")

                time.sleep(1.0)

                print(f"── Rep {rep}/{args.repeats}  REV …")
                leg_rev = run_leg(ser, -spd, args.distance)
                leg_rev.repeat = rep
                results.append(leg_rev)
                print(f"   L={leg_rev.enc_l_counts:+d}  R={leg_rev.enc_r_counts:+d}  "
                      f"diff={leg_rev.diff_counts:+d} ({leg_rev.diff_pct:+.2f}%)")

                time.sleep(1.0)

        ser.close()

    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as exc:
        print(f"{RED}Error: {exc}{NC}")
        import traceback; traceback.print_exc()
    finally:
        restart_bringup()

    # ── Results ──
    if results:
        print_table(results)
        print_summary(results)


if __name__ == "__main__":
    main()
