#!/usr/bin/env python3
"""
sweep_iseed_fwd.py — Runtime sweep of HEADING_HOLD_I_SEED_FWD register.

Drives the robot forward at low speed for several seconds at each candidate
seed value, reads the final heading error, and reports the best candidate.

Must be run with bringup stopped (takes exclusive serial access).
"""
import math
import sys
import time

import serial

from dxl_utils import *  # noqa: F401,F403 — protocol, registers, colours

# Module-level wheel separation — set at startup from firmware
WHEEL_SEP_M = 0.185  # fallback default

# ══════════════════════════════════════════════════════════════════════════════
def run_pass(ser, seed_val: float, speed: float, duration: float) -> dict:
    """Drive forward with a given seed value and return drift stats."""
    # Write the seed
    write_f32(ser, ADDR_HEADING_HOLD_I_SEED_FWD, seed_val)
    time.sleep(0.1)

    # Confirm it took
    actual = dxl_read_f32(ser, ADDR_HEADING_HOLD_I_SEED_FWD)

    # Enable torque and start driving
    dxl_write(ser, ADDR_TORQUE_EN, bytes([1]))
    time.sleep(0.05)
    set_velocity(ser, speed)

    # Collect samples
    samples = []
    t0 = time.time()
    while time.time() - t0 < duration:
        keep_alive(ser)
        hd_err  = dxl_read_f32(ser, ADDR_DBG_HEADING_ERR) or 0.0
        hd_corr = dxl_read_f32(ser, ADDR_HEADING_HOLD_CORR) or 0.0
        vel_l   = dxl_read_f32(ser, ADDR_DBG_VEL_L) or 0.0
        vel_r   = dxl_read_f32(ser, ADDR_DBG_VEL_R) or 0.0
        enc_diff = dxl_read_f32(ser, ADDR_DBG_ENC_DIFF) or 0.0
        samples.append(dict(
            t=time.time() - t0,
            hd_err=hd_err, hd_corr=hd_corr,
            vel_l=vel_l, vel_r=vel_r, enc_diff=enc_diff,
        ))
        time.sleep(0.08)

    # Stop
    set_velocity(ser, 0.0)
    time.sleep(0.3)

    # Read final heading error before it resets
    final_err = dxl_read_f32(ser, ADDR_DBG_HEADING_ERR) or 0.0

    # Disable torque
    dxl_write(ser, ADDR_TORQUE_EN, bytes([0]))
    time.sleep(1.5)  # let robot settle, heading hold resets

    # Stats from last 60% of samples (steady-state)
    n = len(samples)
    trim = n // 5
    mid = samples[trim:-trim] if n > 5 else samples
    mean_err  = sum(s["hd_err"]  for s in mid) / max(len(mid), 1)
    mean_corr = sum(s["hd_corr"] for s in mid) / max(len(mid), 1)
    peak_err  = max(abs(s["hd_err"]) for s in mid) if mid else 0.0

    # Final enc_diff → heading drift in degrees
    final_enc_diff = samples[-1]["enc_diff"] if samples else 0.0
    drift_deg = math.degrees(final_enc_diff / WHEEL_SEP_M)

    return dict(
        seed=seed_val, seed_actual=actual,
        mean_err_deg=math.degrees(mean_err),
        peak_err_deg=math.degrees(peak_err),
        mean_corr=mean_corr,
        drift_deg=drift_deg,
        final_enc_diff=final_enc_diff,
        n_samples=n,
    )


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Sweep HEADING_HOLD_I_SEED_FWD")
    ap.add_argument("--port", default="/dev/ttyTB3")
    ap.add_argument("--speed", type=float, default=0.10)
    ap.add_argument("--duration", type=float, default=4.0)
    ap.add_argument("--seeds", nargs="+", type=float,
                    default=[0.090, 0.100, 0.110, 0.115, 0.120])
    args = ap.parse_args()

    print(f"{BLD}sweep_iseed_fwd.py{NC} — I_SEED_FWD runtime sweep")
    print(f"  port={args.port}  speed={args.speed:.2f} m/s  duration={args.duration:.1f} s")
    print(f"  candidates: {args.seeds}\n")

    ser = serial.Serial(args.port, BAUD, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Verify comms
    millis = dxl_read_i32(ser, ADDR_MILLIS)
    if millis is None:
        print(f"{RED}Cannot reach Pico — check port.{NC}")
        ser.close()
        return 1
    print(f"Pico uptime: {millis} ms\n")

    # Query wheel separation from firmware
    global WHEEL_SEP_M
    ws = query_wheel_separation(ser)
    if ws and ws > 0.01:
        WHEEL_SEP_M = ws
    else:
        print(f"{YLW}Warning: could not read wheel separation, using default {WHEEL_SEP_M:.4f} m{NC}")
    print(f"  Wheel separation: {WHEEL_SEP_M:.6f} m")

    # Read current settings
    current_seed = dxl_read_f32(ser, ADDR_HEADING_HOLD_I_SEED_FWD)
    kp = dxl_read_f32(ser, ADDR_HEADING_HOLD_KP)
    print(f"Current I_SEED_FWD = {current_seed:.6f}   HH_KP = {kp:.2f}\n")

    results = []
    for i, seed in enumerate(args.seeds):
        print(f"{BLD}Pass {i+1}/{len(args.seeds)}:{NC} seed = {seed:.4f} ... ", end="", flush=True)
        r = run_pass(ser, seed, args.speed, args.duration)
        col = GRN if abs(r["drift_deg"]) < 0.5 else (YLW if abs(r["drift_deg"]) < 1.5 else RED)
        print(f"drift={col}{r['drift_deg']:+.2f}°{NC}  "
              f"mean_err={r['mean_err_deg']:+.2f}°  "
              f"peak_err={r['peak_err_deg']:.2f}°  "
              f"corr={r['mean_corr']:+.4f}")
        results.append(r)

    # Restore original seed
    if current_seed is not None:
        write_f32(ser, ADDR_HEADING_HOLD_I_SEED_FWD, current_seed)
        print(f"\nRestored I_SEED_FWD to {current_seed:.6f}")

    ser.close()

    # Summary
    print(f"\n{BLD}{'seed':>8s}  {'drift':>8s}  {'mean_err':>9s}  {'peak_err':>9s}  {'correction':>11s}{NC}")
    print("-" * 55)
    best = min(results, key=lambda r: abs(r["drift_deg"]))
    for r in results:
        marker = " ◀ best" if r is best else ""
        col = GRN if abs(r["drift_deg"]) < 0.5 else (YLW if abs(r["drift_deg"]) < 1.5 else RED)
        print(f"{r['seed']:8.4f}  {col}{r['drift_deg']:+7.2f}°{NC}  "
              f"{r['mean_err_deg']:+8.2f}°  {r['peak_err_deg']:8.2f}°  "
              f"{r['mean_corr']:+10.4f}{marker}")

    print(f"\n{BLD}Recommendation:{NC} set HEADING_HOLD_I_SEED_FWD_DEFAULT to {best['seed']:.6f}f")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
