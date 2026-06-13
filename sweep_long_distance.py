#!/usr/bin/env python3
"""
sweep_long_distance.py — Long-distance (~2 m) drift sweep of heading-hold parameters.

Tests three approaches to reduce long-run forward drift:
  1. Enable a small KI (integral gain) so the controller adapts in real-time
  2. Bump I_SEED_FWD (fixed integral preload)
  3. Increase VEL_TRIM_FWD (feedforward velocity bias)

Each candidate drives forward for --duration seconds at --speed m/s, then
reports heading drift from the encoder difference.

Must be run with bringup stopped (takes exclusive serial access).
"""
import math
import subprocess
import sys
import time

import serial

from dxl_utils import *  # noqa: F401,F403 — protocol, registers, colours

# Module-level wheel separation — set at startup from firmware
WHEEL_SEP_M = 0.185  # fallback default

def colour_drift(deg):
    a = abs(deg)
    if a < 0.5:
        return GRN
    elif a < 1.0:
        return YLW
    return RED

# ══════════════════════════════════════════════════════════════════════════════
def run_pass(ser, speed, duration):
    """Drive forward and return drift stats. Caller sets registers beforehand."""
    # Enable torque and start driving
    dxl_write(ser, ADDR_TORQUE_EN, bytes([1]))
    time.sleep(0.05)
    set_velocity(ser, speed)

    # Collect samples
    samples = []
    t0 = time.time()
    while time.time() - t0 < duration:
        keep_alive(ser)
        hd_err   = dxl_read_f32(ser, ADDR_DBG_HEADING_ERR) or 0.0
        hd_corr  = dxl_read_f32(ser, ADDR_HEADING_HOLD_CORR) or 0.0
        vel_l    = dxl_read_f32(ser, ADDR_DBG_VEL_L) or 0.0
        vel_r    = dxl_read_f32(ser, ADDR_DBG_VEL_R) or 0.0
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

    # Disable torque
    dxl_write(ser, ADDR_TORQUE_EN, bytes([0]))
    time.sleep(2.0)  # let robot settle, heading hold resets

    if not samples:
        return dict(drift_deg=999.0, mean_err_deg=999.0, peak_err_deg=999.0,
                    mean_corr=0.0, n_samples=0, dist_m=0.0)

    # Stats from middle 60% of samples (steady-state)
    n = len(samples)
    lo = n // 5
    hi = n - lo
    mid = samples[lo:hi] if n > 5 else samples
    mean_err  = sum(s["hd_err"]  for s in mid) / max(len(mid), 1)
    mean_corr = sum(s["hd_corr"] for s in mid) / max(len(mid), 1)
    peak_err  = max(abs(s["hd_err"]) for s in mid) if mid else 0.0

    # Final enc_diff → heading drift in degrees
    final_enc_diff = samples[-1]["enc_diff"]
    drift_deg = math.degrees(final_enc_diff / WHEEL_SEP_M)

    # Estimate distance from velocity
    vels = [(abs(s["vel_l"]) + abs(s["vel_r"])) / 2.0 for s in samples[1:]]
    dt = samples[-1]["t"] - samples[0]["t"] if len(samples) > 1 else 0
    dist_m = sum(vels) / len(vels) * dt if vels else 0.0

    return dict(
        drift_deg=drift_deg,
        mean_err_deg=math.degrees(mean_err),
        peak_err_deg=math.degrees(peak_err),
        mean_corr=mean_corr,
        n_samples=n,
        dist_m=dist_m,
    )


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Long-distance drift sweep: KI, I_SEED, VEL_TRIM")
    ap.add_argument("--port", default="/dev/ttyTB3")
    ap.add_argument("--speed", type=float, default=0.10)
    ap.add_argument("--duration", type=float, default=20.0,
                    help="Drive time per pass (20s @ 0.10 = ~2 m)")
    ap.add_argument("--sweep", choices=["ki", "iseed", "vtrim", "all"], default="all",
                    help="Which parameter to sweep (default: all)")
    args = ap.parse_args()

    print(f"\n{BLD}sweep_long_distance.py{NC} — Long-distance drift parameter sweep")
    print(f"  port={args.port}  speed={args.speed:.2f} m/s  duration={args.duration:.1f} s")
    print(f"  sweep={args.sweep}\n")

    # Stop bringup for exclusive serial access (only if running)
    _bringup_was_active = False
    try:
        _bringup_was_active = subprocess.run(
            ["systemctl", "is-active", "--quiet", "turtlebot3-bringup.service"],
            timeout=5).returncode == 0
    except Exception:
        pass
    if _bringup_was_active:
        print("Stopping turtlebot3-bringup.service ...")
        subprocess.run(["sudo", "systemctl", "stop", "turtlebot3-bringup.service"],
                       capture_output=True, timeout=10)
        time.sleep(1.0)
    else:
        print("turtlebot3-bringup.service already stopped.")

    ser = serial.Serial(args.port, BAUD, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Verify comms
    millis = dxl_read_i32(ser, ADDR_MILLIS)
    if millis is None:
        print(f"{RED}Cannot reach Pico — check port.{NC}")
        ser.close()
        return 1
    print(f"Pico uptime: {millis} ms")

    # Query wheel separation from firmware
    global WHEEL_SEP_M
    ws = query_wheel_separation(ser)
    if ws and ws > 0.01:
        WHEEL_SEP_M = ws
    else:
        print(f"{YLW}Warning: could not read wheel separation, using default {WHEEL_SEP_M:.4f} m{NC}")
    print(f"  Wheel separation: {WHEEL_SEP_M:.6f} m")

    # Read current (baseline) register values
    base_ki     = dxl_read_f32(ser, ADDR_HEADING_HOLD_KI) or 0.0
    base_iseed  = dxl_read_f32(ser, ADDR_HEADING_HOLD_I_SEED_FWD) or 0.075
    base_vtrim  = dxl_read_f32(ser, ADDR_VEL_TRIM_FWD) or 0.014
    base_kp     = dxl_read_f32(ser, ADDR_HEADING_HOLD_KP) or 1.0
    print(f"Baseline:  KI={base_ki:.4f}  I_SEED={base_iseed:.6f}  "
          f"VEL_TRIM={base_vtrim:.6f}  KP={base_kp:.2f}\n")

    def restore_baseline():
        write_f32(ser, ADDR_HEADING_HOLD_KI, base_ki)
        write_f32(ser, ADDR_HEADING_HOLD_I_SEED_FWD, base_iseed)
        write_f32(ser, ADDR_VEL_TRIM_FWD, base_vtrim)
        time.sleep(0.1)

    all_results = []  # (label, params_str, result)

    # ── baseline ──────────────────────────────────────────────────────────────
    print(f"{BLD}{'='*60}")
    print(f"  BASELINE  (KI={base_ki}, I_SEED={base_iseed:.4f}, VEL_TRIM={base_vtrim:.4f})")
    print(f"{'='*60}{NC}")
    restore_baseline()
    r = run_pass(ser, args.speed, args.duration)
    c = colour_drift(r["drift_deg"])
    print(f"  drift={c}{r['drift_deg']:+.2f}°{NC}  dist={r['dist_m']:.2f}m  "
          f"mean_err={r['mean_err_deg']:+.2f}°  peak_err={r['peak_err_deg']:.2f}°")
    all_results.append(("BASELINE", f"KI=0, I_SEED={base_iseed:.4f}, VTRIM={base_vtrim:.4f}", r))

    # ── Option 1: Sweep KI ────────────────────────────────────────────────────
    if args.sweep in ("ki", "all"):
        ki_candidates = [0.02, 0.05, 0.08, 0.10]
        print(f"\n{BLD}{'='*60}")
        print(f"  OPTION 1: Sweep KI  (candidates: {ki_candidates})")
        print(f"{'='*60}{NC}")
        for ki_val in ki_candidates:
            restore_baseline()
            write_f32(ser, ADDR_HEADING_HOLD_KI, ki_val)
            time.sleep(0.1)
            label = f"KI={ki_val:.2f}"
            print(f"  {label:12s} ... ", end="", flush=True)
            r = run_pass(ser, args.speed, args.duration)
            c = colour_drift(r["drift_deg"])
            print(f"drift={c}{r['drift_deg']:+.2f}°{NC}  dist={r['dist_m']:.2f}m  "
                  f"mean_err={r['mean_err_deg']:+.2f}°  peak={r['peak_err_deg']:.2f}°")
            all_results.append((f"KI={ki_val:.2f}",
                                f"KI={ki_val}, I_SEED={base_iseed:.4f}, VTRIM={base_vtrim:.4f}", r))

    # ── Option 2: Sweep I_SEED_FWD ────────────────────────────────────────────
    if args.sweep in ("iseed", "all"):
        iseed_candidates = [0.077, 0.078, 0.080, 0.082]
        print(f"\n{BLD}{'='*60}")
        print(f"  OPTION 2: Sweep I_SEED_FWD  (candidates: {iseed_candidates})")
        print(f"{'='*60}{NC}")
        for iseed_val in iseed_candidates:
            restore_baseline()
            write_f32(ser, ADDR_HEADING_HOLD_I_SEED_FWD, iseed_val)
            time.sleep(0.1)
            label = f"ISEED={iseed_val:.3f}"
            print(f"  {label:12s} ... ", end="", flush=True)
            r = run_pass(ser, args.speed, args.duration)
            c = colour_drift(r["drift_deg"])
            print(f"drift={c}{r['drift_deg']:+.2f}°{NC}  dist={r['dist_m']:.2f}m  "
                  f"mean_err={r['mean_err_deg']:+.2f}°  peak={r['peak_err_deg']:.2f}°")
            all_results.append((f"ISEED={iseed_val:.3f}",
                                f"KI=0, I_SEED={iseed_val:.4f}, VTRIM={base_vtrim:.4f}", r))

    # ── Option 3: Sweep VEL_TRIM_FWD ─────────────────────────────────────────
    if args.sweep in ("vtrim", "all"):
        vtrim_candidates = [0.015, 0.016, 0.017, 0.018]
        print(f"\n{BLD}{'='*60}")
        print(f"  OPTION 3: Sweep VEL_TRIM_FWD  (candidates: {vtrim_candidates})")
        print(f"{'='*60}{NC}")
        for vtrim_val in vtrim_candidates:
            restore_baseline()
            write_f32(ser, ADDR_VEL_TRIM_FWD, vtrim_val)
            time.sleep(0.1)
            label = f"VTRIM={vtrim_val:.3f}"
            print(f"  {label:12s} ... ", end="", flush=True)
            r = run_pass(ser, args.speed, args.duration)
            c = colour_drift(r["drift_deg"])
            print(f"drift={c}{r['drift_deg']:+.2f}°{NC}  dist={r['dist_m']:.2f}m  "
                  f"mean_err={r['mean_err_deg']:+.2f}°  peak={r['peak_err_deg']:.2f}°")
            all_results.append((f"VTRIM={vtrim_val:.3f}",
                                f"KI=0, I_SEED={base_iseed:.4f}, VTRIM={vtrim_val:.4f}", r))

    # ── Restore baseline and restart bringup ──────────────────────────────────
    restore_baseline()
    ser.close()

    print(f"\n{BLD}{'='*60}")
    print(f"  SUMMARY — sorted by |drift|")
    print(f"{'='*60}{NC}")
    print(f"  {'Label':14s}  {'Drift':>8s}  {'Dist':>6s}  {'MeanErr':>8s}  {'PeakErr':>8s}  {'MeanCorr':>9s}")
    print(f"  {'-'*64}")

    ranked = sorted(all_results, key=lambda x: abs(x[2]["drift_deg"]))
    for label, params, r in ranked:
        c = colour_drift(r["drift_deg"])
        marker = " ◀ BEST" if (label, params, r) is ranked[0] else ""
        print(f"  {label:14s}  {c}{r['drift_deg']:+7.2f}°{NC}  "
              f"{r['dist_m']:5.2f}m  {r['mean_err_deg']:+7.2f}°  "
              f"{r['peak_err_deg']:7.2f}°  {r['mean_corr']:+8.4f}{marker}")

    best_label, best_params, best_r = ranked[0]
    print(f"\n{BLD}Best:{NC} {best_label} — drift={best_r['drift_deg']:+.2f}° over {best_r['dist_m']:.2f}m")
    print(f"  Parameters: {best_params}")

    # Restart bringup (only if we stopped it)
    if _bringup_was_active:
        print("\nRestarting turtlebot3-bringup.service ...")
        try:
            subprocess.run(["sudo", "systemctl", "start", "turtlebot3-bringup.service"],
                           capture_output=True, timeout=10)
        except subprocess.TimeoutExpired:
            print(f"{YLW}Timeout restarting service — run manually: sudo systemctl start turtlebot3-bringup.service{NC}")
        print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
