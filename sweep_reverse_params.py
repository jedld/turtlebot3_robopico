#!/usr/bin/env python3
"""
sweep_reverse_params.py — Runtime sweep of reverse heading-hold parameters.

Sweeps I_SEED_REV, VEL_TRIM_REV, or both using alternating fwd/rev 1m legs
to find optimal values that minimize reverse drift while preserving forward.

Must be run with bringup stopped.
"""
import math
import subprocess
import sys
import time

import serial

from dxl_utils import *  # noqa: F401,F403 — protocol, registers, colours

def colour_drift(deg):
    a = abs(deg)
    return GRN if a < 0.3 else (YLW if a < 0.7 else RED)


def run_leg(ser, speed, target_dist_m):
    """Drive at speed until target_dist_m reached. Returns drift_deg."""
    timeout_s = (target_dist_m / abs(speed)) * 2.0 + 5.0

    dxl_write(ser, ADDR_TORQUE_EN, bytes([1]))
    time.sleep(0.05)

    enc_l0 = dxl_read_i32(ser, ADDR_ENC_L) or 0
    enc_r0 = dxl_read_i32(ser, ADDR_ENC_R) or 0

    set_velocity(ser, speed)

    samples = []
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        hd_err = dxl_read_f32(ser, ADDR_DBG_HEADING_ERR) or 0.0
        vel_l = dxl_read_f32(ser, ADDR_DBG_VEL_L) or 0.0
        vel_r = dxl_read_f32(ser, ADDR_DBG_VEL_R) or 0.0
        enc_l = dxl_read_i32(ser, ADDR_ENC_L) or enc_l0
        enc_r = dxl_read_i32(ser, ADDR_ENC_R) or enc_r0

        dl = (enc_l - enc_l0) * ENC_M_PER_COUNT
        dr = (enc_r - enc_r0) * ENC_M_PER_COUNT
        avg_dist = (abs(dl) + abs(dr)) / 2.0

        samples.append(dict(
            t=time.time()-t0, hd_err=hd_err,
            vel_l=vel_l, vel_r=vel_r,
            dl=dl, dr=dr, avg_dist=avg_dist,
        ))

        if avg_dist >= target_dist_m:
            break
        time.sleep(0.06)

    set_velocity(ser, 0.0)
    time.sleep(0.3)
    dxl_write(ser, ADDR_TORQUE_EN, bytes([0]))
    time.sleep(1.5)

    if not samples:
        return dict(drift_deg=999.0, dist_m=0.0, mean_err_deg=0.0, peak_err_deg=0.0)

    last = samples[-1]
    enc_diff_m = last["dl"] - last["dr"]
    drift_deg = math.degrees(enc_diff_m / WHEEL_BASE_M)
    dist_m = last["avg_dist"]

    n = len(samples)
    lo = max(1, n // 5)
    hi = n - lo
    mid = samples[lo:hi] if n > 5 else samples[1:]
    mean_err = sum(s["hd_err"] for s in mid) / max(len(mid), 1) if mid else 0.0
    peak_err = max(abs(s["hd_err"]) for s in mid) if mid else 0.0

    return dict(
        drift_deg=drift_deg, dist_m=dist_m,
        mean_err_deg=math.degrees(mean_err),
        peak_err_deg=math.degrees(peak_err),
    )


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Sweep reverse heading-hold parameters")
    ap.add_argument("--port", default="/dev/ttyTB3")
    ap.add_argument("--speed", type=float, default=0.10,
                    help="Test speed (m/s)")
    ap.add_argument("--distance", type=float, default=1.0)
    ap.add_argument("--sweep", choices=["iseed_rev", "vtrim_rev", "both"], default="both")
    args = ap.parse_args()

    print(f"\n{BLD}sweep_reverse_params.py{NC} — Reverse parameter sweep")
    print(f"  speed={args.speed:.2f} m/s  distance={args.distance:.1f}m  sweep={args.sweep}\n")

    # Stop bringup (only if running)
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

    millis = dxl_read_i32(ser, ADDR_MILLIS)
    if millis is None:
        print(f"{RED}Cannot reach Pico.{NC}")
        ser.close()
        return 1
    print(f"Pico uptime: {millis} ms")

    # Query wheel separation from firmware
    global WHEEL_BASE_M
    ws = query_wheel_separation(ser)
    if ws and ws > 0.01:
        WHEEL_BASE_M = ws
    else:
        WHEEL_BASE_M = 0.185
        print(f"{YLW}Warning: could not read wheel separation, using default {WHEEL_BASE_M:.4f} m{NC}")
    print(f"  Wheel separation: {WHEEL_BASE_M:.6f} m")

    # Read baseline
    base_iseed_fwd = dxl_read_f32(ser, ADDR_I_SEED_FWD) or 0.075
    base_iseed_rev = dxl_read_f32(ser, ADDR_I_SEED_REV) or 0.0078
    base_vtrim_fwd = dxl_read_f32(ser, ADDR_VEL_TRIM_FWD) or 0.014
    base_vtrim_rev = dxl_read_f32(ser, ADDR_VEL_TRIM_REV) or 0.0
    base_ki = dxl_read_f32(ser, ADDR_HEADING_HOLD_KI) or 0.08
    base_kp = dxl_read_f32(ser, ADDR_HEADING_HOLD_KP) or 1.0

    print(f"  KP={base_kp:.2f}  KI={base_ki:.4f}")
    print(f"  I_SEED: fwd={base_iseed_fwd:.6f} rev={base_iseed_rev:.6f}")
    print(f"  VEL_TRIM: fwd={base_vtrim_fwd:.6f} rev={base_vtrim_rev:.6f}\n")

    def restore():
        write_f32(ser, ADDR_I_SEED_REV, base_iseed_rev)
        write_f32(ser, ADDR_VEL_TRIM_REV, base_vtrim_rev)
        write_f32(ser, ADDR_I_SEED_FWD, base_iseed_fwd)
        write_f32(ser, ADDR_VEL_TRIM_FWD, base_vtrim_fwd)
        time.sleep(0.1)

    all_results = []  # (label, fwd_r, rev_r)

    def run_pair(label, settle=2.0):
        print(f"  {label:22s} ", end="", flush=True)
        # FWD leg
        fwd = run_leg(ser, +args.speed, args.distance)
        time.sleep(settle)
        # REV leg
        rev = run_leg(ser, -args.speed, args.distance)
        time.sleep(settle)

        cf = colour_drift(fwd["drift_deg"])
        cr = colour_drift(rev["drift_deg"])
        net = fwd["drift_deg"] + rev["drift_deg"]
        cn = colour_drift(net)
        print(f"FWD={cf}{fwd['drift_deg']:+.2f}°{NC}  "
              f"REV={cr}{rev['drift_deg']:+.2f}°{NC}  "
              f"net={cn}{net:+.2f}°{NC}  "
              f"peak_fwd={fwd['peak_err_deg']:.2f}°  peak_rev={rev['peak_err_deg']:.2f}°")
        all_results.append((label, fwd, rev))

    # ── Baseline ──────────────────────────────────────────────────────────
    print(f"{BLD}{'='*70}")
    print(f"  BASELINE")
    print(f"{'='*70}{NC}")
    restore()
    run_pair("BASELINE")

    # ── Sweep I_SEED_REV ──────────────────────────────────────────────────
    if args.sweep in ("iseed_rev", "both"):
        # Fine sweep around baseline 0.0078 and also try zero/negative.
        candidates = [0.000, 0.004, 0.010, 0.015, 0.020]
        print(f"\n{BLD}{'='*70}")
        print(f"  SWEEP I_SEED_REV  (candidates: {candidates})")
        print(f"{'='*70}{NC}")
        for val in candidates:
            restore()
            write_f32(ser, ADDR_I_SEED_REV, val)
            time.sleep(0.1)
            run_pair(f"ISEED_REV={val:.3f}")

    # ── Sweep VEL_TRIM_REV ───────────────────────────────────────────────
    if args.sweep in ("vtrim_rev", "both"):
        # Fine sweep: 0.005 overcorrected last time → try 0.001–0.004.
        candidates = [0.001, 0.002, 0.003, 0.004]
        print(f"\n{BLD}{'='*70}")
        print(f"  SWEEP VEL_TRIM_REV  (candidates: {candidates})")
        print(f"{'='*70}{NC}")
        for val in candidates:
            restore()
            write_f32(ser, ADDR_VEL_TRIM_REV, val)
            time.sleep(0.1)
            run_pair(f"VTRIM_REV={val:.3f}")

    # ── Restore and close ─────────────────────────────────────────────────
    restore()
    ser.close()

    # ── Summary ───────────────────────────────────────────────────────────
    print(f"\n{BLD}{'='*70}")
    print(f"  SUMMARY — sorted by |net drift|")
    print(f"{'='*70}{NC}")
    print(f"  {'Label':22s}  {'FWD':>8s}  {'REV':>8s}  {'Net':>8s}  "
          f"{'|F|+|R|':>8s}  {'PkF':>6s}  {'PkR':>6s}")
    print(f"  {'-'*68}")

    ranked = sorted(all_results, key=lambda x: abs(x[1]["drift_deg"] + x[2]["drift_deg"]))
    for label, fwd, rev in ranked:
        net = fwd["drift_deg"] + rev["drift_deg"]
        total = abs(fwd["drift_deg"]) + abs(rev["drift_deg"])
        cf = colour_drift(fwd["drift_deg"])
        cr = colour_drift(rev["drift_deg"])
        cn = colour_drift(net)
        ct = colour_drift(total / 2.0)
        marker = " ◀ BEST" if (label, fwd, rev) is ranked[0] else ""
        print(f"  {label:22s}  "
              f"{cf}{fwd['drift_deg']:+7.2f}°{NC}  "
              f"{cr}{rev['drift_deg']:+7.2f}°{NC}  "
              f"{cn}{net:+7.2f}°{NC}  "
              f"{ct}{total:7.2f}°{NC}  "
              f"{fwd['peak_err_deg']:5.2f}°  "
              f"{rev['peak_err_deg']:5.2f}°{marker}")

    best_label, best_fwd, best_rev = ranked[0]
    best_net = best_fwd["drift_deg"] + best_rev["drift_deg"]
    print(f"\n{BLD}Best:{NC} {best_label} — net drift={best_net:+.2f}°")

    # Also find best by minimum total |FWD|+|REV|
    best_total = min(all_results, key=lambda x: abs(x[1]["drift_deg"]) + abs(x[2]["drift_deg"]))
    if best_total is not ranked[0]:
        bt_net = best_total[1]["drift_deg"] + best_total[2]["drift_deg"]
        bt_sum = abs(best_total[1]["drift_deg"]) + abs(best_total[2]["drift_deg"])
        print(f"  Best total |FWD|+|REV|: {best_total[0]} — {bt_sum:.2f}° (net={bt_net:+.2f}°)")

    # Restart (only if we stopped it)
    if _bringup_was_active:
        print(f"\nRestarting turtlebot3-bringup.service ...")
        try:
            subprocess.run(["sudo", "systemctl", "start", "turtlebot3-bringup.service"],
                           capture_output=True, timeout=10)
        except subprocess.TimeoutExpired:
            print(f"{YLW}Timeout restarting service — run manually: sudo systemctl start turtlebot3-bringup.service{NC}")
        print("Done.\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
