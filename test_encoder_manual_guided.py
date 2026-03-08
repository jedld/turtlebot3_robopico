#!/usr/bin/env python3
"""
Guided manual encoder test for TurtleBot3 Pico firmware.

This test does NOT drive the motors.
It asks the user to spin each wheel by hand in the physical forward and reverse
 directions while reading encoder counts and firmware-reported signed velocities.

Use this when motor-direction testing is already visually correct but encoder
 sign / wheel mapping is still suspicious.
"""

from __future__ import annotations

import argparse
import os
import struct
import time
from dataclasses import dataclass

import serial

PORT_CANDIDATES = ("/dev/ttyTB3", "/dev/ttyACM0", "/dev/ttyACM1")
BAUD = 1_000_000
DEV_ID = 200

ADDR_ENC_L_COUNT = 184
ADDR_ENC_R_COUNT = 188
ADDR_ENC_RESET = 192
ADDR_DBG_VEL_L = 212
ADDR_DBG_VEL_R = 216

EXPECTED_TICKS_PER_REV = 3840  # FIT0450: 8 PPR × 4 (X4 quad) × 120:1 gearbox

SETTLE_S = 0.40
DISPLAY_DT_S = 0.08

CRC_TABLE = []
for i in range(256):
    c = i << 8
    for _ in range(8):
        c = (((c << 1) ^ 0x8005) & 0xFFFF) if (c & 0x8000) else ((c << 1) & 0xFFFF)
    CRC_TABLE.append(c)


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


def packet(instr: int, params: bytes) -> bytes:
    length = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, DEV_ID, length & 0xFF, (length >> 8) & 0xFF, instr]) + params
    crc = crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])


def read_resp(ser: serial.Serial, timeout: float = 0.25):
    t0 = time.time()
    buf = b""
    while time.time() - t0 < timeout:
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except serial.SerialException:
            # USB-CDC glitch — brief disconnect; wait and retry
            time.sleep(0.02)
            continue
        if chunk:
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


def dxl_read(ser: serial.Serial, addr: int, length: int):
    try:
        ser.reset_input_buffer()
        ser.write(packet(0x02, struct.pack("<HH", addr, length)))
    except serial.SerialException:
        return None
    resp = read_resp(ser)
    if resp and len(resp) >= 9 + length:
        return resp[9:9 + length]
    return None


def dxl_write(ser: serial.Serial, addr: int, data: bytes):
    try:
        ser.reset_input_buffer()
        ser.write(packet(0x03, struct.pack("<H", addr) + data))
    except serial.SerialException:
        return False
    return read_resp(ser) is not None


def detect_port(explicit: str | None):
    if explicit:
        return explicit
    for p in PORT_CANDIDATES:
        if os.path.exists(p):
            return p
    return PORT_CANDIDATES[0]


def read_sample(ser: serial.Serial, retries: int = 3):
    for attempt in range(retries):
        try:
            counts = dxl_read(ser, ADDR_ENC_L_COUNT, 8)
            vels = dxl_read(ser, ADDR_DBG_VEL_L, 8)
        except serial.SerialException:
            time.sleep(0.05)
            continue
        if not counts or not vels or len(counts) < 8 or len(vels) < 8:
            time.sleep(0.03)
            continue
        l = struct.unpack_from("<i", counts, 0)[0]
        r = struct.unpack_from("<i", counts, 4)[0]
        vl = struct.unpack_from("<f", vels, 0)[0]
        vr = struct.unpack_from("<f", vels, 4)[0]
        return l, r, vl, vr
    return None


def reset_encoders(ser: serial.Serial):
    dxl_write(ser, ADDR_ENC_RESET, bytes([1]))
    time.sleep(0.05)


@dataclass
class CaptureResult:
    delta_l: int
    delta_r: int
    peak_vl: float
    peak_vr: float
    sum_vl: float
    sum_vr: float


def average_baseline(ser: serial.Serial, samples: int = 4):
    vals = []
    for _ in range(samples):
        s = read_sample(ser)
        if s is not None:
            vals.append(s)
        time.sleep(0.03)
    if not vals:
        return None
    l = int(round(sum(v[0] for v in vals) / len(vals)))
    r = int(round(sum(v[1] for v in vals) / len(vals)))
    vl = sum(v[2] for v in vals) / len(vals)
    vr = sum(v[3] for v in vals) / len(vals)
    return l, r, vl, vr


def capture_motion(ser: serial.Serial, duration: float):
    print(f"  Hold the wheel still for {SETTLE_S:.1f} s to settle...", flush=True)
    time.sleep(SETTLE_S)

    start = average_baseline(ser)
    if start is None:
        raise RuntimeError("failed to read encoder baseline")
    l0, r0, _, _ = start
    peak_vl = 0.0
    peak_vr = 0.0
    sum_vl = 0.0
    sum_vr = 0.0
    nsamp = 0
    t_end = time.time() + duration

    print(f"  {'elapsed':>7s}  {'ΔL':>8s}  {'ΔR':>8s}  {'velL':>8s}  {'velR':>8s}")
    print("  " + "-" * 50)
    while time.time() < t_end:
        s = read_sample(ser)
        if s is not None:
            l, r, vl, vr = s
            if abs(vl) >= abs(peak_vl):
                peak_vl = vl
            if abs(vr) >= abs(peak_vr):
                peak_vr = vr
            sum_vl += vl
            sum_vr += vr
            nsamp += 1
            elapsed = duration - (t_end - time.time())
            print(f"\r  {elapsed:7.2f}  {l-l0:+8d}  {r-r0:+8d}  {vl:+8.3f}  {vr:+8.3f}", end="", flush=True)
        time.sleep(DISPLAY_DT_S)
    print()
    end = average_baseline(ser)
    if end is None:
        raise RuntimeError("failed to read encoder final sample")
    l1, r1, _, _ = end
    return CaptureResult(
        delta_l=l1 - l0,
        delta_r=r1 - r0,
        peak_vl=peak_vl,
        peak_vr=peak_vr,
        sum_vl=sum_vl / max(nsamp, 1),
        sum_vr=sum_vr / max(nsamp, 1),
    )


def capture_full_revolution(ser: serial.Serial, side: str):
    """Capture encoder ticks while the user spins one wheel exactly one full turn.

    Returns the absolute tick count for the requested side.
    """
    print(f"\n  --- Ticks-per-revolution calibration ({side}) ---")
    print("  Mark the wheel so you can see exactly one full revolution.")
    input(f"  Press Enter, then slowly spin the {side} wheel EXACTLY ONE FULL TURN forward... ")
    reset_encoders(ser)
    time.sleep(0.05)

    start = average_baseline(ser)
    if start is None:
        raise RuntimeError("failed to read encoder baseline")
    l0, r0, _, _ = start

    print(f"  {'ΔL':>8s}  {'ΔR':>8s}   (press Enter when done)")
    print("  " + "-" * 34)

    done = False

    # Non-blocking stdin check so we can live-update counts
    import select
    import sys
    while not done:
        s = read_sample(ser)
        if s is not None:
            l, r, _, _ = s
            print(f"\r  {l - l0:+8d}  {r - r0:+8d}   ", end="", flush=True)
        # Check if the user pressed Enter
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            done = True
        else:
            time.sleep(DISPLAY_DT_S)

    print()
    end = average_baseline(ser)
    if end is None:
        raise RuntimeError("failed to read encoder final sample")
    l1, r1, _, _ = end
    delta_l = l1 - l0
    delta_r = r1 - r0

    if side == "left":
        ticks = abs(delta_l)
        cross = abs(delta_r)
    else:
        ticks = abs(delta_r)
        cross = abs(delta_l)

    pct_err = ((ticks - EXPECTED_TICKS_PER_REV) / EXPECTED_TICKS_PER_REV * 100.0) if ticks else 0.0
    print(f"  Measured ticks ({side:5s}) : {ticks}")
    print(f"  Expected ticks          : {EXPECTED_TICKS_PER_REV}")
    print(f"  Error                   : {pct_err:+.1f}%")
    if cross > ticks * 0.05:
        other = "right" if side == "left" else "left"
        print(f"  WARNING: {other} encoder also moved {cross} ticks — crosstalk or wrong wheel?")
    return ticks


def side_report(side: str, forward, reverse):
    if side == "left":
        f_count, r_count = forward.delta_l, reverse.delta_l
        f_vel, r_vel = forward.sum_vl, reverse.sum_vl
    else:
        f_count, r_count = forward.delta_r, reverse.delta_r
        f_vel, r_vel = forward.sum_vr, reverse.sum_vr
    count_flip = (f_count != 0 and r_count != 0 and ((f_count > 0) != (r_count > 0)))
    vel_flip = (abs(f_vel) > 0.005 and abs(r_vel) > 0.005 and ((f_vel > 0) != (r_vel > 0)))
    return count_flip, vel_flip


def main():
    ap = argparse.ArgumentParser(description="Guided manual encoder test")
    ap.add_argument("--port", default=None)
    ap.add_argument("--time", type=float, default=2.0, help="capture window per spin direction")
    ap.add_argument("--skip-cpr", action="store_true",
                    help="skip the ticks-per-revolution calibration step")
    ap.add_argument("--wheel", choices=["left", "right", "both"], default="both",
                    help="which wheel to test (default: both)")
    args = ap.parse_args()

    port = detect_port(args.port)
    print("\n=== Guided Manual Encoder Test ===")
    print(f"Port       : {port}")
    print(f"Window     : {args.time:.1f} s")
    print("\nInstructions:")
    print("  • Motors should remain idle; spin wheels by hand only.")
    print("  • For each prompt, rotate only the requested wheel.")
    print("  • Use the real robot forward direction as reference.")
    if not args.skip_cpr:
        print("  • You will also be asked to spin each wheel exactly ONE full turn.")

    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.15)

    try:
        sample = read_sample(ser)
        if sample is None:
            print("ERROR: firmware not responding")
            return 1

        tpr_results = {}

        sides = ["left", "right"] if args.wheel == "both" else [args.wheel]
        for side in sides:
            print("\n" + "=" * 72)
            print(f"Testing {side.upper()} encoder")
            print("=" * 72)

            input(f"Press Enter, then spin ONLY the {side} wheel FORWARD by hand... ")
            reset_encoders(ser)
            forward = capture_motion(ser, args.time)

            input(f"Press Enter, then spin ONLY the {side} wheel REVERSE by hand... ")
            reset_encoders(ser)
            reverse = capture_motion(ser, args.time)

            count_flip, vel_flip = side_report(side, forward, reverse)
            print("\nResult:")
            print(f"  Forward  ΔL/ΔR : {forward.delta_l:+d} / {forward.delta_r:+d}")
            print(f"  Reverse  ΔL/ΔR : {reverse.delta_l:+d} / {reverse.delta_r:+d}")
            print(f"  Forward  peak vel L/R : {forward.peak_vl:+.3f} / {forward.peak_vr:+.3f}")
            print(f"  Reverse  peak vel L/R : {reverse.peak_vl:+.3f} / {reverse.peak_vr:+.3f}")
            print(f"  Forward mean vel L/R  : {forward.sum_vl:+.3f} / {forward.sum_vr:+.3f}")
            print(f"  Reverse mean vel L/R  : {reverse.sum_vl:+.3f} / {reverse.sum_vr:+.3f}")
            print(f"  Raw count sign flips  : {'OK' if count_flip else 'NO'}")
            print(f"  Signed velocity flips : {'OK' if vel_flip else 'NO'}")

            # --- Ticks-per-revolution calibration ---
            if not args.skip_cpr:
                tpr = capture_full_revolution(ser, side)
                tpr_results[side] = tpr

        # --- Summary ---
        if tpr_results:
            print("\n" + "=" * 72)
            print("Ticks-per-revolution summary")
            print("=" * 72)
            for side, tpr in tpr_results.items():
                pct = (tpr - EXPECTED_TICKS_PER_REV) / EXPECTED_TICKS_PER_REV * 100.0 if tpr else 0.0
                status = "OK" if abs(pct) < 5.0 else "CHECK"
                print(f"  {side:5s} : {tpr:6d} ticks  (expected {EXPECTED_TICKS_PER_REV}, err {pct:+.1f}%)  [{status}]")
            avg = sum(tpr_results.values()) / len(tpr_results)
            print(f"  avg   : {avg:8.1f} ticks")
            if abs(avg - EXPECTED_TICKS_PER_REV) > EXPECTED_TICKS_PER_REV * 0.05:
                print(f"\n  NOTE: Average deviates >5% from expected {EXPECTED_TICKS_PER_REV}.")
                print("        Your encoder or gearbox may differ from FIT0450 spec.")
                print(f"        Consider updating ENC_COUNTS_PER_WHEEL_REV to {int(round(avg))} in firmware.")

        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
