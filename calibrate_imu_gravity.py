#!/usr/bin/env python3
"""
calibrate_imu_gravity.py — Interactive BNO085 accel gravity calibration

The BNO085's Motion Engine (ME) calibration uses sphere-fitting to correct
accelerometer scale and offset errors.  It needs to observe the gravity vector
from multiple orientations before it can compute accurate corrections.

This script guides you through:
  1. Clearing the old (bad) DCD from BNO085 flash   (RECAL = 2)
  2. Enabling ME calibration and holding each of 6 orientations in turn
  3. Saving the converged DCD to BNO085 flash        (RECAL = 1)
  4. Verifying the final gravity magnitude

Usage:
    python3 calibrate_imu_gravity.py [--port /dev/ttyTB3]

IMPORTANT:
  - Stop turtlebot3-bringup before running this script.
  - The Pico firmware must have the updated RECAL=2 (DCD-clear) support.
"""

import serial
import struct
import time
import sys
import math
import argparse
import subprocess

# ── Dynamixel register addresses (must match main.c) ─────────────────────────
DEV_ID              = 200
BAUD_RATE           = 1_000_000

ADDR_IMU_RECAL      = 59
ADDR_IMU_LIN_ACC_X  = 72   # float32 × 3  (x, y, z)   m/s²
ADDR_IMU_ANG_VEL_X  = 60   # float32 × 3  (x, y, z)   rad/s
ADDR_DBG_IMU_SOURCE = 174  # uint8: 0=simulated 1=BNO085

GRAVITY_TARGET      = 9.80665   # standard gravity (m/s²)
GRAVITY_TOLERANCE   = 0.12      # ±0.12 m/s² = ±1.2 %

# ── Terminal colours ──────────────────────────────────────────────────────────
RED  = "\033[0;31m"
GRN  = "\033[0;32m"
YLW  = "\033[1;33m"
CYN  = "\033[0;36m"
BLD  = "\033[1m"
DIM  = "\033[2m"
NC   = "\033[0m"

def ok(msg):   print(f"{GRN}[  OK  ]{NC}  {msg}")
def fail(msg): print(f"{RED}[ FAIL ]{NC}  {msg}")
def info(msg): print(f"{CYN}[ INFO ]{NC}  {msg}")
def warn(msg): print(f"{YLW}[ WARN ]{NC}  {msg}")
def step(n, msg): print(f"\n{BLD}── Step {n}: {msg} {NC}")

# ── Dynamixel Protocol 2.0 helpers ───────────────────────────────────────────
_CRC_TABLE = []
for _i in range(256):
    _crc = _i << 8
    for _ in range(8):
        _crc = ((_crc << 1) ^ 0x8005) & 0xFFFF if (_crc & 0x8000) else (_crc << 1) & 0xFFFF
    _CRC_TABLE.append(_crc)

def _crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc

def _build_read(dev_id: int, addr: int, length: int) -> bytes:
    params = struct.pack('<HH', addr, length)
    pkt_len = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, dev_id,
                 pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, 0x02])
    pkt = hdr + params
    crc = _crc16(pkt)
    return pkt + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def _build_write(dev_id: int, addr: int, data_bytes) -> bytes:
    params = struct.pack('<H', addr) + bytes(data_bytes)
    pkt_len = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, dev_id,
                 pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, 0x03])
    pkt = hdr + params
    crc = _crc16(pkt)
    return pkt + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def _read_response(ser: serial.Serial, timeout: float = 2.0) -> bytes | None:
    t0 = time.time()
    buf = b''
    while time.time() - t0 < timeout:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
        idx = buf.find(b'\xFF\xFF\xFD\x00')
        if idx >= 0:
            buf = buf[idx:]
            if len(buf) >= 7:
                pkt_len = buf[5] | (buf[6] << 8)
                total = 7 + pkt_len
                if len(buf) >= total:
                    return buf[:total]
    return None

def do_read(ser: serial.Serial, addr: int, length: int) -> bytes | None:
    pkt = _build_read(DEV_ID, addr, length)
    ser.reset_input_buffer()
    ser.write(pkt)
    resp = _read_response(ser)
    if resp is None or len(resp) < 9 + length:
        return None
    return resp[9:-2]

def do_write(ser: serial.Serial, addr: int, data_bytes) -> None:
    pkt = _build_write(DEV_ID, addr, data_bytes)
    ser.reset_input_buffer()
    ser.write(pkt)
    _read_response(ser, timeout=0.5)   # drain ack

# ── Sensor helpers ────────────────────────────────────────────────────────────
def read_accel(ser: serial.Serial) -> tuple[float, float, float] | None:
    """Return (ax, ay, az) in m/s², or None on timeout."""
    data = do_read(ser, ADDR_IMU_LIN_ACC_X, 12)
    if data is None:
        return None
    ax, ay, az = struct.unpack_from('<fff', data, 0)
    return ax, ay, az

def gravity_mag(ser: serial.Serial, samples: int = 10) -> float | None:
    """Average |g| over `samples` readings."""
    mags = []
    for _ in range(samples):
        v = read_accel(ser)
        if v:
            ax, ay, az = v
            mags.append(math.sqrt(ax**2 + ay**2 + az**2))
        time.sleep(0.05)
    return (sum(mags) / len(mags)) if mags else None

def check_bno085_present(ser: serial.Serial) -> bool:
    data = do_read(ser, ADDR_DBG_IMU_SOURCE, 1)
    return data is not None and data[0] == 1

# ── Live gravity bar ──────────────────────────────────────────────────────────
def live_gravity_bar(mag: float, width: int = 30) -> str:
    """ASCII bar showing proximity to 9.81 m/s²."""
    pct   = min(max((mag - 9.0) / (10.6 - 9.0), 0.0), 1.0)
    ideal = int((GRAVITY_TARGET - 9.0) / (10.6 - 9.0) * width)
    filled = int(pct * width)
    bar = list('─' * width)
    for i in range(filled):
        bar[i] = '█'
    if 0 <= ideal < width:
        bar[ideal] = '|'
    color = GRN if abs(mag - GRAVITY_TARGET) < GRAVITY_TOLERANCE else YLW
    return f"{color}[{''.join(bar)}]{NC}  {mag:.4f} m/s²"

# ── Orientation steps ─────────────────────────────────────────────────────────
ORIENTATIONS = [
    ("flat (upright, normal operating position)",
     "Set the robot on a flat surface in its normal driving orientation.",
     lambda ax, ay, az: abs(az) > 8.0 and az > 0),

    ("upside-down (bottom facing up)",
     "Flip the robot completely upside-down and hold it steady.",
     lambda ax, ay, az: abs(az) > 8.0 and az < 0),

    ("left side down (tilt 90° left)",
     "Tilt the robot 90° to the LEFT so the left wheel faces down.",
     lambda ax, ay, az: abs(ay) > 8.0 and ay < 0),

    ("right side down (tilt 90° right)",
     "Tilt the robot 90° to the RIGHT so the right wheel faces down.",
     lambda ax, ay, az: abs(ay) > 8.0 and ay > 0),

    ("nose down (front facing down)",
     "Tilt the robot forward so the front is pointing straight down.",
     lambda ax, ay, az: abs(ax) > 8.0 and ax > 0),

    ("nose up (back facing down, front pointing up)",
     "Tilt the robot backward so the front is pointing straight up.",
     lambda ax, ay, az: abs(ax) > 8.0 and ax < 0),
]

HOLD_SECS = 5   # seconds to hold each orientation once detected

# ── Main calibration flow ─────────────────────────────────────────────────────
def confirm(prompt: str) -> bool:
    try:
        ans = input(f"{YLW}{prompt} [Y/n]: {NC}").strip().lower()
        return ans in ('', 'y', 'yes')
    except (EOFError, KeyboardInterrupt):
        print()
        return False

def wait_enter(prompt: str) -> None:
    try:
        input(f"{DIM}{prompt}{NC}")
    except (EOFError, KeyboardInterrupt):
        print()

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Interactive BNO085 multi-orientation gravity calibration")
    ap.add_argument("--port",    default="/dev/ttyTB3",
                    help="Serial port (default: /dev/ttyTB3)")
    ap.add_argument("--skip-clear", action="store_true",
                    help="Skip DCD clear step (not recommended on first run)")
    ap.add_argument("--hold",   type=int, default=HOLD_SECS,
                    help=f"Seconds to hold each orientation (default: {HOLD_SECS})")
    args = ap.parse_args()

    print()
    print(f"{BLD}{'='*62}{NC}")
    print(f"{BLD}   BNO085 Multi-Orientation Accel Gravity Calibration{NC}")
    print(f"{BLD}{'='*62}{NC}")
    print()
    print("This procedure corrects the BNO085's accelerometer scale bias")
    print("by guiding you through 6 orientations so the ME sphere-fitting")
    print(f"algorithm can converge.  Target: {GRAVITY_TARGET:.5f} m/s²  (±{GRAVITY_TOLERANCE} m/s²)")
    print()
    print(f"{YLW}IMPORTANT — before starting:{NC}")
    print("  • Make sure turtlebot3-bringup is STOPPED")
    print("    (sudo systemctl stop turtlebot3-bringup.service)")
    print("  • Keep the USB cable connected throughout")
    print("  • Work on a clear flat surface — no vibration")
    print()

    # ── Check bringup is not running ─────────────────────────────────────────
    result = subprocess.run(
        ["systemctl", "is-active", "--quiet", "turtlebot3-bringup.service"])
    if result.returncode == 0:
        warn("turtlebot3-bringup.service is currently ACTIVE.")
        if confirm("Stop it now?"):
            subprocess.run(["sudo", "systemctl", "stop", "turtlebot3-bringup.service"],
                           check=True)
            time.sleep(1.5)
            ok("Bringup stopped.")
        else:
            fail("Cannot proceed with bringup running — it holds the serial port.")
            return 2

    # ── Open serial port ──────────────────────────────────────────────────────
    info(f"Opening {args.port} at {BAUD_RATE} baud ...")
    try:
        ser = serial.Serial(args.port, BAUD_RATE, timeout=2.0)
    except serial.SerialException as exc:
        fail(f"Cannot open port: {exc}")
        return 2
    time.sleep(0.4)

    # ── Verify BNO085 is present ──────────────────────────────────────────────
    if not check_bno085_present(ser):
        warn("ADDR_DBG_IMU_SOURCE does not report BNO085 — sensor may be simulated.")
        if not confirm("Continue anyway?"):
            ser.close()
            return 1
    else:
        ok("BNO085 hardware confirmed active.")

    # ── Baseline reading ──────────────────────────────────────────────────────
    step(1, "Baseline gravity reading")
    info("Reading current gravity magnitude (10 samples) ...")
    baseline = gravity_mag(ser, samples=10)
    if baseline is None:
        fail("Could not read accelerometer. Check USB connection and firmware.")
        ser.close()
        return 2
    print(f"  Baseline  {live_gravity_bar(baseline)}")
    print(f"  Error     {baseline - GRAVITY_TARGET:+.4f} m/s²  ({(baseline/GRAVITY_TARGET - 1)*100:+.2f}%)")

    # ── Clear old DCD ─────────────────────────────────────────────────────────
    if not args.skip_clear:
        step(2, "Clear old DCD from BNO085 flash")
        print("  Writing RECAL=2 → disabling ME cal, saving zero offsets, re-enabling ME cal.")
        print("  This erases the accumulated bad calibration data from the sensor flash.")
        wait_enter("  Press Enter to clear DCD ...")
        do_write(ser, ADDR_IMU_RECAL, [2])
        # Wait for firmware to complete the three SHTP commands (~60 ms)
        time.sleep(1.0)
        ok("DCD cleared — BNO085 now starting from zero offsets.")
    else:
        step(2, "Clear DCD — SKIPPED (--skip-clear)")

    # ── Enable ME calibration and collect orientations ─────────────────────────
    step(3, "Multi-orientation calibration")
    print("  Writing RECAL=1 → enabling ME calibration on the BNO085.")
    print("  You will be guided through 6 orientations.  Hold each one STILL")
    print(f"  for {args.hold} seconds until instructed to move.")
    print()
    wait_enter("  Press Enter to begin orientation calibration ...")
    do_write(ser, ADDR_IMU_RECAL, [1])
    # Give firmware a moment to send the ME cal enable command to the BNO085
    time.sleep(0.5)

    for idx, (name, instruction, detect_fn) in enumerate(ORIENTATIONS, start=1):
        print()
        print(f"{BLD}  Orientation {idx}/{len(ORIENTATIONS)}: {name}{NC}")
        print(f"  {instruction}")
        wait_enter("  Press Enter when ready ...")

        # Wait until the sensor reports the expected orientation
        print("  Detecting orientation ...", end="", flush=True)
        deadline = time.time() + 15.0
        detected = False
        while time.time() < deadline:
            v = read_accel(ser)
            if v and detect_fn(*v):
                detected = True
                break
            time.sleep(0.1)
            print(".", end="", flush=True)

        if not detected:
            print()
            warn(f"  Orientation '{name}' not confirmed by sensor — continuing anyway.")
        else:
            print()
            ok(f"  Orientation detected.")

        # Hold still for the specified duration, showing a live countdown + gravity bar
        print(f"  Hold STILL for {args.hold} seconds ...")
        t_start = time.time()
        while time.time() - t_start < args.hold:
            remaining = args.hold - (time.time() - t_start)
            v = read_accel(ser)
            if v:
                mag = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
                print(f"\r  [{remaining:4.1f}s]  {live_gravity_bar(mag, width=25)}", end="", flush=True)
            time.sleep(0.2)
        print()
        ok(f"  Orientation {idx} complete.")

    # ── Save DCD ──────────────────────────────────────────────────────────────
    step(4, "Save calibrated DCD to BNO085 flash")
    print("  The ME engine has updated the accelerometer offsets.")
    print("  Writing RECAL=1 again with the robot FLAT will save the converged DCD.")
    print()
    print(f"{BLD}  Place the robot back in its normal flat orientation and hold still.{NC}")
    wait_enter("  Press Enter to save DCD (will wait 6 s for final convergence) ...")

    do_write(ser, ADDR_IMU_RECAL, [1])
    print("  Saving calibration data ... ", end="", flush=True)
    for i in range(7):
        time.sleep(1.0)
        print(f"{7-i}s ", end="", flush=True)
        v = read_accel(ser)
        if v:
            mag = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
            print(f"[|g|={mag:.3f}] ", end="", flush=True)
    print()
    ok("DCD saved to BNO085 on-chip flash.")

    # ── Final verification ─────────────────────────────────────────────────────
    step(5, "Verify final gravity reading")
    info("Reading post-calibration gravity (20 samples) ...")
    time.sleep(0.5)
    final = gravity_mag(ser, samples=20)
    if final is None:
        fail("Could not read final gravity — check connection.")
        ser.close()
        return 2

    improvement = abs(baseline - GRAVITY_TARGET) - abs(final - GRAVITY_TARGET)
    print()
    print(f"  Baseline  {live_gravity_bar(baseline)}")
    print(f"  Final     {live_gravity_bar(final)}")
    print()
    print(f"  Error before:  {baseline - GRAVITY_TARGET:+.4f} m/s²")
    print(f"  Error after:   {final    - GRAVITY_TARGET:+.4f} m/s²")
    print(f"  Improvement:   {improvement:+.4f} m/s²")

    if abs(final - GRAVITY_TARGET) <= GRAVITY_TOLERANCE:
        ok(f"Gravity magnitude {final:.4f} m/s² is within ±{GRAVITY_TOLERANCE} m/s² of target.")
        calibration_ok = True
    else:
        warn(f"Gravity {final:.4f} m/s² is still outside ±{GRAVITY_TOLERANCE} m/s².")
        print()
        print("  The BNO085 ME sphere-fitting may need more orientation diversity.")
        print("  Try running this script again and tilting more aggressively through")
        print("  each orientation.  Improvement will accumulate across runs.")
        calibration_ok = False

    ser.close()

    # ── Restart bringup ───────────────────────────────────────────────────────
    print()
    if confirm("Restart turtlebot3-bringup.service now?"):
        subprocess.run(["sudo", "systemctl", "start", "turtlebot3-bringup.service"])
        ok("turtlebot3-bringup.service started.")

    print()
    print(f"{BLD}{'='*62}{NC}")
    print(f"{BLD}  Calibration {'PASSED' if calibration_ok else 'COMPLETE (further runs may help)'}{NC}")
    print(f"{BLD}{'='*62}{NC}")
    print()
    return 0 if calibration_ok else 1


if __name__ == "__main__":
    sys.exit(main())
