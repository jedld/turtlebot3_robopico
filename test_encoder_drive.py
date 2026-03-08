#!/usr/bin/env python3
"""
test_encoder_drive.py — Drive motors forward/reverse while reading encoders.

Does NOT require ROS.  Talks directly to the Pico over Dynamixel serial.

Sequence
--------
  1. Reset encoder counts to zero
  2. Drive FORWARD  at --speed m/s for --time seconds, display live counts
  3. Brake (zero velocity)
  4. Print forward summary  (counts, revolutions, estimated distance)
  5. Pause 1 second
  6. Drive REVERSE  at --speed m/s for --time seconds
  7. Brake
  8. Print reverse summary
  9. Print net encoder counts (should be near zero if encoders are correct)

Register map (from main.c)
--------------------------
  149  ADDR_MOTOR_TORQUE_EN   uint8   write 1 = motors enabled
  150  ADDR_CMD_LINEAR_X      int32   units: 0.01 m/s  (10 → 0.10 m/s)
  170  ADDR_CMD_ANGULAR_Z     int32   units: 0.01 rad/s
  184  ADDR_ENC_L_COUNT       int32   left  encoder tick count (X4 quad)
  188  ADDR_ENC_R_COUNT       int32   right encoder tick count (X4 quad)
  192  ADDR_ENC_RESET         uint8   write non-zero → zero both counters

Usage
-----
  Stop bringup first:
    sudo systemctl stop turtlebot3-bringup.service

  python3 test_encoder_drive.py
  python3 test_encoder_drive.py --speed 0.15 --time 3
  python3 test_encoder_drive.py --speed 0.10 --time 2 --no-reverse
"""

import argparse
import struct
import sys
import time

import serial

# ── constants ───────────────────────────────────────────────────────────────

PORT       = "/dev/ttyACM0"
BAUD       = 1_000_000
DEV_ID     = 200

ADDR_MOTOR_TORQUE_EN = 149
ADDR_CMD_LINEAR_X    = 150   # int32, ×0.01 m/s
ADDR_CMD_ANGULAR_Z   = 170   # int32, ×0.01 rad/s
ADDR_ENC_L_COUNT     = 184   # int32
ADDR_ENC_R_COUNT     = 188   # int32
ADDR_ENC_RESET       = 192   # uint8

# DFRobot FIT0450: 8 PPR motor shaft × 120:1 gearbox × 4 (X4) = 3840 counts/output rev
COUNTS_PER_REV   = 3840
WHEEL_RADIUS_M   = 0.03405   # ~68.1 mm diameter wheel from firmware default

CMD_INTERVAL_S   = 0.05      # send velocity update every 50 ms (well inside 2 s timeout)

# ── Dynamixel Protocol 2.0 ───────────────────────────────────────────────────

_CRC_TABLE: list[int] = []
for _i in range(256):
    _c = _i << 8
    for _ in range(8):
        _c = ((_c << 1) ^ 0x8005) & 0xFFFF if (_c & 0x8000) else (_c << 1) & 0xFFFF
    _CRC_TABLE.append(_c)


def _crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


def _pkt(instr: int, params: bytes) -> bytes:
    length = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, DEV_ID,
                 length & 0xFF, (length >> 8) & 0xFF, instr]) + params
    crc = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])


def _read_resp(ser: serial.Serial, timeout: float = 0.25) -> bytes | None:
    t0 = time.time()
    buf = b""
    while time.time() - t0 < timeout:
        chunk = ser.read(ser.in_waiting or 1)
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


def dxl_write(ser: serial.Serial, addr: int, data: bytes) -> bool:
    params = struct.pack("<H", addr) + data
    ser.write(_pkt(0x03, params))
    resp = _read_resp(ser)
    return resp is not None


def dxl_read(ser: serial.Serial, addr: int, length: int) -> bytes | None:
    params = struct.pack("<HH", addr, length)
    ser.write(_pkt(0x02, params))
    resp = _read_resp(ser)
    if resp and len(resp) >= 9 + length:
        return resp[9 : 9 + length]
    return None


# ── helpers ─────────────────────────────────────────────────────────────────

def read_counts(ser: serial.Serial) -> tuple[int, int] | tuple[None, None]:
    data = dxl_read(ser, ADDR_ENC_L_COUNT, 8)
    if data is None or len(data) < 8:
        return None, None
    return struct.unpack_from("<i", data, 0)[0], struct.unpack_from("<i", data, 4)[0]


def set_velocity(ser: serial.Serial, lin_x_ms: float, ang_z_rads: float = 0.0) -> None:
    lin_raw = int(round(lin_x_ms   * 100.0))
    ang_raw = int(round(ang_z_rads * 100.0))
    # Write torque-enable + linear_x in one burst (addresses 149-153)
    # Padding the 1-byte torque field + 4-byte int32 linear_x
    payload = struct.pack("<Bi", 1, lin_raw)
    dxl_write(ser, ADDR_MOTOR_TORQUE_EN, payload)
    # Write angular_z separately
    dxl_write(ser, ADDR_CMD_ANGULAR_Z, struct.pack("<i", ang_raw))


def brake(ser: serial.Serial) -> None:
    set_velocity(ser, 0.0, 0.0)


def counts_to_revs(c: int) -> float:
    return c / COUNTS_PER_REV


def counts_to_dist_m(c: int) -> float:
    import math
    return counts_to_revs(c) * 2.0 * math.pi * WHEEL_RADIUS_M


# ── drive phase ──────────────────────────────────────────────────────────────

def drive_phase(
    ser: serial.Serial,
    label: str,
    speed_ms: float,
    duration_s: float,
) -> tuple[int, int]:
    """Drive at speed_ms for duration_s.  Returns (delta_left, delta_right)."""

    # Snapshot counts at start
    l0, r0 = read_counts(ser)
    if l0 is None:
        print("  ERROR: no response reading initial counts")
        return 0, 0

    t_end = time.time() + duration_s
    t_next_cmd = 0.0

    print(f"\n  {label}: {speed_ms:+.2f} m/s for {duration_s:.1f} s")
    print(f"  {'Elapsed':>8}  {'L count':>10}  {'L rev':>8}  |"
          f"  {'R count':>10}  {'R rev':>8}")
    print("  " + "-" * 68)

    while time.time() < t_end:
        now = time.time()

        # Refresh velocity command
        if now >= t_next_cmd:
            set_velocity(ser, speed_ms)
            t_next_cmd = now + CMD_INTERVAL_S

        # Read and display
        lc, rc = read_counts(ser)
        if lc is not None:
            elapsed = duration_s - (t_end - now)
            print(
                f"\r  {elapsed:>8.2f}s  {lc:>+10d}  {counts_to_revs(lc):>+8.3f}  |"
                f"  {rc:>+10d}  {counts_to_revs(rc):>+8.3f}",
                end="", flush=True,
            )

        time.sleep(0.05)

    brake(ser)
    print()  # newline after carriage-return line

    # Final snapshot
    lf, rf = read_counts(ser)
    if lf is None:
        return 0, 0
    dl = lf - l0
    dr = rf - r0
    print(f"  → ΔL = {dl:+d} counts  ({counts_to_revs(dl):+.3f} rev, "
          f"~{counts_to_dist_m(dl)*100:+.1f} cm)")
    print(f"  → ΔR = {dr:+d} counts  ({counts_to_revs(dr):+.3f} rev, "
          f"~{counts_to_dist_m(dr)*100:+.1f} cm)")
    return dl, dr


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(
        description="Drive motors F/R while reading quadrature encoders"
    )
    ap.add_argument("--speed",      type=float, default=0.10,
                    help="Drive speed in m/s (default 0.10)")
    ap.add_argument("--time",       type=float, default=3.0,
                    help="Duration per phase in seconds (default 3.0)")
    ap.add_argument("--no-reverse", action="store_true",
                    help="Skip the reverse phase")
    ap.add_argument("--port",       default=PORT,
                    help=f"Serial port (default {PORT})")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, baudrate=BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {args.port}: {e}")
        print("Tip: sudo systemctl stop turtlebot3-bringup.service")
        sys.exit(1)

    time.sleep(0.15)
    ser.reset_input_buffer()

    # Sanity-check comms
    l, r = read_counts(ser)
    if l is None:
        print("ERROR: No response from firmware — check USB and that bringup is stopped.")
        ser.close()
        sys.exit(1)

    print(f"Firmware OK.  Pre-test counts: L={l:+d}  R={r:+d}")

    # Reset counters
    dxl_write(ser, ADDR_ENC_RESET, bytes([1]))
    time.sleep(0.05)
    l, r = read_counts(ser)
    print(f"Counters reset.             L={l:+d}  R={r:+d}")

    # ── FORWARD ──────────────────────────────────────────────────────────────
    dl_fwd, dr_fwd = drive_phase(ser, "FORWARD", +args.speed, args.time)

    time.sleep(1.0)

    if not args.no_reverse:
        # ── REVERSE ──────────────────────────────────────────────────────────
        dl_rev, dr_rev = drive_phase(ser, "REVERSE", -args.speed, args.time)

        # Net counts (should cancel if encoders track correctly)
        net_l = dl_fwd + dl_rev
        net_r = dr_fwd + dr_rev
        print("\n" + "=" * 70)
        print("  NET after fwd+rev:")
        print(f"    L net: {net_l:+d} counts  ({counts_to_dist_m(net_l)*100:+.1f} cm residual)")
        print(f"    R net: {net_r:+d} counts  ({counts_to_dist_m(net_r)*100:+.1f} cm residual)")

        # Direction consistency check
        l_ok = (dl_fwd > 0 and dl_rev < 0) or (dl_fwd < 0 and dl_rev > 0)
        r_ok = (dr_fwd > 0 and dr_rev < 0) or (dr_fwd < 0 and dr_rev > 0)
        print(f"    L direction reversal: {'OK' if l_ok else 'FAIL — same sign both ways!'}")
        print(f"    R direction reversal: {'OK' if r_ok else 'FAIL — same sign both ways!'}")
    else:
        print("\n(Reverse phase skipped)")

    brake(ser)
    ser.close()
    print("\nDone.")


if __name__ == "__main__":
    main()
