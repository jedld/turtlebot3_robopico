#!/usr/bin/env python3
"""
test_encoder.py — Test DFRobot FIT0450 quadrature encoders via the Pico firmware.

Encoders are wired to:
  Left  motor : Grove 3 — ENC_A = GP4 (Yellow), ENC_B = GP5 (White)
  Right motor : Grove 4 — ENC_A = GP6 (Yellow), ENC_B = GP7 (White)

Motor spec:
  8 PPR on motor shaft × 120:1 gear ratio = 960 counts/output rev (X1)
  X4 quadrature decode                    = 3840 counts/output rev

Register addresses (from main.c):
  ADDR_ENC_L_COUNT = 184  int32  left  encoder count
  ADDR_ENC_R_COUNT = 188  int32  right encoder count
  ADDR_ENC_RESET   = 192  uint8  write non-zero to zero both counters

Usage:
  python3 test_encoder.py              # live display (stop ROS bringup first)
  python3 test_encoder.py --reset      # reset counters then live display
  python3 test_encoder.py --oneshot    # single read and exit
"""

import serial, struct, time, sys, argparse

PORT   = "/dev/ttyACM0"
DEV_ID = 200
BAUD   = 1000000

# Register addresses
ADDR_ENC_L_COUNT = 184   # int32
ADDR_ENC_R_COUNT = 188   # int32
ADDR_ENC_RESET   = 192   # uint8

COUNTS_PER_REV = 3840    # X4 × 8 PPR × 120:1

# ── Dynamixel Protocol 2.0 helpers ─────────────────────────────────────────

CRC_TABLE = []
for _i in range(256):
    _crc = _i << 8
    for _ in range(8):
        _crc = ((_crc << 1) ^ 0x8005) & 0xFFFF if (_crc & 0x8000) else (_crc << 1) & 0xFFFF
    CRC_TABLE.append(_crc)

def dxl_crc16(data):
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc

def build_packet(dev_id, instr, params=b''):
    length = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, dev_id,
                 length & 0xFF, (length >> 8) & 0xFF, instr])
    pkt = hdr + params
    crc = dxl_crc16(pkt)
    return pkt + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def read_response(ser, timeout=0.3):
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
                if len(buf) >= 7 + pkt_len:
                    return buf[:7 + pkt_len]
    return None

def dxl_read_bytes(ser, addr, length):
    params = struct.pack('<HH', addr, length)
    pkt = build_packet(DEV_ID, 0x02, params)
    ser.write(pkt)
    resp = read_response(ser)
    if resp and len(resp) >= 9 + length:
        return resp[9:9 + length]
    return None

def dxl_write_byte(ser, addr, value):
    params = struct.pack('<HB', addr, value)
    pkt = build_packet(DEV_ID, 0x03, params)
    ser.write(pkt)
    return read_response(ser)

def read_counts(ser):
    """Read both encoder counts in one 8-byte burst."""
    data = dxl_read_bytes(ser, ADDR_ENC_L_COUNT, 8)
    if data is None or len(data) < 8:
        return None, None
    left  = struct.unpack_from('<i', data, 0)[0]
    right = struct.unpack_from('<i', data, 4)[0]
    return left, right

def reset_counters(ser):
    resp = dxl_write_byte(ser, ADDR_ENC_RESET, 1)
    if resp is None:
        print("WARNING: no response to reset command")
    else:
        print("Encoder counters reset.")

# ── main ───────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Test FIT0450 quadrature encoders via Pico firmware")
    ap.add_argument("--reset",   action="store_true", help="Reset counters before monitoring")
    ap.add_argument("--oneshot", action="store_true", help="Print one reading and exit")
    ap.add_argument("--port",    default=PORT,        help=f"Serial port (default {PORT})")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, baudrate=BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {args.port}: {e}")
        print("Tip: stop ROS bringup first — sudo systemctl stop turtlebot3-bringup.service")
        sys.exit(1)

    time.sleep(0.1)
    ser.reset_input_buffer()

    # Quick ping / sanity check
    left, right = read_counts(ser)
    if left is None:
        print("ERROR: No response from firmware — is the Pico connected and flashed?")
        print("Tip: stop ROS bringup first — sudo systemctl stop turtlebot3-bringup.service")
        ser.close()
        sys.exit(1)

    print(f"Firmware responding.  Initial counts: L={left}  R={right}")

    if args.reset:
        reset_counters(ser)
        time.sleep(0.05)

    if args.oneshot:
        left, right = read_counts(ser)
        print(f"LEFT  enc count : {left:+10d}  ({left / COUNTS_PER_REV:+.3f} rev)")
        print(f"RIGHT enc count : {right:+10d}  ({right / COUNTS_PER_REV:+.3f} rev)")
        ser.close()
        return

    print()
    print("Monitoring encoder counts — rotate wheels manually to verify.")
    print("Press Ctrl+C to stop.\n")
    print(f"  {'LEFT count':>14s}  {'LEFT rev':>9s}  |  {'RIGHT count':>14s}  {'RIGHT rev':>9s}")
    print("  " + "-" * 14 + "  " + "-" * 9 + "  |  " + "-" * 14 + "  " + "-" * 9)

    prev_l = prev_r = None
    try:
        while True:
            left, right = read_counts(ser)
            if left is None:
                print("  (read timeout)")
                time.sleep(0.1)
                continue

            dl = '' if prev_l is None else f"  Δ{left - prev_l:+d}"
            dr = '' if prev_r is None else f"  Δ{right - prev_r:+d}"
            prev_l, prev_r = left, right

            l_rev = left  / COUNTS_PER_REV
            r_rev = right / COUNTS_PER_REV
            print(f"\r  {left:>+14d}  {l_rev:>+9.3f}  |  {right:>+14d}  {r_rev:>+9.3f}{dl}{dr}",
                  end='', flush=True)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nStopped.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
