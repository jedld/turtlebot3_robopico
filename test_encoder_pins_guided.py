#!/usr/bin/env python3
"""
Guided raw encoder-pin diagnostic for TurtleBot3 Pico firmware.

Purpose
-------
Identify which physical wheel toggles which encoder GPIO pair in firmware.
This reads:
  - left/right encoder counts
  - raw encoder GPIO state bits
  - IRQ callback counter

Useful when wheel-to-encoder mapping still looks inconsistent.
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
ADDR_ENC_IRQ_DBG = 193
ADDR_ENC_GPIO_RAW = 197

CRC_TABLE = []
for i in range(256):
    c = i << 8
    for _ in range(8):
        c = (((c << 1) ^ 0x8005) & 0xFFFF) if (c & 0x8000) else ((c << 1) & 0xFFFF)
    CRC_TABLE.append(c)


@dataclass
class Snapshot:
    left: int
    right: int
    irq: int
    raw: int


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


def dxl_read(ser: serial.Serial, addr: int, length: int):
    ser.reset_input_buffer()
    ser.write(packet(0x02, struct.pack("<HH", addr, length)))
    resp = read_resp(ser)
    if resp and len(resp) >= 9 + length:
        return resp[9:9 + length]
    return None


def dxl_write(ser: serial.Serial, addr: int, data: bytes):
    ser.reset_input_buffer()
    ser.write(packet(0x03, struct.pack("<H", addr) + data))
    return read_resp(ser) is not None


def detect_port(explicit: str | None):
    if explicit:
        return explicit
    for p in PORT_CANDIDATES:
        if os.path.exists(p):
            return p
    return PORT_CANDIDATES[0]


def reset_encoders(ser: serial.Serial):
    dxl_write(ser, ADDR_ENC_RESET, bytes([1]))
    time.sleep(0.05)


def read_snapshot(ser: serial.Serial) -> Snapshot | None:
    a = dxl_read(ser, ADDR_ENC_L_COUNT, 14)
    if not a or len(a) < 14:
        return None
    return Snapshot(
        left=struct.unpack_from("<i", a, 0)[0],
        right=struct.unpack_from("<i", a, 4)[0],
        irq=struct.unpack_from("<i", a, 9)[0],
        raw=a[13],
    )


def bit(raw: int, n: int) -> int:
    return 1 if (raw & (1 << n)) else 0


def raw_str(raw: int) -> str:
    return f"LA={bit(raw,0)} LB={bit(raw,1)} RA={bit(raw,2)} RB={bit(raw,3)}"


def capture_activity(ser: serial.Serial, duration: float):
    start = read_snapshot(ser)
    if start is None:
        raise RuntimeError("failed to read snapshot")

    toggles = [0, 0, 0, 0]
    prev_raw = start.raw
    t_end = time.time() + duration

    print(f"  {'elapsed':>7s}  {'ΔL':>8s}  {'ΔR':>8s}  {'ΔIRQ':>8s}  {'raw':>22s}")
    print("  " + "-" * 70)
    while time.time() < t_end:
        s = read_snapshot(ser)
        if s is not None:
            changed = prev_raw ^ s.raw
            for i in range(4):
                if changed & (1 << i):
                    toggles[i] += 1
            prev_raw = s.raw
            elapsed = duration - (t_end - time.time())
            print(
                f"\r  {elapsed:7.2f}  {s.left-start.left:+8d}  {s.right-start.right:+8d}  {s.irq-start.irq:+8d}  {raw_str(s.raw):>22s}",
                end="",
                flush=True,
            )
        time.sleep(0.04)
    print()

    end = read_snapshot(ser)
    if end is None:
        raise RuntimeError("failed to read final snapshot")

    return {
        "delta_left": end.left - start.left,
        "delta_right": end.right - start.right,
        "delta_irq": end.irq - start.irq,
        "toggles": {
            "LA": toggles[0],
            "LB": toggles[1],
            "RA": toggles[2],
            "RB": toggles[3],
        },
    }


def main():
    ap = argparse.ArgumentParser(description="Guided raw encoder-pin diagnostic")
    ap.add_argument("--port", default=None)
    ap.add_argument("--time", type=float, default=2.0)
    args = ap.parse_args()

    port = detect_port(args.port)
    print("\n=== Guided Raw Encoder-Pin Diagnostic ===")
    print(f"Port   : {port}")
    print(f"Window : {args.time:.1f} s")
    print("\nBit meaning:")
    print("  LA/LB = firmware logical LEFT encoder A/B inputs")
    print("  RA/RB = firmware logical RIGHT encoder A/B inputs")

    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.15)

    try:
        if read_snapshot(ser) is None:
            print("ERROR: firmware not responding")
            return 1

        for wheel in ("left", "right"):
            print("\n" + "=" * 72)
            print(f"Spin ONLY the {wheel.upper()} wheel by hand")
            print("=" * 72)
            input(f"Press Enter, then spin ONLY the {wheel} wheel repeatedly for {args.time:.1f} s... ")
            reset_encoders(ser)
            result = capture_activity(ser, args.time)
            print("\nResult:")
            print(f"  ΔL count : {result['delta_left']:+d}")
            print(f"  ΔR count : {result['delta_right']:+d}")
            print(f"  ΔIRQ     : {result['delta_irq']:+d}")
            print("  GPIO toggles:")
            for name, value in result["toggles"].items():
                print(f"    {name}: {value}")

        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
