#!/usr/bin/env python3
"""
Manual runtime calibration CLI for TurtleBot3 Pico firmware custom instruction 0x90.

Supports:
  - get            : read current runtime calibration values
  - set            : update one calibration key at runtime
  - save           : persist current runtime values to firmware flash
  - load           : load persisted values from firmware flash
  - reset          : reset runtime values to firmware defaults
  - reset-save     : reset runtime values and persist defaults to flash

Examples:
  python3 dxl_calibration_cli.py get
  python3 dxl_calibration_cli.py set wheel_separation 0.0752
  python3 dxl_calibration_cli.py save
"""

import argparse
import struct
import time
from typing import Optional

import serial

DEV_ID = 200
BAUD_RATE = 1_000_000

INST_CALIBRATION = 0x90
CALIB_CMD_SET = 0x01
CALIB_CMD_GET = 0x02
CALIB_CMD_RESET = 0x03
CALIB_CMD_SAVE = 0x04
CALIB_CMD_LOAD = 0x05
CALIB_CMD_RESET_AND_SAVE = 0x06

KEYS = {
    "wheel_radius": 0x01,
    "wheel_separation": 0x02,
    "max_wheel_speed_ms": 0x03,
    "motor_min_duty": 0x04,
    "motor_kick_duty": 0x05,
    "motor_kick_cycles": 0x06,
    "motor_trim_left": 0x07,
    "motor_trim_right": 0x08,
    "right_motor_reversed": 0x09,
    "swap_motors": 0x0A,
}


def make_crc_table() -> list[int]:
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8005) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        table.append(crc)
    return table


CRC_TABLE = make_crc_table()


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


def build_instruction(dev_id: int, inst: int, params: bytes) -> bytes:
    packet_len = len(params) + 3
    header = bytes([
        0xFF, 0xFF, 0xFD, 0x00,
        dev_id,
        packet_len & 0xFF,
        (packet_len >> 8) & 0xFF,
        inst,
    ])
    packet = header + params
    c = crc16(packet)
    return packet + bytes([c & 0xFF, (c >> 8) & 0xFF])


def read_status_packet(ser: serial.Serial, timeout: float = 0.6) -> Optional[bytes]:
    end = time.time() + timeout
    buf = b""
    while time.time() < end:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk

        idx = buf.find(b"\xFF\xFF\xFD\x00")
        if idx < 0:
            if len(buf) > 4096:
                buf = buf[-64:]
            continue

        if idx > 0:
            buf = buf[idx:]

        if len(buf) < 7:
            continue

        pkt_len = buf[5] | (buf[6] << 8)
        total = 7 + pkt_len
        if len(buf) < total:
            continue

        pkt = buf[:total]
        crc_rx = pkt[-2] | (pkt[-1] << 8)
        crc_ok = crc16(pkt[:-2]) == crc_rx
        if not crc_ok:
            buf = buf[1:]
            continue

        if pkt[7] != 0x55:
            buf = buf[total:]
            continue

        return pkt

    return None


def send_calibration(ser: serial.Serial, subcmd: int, key: Optional[int] = None,
                     value: Optional[float] = None, expect_reply: bool = True) -> Optional[bytes]:
    params = bytes([subcmd])
    if subcmd == CALIB_CMD_SET:
        if key is None or value is None:
            raise ValueError("SET requires key and value")
        params += bytes([key]) + struct.pack("<f", float(value))

    packet = build_instruction(DEV_ID, INST_CALIBRATION, params)
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()

    if not expect_reply:
        return None

    return read_status_packet(ser)


def decode_get_payload(payload: bytes) -> dict:
    if len(payload) < 32 or payload[0] != CALIB_CMD_GET:
        raise ValueError("Unexpected GET payload")

    return {
        "wheel_radius": struct.unpack_from("<f", payload, 1)[0],
        "wheel_separation": struct.unpack_from("<f", payload, 5)[0],
        "max_wheel_speed_ms": struct.unpack_from("<f", payload, 9)[0],
        "right_motor_reversed": int(payload[13]),
        "swap_motors": int(payload[14]),
        "motor_min_duty": struct.unpack_from("<f", payload, 15)[0],
        "motor_kick_duty": struct.unpack_from("<f", payload, 19)[0],
        "motor_kick_cycles": int(payload[23]),
        "motor_trim_left": struct.unpack_from("<f", payload, 24)[0],
        "motor_trim_right": struct.unpack_from("<f", payload, 28)[0],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Manual 0x90 runtime calibration CLI")
    parser.add_argument("--port", default="/dev/ttyTB3", help="serial port (default: /dev/ttyTB3)")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help="baud rate (default: 1000000)")

    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("get", help="read current runtime calibration values")

    setp = sub.add_parser("set", help="set one runtime calibration value")
    setp.add_argument("key", choices=sorted(KEYS.keys()))
    setp.add_argument("value", type=float)

    sub.add_parser("save", help="persist current runtime values to firmware flash")
    sub.add_parser("load", help="load persisted values from firmware flash")
    sub.add_parser("reset", help="reset runtime values to firmware defaults")
    sub.add_parser("reset-save", help="reset runtime values and persist defaults to flash")

    args = parser.parse_args()

    try:
        with serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=0.2) as ser:
            if args.cmd == "get":
                pkt = send_calibration(ser, CALIB_CMD_GET, expect_reply=True)
                if pkt is None:
                    print("ERROR: timeout waiting for status packet")
                    return 2
                err = pkt[8]
                if err != 0:
                    print(f"ERROR: status error code=0x{err:02X}")
                    return 3
                params = pkt[9:-2]
                values = decode_get_payload(params)
                for k, v in values.items():
                    if isinstance(v, float):
                        print(f"{k:22s} {v:.6f}")
                    else:
                        print(f"{k:22s} {v}")
                return 0

            if args.cmd == "set":
                pkt = send_calibration(ser, CALIB_CMD_SET, key=KEYS[args.key], value=args.value,
                                       expect_reply=True)
            elif args.cmd == "save":
                pkt = send_calibration(ser, CALIB_CMD_SAVE, expect_reply=True)
            elif args.cmd == "load":
                pkt = send_calibration(ser, CALIB_CMD_LOAD, expect_reply=True)
            elif args.cmd == "reset":
                pkt = send_calibration(ser, CALIB_CMD_RESET, expect_reply=True)
            elif args.cmd == "reset-save":
                pkt = send_calibration(ser, CALIB_CMD_RESET_AND_SAVE, expect_reply=True)
            else:
                print("ERROR: unknown command")
                return 1

            if pkt is None:
                print("ERROR: timeout waiting for status packet")
                return 2

            err = pkt[8]
            if err != 0:
                print(f"ERROR: status error code=0x{err:02X}")
                return 3

            print("OK")
            return 0

    except serial.SerialException as e:
        print(f"ERROR: serial open/write failed: {e}")
        return 4


if __name__ == "__main__":
    raise SystemExit(main())
