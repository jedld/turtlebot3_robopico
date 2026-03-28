#!/usr/bin/env python3
"""Read firmware diagnostic counters from the TurtleBot3 Pico firmware."""

import argparse
import struct
import time

import serial

PORTS = ("/dev/ttyTB3", "/dev/ttyACM0", "/dev/ttyACM1")
DEV_ID = 200
BAUD = 1_000_000

ADDR_DIAG_USB_TX_STALLS = 224
ADDR_DIAG_SENSOR_MAX_US = 228
ADDR_DIAG_ODOM_MAX_US = 232
ADDR_DIAG_LOOP_MAX_US = 236
ADDR_DIAG_PKT_COUNT = 240
ADDR_DIAG_CRC_FAIL = 244
ADDR_DIAG_VEL_WRITES = 248
ADDR_DIAG_READ_COUNT = 252

ADDR_MOTOR_I2C_NDEV    = 344
ADDR_MOTOR_I2C_DEV0    = 345
ADDR_MOTOR_I2C_DEV1    = 346
ADDR_MOTOR_I2C_DEV2    = 347
ADDR_MOTOR_I2C_STATUS  = 348
ADDR_MOTOR_I2C_ERR_CNT = 352
ADDR_MOTOR_LAST_CMD_M1 = 356   # int8: last raw speed sent to Motor1
ADDR_MOTOR_LAST_CMD_M2 = 357   # int8: last raw speed sent to Motor2
ADDR_MOTOR_WHO_AM_I    = 360   # uint8: WHO_AM_I from device; 0xA4 = genuine Waveshare

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

def read_resp(ser: serial.Serial, timeout: float = 0.3):
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
        return resp[9:9+length]
    return None

def detect_port(explicit: str | None):
    if explicit:
        return explicit
    import os
    for p in PORTS:
        if os.path.exists(p):
            return p
    return PORTS[0]

def main():
    ap = argparse.ArgumentParser(description="Read firmware diagnostic counters")
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    port = detect_port(args.port)
    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.15)

    data = dxl_read(ser, ADDR_DIAG_USB_TX_STALLS, 32)
    if not data or len(data) < 32:
        print("ERROR: no response reading diagnostics")
        ser.close()
        return 1

    names = [
        "usb_tx_stalls",
        "sensor_max_us",
        "odom_max_us",
        "loop_max_us",
        "pkt_count",
        "crc_fail",
        "vel_writes",
        "read_count",
    ]
    values = struct.unpack("<8i", data)
    for name, value in zip(names, values):
        print(f"{name:16s}: {value}")

    # Motor I2C1 diagnostics (addr 344-360)
    mdata = dxl_read(ser, ADDR_MOTOR_I2C_NDEV, 17)
    if mdata and len(mdata) >= 17:
        ndev    = mdata[0]
        devs    = [mdata[i] for i in range(1, 4) if mdata[i] != 0xFF]
        status  = mdata[4]
        err_cnt = struct.unpack_from("<I", mdata, 8)[0]
        m1_last = struct.unpack_from("<b", mdata, 12)[0]
        m2_last = struct.unpack_from("<b", mdata, 13)[0]
        who_am_i = mdata[16]
        print()
        print(f"{'motor_i2c_ndev':16s}: {ndev}  (devices found on I2C1/Grove2)")
        dev_strs = [f"0x{d:02X}" for d in devs]
        print(f"{'motor_i2c_devs':16s}: {dev_strs if dev_strs else '(none)'}")
        status_str = {0: "OK (0x55 responded)", 1: "FAIL (0x55 not found)", 0xFF: "not yet init"}.get(status, f"unknown ({status})")
        print(f"{'motor_i2c_status':16s}: {status_str}")
        print(f"{'motor_i2c_errors':16s}: {err_cnt}")
        print(f"{'motor_last_cmd_m1':16s}: {m1_last:+d}  (last raw int8 to Motor1 register)")
        print(f"{'motor_last_cmd_m2':16s}: {m2_last:+d}  (last raw int8 to Motor2 register)")
        who_str = f"0x{who_am_i:02X}" + (" (OK)" if who_am_i == 0xA4 else " (expected 0xA4)" if who_am_i != 0xFF else " (not read)")
        print(f"{'motor_who_am_i':16s}: {who_str}")
    else:
        print("\nmotor_i2c diag : (no response — old firmware?)")

    ser.close()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
