#!/usr/bin/env python3
"""
Test ultrasonic sensor(s) on the TurtleBot3 Pico.

Enables the ultrasonic sensor(s) via ADDR_USS_ENABLE (184),
then polls ADDR_USS_STATUS (185), ADDR_USS_1_DIST_MM (186),
and ADDR_USS_2_DIST_MM (188) in a loop.

Usage:
    python3 test_ultrasonic.py               # test both sensors
    python3 test_ultrasonic.py 1             # test sensor 1 only (Grove 2)
    python3 test_ultrasonic.py 2             # test sensor 2 only (Grove 3)
    python3 test_ultrasonic.py --diag        # one-shot readout of both sensors
"""

import serial, struct, time, sys

PORT = "/dev/ttyACM0"
DEV_ID = 200

# Register addresses (from main.c)
ADDR_USS_ENABLE    = 184   # 1 byte: bit 0 = USS1, bit 1 = USS2
ADDR_USS_STATUS    = 185   # 1 byte: bit 0 = USS1 valid, bit 1 = USS2 valid,
                           #         bit 4 = USS1 timeout, bit 5 = USS2 timeout
ADDR_USS_1_DIST_MM = 186   # uint16
ADDR_USS_2_DIST_MM = 188   # uint16
ADDR_USS_1_WAIT_US = 190   # uint16
ADDR_USS_1_PULSE_US = 192  # uint16
ADDR_USS_1_DBG_FLAGS = 194 # uint8
ADDR_USS_2_WAIT_US = 195   # uint16
ADDR_USS_2_PULSE_US = 197  # uint16
ADDR_USS_2_DBG_FLAGS = 199 # uint8

# CRC-16 (poly 0x8005, MSB-first) — matches firmware
CRC_TABLE = []
for i in range(256):
    crc = i << 8
    for _ in range(8):
        if crc & 0x8000:
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF
        else:
            crc = (crc << 1) & 0xFFFF
    CRC_TABLE.append(crc)

def dxl_crc16(data):
    crc = 0
    for b in data:
        idx = ((crc >> 8) ^ b) & 0xFF
        crc = ((crc << 8) ^ CRC_TABLE[idx]) & 0xFFFF
    return crc

def build_packet(dev_id, instruction, params=b''):
    length = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, dev_id,
                 length & 0xFF, (length >> 8) & 0xFF, instruction])
    pkt = hdr + params
    crc = dxl_crc16(pkt)
    return pkt + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def read_response(ser, timeout=0.5):
    start = time.time()
    buf = b''
    while time.time() - start < timeout:
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

def dxl_write(ser, addr, data):
    params = struct.pack('<H', addr) + data
    pkt = build_packet(DEV_ID, 0x03, params)  # WRITE = 0x03
    ser.write(pkt)
    return read_response(ser)

def dxl_read(ser, addr, length):
    params = struct.pack('<HH', addr, length)
    pkt = build_packet(DEV_ID, 0x02, params)  # READ = 0x02
    ser.write(pkt)
    resp = read_response(ser)
    if resp and len(resp) >= 11:
        return resp[9:-2]  # strip header(7) + instr(1) + err(1) ... CRC(2)
    return None


def main():
    diag_mode = any(a in ("--diag", "diag") for a in sys.argv[1:])

    if diag_mode:
        ser = serial.Serial(PORT, timeout=0.1)
        time.sleep(0.1)
        ser.reset_input_buffer()

        if dxl_write(ser, ADDR_USS_ENABLE, bytes([0x03])) is None:
            print("ERROR: No response to WRITE — is the firmware running?")
            return

        time.sleep(0.30)
        data = dxl_read(ser, ADDR_USS_ENABLE, 16)
        dxl_write(ser, ADDR_USS_ENABLE, bytes([0x00]))
        ser.close()

        if not data or len(data) < 16:
            print("ERROR: No diagnostic response from firmware.")
            return

        status = data[1]
        d1 = struct.unpack_from('<H', data, 2)[0]
        d2 = struct.unpack_from('<H', data, 4)[0]
        w1 = struct.unpack_from('<H', data, 6)[0]
        p1 = struct.unpack_from('<H', data, 8)[0]
        f1 = data[10]
        w2 = struct.unpack_from('<H', data, 11)[0]
        p2 = struct.unpack_from('<H', data, 13)[0]
        f2 = data[15]

        print("USS one-shot diagnostic")
        print(f"  status        : 0x{status:02X}")
        print(f"  uss1_mm       : {'TIMEOUT' if (status & 0x10) else d1}")
        print(f"  uss2_mm       : {'TIMEOUT' if (status & 0x20) else d2}")
        print(f"  uss1_wait_us  : {w1}")
        print(f"  uss1_pulse_us : {p1}")
        print(f"  uss1_flags    : 0x{f1:02X}")
        print(f"  uss2_wait_us  : {w2}")
        print(f"  uss2_pulse_us : {p2}")
        print(f"  uss2_flags    : 0x{f2:02X}")
        print("    flags: bit0 rise, bit1 fall, bit2 wait-timeout, bit3 pulse-timeout, bit4 line-high-start")
        return

    # Determine which sensors to test
    sensor_mask = 0x03  # both by default
    if len(sys.argv) > 1:
        for arg in sys.argv[1:]:
            if arg in ("1", "2"):
                s = int(arg)
                if s == 1:
                    sensor_mask = 0x01
                elif s == 2:
                    sensor_mask = 0x02

    labels = []
    if sensor_mask & 0x01:
        labels.append("USS1 (Grove 2, GP2)")
    if sensor_mask & 0x02:
        labels.append("USS2 (Grove 3, GP4)")
    print(f"Testing: {', '.join(labels)}")

    ser = serial.Serial(PORT, timeout=0.1)
    time.sleep(0.1)
    ser.reset_input_buffer()

    # Enable the selected sensor(s)
    resp = dxl_write(ser, ADDR_USS_ENABLE, bytes([sensor_mask]))
    if resp is None:
        print("ERROR: No response to WRITE — is the firmware running?")
        return
    print(f"Enabled sensor(s): mask=0x{sensor_mask:02X}")
    time.sleep(0.2)  # let the first reading complete

    print("\nPress Ctrl+C to stop.\n")
    print(f"{'Time':>8s}  {'Status':>8s}  {'USS1 (mm)':>10s}  {'USS2 (mm)':>10s}")
    print("-" * 44)

    try:
        t0 = time.time()
        while True:
            # Read 6 bytes: USS_ENABLE(1) + USS_STATUS(1) + USS1(2) + USS2(2)
            data = dxl_read(ser, ADDR_USS_ENABLE, 6)
            if data and len(data) >= 6:
                enable = data[0]
                status = data[1]
                d1 = struct.unpack_from('<H', data, 2)[0]
                d2 = struct.unpack_from('<H', data, 4)[0]

                elapsed = time.time() - t0

                d1_str = "---"
                d2_str = "---"
                if sensor_mask & 0x01:
                    if status & 0x10:
                        d1_str = "TIMEOUT"
                    elif status & 0x01:
                        d1_str = f"{d1}"
                    else:
                        d1_str = "wait..."
                if sensor_mask & 0x02:
                    if status & 0x20:
                        d2_str = "TIMEOUT"
                    elif status & 0x02:
                        d2_str = f"{d2}"
                    else:
                        d2_str = "wait..."

                    print(f"{elapsed:7.1f}s  0x{status:02X}      {d1_str:>10s}  {d2_str:>10s}",
                      end='\r')
            else:
                print("  (no response)", end='\r')

            time.sleep(0.15)

    except KeyboardInterrupt:
        print("\n\nStopping...")

    # Disable sensors
    dxl_write(ser, ADDR_USS_ENABLE, bytes([0x00]))
    print("Sensors disabled.")
    ser.close()


if __name__ == '__main__':
    main()
