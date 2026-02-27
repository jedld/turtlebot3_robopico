#!/usr/bin/env python3
"""
Diagnostic: compare bulk READ (172 bytes from addr 10, exactly like ROS node)
with individual 4-byte READs of position registers.

Also writes a velocity command to spin motors, waits, then checks positions.
"""

import serial, struct, time, sys

PORT = "/dev/ttyACM1"
DEV_ID = 200

# CRC-16 table matching firmware (poly 0x8005, MSB-first)
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
    length = len(params) + 3  # instruction + CRC(2)
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, dev_id,
                 length & 0xFF, (length >> 8) & 0xFF, instruction])
    pkt = hdr + params
    crc = dxl_crc16(pkt)
    return pkt + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def read_response(ser, timeout=1.0):
    """Read a Dynamixel Protocol 2.0 STATUS packet."""
    start = time.time()
    buf = b''
    while time.time() - start < timeout:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
        # Look for header
        idx = buf.find(b'\xFF\xFF\xFD\x00')
        if idx >= 0:
            buf = buf[idx:]
            if len(buf) >= 7:
                pkt_len = buf[5] | (buf[6] << 8)
                total = 7 + pkt_len
                if len(buf) >= total:
                    pkt = buf[:total]
                    return pkt
    return None


def parse_status(pkt):
    """Parse STATUS packet, return (error, data_bytes) or None."""
    if pkt is None or len(pkt) < 11:
        return None
    pkt_len = pkt[5] | (pkt[6] << 8)
    error = pkt[8]
    data = pkt[9:-2]  # everything between error and CRC
    # Verify CRC
    crc_calc = dxl_crc16(pkt[:-2])
    crc_pkt = pkt[-2] | (pkt[-1] << 8)
    if crc_calc != crc_pkt:
        print(f"  CRC MISMATCH: calc=0x{crc_calc:04X}, pkt=0x{crc_pkt:04X}")
        return None
    return (error, data)


def do_read(ser, addr, length):
    """Send a READ instruction, return (error, data) or None."""
    params = struct.pack('<HH', addr, length)
    pkt = build_packet(DEV_ID, 0x02, params)
    ser.reset_input_buffer()
    ser.write(pkt)
    resp = read_response(ser)
    return parse_status(resp)


def do_write(ser, addr, data_bytes):
    """Send a WRITE instruction."""
    params = struct.pack('<H', addr) + data_bytes
    pkt = build_packet(DEV_ID, 0x03, params)
    ser.reset_input_buffer()
    ser.write(pkt)
    resp = read_response(ser)
    return parse_status(resp)


def main():
    ser = serial.Serial(PORT, 115200, timeout=0.1)
    time.sleep(0.2)
    ser.reset_input_buffer()

    print("=" * 60)
    print("DIAGNOSTIC: Bulk READ vs Individual READs")
    print("=" * 60)

    # 1. PING
    pkt = build_packet(DEV_ID, 0x01)
    ser.reset_input_buffer()
    ser.write(pkt)
    resp = read_response(ser)
    result = parse_status(resp)
    if result:
        print(f"PING OK, error=0x{result[0]:02X}, data={result[1].hex()}")
    else:
        print("PING FAILED!")
        ser.close()
        sys.exit(1)

    # 2. Enable torque
    print("\nEnabling torque (addr 149 = 1)...")
    result = do_write(ser, 149, bytes([1]))
    print(f"  Torque write: error=0x{result[0]:02X}" if result else "  FAILED")

    # 3. Write velocity: linear.x = 10 (0.10 m/s * 100), angular.z = 0
    print("\nWriting velocity: linear_x=10 (0.10 m/s)...")
    vel_data = struct.pack('<i', 10) + b'\x00' * 16 + struct.pack('<i', 0)
    result = do_write(ser, 150, vel_data)
    print(f"  Velocity write: error=0x{result[0]:02X}" if result else "  FAILED")

    # Wait for positions to accumulate
    print("\nWaiting 2 seconds for positions to accumulate...")
    time.sleep(2)

    # 4. Individual 4-byte READs
    print("\n--- Individual READs ---")
    for name, addr in [("present_vel_L", 128), ("present_vel_R", 132),
                        ("present_pos_L", 136), ("present_pos_R", 140),
                        ("millis", 10)]:
        result = do_read(ser, addr, 4)
        if result:
            val = struct.unpack('<i', result[1])[0]
            print(f"  {name:16s} (addr {addr:3d}): {val}")
        else:
            print(f"  {name:16s} (addr {addr:3d}): READ FAILED")

    # 5. Bulk READ: addr 10, length 172 (exactly like ROS node)
    print("\n--- Bulk READ (addr=10, len=172) ---")
    result = do_read(ser, 10, 172)
    if result:
        error, data = result
        print(f"  Error: 0x{error:02X}, Data length: {len(data)}")
        if len(data) == 172:
            # Extract specific fields
            def extract_i32(offset):
                return struct.unpack('<i', data[offset:offset+4])[0]
            
            millis_val = extract_i32(0)    # addr 10 - offset 0
            vel_l = extract_i32(118)       # addr 128 - offset 118
            vel_r = extract_i32(122)       # addr 132 - offset 122
            pos_l = extract_i32(126)       # addr 136 - offset 126
            pos_r = extract_i32(130)       # addr 140 - offset 130
            torque = data[139]             # addr 149 - offset 139
            cmd_lin_x = extract_i32(140)   # addr 150 - offset 140
            cmd_ang_z = extract_i32(160)   # addr 170 - offset 160
            
            print(f"  millis:       {millis_val}")
            print(f"  vel_L:        {vel_l}")
            print(f"  vel_R:        {vel_r}")
            print(f"  pos_L:        {pos_l}")
            print(f"  pos_R:        {pos_r}")
            print(f"  torque_en:    {torque}")
            print(f"  cmd_lin_x:    {cmd_lin_x}")
            print(f"  cmd_ang_z:    {cmd_ang_z}")
            
            # Also hex dump around the position area
            print(f"\n  Hex dump bytes[118..143] (vel+pos area):")
            segment = data[118:144]
            print(f"    {segment.hex(' ')}")
            
            # Check for FF FF FD pattern anywhere in data
            for i in range(len(data) - 2):
                if data[i] == 0xFF and data[i+1] == 0xFF and data[i+2] == 0xFD:
                    print(f"  WARNING: Found FF FF FD at offset {i} (addr {10+i})")
        else:
            print(f"  WRONG DATA LENGTH: expected 172, got {len(data)}")
            print(f"  Raw hex: {data.hex(' ')}")
    else:
        print("  BULK READ FAILED!")

    # 6. Stop motors
    print("\nStopping motors...")
    vel_data = struct.pack('<i', 0) + b'\x00' * 16 + struct.pack('<i', 0)
    do_write(ser, 150, vel_data)
    
    ser.close()
    print("\nDone.")


if __name__ == "__main__":
    main()
