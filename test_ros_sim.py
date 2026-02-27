#!/usr/bin/env python3
"""
Simulate the exact ROS turtlebot3_node traffic pattern:
- Bulk READ (172 bytes from addr 10) every 50ms
- Heartbeat WRITE (1 byte to addr 19) every 100ms
- Velocity WRITE (24 bytes to addr 150) once at start

Then read diagnostic counters to verify nothing is dropped.
"""
import serial, struct, time, sys, threading

PORT = "/dev/ttyACM1"
DEV_ID = 200

CRC_TABLE = []
for i in range(256):
    crc = i << 8
    for _ in range(8):
        crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    CRC_TABLE.append(crc)

def dxl_crc16(data):
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc

def build_pkt(dev_id, inst, params=b''):
    length = len(params) + 3
    hdr = bytes([0xFF,0xFF,0xFD,0x00,dev_id,length&0xFF,(length>>8)&0xFF,inst])
    pkt = hdr + params
    crc = dxl_crc16(pkt)
    return pkt + bytes([crc&0xFF,(crc>>8)&0xFF])

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

def parse_status(pkt):
    if pkt is None or len(pkt) < 11:
        return None
    error = pkt[8]
    data = pkt[9:-2]
    crc_calc = dxl_crc16(pkt[:-2])
    crc_pkt = pkt[-2] | (pkt[-1] << 8)
    if crc_calc != crc_pkt:
        return None  # CRC fail
    return (error, data)

def do_read(ser, addr, length):
    params = struct.pack('<HH', addr, length)
    pkt = build_pkt(DEV_ID, 0x02, params)
    ser.write(pkt)
    resp = read_response(ser)
    return parse_status(resp)

def do_write(ser, addr, data_bytes):
    params = struct.pack('<H', addr) + data_bytes
    pkt = build_pkt(DEV_ID, 0x03, params)
    ser.write(pkt)
    resp = read_response(ser, timeout=0.2)
    return parse_status(resp)

def main():
    # Find correct port
    port = PORT
    for p in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"]:
        try:
            import subprocess
            result = subprocess.run(["udevadm", "info", "--query=property", p],
                                    capture_output=True, text=True, timeout=2)
            if "TurtleBot3_Pico_OpenCR" in result.stdout:
                port = p
                break
        except:
            pass

    ser = serial.Serial(port, 1000000, timeout=0.1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    print(f"Using port: {port}")
    print("=" * 60)

    # PING
    pkt = build_pkt(DEV_ID, 0x01)
    ser.write(pkt)
    resp = read_response(ser)
    result = parse_status(resp)
    if not result:
        print("PING FAILED!")
        ser.close()
        sys.exit(1)
    print(f"PING OK (error=0x{result[0]:02X})")

    # Enable torque
    do_write(ser, 149, bytes([1]))
    print("Torque enabled")

    # Write velocity: linear_x = 5 (0.05 m/s), angular_z = 0
    vel_data = struct.pack('<i', 5) + b'\x00' * 16 + struct.pack('<i', 0)
    result = do_write(ser, 150, vel_data)
    print(f"Velocity WRITE (lin_x=5): {'OK' if result else 'FAIL'}")

    # Now simulate the ROS node: alternate READs every 50ms, hearts every 100ms
    read_ok = 0
    read_fail = 0
    hb_count = 0
    iteration = 0
    test_duration = 5.0  # seconds

    print(f"\nSimulating ROS node for {test_duration}s...")
    print("  (READ every 50ms, heartbeat every 100ms)")

    start = time.time()
    next_read = start
    next_hb = start + 0.1
    while time.time() - start < test_duration:
        now = time.time()

        # Bulk READ (addr=10, len=172) — same as ROS node
        if now >= next_read:
            next_read = now + 0.05
            ser.reset_input_buffer()
            result = do_read(ser, 10, 172)
            if result:
                read_ok += 1
                if read_ok == 1 or read_ok % 20 == 0:
                    error, data = result
                    if len(data) == 172:
                        vel_l = struct.unpack('<i', data[118:122])[0]
                        vel_r = struct.unpack('<i', data[122:126])[0]
                        pos_l = struct.unpack('<i', data[126:130])[0]
                        pos_r = struct.unpack('<i', data[130:134])[0]
                        print(f"  READ #{read_ok}: vel_L={vel_l} vel_R={vel_r} pos_L={pos_l} pos_R={pos_r}")
            else:
                read_fail += 1

        # Heartbeat WRITE (addr=19)
        if now >= next_hb:
            next_hb = now + 0.1
            hb_count += 1
            do_write(ser, 19, bytes([hb_count & 0xFF]))

        time.sleep(0.005)

    print(f"\nResults after {test_duration}s:")
    print(f"  READ OK:   {read_ok}")
    print(f"  READ FAIL: {read_fail}")
    print(f"  Heartbeats: {hb_count}")

    # Read diagnostic counters (addr 240-255)
    print("\n--- Firmware Diagnostic Counters ---")
    ser.reset_input_buffer()
    time.sleep(0.1)
    result = do_read(ser, 240, 16)
    if result:
        error, data = result
        if len(data) == 16:
            pkt_count  = struct.unpack('<I', data[0:4])[0]
            crc_fail   = struct.unpack('<I', data[4:8])[0]
            vel_writes = struct.unpack('<I', data[8:12])[0]
            read_count = struct.unpack('<I', data[12:16])[0]
            print(f"  Packets dispatched: {pkt_count}")
            print(f"  CRC failures:       {crc_fail}")
            print(f"  Velocity writes:    {vel_writes}")
            print(f"  READ instructions:  {read_count}")
        else:
            print(f"  Wrong data length: {len(data)}")
    else:
        print("  Failed to read diagnostic counters!")

    # Read final positions
    print("\n--- Final Position Registers ---")
    result = do_read(ser, 128, 16)
    if result:
        error, data = result
        if len(data) == 16:
            vel_l = struct.unpack('<i', data[0:4])[0]
            vel_r = struct.unpack('<i', data[4:8])[0]
            pos_l = struct.unpack('<i', data[8:12])[0]
            pos_r = struct.unpack('<i', data[12:16])[0]
            print(f"  vel_L={vel_l} vel_R={vel_r}")
            print(f"  pos_L={pos_l} pos_R={pos_r}")

    # Stop motors
    vel_data = struct.pack('<i', 0) + b'\x00' * 16 + struct.pack('<i', 0)
    do_write(ser, 150, vel_data)
    print("\nMotors stopped.")

    ser.close()

if __name__ == "__main__":
    main()
