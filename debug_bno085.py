#!/usr/bin/env python3
"""
debug_bno085.py — BNO085 init diagnostics via Dynamixel Protocol 2.0

Reads the debug registers written by the firmware at boot to report exactly
why the BNO085 failed to initialise (or confirm it succeeded).

IMPORTANT: Run this WITHOUT bringup active — the script needs exclusive access
to the serial port.

Usage:
    python3 debug_bno085.py [--port /dev/ttyACM0]
    python3 debug_bno085.py --raw      # also dump raw IMU register values

Exit codes:
    0  BNO085 detected and registers look healthy
    1  BNO085 not detected (see output for reason)
    2  Could not open serial port / read registers
"""

import serial
import struct
import time
import sys
import argparse

DEV_ID  = 200
PORT_DEFAULT = "/dev/ttyACM0"

# ── register addresses (must match main.c) ────────────────────────────────────
ADDR_IMU_ANG_VEL_X  = 60
ADDR_IMU_LIN_ACC_X  = 72
ADDR_IMU_MAG_X      = 84
ADDR_IMU_ORIENT_W   = 96

ADDR_DBG_IMU_SOURCE = 174
ADDR_DBG_BNO085_RC  = 175
ADDR_DBG_I2C0_NDEV  = 176
ADDR_DBG_I2C0_DEV0  = 177   # 7 consecutive bytes

IMU_SOURCE_NAMES = {0: "Simulated (fallback)", 1: "BNO085", 2: "BNO055", 0xFF: "(not set yet)"}
BNO085_RC_NAMES  = {
    0:    "OK — init succeeded",
    1:    "SHTP timeout at 0x4A (SA0→GND) — sensor may not be responding",
    2:    "SHTP timeout at BOTH 0x4A and 0x4B — sensor unreachable",
    3:    "I2C scan found NO device at 0x4A or 0x4B — wiring fault",
    0xFF: "(not set — firmware still booting?)",
}

RED  = "\033[0;31m"
GRN  = "\033[0;32m"
YLW  = "\033[1;33m"
CYN  = "\033[0;36m"
BLD  = "\033[1m"
NC   = "\033[0m"

def ok(msg):   print(f"{GRN}[PASS]{NC}  {msg}")
def fail(msg): print(f"{RED}[FAIL]{NC}  {msg}")
def info(msg): print(f"{CYN}[INFO]{NC}  {msg}")
def warn(msg): print(f"{YLW}[WARN]{NC}  {msg}")

# ── Dynamixel Protocol 2.0 helpers ───────────────────────────────────────────

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

def build_read(dev_id, addr, length):
    params = struct.pack('<HH', addr, length)
    pkt_len = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, dev_id,
                 pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, 0x02])
    pkt = hdr + params
    crc = dxl_crc16(pkt)
    return pkt + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def read_response(ser, timeout=1.0):
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

def do_read(ser, addr, length):
    pkt = build_read(DEV_ID, addr, length)
    ser.reset_input_buffer()
    ser.write(pkt)
    resp = read_response(ser)
    if resp is None:
        return None
    if len(resp) < 11:
        return None
    crc_calc = dxl_crc16(resp[:-2])
    crc_pkt  = resp[-2] | (resp[-1] << 8)
    if crc_calc != crc_pkt:
        warn(f"CRC mismatch at addr {addr}")
        return None
    return resp[9:-2]

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="BNO085 firmware debug register reader")
    ap.add_argument("--port",    default=PORT_DEFAULT, help=f"Serial port (default {PORT_DEFAULT})")
    ap.add_argument("--raw",     action="store_true",  help="Also dump raw IMU register values")
    ap.add_argument("--timeout", type=float, default=1.0, help="Per-read timeout in seconds")
    args = ap.parse_args()

    print()
    print(f"{BLD}{'='*60}{NC}")
    print(f"{BLD}  BNO085 Firmware Debug  ({args.port}){NC}")
    print(f"{BLD}  Stop bringup before running this script{NC}")
    print(f"{BLD}{'='*60}{NC}")
    print()

    try:
        ser = serial.Serial(args.port, 115200, timeout=0.1)
        time.sleep(0.1)
    except serial.SerialException as e:
        print(f"{RED}[ERROR]{NC} Cannot open {args.port}: {e}")
        print("        Is bringup still running? (it holds the port)")
        sys.exit(2)

    # ── Read debug registers ──────────────────────────────────────────────────
    info(f"Reading debug registers from device {DEV_ID} …")
    time.sleep(0.05)

    dbg_block = do_read(ser, ADDR_DBG_IMU_SOURCE, 10)  # source + rc + ndev + 7 addrs
    if dbg_block is None or len(dbg_block) < 10:
        print(f"{RED}[ERROR]{NC} No response — is the Pico powered and running the new firmware?")
        print("        Flash firmware/build/turtlebot3_pico_fw.uf2 first.")
        ser.close()
        sys.exit(2)

    imu_source = dbg_block[0]
    bno085_rc  = dbg_block[1]
    i2c0_ndev  = dbg_block[2]
    i2c0_devs  = [dbg_block[3 + i] for i in range(7) if dbg_block[3 + i] != 0xFF]

    print(f"{BLD}── IMU Source ───────────────────────────────────────────────{NC}")
    src_name = IMU_SOURCE_NAMES.get(imu_source, f"Unknown (0x{imu_source:02X})")
    if imu_source == 1:
        ok(f"IMU source = {src_name}")
    elif imu_source == 0:
        fail(f"IMU source = {src_name}")
    else:
        warn(f"IMU source = {src_name}")

    print()
    print(f"{BLD}── BNO085 Init Result ───────────────────────────────────────{NC}")
    rc_name = BNO085_RC_NAMES.get(bno085_rc, f"Unknown code 0x{bno085_rc:02X}")
    if bno085_rc == 0:
        ok(f"BNO085 init code = 0 ({rc_name})")
    else:
        fail(f"BNO085 init code = {bno085_rc}  →  {rc_name}")

    print()
    print(f"{BLD}── I2C0 Bus Scan (GP0=SDA / GP1=SCL) ───────────────────────{NC}")
    info(f"Devices found on I2C0 during boot: {i2c0_ndev}")

    if i2c0_ndev == 0:
        fail("No I2C devices were detected on GP0/GP1 at all.  Check:")
        print("      1. STEMMA QT cable fully seated at both ends")
        print("      2. Grove 1 port used (not Grove 2, 3 … )")
        print("      3. Cable not damaged / miswired")
        print("      4. BNO085 VIN connected to 3.3V (red wire on Grove)")
    else:
        for addr in i2c0_devs:
            tag = ""
            if addr == 0x4A:   tag = "  ← BNO085 (SA0→GND, expected)"
            elif addr == 0x4B: tag = "  ← BNO085 (SA0→VCC)"
            info(f"  0x{addr:02X} ({addr:3d}){tag}")

        bno085_found_on_bus = any(a in (0x4A, 0x4B) for a in i2c0_devs)
        if bno085_found_on_bus:
            ok("BNO085 I2C address IS visible on the bus.")
            if bno085_rc != 0:
                warn("Sensor is on the bus but SHTP timed out.  Possible causes:")
                print("      • Power was applied too quickly (BNO085 needs >150 ms to boot)")
                print("      • I2C pullups too weak (check 4.7 kΩ on SDA/SCL — Adafruit board has them)")
                print("      • Sensor in a bad state — try a hard power cycle (not just reset)")
        else:
            fail(f"BNO085 not found at 0x4A or 0x4B.  Found instead: "
                 f"{[hex(a) for a in i2c0_devs]}")
            print("      Verify SA0 pin connection — it determines the I2C address:")
            print("      SA0 tied to GND → 0x4A   |   SA0 tied to VCC → 0x4B")
            print("      (On the Adafruit board SA0 is the PS1 pad, default floating → 0x4A)")

    # ── optional raw IMU dump ─────────────────────────────────────────────────
    if args.raw:
        print()
        print(f"{BLD}── Raw IMU Register Dump ────────────────────────────────────{NC}")

        av = do_read(ser, ADDR_IMU_ANG_VEL_X, 12)
        la = do_read(ser, ADDR_IMU_LIN_ACC_X, 12)
        mg = do_read(ser, ADDR_IMU_MAG_X,     12)
        qw = do_read(ser, ADDR_IMU_ORIENT_W,  16)

        def f32x3(data):
            if data and len(data) >= 12:
                return struct.unpack('<fff', data[:12])
            return (None, None, None)

        def f32x4(data):
            if data and len(data) >= 16:
                return struct.unpack('<ffff', data[:16])
            return (None, None, None, None)

        import math
        ax, ay, az = f32x3(av)
        lx, ly, lz = f32x3(la)
        mx, my, mz = f32x3(mg)
        qW, qX, qY, qZ = f32x4(qw)

        if ax is not None:
            info(f"  angular_vel  x={ax:+.4f}  y={ay:+.4f}  z={az:+.4f}  rad/s")
        if lx is not None:
            g_mag = math.sqrt(lx**2 + ly**2 + lz**2)
            info(f"  linear_accel x={lx:+.4f}  y={ly:+.4f}  z={lz:+.4f}  m/s²  (|g|={g_mag:.3f})")
        if mx is not None:
            b_ut = math.sqrt(mx**2 + my**2 + mz**2) * 1e6
            info(f"  mag_field    x={mx:+.6f}  y={my:+.6f}  z={mz:+.6f}  T  (|B|={b_ut:.2f} µT)")
            if b_ut < 1.0:
                fail(f"  Mag field ≈ 0 µT — report ID 0x03 may not be arriving (check firmware)")
            elif 25.0 <= b_ut <= 65.0:
                ok(f"  |mag| = {b_ut:.2f} µT — within Earth's typical range (25–65 µT)")
            else:
                warn(f"  |mag| = {b_ut:.2f} µT — outside typical Earth range (25–65 µT)")
        if qW is not None:
            info(f"  orientation  w={qW:+.4f}  x={qX:+.4f}  y={qY:+.4f}  z={qZ:+.4f}")

        # Check if everything is constant-zero (simulated)
        if ax == 0.0 and ay == 0.0 and az == 0.0 and lx == 0.0 and ly == 0.0:
            warn("  All angular_vel and linear_accel (x,y) are exactly 0.0 → simulated fallback")
        elif lz is not None and 9.0 < abs(lz) < 10.6:
            ok(f"  |linear_accel z| ≈ 9.81 m/s² consistent with real gravity")

    ser.close()
    print()

    if imu_source == 1:
        print(f"{GRN}{BLD}BNO085 is initialised and active.{NC}")
        sys.exit(0)
    else:
        print(f"{RED}{BLD}BNO085 NOT active — see failure details above.{NC}")
        sys.exit(1)


if __name__ == "__main__":
    main()
