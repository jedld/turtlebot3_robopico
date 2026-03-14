#!/usr/bin/env python3
"""
Direct-serial heading-hold debugger.
Reads ADDR_DBG_HEADING_ERR, ADDR_HEADING_HOLD_CORR, ADDR_DBG_VEL_L/R,
ADDR_DBG_ENC_TRIM, ADDR_DBG_ENC_DIFF while commanding forward velocity.

Must be run with bringup stopped (it takes exclusive serial access).
"""
import struct
import sys
import time
import serial

# ── Dynamixel protocol constants ─────────────────────────────────────────────
DEV_ID   = 200
DXL_BAUD = 1_000_000
INST_READ  = 0x02
INST_WRITE = 0x03

# Register addresses (must match firmware/main.c)
ADDR_MILLIS         = 10
ADDR_TORQUE_EN      = 149
ADDR_CMD_LINEAR_X   = 150  # int32 — 0.01 m/s
ADDR_CMD_ANGULAR_Z  = 170  # int32 — 0.01 rad/s
ADDR_DBG_VEL_L      = 212  # float32
ADDR_DBG_VEL_R      = 216  # float32
ADDR_HEADING_HOLD_KP  = 280  # float32
ADDR_DBG_HEADING_ERR  = 284  # float32
ADDR_HEADING_HOLD_EN  = 288  # uint8
ADDR_HEADING_HOLD_KI  = 292  # float32
ADDR_HEADING_HOLD_CORR = 296  # float32
ADDR_ENC_TRIM_KP      = 300  # float32
ADDR_DBG_ENC_TRIM     = 304  # float32
ADDR_DBG_ENC_DIFF     = 308  # float32
ADDR_IMU_ANG_VEL_Z    = 68   # float32 — raw gyro Z

# ── CRC ───────────────────────────────────────────────────────────────────────
def _make_crc_table():
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

_CRC_TABLE = _make_crc_table()

def _crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc

def _build_packet(dev_id, inst, params):
    pkt_len = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, dev_id,
                 pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, inst])
    pkt = hdr + params
    crc = _crc16(pkt)
    return pkt + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def _read_response(ser, timeout=0.15):
    buf = b''
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            buf += chunk
            # Check if we have a complete packet
            if len(buf) >= 11:
                pkt_len = buf[5] | (buf[6] << 8)
                total_len = pkt_len + 7
                if len(buf) >= total_len:
                    return buf[:total_len]
    return None

def dxl_write(ser, addr, data):
    params = struct.pack("<H", addr) + data
    pkt = _build_packet(DEV_ID, INST_WRITE, params)
    ser.write(pkt)
    _read_response(ser, timeout=0.05)

def dxl_read_f32(ser, addr):
    params = struct.pack("<HH", addr, 4)
    pkt = _build_packet(DEV_ID, INST_READ, params)
    ser.write(pkt)
    resp = _read_response(ser)
    if resp and len(resp) >= 15:
        err = resp[8]
        if err == 0:
            return struct.unpack("<f", resp[9:13])[0]
    return None

def dxl_read_u8(ser, addr):
    params = struct.pack("<HH", addr, 1)
    pkt = _build_packet(DEV_ID, INST_READ, params)
    ser.write(pkt)
    resp = _read_response(ser)
    if resp and len(resp) >= 12:
        err = resp[8]
        if err == 0:
            return resp[9]
    return None

def dxl_read_i32(ser, addr):
    params = struct.pack("<HH", addr, 4)
    pkt = _build_packet(DEV_ID, INST_READ, params)
    ser.write(pkt)
    resp = _read_response(ser)
    if resp and len(resp) >= 15:
        err = resp[8]
        if err == 0:
            return struct.unpack("<i", resp[9:13])[0]
    return None

def keep_alive(ser):
    """Read ADDR_MILLIS to keep the host-timeout alive."""
    dxl_read_i32(ser, ADDR_MILLIS)

import subprocess, os
BRINGUP_SERVICE = "turtlebot3-bringup.service"

def bringup_is_active():
    try:
        r = subprocess.run(["systemctl", "is-active", BRINGUP_SERVICE],
                           capture_output=True, text=True)
        return r.stdout.strip() == "active"
    except:
        return False

def bringup_stop():
    subprocess.run(["sudo", "systemctl", "stop", BRINGUP_SERVICE],
                   capture_output=True)
    time.sleep(1.0)
    return not bringup_is_active()

def bringup_start():
    subprocess.run(["sudo", "systemctl", "start", BRINGUP_SERVICE],
                   capture_output=True)
    time.sleep(2.0)

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Debug heading hold via direct serial")
    parser.add_argument("--speed", type=float, default=0.10, help="forward speed m/s")
    parser.add_argument("--duration", type=float, default=3.0, help="drive duration seconds")
    parser.add_argument("--port", default="/dev/ttyTB3")
    args = parser.parse_args()

    stopped_bringup = False
    if bringup_is_active():
        print("Stopping bringup ...", flush=True)
        bringup_stop()
        stopped_bringup = True
        print("  stopped.")

    try:
        ser = serial.Serial(args.port, DXL_BAUD, timeout=0.05)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.5)
        ser.reset_input_buffer()

        # Ping
        print("Pinging Pico ...", end=" ", flush=True)
        millis = None
        for _ in range(20):
            millis = dxl_read_i32(ser, ADDR_MILLIS)
            if millis is not None:
                break
            time.sleep(0.5)
        if millis is None:
            print("FAILED")
            return 1
        print(f"OK (uptime {millis} ms)")

        # Read heading hold parameters
        hh_en = dxl_read_u8(ser, ADDR_HEADING_HOLD_EN)
        hh_kp = dxl_read_f32(ser, ADDR_HEADING_HOLD_KP)
        hh_ki = dxl_read_f32(ser, ADDR_HEADING_HOLD_KI)
        enc_kp = dxl_read_f32(ser, ADDR_ENC_TRIM_KP)
        print(f"\nHeading Hold: EN={hh_en}  Kp={hh_kp}  Ki={hh_ki}")
        print(f"Enc Trim:     Kp={enc_kp}")
        print(f"Gyro-Z at rest: {dxl_read_f32(ser, ADDR_IMU_ANG_VEL_Z):.6f} rad/s")

        # Enable torque
        dxl_write(ser, ADDR_TORQUE_EN, bytes([1]))
        print(f"\nDriving at {args.speed:.2f} m/s for {args.duration:.1f}s ...")
        print(f"{'t':>5s}  {'gyro_z':>8s}  {'hd_err':>8s}  {'hd_corr':>8s}  "
              f"{'vel_L':>8s}  {'vel_R':>8s}  {'enc_trim':>8s}  {'enc_diff':>8s}")
        print("-" * 80)

        lin_raw = struct.pack("<i", int(round(args.speed * 100)))
        ang_raw = struct.pack("<i", 0)

        t0 = time.time()
        while time.time() - t0 < args.duration:
            # Command velocity
            dxl_write(ser, ADDR_CMD_LINEAR_X, lin_raw)
            dxl_write(ser, ADDR_CMD_ANGULAR_Z, ang_raw)
            keep_alive(ser)

            # Read debug registers
            gyro_z = dxl_read_f32(ser, ADDR_IMU_ANG_VEL_Z) or 0.0
            hd_err = dxl_read_f32(ser, ADDR_DBG_HEADING_ERR) or 0.0
            hd_corr = dxl_read_f32(ser, ADDR_HEADING_HOLD_CORR) or 0.0
            vel_l = dxl_read_f32(ser, ADDR_DBG_VEL_L) or 0.0
            vel_r = dxl_read_f32(ser, ADDR_DBG_VEL_R) or 0.0
            enc_trim = dxl_read_f32(ser, ADDR_DBG_ENC_TRIM) or 0.0
            enc_diff = dxl_read_f32(ser, ADDR_DBG_ENC_DIFF) or 0.0

            t_now = time.time() - t0
            print(f"{t_now:5.2f}  {gyro_z:+8.4f}  {hd_err:+8.4f}  {hd_corr:+8.4f}  "
                  f"{vel_l:+8.4f}  {vel_r:+8.4f}  {enc_trim:+8.4f}  {enc_diff:+8.6f}")

            time.sleep(0.1)

        # Stop
        dxl_write(ser, ADDR_CMD_LINEAR_X, struct.pack("<i", 0))
        dxl_write(ser, ADDR_CMD_ANGULAR_Z, struct.pack("<i", 0))
        keep_alive(ser)
        time.sleep(0.5)

        # Final state
        print("\n--- Final state ---")
        hd_err = dxl_read_f32(ser, ADDR_DBG_HEADING_ERR) or 0.0
        hd_corr = dxl_read_f32(ser, ADDR_HEADING_HOLD_CORR) or 0.0
        enc_diff = dxl_read_f32(ser, ADDR_DBG_ENC_DIFF) or 0.0
        print(f"  Heading err:  {hd_err:+.6f} rad ({hd_err*180/3.14159:.2f}°)")
        print(f"  Correction:   {hd_corr:+.6f} rad/s")
        print(f"  Enc diff:     {enc_diff:+.6f} m")

        ser.close()

    finally:
        if stopped_bringup:
            print("\nRestarting bringup ...")
            bringup_start()
            print("  bringup restarted.")

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
