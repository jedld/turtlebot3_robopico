"""dxl_utils.py — Shared Dynamixel v2 protocol helpers for TurtleBot3 Pico scripts.

Provides CRC computation, packet building, response parsing, register
read/write helpers, calibration queries, and the firmware register map.
"""
from __future__ import annotations

import math
import struct
import time
from typing import Optional

# ── Protocol constants ─────────────────────────────────────────────────────
DEV_ID   = 200
DXL_ID   = DEV_ID          # alias used by some scripts
DXL_BAUD = 1_000_000
BAUD     = DXL_BAUD        # alias
DXL_PORTS = ("/dev/ttyTB3", "/dev/ttyACM0", "/dev/ttyACM1")

INST_READ        = 0x02
INST_WRITE       = 0x03
INST_CALIBRATION = 0x90
CALIB_CMD_GET    = 0x02

BRINGUP_SERVICE = "turtlebot3-bringup.service"
BRINGUP_SVC     = BRINGUP_SERVICE   # alias

# ── Firmware register map (must match firmware/main.c) ─────────────────────
REG_MILLIS              = 10
REG_IMU_ANG_VEL_Z       = 68
REG_IMU_ORIENT_W        = 96
REG_IMU_ORIENT_X        = 100
REG_IMU_ORIENT_Y        = 104
REG_IMU_ORIENT_Z        = 108
REG_GAME_ROT_W          = 368   # Game Rotation Vector (gyro+accel, no mag)
REG_GAME_ROT_X          = 372
REG_GAME_ROT_Y          = 376
REG_GAME_ROT_Z          = 380
REG_PRESENT_VEL_L       = 128
REG_PRESENT_VEL_R       = 132
REG_PRESENT_POS_L       = 136
REG_PRESENT_POS_R       = 140
REG_TORQUE_EN           = 149
REG_CMD_LINEAR_X        = 150
REG_CMD_ANGULAR_Z       = 170
REG_DBG_IMU_SOURCE      = 174
REG_ENC_L               = 184
REG_ENC_R               = 188
REG_ENC_RESET           = 192
REG_PID_KP              = 200
REG_PID_KI              = 204
REG_PID_KD              = 208
REG_DBG_VEL_L           = 212
REG_DBG_VEL_R           = 216
REG_I_SEED_FWD          = 256
REG_I_SEED_REV          = 260
REG_HDG_ITERM           = 264
REG_VEL_TRIM_FWD        = 268
REG_VEL_TRIM_REV        = 272
REG_HEADING_HOLD_KP     = 280
REG_DBG_HEADING_ERR     = 284
REG_HEADING_HOLD_EN     = 288
REG_HEADING_HOLD_KI     = 292
REG_HEADING_HOLD_CORR   = 296
REG_ENC_TRIM_KP         = 300
REG_DBG_ENC_TRIM        = 304
REG_DBG_ENC_DIFF        = 308
REG_HEADING_HOLD_KD     = 312
REG_IMU_HEADING_ALPHA   = 316
REG_IMU_HEADING_EN      = 320
REG_DBG_HEADING_FUSED   = 324
REG_DBG_HEADING_GYRO    = 328
REG_DBG_HEADING_ENC     = 332
REG_IMU_HEADING_BIAS_BETA = 336
REG_DBG_GYRO_BIAS       = 340

# ── Register aliases (backward compatibility) ──────────────────────────────
# ADDR_* naming used by Style-A sweep/test scripts
ADDR_MILLIS             = REG_MILLIS
ADDR_TORQUE_EN          = REG_TORQUE_EN
ADDR_CMD_LINEAR_X       = REG_CMD_LINEAR_X
ADDR_CMD_ANGULAR_Z      = REG_CMD_ANGULAR_Z
ADDR_ENC_L              = REG_ENC_L
ADDR_ENC_R              = REG_ENC_R
ADDR_ENC_RESET          = REG_ENC_RESET
ADDR_DBG_VEL_L          = REG_DBG_VEL_L
ADDR_DBG_VEL_R          = REG_DBG_VEL_R
ADDR_I_SEED_FWD         = REG_I_SEED_FWD
ADDR_I_SEED_REV         = REG_I_SEED_REV
ADDR_HDG_ITERM          = REG_HDG_ITERM
ADDR_VEL_TRIM_FWD       = REG_VEL_TRIM_FWD
ADDR_VEL_TRIM_REV       = REG_VEL_TRIM_REV
ADDR_HEADING_HOLD_KP    = REG_HEADING_HOLD_KP
ADDR_DBG_HEADING_ERR    = REG_DBG_HEADING_ERR
ADDR_HEADING_HOLD_EN    = REG_HEADING_HOLD_EN
ADDR_HEADING_HOLD_KI    = REG_HEADING_HOLD_KI
ADDR_HEADING_HOLD_CORR  = REG_HEADING_HOLD_CORR
ADDR_DBG_ENC_DIFF       = REG_DBG_ENC_DIFF
ADDR_HEADING_HOLD_I_SEED_FWD = REG_I_SEED_FWD
# Alternate names used by some scripts
REG_MOTOR_TORQUE        = REG_TORQUE_EN
REG_ENC_L_COUNT         = REG_ENC_L
REG_ENC_R_COUNT         = REG_ENC_R
REG_HEADING_CORR        = REG_HEADING_HOLD_CORR
REG_HDG_I_SEED_FWD      = REG_I_SEED_FWD
REG_HDG_I_SEED_REV      = REG_I_SEED_REV

# ── Robot geometry ─────────────────────────────────────────────────────────
WHEEL_RADIUS_M     = 0.037009   # calibrated 2026-03-23 (was 0.03405)
ENC_COUNTS_PER_REV = 1936.0
ENC_M_PER_COUNT    = (2.0 * math.pi * WHEEL_RADIUS_M) / ENC_COUNTS_PER_REV

# ── CRC-16 (Dynamixel v2 / BUYPASS variant) ───────────────────────────────
_CRC_TABLE: list[int] = []
for _i in range(256):
    _c = _i << 8
    for _ in range(8):
        _c = ((_c << 1) ^ 0x8005) & 0xFFFF if (_c & 0x8000) else (_c << 1) & 0xFFFF
    _CRC_TABLE.append(_c)


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


# ── Packet builders ────────────────────────────────────────────────────────
def build_packet(dev_id: int, inst: int, params: bytes) -> bytes:
    """Build a Dynamixel v2 instruction packet."""
    pkt_len = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, dev_id,
                 pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, inst])
    pkt = hdr + params
    crc = crc16(pkt)
    return pkt + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def make_read_pkt(addr: int, length: int, dev_id: int = DEV_ID) -> bytes:
    """Build a Dynamixel v2 READ instruction packet."""
    return build_packet(dev_id, INST_READ, struct.pack("<HH", addr, length))


def make_write_pkt(addr: int, payload: bytes, dev_id: int = DEV_ID) -> bytes:
    """Build a Dynamixel v2 WRITE instruction packet."""
    return build_packet(dev_id, INST_WRITE, struct.pack("<H", addr) + payload)


def make_write_i32_pkt(addr: int, value: int, dev_id: int = DEV_ID) -> bytes:
    return make_write_pkt(addr, struct.pack("<i", value), dev_id)


def make_write_f32_pkt(addr: int, value: float, dev_id: int = DEV_ID) -> bytes:
    return make_write_pkt(addr, struct.pack("<f", value), dev_id)


def make_write_u8_pkt(addr: int, value: int, dev_id: int = DEV_ID) -> bytes:
    return make_write_pkt(addr, bytes([value & 0xFF]), dev_id)


# ── Response parsers ───────────────────────────────────────────────────────
def read_response_raw(ser, timeout: float = 0.15) -> Optional[bytes]:
    """Read a Dynamixel v2 status packet (returns full raw packet or None).

    Simple parser suitable for point-to-point communication where only one
    device responds at a time.
    """
    buf = b''
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            buf += chunk
            if len(buf) >= 7:
                pkt_len = buf[5] | (buf[6] << 8)
                if len(buf) >= 7 + pkt_len:
                    return buf[:7 + pkt_len]
    return None


def read_response(ser, expected: int, timeout: float = 0.25,
                  dev_id: int = DEV_ID) -> Optional[bytes]:
    """Read and validate a Dynamixel v2 status packet.

    Returns just the payload bytes (*expected* count) or None.
    Validates header, device ID, instruction byte (0x55), and payload size.
    Tolerant of interleaved / corrupted traffic on a shared bus.
    """
    HEADER = b"\xFF\xFF\xFD\x00"
    t0 = time.monotonic()
    buf = b""
    while time.monotonic() - t0 < timeout:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
        while True:
            idx = buf.find(HEADER)
            if idx < 0:
                buf = buf[-3:] if len(buf) >= 3 else buf
                break
            buf = buf[idx:]
            if len(buf) < 9:
                break
            if buf[4] != dev_id or buf[7] != 0x55:
                buf = buf[4:]
                continue
            plen = buf[5] | (buf[6] << 8)
            num_params = plen - 4
            total = 7 + plen
            if len(buf) < total:
                break
            if num_params != expected:
                buf = buf[4:]
                continue
            return buf[9:9 + expected]
    return None


# ── Register helpers ───────────────────────────────────────────────────────
def dxl_write(ser, addr: int, data: bytes, dev_id: int = DEV_ID) -> None:
    """Write raw bytes to a Dynamixel register."""
    ser.reset_input_buffer()
    ser.write(build_packet(dev_id, INST_WRITE, struct.pack("<H", addr) + data))
    read_response_raw(ser, timeout=0.05)


def dxl_read_f32(ser, addr: int, dev_id: int = DEV_ID) -> Optional[float]:
    """Read a 32-bit float register, returning None on failure."""
    ser.reset_input_buffer()
    ser.write(build_packet(dev_id, INST_READ, struct.pack("<HH", addr, 4)))
    resp = read_response_raw(ser)
    if resp and len(resp) >= 15 and resp[8] == 0:
        return struct.unpack("<f", resp[9:13])[0]
    return None


def dxl_read_i32(ser, addr: int, dev_id: int = DEV_ID) -> Optional[int]:
    """Read a 32-bit signed integer register, returning None on failure."""
    ser.reset_input_buffer()
    ser.write(build_packet(dev_id, INST_READ, struct.pack("<HH", addr, 4)))
    resp = read_response_raw(ser)
    if resp and len(resp) >= 15 and resp[8] == 0:
        return struct.unpack("<i", resp[9:13])[0]
    return None


def dxl_read_u8(ser, addr: int, dev_id: int = DEV_ID) -> Optional[int]:
    """Read a single uint8 register, returning None on failure."""
    ser.reset_input_buffer()
    ser.write(build_packet(dev_id, INST_READ, struct.pack("<HH", addr, 1)))
    resp = read_response_raw(ser)
    if resp and len(resp) >= 12 and resp[8] == 0:
        return resp[9]
    return None


def write_f32(ser, addr: int, val: float, dev_id: int = DEV_ID) -> None:
    dxl_write(ser, addr, struct.pack("<f", val), dev_id)


def write_i32(ser, addr: int, val: int, dev_id: int = DEV_ID) -> None:
    dxl_write(ser, addr, struct.pack("<i", val), dev_id)


def write_u8(ser, addr: int, val: int, dev_id: int = DEV_ID) -> None:
    dxl_write(ser, addr, bytes([val & 0xFF]), dev_id)


def set_velocity(ser, lin_x: float, ang_z: float = 0.0,
                 dev_id: int = DEV_ID) -> None:
    """Write linear and angular velocity command registers."""
    dxl_write(ser, REG_CMD_LINEAR_X,
              struct.pack("<i", int(round(lin_x * 100))), dev_id)
    dxl_write(ser, REG_CMD_ANGULAR_Z,
              struct.pack("<i", int(round(ang_z * 100))), dev_id)


def keep_alive(ser, dev_id: int = DEV_ID) -> None:
    """Ping the firmware by reading the millis register."""
    dxl_read_i32(ser, REG_MILLIS, dev_id)


# ── Calibration queries ───────────────────────────────────────────────────
def query_calibration_blob(ser, dev_id: int = DEV_ID) -> Optional[bytes]:
    """Send CALIB_CMD_GET and return the raw response packet, or None."""
    ser.reset_input_buffer()
    ser.write(build_packet(dev_id, INST_CALIBRATION, bytes([CALIB_CMD_GET])))
    return read_response_raw(ser, timeout=0.25)


def query_wheel_separation(ser, dev_id: int = DEV_ID) -> Optional[float]:
    """Read wheel separation from firmware calibration blob."""
    resp = query_calibration_blob(ser, dev_id)
    if resp and len(resp) >= 18 and resp[8] == 0:
        return struct.unpack_from("<f", resp, 14)[0]
    return None


def query_wheel_radius(ser, dev_id: int = DEV_ID) -> Optional[float]:
    """Read wheel radius from firmware calibration blob."""
    resp = query_calibration_blob(ser, dev_id)
    if resp and len(resp) >= 14 and resp[8] == 0:
        return struct.unpack_from("<f", resp, 10)[0]
    return None


# ── Terminal colours ──────────────────────────────────────────────────────
RED = "\033[0;31m"
GRN = "\033[0;32m"
YLW = "\033[1;33m"
CYN = "\033[0;36m"
BLD = "\033[1m"
NC  = "\033[0m"
