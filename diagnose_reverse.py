#!/usr/bin/env python3
"""
diagnose_reverse.py — Forward vs. reverse straight-line symmetry diagnostic.

Runs paired forward/reverse passes and records at ~10 Hz:
  • Left / right wheel velocities and positions (from /joint_states)
  • IMU yaw and gyro-Z (from /imu)
  • Odometry XY and heading (from /odom)
  • Firmware registers polled directly over Dynamixel serial:
      – Measured wheel speeds (m/s)
      – Heading error and heading-hold correction (rad / rad/s)
      – Accumulated L−R encoder diff (m)
      – Raw encoder tick counts

Outputs:
  • Live per-pass summary to stdout
  • diagnose_reverse_YYYYMMDD_HHMMSS.csv  (one row per ~100 ms sample)
  • Final comparison table: forward vs reverse

Usage:
  python3 diagnose_reverse.py                   # 1 pair, 0.10 m/s, 3 s
  python3 diagnose_reverse.py --speed 0.08 --duration 4 --pairs 2
  python3 diagnose_reverse.py --no-serial        # skip Dynamixel register reads

Requires: turtlebot3-bringup.service running
"""

import argparse
import csv
import math
import os
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState

# ── colours ───────────────────────────────────────────────────────────────────
RED = "\033[0;31m"; GRN = "\033[0;32m"; YLW = "\033[1;33m"
CYN = "\033[0;36m"; BLD = "\033[1m";    NC  = "\033[0m"

# ── ROS QoS ──────────────────────────────────────────────────────────────────
BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── Dynamixel constants ───────────────────────────────────────────────────────
DXL_PORTS   = ("/dev/ttyTB3", "/dev/ttyACM0", "/dev/ttyACM1")
DXL_ID      = 200
DXL_BAUD    = 1_000_000

# Register map (matches firmware/main.c)
REG_PRESENT_VEL_L  = 128   # int32  — Dynamixel RPM units
REG_PRESENT_VEL_R  = 132   # int32
REG_PRESENT_POS_L  = 136   # int32  — Dynamixel ticks
REG_PRESENT_POS_R  = 140   # int32
REG_ENC_L_COUNT    = 184   # int32  — raw quadrature counts
REG_ENC_R_COUNT    = 188   # int32
REG_DBG_VEL_L      = 212   # float  — measured velocity m/s
REG_DBG_VEL_R      = 216   # float
# NOTE: 240–255 are DIAG counters in the firmware — heading-hold regs start at 280.
REG_HEADING_HOLD_KP= 280   # float
REG_DBG_HEADING_ERR= 284   # float  — heading error (rad)
REG_HEADING_HOLD_EN= 288   # uint8
REG_HEADING_HOLD_KI= 292   # float
REG_HEADING_CORR   = 296   # float  — ang_z correction (rad/s)
REG_ENC_TRIM_KP    = 300   # float
REG_DBG_ENC_TRIM   = 304   # float
REG_DBG_ENC_DIFF   = 308   # float  — accumulated L−R distance (m)

# IMU gyro-Z (firmware addr 68 — included in bulk read for wobble detection)
REG_IMU_ANG_VEL_Z  = 68   # float  — gyro Z rad/s

# Bulk read: start at IMU gyro-Z (68) to capture gyro data + all firmware regs in one shot.
# 68..311 = 244 bytes.  Firmware REG_SIZE=312 so max addr 311 is in range.
BULK_ADDR   = REG_IMU_ANG_VEL_Z   # 68
BULK_LEN    = (REG_DBG_ENC_DIFF - BULK_ADDR) + 4  # 244 bytes  (68..311)

WHEEL_RADIUS   = 0.033   # m
DXL_VEL_UNIT   = 0.229   # RPM per unit
RPM_TO_RADS    = 2.0 * math.pi / 60.0

# ── Dynamixel write helper ────────────────────────────────────────────────────
REG_CMD_LINEAR_X  = 150   # int32, units of 0.01 m/s
REG_CMD_ANGULAR_Z = 170   # int32, units of 0.01 rad/s
REG_MOTOR_TORQUE  = 149   # uint8

def _make_write_i32_pkt(addr: int, value: int) -> bytes:
    params  = struct.pack("<H", addr) + struct.pack("<i", value)
    plen    = len(params) + 3
    hdr     = bytes([0xFF, 0xFF, 0xFD, 0x00, DXL_ID,
                     plen & 0xFF, (plen >> 8) & 0xFF, 0x03]) + params
    crc     = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])
_CRC_TABLE = []
for _i in range(256):
    _c = _i << 8
    for _ in range(8):
        _c = ((_c << 1) ^ 0x8005) & 0xFFFF if (_c & 0x8000) else ((_c << 1) & 0xFFFF)
    _CRC_TABLE.append(_c)

def _crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc

def _make_read_pkt(addr: int, length: int) -> bytes:
    params  = struct.pack("<HH", addr, length)
    plen    = len(params) + 3
    hdr     = bytes([0xFF, 0xFF, 0xFD, 0x00, DXL_ID,
                     plen & 0xFF, (plen >> 8) & 0xFF, 0x02]) + params
    crc     = _crc16(hdr)
    return hdr + bytes([crc & 0xFF, crc >> 8])

def _read_response(ser, expected: int, timeout: float = 0.25) -> Optional[bytes]:
    """Read a Dynamixel v2 status packet, verifying device ID, instruction, and length.
    Skips over packets that don't match our expected device / size so concurrent
    turtlebot3_node traffic doesn't corrupt the result.

    Key invariant: when the 4-byte header is not yet fully in buf, we keep up to
    3 trailing bytes as a potential partial match (e.g. buf ends with \\xff\\xff\\xfd
    before the \\x00 arrives).  Discarding those bytes would split the header across
    two read() calls and cause the search to miss it permanently.
    """
    HEADER = b"\xFF\xFF\xFD\x00"
    t0  = time.monotonic()
    buf = b""
    while time.monotonic() - t0 < timeout:
        waiting = ser.in_waiting
        chunk = ser.read(waiting if waiting > 0 else 1)
        if chunk:
            buf += chunk
        # Search for valid header, validating each candidate
        while True:
            idx = buf.find(HEADER)
            if idx < 0:
                # No complete header found — keep the last 3 bytes in case they
                # are the start of \xff\xff\xfd that is split across two reads.
                buf = buf[-3:] if len(buf) >= 3 else buf
                break
            buf = buf[idx:]
            if len(buf) < 9:
                break  # need more data for ID + len + instr fields
            # Verify device ID (byte 4) and instruction 0x55 (status, byte 7)
            if buf[4] != DXL_ID or buf[7] != 0x55:
                buf = buf[4:]  # skip this header candidate and search again
                continue
            plen = buf[5] | (buf[6] << 8)   # length field: INSTR+ERROR+PARAMS+CRC2
            num_params = plen - 4            # subtract INSTR(1)+ERROR(1)+CRC(2)
            total      = 7 + plen            # full packet size including 4-byte header
            if len(buf) < total:
                break  # packet not fully received yet, keep reading
            if num_params != expected:
                buf = buf[4:]  # wrong payload size — skip and search again
                continue
            return buf[9:9 + expected]       # correct packet: return payload bytes
    return None

# ── Firmware register snapshot ────────────────────────────────────────────────
@dataclass
class FwRegs:
    ts: float = 0.0
    vel_l_ms:     float = 0.0
    vel_r_ms:     float = 0.0
    pos_l_ticks:  int   = 0
    pos_r_ticks:  int   = 0
    enc_l_count:  int   = 0
    enc_r_count:  int   = 0
    dbg_vel_l:    float = 0.0
    dbg_vel_r:    float = 0.0
    heading_err:  float = 0.0
    heading_corr: float = 0.0
    enc_diff:     float = 0.0
    heading_hold_en: int = 0
    gyro_z:       float = 0.0   # IMU angular velocity Z (rad/s)

def _parse_bulk(data: bytes) -> Optional[FwRegs]:
    if len(data) < BULK_LEN:
        return None
    def i32(off): return struct.unpack_from("<i", data, off - BULK_ADDR)[0]
    def f32(off): return struct.unpack_from("<f", data, off - BULK_ADDR)[0]
    def u8(off):  return data[off - BULK_ADDR]

    r          = FwRegs()
    r.ts       = time.monotonic()
    vel_l_rpm  = i32(REG_PRESENT_VEL_L) * DXL_VEL_UNIT
    vel_r_rpm  = i32(REG_PRESENT_VEL_R) * DXL_VEL_UNIT
    r.vel_l_ms = vel_l_rpm * RPM_TO_RADS * WHEEL_RADIUS
    r.vel_r_ms = vel_r_rpm * RPM_TO_RADS * WHEEL_RADIUS
    r.pos_l_ticks  = i32(REG_PRESENT_POS_L)
    r.pos_r_ticks  = i32(REG_PRESENT_POS_R)
    r.enc_l_count  = i32(REG_ENC_L_COUNT)
    r.enc_r_count  = i32(REG_ENC_R_COUNT)
    r.dbg_vel_l    = f32(REG_DBG_VEL_L)
    r.dbg_vel_r    = f32(REG_DBG_VEL_R)
    r.heading_err  = f32(REG_DBG_HEADING_ERR)
    r.heading_corr = f32(REG_HEADING_CORR)
    r.enc_diff     = f32(REG_DBG_ENC_DIFF)
    r.heading_hold_en = u8(REG_HEADING_HOLD_EN)
    r.gyro_z       = f32(REG_IMU_ANG_VEL_Z)
    return r

# ── Wobble / oscillation analysis ────────────────────────────────────────────
def _wobble_analysis(gyro_zs: list, tss: list) -> dict:
    """Detect heading-hold hunting (oscillation) from IMU gyro-Z time series.

    Returns a dict with:
      std      — standard deviation of gyro-Z (rad/s)
      peak     — absolute peak value (rad/s)
      freq_hz  — estimated oscillation frequency from zero-crossing count
      wobble   — True if clear oscillation detected
      msg      — human-readable verdict string
    """
    DBAND   = 0.015   # rad/s deadband to ignore noise (~0.9°/s)
    result  = dict(std=0.0, peak=0.0, freq_hz=0.0, wobble=False, msg="")
    n = len(gyro_zs)
    if n < 4:
        result["msg"] = "too few samples"
        return result

    mean  = sum(gyro_zs) / n
    std   = math.sqrt(sum((g - mean) ** 2 for g in gyro_zs) / max(n - 1, 1))
    peak  = max(abs(g) for g in gyro_zs)
    result["std"]  = std
    result["peak"] = peak

    # Count zero crossings (ignoring values inside deadband) to estimate frequency
    crossings = 0
    prev_sign = None
    for g in gyro_zs:
        if abs(g) < DBAND:
            continue
        s = 1 if g > 0 else -1
        if prev_sign is not None and s != prev_sign:
            crossings += 1
        prev_sign = s

    duration = (tss[-1] - tss[0]) if len(tss) >= 2 else 1.0
    # Each full cycle = 2 zero crossings
    freq_hz = (crossings / 2.0) / max(duration, 1e-3)
    result["freq_hz"] = freq_hz

    # Wobble: non-trivial amplitude AND crosses zero at >= 0.3 Hz with >= 4 crossings
    wobble = std > 0.04 and crossings >= 4 and freq_hz >= 0.3
    result["wobble"] = wobble

    std_deg  = math.degrees(std)
    peak_deg = math.degrees(peak)
    if wobble:
        result["msg"] = (
            f"⚠ WOBBLE  ±{peak_deg:.1f}°/s peak  std={std_deg:.1f}°/s  "
            f"freq={freq_hz:.2f} Hz  → reduce KP"
        )
    else:
        result["msg"] = (
            f"stable   std={std_deg:.1f}°/s  peak=±{peak_deg:.1f}°/s"
        )
    return result

# ── Per-sample record ─────────────────────────────────────────────────────────
@dataclass
class Sample:
    ts:          float = 0.0
    phase:       str   = ""   # "fwd" or "rev"
    pair:        int   = 0
    # ROS
    odom_x:      float = 0.0
    odom_y:      float = 0.0
    odom_yaw:    float = 0.0
    imu_yaw:     float = 0.0
    gyro_z:      float = 0.0
    js_vel_l:    float = 0.0
    js_vel_r:    float = 0.0
    js_pos_l:    float = 0.0
    js_pos_r:    float = 0.0
    # Firmware
    fw_vel_l:    float = 0.0
    fw_vel_r:    float = 0.0
    fw_dbg_vl:   float = 0.0
    fw_dbg_vr:   float = 0.0
    fw_hdg_err:  float = 0.0
    fw_hdg_corr: float = 0.0
    fw_enc_diff: float = 0.0
    fw_enc_l:    int   = 0
    fw_enc_r:    int   = 0
    fw_hold_en:  int   = 0
    fw_gyro_z:   float = 0.0   # IMU gyro-Z rad/s (from firmware registers)

CSV_HEADER = [
    "ts","phase","pair",
    "odom_x","odom_y","odom_yaw_deg","imu_yaw_deg","gyro_z_rads",
    "js_vel_l_ms","js_vel_r_ms","js_pos_l_rad","js_pos_r_rad",
    "fw_vel_l_ms","fw_vel_r_ms","fw_dbg_vl_ms","fw_dbg_vr_ms",
    "fw_hdg_err_rad","fw_hdg_err_deg","fw_hdg_corr_rads",
    "fw_enc_diff_m","fw_enc_l","fw_enc_r","fw_hold_en",
    "fw_gyro_z_rads","fw_gyro_z_degs",
]

def sample_to_row(s: Sample) -> list:
    return [
        f"{s.ts:.4f}", s.phase, s.pair,
        f"{s.odom_x:.5f}", f"{s.odom_y:.5f}",
        f"{math.degrees(s.odom_yaw):.3f}", f"{math.degrees(s.imu_yaw):.3f}",
        f"{s.gyro_z:.5f}",
        f"{s.js_vel_l:.5f}", f"{s.js_vel_r:.5f}",
        f"{s.js_pos_l:.5f}", f"{s.js_pos_r:.5f}",
        f"{s.fw_vel_l:.5f}", f"{s.fw_vel_r:.5f}",
        f"{s.fw_dbg_vl:.5f}", f"{s.fw_dbg_vr:.5f}",
        f"{s.fw_hdg_err:.5f}", f"{math.degrees(s.fw_hdg_err):.3f}",
        f"{s.fw_hdg_corr:.5f}",
        f"{s.fw_enc_diff:.5f}", s.fw_enc_l, s.fw_enc_r, s.fw_hold_en,
        f"{s.fw_gyro_z:.5f}", f"{math.degrees(s.fw_gyro_z):.3f}",
    ]

# ── ROS2 node ─────────────────────────────────────────────────────────────────
def _quat_to_yaw(o) -> float:
    siny = 2.0 * (o.w * o.z + o.x * o.y)
    cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    return math.atan2(siny, cosy)

class DiagReverseNode(Node):
    def __init__(self):
        super().__init__("diagnose_reverse")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._odom: Optional[Odometry] = None
        self._imu:  Optional[Imu]      = None
        self._joint: Optional[JointState] = None
        self.create_subscription(Odometry,    "/odom",         self._odom_cb,  BEST_EFFORT_QOS)
        self.create_subscription(Imu,         "/imu",          self._imu_cb,   BEST_EFFORT_QOS)
        self.create_subscription(JointState,  "/joint_states", self._joint_cb, BEST_EFFORT_QOS)

    def _odom_cb(self, m: Odometry):   self._odom  = m
    def _imu_cb(self,  m: Imu):        self._imu   = m
    def _joint_cb(self,m: JointState): self._joint = m

    def spin_for(self, secs: float):
        t = time.monotonic() + secs
        while time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, lin: float, ang: float = 0.0):
        msg = Twist()
        msg.linear.x = lin; msg.angular.z = ang
        self._pub.publish(msg)

    def stop(self, settle: float = 0.5):
        self.send(0.0)
        self.spin_for(settle)

    def snapshot(self) -> dict:
        rclpy.spin_once(self, timeout_sec=0.0)
        out: dict = {}
        o = self._odom
        if o:
            out["odom_x"]   = o.pose.pose.position.x
            out["odom_y"]   = o.pose.pose.position.y
            out["odom_yaw"] = _quat_to_yaw(o.pose.pose.orientation)
        i = self._imu
        if i:
            out["imu_yaw"] = _quat_to_yaw(i.orientation)
            out["gyro_z"]  = i.angular_velocity.z
        j = self._joint
        if j and len(j.velocity) >= 2 and len(j.position) >= 2:
            out["js_vel_l"] = j.velocity[0] * WHEEL_RADIUS
            out["js_vel_r"] = j.velocity[1] * WHEEL_RADIUS
            out["js_pos_l"] = j.position[0]
            out["js_pos_r"] = j.position[1]
        return out

    def wait_topics(self, timeout: float = 20.0) -> bool:
        t = time.monotonic() + timeout
        print("  Waiting for /odom, /imu, /joint_states … ", end="", flush=True)
        while time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._odom and self._imu and self._joint:
                print(f"{GRN}OK{NC}")
                return True
        miss = [s for s, v in [("/odom",self._odom),("/imu",self._imu),("/joint_states",self._joint)] if not v]
        print(f"{RED}TIMEOUT{NC}  (missing: {', '.join(miss)})")
        return False


# ── Serial poller thread ──────────────────────────────────────────────────────
class SerialPoller:
    def __init__(self, port: str):
        import serial as _serial
        self._ser = _serial.Serial(port, DXL_BAUD, timeout=0.05)
        time.sleep(0.1)
        self._pkt     = _make_read_pkt(BULK_ADDR, BULK_LEN)
        self._data_lock = threading.Lock()   # protects _latest
        self._bus_lock  = threading.Lock()   # serialises all serial I/O (half-duplex bus)
        self._latest: Optional[FwRegs] = None
        self._running = True
        self.poll_ok   = 0   # number of successful register snapshots
        self.poll_fail = 0   # number of failed/empty reads
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            try:
                with self._bus_lock:
                    self._ser.reset_input_buffer()
                    self._ser.write(self._pkt)
                    raw = _read_response(self._ser, BULK_LEN)
                if raw:
                    regs = _parse_bulk(raw)
                    if regs:
                        with self._data_lock:
                            self._latest = regs
                        self.poll_ok += 1
                    else:
                        self.poll_fail += 1
                else:
                    self.poll_fail += 1
            except Exception as _e:
                self.poll_fail += 1
            time.sleep(0.08)   # ~12 Hz

    def get(self) -> Optional[FwRegs]:
        with self._data_lock:
            return self._latest

    def write_velocity(self, lin_x_ms: float, ang_z_rads: float = 0.0):
        """Write velocity command registers directly (bypasses ROS / turtlebot3_node).
        Acquires the bus lock so writes never collide with the poll-read cycle."""
        lin_val = int(round(lin_x_ms   * 100.0))   # m/s → 0.01 m/s units
        ang_val = int(round(ang_z_rads * 100.0))   # rad/s → 0.01 rad/s units
        try:
            with self._bus_lock:
                self._ser.write(_make_write_i32_pkt(REG_CMD_LINEAR_X,  lin_val))
                # Read (and discard) the firmware's write-acknowledge STATUS packet
                # so it doesn't accumulate in the RX buffer for the poller thread.
                self._ser.read(11)   # typical write-ACK: 4 hdr + ID + len_l + len_h + instr + err + crc2
                self._ser.write(_make_write_i32_pkt(REG_CMD_ANGULAR_Z, ang_val))
                self._ser.read(11)
        except Exception:
            pass

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        try:
            self._ser.close()
        except Exception:
            pass


# ── Drive pass ────────────────────────────────────────────────────────────────
def run_drive_pass(node: DiagReverseNode,
                   poller: Optional[SerialPoller],
                   speed     : float,
                   duration  : float,
                   phase     : str,
                   pair_num  : int,
                   t0_global : float) -> tuple[list[Sample], dict]:
    """Drive at `speed` for `duration` seconds, sampling at ~10 Hz.

    Returns (samples, summary_dict).
    """
    direction_label = "FORWARD" if speed >= 0 else "REVERSE"
    print(f"  [{direction_label}]  speed={speed:+.3f} m/s  duration={duration:.1f} s", flush=True)

    samples: list[Sample] = []
    node.spin_for(0.3)

    # Baseline
    base = node.snapshot()
    base_odom_x   = base.get("odom_x", 0.0)
    base_odom_y   = base.get("odom_y", 0.0)
    base_odom_yaw = base.get("odom_yaw", 0.0)
    base_imu_yaw  = base.get("imu_yaw", 0.0)

    # Drive loop
    t_start = time.monotonic()
    t_next_sample = t_start

    while True:
        elapsed = time.monotonic() - t_start
        if elapsed >= duration:
            break
        node.send(speed)
        rclpy.spin_once(node, timeout_sec=0.02)

        now = time.monotonic()
        if now >= t_next_sample:
            t_next_sample = now + 0.10
            ros_snap = node.snapshot()
            fw  = poller.get() if poller else None

            s = Sample()
            s.ts          = now - t0_global
            s.phase       = phase
            s.pair        = pair_num
            s.odom_x      = ros_snap.get("odom_x", 0.0)
            s.odom_y      = ros_snap.get("odom_y", 0.0)
            s.odom_yaw    = ros_snap.get("odom_yaw", 0.0)
            s.imu_yaw     = ros_snap.get("imu_yaw", 0.0)
            s.gyro_z      = ros_snap.get("gyro_z", 0.0)
            s.js_vel_l    = ros_snap.get("js_vel_l", 0.0)
            s.js_vel_r    = ros_snap.get("js_vel_r", 0.0)
            s.js_pos_l    = ros_snap.get("js_pos_l", 0.0)
            s.js_pos_r    = ros_snap.get("js_pos_r", 0.0)
            if fw:
                s.fw_vel_l    = fw.vel_l_ms
                s.fw_vel_r    = fw.vel_r_ms
                s.fw_dbg_vl   = fw.dbg_vel_l
                s.fw_dbg_vr   = fw.dbg_vel_r
                s.fw_hdg_err  = fw.heading_err
                s.fw_hdg_corr = fw.heading_corr
                s.fw_enc_diff = fw.enc_diff
                s.fw_enc_l    = fw.enc_l_count
                s.fw_enc_r    = fw.enc_r_count
                s.fw_hold_en  = fw.heading_hold_en
            samples.append(s)

    node.stop(settle=0.6)

    # Final snapshot
    end = node.snapshot()
    d_x   = end.get("odom_x", base_odom_x)   - base_odom_x
    d_y   = end.get("odom_y", base_odom_y)   - base_odom_y
    dist  = math.sqrt(d_x**2 + d_y**2)
    d_hdg = math.degrees(end.get("odom_yaw", base_odom_yaw) - base_odom_yaw)
    d_imu = math.degrees(end.get("imu_yaw",  base_imu_yaw)  - base_imu_yaw)
    # normalise to ±180
    def norm(a): return (a + 180) % 360 - 180
    d_hdg = norm(d_hdg)
    d_imu = norm(d_imu)

    # Trim steady-state samples (middle 60%)
    trim  = len(samples) // 5
    mid   = samples[trim:-trim] if len(samples) > 5 else samples
    def safe_mean(vals): return sum(vals)/len(vals) if vals else 0.0

    vl_mid  = [s.js_vel_l  for s in mid]
    vr_mid  = [s.js_vel_r  for s in mid]
    gz_mid  = [s.gyro_z    for s in mid]
    hd_mid  = [s.fw_hdg_err  for s in mid]
    hc_mid  = [s.fw_hdg_corr for s in mid]
    ed_mid  = [s.fw_enc_diff  for s in mid]
    dvl_mid = [s.fw_dbg_vl   for s in mid]
    dvr_mid = [s.fw_dbg_vr   for s in mid]

    mean_vl   = safe_mean(vl_mid)
    mean_vr   = safe_mean(vr_mid)
    mean_gz   = safe_mean(gz_mid)
    mean_hd   = safe_mean(hd_mid)
    mean_hc   = safe_mean(hc_mid)
    mean_ed   = safe_mean(ed_mid)
    mean_dvl  = safe_mean(dvl_mid)
    mean_dvr  = safe_mean(dvr_mid)

    ratio     = (mean_vl / mean_vr) if abs(mean_vr) > 1e-4 else float("nan")
    fw_ratio  = (mean_dvl / mean_dvr) if abs(mean_dvr) > 1e-4 else float("nan")

    summary = dict(
        phase=phase, pair=pair_num, speed=speed,
        dist_m=dist, dx_m=d_x, dy_m=d_y,
        hdg_drift_deg=d_hdg, imu_drift_deg=d_imu,
        mean_vl_ms=mean_vl, mean_vr_ms=mean_vr, js_ratio=ratio,
        mean_gyro_rads=mean_gz,
        fw_mean_vl_ms=mean_dvl, fw_mean_vr_ms=mean_dvr, fw_ratio=fw_ratio,
        fw_mean_hdg_err_deg=math.degrees(mean_hd),
        fw_mean_hdg_corr_rads=mean_hc,
        fw_mean_enc_diff_m=mean_ed,
    )

    # Live printout
    ratio_pct  = (ratio - 1.0) * 100 if math.isfinite(ratio) else float("nan")
    fw_rat_pct = (fw_ratio - 1.0) * 100 if math.isfinite(fw_ratio) else float("nan")
    col_r  = GRN if abs(ratio_pct)  < 2 else (YLW if abs(ratio_pct)  < 6 else RED)
    col_fw = GRN if abs(fw_rat_pct) < 2 else (YLW if abs(fw_rat_pct) < 6 else RED)

    print(f"    Odom dist:        {dist*100:6.1f} cm   (dx={d_x*100:+.1f} dy={d_y*100:+.1f} cm)")
    print(f"    Heading drift:    {d_hdg:+6.2f}°  (odom)    {d_imu:+6.2f}°  (IMU)")
    print(f"    Gyro-Z mean:      {math.degrees(mean_gz):+6.2f}°/s")
    print(f"    JointState vel:   L={mean_vl*1000:+6.1f} mm/s   R={mean_vr*1000:+6.1f} mm/s  "
          f"ratio L/R={col_r}{ratio:.4f}  ({ratio_pct:+.2f}%){NC}")
    if math.isfinite(fw_ratio):
        print(f"    FW encoder vel:   L={mean_dvl*1000:+6.1f} mm/s   R={mean_dvr*1000:+6.1f} mm/s  "
              f"ratio L/R={col_fw}{fw_ratio:.4f}  ({fw_rat_pct:+.2f}%){NC}")
        print(f"    FW heading err:   {math.degrees(mean_hd):+6.3f}°  (mean during pass)")
        print(f"    FW hold corr:     {math.degrees(mean_hc):+6.3f}°/s  (ang_z correction)")
        print(f"    FW enc L−R diff:  {mean_ed*1000:+7.2f} mm  (accumulated)")

    return samples, summary


# ── Pure-serial drive pass (used when bringup is stopped) ────────────────────
def run_serial_drive_pass(poller: "SerialPoller",
                          speed: float, duration: float,
                          phase: str, pair_num: int,
                          t0_global: float) -> tuple[list[Sample], dict]:
    """Drive the robot via Dynamixel register writes and sample firmware regs.
    Does not require ROS2 / turtlebot3_node.

    All I/O is done inline on the calling thread (no background thread races):
    each iteration writes the velocity command, then immediately reads registers
    while the bus is idle.  This guarantees the read never conflicts with a
    concurrent write and that the poller's _read_response timeout can never
    fire mid-response.
    """
    READ_PKT = _make_read_pkt(BULK_ADDR, BULK_LEN)

    def _send_vel(ser, v: float, a: float = 0.0) -> None:
        """Write lin_x and ang_z registers; discard any write-ACK bytes."""
        lin_val = int(round(v * 100.0))
        ang_val = int(round(a * 100.0))
        ser.write(_make_write_i32_pkt(REG_CMD_LINEAR_X, lin_val))
        time.sleep(0.003)                  # let firmware ACK (11 bytes ≈ 0.11 ms at 1 Mbaud)
        ser.read(max(ser.in_waiting, 1))   # drain write-ACK without blocking on fixed count
        ser.write(_make_write_i32_pkt(REG_CMD_ANGULAR_Z, ang_val))
        time.sleep(0.003)
        ser.read(max(ser.in_waiting, 1))

    def _read_regs(ser) -> Optional[FwRegs]:
        """Send one bulk READ and parse the response.  ~2 ms at 1 Mbaud."""
        ser.reset_input_buffer()
        ser.write(READ_PKT)
        raw = _read_response(ser, BULK_LEN)
        return _parse_bulk(raw) if raw else None

    samples: list[Sample] = []
    enc_l_start: Optional[int] = None
    enc_r_start: Optional[int] = None
    poll_ok = 0
    poll_fail = 0

    print(f"    Driving {'forward' if speed > 0 else 'REVERSE'} at {speed:+.3f} m/s "
          f"for {duration:.1f} s …", flush=True)

    # Stop background poller while we drive to avoid bus contention
    poller._running = False
    poller._thread.join(timeout=0.5)

    ser = poller._ser
    t_start = time.monotonic()
    t_end   = t_start + duration

    try:
        # Issue first velocity command
        _send_vel(ser, speed)

        while time.monotonic() < t_end:
            # Refresh velocity (every ~100 ms keeps watchdog alive)
            _send_vel(ser, speed)

            # Read all debug registers inline
            fw = _read_regs(ser)
            if fw:
                poll_ok += 1
                if enc_l_start is None:
                    enc_l_start = fw.enc_l_count
                    enc_r_start = fw.enc_r_count
                s = Sample(
                    ts          = time.monotonic() - t0_global,
                    phase       = phase,
                    pair        = pair_num,
                    fw_dbg_vl   = fw.dbg_vel_l,
                    fw_dbg_vr   = fw.dbg_vel_r,
                    fw_hdg_err  = fw.heading_err,
                    fw_hdg_corr = fw.heading_corr,
                    fw_enc_diff = fw.enc_diff,
                    fw_enc_l    = fw.enc_l_count,
                    fw_enc_r    = fw.enc_r_count,
                    fw_hold_en  = fw.heading_hold_en,
                    fw_gyro_z   = fw.gyro_z,
                )
                samples.append(s)
            else:
                poll_fail += 1

            time.sleep(0.08)   # ~10 Hz sample rate

    finally:
        # Stop robot and restart background poller for later use (e.g. settle reads)
        _send_vel(ser, 0.0)
        poller._running = True
        poller._thread = threading.Thread(target=poller._loop, daemon=True)
        poller._thread.start()

    time.sleep(0.3)   # settle
    # Get a fresh snapshot directly while background thread is paused on the lock
    with poller._bus_lock:
        fw_final = _read_regs(ser) or poller.get()

    print(f"    [poll: {poll_ok} OK / {poll_fail} fail  "
          f"| samples={len(samples)}]", flush=True)
    # Compute summary from samples
    def safe_mean(vals):
        return sum(vals) / len(vals) if vals else 0.0

    enc_l_end = fw_final.enc_l_count if fw_final else (enc_l_start or 0)
    enc_r_end = fw_final.enc_r_count if fw_final else (enc_r_start or 0)
    enc_counts_per_m = 3840.0 / (2.0 * math.pi * WHEEL_RADIUS)  # counts per metre

    dl_m = ((enc_l_end - (enc_l_start or enc_l_end)) / enc_counts_per_m)
    dr_m = ((enc_r_end - (enc_r_start or enc_r_end)) / enc_counts_per_m)

    dvls  = [s.fw_dbg_vl   for s in samples]
    dvrs  = [s.fw_dbg_vr   for s in samples]
    hds   = [s.fw_hdg_err  for s in samples]
    hcs   = [s.fw_hdg_corr for s in samples]
    eds   = [s.fw_enc_diff for s in samples]
    gzs   = [s.fw_gyro_z   for s in samples]
    gtss  = [s.ts           for s in samples]

    mean_dvl = safe_mean(dvls)
    mean_dvr = safe_mean(dvrs)
    mean_hd  = safe_mean(hds)
    mean_hc  = safe_mean(hcs)
    mean_ed  = safe_mean(eds)

    wobble   = _wobble_analysis(gzs, gtss)

    fw_ratio = (mean_dvl / mean_dvr) if abs(mean_dvr) > 1e-4 else float("nan")
    fw_rat_pct = (fw_ratio - 1.0) * 100 if math.isfinite(fw_ratio) else float("nan")
    col_fw = GRN if abs(fw_rat_pct) < 2 else (YLW if abs(fw_rat_pct) < 6 else RED)

    hold_en_val = samples[-1].fw_hold_en if samples else 0

    print(f"    FW encoder vel:   L={mean_dvl*1000:+6.1f} mm/s   R={mean_dvr*1000:+6.1f} mm/s  "
          f"ratio L/R={col_fw}{fw_ratio:.4f}  ({fw_rat_pct:+.2f}%){NC}")
    print(f"    FW heading err:   {math.degrees(mean_hd):+6.3f}°  (mean during pass)  "
          f"hold_en={hold_en_val}")
    print(f"    FW hold corr:     {math.degrees(mean_hc):+6.3f}°/s  (ang_z correction)")
    print(f"    FW enc L−R diff:  {mean_ed*1000:+7.2f} mm  (accumulated)")
    print(f"    Enc travel:       L={dl_m*100:+.1f} cm   R={dr_m*100:+.1f} cm")
    col_wb = RED if wobble["wobble"] else GRN
    print(f"    Gyro-Z wobble:    {col_wb}{wobble['msg']}{NC}")

    summary = dict(
        phase=phase, pair=pair_num, speed=speed,
        dist_m=abs(dl_m + dr_m) / 2.0,
        fw_mean_vl_ms=mean_dvl, fw_mean_vr_ms=mean_dvr, fw_ratio=fw_ratio,
        fw_mean_hdg_err_deg=math.degrees(mean_hd),
        fw_mean_hdg_corr_rads=mean_hc,
        fw_mean_enc_diff_m=mean_ed,
        gyro_z_std_rads=wobble["std"],
        gyro_z_peak_rads=wobble["peak"],
        gyro_z_freq_hz=wobble["freq_hz"],
        gyro_z_wobble=1.0 if wobble["wobble"] else 0.0,
        # placeholders for ROS fields not available in serial-only mode
        dx_m=0.0, dy_m=0.0, hdg_drift_deg=0.0, imu_drift_deg=0.0,
        mean_vl_ms=mean_dvl, mean_vr_ms=mean_dvr, js_ratio=fw_ratio,
        mean_gyro_rads=0.0,
    )
    return samples, summary


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description="Forward vs reverse symmetry diagnostic.\n\n"
                    "Two modes (auto-selected):\n"
                    "  Default           — bringup running: drive via /cmd_vel, read ROS topics.\n"
                    "                      Serial register reads skipped (avoids crash).\n"
                    "  --stop-bringup    — bringup stopped: drive via Dynamixel registers directly,\n"
                    "                      full firmware register diagnostics. No ROS2 needed.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--speed",        type=float, default=0.10,
                    help="drive speed m/s (default 0.10)")
    ap.add_argument("--duration",     type=float, default=3.0,
                    help="seconds per pass (default 3.0)")
    ap.add_argument("--pairs",        type=int,   default=1,
                    help="number of fwd+rev pairs (default 1)")
    ap.add_argument("--no-csv",       action="store_true",
                    help="do not write CSV file")
    ap.add_argument("--port",         default=None,
                    help="serial port override")
    ap.add_argument("--stop-bringup", action="store_true",
                    help="stop turtlebot3-bringup before test (pure-serial mode; no ROS2 needed)")
    args = ap.parse_args()

    BRINGUP_SVC = "turtlebot3-bringup.service"
    _bringup_was_active = False

    def _svc_active(name: str) -> bool:
        try:
            r = subprocess.run(["systemctl", "is-active", "--quiet", name],
                               capture_output=True, timeout=3)
            return r.returncode == 0
        except Exception:
            return False

    print(f"\n{BLD}══════════════════════════════════════════════════════{NC}")
    print(f"{BLD} diagnose_reverse.py — Forward vs Reverse Symmetry {NC}")
    print(f"{BLD} speed={args.speed:.3f} m/s  duration={args.duration:.1f} s  pairs={args.pairs}{NC}")
    print(f"{BLD}══════════════════════════════════════════════════════{NC}\n")

    # ── Decide operating mode ──────────────────────────────────────────────
    bringup_running = _svc_active(BRINGUP_SVC)

    if args.stop_bringup:
        # ════════════════════════════════════════════════════════════════════
        # PURE-SERIAL MODE: stop bringup, drive via Dynamixel, no ROS2
        # ════════════════════════════════════════════════════════════════════
        if bringup_running:
            print(f"  {YLW}Stopping {BRINGUP_SVC} …{NC}")
            subprocess.run(["sudo", "systemctl", "stop", BRINGUP_SVC], timeout=15)
            time.sleep(1.5)   # wait for port to be released
            _bringup_was_active = True
        else:
            # Bringup was never running (e.g. just freshly flashed).
            # Give the firmware a moment to finish IMU init before we start polling.
            time.sleep(2.0)

        port = args.port
        if not port:
            for p in DXL_PORTS:
                if os.path.exists(p):
                    port = p
                    break
        if not port:
            print(f"{RED}ERROR: no serial port found ({', '.join(DXL_PORTS)}){NC}")
            return 1

        try:
            import serial as _serial_mod  # noqa: F401
        except ImportError:
            print(f"{RED}ERROR: pyserial not installed (pip install pyserial){NC}")
            return 1

        try:
            poller = SerialPoller(port)
        except Exception as e:
            print(f"{RED}ERROR: cannot open {port}: {e}{NC}")
            return 1

        print(f"  Serial port: {port}  ({DXL_BAUD} baud)")
        # Wait for first valid register response.
        # Fresh-boot IMU init (BNO055) can take up to 5 s; allow 10 s total.
        print(f"  Waiting for firmware response … ", end="", flush=True)
        for _wi in range(100):   # up to 10 s
            time.sleep(0.1)
            r = poller.get()
            if r:
                print(f"{GRN}OK{NC}  heading_hold_en={r.heading_hold_en}  "
                      f"dbg_vel_L={r.dbg_vel_l:+.4f} m/s")
                break
            if _wi % 10 == 9:
                print(".", end="", flush=True)
        else:
            print(f"\n{RED}TIMEOUT — no Dynamixel response from firmware{NC}")
            # Debug: send one raw READ and dump whatever arrives
            try:
                import serial as _serial_mod2
                _probe_ser = _serial_mod2.Serial(port, DXL_BAUD, timeout=0.5)
                _probe_pkt = _make_read_pkt(BULK_ADDR, BULK_LEN)
                _probe_ser.reset_input_buffer()
                _probe_ser.write(_probe_pkt)
                _raw = _probe_ser.read(256)
                _probe_ser.close()
                if _raw:
                    print(f"  Raw bytes received ({len(_raw)}): {_raw[:64].hex(' ')}")
                    print(f"  Sent pkt ({len(_probe_pkt)}):      {_probe_pkt.hex(' ')}")
                    print(f"  BULK_ADDR={BULK_ADDR}  BULK_LEN={BULK_LEN}")
                else:
                    print(f"  No bytes received — firmware may not be running or port wrong")
            except Exception as _pe:
                print(f"  Raw probe failed: {_pe}")
            poller.close()
            return 1

        all_samples:   list[Sample] = []
        all_summaries: list[dict]   = []
        t0 = time.monotonic()

        try:
            for pair in range(1, args.pairs + 1):
                print(f"\n{CYN}──── Pair {pair}/{args.pairs} ────{NC}")

                print(f"\n  {BLD}[ FORWARD ]{NC}")
                fwd_samples, fwd_sum = run_serial_drive_pass(
                    poller, +args.speed, args.duration, "fwd", pair, t0)
                all_samples.extend(fwd_samples)
                all_summaries.append(fwd_sum)

                print(f"\n  Settle 2 s …")
                time.sleep(2.0)

                print(f"\n  {BLD}[ REVERSE ]{NC}")
                rev_samples, rev_sum = run_serial_drive_pass(
                    poller, -args.speed, args.duration, "rev", pair, t0)
                all_samples.extend(rev_samples)
                all_summaries.append(rev_sum)

                if pair < args.pairs:
                    print(f"\n  {YLW}Settle 3 s before next pair …{NC}")
                    time.sleep(3.0)

        except KeyboardInterrupt:
            print(f"\n{YLW}Interrupted — stopping.{NC}")
            poller.write_velocity(0.0)
        finally:
            poller.close()
            if _bringup_was_active:
                print(f"\n  Restarting {BRINGUP_SVC} …")
                subprocess.run(["sudo", "systemctl", "start", BRINGUP_SVC], timeout=15)
                print(f"  {GRN}{BRINGUP_SVC} restarted.{NC}")

    else:
        # ════════════════════════════════════════════════════════════════════
        # ROS MODE: bringup provides /odom, /imu, /joint_states; no serial
        # ════════════════════════════════════════════════════════════════════
        if not bringup_running:
            print(f"{RED}ERROR: {BRINGUP_SVC} is not running.{NC}")
            print(f"  Start it with:  sudo systemctl start {BRINGUP_SVC}")
            print(f"  Or use --stop-bringup for pure-serial mode (full register diagnostics).")
            return 1

        print(f"  Mode: ROS topics  (bringup running — serial skipped to avoid crash)")
        print(f"  For firmware register reads use: {BLD}--stop-bringup{NC}\n")

        os.environ.setdefault('ROS_DOMAIN_ID', '42')
        rclpy.init()
        node = DiagReverseNode()

        all_samples:   list[Sample] = []
        all_summaries: list[dict]   = []
        t0 = time.monotonic()

        try:
            if not node.wait_topics():
                return 1

            for pair in range(1, args.pairs + 1):
                print(f"\n{CYN}──── Pair {pair}/{args.pairs} ────{NC}")

                print(f"\n  {BLD}[ FORWARD ]{NC}")
                fwd_samples, fwd_sum = run_drive_pass(
                    node, None, +args.speed, args.duration, "fwd", pair, t0)
                all_samples.extend(fwd_samples)
                all_summaries.append(fwd_sum)

                print(f"\n  Settle 1.5 s …")
                node.spin_for(1.5)

                print(f"\n  {BLD}[ REVERSE ]{NC}")
                rev_samples, rev_sum = run_drive_pass(
                    node, None, -args.speed, args.duration, "rev", pair, t0)
                all_samples.extend(rev_samples)
                all_summaries.append(rev_sum)

                if pair < args.pairs:
                    print(f"\n  {YLW}Settle 3 s before next pair …{NC}")
                    node.spin_for(3.0)

        except KeyboardInterrupt:
            print(f"\n{YLW}Interrupted — stopping.{NC}")
        finally:
            node.stop()
            node.destroy_node()
            rclpy.shutdown()

    # ── Comparison report (shared by both modes) ───────────────────────────
    if not all_summaries:
        return 0

    fwd_sums = [s for s in all_summaries if s["phase"] == "fwd"]
    rev_sums = [s for s in all_summaries if s["phase"] == "rev"]

    def avg(lst, key, default=0.0):
        vals = [x[key] for x in lst if key in x]
        return sum(vals) / len(vals) if vals else default

    print(f"\n{BLD}══════════════════════════════════════════════════════{NC}")
    print(f"{BLD} COMPARISON: Forward vs Reverse{NC}")
    print(f"{BLD}══════════════════════════════════════════════════════{NC}")

    rows = [
        ("Distance travelled (cm)",   avg(fwd_sums,"dist_m")*100,    avg(rev_sums,"dist_m")*100),
        ("Heading drift odom (°)",    avg(fwd_sums,"hdg_drift_deg"), avg(rev_sums,"hdg_drift_deg")),
        ("Heading drift IMU  (°)",    avg(fwd_sums,"imu_drift_deg"), avg(rev_sums,"imu_drift_deg")),
        ("Mean gyro-Z (°/s)",         math.degrees(avg(fwd_sums,"mean_gyro_rads")),
                                      math.degrees(avg(rev_sums,"mean_gyro_rads"))),
        ("Vel L (mm/s)",              avg(fwd_sums,"fw_mean_vl_ms")*1000, avg(rev_sums,"fw_mean_vl_ms")*1000),
        ("Vel R (mm/s)",              avg(fwd_sums,"fw_mean_vr_ms")*1000, avg(rev_sums,"fw_mean_vr_ms")*1000),
        ("Vel ratio L/R",             avg(fwd_sums,"fw_ratio",1.0),        avg(rev_sums,"fw_ratio",1.0)),
        ("FW heading err  (°)",       avg(fwd_sums,"fw_mean_hdg_err_deg"), avg(rev_sums,"fw_mean_hdg_err_deg")),
        ("FW hold corr  (°/s)",       math.degrees(avg(fwd_sums,"fw_mean_hdg_corr_rads")),
                                      math.degrees(avg(rev_sums,"fw_mean_hdg_corr_rads"))),
        ("FW enc L−R diff (mm)",      avg(fwd_sums,"fw_mean_enc_diff_m")*1000,
                                      avg(rev_sums,"fw_mean_enc_diff_m")*1000),
        ("Gyro-Z std (°/s)",          math.degrees(avg(fwd_sums,"gyro_z_std_rads",0.0)),
                                      math.degrees(avg(rev_sums,"gyro_z_std_rads",0.0))),
        ("Gyro-Z peak (°/s)",         math.degrees(avg(fwd_sums,"gyro_z_peak_rads",0.0)),
                                      math.degrees(avg(rev_sums,"gyro_z_peak_rads",0.0))),
        ("Gyro-Z osc freq (Hz)",       avg(fwd_sums,"gyro_z_freq_hz",0.0),
                                       avg(rev_sums,"gyro_z_freq_hz",0.0)),
    ]

    col_w = 28
    print(f"  {'Metric':<{col_w}} {'Forward':>10}  {'Reverse':>10}  {'Δ (rev−fwd)':>13}")
    print(f"  {'─'*col_w} {'─'*10}  {'─'*10}  {'─'*13}")
    for label, fv, rv in rows:
        delta = rv - fv
        dfmt  = f"{delta:+10.3f}"
        flag  = ""
        if "drift" in label.lower() and "°" in label and abs(delta) > 3.0:
            flag = f"  {YLW}⚠ large asymmetry{NC}"
        elif "ratio" in label.lower() and abs(delta) > 0.05:
            flag = f"  {YLW}⚠ asymmetric{NC}"
        print(f"  {label:<{col_w}} {fv:>10.3f}  {rv:>10.3f}  {dfmt}{flag}")

    # Diagnostic interpretation
    print(f"\n{BLD}  Interpretation:{NC}")
    rev_hdg = avg(rev_sums, "hdg_drift_deg")
    fwd_hdg = avg(fwd_sums, "hdg_drift_deg")

    if abs(rev_hdg) > abs(fwd_hdg) + 2.0:
        dir_s = "left (CCW)" if rev_hdg > 0 else "right (CW)"
        print(f"  {YLW}⚠  Reverse drifts {dir_s} by {rev_hdg:+.1f}° vs fwd {fwd_hdg:+.1f}°{NC}")

    if args.stop_bringup:
        rev_hd = avg(rev_sums, "fw_mean_hdg_err_deg")
        rev_hc = math.degrees(avg(rev_sums, "fw_mean_hdg_corr_rads"))
        rev_ed = avg(rev_sums, "fw_mean_enc_diff_m") * 1000
        fwd_hc = math.degrees(avg(fwd_sums, "fw_mean_hdg_corr_rads"))
        fwd_ed = avg(fwd_sums, "fw_mean_enc_diff_m") * 1000

        if abs(rev_hd) > 2.0 and abs(rev_hc) < 0.5:
            print(f"  {RED}✗  Heading error {rev_hd:+.2f}° in reverse but correction only "
                  f"{rev_hc:+.2f}°/s — heading-hold not correcting!{NC}")
        elif abs(rev_hd) > 2.0:
            print(f"  {YLW}⚠  Heading error {rev_hd:+.2f}° with correction {rev_hc:+.2f}°/s "
                  f"— correction may be insufficient.{NC}")

        sign_flip = (fwd_hc * rev_hc < 0) and abs(fwd_hc) > 0.1 and abs(rev_hc) > 0.1
        if sign_flip:
            # Sign flip between fwd and rev correction is EXPECTED when different
            # motors dominate in each direction.  enc_diff encodes absolute (L−R)
            # differential, so the sign reflects which wheel is faster, not direction.
            # Only report as noteworthy if the heading error is also large.
            if abs(rev_hdg) > 5.0 or abs(fwd_hdg) > 5.0:
                print(f"  {YLW}ℹ  Correction sign differs fwd ({fwd_hc:+.2f}°/s) vs "
                      f"rev ({rev_hc:+.2f}°/s) — different motor dominates each direction.{NC}")

        if abs(rev_ed) > 5.0:
            wheel = "left" if rev_ed > 0 else "right"
            print(f"  {YLW}⚠  Enc L−R diff = {rev_ed:+.2f} mm in reverse "
                  f"({wheel} wheel faster backward){NC}")
        if abs(fwd_ed) > 5.0:
            wheel = "left" if fwd_ed > 0 else "right"
            print(f"  {YLW}⚠  Enc L−R diff = {fwd_ed:+.2f} mm forward "
                  f"({wheel} wheel faster forward){NC}")

        # Wobble / oscillation check
        fwd_wobble = avg(fwd_sums, "gyro_z_wobble", 0.0) >= 0.5
        rev_wobble = avg(rev_sums, "gyro_z_wobble", 0.0) >= 0.5
        fwd_std_deg = math.degrees(avg(fwd_sums, "gyro_z_std_rads", 0.0))
        rev_std_deg = math.degrees(avg(rev_sums, "gyro_z_std_rads", 0.0))
        fwd_freq    = avg(fwd_sums, "gyro_z_freq_hz", 0.0)
        rev_freq    = avg(rev_sums, "gyro_z_freq_hz", 0.0)
        if fwd_wobble or rev_wobble:
            phases = []
            if fwd_wobble:
                phases.append(f"forward ({fwd_std_deg:.1f}°/s std, {fwd_freq:.2f} Hz)")
            if rev_wobble:
                phases.append(f"reverse ({rev_std_deg:.1f}°/s std, {rev_freq:.2f} Hz)")
            print(f"  {RED}❌ Oscillation (wobble) detected in: {', '.join(phases)}{NC}")
            print(f"     The heading-hold controller is hunting. Suggestions:")
            print(f"       1. Reduce HEADING_HOLD_KP_DEFAULT (currently ~4.0) → try 2.0–3.0")
            print(f"       2. If only at high speed, also reduce HEADING_HOLD_MAX_CORR")
            print(f"       3. Slightly increase HEADING_HOLD_DEADBAND (currently 0.003 rad)")
        else:
            max_std = max(fwd_std_deg, rev_std_deg)
            if max_std < 2.0:
                print(f"  {GRN}✔  No wobble detected (peak gyro-Z std {max_std:.1f}°/s — stable){NC}")
            else:
                print(f"  {YLW}⚠  Gyro-Z std {max_std:.1f}°/s — borderline; monitor at higher speeds{NC}")
    print()

    # ── Write CSV ──────────────────────────────────────────────────────────
    if not args.no_csv and all_samples:
        ts_str   = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = Path(__file__).resolve().parent / f"diagnose_reverse_{ts_str}.csv"
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(CSV_HEADER)
            for s in all_samples:
                w.writerow(sample_to_row(s))
        print(f"  {GRN}CSV saved:{NC} {csv_path}  ({len(all_samples)} rows)\n")

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
