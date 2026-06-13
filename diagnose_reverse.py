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
from sensor_msgs.msg import Imu, JointState, Range

# ── colours ───────────────────────────────────────────────────────────────────
from dxl_utils import *  # noqa: F401,F403 — protocol, registers, colours

# ── ROS QoS ──────────────────────────────────────────────────────────────────
BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── Script-specific constants ─────────────────────────────────────────────────
# Bulk read: start at IMU gyro-Z (68) to capture all firmware regs in one shot.
BULK_ADDR   = REG_IMU_ANG_VEL_Z   # 68
BULK_LEN    = (REG_HEADING_HOLD_KD - BULK_ADDR) + 4  # 248 bytes  (68..315)

WHEEL_RADIUS   = 0.033   # m  (approx, used for Dynamixel RPM → m/s)
DXL_VEL_UNIT   = 0.229   # RPM per unit
RPM_TO_RADS    = 2.0 * math.pi / 60.0

# ── Ultrasonic collision detection ────────────────────────────────────────────
US_LEFT_GPIO   = 23
US_RIGHT_GPIO  = 24
US_COLLISION_MM = 150.0   # 15 cm


def _us_read_mm(h: int, gpio: int, timeout_us: int = 30000):
    """Single-wire trigger + echo on lgpio handle *h*. Returns (mm, err)."""
    try:
        import lgpio
    except ImportError:
        return None, "lgpio-missing"
    try:
        lgpio.gpio_claim_output(h, gpio, 0)
        time.sleep(2e-6)
        lgpio.gpio_write(h, gpio, 1)
        time.sleep(10e-6)
        lgpio.gpio_write(h, gpio, 0)
        lgpio.gpio_free(h, gpio)
        lgpio.gpio_claim_input(h, gpio)
        start_ns = time.perf_counter_ns()
        timeout_ns = int(timeout_us * 1000)
        while lgpio.gpio_read(h, gpio) == 0:
            if (time.perf_counter_ns() - start_ns) > timeout_ns:
                lgpio.gpio_free(h, gpio)
                return None, "wait-timeout"
        rise_ns = time.perf_counter_ns()
        while lgpio.gpio_read(h, gpio) == 1:
            if (time.perf_counter_ns() - rise_ns) > timeout_ns:
                lgpio.gpio_free(h, gpio)
                return None, "pulse-timeout"
        fall_ns = time.perf_counter_ns()
        lgpio.gpio_free(h, gpio)
        pulse_us = (fall_ns - rise_ns) / 1000.0
        dist_mm = (pulse_us * 10.0) / 58.0
        if dist_mm < 20.0 or dist_mm > 3500.0:
            return None, "out-of-range"
        return dist_mm, None
    except Exception as e:
        return None, str(e)


class UltrasonicGuard:
    """Background thread that polls ultrasonic sensors and sets an abort flag
    when an obstacle is detected within US_COLLISION_MM.

    Safe to construct even if lgpio is unavailable — ``available`` will be False
    and the guard becomes a no-op.
    """

    def __init__(self, left_gpio: int = US_LEFT_GPIO,
                 right_gpio: int = US_RIGHT_GPIO,
                 collision_mm: float = US_COLLISION_MM):
        self.collision_mm = collision_mm
        self.abort = False          # set True on collision detection
        self.last_left_mm:  Optional[float] = None
        self.last_right_mm: Optional[float] = None
        self.available = False
        self._running = False
        self._left_gpio  = left_gpio
        self._right_gpio = right_gpio
        self._h: Optional[int] = None
        self._thread: Optional[threading.Thread] = None

        try:
            import lgpio
            # Auto-detect gpiochip: Pi 5 uses gpiochip4, Pi 4 uses 0
            for chip in (4, 0):
                try:
                    self._h = lgpio.gpiochip_open(chip)
                    break
                except Exception:
                    continue
            if self._h is None:
                return
            self.available = True
        except ImportError:
            pass

    def start(self):
        if not self.available:
            return
        self.abort = False
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def close(self):
        self.stop()
        if self._h is not None:
            try:
                import lgpio
                lgpio.gpiochip_close(self._h)
            except Exception:
                pass
            self._h = None
        self.available = False

    def _loop(self):
        while self._running:
            left_mm, _ = _us_read_mm(self._h, self._left_gpio)
            time.sleep(0.03)
            right_mm, _ = _us_read_mm(self._h, self._right_gpio)
            self.last_left_mm  = left_mm
            self.last_right_mm = right_mm
            if left_mm is not None and left_mm < self.collision_mm:
                self.abort = True
                self._running = False
                return
            if right_mm is not None and right_mm < self.collision_mm:
                self.abort = True
                self._running = False
                return
            time.sleep(0.06)  # ~10 Hz per sensor


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
    heading_hold_kp: float = 0.0  # current KP value from firmware
    heading_hold_kd: float = 0.0  # current KD value from firmware
    heading_hold_iterm: float = 0.0  # current integral accumulator
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
    r.heading_hold_kp = f32(REG_HEADING_HOLD_KP)
    r.heading_hold_kd = f32(REG_HEADING_HOLD_KD)
    r.heading_hold_iterm = f32(REG_HDG_ITERM)
    r.gyro_z       = f32(REG_IMU_ANG_VEL_Z)
    return r

# ── Wobble / oscillation analysis ────────────────────────────────────────────
def _osc_stats(values: list, tss: list, deadband: float = 0.015) -> dict:
    """Compute oscillation statistics (std, peak, frequency) for a signal.

    Returns dict with keys: std, peak, freq_hz, crossings, wobble.
    """
    result = dict(std=0.0, peak=0.0, freq_hz=0.0, crossings=0, wobble=False)
    n = len(values)
    if n < 4:
        return result
    mean = sum(values) / n
    std  = math.sqrt(sum((v - mean) ** 2 for v in values) / max(n - 1, 1))
    peak = max(abs(v) for v in values)
    result["std"]  = std
    result["peak"] = peak
    # Count zero crossings (ignoring values inside deadband)
    crossings = 0
    prev_sign = None
    for v in values:
        if abs(v) < deadband:
            continue
        s = 1 if v > 0 else -1
        if prev_sign is not None and s != prev_sign:
            crossings += 1
        prev_sign = s
    duration = (tss[-1] - tss[0]) if len(tss) >= 2 else 1.0
    freq_hz = (crossings / 2.0) / max(duration, 1e-3)
    result["freq_hz"]   = freq_hz
    result["crossings"] = crossings
    result["wobble"] = std > 0.04 and crossings >= 4 and freq_hz >= 0.3
    return result

def _wobble_analysis(gyro_zs: list, tss: list,
                     enc_vls: list | None = None,
                     enc_vrs: list | None = None) -> dict:
    """Detect heading-hold hunting from gyro-Z AND encoder velocity differential.

    Returns a dict with:
      std, peak, freq_hz, wobble, msg  — gyro-Z based (as before)
      enc_std, enc_peak, enc_freq_hz, enc_wobble, enc_msg  — encoder-based
      source  — "encoder" or "gyro" (which one determined the verdict)
    """
    result = dict(std=0.0, peak=0.0, freq_hz=0.0, wobble=False, msg="",
                  enc_std=0.0, enc_peak=0.0, enc_freq_hz=0.0, enc_wobble=False,
                  enc_msg="", source="gyro")

    # Gyro-Z oscillation stats
    gyro = _osc_stats(gyro_zs, tss, deadband=0.015)
    result["std"]     = gyro["std"]
    result["peak"]    = gyro["peak"]
    result["freq_hz"] = gyro["freq_hz"]

    # Encoder-based angular velocity: ω = (v_l − v_r) / wheel_base
    has_enc = (enc_vls is not None and enc_vrs is not None
               and len(enc_vls) == len(enc_vrs) and len(enc_vls) >= 4)
    if has_enc:
        enc_omega = [(vl - vr) / WHEEL_BASE for vl, vr in zip(enc_vls, enc_vrs)]
        # Detrend: remove linear component so std measures oscillation, not drift.
        n_enc = len(enc_omega)
        ts_rel = [t - tss[0] for t in tss[:n_enc]]
        mean_t = sum(ts_rel) / n_enc
        mean_o = sum(enc_omega) / n_enc
        cov_to = sum((t - mean_t) * (o - mean_o) for t, o in zip(ts_rel, enc_omega))
        var_t  = sum((t - mean_t) ** 2 for t in ts_rel)
        slope  = cov_to / var_t if var_t > 1e-12 else 0.0
        intercept = mean_o - slope * mean_t
        detrended = [o - (slope * t + intercept) for t, o in zip(ts_rel, enc_omega)]
        enc = _osc_stats(detrended, tss, deadband=0.020)  # wider deadband — encoder quantization noise floor ~0.9°/s
        result["enc_std"]     = enc["std"]
        result["enc_peak"]    = enc["peak"]
        result["enc_freq_hz"] = enc["freq_hz"]
        result["enc_wobble"]  = enc["wobble"] and enc["std"] > 0.06  # 3.4°/s threshold; below this is motor ripple
        # Encoder wobble is the ground truth (immune to vibration).
        # Gyro wobble without encoder wobble → vibration noise, not real oscillation.
        result["wobble"] = enc["wobble"]
        result["source"] = "encoder"
    else:
        # Fall back to gyro-based detection
        result["wobble"] = gyro["wobble"]
        result["source"] = "gyro"

    # Format messages
    std_deg  = math.degrees(result["std"])
    peak_deg = math.degrees(result["peak"])
    if result["source"] == "encoder" and has_enc:
        enc_std_deg  = math.degrees(result["enc_std"])
        enc_peak_deg = math.degrees(result["enc_peak"])
        if result["enc_wobble"]:
            result["enc_msg"] = (
                f"⚠ WOBBLE  ±{enc_peak_deg:.1f}°/s peak  std={enc_std_deg:.1f}°/s  "
                f"freq={result['enc_freq_hz']:.2f} Hz  → reduce KP"
            )
        else:
            result["enc_msg"] = (
                f"stable   std={enc_std_deg:.1f}°/s  peak=±{enc_peak_deg:.1f}°/s"
            )

    if gyro["wobble"]:
        label = "(vibration)" if (has_enc and not result["enc_wobble"]) else ""
        result["msg"] = (
            f"std={std_deg:.1f}°/s  peak=±{peak_deg:.1f}°/s  "
            f"freq={result['freq_hz']:.2f} Hz  {label}"
        )
    else:
        result["msg"] = f"stable   std={std_deg:.1f}°/s  peak=±{peak_deg:.1f}°/s"
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
    def __init__(self, collision_mm: float = US_COLLISION_MM):
        super().__init__("diagnose_reverse")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._odom: Optional[Odometry] = None
        self._imu:  Optional[Imu]      = None
        self._joint: Optional[JointState] = None
        self._us_left:  Optional[Range] = None
        self._us_right: Optional[Range] = None
        self._collision_mm = collision_mm
        self.create_subscription(Odometry,    "/odom",            self._odom_cb,   BEST_EFFORT_QOS)
        self.create_subscription(Imu,         "/imu",             self._imu_cb,    BEST_EFFORT_QOS)
        self.create_subscription(JointState,  "/joint_states",    self._joint_cb,  BEST_EFFORT_QOS)
        self.create_subscription(Range,       "/ultrasonic/left", self._us_l_cb,   BEST_EFFORT_QOS)
        self.create_subscription(Range,       "/ultrasonic/right",self._us_r_cb,   BEST_EFFORT_QOS)

    def _odom_cb(self, m: Odometry):   self._odom  = m
    def _imu_cb(self,  m: Imu):        self._imu   = m
    def _joint_cb(self,m: JointState): self._joint = m
    def _us_l_cb(self, m: Range):      self._us_left  = m
    def _us_r_cb(self, m: Range):      self._us_right = m

    def collision_detected(self) -> bool:
        """Return True if either ultrasonic sensor reports closer than threshold."""
        thresh_m = self._collision_mm / 1000.0
        for rng in (self._us_left, self._us_right):
            if rng is not None and 0.0 < rng.range < thresh_m:
                return True
        return False

    def spin_for(self, secs: float):
        t = time.monotonic() + secs
        while time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, lin: float, ang: float = 0.0):
        if not rclpy.ok():
            return
        msg = Twist()
        msg.linear.x = lin; msg.angular.z = ang
        self._pub.publish(msg)

    def stop(self, settle: float = 0.5):
        if not rclpy.ok():
            return
        try:
            self.send(0.0)
            self.spin_for(settle)
        except Exception:
            pass

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
        self._pkt     = make_read_pkt(BULK_ADDR, BULK_LEN)
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
                    raw = read_response(self._ser, BULK_LEN)
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
                self._ser.write(make_write_pkt(REG_CMD_LINEAR_X, struct.pack("<i", lin_val)))
                # Read (and discard) the firmware's write-acknowledge STATUS packet
                # so it doesn't accumulate in the RX buffer for the poller thread.
                self._ser.read(11)   # typical write-ACK: 4 hdr + ID + len_l + len_h + instr + err + crc2
                self._ser.write(make_write_pkt(REG_CMD_ANGULAR_Z, struct.pack("<i", ang_val)))
                self._ser.read(11)
        except Exception:
            pass

    def query_wheel_separation(self) -> Optional[float]:
        """Read wheel separation from firmware calibration blob."""
        try:
            with self._bus_lock:
                return query_wheel_separation(self._ser)
        except Exception:
            pass
        return None

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
        # Ultrasonic collision check (ROS mode)
        if node.collision_detected():
            node.stop(settle=0.3)
            print(f"  {RED}COLLISION{NC}  ultrasonic e-stop at {elapsed:.1f} s")
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

    # Wobble analysis from ROS gyro-Z + encoder velocity (if available)
    gz_all = [s.gyro_z for s in samples]
    ts_all = [s.ts     for s in samples]
    enc_vl_all = [s.fw_dbg_vl for s in samples]
    enc_vr_all = [s.fw_dbg_vr for s in samples]
    # Only pass encoder velocities if they contain real data (not all zeros)
    has_enc_vel = any(abs(v) > 0.001 for v in enc_vl_all)
    wobble = _wobble_analysis(gz_all, ts_all,
                              enc_vls=enc_vl_all if has_enc_vel else None,
                              enc_vrs=enc_vr_all if has_enc_vel else None)

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
        gyro_z_std_rads=wobble["std"],
        gyro_z_peak_rads=wobble["peak"],
        gyro_z_freq_hz=wobble["freq_hz"],
        gyro_z_wobble=1.0 if wobble["wobble"] else 0.0,
        enc_wob_std_rads=wobble.get("enc_std", 0.0),
        enc_wob_peak_rads=wobble.get("enc_peak", 0.0),
        enc_wob_freq_hz=wobble.get("enc_freq_hz", 0.0),
        enc_wobble=1.0 if wobble.get("enc_wobble", False) else 0.0,
        heading_hold_kp=None,
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
    col_wb = RED if wobble["wobble"] else GRN
    print(f"    Gyro-Z wobble:    {col_wb}{wobble['msg']}{NC}")

    return samples, summary


# ── Motor startup asymmetry analysis ─────────────────────────────────────────
def _analyze_startup(ser, speed: float, burst_s: float = 0.6) -> dict:
    """High-frequency sampling of the first `burst_s` seconds after commanding
    velocity to detect motor startup asymmetry.

    Reads encoder counts, debug velocities, heading error, and gyro-Z as fast
    as the serial bus allows (~2-3 ms per round-trip = 300-500 Hz) for the first
    burst_s seconds.

    Returns dict with:
      - lag_ms:       estimated L-R motor start time difference (ms, >0 = left leads)
      - drift_deg:    heading drift accumulated during transient (degrees)
      - hdg_err_deg:  peak heading error during transient (degrees)
      - ramp_time_l_ms, ramp_time_r_ms: time for each motor to reach >50% of target velocity
      - n_samples:    number of high-speed samples collected
      - first_move_l_ms, first_move_r_ms: time until each encoder starts moving
      - samples:      raw list of (t_ms, enc_l, enc_r, vel_l, vel_r, hdg_err, gyro_z)
    """
    READ_PKT = make_read_pkt(BULK_ADDR, BULK_LEN)

    def _fast_read(ser) -> Optional[FwRegs]:
        ser.reset_input_buffer()
        ser.write(READ_PKT)
        raw = read_response(ser, BULK_LEN, timeout=0.05)
        return _parse_bulk(raw) if raw else None

    # Take a baseline snapshot before commanding velocity
    baseline = _fast_read(ser)
    if not baseline:
        return dict(lag_ms=0.0, drift_deg=0.0, hdg_err_deg=0.0,
                    ramp_time_l_ms=0.0, ramp_time_r_ms=0.0,
                    first_move_l_ms=0.0, first_move_r_ms=0.0,
                    n_samples=0, samples=[])

    enc_l_base = baseline.enc_l_count
    enc_r_base = baseline.enc_r_count

    # Command velocity — this is the moment we're measuring from
    _send_vel_cmd(ser, speed)
    t0 = time.monotonic()

    raw_samples = []  # (t_ms, enc_l, enc_r, vel_l, vel_r, hdg_err, gyro_z)

    while time.monotonic() - t0 < burst_s:
        fw = _fast_read(ser)
        if fw:
            t_ms = (time.monotonic() - t0) * 1000.0
            raw_samples.append((
                t_ms,
                fw.enc_l_count, fw.enc_r_count,
                fw.dbg_vel_l,   fw.dbg_vel_r,
                fw.heading_err, fw.gyro_z,
            ))
        # Refresh velocity command every ~100ms to keep watchdog alive
        if raw_samples and raw_samples[-1][0] > 0 and int(raw_samples[-1][0]) % 100 < 5:
            _send_vel_cmd(ser, speed)

    if len(raw_samples) < 3:
        return dict(lag_ms=0.0, drift_deg=0.0, hdg_err_deg=0.0,
                    ramp_time_l_ms=0.0, ramp_time_r_ms=0.0,
                    first_move_l_ms=0.0, first_move_r_ms=0.0,
                    n_samples=len(raw_samples), samples=raw_samples)

    # --- First encoder movement detection ---
    # Find when each encoder first changes from baseline
    ENC_MOVE_THRESH = 2    # counts — noise-immune threshold
    first_move_l_ms = None
    first_move_r_ms = None
    for t_ms, enc_l, enc_r, *_ in raw_samples:
        if first_move_l_ms is None and abs(enc_l - enc_l_base) >= ENC_MOVE_THRESH:
            first_move_l_ms = t_ms
        if first_move_r_ms is None and abs(enc_r - enc_r_base) >= ENC_MOVE_THRESH:
            first_move_r_ms = t_ms
        if first_move_l_ms is not None and first_move_r_ms is not None:
            break

    if first_move_l_ms is None:
        first_move_l_ms = burst_s * 1000
    if first_move_r_ms is None:
        first_move_r_ms = burst_s * 1000

    # Lag: positive = left starts first, negative = right starts first
    lag_ms = first_move_r_ms - first_move_l_ms

    # --- Ramp time: when each motor reaches 50% of target velocity ---
    target_v = abs(speed)
    half_v   = target_v * 0.5
    ramp_l_ms = burst_s * 1000
    ramp_r_ms = burst_s * 1000
    for t_ms, _, _, vel_l, vel_r, *_ in raw_samples:
        if ramp_l_ms >= burst_s * 1000 and abs(vel_l) >= half_v:
            ramp_l_ms = t_ms
        if ramp_r_ms >= burst_s * 1000 and abs(vel_r) >= half_v:
            ramp_r_ms = t_ms
        if ramp_l_ms < burst_s * 1000 and ramp_r_ms < burst_s * 1000:
            break

    # --- Heading drift during transient ---
    # Encoder-based drift at end of burst
    enc_counts_per_m = 3840.0 / (2.0 * math.pi * WHEEL_RADIUS)
    last = raw_samples[-1]
    dl_m = (last[1] - enc_l_base) / enc_counts_per_m
    dr_m = (last[2] - enc_r_base) / enc_counts_per_m
    drift_rad = (dl_m - dr_m) / WHEEL_BASE
    drift_deg = math.degrees(drift_rad)

    # Peak heading error reported by firmware during transient
    peak_hdg = max(abs(s[5]) for s in raw_samples)
    peak_hdg_deg = math.degrees(peak_hdg)

    return dict(
        lag_ms=lag_ms,
        drift_deg=drift_deg,
        hdg_err_deg=peak_hdg_deg,
        ramp_time_l_ms=ramp_l_ms,
        ramp_time_r_ms=ramp_r_ms,
        first_move_l_ms=first_move_l_ms,
        first_move_r_ms=first_move_r_ms,
        n_samples=len(raw_samples),
        samples=raw_samples,
    )


# ── Pure-serial drive pass (used when bringup is stopped) ────────────────────
def run_serial_drive_pass(poller: "SerialPoller",
                          speed: float, duration: float,
                          phase: str, pair_num: int,
                          t0_global: float,
                          us_guard: Optional["UltrasonicGuard"] = None) -> tuple[list[Sample], dict]:
    """Drive the robot via Dynamixel register writes and sample firmware regs.
    Does not require ROS2 / turtlebot3_node.

    All I/O is done inline on the calling thread (no background thread races):
    each iteration writes the velocity command, then immediately reads registers
    while the bus is idle.  This guarantees the read never conflicts with a
    concurrent write and that the poller's _read_response timeout can never
    fire mid-response.
    """
    READ_PKT = make_read_pkt(BULK_ADDR, BULK_LEN)

    def _send_vel(ser, v: float, a: float = 0.0) -> None:
        """Write lin_x and ang_z registers; discard any write-ACK bytes."""
        lin_val = int(round(v * 100.0))
        ang_val = int(round(a * 100.0))
        ser.write(make_write_pkt(REG_CMD_LINEAR_X, struct.pack("<i", lin_val)))
        time.sleep(0.003)                  # let firmware ACK (11 bytes ≈ 0.11 ms at 1 Mbaud)
        ser.read(max(ser.in_waiting, 1))   # drain write-ACK without blocking on fixed count
        ser.write(make_write_pkt(REG_CMD_ANGULAR_Z, struct.pack("<i", ang_val)))
        time.sleep(0.003)
        ser.read(max(ser.in_waiting, 1))

    def _read_regs(ser) -> Optional[FwRegs]:
        """Send one bulk READ and parse the response.  ~2 ms at 1 Mbaud."""
        ser.reset_input_buffer()
        ser.write(READ_PKT)
        raw = read_response(ser, BULK_LEN)
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

    # ── High-speed startup transient analysis (first ~0.6s) ──────────────
    startup_info = _analyze_startup(ser, speed, burst_s=0.6)

    t_start = time.monotonic()
    t_end   = t_start + max(duration - 0.6, 0.5)  # remaining time after startup burst

    try:
        # Continue driving at the commanded speed (startup analysis already sent it)
        _send_vel(ser, speed)

        while time.monotonic() < t_end:
            # ── Collision check ──────────────────────────────────────
            if us_guard and us_guard.abort:
                print(f"\n    {RED}⚠ COLLISION DETECTED — emergency stop!{NC}")
                _send_vel(ser, 0.0)
                break

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

    # Prefer first/last sample encoder counts (more reliable than fw_final read)
    if len(samples) >= 2:
        enc_l_start_s = samples[0].fw_enc_l
        enc_r_start_s = samples[0].fw_enc_r
        enc_l_end_s   = samples[-1].fw_enc_l
        enc_r_end_s   = samples[-1].fw_enc_r
        dl_m = (enc_l_end_s - enc_l_start_s) / enc_counts_per_m
        dr_m = (enc_r_end_s - enc_r_start_s) / enc_counts_per_m
    else:
        dl_m = ((enc_l_end - (enc_l_start or enc_l_end)) / enc_counts_per_m)
        dr_m = ((enc_r_end - (enc_r_start or enc_r_end)) / enc_counts_per_m)

    # Trim steady-state: middle 60% (skip ramp-up / ramp-down)
    trim  = len(samples) // 5
    mid   = samples[trim:-trim] if len(samples) > 5 else samples

    dvls  = [s.fw_dbg_vl   for s in mid]
    dvrs  = [s.fw_dbg_vr   for s in mid]
    hds   = [s.fw_hdg_err  for s in mid]
    hcs   = [s.fw_hdg_corr for s in mid]
    eds   = [s.fw_enc_diff for s in mid]
    gzs   = [s.fw_gyro_z   for s in samples]   # wobble: use ALL samples
    gtss  = [s.ts           for s in samples]
    # Encoder-based angular velocities for all samples (immune to vibration)
    enc_vls = [s.fw_dbg_vl for s in samples]
    enc_vrs = [s.fw_dbg_vr for s in samples]

    mean_dvl = safe_mean(dvls)
    mean_dvr = safe_mean(dvrs)
    mean_hd  = safe_mean(hds)
    mean_hc  = safe_mean(hcs)
    mean_ed  = safe_mean(eds)

    wobble   = _wobble_analysis(gzs, gtss, enc_vls=enc_vls, enc_vrs=enc_vrs)

    # Integrate gyro-Z over time for heading drift estimate
    gyro_hdg_drift_rad = 0.0
    for i in range(1, len(samples)):
        dt = samples[i].ts - samples[i-1].ts
        gyro_hdg_drift_rad += samples[i].fw_gyro_z * dt
    mean_gyro_z_rads = safe_mean([s.fw_gyro_z for s in mid])

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
    # Encoder-based heading drift: Δθ = (dl - dr) / wheel_base
    enc_hdg_drift_rad = (dl_m - dr_m) / WHEEL_BASE

    print(f"    Enc travel:       L={dl_m*100:+.1f} cm   R={dr_m*100:+.1f} cm")
    col_enc_hdg = GRN if abs(math.degrees(enc_hdg_drift_rad)) < 2.0 else (
        YLW if abs(math.degrees(enc_hdg_drift_rad)) < 5.0 else RED)
    print(f"    Heading drift:    {col_enc_hdg}{math.degrees(enc_hdg_drift_rad):+6.2f}°{NC}  (encoder-based)")
    print(f"    Gyro-Z drift:     {math.degrees(gyro_hdg_drift_rad):+6.2f}°  (∫gyro-Z — includes oscillation bias)")
    print(f"    Mean gyro-Z:      {math.degrees(mean_gyro_z_rads):+6.2f}°/s")
    # Encoder-based wobble — ground truth (immune to BNO055 vibration noise)
    if wobble["source"] == "encoder" and wobble["enc_msg"]:
        col_enc_wb = RED if wobble["enc_wobble"] else GRN
        print(f"    Enc wobble:       {col_enc_wb}{wobble['enc_msg']}{NC}")
    # Gyro-Z wobble — informational (may include vibration artifacts)
    col_wb = YLW if wobble.get("std", 0) > 0.04 else GRN
    print(f"    Gyro-Z wobble:    {col_wb}{wobble['msg']}{NC}")

    # ── Startup motor asymmetry analysis ──────────────────────────────────
    if startup_info and startup_info.get("n_samples", 0) >= 3:
        si = startup_info
        lag = si["lag_ms"]
        col_lag = GRN if abs(lag) < 20 else (YLW if abs(lag) < 50 else RED)
        leader = "L leads" if lag > 0 else ("R leads" if lag < 0 else "even")
        print(f"    Startup lag:      {col_lag}{abs(lag):.0f} ms ({leader}){NC}  "
              f"first move L={si['first_move_l_ms']:.0f} ms  R={si['first_move_r_ms']:.0f} ms")

        ramp_diff = abs(si["ramp_time_l_ms"] - si["ramp_time_r_ms"])
        col_ramp = GRN if ramp_diff < 30 else (YLW if ramp_diff < 80 else RED)
        print(f"    Ramp to 50%:      {col_ramp}L={si['ramp_time_l_ms']:.0f} ms  "
              f"R={si['ramp_time_r_ms']:.0f} ms  (Δ={ramp_diff:.0f} ms){NC}")

        col_sd = GRN if abs(si["drift_deg"]) < 1.0 else (YLW if abs(si["drift_deg"]) < 3.0 else RED)
        print(f"    Startup drift:    {col_sd}{si['drift_deg']:+.2f}°{NC}  (encoder-based, first {len(si['samples'])} samples)")
        print(f"    Peak hdg err:     {si['hdg_err_deg']:.2f}°  (during transient)")
    else:
        startup_info = startup_info or {}

    actual_kp = None
    actual_kd = None
    fw_snap = poller.get()
    if fw_snap and fw_snap.heading_hold_kp > 0:
        actual_kp = fw_snap.heading_hold_kp
    if fw_snap:
        actual_kd = fw_snap.heading_hold_kd

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
        # Encoder-based wobble metrics (ground truth)
        enc_wob_std_rads=wobble.get("enc_std", 0.0),
        enc_wob_peak_rads=wobble.get("enc_peak", 0.0),
        enc_wob_freq_hz=wobble.get("enc_freq_hz", 0.0),
        enc_wobble=1.0 if wobble.get("enc_wobble", False) else 0.0,
        heading_hold_kp=actual_kp,
        heading_hold_kd=actual_kd,
        # Encoder-based heading drift (reliable even with wobble)
        hdg_drift_deg=math.degrees(enc_hdg_drift_rad),
        # Gyro-Z integrated drift (for reference; unreliable when wobble present)
        gyro_hdg_drift_deg=math.degrees(gyro_hdg_drift_rad),
        imu_drift_deg=math.degrees(enc_hdg_drift_rad),
        dx_m=0.0, dy_m=0.0,
        mean_vl_ms=mean_dvl, mean_vr_ms=mean_dvr, js_ratio=fw_ratio,
        mean_gyro_rads=mean_gyro_z_rads,
        # Motor startup asymmetry metrics
        startup_lag_ms=startup_info.get("lag_ms", 0.0),
        startup_drift_deg=startup_info.get("drift_deg", 0.0),
        startup_peak_hdg_err_deg=startup_info.get("hdg_err_deg", 0.0),
        startup_ramp_l_ms=startup_info.get("ramp_time_l_ms", 0.0),
        startup_ramp_r_ms=startup_info.get("ramp_time_r_ms", 0.0),
        startup_first_move_l_ms=startup_info.get("first_move_l_ms", 0.0),
        startup_first_move_r_ms=startup_info.get("first_move_r_ms", 0.0),
    )
    return samples, summary


# ── Auto-tune ────────────────────────────────────────────────────────────────
def _send_vel_cmd(ser, v: float, a: float = 0.0) -> None:
    """Write lin_x and ang_z registers; discard any write-ACK bytes.

    Module-level version of the local _send_vel inside run_serial_drive_pass.
    """
    lin_val = int(round(v * 100.0))
    ang_val = int(round(a * 100.0))
    ser.write(make_write_pkt(REG_CMD_LINEAR_X, struct.pack("<i", lin_val)))
    time.sleep(0.003)
    ser.read(max(ser.in_waiting, 1))
    ser.write(make_write_pkt(REG_CMD_ANGULAR_Z, struct.pack("<i", ang_val)))
    time.sleep(0.003)
    ser.read(max(ser.in_waiting, 1))

def _write_f32(ser, addr: int, value: float) -> None:
    """Write a float register via Dynamixel, draining the write-ACK."""
    ser.write(make_write_pkt(addr, struct.pack("<f", value)))
    time.sleep(0.003)
    ser.read(max(ser.in_waiting, 1))

def _write_u8(ser, addr: int, value: int) -> None:
    """Write a uint8 register via Dynamixel, draining the write-ACK."""
    ser.write(make_write_pkt(addr, bytes([value & 0xFF])))
    time.sleep(0.003)
    ser.read(max(ser.in_waiting, 1))

def _read_regs_inline(ser) -> Optional[FwRegs]:
    """Do a single inline bulk-read and parse. Assumes bus is idle."""
    ser.reset_input_buffer()
    ser.write(make_read_pkt(BULK_ADDR, BULK_LEN))
    raw = read_response(ser, BULK_LEN, timeout=0.15)
    return _parse_bulk(raw) if raw else None

def _collect_wobble_window(ser, speed: float, window_s: float = 3.0,
                          us_guard: Optional["UltrasonicGuard"] = None) -> dict:
    """Drive at `speed` for `window_s` seconds and return encoder-based wobble + drift stats.

    Returns dict with keys: enc_std, enc_peak, enc_freq_hz, enc_wobble,
    mean_vl, mean_vr, n_samples, drift_deg, heading_err_deg.
    """
    tss     = []
    enc_vls = []
    enc_vrs = []
    enc_l_counts = []   # raw encoder counts for drift computation
    enc_r_counts = []
    heading_errs = []   # firmware heading error (rad)
    t0 = time.monotonic()
    while time.monotonic() - t0 < window_s:
        if us_guard and us_guard.abort:
            _send_vel_cmd(ser, 0.0)
            break
        _send_vel_cmd(ser, speed)
        fw = _read_regs_inline(ser)
        if fw:
            tss.append(time.monotonic())
            enc_vls.append(fw.dbg_vel_l)
            enc_vrs.append(fw.dbg_vel_r)
            enc_l_counts.append(fw.enc_l_count)
            enc_r_counts.append(fw.enc_r_count)
            heading_errs.append(fw.heading_err)
        time.sleep(0.08)
    # Compute encoder angular velocity ω = (vl - vr) / wheel_base
    if len(tss) < 6:
        return dict(enc_std=float("inf"), enc_peak=float("inf"),
                    enc_freq_hz=0.0, enc_wobble=True,
                    mean_vl=0.0, mean_vr=0.0, n_samples=len(tss),
                    drift_deg=float("inf"), heading_err_deg=float("inf"))
    enc_omega = [(vl - vr) / WHEEL_BASE for vl, vr in zip(enc_vls, enc_vrs)]

    # Detrend: remove linear component so std measures oscillation, not veering.
    # Without this, a robot veering due to motor asymmetry (monotonic heading
    # error growth) inflates enc_std and fools the tuner into thinking the
    # controller is hunting when it's actually just too weak.
    n = len(enc_omega)
    ts_rel = [t - tss[0] for t in tss]
    mean_t = sum(ts_rel) / n
    mean_o = sum(enc_omega) / n
    cov_to = sum((t - mean_t) * (o - mean_o) for t, o in zip(ts_rel, enc_omega))
    var_t  = sum((t - mean_t) ** 2 for t in ts_rel)
    slope  = cov_to / var_t if var_t > 1e-12 else 0.0
    intercept = mean_o - slope * mean_t
    detrended = [o - (slope * t + intercept) for t, o in zip(ts_rel, enc_omega)]
    stats = _osc_stats(detrended, tss, deadband=0.020)

    # Also report the raw (non-detrended) std for diagnostics.
    raw_std = math.sqrt(sum((v - mean_o) ** 2 for v in enc_omega) / max(n - 1, 1))

    mean_vl = sum(enc_vls) / len(enc_vls)
    mean_vr = sum(enc_vrs) / len(enc_vrs)

    # Encoder-based heading drift: Δθ = (dl - dr) / wheel_base
    enc_counts_per_m = 3840.0 / (2.0 * math.pi * WHEEL_RADIUS)
    dl_m = (enc_l_counts[-1] - enc_l_counts[0]) / enc_counts_per_m
    dr_m = (enc_r_counts[-1] - enc_r_counts[0]) / enc_counts_per_m
    drift_rad = (dl_m - dr_m) / WHEEL_BASE
    drift_deg = math.degrees(drift_rad)

    # Mean heading error reported by firmware heading-hold controller
    mean_hdg_err_deg = math.degrees(sum(heading_errs) / len(heading_errs))

    return dict(
        enc_std=stats["std"], enc_peak=stats["peak"],
        enc_freq_hz=stats["freq_hz"],
        enc_wobble=stats["wobble"] and stats["std"] > 0.06,
        mean_vl=mean_vl, mean_vr=mean_vr, n_samples=len(tss),
        drift_deg=drift_deg, heading_err_deg=mean_hdg_err_deg,
    )


# Gain table: (register_addr, label, initial_value, min, max, step_down_factor)
_TUNABLE_GAINS = [
    # Velocity PID
    (REG_PID_KP,          "vel_pid.Kp",    None, 0.3,  3.0),
    (REG_PID_KI,          "vel_pid.Ki",    None, 1.0, 10.0),
    # Heading-hold
    (REG_HEADING_HOLD_KP, "hdg_hold.Kp",  None, 0.2,  4.0),
    (REG_HEADING_HOLD_KI, "hdg_hold.Ki",  None, 0.0,  2.0),
    (REG_HEADING_HOLD_KD, "hdg_hold.Kd",  None, 0.0,  2.0),
]

# Stability target: encoder angular-velocity std below this (rad/s ≈ 2.3°/s)
_STABLE_STD_RADS   = 0.04
# Velocity accuracy: must stay within this fraction of commanded speed
_VEL_ACCURACY_FRAC = 0.10  # 10%
# Maximum acceptable heading drift per measurement window (degrees).
# 3° is realistic for hardware with ~8% motor asymmetry at 0.1 m/s.
_MAX_DRIFT_DEG     = 3.0


def run_auto_tune(poller, speed: float, max_rounds: int = 12,
                  us_guard: Optional["UltrasonicGuard"] = None) -> dict:
    """Three-phase auto-tuner + optional feedforward calibration.

    Phase 1 — Kp search:  binary-search hdg_hold.Kp for max stable value.
    Phase 2 — Ki ramp:    increase Ki until drift < target.
    Phase 3 — Kd damping: if wobble persists, ramp Kd to suppress it.
    Phase 4 — Feedforward: learn direction-dependent integral seed to
              eliminate startup transient (iterative learning control).

    Returns dict of {label: final_value} for each tuned gain.
    """
    ser = poller._ser

    # ── Read current gains ───────────────────────────────────────────
    poller._running = False
    poller._thread.join(timeout=0.5)

    fw = _read_regs_inline(ser)
    if not fw:
        print(f"  {RED}ERROR: cannot read firmware registers{NC}")
        poller._running = True
        poller._thread = threading.Thread(target=poller._loop, daemon=True)
        poller._thread.start()
        return {}

    def _read_f32(addr: int, default: float = 0.0) -> float:
        raw = _read_register_raw(ser, addr)
        if raw and len(raw) >= 4:
            return struct.unpack_from("<f", raw)[0]
        return default

    gains = {
        "vel_pid.Kp":   _read_f32(REG_PID_KP, 1.0),
        "vel_pid.Ki":   _read_f32(REG_PID_KI, 4.0),
        "hdg_hold.Kp":  fw.heading_hold_kp,
        "hdg_hold.Ki":  _read_f32(REG_HEADING_HOLD_KI, 0.0),
        "hdg_hold.Kd":  fw.heading_hold_kd,
    }
    initial_gains = dict(gains)

    poller._running = True
    poller._thread = threading.Thread(target=poller._loop, daemon=True)
    poller._thread.start()

    gain_info = {}
    for addr, label, _, lo, hi in _TUNABLE_GAINS:
        gain_info[label] = dict(addr=addr, lo=lo, hi=hi)

    print(f"\n  {BLD}Starting gains:{NC}")
    for label, val in gains.items():
        print(f"    {label:16s} = {val:.3f}")

    # ── Helper: apply gains, measure fwd + rev ──────────────────────
    def _measure():
        poller._running = False
        poller._thread.join(timeout=0.5)
        for lbl, val in gains.items():
            if lbl in gain_info:
                _write_f32(ser, gain_info[lbl]["addr"], val)
                time.sleep(0.003)
        print(f"    Measuring (fwd) …  ", end="", flush=True)
        fwd = _collect_wobble_window(ser, abs(speed), window_s=4.0,
                                     us_guard=us_guard)
        _send_vel_cmd(ser, 0.0)
        time.sleep(0.5)
        print(f"rev …  ", end="", flush=True)
        rev = _collect_wobble_window(ser, -abs(speed), window_s=4.0,
                                     us_guard=us_guard)
        _send_vel_cmd(ser, 0.0)
        poller._running = True
        poller._thread = threading.Thread(target=poller._loop, daemon=True)
        poller._thread.start()
        return fwd, rev

    def _fmt(fwd, rev):
        wb = fwd["enc_wobble"] or rev["enc_wobble"]
        tgt = abs(speed)
        fve = abs((abs(fwd["mean_vl"]) + abs(fwd["mean_vr"])) / 2 - tgt) / tgt if tgt > 0 else 0
        rve = abs((abs(rev["mean_vl"]) + abs(rev["mean_vr"])) / 2 - tgt) / tgt if tgt > 0 else 0
        return (f"enc_std fwd={math.degrees(fwd['enc_std']):.1f}°/s  "
                f"rev={math.degrees(rev['enc_std']):.1f}°/s  "
                f"wobble={'YES' if wb else 'no'}  "
                f"vel_err fwd={fve*100:.0f}%  rev={rve*100:.0f}%  "
                f"drift fwd={fwd['drift_deg']:+.1f}°  rev={rev['drift_deg']:+.1f}°")

    # ────────────────────────────────────────────────────────────────
    #  PHASE 1: Binary search for highest stable hdg_hold.Kp
    # ────────────────────────────────────────────────────────────────
    # Adaptive wobble detection: instead of a fixed threshold, measure
    # the baseline enc_std at the LOWEST tested Kp.  That baseline is
    # motor/encoder noise — not controller-induced oscillation.  Then
    # flag wobble only when enc_std exceeds baseline by a clear margin.
    #
    # Absolute floor: anything below 0.09 rad/s (5.2°/s) is NEVER wobble
    # regardless of the baseline delta.
    _WOBBLE_ABS_FLOOR = 0.09    # rad/s — below this is never wobble
    _WOBBLE_DELTA     = 0.035   # rad/s — must exceed baseline by this much

    print(f"\n  {CYN}Phase 1: Finding max stable hdg_hold.Kp …{NC}")

    kp_lo = gain_info["hdg_hold.Kp"]["lo"]   # 0.2
    kp_hi = gain_info["hdg_hold.Kp"]["hi"]   # 4.0
    max_phase1 = min(7, max_rounds - 4)       # reserve ≥ 4 rounds for Ki
    rounds_used = 0
    best_stable_kp = kp_lo
    last_stable_drift = float("inf")
    last_stable_std = float("inf")
    std_history = []   # track std per round for plateau detection
    baseline_std = None  # will be set from first stable measurement

    for p1 in range(1, max_phase1 + 1):
        rounds_used += 1
        # First round: test at current (firmware default) Kp
        kp_test = gains["hdg_hold.Kp"] if p1 == 1 else round((kp_lo + kp_hi) / 2, 3)
        gains["hdg_hold.Kp"] = kp_test

        print(f"\n  {CYN}── Phase 1 Round {p1} (Kp={kp_test:.3f}) ──{NC}")
        fwd, rev = _measure()
        drift = max(abs(fwd["drift_deg"]), abs(rev["drift_deg"]))
        cur_std = max(fwd["enc_std"], rev["enc_std"])
        print(_fmt(fwd, rev))

        # Track baseline: minimum enc_std seen across all rounds.
        # This represents the mechanical noise floor.
        if baseline_std is None:
            baseline_std = cur_std
        else:
            baseline_std = min(baseline_std, cur_std)

        # Adaptive wobble criterion:
        #  1) enc_std must exceed the absolute floor (5.2°/s), AND
        #  2) enc_std must exceed baseline + delta (baseline rises above
        #     noise only when the controller actually oscillates).
        #  3) Oscillation freq ≥ 0.3 Hz (rules out unidirectional drift).
        wobble_threshold = max(_WOBBLE_ABS_FLOOR, baseline_std + _WOBBLE_DELTA)
        fwd_tune_wobble = (fwd["enc_std"] > wobble_threshold
                           and fwd["enc_freq_hz"] >= 0.3)
        rev_tune_wobble = (rev["enc_std"] > wobble_threshold
                           and rev["enc_freq_hz"] >= 0.3)
        wobble = fwd_tune_wobble or rev_tune_wobble

        std_history.append(cur_std)

        # Plateau detection: if last 3 rounds' stds are within 20% of each
        # other despite halving Kp, the residual is baseline motor noise —
        # not from the heading-hold controller.  Accept current Kp.
        if len(std_history) >= 3:
            recent = std_history[-3:]
            mean_recent = sum(recent) / len(recent)
            spread = max(recent) - min(recent)
            if spread < 0.20 * mean_recent:
                print(f"    ℹ Std plateaued at ~{math.degrees(mean_recent):.1f}°/s "
                      f"(baseline noise) — accepting Kp={kp_test:.3f}")
                best_stable_kp = kp_test
                last_stable_drift = drift
                last_stable_std = cur_std
                break

        if wobble:
            kp_hi = kp_test
            print(f"    ↓ Kp {kp_test:.3f} wobbles (std={math.degrees(cur_std):.1f}°/s "
                  f"> thresh {math.degrees(wobble_threshold):.1f}°/s) — upper bound → {kp_hi:.3f}")
        else:
            kp_lo = kp_test
            best_stable_kp = kp_test
            last_stable_drift = drift
            last_stable_std = cur_std
            print(f"    ✓ Kp {kp_test:.3f} stable — lower bound → {kp_lo:.3f}")

        if kp_hi - kp_lo < 0.08:
            print(f"    Kp converged: [{kp_lo:.3f}, {kp_hi:.3f}]")
            break
        time.sleep(0.5)

    # If nothing was ever marked stable, use the last tested Kp (best available).
    if last_stable_drift == float("inf"):
        best_stable_kp = gains["hdg_hold.Kp"]
        last_stable_drift = drift
        last_stable_std = cur_std

    # Apply 20% safety margin — marginal stability is unreliable,
    # and reverse conditions are noisier than tuning environment.
    safe_kp = round(best_stable_kp * 0.80, 3)
    safe_kp = max(safe_kp, gain_info["hdg_hold.Kp"]["lo"])
    if safe_kp < best_stable_kp:
        print(f"\n  Kp found: {best_stable_kp:.3f} → applying 20% margin → {safe_kp:.3f}")
    gains["hdg_hold.Kp"] = safe_kp
    print(f"  Kp locked at {gains['hdg_hold.Kp']:.3f}")

    # ────────────────────────────────────────────────────────────────
    #  PHASE 2: Ramp Ki to eliminate steady-state drift
    # ────────────────────────────────────────────────────────────────
    best_gains = dict(gains)
    best_drift = last_stable_drift
    best_std = last_stable_std

    # Always run Phase 2: the short measurement window in Phase 1 can
    # underestimate drift compared to the longer verification pass.
    remaining = max(4, max_rounds - rounds_used)
    print(f"\n  {CYN}Phase 2: Ramping hdg_hold.Ki for drift elimination "
          f"({remaining} rounds avail) …{NC}")

    ki = gains["hdg_hold.Ki"]
    ki_step = 0.20
    for ki_rnd in range(1, remaining + 1):
        ki = round(min(ki + ki_step, gain_info["hdg_hold.Ki"]["hi"]), 3)
        gains["hdg_hold.Ki"] = ki

        print(f"\n  {CYN}── Phase 2 Round {ki_rnd} (Ki={ki:.3f}, step={ki_step:.2f}) ──{NC}")
        fwd, rev = _measure()
        # Use the same adaptive wobble criterion from Phase 1.
        wobble_threshold = max(_WOBBLE_ABS_FLOOR, baseline_std + _WOBBLE_DELTA)
        fwd_tune_wobble = (fwd["enc_std"] > wobble_threshold
                           and fwd["enc_freq_hz"] >= 0.3)
        rev_tune_wobble = (rev["enc_std"] > wobble_threshold
                           and rev["enc_freq_hz"] >= 0.3)
        wobble = fwd_tune_wobble or rev_tune_wobble
        drift = max(abs(fwd["drift_deg"]), abs(rev["drift_deg"]))
        cur_std = max(fwd["enc_std"], rev["enc_std"])
        print(_fmt(fwd, rev))

        if drift < best_drift or (abs(drift - best_drift) < 0.5
                                   and cur_std < best_std):
            best_drift = drift
            best_std = cur_std
            best_gains = dict(gains)

        if drift < _MAX_DRIFT_DEG and not wobble:
            best_gains = dict(gains)
            best_drift = drift
            best_std = cur_std
            print(f"    {GRN}✔ Drift {drift:.1f}° < {_MAX_DRIFT_DEG}° — done!{NC}")
            break

        if wobble:
            # Back off Ki and try a smaller step; abort only if step is tiny.
            ki = round(max(0, ki - ki_step), 3)
            ki_step = round(ki_step * 0.5, 3)
            if ki_step < 0.05:
                gains["hdg_hold.Ki"] = ki
                best_gains["hdg_hold.Ki"] = ki
                print(f"    ↓ Ki wobble — at min step; keeping Ki={ki:.3f}")
                break
            gains["hdg_hold.Ki"] = ki
            print(f"    ↓ Ki caused wobble — backing off to {ki:.3f}, "
                  f"halving step → {ki_step:.3f}")
            continue

        drift_dir = "left" if (fwd["drift_deg"] + rev["drift_deg"]) > 0 else "right"
        print(f"    ⚠ Drift {drift:.1f}° ({drift_dir}) — increasing Ki")

        if ki >= gain_info["hdg_hold.Ki"]["hi"]:
            print(f"    {YLW}Ki at maximum — cannot increase further{NC}")
            break
        time.sleep(0.5)

    # ────────────────────────────────────────────────────────────────
    #  PHASE 3: Kd damping — reduce wobble without sacrificing drift
    # ────────────────────────────────────────────────────────────────
    # Only if wobble persists after Kp/Ki tuning.
    gains = dict(best_gains)
    _phase3_wobble_thresh = max(_WOBBLE_ABS_FLOOR, baseline_std + _WOBBLE_DELTA)
    if best_std > _phase3_wobble_thresh:
        kd_rounds = max(3, max_rounds - rounds_used - 2)  # reserve 2 for Phase 4
        print(f"\n  {CYN}Phase 3: Ramping hdg_hold.Kd for wobble damping "
              f"({kd_rounds} rounds avail) …{NC}")

        kd = gains.get("hdg_hold.Kd", 0.0)
        kd_step = 0.15
        for kd_rnd in range(1, kd_rounds + 1):
            rounds_used += 1
            kd = round(min(kd + kd_step, gain_info["hdg_hold.Kd"]["hi"]), 3)
            gains["hdg_hold.Kd"] = kd

            print(f"\n  {CYN}── Phase 3 Round {kd_rnd} (Kd={kd:.3f}, step={kd_step:.2f}) ──{NC}")
            fwd, rev = _measure()
            drift = max(abs(fwd["drift_deg"]), abs(rev["drift_deg"]))
            cur_std = max(fwd["enc_std"], rev["enc_std"])
            print(_fmt(fwd, rev))

            if cur_std < best_std or (abs(cur_std - best_std) < 0.01
                                       and drift < best_drift):
                best_std = cur_std
                best_drift = drift
                best_gains = dict(gains)
                print(f"    ✓ Kd {kd:.3f} improved: std={math.degrees(cur_std):.1f}°/s")
            else:
                # Kd made things worse or no improvement — back off
                kd = round(max(0, kd - kd_step), 3)
                kd_step = round(kd_step * 0.5, 3)
                gains["hdg_hold.Kd"] = kd
                if kd_step < 0.05:
                    print(f"    ↓ Kd did not help — keeping Kd={kd:.3f}")
                    break
                print(f"    ↓ Kd {kd + kd_step:.3f} worse — trying smaller step {kd_step:.3f}")
                continue

            if cur_std <= _STABLE_STD_RADS:
                print(f"    {GRN}✔ Wobble eliminated (std={math.degrees(cur_std):.1f}°/s){NC}")
                break

            if kd >= gain_info["hdg_hold.Kd"]["hi"]:
                print(f"    {YLW}Kd at maximum — cannot increase further{NC}")
                break
            time.sleep(0.5)
    else:
        print(f"\n  Phase 3: Skipped (wobble already acceptable)")

    # ── Apply best gains ─────────────────────────────────────────────
    print(f"\n  {BLD}Final tuned gains:{NC}")
    poller._running = False
    poller._thread.join(timeout=0.5)
    for lbl, val in best_gains.items():
        info = gain_info.get(lbl)
        if info:
            _write_f32(ser, info["addr"], val)
            time.sleep(0.003)
        chg = abs(val - initial_gains.get(lbl, val)) > 0.001
        marker = " ◄ changed" if chg else ""
        print(f"    {lbl:16s} = {val:.3f}{marker}")

    poller._running = True
    poller._thread = threading.Thread(target=poller._loop, daemon=True)
    poller._thread.start()

    print(f"\n  Best enc_std: {math.degrees(best_std):.1f}°/s  "
          f"({'STABLE' if best_std <= _STABLE_STD_RADS else 'has wobble'})")
    if best_drift < float("inf"):
        drift_status = ('OK' if best_drift < _MAX_DRIFT_DEG
                        else f'still drifting ({best_drift:.1f}°)')
        print(f"  Best drift:   {best_drift:.1f}°  ({drift_status})")
    print(f"  Gains are loaded into RAM — they will reset on reboot.")
    print(f"  To make permanent, update the #define defaults in firmware/main.c.")

    # ────────────────────────────────────────────────────────────────
    #  PHASE 4: Feedforward — correction-derived vel-trim + I-seed
    # ────────────────────────────────────────────────────────────────
    # Two-step approach:
    #   Step A: Drive with heading-hold ON, trims/seeds zeroed, for a
    #           long window (15 s).  Measure the total PID correction
    #           (heading_corr) AND heading-error drift rate.  Derive
    #           the hardware velocity asymmetry from the relationship:
    #
    #             d(heading_err)/dt = −correction − hw_diff/WB
    #
    #           Rearranging:
    #             hw_diff = −(mean_corr + heading_err_slope) × WB
    #             vel_trim = hw_diff
    #
    #           This accounts for both the correction the controller
    #           applies AND any residual drift the controller can't
    #           fully handle (e.g. when heading oscillates due to
    #           deadband gating of the integrator).
    #
    #   Step B: Drive again with trims applied.  Capture residual iterm → I-seed.
    print(f"\n  {CYN}Phase 4: Learning feedforward (correction-derived trim + I-seed) …{NC}")

    def _measure_hw_asymmetry(direction_speed: float, label: str,
                               window: float = 15.0) -> dict:
        """Drive with heading-hold ON, trims zeroed.  Derive hw velocity asymmetry
        from the PID correction + heading error drift rate.
        """
        poller._running = False
        poller._thread.join(timeout=0.5)
        # Load tuned gains; zero all trims/seeds
        for lbl, val in best_gains.items():
            info = gain_info.get(lbl)
            if info:
                _write_f32(ser, info["addr"], val)
                time.sleep(0.003)
        _write_f32(ser, REG_VEL_TRIM_FWD, 0.0)
        _write_f32(ser, REG_VEL_TRIM_REV, 0.0)
        _write_f32(ser, REG_HDG_I_SEED_FWD, 0.0)
        _write_f32(ser, REG_HDG_I_SEED_REV, 0.0)
        time.sleep(0.01)

        # Collect time-stamped samples for the steady-state portion.
        times  = []
        errs   = []
        corrs  = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < window:
            if us_guard and us_guard.abort:
                _send_vel_cmd(ser, 0.0)
                break
            _send_vel_cmd(ser, direction_speed)
            fw = _read_regs_inline(ser)
            if fw:
                elapsed = time.monotonic() - t0
                # Use second half — let transients settle first
                if elapsed > window * 0.5:
                    times.append(elapsed)
                    errs.append(fw.heading_err)
                    corrs.append(fw.heading_corr)
            time.sleep(0.08)
        _send_vel_cmd(ser, 0.0)
        time.sleep(0.3)
        poller._running = True
        poller._thread = threading.Thread(target=poller._loop, daemon=True)
        poller._thread.start()

        if len(times) < 5:
            print(f"    {label}: insufficient samples — skipping")
            return {"vel_trim": 0.0, "hw_diff": 0.0}

        mean_corr = sum(corrs) / len(corrs)
        mean_err  = sum(errs)  / len(errs)

        # Linear regression of heading_err vs time to get drift rate.
        # heading_err_slope = d(heading_err)/dt during the steady-state window.
        n = len(times)
        t_mean = sum(times) / n
        e_mean = sum(errs) / n
        num = sum((t - t_mean) * (e - e_mean) for t, e in zip(times, errs))
        den = sum((t - t_mean) ** 2 for t in times)
        err_slope = num / den if abs(den) > 1e-12 else 0.0   # rad/s

        # Firmware: d(heading_err)/dt = -correction - hw_diff/WB  (with trim=0)
        # Rearranging: hw_diff = -(mean_corr + err_slope) × WB
        hw_diff  = -(mean_corr + err_slope) * WHEEL_BASE
        vel_trim = hw_diff

        print(f"    {label}: corr={math.degrees(mean_corr):+.1f}°/s  "
              f"err={math.degrees(mean_err):+.1f}°  "
              f"drift_rate={math.degrees(err_slope):+.2f}°/s  "
              f"hw_diff={hw_diff*1000:+.1f} mm/s  trim={vel_trim*1000:+.1f} mm/s")
        return {"vel_trim": vel_trim, "hw_diff": hw_diff}

    def _measure_iseed(direction_speed: float, label: str, window: float = 12.0) -> dict:
        """Drive with heading-hold ON + trims applied, capture residual iterm → I-seed."""
        poller._running = False
        poller._thread.join(timeout=0.5)
        # Ensure tuned gains are loaded
        for lbl, val in best_gains.items():
            info = gain_info.get(lbl)
            if info:
                _write_f32(ser, info["addr"], val)
                time.sleep(0.003)

        iterms = []
        errs   = []
        corrs  = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < window:
            if us_guard and us_guard.abort:
                _send_vel_cmd(ser, 0.0)
                break
            _send_vel_cmd(ser, direction_speed)
            fw = _read_regs_inline(ser)
            if fw:
                elapsed = time.monotonic() - t0
                # Use last 25% of window — integrator most converged
                if elapsed > window * 0.75:
                    iterms.append(fw.heading_hold_iterm)
                    errs.append(fw.heading_err)
                    corrs.append(fw.heading_corr)
            time.sleep(0.08)
        _send_vel_cmd(ser, 0.0)
        time.sleep(0.3)
        poller._running = True
        poller._thread = threading.Thread(target=poller._loop, daemon=True)
        poller._thread.start()

        if len(iterms) < 3:
            print(f"    {label}: insufficient samples — skipping")
            return {"iterm": 0.0}

        # Use last few samples (most converged)
        n_tail = min(5, len(iterms))
        iterm   = sum(iterms[-n_tail:]) / n_tail
        mean_err  = sum(errs) / len(errs)
        mean_corr = sum(corrs) / len(corrs)

        print(f"    {label}: iterm={iterm:+.4f}  err={math.degrees(mean_err):+.1f}°  "
              f"corr={math.degrees(mean_corr):+.1f}°/s")
        return {"iterm": iterm}

    # ── Step A: Correction-derived velocity trim ──
    print("    Step A — measuring hw asymmetry (heading-hold ON, trims zeroed, 15 s):")
    fwd_hw = _measure_hw_asymmetry(+abs(speed), "Forward")
    time.sleep(0.5)
    rev_hw = _measure_hw_asymmetry(-abs(speed), "Reverse")
    fwd_vtrim = fwd_hw["vel_trim"]
    rev_vtrim = rev_hw["vel_trim"]

    # Write trims immediately so Step B runs with them
    def _write_ff_values(fs, rs, fv, rv):
        poller._running = False
        poller._thread.join(timeout=0.5)
        _write_f32(ser, REG_HDG_I_SEED_FWD, fs)
        time.sleep(0.003)
        _write_f32(ser, REG_HDG_I_SEED_REV, rs)
        time.sleep(0.003)
        _write_f32(ser, REG_VEL_TRIM_FWD, fv)
        time.sleep(0.003)
        _write_f32(ser, REG_VEL_TRIM_REV, rv)
        time.sleep(0.003)
        poller._running = True
        poller._thread = threading.Thread(target=poller._loop, daemon=True)
        poller._thread.start()

    _write_ff_values(0.0, 0.0, fwd_vtrim, rev_vtrim)
    time.sleep(0.5)

    # ── Step B: I-seed learning (heading-hold ON, trims applied) ──
    print("    Step B — I-seed learning (heading-hold ON, trims active, 12 s):")
    fwd_is = _measure_iseed(+abs(speed), "Forward")
    time.sleep(0.5)
    rev_is = _measure_iseed(-abs(speed), "Reverse")
    fwd_seed = fwd_is["iterm"]
    rev_seed = rev_is["iterm"]

    _write_ff_values(fwd_seed, rev_seed, fwd_vtrim, rev_vtrim)

    print(f"\n  Feedforward parameters:")
    print(f"    Velocity trims:  fwd={fwd_vtrim*1000:+.1f}  rev={rev_vtrim*1000:+.1f} mm/s")
    print(f"    Integral seeds:  fwd={fwd_seed:+.4f}  rev={rev_seed:+.4f} rad/s")
    if abs(rev_vtrim) > 0.001:
        pct = abs(rev_vtrim) / abs(speed) * 100
        print(f"    Reverse motor asymmetry: {pct:.1f}% — velocity trim compensates this directly")
    print(f"  {YLW}Note: requires firmware with velocity trim + I-seed registers.{NC}\n")

    best_gains["fwd_i_seed"] = fwd_seed
    best_gains["rev_i_seed"] = rev_seed
    best_gains["fwd_vel_trim"] = fwd_vtrim
    best_gains["rev_vel_trim"] = rev_vtrim

    return best_gains


def _read_register_raw(ser, addr: int, length: int = 4) -> Optional[bytes]:
    """Read raw bytes from a single register (inline, bus must be idle)."""
    ser.reset_input_buffer()
    ser.write(make_read_pkt(addr, length))
    return read_response(ser, length, timeout=0.15)


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
    ap.add_argument("--duration",     type=float, default=5.0,
                    help="seconds per pass (default 5.0)")
    ap.add_argument("--pairs",        type=int,   default=1,
                    help="number of fwd+rev pairs (default 1)")
    ap.add_argument("--no-csv",       action="store_true",
                    help="do not write CSV file")
    ap.add_argument("--port",         default=None,
                    help="serial port override")
    ap.add_argument("--stop-bringup", action="store_true",
                    help="stop turtlebot3-bringup before test (pure-serial mode; no ROS2 needed)")
    ap.add_argument("--auto-tune",    action="store_true",
                    help="auto-tune PID gains while driving until wobble is eliminated "
                         "(implies --stop-bringup)")
    ap.add_argument("--no-ultrasonic", action="store_true",
                    help="disable ultrasonic collision detection during drive passes")
    ap.add_argument("--collision-mm", type=float, default=US_COLLISION_MM,
                    help=f"ultrasonic collision threshold in mm (default {US_COLLISION_MM:.0f})")
    args = ap.parse_args()

    if args.auto_tune:
        args.stop_bringup = True

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
                _probe_pkt = make_read_pkt(BULK_ADDR, BULK_LEN)
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

        # ── Ultrasonic collision guard (GPIO-direct, serial mode) ──────
        us_guard: Optional[UltrasonicGuard] = None
        if not args.no_ultrasonic:
            try:
                us_guard = UltrasonicGuard(collision_mm=args.collision_mm)
                print(f"  Ultrasonic guard: {GRN}active{NC}  threshold={args.collision_mm:.0f} mm")
            except Exception as _ue:
                print(f"  {YLW}Ultrasonic guard unavailable: {_ue}{NC}")

        all_samples:   list[Sample] = []
        all_summaries: list[dict]   = []
        t0 = time.monotonic()

        # ── Auto-tune phase (if requested) ─────────────────────────────
        if args.auto_tune:
            print(f"\n{BLD}══════════════════════════════════════════════════════{NC}")
            print(f"{BLD} AUTO-TUNE: iteratively adjusting PID gains{NC}")
            print(f"{BLD}══════════════════════════════════════════════════════{NC}")
            try:
                tuned = run_auto_tune(poller, args.speed, max_rounds=12,
                                      us_guard=us_guard)
            except KeyboardInterrupt:
                print(f"\n{YLW}Auto-tune interrupted — stopping.{NC}")
                poller.write_velocity(0.0)
                tuned = {}
            if tuned:
                print(f"  Running verification pass with tuned gains …\n")
            else:
                print(f"  {YLW}Auto-tune did not converge — running diagnostic with current gains{NC}\n")

        try:
            for pair in range(1, args.pairs + 1):
                print(f"\n{CYN}──── Pair {pair}/{args.pairs} ────{NC}")

                print(f"\n  {BLD}[ FORWARD ]{NC}")
                fwd_samples, fwd_sum = run_serial_drive_pass(
                    poller, +args.speed, args.duration, "fwd", pair, t0,
                    us_guard=us_guard)
                all_samples.extend(fwd_samples)
                all_summaries.append(fwd_sum)

                print(f"\n  Settle 2 s …")
                time.sleep(2.0)

                print(f"\n  {BLD}[ REVERSE ]{NC}")
                rev_samples, rev_sum = run_serial_drive_pass(
                    poller, -args.speed, args.duration, "rev", pair, t0,
                    us_guard=us_guard)
                all_samples.extend(rev_samples)
                all_summaries.append(rev_sum)

                if pair < args.pairs:
                    print(f"\n  {YLW}Settle 3 s before next pair …{NC}")
                    time.sleep(3.0)

        except KeyboardInterrupt:
            print(f"\n{YLW}Interrupted — stopping.{NC}")
            poller.write_velocity(0.0)
        finally:
            if us_guard:
                us_guard.close()
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
        node = DiagReverseNode(collision_mm=args.collision_mm)

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

    # In serial mode we also have gyro-Z integrated drift (for reference)
    have_gyro_drift = any("gyro_hdg_drift_deg" in s for s in all_summaries)

    rows = [
        ("Distance travelled (cm)",   avg(fwd_sums,"dist_m")*100,    avg(rev_sums,"dist_m")*100),
        ("Heading drift (enc) (°)",   avg(fwd_sums,"hdg_drift_deg"), avg(rev_sums,"hdg_drift_deg")),
    ]
    if have_gyro_drift:
        rows.append(
            ("Gyro-Z drift (raw) (°)",avg(fwd_sums,"gyro_hdg_drift_deg",0.0),
                                      avg(rev_sums,"gyro_hdg_drift_deg",0.0)),
        )
    else:
        rows.append(
            ("Heading drift IMU  (°)",avg(fwd_sums,"imu_drift_deg"), avg(rev_sums,"imu_drift_deg")),
        )
    rows += [
        ("Mean gyro-Z (°/s)",         math.degrees(avg(fwd_sums,"mean_gyro_rads")),
                                      math.degrees(avg(rev_sums,"mean_gyro_rads"))),
        ("|Vel L| (mm/s)",            abs(avg(fwd_sums,"fw_mean_vl_ms"))*1000,
                                      abs(avg(rev_sums,"fw_mean_vl_ms"))*1000),
        ("|Vel R| (mm/s)",            abs(avg(fwd_sums,"fw_mean_vr_ms"))*1000,
                                      abs(avg(rev_sums,"fw_mean_vr_ms"))*1000),
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
        ("Enc ω std (°/s)",           math.degrees(avg(fwd_sums,"enc_wob_std_rads",0.0)),
                                      math.degrees(avg(rev_sums,"enc_wob_std_rads",0.0))),
        ("Enc ω peak (°/s)",          math.degrees(avg(fwd_sums,"enc_wob_peak_rads",0.0)),
                                      math.degrees(avg(rev_sums,"enc_wob_peak_rads",0.0))),
        ("Enc ω freq (Hz)",            avg(fwd_sums,"enc_wob_freq_hz",0.0),
                                       avg(rev_sums,"enc_wob_freq_hz",0.0)),
    ]

    # Motor startup asymmetry rows (serial mode only)
    have_startup = any("startup_lag_ms" in s for s in all_summaries)
    if have_startup:
        rows += [
            ("Startup lag (ms)",           avg(fwd_sums,"startup_lag_ms",0.0),
                                           avg(rev_sums,"startup_lag_ms",0.0)),
            ("Startup drift (°)",          avg(fwd_sums,"startup_drift_deg",0.0),
                                           avg(rev_sums,"startup_drift_deg",0.0)),
            ("Startup peak hdg err (°)",   avg(fwd_sums,"startup_peak_hdg_err_deg",0.0),
                                           avg(rev_sums,"startup_peak_hdg_err_deg",0.0)),
            ("Ramp L to 50% (ms)",         avg(fwd_sums,"startup_ramp_l_ms",0.0),
                                           avg(rev_sums,"startup_ramp_l_ms",0.0)),
            ("Ramp R to 50% (ms)",         avg(fwd_sums,"startup_ramp_r_ms",0.0),
                                           avg(rev_sums,"startup_ramp_r_ms",0.0)),
        ]

    col_w = 28
    print(f"  {'Metric':<{col_w}} {'Forward':>10}  {'Reverse':>10}  {'Δ (rev−fwd)':>13}")
    print(f"  {'─'*col_w} {'─'*10}  {'─'*10}  {'─'*13}")
    for label, fv, rv in rows:
        delta = rv - fv
        dfmt  = f"{delta:+10.3f}"
        flag  = ""
        if "drift" in label.lower() and "°" in label and "raw" not in label.lower() and abs(delta) > 3.0:
            flag = f"  {YLW}⚠ large asymmetry{NC}"
        elif "ratio" in label.lower() and abs(delta) > 0.05:
            flag = f"  {YLW}⚠ asymmetric{NC}"
        print(f"  {label:<{col_w}} {fv:>10.3f}  {rv:>10.3f}  {dfmt}{flag}")

    # Diagnostic interpretation
    print(f"\n{BLD}  Interpretation:{NC}")
    # Use encoder-based heading drift (reliable even with wobble)
    fwd_hdg = avg(fwd_sums, "hdg_drift_deg")   # encoder-based in serial mode, odom in ROS mode
    rev_hdg = avg(rev_sums, "hdg_drift_deg")
    issues = []   # collect issue strings for final verdict

    if abs(rev_hdg) > abs(fwd_hdg) + 2.0:
        dir_s = "left (CCW)" if rev_hdg > 0 else "right (CW)"
        print(f"  {YLW}⚠  Reverse drifts {dir_s} by {rev_hdg:+.1f}° vs fwd {fwd_hdg:+.1f}°{NC}")
        issues.append("heading asymmetry")

    # ── Motor startup asymmetry analysis ──────────────────────────────────
    if have_startup:
        fwd_lag = avg(fwd_sums, "startup_lag_ms", 0.0)
        rev_lag = avg(rev_sums, "startup_lag_ms", 0.0)
        fwd_sdrift = avg(fwd_sums, "startup_drift_deg", 0.0)
        rev_sdrift = avg(rev_sums, "startup_drift_deg", 0.0)
        worst_lag = max(abs(fwd_lag), abs(rev_lag))
        worst_sdrift = max(abs(fwd_sdrift), abs(rev_sdrift))

        if worst_lag > 50:
            faster_fwd = "left" if fwd_lag > 0 else "right"
            faster_rev = "left" if rev_lag > 0 else "right"
            print(f"  {YLW}⚠  Motor startup asymmetry: {worst_lag:.0f} ms lag "
                  f"(fwd: {faster_fwd} leads by {abs(fwd_lag):.0f} ms, "
                  f"rev: {faster_rev} leads by {abs(rev_lag):.0f} ms){NC}")
            print(f"     One motor reaches static-friction breakaway before the other.")
            print(f"     Suggestions:")
            print(f"       1. Calibrate per-motor kick-start duty (currently both 0.20)")
            print(f"       2. Calibrate per-motor min-duty (currently both 0.15)")
            print(f"       3. Check mechanical: wheel stiffness, gearbox, tire grip")
            issues.append("startup lag")
        elif worst_lag > 20:
            print(f"  {YLW}ℹ  Minor startup lag ({worst_lag:.0f} ms) — motors start within ~{worst_lag:.0f} ms of each other{NC}")

        if worst_sdrift > 2.0:
            print(f"  {YLW}⚠  Startup causes {worst_sdrift:.1f}° drift before heading-hold engages "
                  f"(fwd {fwd_sdrift:+.1f}°, rev {rev_sdrift:+.1f}°){NC}")
            print(f"     This initial offset persists through the drive pass.")
            issues.append("startup drift")
        elif worst_sdrift > 0.5:
            print(f"  {GRN}ℹ  Startup drift is small ({worst_sdrift:.1f}°) — heading-hold compensates{NC}")

    # Retrieve actual KP and KD from firmware (if available from any summary)
    actual_kp = None
    actual_kd = None
    for s_dict in all_summaries:
        kp = s_dict.get("heading_hold_kp")
        if kp is not None and kp > 0:
            actual_kp = kp
        kd = s_dict.get("heading_hold_kd")
        if kd is not None:
            actual_kd = kd
        if actual_kp is not None:
            break
    kp_str = f"{actual_kp:.1f}" if actual_kp else "~2.0"
    kd_str = f"{actual_kd:.1f}" if actual_kd is not None else "?"

    if args.stop_bringup:
        rev_hd = avg(rev_sums, "fw_mean_hdg_err_deg")
        rev_hc = math.degrees(avg(rev_sums, "fw_mean_hdg_corr_rads"))
        rev_ed = avg(rev_sums, "fw_mean_enc_diff_m") * 1000
        fwd_hc = math.degrees(avg(fwd_sums, "fw_mean_hdg_corr_rads"))
        fwd_ed = avg(fwd_sums, "fw_mean_enc_diff_m") * 1000

        if abs(rev_hd) > 2.0 and abs(rev_hc) < 0.5:
            print(f"  {RED}✗  Heading error {rev_hd:+.2f}° in reverse but correction only "
                  f"{rev_hc:+.2f}°/s — heading-hold not correcting!{NC}")
            issues.append("heading-hold inactive in reverse")
        elif abs(rev_hd) > 2.0:
            print(f"  {YLW}⚠  Heading error {rev_hd:+.2f}° with correction {rev_hc:+.2f}°/s "
                  f"— correction may be insufficient.{NC}")
            issues.append("heading correction insufficient")

        sign_flip = (fwd_hc * rev_hc < 0) and abs(fwd_hc) > 0.1 and abs(rev_hc) > 0.1
        if sign_flip:
            if abs(rev_hdg) > 5.0 or abs(fwd_hdg) > 5.0:
                print(f"  {YLW}ℹ  Correction sign differs fwd ({fwd_hc:+.2f}°/s) vs "
                      f"rev ({rev_hc:+.2f}°/s) — different motor dominates each direction.{NC}")

        if abs(rev_ed) > 5.0:
            wheel = "left" if rev_ed > 0 else "right"
            print(f"  {YLW}⚠  Enc L−R diff = {rev_ed:+.2f} mm in reverse "
                  f"({wheel} wheel faster backward){NC}")
            issues.append("encoder imbalance (reverse)")
        if abs(fwd_ed) > 5.0:
            wheel = "left" if fwd_ed > 0 else "right"
            print(f"  {YLW}⚠  Enc L−R diff = {fwd_ed:+.2f} mm forward "
                  f"({wheel} wheel faster forward){NC}")
            issues.append("encoder imbalance (forward)")

        # Note if gyro-Z drift is very different from encoder drift (indicates gyro bias)
        if have_gyro_drift:
            fwd_gyro_d = avg(fwd_sums, "gyro_hdg_drift_deg", 0.0)
            rev_gyro_d = avg(rev_sums, "gyro_hdg_drift_deg", 0.0)
            fwd_enc_d  = avg(fwd_sums, "hdg_drift_deg")
            rev_enc_d  = avg(rev_sums, "hdg_drift_deg")
            gyro_enc_gap = max(abs(fwd_gyro_d - fwd_enc_d), abs(rev_gyro_d - rev_enc_d))
            if gyro_enc_gap > 3.0:
                print(f"  {YLW}ℹ  Gyro-Z integrated drift differs from encoder drift "
                      f"by up to {gyro_enc_gap:.1f}° — oscillation bias in raw gyro{NC}")

    # Speed symmetry check (both modes)
    fwd_speed_mag = avg(fwd_sums, "dist_m") / args.duration if args.duration > 0 else 0
    rev_speed_mag = avg(rev_sums, "dist_m") / args.duration if args.duration > 0 else 0
    if fwd_speed_mag > 0:
        speed_asym = abs(rev_speed_mag - fwd_speed_mag) / fwd_speed_mag * 100
        if speed_asym > 5.0:
            print(f"  {YLW}⚠  Speed asymmetry: fwd {fwd_speed_mag*1000:.1f} mm/s vs "
                  f"rev {rev_speed_mag*1000:.1f} mm/s ({speed_asym:.1f}% difference){NC}")
            issues.append("speed asymmetry")

    # Wobble / oscillation check (both modes)
    # Encoder-based wobble is ground truth (immune to BNO055 vibration noise).
    # Gyro wobble without encoder wobble is just vibration — informational only.
    have_enc_wobble_data = any("enc_wobble" in s for s in all_summaries)
    if have_enc_wobble_data:
        fwd_wobble = avg(fwd_sums, "enc_wobble", 0.0) >= 0.5
        rev_wobble = avg(rev_sums, "enc_wobble", 0.0) >= 0.5
        fwd_std_deg = math.degrees(avg(fwd_sums, "enc_wob_std_rads", 0.0))
        rev_std_deg = math.degrees(avg(rev_sums, "enc_wob_std_rads", 0.0))
        fwd_freq    = avg(fwd_sums, "enc_wob_freq_hz", 0.0)
        rev_freq    = avg(rev_sums, "enc_wob_freq_hz", 0.0)
    else:
        # Fallback — gyro only (ROS mode without serial)
        fwd_wobble = avg(fwd_sums, "gyro_z_wobble", 0.0) >= 0.5
        rev_wobble = avg(rev_sums, "gyro_z_wobble", 0.0) >= 0.5
        fwd_std_deg = math.degrees(avg(fwd_sums, "gyro_z_std_rads", 0.0))
        rev_std_deg = math.degrees(avg(rev_sums, "gyro_z_std_rads", 0.0))
        fwd_freq    = avg(fwd_sums, "gyro_z_freq_hz", 0.0)
        rev_freq    = avg(rev_sums, "gyro_z_freq_hz", 0.0)

    # Gyro-Z stats for informational display
    gyro_fwd_std = math.degrees(avg(fwd_sums, "gyro_z_std_rads", 0.0))
    gyro_rev_std = math.degrees(avg(rev_sums, "gyro_z_std_rads", 0.0))
    gyro_fwd_wobble = avg(fwd_sums, "gyro_z_wobble", 0.0) >= 0.5
    gyro_rev_wobble = avg(rev_sums, "gyro_z_wobble", 0.0) >= 0.5

    if fwd_wobble or rev_wobble:
        src = "encoder" if have_enc_wobble_data else "gyro"
        phases = []
        if fwd_wobble:
            phases.append(f"forward ({fwd_std_deg:.1f}°/s std, {fwd_freq:.2f} Hz)")
        if rev_wobble:
            phases.append(f"reverse ({rev_std_deg:.1f}°/s std, {rev_freq:.2f} Hz)")
        print(f"  {YLW}⚠  Oscillation (wobble) detected ({src}): {', '.join(phases)}{NC}")
        print(f"     The heading-hold controller is hunting (KP={kp_str}, KD={kd_str}). Suggestions:")
        kp_low  = max(actual_kp * 0.5, 0.5) if actual_kp else 1.0
        kp_high = max(actual_kp * 0.75, 1.0) if actual_kp else 1.5
        print(f"       1. Reduce HEADING_HOLD_KP (currently {kp_str}) → try {kp_low:.1f}–{kp_high:.1f}")
        if actual_kd is not None and actual_kd < 0.01:
            print(f"       2. Add derivative damping: HEADING_HOLD_KD → try 0.2–0.5")
        elif actual_kd is not None:
            kd_high = actual_kd * 1.5
            print(f"       2. Increase HEADING_HOLD_KD (currently {kd_str}) → try {kd_high:.1f}")
        print(f"       3. If only at high speed, also reduce HEADING_HOLD_MAX_CORR")
        print(f"       4. Slightly increase HEADING_HOLD_DEADBAND")
        issues.append("wobble")
    else:
        # No encoder wobble — check gyro for vibration info
        max_enc_std = max(fwd_std_deg, rev_std_deg) if have_enc_wobble_data else 0.0
        max_gyro_std = max(gyro_fwd_std, gyro_rev_std)
        if gyro_fwd_wobble or gyro_rev_wobble:
            print(f"  {GRN}✔  No real wobble (encoder ω std {max_enc_std:.1f}°/s — stable){NC}")
            print(f"  {YLW}ℹ  Gyro-Z shows {max_gyro_std:.1f}°/s std — this is BNO055 vibration noise, not heading oscillation{NC}")
        elif max_gyro_std < 2.0:
            print(f"  {GRN}✔  No wobble detected (gyro-Z std {max_gyro_std:.1f}°/s, enc ω std {max_enc_std:.1f}°/s — stable){NC}")
        else:
            print(f"  {YLW}⚠  Gyro-Z std {max_gyro_std:.1f}°/s — borderline; monitor at higher speeds{NC}")

    # ── Overall verdict ────────────────────────────────────────────────────
    dist_delta_pct = 0.0
    fwd_dist = avg(fwd_sums, "dist_m")
    rev_dist = avg(rev_sums, "dist_m")
    if fwd_dist > 0:
        dist_delta_pct = abs(rev_dist - fwd_dist) / fwd_dist * 100

    # Heading drift uses encoder-based values (reliable)
    max_hdg_drift = max(abs(fwd_hdg), abs(rev_hdg))

    # FAIL: large heading drift (encoder-based) — indicates real steering problem
    # WARN: wobble (tunable), moderate heading drift, speed asymmetry
    critical_fail = max_hdg_drift > 5.0
    if critical_fail:
        issues.append("large heading drift")
    elif max_hdg_drift > _MAX_DRIFT_DEG:
        print(f"  {YLW}⚠  Heading drift {max_hdg_drift:.1f}° exceeds {_MAX_DRIFT_DEG:.0f}° target "
              f"(fwd {fwd_hdg:+.1f}°, rev {rev_hdg:+.1f}°) — consider --auto-tune{NC}")
        issues.append("heading drift")
    has_wobble = fwd_wobble or rev_wobble
    warn_count = len(issues)

    print()
    if critical_fail:
        print(f"  {RED}{BLD}RESULT: FAIL{NC}  — {', '.join(issues)}")
    elif warn_count > 0:
        print(f"  {YLW}{BLD}RESULT: WARN{NC}  — {', '.join(issues)}")
    else:
        print(f"  {GRN}{BLD}RESULT: PASS{NC}  — forward/reverse symmetry OK "
              f"(dist Δ={dist_delta_pct:.1f}%, heading drift <{max_hdg_drift:.1f}°)")
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
