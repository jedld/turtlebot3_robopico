#!/usr/bin/env python3
"""
calibrate_drift.py — Detect and fix left/right drift using ultrasonic sensors.

Drives the robot straight toward a flat wall and measures left/right ultrasonic
distance change to detect angular drift.  Computes the velocity-level feedforward
trim (VEL_TRIM_FWD) needed to cancel it, and writes the #define default to
firmware/main.c so the correction persists after reflashing.

The trim is applied as:
    v_left  += vel_trim / 2
    v_right -= vel_trim / 2
Positive vel_trim corrects left drift (speeds up left, slows right).

Workflow
--------
1.  Align robot to wall using LiDAR + ultrasonic fine-align (same as calibrate_linear.py)
2.  Reverse to create space (>= 100 cm clearance)
3.  Sample L/R ultrasonic before driving
4.  Drive forward a fixed distance
5.  Sample L/R ultrasonic after driving
6.  Compute angular drift:  θ = (ΔR − ΔL) / sensor_sep
7.  Compute vel_trim = θ × speed × wheel_sep / distance
8.  Repeat for --passes and average
9.  Write VEL_TRIM_FWD_DEFAULT to firmware/main.c

Additionally, this script also writes VEL_TRIM_FWD to the firmware register
at runtime via Dynamixel, so the correction takes effect immediately.

Drift measurement modes
-----------------------
- `uss` (default): uses left/right ultrasonic differential to estimate heading drift
- `imu`: uses IMU yaw change to estimate heading drift and encoder distance for travel

Usage
-----
  python3 calibrate_drift.py                   # detect drift (dry-run)
  python3 calibrate_drift.py --apply           # detect + write firmware/main.c
  python3 calibrate_drift.py --apply --flash   # detect + write + rebuild + flash
  python3 calibrate_drift.py --passes 5        # more passes for better accuracy
  python3 calibrate_drift.py --verify-only     # one pass, just report
    python3 calibrate_drift.py --drift-source imu  # use IMU yaw + encoders, no USS drift measurement

By default the script attempts to start required system services before waiting
for topics:
- `turtlebot3-bringup.service`
- `pi-ultrasonic-dual.service` (USS mode only)
"""

import argparse
import math
import os
import re
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, JointState, LaserScan, Range

# ── paths ─────────────────────────────────────────────────────────────────────

SCRIPT_DIR   = Path(__file__).resolve().parent
MAIN_C       = SCRIPT_DIR / "firmware" / "main.c"
FIRMWARE_DIR = SCRIPT_DIR / "firmware"
BUILD_SCRIPT = FIRMWARE_DIR / "build.sh"

# ── tunable constants ─────────────────────────────────────────────────────────

# LiDAR coarse alignment
ALIGN_SECTOR_DEG  = 60.0
ALIGN_TOL_DEG     = 8.0
ALIGN_SPEED       = 0.35    # rad/s
ALIGN_MAX_ITER    = 5

# Ultrasonic fine alignment
USS_FINE_TOL_M    = 0.010   # accept if |dL − dR| < 1 cm
USS_SENSOR_SEP_M  = 0.15    # lateral distance between the two ultrasonic transducers
USS_FINE_SPEED    = 0.20    # rad/s
USS_FINE_MAX_ITER = 8

# Space / distance
MIN_SPACE_M       = 1.00    # minimum forward clearance to start a pass
WALL_STOP_M       = 0.22    # emergency stop if USS < this
WALL_CLEARANCE_M  = 0.25    # clearance to leave between robot and wall
MIN_DRIVE_M       = 0.30    # minimum per-pass test distance
MAX_DRIVE_M       = 0.60    # maximum per-pass test distance
MIN_BACKUP_M      = 0.30
DEFAULT_DRIVE_M   = 0.50    # default test distance

# Ultrasonic sampling
USS_SAMPLES       = 10
USS_SETTLE_S      = 0.10
USS_MAX_RANGE_M   = 3.0

# Encoder
WHEEL_RADIUS_DEFAULT_M = 0.034050

# Dynamixel protocol for register I/O
DXL_PORT    = "/dev/ttyACM0"
DEV_ID      = 200
INST_WRITE  = 0x03
INST_READ   = 0x02

# Register addresses in firmware
ADDR_VEL_TRIM_FWD       = 268
ADDR_VEL_TRIM_REV       = 272
ADDR_DBG_HEADING_ERR    = 284
ADDR_HEADING_HOLD_CORR  = 296
ADDR_HEADING_HOLD_EN    = 288

# Backup / reverse
BACKUP_SPEED_MARGIN   = 0.03
BACKUP_PROBE_S        = 0.50
BACKUP_MOVE_ENC       = 0.003
BACKUP_MAX_M          = 1.50
BACKUP_POLL_S         = 0.05
MOTOR_MIN_DUTY_DEFAULT = 0.15

# QoS compatible with TurtleBot3 bringup
BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Colours
RED = "\033[0;31m"
GRN = "\033[0;32m"
YLW = "\033[1;33m"
CYN = "\033[0;36m"
BLD = "\033[1m"
NC  = "\033[0m"

FORWARD_WINDOW_DEG = 8.0
SERVICE_START_SETTLE_S = 6.0


# ── firmware source helpers ───────────────────────────────────────────────────

def read_define_float(path: Path, name: str) -> Optional[float]:
    pat = re.compile(rf"^\s*#define\s+{re.escape(name)}\s+([\d.+\-eEfF]+)")
    for line in path.read_text().splitlines():
        m = pat.match(line)
        if m:
            return float(m.group(1).rstrip("fF"))
    return None


def write_define_float(path: Path, name: str, value: float,
                       comment: str = "") -> None:
    content = path.read_text()
    suffix  = f"  // {comment}" if comment else ""
    pat     = re.compile(
        rf"(#define\s+{re.escape(name)}\s+)[\d.+\-eEfF]+f?([ \t]*(?://[^\n]*)?)"
    )
    new_content, count = pat.subn(
        lambda m: f"{m.group(1)}{value:.6f}f{suffix}",
        content, count=1,
    )
    if count == 0:
        raise RuntimeError(f"Could not find #define {name} in {path}")
    path.write_text(new_content)


# ── Dynamixel 2.0 serial helpers ─────────────────────────────────────────────

_CRC_TABLE = []
for _i in range(256):
    _crc = _i << 8
    for _ in range(8):
        if _crc & 0x8000:
            _crc = ((_crc << 1) ^ 0x8005) & 0xFFFF
        else:
            _crc = (_crc << 1) & 0xFFFF
    _CRC_TABLE.append(_crc)


def _dxl_crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        idx = ((crc >> 8) ^ b) & 0xFF
        crc = ((crc << 8) ^ _CRC_TABLE[idx]) & 0xFFFF
    return crc


def _build_packet(dev_id: int, instruction: int, params: bytes = b'') -> bytes:
    length = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, dev_id,
                 length & 0xFF, (length >> 8) & 0xFF, instruction])
    pkt = hdr + params
    crc = _dxl_crc16(pkt)
    return pkt + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _read_response(ser, timeout: float = 0.5) -> Optional[bytes]:
    import serial as _serial  # noqa
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


def dxl_write_register(addr: int, data: bytes, port: str = DXL_PORT) -> bool:
    """Write data to a firmware register via Dynamixel WRITE instruction."""
    import serial
    try:
        ser = serial.Serial(port, timeout=0.1)
        time.sleep(0.05)
        ser.reset_input_buffer()
        params = struct.pack('<H', addr) + data
        pkt = _build_packet(DEV_ID, INST_WRITE, params)
        ser.write(pkt)
        resp = _read_response(ser)
        ser.close()
        return resp is not None
    except Exception as e:
        print(f"  {RED}DXL write error: {e}{NC}")
        return False


def dxl_read_register(addr: int, length: int, port: str = DXL_PORT) -> Optional[bytes]:
    """Read data from a firmware register via Dynamixel READ instruction."""
    import serial
    try:
        ser = serial.Serial(port, timeout=0.1)
        time.sleep(0.05)
        ser.reset_input_buffer()
        params = struct.pack('<HH', addr, length)
        pkt = _build_packet(DEV_ID, INST_READ, params)
        ser.write(pkt)
        resp = _read_response(ser)
        ser.close()
        if resp and len(resp) >= 11:
            return resp[9:-2]
        return None
    except Exception as e:
        print(f"  {RED}DXL read error: {e}{NC}")
        return None


def dxl_write_float(addr: int, value: float, port: str = DXL_PORT) -> bool:
    return dxl_write_register(addr, struct.pack('<f', value), port)


def dxl_read_float(addr: int, port: str = DXL_PORT) -> Optional[float]:
    data = dxl_read_register(addr, 4, port)
    if data and len(data) >= 4:
        return struct.unpack('<f', data[:4])[0]
    return None


def ensure_required_services(drift_source: str,
                             auto_start: bool = True) -> bool:
    services = ["turtlebot3-bringup.service"]
    if drift_source == "uss":
        services.append("pi-ultrasonic-dual.service")

    inactive: list[str] = []
    for service in services:
        result = subprocess.run(
            ["systemctl", "is-active", "--quiet", service],
            capture_output=True,
        )
        if result.returncode != 0:
            inactive.append(service)

    if not inactive:
        print(f"  {GRN}Required services already active.{NC}")
        return True

    print(f"  {YLW}Inactive services detected:{NC} {', '.join(inactive)}")
    if not auto_start:
        print(f"  {YLW}Auto-start disabled. Start required services and retry.{NC}")
        return False

    print(f"  Starting required services …")
    result = subprocess.run(["sudo", "systemctl", "start", *inactive])
    if result.returncode != 0:
        print(f"  {RED}Failed to start required services.{NC}")
        return False

    print(f"  Waiting {SERVICE_START_SETTLE_S:.0f}s for services to settle …")
    time.sleep(SERVICE_START_SETTLE_S)

    still_inactive: list[str] = []
    for service in inactive:
        result = subprocess.run(
            ["systemctl", "is-active", "--quiet", service],
            capture_output=True,
        )
        if result.returncode != 0:
            still_inactive.append(service)

    if still_inactive:
        print(f"  {RED}Services failed to become active:{NC} {', '.join(still_inactive)}")
        return False

    print(f"  {GRN}Required services started.{NC}")
    return True


def restart_required_services(drift_source: str) -> bool:
    services = ["turtlebot3-bringup.service"]
    if drift_source == "uss":
        services.append("pi-ultrasonic-dual.service")

    print(f"  Restarting required services …")
    result = subprocess.run(["sudo", "systemctl", "restart", *services])
    if result.returncode != 0:
        print(f"  {RED}Failed to restart required services.{NC}")
        return False

    print(f"  Waiting {SERVICE_START_SETTLE_S:.0f}s for services to settle …")
    time.sleep(SERVICE_START_SETTLE_S)
    return True


# ── LiDAR wall bearing ───────────────────────────────────────────────────────

def find_wall_bearing(scan: LaserScan,
                      sector_deg: float = ALIGN_SECTOR_DEG,
                      max_range_m: float = 4.0) -> Optional[float]:
    """Return bearing (rad) to nearest wall in forward sector ±sector_deg."""
    if not scan or not scan.ranges:
        return None
    sector_rad = math.radians(sector_deg)
    n = len(scan.ranges)
    best_r = max_range_m
    best_ang = None
    for i in range(n):
        ang = scan.angle_min + i * scan.angle_increment
        if abs(ang) > sector_rad:
            continue
        r = scan.ranges[i]
        if not math.isfinite(r) or r < scan.range_min or r > max_range_m:
            continue
        if r < best_r:
            best_r = r
            best_ang = ang
    return best_ang


def angle_diff(a: float, b: float) -> float:
    delta = a - b
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# ── ROS node ──────────────────────────────────────────────────────────────────

class DriftCalibNode(Node):
    """
    Subscribes:  /scan, /ultrasonic/left, /ultrasonic/right,
                 /joint_states, /odom, /imu
    Publishes:   /cmd_vel
    """

    def __init__(self):
        super().__init__("drift_calib")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Ultrasonic
        self._us: dict[str, Optional[float]] = {"left": None, "right": None}
        self._us_ok: dict[str, bool] = {"left": False, "right": False}
        self.create_subscription(Range, "/ultrasonic/left",
                                 lambda m: self._us_cb(m, "left"), BEST_EFFORT_QOS)
        self.create_subscription(Range, "/ultrasonic/right",
                                 lambda m: self._us_cb(m, "right"), BEST_EFFORT_QOS)

        # LiDAR
        self._scan: Optional[LaserScan] = None
        self._scan_ok = False
        self.create_subscription(LaserScan, "/scan",
                                 self._scan_cb, BEST_EFFORT_QOS)

        # Encoders
        self._joint_ok = False
        self._enc_collecting = False
        self._enc_pos: dict[str, Optional[float]] = {"left": None, "right": None}
        self._enc_delta: dict[str, float] = {"left": 0.0, "right": 0.0}
        self.create_subscription(JointState, "/joint_states",
                                 self._joint_cb, BEST_EFFORT_QOS)

        # Odometry
        self._odom_ok = False
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self.create_subscription(Odometry, "/odom",
                                 self._odom_cb, BEST_EFFORT_QOS)

        # IMU
        self._imu_ok = False
        self._imu_yaw = 0.0
        self.create_subscription(Imu, "/imu",
                     self._imu_cb, BEST_EFFORT_QOS)

    # ── callbacks ─────────────────────────────────────────────────────────

    def _us_cb(self, msg: Range, side: str) -> None:
        self._us_ok[side] = True
        r = msg.range
        if math.isfinite(r) and msg.min_range <= r <= min(msg.max_range, USS_MAX_RANGE_M):
            self._us[side] = r
        else:
            self._us[side] = None

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_ok = True

    def _joint_cb(self, msg: JointState) -> None:
        self._joint_ok = True
        if not self._enc_collecting:
            return
        try:
            li = msg.name.index("wheel_left_joint")
            ri = msg.name.index("wheel_right_joint")
        except ValueError:
            return
        l_pos, r_pos = msg.position[li], msg.position[ri]
        for side, pos in (("left", l_pos), ("right", r_pos)):
            prev = self._enc_pos[side]
            if prev is not None:
                delta = pos - prev
                while delta > math.pi: delta -= 2.0 * math.pi
                while delta < -math.pi: delta += 2.0 * math.pi
                self._enc_delta[side] += delta
            self._enc_pos[side] = pos

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_ok = True
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_yaw = quat_to_yaw(msg.pose.pose.orientation)

    def _imu_cb(self, msg: Imu) -> None:
        self._imu_ok = True
        self._imu_yaw = quat_to_yaw(msg.orientation)

    # ── helpers ───────────────────────────────────────────────────────────

    def spin_for(self, secs: float) -> None:
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, lin: float = 0.0, ang: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self._pub.publish(msg)

    def drive_for(self, lin: float, ang: float, secs: float) -> None:
        end = time.time() + secs
        while time.time() < end:
            self.send(lin, ang)
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self, settle: float = 0.5) -> None:
        self.send(0.0)
        self.spin_for(settle)

    def wait_topics(self, timeout: float = 8.0,
                    require_uss: bool = True,
                    require_imu: bool = False) -> dict[str, bool]:
        """Wait for essential topics. Returns dict of topic availability."""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            uss_ready = all(self._us_ok.values()) if require_uss else True
            imu_ready = self._imu_ok if require_imu else True
            if self._odom_ok and uss_ready and self._scan_ok and self._joint_ok and imu_ready:
                break
        return {
            "odom": self._odom_ok,
            "uss_left": self._us_ok["left"],
            "uss_right": self._us_ok["right"],
            "scan": self._scan_ok,
            "joints": self._joint_ok,
            "imu": self._imu_ok,
            "cmd_vel_subscriber": self.count_subscribers("/cmd_vel") > 0,
        }

    # ── encoder ───────────────────────────────────────────────────────────

    def enc_reset(self) -> None:
        self._enc_delta = {"left": 0.0, "right": 0.0}
        self._enc_pos = {"left": None, "right": None}
        self._enc_collecting = True

    def enc_stop(self) -> None:
        self._enc_collecting = False

    def enc_distance(self, wheel_radius: float) -> float:
        return wheel_radius * (abs(self._enc_delta["left"]) + abs(self._enc_delta["right"])) / 2.0

    # ── ultrasonic sampling ───────────────────────────────────────────────

    def us_sample(self, n: int = USS_SAMPLES) -> dict[str, Optional[float]]:
        """Collect n readings each side, outlier-reject, return means."""
        buckets: dict[str, list[float]] = {"left": [], "right": []}
        # Spin a few times first to flush stale data
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.15)
        for _ in range(n):
            rclpy.spin_once(self, timeout_sec=0.20)
            time.sleep(max(USS_SETTLE_S, 0.15))
            for s in ("left", "right"):
                v = self._us[s]
                if v is not None:
                    buckets[s].append(v)

        result: dict[str, Optional[float]] = {}
        for s in ("left", "right"):
            vals = buckets[s]
            if len(vals) < max(2, n // 3):
                result[s] = None
                continue
            vals_s = sorted(vals)
            med = vals_s[len(vals_s) // 2]
            filtered = [v for v in vals_s if abs(v - med) < max(0.02, med * 0.10)]
            result[s] = sum(filtered) / len(filtered) if filtered else med
        return result

    def us_mean(self, n: int = USS_SAMPLES) -> Optional[float]:
        s = self.us_sample(n)
        vals = [v for v in s.values() if v is not None]
        return sum(vals) / len(vals) if vals else None

    def front_clearance(self, half_width_deg: float = FORWARD_WINDOW_DEG) -> Optional[float]:
        scan = self._scan
        if scan is None or not scan.ranges:
            return None
        half_width = math.radians(half_width_deg)
        valid: list[float] = []
        for idx, radius in enumerate(scan.ranges):
            if not math.isfinite(radius) or radius < scan.range_min or radius > scan.range_max:
                continue
            angle = scan.angle_min + idx * scan.angle_increment
            if abs(angle) <= half_width:
                valid.append(radius)
        return min(valid) if valid else None


# ── alignment ─────────────────────────────────────────────────────────────────

def align_to_wall(node: DriftCalibNode,
                  tol_deg: float = ALIGN_TOL_DEG,
                  speed: float = ALIGN_SPEED,
                  max_iter: int = ALIGN_MAX_ITER) -> bool:
    """Coarse LiDAR alignment: rotate until facing wall."""
    print(f"\n{BLD}[Align]{NC} Finding wall in ±{ALIGN_SECTOR_DEG:.0f}° sector …")
    for i in range(max_iter):
        rclpy.spin_once(node, timeout_sec=0.2)
        scan = node._scan
        if scan is None:
            print(f"  No /scan data")
            return False
        bearing = find_wall_bearing(scan)
        if bearing is None:
            print(f"  No wall detected — is there a flat wall ahead?")
            return False
        dist_cm = min(scan.ranges[len(scan.ranges) // 2] * 100, 999)
        err_deg = math.degrees(bearing)
        print(f"  Iter {i+1}/{max_iter}: wall ≈ {dist_cm:.0f} cm  bearing error = {err_deg:+.1f}°")
        if abs(err_deg) < tol_deg:
            print(f"  Aligned! (error = {err_deg:.1f}° < {tol_deg:.1f}°)")
            return True
        # Rotate toward the wall
        rotate_time = min(abs(bearing) / speed, 2.0)
        direction = 1.0 if bearing > 0 else -1.0
        node.drive_for(0.0, direction * speed, rotate_time)
        node.stop(0.5)
    print(f"  {YLW}Alignment did not converge in {max_iter} iterations.{NC}")
    return True  # proceed anyway


def align_fine_uss(node: DriftCalibNode,
                   sensor_sep: float = USS_SENSOR_SEP_M,
                   tol_m: float = USS_FINE_TOL_M,
                   speed: float = USS_FINE_SPEED,
                   max_iter: int = USS_FINE_MAX_ITER) -> bool:
    """Fine-align using L/R ultrasonic differential."""
    print(f"\n{BLD}[Fine-align]{NC} Equalising L/R ultrasonic readings "
          f"(sep={sensor_sep*100:.0f} cm, tol={tol_m*1000:.0f} mm) …")
    for i in range(max_iter):
        samples = node.us_sample(n=5)
        dl = samples.get("left")
        dr = samples.get("right")
        if dl is None or dr is None:
            print(f"  {YLW}USS sensor not responding, skipping fine-align.{NC}")
            return False
        delta = dl - dr
        print(f"  Iter {i+1}/{max_iter}:  L={dl*100:.1f} cm  R={dr*100:.1f} cm  "
              f"Δ={delta*100:+.1f} cm")
        if abs(delta) < tol_m:
            print(f"  Fine-aligned! (Δ={delta*1000:+.0f} mm < {tol_m*1000:.0f} mm)")
            return True
        # Rotate to equalise: if delta > 0 → left is further → robot angled right → rotate left (positive)
        clamped = max(-sensor_sep * 0.98, min(sensor_sep * 0.98, delta))
        rotate_rad = math.asin(clamped / sensor_sep)
        rotate_time = min(abs(rotate_rad) / speed, 2.0)
        direction = 1.0 if rotate_rad > 0 else -1.0
        node.drive_for(0.0, direction * speed, rotate_time)
        node.stop(0.4)

    samples = node.us_sample(n=5)
    dl = samples.get("left", 0.0) or 0.0
    dr = samples.get("right", 0.0) or 0.0
    delta = dl - dr
    print(f"  Fine-align ended: Δ={delta*1000:+.0f} mm — proceeding.")
    return True


# ── backup (reverse to create space) ─────────────────────────────────────────

def backup_to_distance(node: DriftCalibNode, target_m: float,
                       wheel_radius: float,
                       speed: float = 0.05,
                       clearance_source: str = "uss") -> bool:
    """Reverse until USS mean reaches target_m. Returns True on success."""

    # Read min-duty from firmware source to compute minimum working speed
    min_duty_l = read_define_float(MAIN_C, "MOTOR_MIN_DUTY_LEFT_DEFAULT") or MOTOR_MIN_DUTY_DEFAULT
    min_duty_r = read_define_float(MAIN_C, "MOTOR_MIN_DUTY_RIGHT_DEFAULT") or MOTOR_MIN_DUTY_DEFAULT
    max_speed  = read_define_float(MAIN_C, "MAX_WHEEL_SPEED_MS_DEFAULT") or 0.10
    min_speed  = max(min_duty_l, min_duty_r) * max_speed + BACKUP_SPEED_MARGIN
    speed = max(speed, min_speed)

    print(f"  Reversing at {speed*100:.1f} cm/s to ≥{target_m*100:.0f} cm …")

    node.enc_reset()
    total_reversed = 0.0
    while total_reversed < BACKUP_MAX_M:
        node.send(-speed)
        rclpy.spin_once(node, timeout_sec=BACKUP_POLL_S)

        if clearance_source == "imu":
            d = node.front_clearance()
        else:
            d = node.us_mean(n=3)
        enc_d = node.enc_distance(wheel_radius)

        if d is not None and d >= target_m:
            node.stop(0.5)
            node.enc_stop()
            print(f"  Target reached. Reversed {enc_d*100:.1f} cm  distance={d*100:.1f} cm")
            return True
        total_reversed = enc_d

    node.stop(0.5)
    node.enc_stop()
    print(f"  {YLW}Max backup reached ({BACKUP_MAX_M*100:.0f} cm){NC}")
    d = node.front_clearance() if clearance_source == "imu" else node.us_mean(n=3)
    return d is not None and d >= target_m * 0.9


# ── drift measurement pass ───────────────────────────────────────────────────

def measure_drift(node: DriftCalibNode,
                  drive_distance_m: float,
                  drive_speed: float,
                  wheel_radius: float,
                  sensor_sep: float,
                  drift_source: str,
                  pass_num: int,
                  total_passes: int) -> Optional[dict]:
    """
    Single drift measurement pass.
    Returns dict with drift metrics, or None on failure.
    """
    print(f"\n{BLD}── Pass {pass_num}/{total_passes} ──{NC}  "
          f"target = {drive_distance_m*100:.1f} cm  speed = {drive_speed:.3f} m/s")

    bl = br = al = ar = None
    theta_rad = 0.0
    direction = "STRAIGHT"

    if drift_source == "uss":
        print(f"  Sampling ultrasonic before …", end=" ", flush=True)
        before = node.us_sample(n=USS_SAMPLES)
        bl = before.get("left")
        br = before.get("right")
        if bl is None or br is None:
            print(f"\n  {RED}USS sensor not responding — cannot measure drift{NC}")
            return None
        print(f"left={bl*100:.1f}cm  right={br*100:.1f}cm")
    else:
        print(f"  Drift source: IMU yaw + encoder distance")

    # Record yaw before
    rclpy.spin_once(node, timeout_sec=0.1)
    yaw_before = node._odom_yaw
    imu_yaw_before = node._imu_yaw

    # Reset encoders
    node.enc_reset()

    # Drive forward
    drive_time = drive_distance_m / drive_speed
    print(f"  Driving {drive_distance_m*100:.1f} cm …", end=" ", flush=True)

    drive_end = time.time() + drive_time
    while time.time() < drive_end:
        # Safety: check USS
        if drift_source == "imu":
            clearance_now = node.front_clearance()
        else:
            clearance_now = node._us.get("left") or node._us.get("right")
        if clearance_now is not None and clearance_now < WALL_STOP_M:
            node.stop(0.3)
            label = "LiDAR" if drift_source == "imu" else "USS"
            print(f"\n  {RED}SAFETY STOP — {label} < {WALL_STOP_M*100:.0f} cm{NC}")
            break
        node.send(drive_speed)
        rclpy.spin_once(node, timeout_sec=0.05)

    node.stop(0.8)
    node.enc_stop()
    print(f"done  ({drive_time:.2f} s)")

    # Encoder distances
    enc_dist = node.enc_distance(wheel_radius)
    enc_l_deg = math.degrees(node._enc_delta["left"])
    enc_r_deg = math.degrees(node._enc_delta["right"])
    print(f"  Encoder:  {enc_dist*100:.2f} cm  (ΔL={enc_l_deg:+.1f}°  ΔR={enc_r_deg:+.1f}°)")

    # Yaw after
    rclpy.spin_once(node, timeout_sec=0.1)
    yaw_after = node._odom_yaw
    yaw_change = angle_diff(yaw_after, yaw_before)
    imu_yaw_after = node._imu_yaw
    imu_yaw_change = angle_diff(imu_yaw_after, imu_yaw_before)

    delta_l = delta_r = drift_diff = 0.0
    if drift_source == "uss":
        print(f"  Sampling ultrasonic after  …", end=" ", flush=True)
        after = node.us_sample(n=USS_SAMPLES)
        al = after.get("left")
        ar = after.get("right")
        if al is None or ar is None:
            print(f"\n  {RED}USS sensor not responding after drive{NC}")
            return None
        print(f"left={al*100:.1f}cm  right={ar*100:.1f}cm")

        # If ΔR > ΔL → right side got more closer → robot angled left → left drift
        delta_l = bl - al
        delta_r = br - ar
        drift_diff = delta_r - delta_l
        theta_rad = math.atan2(drift_diff, sensor_sep)

        if drift_diff > 0.005:
            direction = "LEFT"
        elif drift_diff < -0.005:
            direction = "RIGHT"

        print(f"\n  USS ΔL = {delta_l*100:+.2f} cm    ΔR = {delta_r*100:+.2f} cm")
        print(f"  Drift diff (ΔR−ΔL) = {drift_diff*100:+.2f} cm")
        print(f"  Angular drift = {math.degrees(theta_rad):+.2f}°   → {direction}")
        print(f"  IMU yaw change  = {math.degrees(imu_yaw_change):+.2f}°")
        print(f"  Odom yaw change = {math.degrees(yaw_change):+.2f}°")
    else:
        theta_rad = imu_yaw_change
        if theta_rad > math.radians(1.0):
            direction = "LEFT"
        elif theta_rad < -math.radians(1.0):
            direction = "RIGHT"
        print(f"\n  IMU yaw change  = {math.degrees(imu_yaw_change):+.2f}°   → {direction}")
        print(f"  Odom yaw change = {math.degrees(yaw_change):+.2f}°")

    # Compute needed vel_trim:
    # The robot drifted theta_rad over distance enc_dist at speed drive_speed
    # Time = enc_dist / drive_speed
    # Drift angular velocity = theta_rad / time = theta_rad * drive_speed / enc_dist
    # To cancel: vel_trim = drift_ang_vel * wheel_sep
    # where wheel_sep is wheel_separation
    wheel_sep = read_define_float(MAIN_C, "WHEEL_SEPARATION_DEFAULT") or 0.121642
    if enc_dist > 0.01:
        drift_rate = theta_rad * drive_speed / enc_dist
        vel_trim = drift_rate * wheel_sep
    else:
        vel_trim = 0.0

    print(f"  Computed vel_trim correction = {vel_trim:+.6f} m/s")

    return {
        "drift_source": drift_source,
        "before_l": bl, "before_r": br,
        "after_l": al, "after_r": ar,
        "delta_l": delta_l, "delta_r": delta_r,
        "drift_diff": drift_diff,
        "theta_rad": theta_rad,
        "enc_dist": enc_dist,
        "yaw_change": yaw_change,
        "direction": direction,
        "vel_trim": vel_trim,
    }


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Detect and fix left/right drift using ultrasonic sensors")
    parser.add_argument("--speed", type=float, default=0.070,
                        help="forward drive speed (m/s, default 0.070)")
    parser.add_argument("--distance", type=float, default=None,
                        help="test distance in metres (auto-computed from USS clearance if omitted)")
    parser.add_argument("--passes", type=int, default=3,
                        help="number of measurement passes (default 3)")
    parser.add_argument("--sensor-sep", type=float, default=USS_SENSOR_SEP_M,
                        help=f"ultrasonic sensor lateral separation in metres (default {USS_SENSOR_SEP_M})")
    parser.add_argument("--apply", action="store_true",
                        help="write VEL_TRIM_FWD_DEFAULT to firmware/main.c")
    parser.add_argument("--flash", action="store_true",
                        help="also rebuild and flash the firmware (implies --apply)")
    parser.add_argument("--verify-only", action="store_true",
                        help="one pass, just report, no changes")
    parser.add_argument("--no-align", action="store_true",
                        help="skip LiDAR/USS alignment (assume robot is facing wall)")
    parser.add_argument("--runtime-only", action="store_true",
                        help="write trim to firmware register only (no source changes)")
    parser.add_argument("--drift-source", choices=("uss", "imu"), default="uss",
                        help="how to estimate heading drift: ultrasonic differential or IMU yaw (default: uss)")
    parser.add_argument("--no-auto-start-services", action="store_true",
                        help="do not attempt to start required system services before waiting for topics")
    args = parser.parse_args()

    if args.flash:
        args.apply = True
    if args.verify_only:
        args.passes = 1

    # Read current firmware parameters
    wheel_radius = read_define_float(MAIN_C, "WHEEL_RADIUS_DEFAULT") or WHEEL_RADIUS_DEFAULT_M
    max_speed = read_define_float(MAIN_C, "MAX_WHEEL_SPEED_MS_DEFAULT") or 0.10
    wheel_sep = read_define_float(MAIN_C, "WHEEL_SEPARATION_DEFAULT") or 0.121642
    current_trim_fwd = read_define_float(MAIN_C, "VEL_TRIM_FWD_DEFAULT")

    print(f"\n{BLD}=== Drift Calibration (ultrasonic L/R differential) ==={NC}")
    print(f"  Firmware           : {MAIN_C}")
    print(f"  Wheel radius       : {wheel_radius*1000:.1f} mm")
    print(f"  Wheel separation   : {wheel_sep*1000:.1f} mm")
    print(f"  MAX_WHEEL_SPEED_MS : {max_speed:.6f} m/s")
    print(f"  Drive speed        : {args.speed:.3f} m/s")
    print(f"  Passes             : {args.passes}")
    print(f"  Drift source       : {args.drift_source}")
    print(f"  Sensor separation  : {args.sensor_sep*100:.1f} cm")
    if current_trim_fwd is not None:
        print(f"  Current VEL_TRIM_FWD_DEFAULT : {current_trim_fwd:+.6f} m/s")
    else:
        print(f"  VEL_TRIM_FWD_DEFAULT : {YLW}not found in source (will be added){NC}")

    # Initialise ROS
    rclpy.init()
    node = DriftCalibNode()

    print(f"\nChecking required services …")
    services_ok = ensure_required_services(
        args.drift_source,
        auto_start=not args.no_auto_start_services,
    )
    if not services_ok:
        node.destroy_node()
        rclpy.shutdown()
        return

    required = ["odom", "scan", "joints"]
    if args.drift_source == "uss":
        required.extend(["uss_left", "uss_right"])
    if args.drift_source == "imu":
        required.append("imu")

    def check_topics_once() -> tuple[dict[str, bool], list[str], bool]:
        print(f"\nWaiting for topics …", end=" ", flush=True)
        avail_local = node.wait_topics(timeout=10.0,
                                       require_uss=(args.drift_source == "uss"),
                                       require_imu=(args.drift_source == "imu"))
        missing_local = [k for k in required if not avail_local.get(k, False)]
        cmd_vel_ok = avail_local.get("cmd_vel_subscriber", False)
        return avail_local, missing_local, cmd_vel_ok

    avail, missing, cmd_vel_ok = check_topics_once()
    if missing or not cmd_vel_ok:
        if missing:
            print(f"\n  {RED}Missing: {', '.join(missing)}{NC}")
        if not cmd_vel_ok:
            print(f"\n  {RED}No active subscriber on /cmd_vel{NC}")
        if not args.no_auto_start_services:
            print(f"  {YLW}Required telemetry is missing; attempting service restart recovery.{NC}")
            if restart_required_services(args.drift_source):
                avail, missing, cmd_vel_ok = check_topics_once()

    if missing:
        print(f"\n  {RED}Missing: {', '.join(missing)}{NC}")
        if "uss_left" in missing or "uss_right" in missing:
            print(f"  {YLW}USS service may not be running. Check: "
                  f"systemctl status pi-ultrasonic-dual.service{NC}")
        print(f"  {YLW}Required robot telemetry is not live. Start bringup before calibrating.{NC}")
        node.destroy_node()
        rclpy.shutdown()
        return
    if not cmd_vel_ok:
        print(f"\n  {RED}No active subscriber on /cmd_vel{NC}")
        print(f"  {YLW}The robot base is not currently accepting velocity commands. Start bringup and retry.{NC}")
        node.destroy_node()
        rclpy.shutdown()
        return

    print("OK  (all required topics responding)")

    results = []

    try:
        for pass_num in range(1, args.passes + 1):
            # ── Alignment ────────────────────────────────────────────────
            if not args.no_align:
                align_to_wall(node)
                if args.drift_source == "uss":
                    align_fine_uss(node, sensor_sep=args.sensor_sep)
                else:
                    print(f"\n{BLD}[Fine-align]{NC} Skipped in IMU mode.")

            # ── Space check ──────────────────────────────────────────────
            if args.drift_source == "imu":
                d_wall = node.front_clearance()
            else:
                d_wall = node.us_mean(n=5)
            if d_wall is None:
                source_name = "LiDAR" if args.drift_source == "imu" else "USS"
                print(f"  {RED}Cannot read {source_name} distance{NC}")
                continue

            source_name = "LiDAR" if args.drift_source == "imu" else "USS"
            print(f"\n{BLD}[Space]{NC} Distance to wall ({source_name}): {d_wall*100:.1f} cm")

            if d_wall * 100 < MIN_SPACE_M * 100:
                print(f"  Too close ({d_wall*100:.0f} cm < {MIN_SPACE_M*100:.0f} cm). "
                      f"Reversing …")
                target = MIN_SPACE_M * 1.1
                if not backup_to_distance(node, target, wheel_radius,
                                          clearance_source=args.drift_source):
                    print(f"  {RED}Could not create enough space.{NC}")
                    continue

                # After backup, re-measure
                d_wall = node.front_clearance() if args.drift_source == "imu" else node.us_mean(n=5)
                if d_wall is None:
                    continue

            # Compute test distance
            available = d_wall - WALL_CLEARANCE_M
            if args.distance:
                test_dist = max(MIN_DRIVE_M, min(args.distance, available, MAX_DRIVE_M))
            else:
                test_dist = max(MIN_DRIVE_M, min(available, MAX_DRIVE_M))
            print(f"  Available = {available*100:.1f} cm  → test distance = {test_dist*100:.1f} cm")

            # ── Measurement ──────────────────────────────────────────────
            result = measure_drift(
                node, test_dist, args.speed, wheel_radius,
                args.sensor_sep, args.drift_source, pass_num, args.passes)

            if result:
                results.append(result)

            # ── Between passes: manual reposition ────────────────────────
            if pass_num < args.passes:
                print(f"\n  Drive robot back to start position.")
                input(f"  Press Enter when ready for pass {pass_num + 1} … ")

    except KeyboardInterrupt:
        print(f"\n{YLW}Interrupted.{NC}")
        node.stop()

    # ── Results ───────────────────────────────────────────────────────────
    node.stop()

    if not results:
        print(f"\n{RED}No valid measurement passes.{NC}")
        node.destroy_node()
        rclpy.shutdown()
        return

    print(f"\n{BLD}───── Drift Calibration Results ─────────────────────────{NC}")
    trims = []
    for i, r in enumerate(results):
        theta_deg = math.degrees(r["theta_rad"])
        print(f"  Pass {i+1} [{r['drift_source']}]: θ = {theta_deg:+.2f}°  direction = {r['direction']}  "
              f"vel_trim = {r['vel_trim']:+.6f} m/s")
        trims.append(r["vel_trim"])

    # Remove outlier passes (if σ > mean, drop furthest from median)
    if len(trims) >= 3:
        avg = sum(trims) / len(trims)
        sigma = (sum((t - avg) ** 2 for t in trims) / len(trims)) ** 0.5
        if sigma > abs(avg) * 0.5 and sigma > 0.001:
            trims_sorted = sorted(trims, key=lambda t: abs(t - sorted(trims)[len(trims) // 2]))
            trims = trims_sorted[:-1]  # drop the furthest from median
            print(f"  (Dropped 1 outlier pass; using {len(trims)} passes)")

    avg_trim = sum(trims) / len(trims)
    sigma = (sum((t - avg_trim) ** 2 for t in trims) / len(trims)) ** 0.5 if len(trims) > 1 else 0.0

    # Add to existing trim if there's already a non-zero default
    existing_trim = current_trim_fwd if current_trim_fwd is not None else 0.0
    new_trim = existing_trim + avg_trim

    print(f"\n  Average vel_trim correction : {avg_trim:+.6f} m/s  (σ = {sigma:.6f})")
    if existing_trim != 0.0:
        print(f"  Existing VEL_TRIM_FWD       : {existing_trim:+.6f} m/s")
    print(f"  New VEL_TRIM_FWD            : {new_trim:+.6f} m/s")

    if avg_trim > 0.001:
        print(f"\n  {YLW}Robot drifts LEFT → positive trim "
              f"(speeds up left, slows right){NC}")
    elif avg_trim < -0.001:
        print(f"\n  {YLW}Robot drifts RIGHT → negative trim "
              f"(speeds up right, slows left){NC}")
    else:
        print(f"\n  {GRN}Robot drives straight — no significant drift detected.{NC}")

    # Safety check
    if abs(new_trim) > 0.03:
        print(f"\n  {RED}Warning: vel_trim {new_trim:+.6f} m/s > 30 mm/s safety limit.{NC}")
        print(f"  {RED}Verify robot was facing a flat wall squarely.{NC}")

    # ── Apply ─────────────────────────────────────────────────────────────

    if args.verify_only:
        print(f"\n  {CYN}Verify-only mode — no changes applied.{NC}")
    elif args.runtime_only:
        # Write to firmware register at runtime (no source changes)
        print(f"\n  Writing VEL_TRIM_FWD = {new_trim:.6f} to firmware register …", end=" ")
        if dxl_write_float(ADDR_VEL_TRIM_FWD, new_trim):
            print(f"{GRN}OK{NC}")
            verify = dxl_read_float(ADDR_VEL_TRIM_FWD)
            if verify is not None:
                print(f"  Readback: {verify:+.6f} m/s")
        else:
            print(f"{RED}FAILED{NC}")
        print(f"\n  {YLW}Note: runtime-only — trim resets on next power cycle.{NC}")
        print(f"  To persist: re-run with --apply to write firmware/main.c")
    elif args.apply:
        # Ensure VEL_TRIM_FWD_DEFAULT exists in source
        if current_trim_fwd is None:
            # Need to add the #define to main.c
            _add_vel_trim_defines(new_trim, 0.0)
            print(f"\n  Added VEL_TRIM_FWD_DEFAULT = {new_trim:.6f}f to firmware/main.c")
        else:
            write_define_float(MAIN_C, "VEL_TRIM_FWD_DEFAULT", new_trim,
                               f"calibrated {time.strftime('%Y-%m-%d')}: drift_calib, "
                               f"trim={avg_trim:+.6f}")
            print(f"\n  Updated VEL_TRIM_FWD_DEFAULT = {new_trim:.6f}f in firmware/main.c")

        # Also write to firmware register for immediate effect
        print(f"  Writing to firmware register …", end=" ")
        if dxl_write_float(ADDR_VEL_TRIM_FWD, new_trim):
            print(f"{GRN}OK{NC}")
        else:
            print(f"{YLW}skipped (ROS bringup may be using the port){NC}")

        if args.flash:
            print(f"\n  Rebuilding and flashing firmware …")
            result = subprocess.run(
                ["bash", str(BUILD_SCRIPT), "flash"],
                cwd=str(FIRMWARE_DIR),
                env={**os.environ, "IMU_TYPE": "2"},
                capture_output=True, text=True
            )
            if result.returncode == 0:
                print(f"  {GRN}Flash successful.{NC}")
                print(f"  Restart bringup: sudo systemctl restart turtlebot3-bringup.service")
            else:
                print(f"  {RED}Flash failed:{NC}")
                print(result.stderr[-500:] if result.stderr else "(no stderr)")
        else:
            print(f"\n  To rebuild + flash:")
            print(f"    cd firmware && IMU_TYPE=2 ./build.sh flash")
            print(f"    sudo systemctl restart turtlebot3-bringup.service")
    else:
        print(f"\n  {CYN}Dry-run — add --apply to write firmware/main.c{NC}")
        print(f"  Or --runtime-only to apply immediately (volatile until reboot)")

    node.destroy_node()
    rclpy.shutdown()


# ── helper to add VEL_TRIM #defines to main.c ────────────────────────────────

def _add_vel_trim_defines(fwd_val: float, rev_val: float) -> None:
    """Insert VEL_TRIM_FWD/REV_DEFAULT #defines near the heading hold section."""
    content = MAIN_C.read_text()

    # Find a good insertion point — right after the encoder trim defines
    marker = "#define ENC_TRIM_MAX_CORR"
    idx = content.find(marker)
    if idx < 0:
        raise RuntimeError(f"Cannot find '{marker}' in {MAIN_C}")

    # Find the end of that line
    eol = content.index('\n', idx)

    insert_text = (
        f"\n\n// Velocity-level feedforward trim defaults — compensates known motor"
        f"\n// asymmetry that causes left/right drift during straight-line driving."
        f"\n// Learned by calibrate_drift.py, applied as:"
        f"\n//   v_left  += trim / 2"
        f"\n//   v_right -= trim / 2"
        f"\n// Positive = correct left drift (speed up left, slow right)."
        f"\n#define VEL_TRIM_FWD_DEFAULT          {fwd_val:.6f}f  // forward driving"
        f"\n#define VEL_TRIM_REV_DEFAULT          {rev_val:.6f}f  // reverse driving"
    )

    new_content = content[:eol + 1] + insert_text + content[eol + 1:]
    MAIN_C.write_text(new_content)

    # Also update init_registers to use the defaults instead of 0.0
    new_content = MAIN_C.read_text()
    new_content = new_content.replace(
        "w_f32(ADDR_VEL_TRIM_FWD, 0.0f);",
        "w_f32(ADDR_VEL_TRIM_FWD, VEL_TRIM_FWD_DEFAULT);"
    )
    new_content = new_content.replace(
        "w_f32(ADDR_VEL_TRIM_REV, 0.0f);",
        "w_f32(ADDR_VEL_TRIM_REV, VEL_TRIM_REV_DEFAULT);"
    )
    MAIN_C.write_text(new_content)


if __name__ == "__main__":
    main()
