#!/usr/bin/env python3
"""
calibrate_distance.py — Ultrasonic-based wheel-radius calibration for TurtleBot3.

Subscribes to /ultrasonic/left and /ultrasonic/right ROS topics (published by
pi_ultrasonic_dual_ros2.py as sensor_msgs/Range) to obtain ground-truth
forward distance measurements.  The robot drives toward a flat wall, and the
difference between ultrasonic readings before and after the move gives the
actual distance traveled.  This is compared with ROS odometry to compute a
wheel_radius correction factor.

Workflow (per pass):
  0. Align perpendicular to wall using left/right USS differential
  1. Read initial ultrasonic distance(s) from ROS topics
  2. Record initial odometry position
  3. Drive forward (closed-loop USS) until target distance reached
  4. Stop and settle, read final ultrasonic distance(s)
  5. Compute: correction = uss_delta / odom_delta
  6. Auto-reverse back to start position (closed-loop USS)
  7. Re-align and repeat

After all passes, the averaged correction is applied via the runtime
calibration instruction (0x90 SET) and optionally persisted to flash and
source files.

Safety:
  • Stops immediately if ultrasonic distance < WALL_STOP_MM (200 mm default)
  • Wall proximity check before each pass
  • Sanity limits on correction factor (reject if > ±30 %)

Requirements:
  • ROS 2 bringup running (/odom topic active)
  • pi_ultrasonic_dual_ros2.py running (publishes /ultrasonic/left, /right)

Usage:
  # Dry-run (measure and report only):
  python3 calibrate_distance.py

  # Apply result to runtime + flash + source:
  python3 calibrate_distance.py --apply

  # Custom parameters:
  python3 calibrate_distance.py --distance 0.15 --speed 0.04 --passes 5 --apply

  # Use only left sensor:
  python3 calibrate_distance.py --sensor left

  # Show live ultrasonic readings without calibrating:
  python3 calibrate_distance.py --monitor
"""

import argparse
import math
import re
import struct
import sys
import time
from pathlib import Path
from typing import Optional

import serial

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Range

# Sensor / odometry QoS compatible with TurtleBot3 bringup
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── paths & constants ─────────────────────────────────────────────────────────

SCRIPT_DIR   = Path(__file__).resolve().parent
MAIN_C       = SCRIPT_DIR / "firmware" / "main.c"

DEV_ID       = 200
DXL_BAUD     = 1_000_000

# Runtime calibration (custom instruction 0x90)
INST_CALIBRATION = 0x90
CALIB_CMD_SET    = 0x01
CALIB_CMD_GET    = 0x02
CALIB_CMD_SAVE   = 0x04

CALIB_KEY_WHEEL_RADIUS = 0x01

# Ultrasonic ROS topic defaults
DEFAULT_USS_LEFT_TOPIC  = "/ultrasonic/left"
DEFAULT_USS_RIGHT_TOPIC = "/ultrasonic/right"

# Defaults
DEFAULT_PORT         = "/dev/ttyTB3"
DEFAULT_DISTANCE_M   = 0.15      # target travel per pass (m)
DEFAULT_SPEED        = 0.04      # forward speed (m/s)
DEFAULT_PASSES       = 3
DEFAULT_WALL_STOP_MM = 200       # minimum allowed approach distance (mm)
MAX_CORRECTION_PCT   = 0.30      # reject if correction > ±30 %
SETTLE_TIME          = 1.0       # seconds to wait after stopping motors
USS_READ_SAMPLES     = 10        # number of USS readings to average per measurement
USS_READ_INTERVAL    = 0.12      # seconds between USS readings

# Alignment parameters
ALIGN_TOL_M          = 0.010     # convergence: 1 cm left/right differential
ALIGN_SENSOR_SEP_M   = 0.15      # lateral distance between ultrasonic sensors (m)
ALIGN_SPEED          = 0.20      # in-place rotation speed (rad/s)
ALIGN_MAX_ITER       = 10        # max alignment iterations
ALIGN_SETTLE_S       = 0.4       # settle time after each rotation

# Heading-hold (anti-drift) during straight-line driving
HEADING_HOLD_KP      = 1.5       # proportional gain (rad/s per rad heading error)
HEADING_HOLD_MAX     = 0.30      # max angular correction (rad/s)
HEADING_HOLD_DEADBAND = 0.005    # ignore heading error below this (rad)

# Wheel radius bounds
WHEEL_RADIUS_MIN     = 0.020     # 20 mm — anything below is clearly wrong
WHEEL_RADIUS_MAX     = 0.060     # 60 mm — anything above is clearly wrong

# Colours
RED = "\033[0;31m"
GRN = "\033[0;32m"
YLW = "\033[1;33m"
CYN = "\033[0;36m"
BLD = "\033[1m"
NC  = "\033[0m"


# ── Dynamixel Protocol 2.0 helpers ───────────────────────────────────────────

def _make_crc_table() -> list[int]:
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


def _build_read(dev_id: int, addr: int, length: int) -> bytes:
    """Build a Dynamixel READ instruction packet."""
    params = struct.pack('<HH', addr, length)
    pkt_len = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, dev_id,
                 pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, 0x02])
    pkt = hdr + params
    crc = _crc16(pkt)
    return pkt + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _build_write(dev_id: int, addr: int, data_bytes: bytes) -> bytes:
    """Build a Dynamixel WRITE instruction packet."""
    params = struct.pack('<H', addr) + data_bytes
    pkt_len = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, dev_id,
                 pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, 0x03])
    pkt = hdr + params
    crc = _crc16(pkt)
    return pkt + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _build_calibration(dev_id: int, subcmd: int, key: int = 0,
                        value: float = 0.0) -> bytes:
    """Build a custom 0x90 calibration instruction packet."""
    if subcmd == CALIB_CMD_SET:
        params = bytes([subcmd, key]) + struct.pack('<f', float(value))
    elif subcmd == CALIB_CMD_GET:
        params = bytes([subcmd, key])
    else:
        params = bytes([subcmd])
    pkt_len = len(params) + 3
    hdr = bytes([0xFF, 0xFF, 0xFD, 0x00, dev_id,
                 pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, INST_CALIBRATION])
    pkt = hdr + params
    crc = _crc16(pkt)
    return pkt + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _read_response(ser: serial.Serial, timeout: float = 1.0) -> Optional[bytes]:
    """Read a Dynamixel status (response) packet from the serial port."""
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


# ── serial helpers ────────────────────────────────────────────────────────────

class DxlPort:
    """Simple Dynamixel serial wrapper with READ, WRITE, and calibration."""

    def __init__(self, port: str):
        self.ser = serial.Serial(port, DXL_BAUD, timeout=0.05, write_timeout=0.5)
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def read_reg(self, addr: int, length: int) -> Optional[bytes]:
        pkt = _build_read(DEV_ID, addr, length)
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        self.ser.flush()
        resp = _read_response(self.ser, timeout=0.5)
        if resp and len(resp) >= 11:
            # Status packet: [hdr(7)] [inst=0x55] [error] [data...] [crc(2)]
            error = resp[8]
            if error != 0:
                return None
            data = resp[9:-2]
            if len(data) >= length:
                return data[:length]
        return None

    def write_reg(self, addr: int, data: bytes) -> bool:
        pkt = _build_write(DEV_ID, addr, data)
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        self.ser.flush()
        resp = _read_response(self.ser, timeout=0.3)
        if resp and len(resp) >= 9:
            return resp[8] == 0  # error byte
        return False

    def calib_set(self, key: int, value: float) -> None:
        pkt = _build_calibration(0xFE, CALIB_CMD_SET, key, value)
        self.ser.write(pkt)
        self.ser.flush()
        time.sleep(0.02)

    def calib_get(self, key: int) -> Optional[float]:
        pkt = _build_calibration(DEV_ID, CALIB_CMD_GET, key)
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        self.ser.flush()
        resp = _read_response(self.ser, timeout=0.5)
        if resp and len(resp) >= 13:
            data = resp[9:-2]
            if len(data) >= 4:
                return struct.unpack('<f', data[:4])[0]
        return None

    def calib_save(self) -> None:
        pkt = _build_calibration(0xFE, CALIB_CMD_SAVE)
        self.ser.write(pkt)
        self.ser.flush()
        time.sleep(0.10)




# ── source file helpers ───────────────────────────────────────────────────────

def read_define_float(path: Path, name: str) -> Optional[float]:
    pat = re.compile(rf"^\s*#define\s+{re.escape(name)}\s+([\d.+\-eEfF]+)")
    for line in path.read_text().splitlines():
        m = pat.match(line)
        if m:
            return float(m.group(1).rstrip("fF"))
    return None


def write_define_float(path: Path, name: str, value: float,
                       fmt: str = ".6f", comment: str = "") -> None:
    content = path.read_text()
    suffix  = f"  // {comment}" if comment else ""
    pat     = re.compile(
        rf"(#define\s+{re.escape(name)}\s+)[\d.+\-eEfF]+f?([ \t]*(?://[^\n]*)?)"
    )
    def repl(m):
        return f"{m.group(1)}{value:{fmt}}f{suffix}"
    new_content, count = pat.subn(repl, content, count=1)
    if count == 0:
        raise RuntimeError(f"Could not find #define {name} in {path}")
    path.write_text(new_content)


# ── ROS 2 node ───────────────────────────────────────────────────────────────

class DistanceCalibNode(Node):
    """Subscribes to /odom, ultrasonic topics, and publishes /cmd_vel."""

    def __init__(self, sensor_sides: list[str]):
        super().__init__("calibrate_distance")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_ok = False
        self.create_subscription(
            Odometry, "/odom", self._odom_cb, SENSOR_QOS)

        # Ultrasonic (subscribe to Range topics)
        self._us: dict[str, Optional[float]] = {s: None for s in sensor_sides}
        self._us_ok: dict[str, bool] = {s: False for s in sensor_sides}
        topic_map = {"left": DEFAULT_USS_LEFT_TOPIC, "right": DEFAULT_USS_RIGHT_TOPIC}
        for side in sensor_sides:
            topic = topic_map[side]
            self.create_subscription(
                Range, topic,
                lambda msg, s=side: self._us_cb(msg, s), SENSOR_QOS)

    def _odom_cb(self, msg: Odometry):
        self._odom_ok = True
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)

    def _us_cb(self, msg: Range, side: str) -> None:
        self._us_ok[side] = True
        r = msg.range
        if math.isfinite(r) and msg.min_range <= r <= msg.max_range:
            self._us[side] = r  # metres
        else:
            self._us[side] = None

    def wait_for_odom(self, timeout: float = 8.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._odom_ok:
                return True
        return False

    def wait_for_uss(self, sides: list[str], timeout: float = 8.0) -> list[str]:
        """Wait until at least one ultrasonic topic has delivered data.
        Returns list of sides that are responding."""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            ready = [s for s in sides if self._us_ok[s]]
            if ready:
                return ready
        return []

    def get_odom_xy(self) -> tuple[float, float]:
        rclpy.spin_once(self, timeout_sec=0.05)
        return (self._odom_x, self._odom_y)

    def get_yaw(self) -> float:
        """Return current yaw (rad) from odometry."""
        return self._odom_yaw

    def us_read_mm(self, side: str) -> Optional[float]:
        """Return latest cached ultrasonic reading in mm, or None."""
        v = self._us[side]
        return v * 1000.0 if v is not None else None

    def us_read_m(self, side: str) -> Optional[float]:
        """Return latest cached ultrasonic reading in metres, or None."""
        return self._us[side]

    def drive(self, speed: float) -> None:
        msg = Twist()
        msg.linear.x = speed
        self.pub.publish(msg)

    def drive_twist(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    def drive_for(self, linear: float, angular: float, secs: float) -> None:
        """Publish cmd_vel continuously for `secs` seconds, then stop."""
        end = time.time() + secs
        while time.time() < end:
            self.drive_twist(linear, angular)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop(0.3)

    def stop(self, settle: float = 0.5) -> None:
        msg = Twist()
        self.pub.publish(msg)
        end = time.time() + settle
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def spin_ros(self, duration: float) -> None:
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


# ── measurement helpers ───────────────────────────────────────────────────────

def read_uss_averaged(side: str, node: DistanceCalibNode,
                      num_samples: int = USS_READ_SAMPLES,
                      interval: float = USS_READ_INTERVAL) -> Optional[float]:
    """Collect `num_samples` readings from the ROS topic, discard outliers, return mean mm."""
    readings: list[float] = []
    for _ in range(num_samples):
        node.spin_ros(interval)
        val = node.us_read_mm(side)
        if val is not None:
            readings.append(val)

    if len(readings) < num_samples // 2:
        return None  # too many timeouts

    # Discard outliers (>2σ from median)
    readings.sort()
    median = readings[len(readings) // 2]
    filtered = [r for r in readings if abs(r - median) < max(20, median * 0.10)]
    if not filtered:
        filtered = readings

    return sum(filtered) / len(filtered)


def read_all_uss(sensor_sides: list[str], node: DistanceCalibNode,
                 num_samples: int = USS_READ_SAMPLES) -> dict[str, Optional[float]]:
    """Read averaging for each enabled sensor via ROS topic."""
    results = {}
    for side in sensor_sides:
        results[side] = read_uss_averaged(side, node, num_samples)
    return results


# ── closed-loop alignment ────────────────────────────────────────────────────

def align_to_wall(node: DistanceCalibNode,
                  tol_m: float      = ALIGN_TOL_M,
                  sensor_sep: float = ALIGN_SENSOR_SEP_M,
                  speed: float      = ALIGN_SPEED,
                  max_iter: int     = ALIGN_MAX_ITER) -> bool:
    """
    Rotate in-place until left and right ultrasonic readings match (± tol_m).

    Both sensors face the wall, so:
      d_left > d_right → robot yawed CCW → rotate CW  (negative angular.z)
      d_left < d_right → robot yawed CW  → rotate CCW (positive angular.z)

    Returns True if aligned within tolerance.
    """
    print(f"  Aligning to wall …", end="", flush=True)

    for i in range(max_iter):
        # Sample both sensors
        d_left = read_uss_averaged("left", node, num_samples=5, interval=0.08)
        d_right = read_uss_averaged("right", node, num_samples=5, interval=0.08)

        if d_left is None or d_right is None:
            print(f" {YLW}(sensor timeout iter {i+1}){NC}", end="", flush=True)
            node.spin_ros(0.3)
            continue

        # Convert mm → m for geometry
        dl_m = d_left / 1000.0
        dr_m = d_right / 1000.0
        delta = dl_m - dr_m

        if abs(delta) <= tol_m:
            print(f" {GRN}OK{NC} (Δ={delta*1000:.1f} mm after {i+1} iter)")
            return True

        # Compute rotation angle: θ = arcsin(Δd / sensor_separation)
        clamped = max(-sensor_sep * 0.98, min(sensor_sep * 0.98, delta))
        rotate_rad = math.asin(clamped / sensor_sep)
        rotate_time = abs(rotate_rad) / speed
        rotate_time = max(rotate_time, 0.05)  # minimum pulse

        ang_z = -math.copysign(speed, rotate_rad)
        node.drive_for(0.0, ang_z, rotate_time)
        node.spin_ros(ALIGN_SETTLE_S)

        print(f".", end="", flush=True)

    # Final check
    d_left = read_uss_averaged("left", node, num_samples=5, interval=0.08)
    d_right = read_uss_averaged("right", node, num_samples=5, interval=0.08)
    if d_left is not None and d_right is not None:
        final_delta = abs(d_left - d_right) / 1000.0
        if final_delta <= tol_m * 3:
            print(f" {YLW}close enough{NC} (Δ={final_delta*1000:.1f} mm)")
            return True

    print(f" {RED}failed{NC}")
    return False


def drive_closed_loop(node: DistanceCalibNode,
                      sensor_sides: list[str],
                      target_dist_mm: float,
                      speed: float,
                      wall_stop_mm: float,
                      direction: str = "forward") -> tuple[float, dict[str, Optional[float]], dict[str, Optional[float]]]:
    """
    Drive forward or reverse by `target_dist_mm` using ultrasonic closed-loop
    with odometry-based heading-hold to prevent drift.

    Reads USS before, drives until USS delta reaches target (or odom as
    fallback), reads USS after.  During the drive, a proportional controller
    corrects angular velocity based on heading error from odom.

    Returns (odom_delta_m, d_before_mm, d_after_mm).
    """
    is_fwd = (direction == "forward")
    lin_speed = speed if is_fwd else -speed

    # 1. Read initial ultrasonic distances
    d_before = read_all_uss(sensor_sides, node)
    valid = [s for s in sensor_sides if d_before[s] is not None]
    if not valid:
        return (0.0, d_before, d_before)

    # 2. Record initial odometry and heading
    odom_x0, odom_y0 = node.get_odom_xy()
    yaw0 = node.get_yaw()

    # 3. Drive with closed-loop USS monitoring + heading hold
    dir_label = "forward" if is_fwd else "reverse"
    print(f"  Driving {dir_label} at {abs(speed):.3f} m/s …", end="", flush=True)

    t_start = time.time()
    max_time = (target_dist_mm / 1000.0) / abs(speed) * 4.0  # generous timeout

    while True:
        # Heading-hold: compute angular correction
        rclpy.spin_once(node, timeout_sec=0.03)
        yaw_now = node.get_yaw()
        heading_err = yaw_now - yaw0
        # Normalize to [-π, π]
        while heading_err > math.pi:  heading_err -= 2.0 * math.pi
        while heading_err < -math.pi: heading_err += 2.0 * math.pi

        ang_corr = 0.0
        if abs(heading_err) > HEADING_HOLD_DEADBAND:
            ang_corr = -HEADING_HOLD_KP * heading_err
            ang_corr = max(-HEADING_HOLD_MAX, min(HEADING_HOLD_MAX, ang_corr))

        node.drive_twist(lin_speed, ang_corr)

        # Check USS delta
        uss_deltas = []
        for s in valid:
            d_now = node.us_read_mm(s)
            if d_now is not None and d_before[s] is not None:
                if is_fwd:
                    delta = d_before[s] - d_now
                else:
                    delta = d_now - d_before[s]
                uss_deltas.append(delta)

                # Safety: wall proximity on forward approach
                if is_fwd and d_now < wall_stop_mm:
                    node.stop(0.5)
                    print(f"\n  {RED}WALL STOP:{NC} USS-{s}={d_now:.0f} mm < {wall_stop_mm} mm")
                    break

        # Check if target reached (average USS delta)
        if uss_deltas:
            avg_delta = sum(uss_deltas) / len(uss_deltas)
            if avg_delta >= target_dist_mm:
                break

        # Fallback: odom-based distance check
        ox, oy = node.get_odom_xy()
        odom_dist = math.sqrt((ox - odom_x0) ** 2 + (oy - odom_y0) ** 2) * 1000.0
        if odom_dist >= target_dist_mm * 1.5:
            break

        if time.time() - t_start > max_time:
            print(f" {YLW}timeout{NC}", end="", flush=True)
            break

    node.stop(SETTLE_TIME)
    print(f" done")

    # 4. Record final odometry
    odom_x1, odom_y1 = node.get_odom_xy()
    odom_delta = math.sqrt((odom_x1 - odom_x0) ** 2 + (odom_y1 - odom_y0) ** 2)

    # 5. Read final ultrasonic distances
    d_after = read_all_uss(sensor_sides, node)

    return (odom_delta, d_before, d_after)


# ── main calibration ─────────────────────────────────────────────────────────

def run_monitor(sensor_sides: list[str], node: DistanceCalibNode):
    """Continuously print ultrasonic readings from ROS topics (Ctrl-C to exit)."""
    print(f"\n{BLD}Live ultrasonic monitor (ROS topics){NC}  (Ctrl-C to exit)\n")
    try:
        while True:
            node.spin_ros(0.15)
            parts = []
            for side in sensor_sides:
                val = node.us_read_mm(side)
                if val is not None:
                    parts.append(f"USS-{side}: {val:7.1f} mm ({val/10:.1f} cm)")
                else:
                    parts.append(f"USS-{side}: ----  (no data)")
            print(f"  {' | '.join(parts)}", end="\r")
    except KeyboardInterrupt:
        print("\n")


def run_calibration(args):
    """Execute the ultrasonic distance calibration procedure."""
    port_path = args.port
    target_dist_m = args.distance
    speed = args.speed
    n_passes = args.passes
    wall_stop_mm = args.wall_stop
    apply = args.apply
    sensor_sides = [s.strip() for s in args.sensor.split(",")]

    print(f"\n{BLD}=== Ultrasonic Distance Calibration (ROS topics) ==={NC}")
    print(f"  Port:       {port_path}")
    print(f"  Sensors:    {sensor_sides}")
    print(f"  Target:     {target_dist_m:.3f} m per pass")
    print(f"  Speed:      {speed:.3f} m/s")
    print(f"  Passes:     {n_passes}")
    print(f"  Wall stop:  {wall_stop_mm} mm")
    print(f"  Apply:      {'yes' if apply else 'dry-run'}\n")

    # --- Open Dynamixel port (for calibration read/write only) ---
    dxl: Optional[DxlPort] = None
    try:
        dxl = DxlPort(port_path)
    except serial.SerialException as e:
        print(f"{YLW}WARN:{NC} Cannot open {port_path}: {e}")
        print("  Calibration values will be read from source only.")

    # --- Init ROS ---
    rclpy.init()
    node = DistanceCalibNode(sensor_sides)

    try:
        # Wait for odometry
        print(f"  Waiting for /odom topic …", end="", flush=True)
        if not node.wait_for_odom(10.0):
            print(f"\n{RED}ERROR:{NC} No /odom messages received.  Is bringup running?")
            sys.exit(2)
        print(f" {GRN}OK{NC}")

        # --- Read current wheel_radius ---
        current_radius = None
        if dxl is not None:
            current_radius = dxl.calib_get(CALIB_KEY_WHEEL_RADIUS)
        if current_radius is None or current_radius < WHEEL_RADIUS_MIN:
            current_radius = read_define_float(MAIN_C, "WHEEL_RADIUS_DEFAULT")
        if current_radius is None or current_radius < WHEEL_RADIUS_MIN:
            current_radius = 0.03405  # TurtleBot3 Burger default
        print(f"  Current WHEEL_RADIUS = {current_radius:.6f} m")

        # --- Verify ultrasonic ROS topics are responding ---
        print(f"  Waiting for ultrasonic topics …", end="", flush=True)
        ready_sides = node.wait_for_uss(sensor_sides, timeout=8.0)
        if not ready_sides:
            print(f"\n{RED}ERROR:{NC} No ultrasonic data received on any topic.")
            print(f"  Is pi_ultrasonic_dual_ros2.py running?")
            sys.exit(2)
        print(f" {GRN}OK{NC} ({', '.join(ready_sides)})")

        missing = [s for s in sensor_sides if s not in ready_sides]
        if missing:
            print(f"  {YLW}WARN:{NC} No data from: {', '.join(missing)} — using {', '.join(ready_sides)} only")
            sensor_sides = ready_sides

        for side in sensor_sides:
            val = node.us_read_mm(side)
            if val is not None:
                print(f"  USS-{side} initial reading: {val:.1f} mm ({val/10:.1f} cm)")
            else:
                print(f"  USS-{side}: waiting for valid range …")

        # --- Run calibration passes ---
        corrections: list[float] = []
        both_sides = ("left" in sensor_sides and "right" in sensor_sides)

        for p in range(1, n_passes + 1):
            print(f"\n{BLD}── Pass {p}/{n_passes} ──{NC}")

            # 0. Align perpendicular to wall (requires both sensors)
            if both_sides:
                align_to_wall(node, sensor_sep=args.sensor_sep)

            # 1. Read initial distances and verify proximity
            print(f"  Reading initial distances …")
            d_check = read_all_uss(sensor_sides, node)
            valid_sensors = [s for s in sensor_sides if d_check[s] is not None]
            if not valid_sensors:
                print(f"  {YLW}No valid USS readings, skipping pass{NC}")
                continue

            for s in valid_sensors:
                print(f"    USS-{s}: {d_check[s]:.1f} mm")

            min_dist = min(d_check[s] for s in valid_sensors)
            if min_dist < wall_stop_mm + target_dist_m * 1000:
                print(f"  {YLW}Too close to wall ({min_dist:.0f} mm) for "
                      f"{target_dist_m*1000:.0f} mm travel.  Reversing first …{NC}")
                # Auto-reverse to make room
                need_mm = (wall_stop_mm + target_dist_m * 1000 + 50) - min_dist
                drive_closed_loop(node, sensor_sides, need_mm, speed,
                                  wall_stop_mm, direction="reverse")
                if both_sides:
                    align_to_wall(node, sensor_sep=args.sensor_sep)

            # 2. Forward pass — closed-loop USS drive
            odom_delta, d_before, d_after = drive_closed_loop(
                node, sensor_sides, target_dist_m * 1000.0, speed,
                wall_stop_mm, direction="forward")

            # 3. Compute correction factors
            pass_corrections = []
            for s in valid_sensors:
                if d_before[s] is None or d_after[s] is None:
                    print(f"    USS-{s}: no valid reading — skip")
                    continue
                uss_delta_mm = d_before[s] - d_after[s]
                uss_delta_m = uss_delta_mm / 1000.0

                print(f"    USS-{s}: before={d_before[s]:.1f} mm  "
                      f"after={d_after[s]:.1f} mm  "
                      f"delta={uss_delta_mm:.1f} mm ({uss_delta_m*100:.2f} cm)")

                if uss_delta_mm < 5:
                    print(f"    {YLW}USS delta too small ({uss_delta_mm:.1f} mm) — skip{NC}")
                    continue

                ratio = uss_delta_m / odom_delta if odom_delta > 0.001 else 0
                pass_corrections.append(ratio)

            if pass_corrections:
                avg_ratio = sum(pass_corrections) / len(pass_corrections)
                print(f"  Odometry delta:  {odom_delta*100:.2f} cm")
                print(f"  Correction ratio: {avg_ratio:.4f}  "
                      f"({'over' if avg_ratio < 1 else 'under'}-reporting by "
                      f"{abs(1 - avg_ratio) * 100:.1f}%)")
                corrections.append(avg_ratio)
            else:
                print(f"  {YLW}No valid corrections this pass{NC}")

            # 4. Auto-reverse back to start position
            if p < n_passes:
                print(f"\n  Auto-reversing to start position …")
                if both_sides:
                    align_to_wall(node, sensor_sep=args.sensor_sep)
                drive_closed_loop(node, sensor_sides,
                                  target_dist_m * 1000.0, speed,
                                  wall_stop_mm, direction="reverse")
                if both_sides:
                    align_to_wall(node, sensor_sep=args.sensor_sep)

        # --- Aggregate results ---
        if not corrections:
            print(f"\n{RED}No valid correction factors obtained.{NC}")
            return

        # Remove outliers  
        corrections.sort()
        if len(corrections) > 2:
            # Trim top/bottom
            corrections = corrections[1:-1]

        avg_correction = sum(corrections) / len(corrections)
        std_correction = (sum((c - avg_correction) ** 2 for c in corrections) /
                          len(corrections)) ** 0.5 if len(corrections) > 1 else 0.0

        new_radius = current_radius * avg_correction

        # Clamp to physical bounds
        if new_radius < WHEEL_RADIUS_MIN or new_radius > WHEEL_RADIUS_MAX:
            print(f"\n  {RED}Computed radius {new_radius:.6f} m is outside valid range "
                  f"[{WHEEL_RADIUS_MIN:.3f}, {WHEEL_RADIUS_MAX:.3f}].{NC}")
            print(f"  This likely indicates bad sensor data. Aborting.")
            return

        print(f"\n{BLD}=== Results ==={NC}")
        print(f"  Passes used:       {len(corrections)}")
        print(f"  Correction factor: {avg_correction:.4f}  (σ={std_correction:.4f})")
        print(f"  Current radius:    {current_radius:.6f} m")
        print(f"  New radius:        {new_radius:.6f} m")

        # Sanity check
        pct_change = abs(avg_correction - 1.0)
        if pct_change > MAX_CORRECTION_PCT:
            print(f"\n  {RED}Correction too large ({pct_change*100:.1f}% > "
                  f"{MAX_CORRECTION_PCT*100:.0f}%).{NC}")
            print(f"  This may indicate bad sensor readings or robot wasn't "
                  f"perpendicular to wall.")
            print(f"  Skipping auto-apply.  Re-run with --apply to force.\n")
            if not args.force:
                return

        if apply:
            print(f"\n  Applying WHEEL_RADIUS = {new_radius:.6f} …")

            if dxl is not None:
                # Runtime
                dxl.calib_set(CALIB_KEY_WHEEL_RADIUS, new_radius)
                print(f"    {GRN}✓{NC} Runtime updated")

                # Flash
                dxl.calib_save()
                print(f"    {GRN}✓{NC} Saved to flash")
            else:
                print(f"    {YLW}⚠{NC} No serial connection — skipping runtime/flash update")

            # Source
            try:
                write_define_float(MAIN_C, "WHEEL_RADIUS_DEFAULT", new_radius,
                                   fmt=".6f",
                                   comment=f"calibrated {time.strftime('%Y-%m-%d')}")
                print(f"    {GRN}✓{NC} Source updated: {MAIN_C}")
            except Exception as e:
                print(f"    {YLW}⚠{NC} Source update failed: {e}")

            print(f"\n  {GRN}Calibration applied!{NC}\n")
        else:
            print(f"\n  {YLW}Dry-run:{NC} no changes applied.")
            print(f"  Re-run with --apply to persist.\n")

    except KeyboardInterrupt:
        print(f"\n{YLW}Interrupted.{NC}")
        node.stop(0.3)
    finally:
        if dxl is not None:
            dxl.close()
        node.destroy_node()
        rclpy.shutdown()


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Ultrasonic-based wheel-radius calibration for TurtleBot3 (ROS topics)")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"serial port for calibration read/write (default: {DEFAULT_PORT})")
    parser.add_argument("--distance", type=float, default=DEFAULT_DISTANCE_M,
                        help=f"travel distance per pass in metres (default: {DEFAULT_DISTANCE_M})")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED,
                        help=f"forward speed in m/s (default: {DEFAULT_SPEED})")
    parser.add_argument("--passes", type=int, default=DEFAULT_PASSES,
                        help=f"number of calibration passes (default: {DEFAULT_PASSES})")
    parser.add_argument("--wall-stop", type=int, default=DEFAULT_WALL_STOP_MM,
                        help=f"minimum wall distance in mm (default: {DEFAULT_WALL_STOP_MM})")
    parser.add_argument("--sensor", default="left,right",
                        help="sensor(s) to use: 'left', 'right', or 'left,right' (default: left,right)")
    parser.add_argument("--sensor-sep", type=float, default=ALIGN_SENSOR_SEP_M,
                        help=f"lateral separation between sensors in metres (default: {ALIGN_SENSOR_SEP_M})")
    parser.add_argument("--apply", action="store_true",
                        help="apply correction to runtime + flash + source")
    parser.add_argument("--force", action="store_true",
                        help="apply even if correction exceeds safety limit")
    parser.add_argument("--monitor", action="store_true",
                        help="live-print ultrasonic readings (no calibration)")
    args = parser.parse_args()

    sensor_sides = [s.strip() for s in args.sensor.split(",")]

    if args.monitor:
        rclpy.init()
        node = DistanceCalibNode(sensor_sides)
        try:
            run_monitor(sensor_sides, node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        run_calibration(args)


if __name__ == "__main__":
    main()
