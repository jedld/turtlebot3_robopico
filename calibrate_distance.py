#!/usr/bin/env python3
"""
calibrate_distance.py — Ultrasonic-based wheel-radius calibration for TurtleBot3.

Uses one or two front-facing Grove Ultrasonic Ranger V2.0 sensors (connected to
Grove 2 / Grove 3 on the Cytron Robo Pico) to obtain ground-truth forward
distance measurements.  The robot drives toward a flat wall, and the difference
between ultrasonic readings before and after the move gives the actual distance
traveled.  This is compared with ROS odometry to compute a wheel_radius
correction factor.

Workflow (per pass):
  1. Enable ultrasonic sensors via Dynamixel register write (ADDR_USS_ENABLE)
  2. Read initial ultrasonic distance(s)
  3. Record initial odometry position
  4. Drive forward at low speed until target distance is reached
  5. Stop and settle, read final ultrasonic distance(s)
  6. Compute: correction = uss_delta / odom_delta
  7. Adjust WHEEL_RADIUS ← current_radius × correction

After all passes, the averaged correction is applied via the runtime
calibration instruction (0x90 SET) and optionally persisted to flash and
source files.

Safety:
  • Stops immediately if ultrasonic distance < WALL_STOP_MM (200 mm default)
  • Wall proximity check before each pass
  • Sanity limits on correction factor (reject if > ±30 %)

Requirements:
  • ROS 2 bringup running (/odom topic active)
  • Firmware with ultrasonic support (ADDR_USS_ENABLE register)
  • At least one Grove Ultrasonic Ranger V2.0 connected

Usage:
  # Dry-run (measure and report only):
  python3 calibrate_distance.py

  # Apply result to runtime + flash + source:
  python3 calibrate_distance.py --apply

  # Custom parameters:
  python3 calibrate_distance.py --distance 0.15 --speed 0.04 --passes 5 --apply

  # Use only sensor 1:
  python3 calibrate_distance.py --sensor 1

  # Show live ultrasonic readings without calibrating:
  python3 calibrate_distance.py --monitor
"""

import argparse
import math
import os
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

# Dynamixel register addresses (must match firmware/main.c)
ADDR_USS_ENABLE     = 184   # 1 byte: bit0=USS1, bit1=USS2
ADDR_USS_STATUS     = 185   # 1 byte: bit0=USS1 valid, bit1=USS2 valid, bit4/5=timeout
ADDR_USS_1_DIST_MM  = 186   # uint16: sensor 1 distance (mm)
ADDR_USS_2_DIST_MM  = 188   # uint16: sensor 2 distance (mm)

# Runtime calibration (custom instruction 0x90)
INST_CALIBRATION = 0x90
CALIB_CMD_SET    = 0x01
CALIB_CMD_GET    = 0x02
CALIB_CMD_SAVE   = 0x04

CALIB_KEY_WHEEL_RADIUS = 0x01

USS_INVALID = 0xFFFF

# Defaults
DEFAULT_PORT         = "/dev/ttyACM0"
DEFAULT_DISTANCE_M   = 0.15      # target travel per pass (m)
DEFAULT_SPEED        = 0.04      # forward speed (m/s)
DEFAULT_PASSES       = 3
DEFAULT_WALL_STOP_MM = 200       # minimum allowed approach distance (mm)
MAX_CORRECTION_PCT   = 0.30      # reject if correction > ±30 %
SETTLE_TIME          = 1.0       # seconds to wait after stopping motors
USS_READ_SAMPLES     = 10        # number of USS readings to average per measurement
USS_READ_INTERVAL    = 0.12      # seconds between USS readings

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

    def uss_enable(self, mask: int) -> bool:
        """Enable ultrasonic sensors (bit 0 = USS1, bit 1 = USS2)."""
        return self.write_reg(ADDR_USS_ENABLE, bytes([mask & 0x03]))

    def uss_disable(self) -> bool:
        return self.write_reg(ADDR_USS_ENABLE, bytes([0x00]))

    def uss_read_mm(self, sensor: int) -> Optional[int]:
        """Read one ultrasonic sensor distance in mm.  Returns None on timeout."""
        addr = ADDR_USS_1_DIST_MM if sensor == 1 else ADDR_USS_2_DIST_MM
        data = self.read_reg(addr, 2)
        if data is None:
            return None
        val = struct.unpack('<H', data)[0]
        return None if val == USS_INVALID else val

    def uss_status(self) -> Optional[int]:
        data = self.read_reg(ADDR_USS_STATUS, 1)
        return data[0] if data else None


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
    """Subscribes to /odom and publishes /cmd_vel for distance calibration."""

    def __init__(self):
        super().__init__("calibrate_distance")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_ok = False
        self.create_subscription(
            Odometry, "/odom", self._odom_cb, SENSOR_QOS)

    def _odom_cb(self, msg: Odometry):
        self._odom_ok = True
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y

    def wait_for_odom(self, timeout: float = 8.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._odom_ok:
                return True
        return False

    def get_odom_xy(self) -> tuple[float, float]:
        rclpy.spin_once(self, timeout_sec=0.05)
        return (self._odom_x, self._odom_y)

    def drive(self, speed: float) -> None:
        msg = Twist()
        msg.linear.x = speed
        self.pub.publish(msg)

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

def read_uss_averaged(dxl: DxlPort, sensor: int, node: DistanceCalibNode,
                      num_samples: int = USS_READ_SAMPLES,
                      interval: float = USS_READ_INTERVAL) -> Optional[float]:
    """Read `num_samples` ultrasonic readings, discard outliers, return mean mm."""
    readings: list[int] = []
    for _ in range(num_samples):
        node.spin_ros(interval)
        val = dxl.uss_read_mm(sensor)
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


def read_both_uss(dxl: DxlPort, sensors: list[int], node: DistanceCalibNode,
                  num_samples: int = USS_READ_SAMPLES) -> dict[int, Optional[float]]:
    """Read averaging for each enabled sensor."""
    results = {}
    for s in sensors:
        results[s] = read_uss_averaged(dxl, s, node, num_samples)
    return results


# ── main calibration ─────────────────────────────────────────────────────────

def run_monitor(dxl: DxlPort, node: DistanceCalibNode, sensors: list[int]):
    """Continuously print ultrasonic readings (Ctrl-C to exit)."""
    enable_mask = sum(1 << (s - 1) for s in sensors)
    dxl.uss_enable(enable_mask)
    time.sleep(0.5)
    print(f"\n{BLD}Live ultrasonic monitor{NC}  (Ctrl-C to exit)\n")
    try:
        while True:
            parts = []
            for s in sensors:
                val = dxl.uss_read_mm(s)
                if val is not None:
                    parts.append(f"USS{s}: {val:5d} mm ({val/10:.1f} cm)")
                else:
                    parts.append(f"USS{s}: ----  (timeout)")
            status = dxl.uss_status()
            print(f"  {' | '.join(parts)}  [status=0x{status:02X}]" if status is not None
                  else f"  {' | '.join(parts)}", end="\r")
            node.spin_ros(0.15)
    except KeyboardInterrupt:
        print("\n")
    finally:
        dxl.uss_disable()


def run_calibration(args):
    """Execute the ultrasonic distance calibration procedure."""
    port_path = args.port
    target_dist_m = args.distance
    speed = args.speed
    n_passes = args.passes
    wall_stop_mm = args.wall_stop
    apply = args.apply
    sensors = [int(s) for s in args.sensor.split(",")]

    print(f"\n{BLD}=== Ultrasonic Distance Calibration ==={NC}")
    print(f"  Port:       {port_path}")
    print(f"  Sensors:    {sensors}")
    print(f"  Target:     {target_dist_m:.3f} m per pass")
    print(f"  Speed:      {speed:.3f} m/s")
    print(f"  Passes:     {n_passes}")
    print(f"  Wall stop:  {wall_stop_mm} mm")
    print(f"  Apply:      {'yes' if apply else 'dry-run'}\n")

    # --- Open Dynamixel port ---
    try:
        dxl = DxlPort(port_path)
    except serial.SerialException as e:
        print(f"{RED}ERROR:{NC} Cannot open {port_path}: {e}")
        print("  Is the Pico connected?  Is bringup NOT running?")
        print("  (This script needs exclusive serial access.)")
        sys.exit(2)

    # --- Init ROS ---
    rclpy.init()
    node = DistanceCalibNode()

    try:
        # Wait for odometry
        print(f"  Waiting for /odom topic …", end="", flush=True)
        if not node.wait_for_odom(10.0):
            print(f"\n{RED}ERROR:{NC} No /odom messages received.  Is bringup running?")
            sys.exit(2)
        print(f" {GRN}OK{NC}")

        # --- Read current wheel_radius ---
        current_radius = dxl.calib_get(CALIB_KEY_WHEEL_RADIUS)
        if current_radius is None:
            # Fallback: read from source
            current_radius = read_define_float(MAIN_C, "WHEEL_RADIUS_DEFAULT")
        if current_radius is None:
            current_radius = 0.03405  # TurtleBot3 Burger default
        print(f"  Current WHEEL_RADIUS = {current_radius:.6f} m")

        # --- Enable ultrasonic sensors ---
        enable_mask = sum(1 << (s - 1) for s in sensors)
        if not dxl.uss_enable(enable_mask):
            print(f"{YLW}WARN:{NC} USS enable write did not get ACK (may still work)")
        time.sleep(1.0)  # wait for first readings to populate

        # Verify sensors are responding
        for s in sensors:
            val = dxl.uss_read_mm(s)
            if val is None:
                print(f"{RED}ERROR:{NC} Ultrasonic sensor {s} returns no reading.")
                print(f"  Check wiring: sensor {s} → Grove {'2' if s == 1 else '3'} "
                      f"on Robo Pico")
                sys.exit(2)
            print(f"  USS{s} initial reading: {val} mm ({val/10:.1f} cm)")

        # --- Run calibration passes ---
        corrections: list[float] = []

        for p in range(1, n_passes + 1):
            print(f"\n{BLD}── Pass {p}/{n_passes} ──{NC}")

            # 1. Read initial ultrasonic distance
            print(f"  Reading initial distances …")
            d_before = read_both_uss(dxl, sensors, node)
            for s in sensors:
                if d_before[s] is None:
                    print(f"  {RED}USS{s}: no valid reading — skip pass{NC}")
                    continue
                print(f"    USS{s} before: {d_before[s]:.1f} mm")

            # Check all sensors got readings
            valid_sensors = [s for s in sensors if d_before[s] is not None]
            if not valid_sensors:
                print(f"  {YLW}No valid USS readings, skipping pass{NC}")
                continue

            # Wall proximity check
            min_dist = min(d_before[s] for s in valid_sensors)
            if min_dist < wall_stop_mm + target_dist_m * 1000:
                print(f"  {YLW}Too close to wall ({min_dist:.0f} mm) for "
                      f"{target_dist_m*1000:.0f} mm travel.  Move robot back.{NC}")
                input(f"  Press Enter when ready, or Ctrl-C to abort … ")
                # Re-read
                d_before = read_both_uss(dxl, sensors, node)
                valid_sensors = [s for s in sensors if d_before[s] is not None]

            # 2. Record initial odometry
            odom_x0, odom_y0 = node.get_odom_xy()

            # 3. Drive forward
            print(f"  Driving forward at {speed:.3f} m/s …", end="", flush=True)
            driven = 0.0
            t_start = time.time()
            max_time = target_dist_m / speed * 3.0  # safety timeout

            while driven < target_dist_m:
                node.drive(speed)
                rclpy.spin_once(node, timeout_sec=0.03)

                ox, oy = node.get_odom_xy()
                driven = math.sqrt((ox - odom_x0) ** 2 + (oy - odom_y0) ** 2)

                # Safety: check ultrasonic distance during drive
                for s in valid_sensors:
                    d_now = dxl.uss_read_mm(s)
                    if d_now is not None and d_now < wall_stop_mm:
                        node.stop(0.5)
                        print(f"\n  {RED}WALL STOP:{NC} USS{s}={d_now} mm < "
                              f"{wall_stop_mm} mm")
                        break

                if time.time() - t_start > max_time:
                    node.stop(0.5)
                    print(f"\n  {YLW}Timeout{NC}")
                    break

            node.stop(SETTLE_TIME)
            print(f" done")

            # 4. Record final odometry
            odom_x1, odom_y1 = node.get_odom_xy()
            odom_delta = math.sqrt((odom_x1 - odom_x0) ** 2 +
                                   (odom_y1 - odom_y0) ** 2)

            # 5. Read final ultrasonic distance
            print(f"  Reading final distances …")
            d_after = read_both_uss(dxl, sensors, node)

            pass_corrections = []
            for s in valid_sensors:
                if d_after[s] is None:
                    print(f"    USS{s}: no valid reading after move")
                    continue
                uss_delta_mm = d_before[s] - d_after[s]
                uss_delta_m = uss_delta_mm / 1000.0

                print(f"    USS{s}: before={d_before[s]:.1f} mm  "
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
                print(f"  USS delta (avg): {sum(d_before[s] - (d_after[s] or 0) for s in valid_sensors if d_after[s] is not None) / len(pass_corrections) / 10:.2f} cm")
                print(f"  Correction ratio: {avg_ratio:.4f}  "
                      f"({'over' if avg_ratio < 1 else 'under'}-reporting by "
                      f"{abs(1 - avg_ratio) * 100:.1f}%)")
                corrections.append(avg_ratio)
            else:
                print(f"  {YLW}No valid corrections this pass{NC}")

            # Pause between passes
            if p < n_passes:
                print(f"\n  Returning to start position … (drive robot back manually)")
                input(f"  Press Enter when ready for next pass … ")

        # --- Aggregate results ---
        if not corrections:
            print(f"\n{RED}No valid correction factors obtained.{NC}")
            dxl.uss_disable()
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
                dxl.uss_disable()
                return

        if apply:
            print(f"\n  Applying WHEEL_RADIUS = {new_radius:.6f} …")

            # Runtime
            dxl.calib_set(CALIB_KEY_WHEEL_RADIUS, new_radius)
            print(f"    {GRN}✓{NC} Runtime updated")

            # Flash
            dxl.calib_save()
            print(f"    {GRN}✓{NC} Saved to flash")

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
        dxl.uss_disable()
        dxl.close()
        node.destroy_node()
        rclpy.shutdown()


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Ultrasonic-based wheel-radius calibration for TurtleBot3")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--distance", type=float, default=DEFAULT_DISTANCE_M,
                        help=f"travel distance per pass in metres (default: {DEFAULT_DISTANCE_M})")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED,
                        help=f"forward speed in m/s (default: {DEFAULT_SPEED})")
    parser.add_argument("--passes", type=int, default=DEFAULT_PASSES,
                        help=f"number of calibration passes (default: {DEFAULT_PASSES})")
    parser.add_argument("--wall-stop", type=int, default=DEFAULT_WALL_STOP_MM,
                        help=f"minimum wall distance in mm (default: {DEFAULT_WALL_STOP_MM})")
    parser.add_argument("--sensor", default="1,2",
                        help="sensor(s) to use: '1', '2', or '1,2' (default: 1,2)")
    parser.add_argument("--apply", action="store_true",
                        help="apply correction to runtime + flash + source")
    parser.add_argument("--force", action="store_true",
                        help="apply even if correction exceeds safety limit")
    parser.add_argument("--monitor", action="store_true",
                        help="live-print ultrasonic readings (no calibration)")
    args = parser.parse_args()

    if args.monitor:
        rclpy.init()
        node = DistanceCalibNode()
        sensors = [int(s) for s in args.sensor.split(",")]
        try:
            dxl = DxlPort(args.port)
        except serial.SerialException as e:
            print(f"{RED}ERROR:{NC} Cannot open {args.port}: {e}")
            sys.exit(2)
        try:
            run_monitor(dxl, node, sensors)
        finally:
            dxl.uss_disable()
            dxl.close()
            node.destroy_node()
            rclpy.shutdown()
    else:
        run_calibration(args)


if __name__ == "__main__":
    main()
