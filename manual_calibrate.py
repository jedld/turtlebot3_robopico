#!/usr/bin/env python3
"""
manual_calibrate.py — Interactive motor & wheel calibration for TurtleBot3 Pico.

Provides a live TUI for:
  • Viewing all runtime calibration values
  • Driving the robot manually (both motors, or left/right individually)
  • Adjusting any calibration parameter in real-time
  • Monitoring IMU gyro-Z and (optionally) odometry while driving
  • Saving / loading / resetting calibration to firmware flash

Requires: ROS 2 running (for IMU readout). Motor commands go directly
over serial — no turtlebot3 bringup node needed.

Usage:
  python3 manual_calibrate.py [--port /dev/ttyTB3] [--speed 0.08]
"""

import argparse
import math
import os
import struct
import sys
import termios
import time
import tty
from threading import Lock
from typing import Optional

import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu

# ── Dynamixel protocol constants ──────────────────────────────────────────────

DEV_ID = 200
DXL_BAUD = 1_000_000
INST_CALIBRATION = 0x90

CALIB_CMD_SET  = 0x01
CALIB_CMD_GET  = 0x02
CALIB_CMD_RESET = 0x03
CALIB_CMD_SAVE = 0x04
CALIB_CMD_LOAD = 0x05
CALIB_CMD_RESET_AND_SAVE = 0x06

# All calibration keys supported by firmware
CALIB_KEYS = {
    "wheel_radius":         0x01,
    "wheel_separation":     0x02,
    "max_wheel_speed_ms":   0x03,
    "motor_min_duty":       0x04,   # sets BOTH left & right
    "motor_kick_duty":      0x05,
    "motor_kick_cycles":    0x06,
    "motor_trim_left":      0x07,
    "motor_trim_right":     0x08,
    "right_motor_reversed": 0x09,
    "swap_motors":          0x0A,
    "motor_min_duty_left":  0x0B,
    "motor_min_duty_right": 0x0C,
    "left_motor_reversed":  0x0D,
}

# Readable labels for display
CALIB_LABELS = {
    "wheel_radius":         "Wheel radius (m)",
    "wheel_separation":     "Wheel separation (m)",
    "max_wheel_speed_ms":   "Max wheel speed (m/s)",
    "motor_min_duty":       "Motor min duty (both)",
    "motor_kick_duty":      "Motor kick duty",
    "motor_kick_cycles":    "Motor kick cycles",
    "motor_trim_left":      "Motor trim left",
    "motor_trim_right":     "Motor trim right",
    "right_motor_reversed": "Right motor reversed",
    "swap_motors":          "Swap L/R motors",
    "motor_min_duty_left":  "Motor min duty LEFT",
    "motor_min_duty_right": "Motor min duty RIGHT",
    "left_motor_reversed":  "Left motor reversed",
}

# Sensible step sizes for interactive adjustment
CALIB_STEPS = {
    "wheel_radius":         0.0005,
    "wheel_separation":     0.001,
    "max_wheel_speed_ms":   0.005,
    "motor_min_duty":       0.02,
    "motor_kick_duty":      0.02,
    "motor_kick_cycles":    1.0,
    "motor_trim_left":      0.01,
    "motor_trim_right":     0.01,
    "right_motor_reversed": 1.0,
    "swap_motors":          1.0,
    "motor_min_duty_left":  0.02,
    "motor_min_duty_right": 0.02,
    "left_motor_reversed":  1.0,
}

# QoS matching TurtleBot3 sensor topics
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ── CRC & Dynamixel helpers ──────────────────────────────────────────────────

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


def _build_instruction(dev_id: int, inst: int, params: bytes) -> bytes:
    pkt_len = len(params) + 3
    hdr = bytes([
        0xFF, 0xFF, 0xFD, 0x00,
        dev_id,
        pkt_len & 0xFF, (pkt_len >> 8) & 0xFF,
        inst,
    ])
    pkt = hdr + params
    crc = _crc16(pkt)
    return pkt + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _read_status_packet(ser: serial.Serial, timeout: float = 0.6) -> Optional[bytes]:
    end = time.time() + timeout
    buf = b""
    while time.time() < end:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk
        idx = buf.find(b"\xFF\xFF\xFD\x00")
        if idx < 0:
            if len(buf) > 4096:
                buf = buf[-64:]
            continue
        if idx > 0:
            buf = buf[idx:]
        if len(buf) < 7:
            continue
        pkt_len = buf[5] | (buf[6] << 8)
        total = 7 + pkt_len
        if len(buf) < total:
            continue
        pkt = buf[:total]
        crc_rx = pkt[-2] | (pkt[-1] << 8)
        if _crc16(pkt[:-2]) != crc_rx:
            buf = buf[1:]
            continue
        if pkt[7] != 0x55:
            buf = buf[total:]
            continue
        return pkt
    return None


def send_calib(port: str, subcmd: int, key: Optional[int] = None,
               value: Optional[float] = None, expect_reply: bool = True
               ) -> Optional[bytes]:
    """Send a calibration command and optionally wait for a status reply."""
    params = bytes([subcmd])
    if subcmd == CALIB_CMD_SET:
        if key is None or value is None:
            raise ValueError("SET requires key and value")
        params += bytes([key]) + struct.pack("<f", float(value))

    pkt = _build_instruction(DEV_ID, INST_CALIBRATION, params)
    with serial.Serial(port, DXL_BAUD, timeout=0.05, write_timeout=0.2) as ser:
        ser.reset_input_buffer()
        ser.write(pkt)
        ser.flush()
        if expect_reply:
            return _read_status_packet(ser)
    return None


def send_calib_broadcast(port: str, subcmd: int, key: Optional[int] = None,
                         value: Optional[float] = None) -> None:
    """Send a calibration command via broadcast (no reply expected).
    Useful when the serial bus is shared with the turtlebot3 node."""
    params = bytes([subcmd])
    if subcmd == CALIB_CMD_SET:
        if key is None or value is None:
            raise ValueError("SET requires key and value")
        params += bytes([key]) + struct.pack("<f", float(value))

    pkt = _build_instruction(0xFE, INST_CALIBRATION, params)
    with serial.Serial(port, DXL_BAUD, timeout=0.02, write_timeout=0.2) as ser:
        ser.write(pkt)
        ser.flush()
    time.sleep(0.02)


def get_calibration_values(port: str) -> Optional[dict]:
    """Read all runtime calibration values from firmware."""
    pkt = send_calib(port, CALIB_CMD_GET)
    if pkt is None:
        return None
    err = pkt[8]
    if err != 0:
        return None
    payload = pkt[9:-2]
    if len(payload) < 37 or payload[0] != CALIB_CMD_GET:
        return None
    return {
        "wheel_radius":         struct.unpack_from("<f", payload,  1)[0],
        "wheel_separation":     struct.unpack_from("<f", payload,  5)[0],
        "max_wheel_speed_ms":   struct.unpack_from("<f", payload,  9)[0],
        "right_motor_reversed": int(payload[13]),
        "swap_motors":          int(payload[14]),
        "motor_min_duty_left":  struct.unpack_from("<f", payload, 15)[0],
        "motor_kick_duty":      struct.unpack_from("<f", payload, 19)[0],
        "motor_kick_cycles":    int(payload[23]),
        "motor_trim_left":      struct.unpack_from("<f", payload, 24)[0],
        "motor_trim_right":     struct.unpack_from("<f", payload, 28)[0],
        "motor_min_duty_right": struct.unpack_from("<f", payload, 32)[0],
        "left_motor_reversed":  int(payload[36]),
    }


# ── Persistent serial bus for real-time motor control ─────────────────────────

# Firmware register addresses for velocity commands (Dynamixel WRITE 0x03)
ADDR_CMD_LINEAR_X  = 150   # int32, units of 0.01 m/s
ADDR_CMD_ANGULAR_Z = 170   # int32, units of 0.01 rad/s
INST_WRITE = 0x03


class DxlBus:
    """Persistent Dynamixel serial connection for low-latency motor control.

    Sends velocity commands via WRITE instruction directly to firmware
    registers — no ROS bringup node required.
    """

    def __init__(self, port: str, baud: int = DXL_BAUD, dev_id: int = DEV_ID):
        self.port = port
        self.dev_id = dev_id
        self._ser = serial.Serial(port, baud, timeout=0.02, write_timeout=0.2)

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()

    def reopen(self):
        """Reopen the serial port after a temporary close."""
        if not self._ser.is_open:
            self._ser.open()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def _send_raw(self, pkt: bytes):
        """Write a raw Dynamixel packet."""
        self._ser.write(pkt)
        self._ser.flush()

    def write_velocity(self, linear_x: float = 0.0, angular_z: float = 0.0):
        """Send a velocity command directly to firmware registers.

        Uses broadcast ID (0xFE) so no status reply is expected and there's
        no blocking read — keeps the loop fast.
        The firmware expects int32 in units of 0.01 m/s and 0.01 rad/s,
        written to ADDR_CMD_LINEAR_X (150) and ADDR_CMD_ANGULAR_Z (170).
        We write both in a single 24-byte WRITE spanning addr 150..173
        (matching what the turtlebot3 ROS node does).
        """
        lin_i = int(round(linear_x * 100.0))
        ang_i = int(round(angular_z * 100.0))
        # Build register data: addr 150..173 (24 bytes)
        # Bytes 0-3:  LINEAR_X (int32 LE) at addr 150
        # Bytes 4-19: padding (zeros, covers addrs 154-169)
        # Bytes 20-23: ANGULAR_Z (int32 LE) at addr 170
        reg_data = struct.pack("<i", lin_i)
        reg_data += b"\x00" * 16   # addrs 154-169 (unused, keep existing)
        reg_data += struct.pack("<i", ang_i)
        # WRITE instruction params: [addr_lo, addr_hi, data...]
        params = struct.pack("<H", ADDR_CMD_LINEAR_X) + reg_data
        pkt = _build_instruction(0xFE, INST_WRITE, params)
        self._send_raw(pkt)

    def stop_motors(self):
        """Send zero velocity a few times to ensure motors stop."""
        for _ in range(3):
            self.write_velocity(0.0, 0.0)
            time.sleep(0.02)

    def send_calib_set(self, key: int, value: float):
        """Send a calibration SET via broadcast (no reply wait)."""
        params = bytes([CALIB_CMD_SET, key]) + struct.pack("<f", float(value))
        pkt = _build_instruction(0xFE, INST_CALIBRATION, params)
        self._send_raw(pkt)
        time.sleep(0.015)  # small delay for firmware to process


# ── ROS node ──────────────────────────────────────────────────────────────────

class ManualCalibNode(Node):
    """Minimal ROS node: subscribes /imu for live gyro readout."""

    def __init__(self):
        super().__init__("manual_calibrate")
        self.sub_imu = self.create_subscription(
            Imu, "/imu", self._imu_cb, SENSOR_QOS)

        self._lock = Lock()
        self._gyro_z = 0.0
        self._imu_ts = 0.0
        self._imu_ok = False

    def _imu_cb(self, msg: Imu):
        with self._lock:
            self._gyro_z = msg.angular_velocity.z
            self._imu_ts = time.time()
            self._imu_ok = True

    @property
    def gyro_z(self) -> float:
        with self._lock:
            return self._gyro_z

    @property
    def imu_ok(self) -> bool:
        with self._lock:
            return self._imu_ok and (time.time() - self._imu_ts) < 2.0


# ── Terminal raw-key helpers ──────────────────────────────────────────────────

def _getch_raw(fd, timeout: float = 0.1) -> str:
    """Read a single keypress from raw terminal. Returns '' on timeout."""
    import select
    rlist, _, _ = select.select([fd], [], [], timeout)
    if rlist:
        ch = os.read(fd, 8).decode("utf-8", errors="ignore")
        return ch
    return ""


class RawTerminal:
    """Context manager for raw (unbuffered, no-echo) terminal mode."""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, *args):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def getkey(self, timeout: float = 0.1) -> str:
        return _getch_raw(self.fd, timeout)


# ── Formatting helpers ────────────────────────────────────────────────────────

BOLD  = "\033[1m"
DIM   = "\033[2m"
GREEN = "\033[32m"
CYAN  = "\033[36m"
YELLOW = "\033[33m"
RED   = "\033[31m"
RESET = "\033[0m"
CLEAR = "\033[2J\033[H"


def fmt_val(name: str, val) -> str:
    """Format a calibration value for display."""
    if isinstance(val, float):
        return f"{val:.6f}"
    return str(val)


# ── Drive modes ───────────────────────────────────────────────────────────────

def drive_mode(node: ManualCalibNode, bus: DxlBus, port: str,
               speed: float, ang_speed: float, mode: str = "both"):
    """
    Interactive keyboard-driven motor test.

    mode: "both"  — normal teleop (WASD)
          "left"  — only left motor: pivot CW (left-fwd) / CCW (left-bwd)
          "right" — only right motor: pivot CCW (right-fwd) / CW (right-bwd)
    """
    mode_labels = {"both": "Both Motors", "left": "Left Motor Only",
                   "right": "Right Motor Only"}
    label = mode_labels.get(mode, mode)

    # For single-motor driving, set the OTHER motor's min_duty to 0
    # so dead-zone compensation doesn't activate it.
    restore_cmds: list[tuple[int, float]] = []
    if mode == "left":
        # Disable right motor dead-zone — raw throttle will be ~0 so it stays still
        bus.close()
        vals = get_calibration_values(port)
        if vals:
            restore_cmds.append((CALIB_KEYS["motor_min_duty_right"], vals["motor_min_duty_right"]))
        send_calib(port, CALIB_CMD_SET, CALIB_KEYS["motor_min_duty_right"], 0.0)
        bus.reopen()
        time.sleep(0.03)
    elif mode == "right":
        bus.close()
        vals = get_calibration_values(port)
        if vals:
            restore_cmds.append((CALIB_KEYS["motor_min_duty_left"], vals["motor_min_duty_left"]))
        send_calib(port, CALIB_CMD_SET, CALIB_KEYS["motor_min_duty_left"], 0.0)
        bus.reopen()
        time.sleep(0.03)

    print(CLEAR, end="")
    print(f"{BOLD}═══ Drive Mode: {label} ═══{RESET}\n")
    print(f"  {BOLD}W{RESET} / {BOLD}S{RESET}  — forward / backward   (speed: {speed:.3f} m/s)")
    print(f"  {BOLD}A{RESET} / {BOLD}D{RESET}  — turn left / right    (speed: {ang_speed:.2f} rad/s)")
    print(f"  {BOLD}+{RESET} / {BOLD}-{RESET}  — increase / decrease speed")
    print(f"  {BOLD}Space{RESET}  — emergency stop")
    print(f"  {BOLD}Q{RESET}      — quit drive mode\n")
    if mode != "both":
        print(f"  {YELLOW}Single-motor mode:{RESET} other motor's MIN_DUTY set to 0\n")
    print(f"  {DIM}IMU gyro-Z shown live below{RESET}\n")

    lin_x = 0.0
    ang_z = 0.0
    running = True

    with RawTerminal() as term:
        while running:
            rclpy.spin_once(node, timeout_sec=0.01)
            key = term.getkey(timeout=0.05)

            moved = False
            if key == "w":
                if mode == "both":
                    lin_x = speed
                    ang_z = 0.0
                else:
                    # Single motor: pivot so that motor goes forward
                    lin_x = 0.0
                    ang_z = -ang_speed if mode == "left" else ang_speed
                moved = True
            elif key == "s":
                if mode == "both":
                    lin_x = -speed
                    ang_z = 0.0
                else:
                    lin_x = 0.0
                    ang_z = ang_speed if mode == "left" else -ang_speed
                moved = True
            elif key == "a":
                if mode == "both":
                    lin_x = 0.0
                    ang_z = ang_speed
                else:
                    lin_x = 0.0
                    ang_z = ang_speed if mode == "left" else -ang_speed
                moved = True
            elif key == "d":
                if mode == "both":
                    lin_x = 0.0
                    ang_z = -ang_speed
                else:
                    lin_x = 0.0
                    ang_z = -ang_speed if mode == "left" else ang_speed
                moved = True
            elif key == " ":
                lin_x = 0.0
                ang_z = 0.0
                moved = True
            elif key == "+":
                speed = min(speed + 0.01, 0.50)
                ang_speed = min(ang_speed + 0.1, 3.0)
            elif key == "-":
                speed = max(speed - 0.01, 0.01)
                ang_speed = max(ang_speed - 0.1, 0.1)
            elif key in ("q", "Q", "\x03"):  # q or Ctrl-C
                running = False
                lin_x = 0.0
                ang_z = 0.0
            elif key == "":
                # Timeout — no key pressed, keep sending last command
                pass
            else:
                # Unknown key — ignore, keep current command
                pass

            bus.write_velocity(lin_x, ang_z)

            # Print status line (overwrite)
            imu_str = f"{node.gyro_z:+.3f} rad/s" if node.imu_ok else "no IMU"
            dir_str = "STOP"
            if lin_x > 0:
                dir_str = "FWD "
            elif lin_x < 0:
                dir_str = "BWD "
            elif ang_z > 0:
                dir_str = "LEFT"
            elif ang_z < 0:
                dir_str = "RGHT"

            sys.stdout.write(
                f"\r  {BOLD}[{dir_str}]{RESET}  "
                f"lin={lin_x:+.3f}  ang={ang_z:+.2f}  "
                f"speed={speed:.3f}/{ang_speed:.2f}  "
                f"gyro-Z={imu_str}    "
            )
            sys.stdout.flush()

    bus.stop_motors()
    print("\n")

    # Restore other motor's min_duty
    for key_id, val in restore_cmds:
        bus.send_calib_set(key_id, val)
        time.sleep(0.02)


# ── Parameter adjustment ──────────────────────────────────────────────────────

def adjust_parameter(node: ManualCalibNode, bus: DxlBus, port: str, name: str,
                     current_val: float, speed: float = 0.08,
                     ang_speed: float = 0.5) -> Optional[float]:
    """
    Interactive parameter adjustment with +/- keys or direct value entry.
    W/S/A/D drive the motors so you can feel the effect of the change.
    Returns the new value, or None if cancelled.
    """
    step = CALIB_STEPS.get(name, 0.01)
    label = CALIB_LABELS.get(name, name)
    val = current_val
    lin_x = 0.0
    ang_z = 0.0

    print(f"\n  {BOLD}Adjusting: {label}{RESET}")
    print(f"  Current value: {val:.6f}")
    print(f"  Step size    : {step}")
    print(f"\n  {BOLD}+{RESET}/{BOLD}={RESET} — increase    {BOLD}-{RESET} — decrease")
    print(f"  {BOLD}]{RESET}   — 10× step     {BOLD}[{RESET} — 0.1× step")
    print(f"  {BOLD}V{RESET}   — enter value   {BOLD}Enter{RESET} — confirm")
    print(f"  {BOLD}Esc{RESET} — cancel")
    print(f"\n  {BOLD}W{RESET}/{BOLD}S{RESET} — drive fwd/bwd   {BOLD}A{RESET}/{BOLD}D{RESET} — turn left/right")
    print(f"  {BOLD}Space{RESET} — stop motors\n")

    with RawTerminal() as term:
        prev_val = val  # track changes to avoid unnecessary serial comms
        while True:
            rclpy.spin_once(node, timeout_sec=0.01)
            key = term.getkey(timeout=0.05)

            val_changed = False
            if key in ("+", "="):
                val += step
                val_changed = True
            elif key == "-":
                val -= step
                val_changed = True
            elif key == "]":
                val += step * 10
                val_changed = True
            elif key == "[":
                val -= step * 10
                val_changed = True
            elif key in ("\r", "\n"):
                # Confirm
                lin_x, ang_z = 0.0, 0.0
                bus.stop_motors()
                break
            elif key == "\x1b":
                # Esc — cancel
                bus.stop_motors()
                print(f"\r  {YELLOW}Cancelled.{RESET}              ")
                return None
            elif key in ("v", "V"):
                # Enter value manually — stop motors, switch to cooked mode
                bus.stop_motors()
                lin_x, ang_z = 0.0, 0.0
                termios.tcsetattr(term.fd, termios.TCSADRAIN, term.old_settings)
                try:
                    raw = input(f"\r  Enter value: ")
                    val = float(raw)
                    val_changed = True
                except (ValueError, EOFError):
                    print(f"  {RED}Invalid input.{RESET}")
                finally:
                    tty.setraw(term.fd)
            elif key == "w":
                lin_x, ang_z = speed, 0.0
            elif key == "s":
                lin_x, ang_z = -speed, 0.0
            elif key == "a":
                lin_x, ang_z = 0.0, ang_speed
            elif key == "d":
                lin_x, ang_z = 0.0, -ang_speed
            elif key == " ":
                lin_x, ang_z = 0.0, 0.0
            elif key in ("q", "Q", "\x03"):
                bus.stop_motors()
                print(f"\r  {YELLOW}Cancelled.{RESET}              ")
                return None
            elif key == "":
                # Timeout — keep current drive command
                pass
            else:
                # Unknown key — don't change drive
                pass

            # Publish velocity every iteration to keep motors alive
            bus.write_velocity(lin_x, ang_z)

            # Only send calibration command when value actually changed
            # Use broadcast to avoid blocking the loop
            if val_changed:
                bus.send_calib_set(CALIB_KEYS[name], val)
                prev_val = val

            # Status line with value + drive state + IMU
            imu_str = f"{node.gyro_z:+.3f}" if node.imu_ok else "--"
            dir_str = "STOP"
            if lin_x > 0:
                dir_str = "FWD "
            elif lin_x < 0:
                dir_str = "BWD "
            elif ang_z > 0:
                dir_str = "LEFT"
            elif ang_z < 0:
                dir_str = "RGHT"

            sys.stdout.write(
                f"\r  {BOLD}{label}{RESET}={GREEN}{val:.6f}{RESET}  "
                f"step={step:.4f}  "
                f"[{BOLD}{dir_str}{RESET}] "
                f"gyro={imu_str}     "
            )
            sys.stdout.flush()

    bus.stop_motors()
    print(f"\n  {GREEN}Confirmed: {val:.6f}{RESET}\n")
    return val


# ── Main menu ─────────────────────────────────────────────────────────────────

def print_values(vals: dict):
    """Pretty-print all calibration values."""
    print(f"\n  {BOLD}{'Parameter':<28s}  {'Value':>12s}{RESET}")
    print(f"  {'─' * 42}")

    # Order for display
    display_order = [
        "wheel_radius", "wheel_separation", "max_wheel_speed_ms",
        "",
        "motor_min_duty_left", "motor_min_duty_right",
        "motor_kick_duty", "motor_kick_cycles",
        "motor_trim_left", "motor_trim_right",
        "",
        "left_motor_reversed", "right_motor_reversed", "swap_motors",
    ]
    for name in display_order:
        if name == "":
            print()
            continue
        if name not in vals:
            continue
        label = CALIB_LABELS.get(name, name)
        v = vals[name]
        if isinstance(v, float):
            print(f"  {label:<28s}  {v:>12.6f}")
        else:
            print(f"  {label:<28s}  {v:>12d}")
    print()


def main_menu(node: ManualCalibNode, bus: DxlBus, port: str,
              speed: float, ang_speed: float):
    """Main interactive menu loop."""

    while True:
        print(f"{BOLD}{'═' * 56}{RESET}")
        print(f"{BOLD}   Manual Motor & Wheel Calibration{RESET}")
        print(f"{BOLD}{'═' * 56}{RESET}")
        print()
        print(f"  {BOLD}1{RESET}  Show current calibration values")
        print(f"  {BOLD}2{RESET}  Drive — both motors (teleop)")
        print(f"  {BOLD}3{RESET}  Drive — left motor only")
        print(f"  {BOLD}4{RESET}  Drive — right motor only")
        print(f"  {BOLD}5{RESET}  Adjust a calibration parameter")
        print()
        print(f"  {BOLD}S{RESET}  Save to flash")
        print(f"  {BOLD}L{RESET}  Load from flash")
        print(f"  {BOLD}R{RESET}  Reset to firmware defaults")
        print(f"  {BOLD}Q{RESET}  Quit")
        print()

        try:
            choice = input(f"  {CYAN}>{RESET} ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if choice == "1":
            bus.close()
            vals = get_calibration_values(port)
            bus.reopen()
            if vals is None:
                print(f"\n  {RED}ERROR: Could not read values from firmware.{RESET}\n")
            else:
                print_values(vals)
            _pause()

        elif choice == "2":
            drive_mode(node, bus, port, speed, ang_speed, mode="both")

        elif choice == "3":
            drive_mode(node, bus, port, speed, ang_speed, mode="left")

        elif choice == "4":
            drive_mode(node, bus, port, speed, ang_speed, mode="right")

        elif choice == "5":
            bus.close()
            vals = get_calibration_values(port)
            bus.reopen()
            if vals is None:
                print(f"\n  {RED}ERROR: Could not read values from firmware.{RESET}\n")
                _pause()
                continue

            # Show adjustable parameters with numbered choices
            adjustable = [
                "wheel_radius", "wheel_separation", "max_wheel_speed_ms",
                "motor_min_duty_left", "motor_min_duty_right",
                "motor_min_duty",
                "motor_kick_duty", "motor_kick_cycles",
                "motor_trim_left", "motor_trim_right",
                "left_motor_reversed", "right_motor_reversed", "swap_motors",
            ]
            print(f"\n  {BOLD}Select parameter to adjust:{RESET}\n")
            for idx, name in enumerate(adjustable, 1):
                label = CALIB_LABELS.get(name, name)
                cur = vals.get(name)
                if cur is not None:
                    cur_str = f"= {fmt_val(name, cur)}"
                elif name == "motor_min_duty":
                    cur_str = f"(sets both L={vals.get('motor_min_duty_left', '?')} / R={vals.get('motor_min_duty_right', '?')})"
                else:
                    cur_str = ""
                print(f"  {BOLD}{idx:>3d}{RESET}  {label:<28s}  {DIM}{cur_str}{RESET}")
            print(f"\n  {BOLD}  0{RESET}  Cancel\n")

            try:
                sel = input(f"  {CYAN}#{RESET} ").strip()
                sel_idx = int(sel) if sel else 0
            except (ValueError, EOFError, KeyboardInterrupt):
                sel_idx = 0

            if 1 <= sel_idx <= len(adjustable):
                name = adjustable[sel_idx - 1]
                cur = vals.get(name)
                if cur is None:
                    # motor_min_duty (both) — start from max of left/right
                    cur = max(vals.get("motor_min_duty_left", 0.5),
                              vals.get("motor_min_duty_right", 0.5))
                new_val = adjust_parameter(node, bus, port, name, float(cur),
                                            speed=speed, ang_speed=ang_speed)
                if new_val is not None:
                    print(f"  {GREEN}Set {name} = {new_val:.6f} (runtime){RESET}\n")
            else:
                print()

        elif choice in ("s", "save"):
            bus.close()
            pkt = send_calib(port, CALIB_CMD_SAVE)
            bus.reopen()
            if pkt and pkt[8] == 0:
                print(f"\n  {GREEN}Saved to flash successfully.{RESET}\n")
            else:
                print(f"\n  {RED}Save failed (no reply or error).{RESET}\n")
            _pause()

        elif choice in ("l", "load"):
            bus.close()
            pkt = send_calib(port, CALIB_CMD_LOAD)
            bus.reopen()
            if pkt and pkt[8] == 0:
                print(f"\n  {GREEN}Loaded from flash successfully.{RESET}\n")
            else:
                print(f"\n  {RED}Load failed (no reply or error).{RESET}\n")
            _pause()

        elif choice in ("r", "reset"):
            confirm = input(f"  {YELLOW}Reset to compile-time defaults? (y/N):{RESET} ")
            if confirm.strip().lower() == "y":
                bus.close()
                pkt = send_calib(port, CALIB_CMD_RESET)
                bus.reopen()
                if pkt and pkt[8] == 0:
                    print(f"\n  {GREEN}Reset to defaults (runtime only).{RESET}")
                    print(f"  Use {BOLD}S{RESET} to also persist to flash.\n")
                else:
                    print(f"\n  {RED}Reset failed.{RESET}\n")
            _pause()

        elif choice in ("q", "quit", "exit"):
            break

        else:
            print(f"\n  {DIM}Unknown choice '{choice}'{RESET}\n")


def _pause():
    """Press enter to continue."""
    try:
        input(f"  {DIM}Press Enter to continue...{RESET}")
    except (EOFError, KeyboardInterrupt):
        pass


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Interactive manual motor & wheel calibration")
    parser.add_argument("--port", default="/dev/ttyTB3",
                        help="Dynamixel serial port (default: /dev/ttyTB3)")
    parser.add_argument("--speed", type=float, default=0.08,
                        help="initial linear speed for driving (m/s, default: 0.08)")
    parser.add_argument("--ang-speed", type=float, default=0.5,
                        help="initial angular speed for driving (rad/s, default: 0.5)")
    args = parser.parse_args()

    # Quick connectivity check
    print(f"\n  Checking firmware on {args.port} ... ", end="", flush=True)
    vals = get_calibration_values(args.port)
    if vals is None:
        print(f"{RED}FAILED{RESET}")
        print(f"  Could not communicate with firmware. Is the robot powered on?\n")
        return 1
    print(f"{GREEN}OK{RESET}")
    print_values(vals)

    # Init ROS (for IMU only — motor commands go direct over serial)
    rclpy.init()
    node = ManualCalibNode()

    # Open persistent serial connection for motor control
    try:
        bus = DxlBus(args.port)
    except serial.SerialException as e:
        print(f"\n  {RED}ERROR: Could not open serial bus: {e}{RESET}\n")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # Spin once to check IMU
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.imu_ok:
        print(f"  IMU: {GREEN}detected{RESET}\n")
    else:
        print(f"  IMU: {YELLOW}not detected — gyro readout will be unavailable{RESET}\n")

    try:
        main_menu(node, bus, args.port, args.speed, args.ang_speed)
    except KeyboardInterrupt:
        pass
    finally:
        bus.stop_motors()
        bus.close()
        node.destroy_node()
        rclpy.shutdown()

    print(f"\n  {DIM}Goodbye.{RESET}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
