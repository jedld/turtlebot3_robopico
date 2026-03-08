#!/usr/bin/env python3
"""
Auto-calibrate per-motor MOTOR_MIN_DUTY (dead-zone compensation) and
MOTOR_KICK_DUTY using wheel encoders.  Detects motor asymmetry by testing
each motor individually.

Architecture
------------
The custom calibration instruction (0x90) cannot be forwarded through
turtlebot3_node, so the script must hold the serial port directly.  It
stops turtlebot3-bringup.service before opening the port, runs all probing
via direct Dynamixel packets, then restarts bringup when done.

Encoder readings come from firmware debug registers ADDR_DBG_VEL_L (212)
and ADDR_DBG_VEL_R (216) — the same encoder-measured m/s values that
appear on /joint_states.  No ROS stack is needed at probe time.

Strategy
--------
1. Stop bringup (or prompt to do so) to free the serial port.
2. Enable motor torque and disable kick-start (KICK_CYCLES=0).
3. Test each motor (left / right) in both directions via a pivot turn:
   tested motor gets MIN_DUTY boosted; the other motor gets 0.
4. Binary-search each motor's MIN_DUTY until the encoder detects rotation.
5. Report asymmetry, add safety margin, verify with pivot trials.
6. Persist to flash + source (--apply), then restart bringup.

Usage
-----
  python3 calibrate_deadzone.py          # dry-run: report only
  python3 calibrate_deadzone.py --apply  # persist to flash + firmware/main.c
"""

import argparse
import os
import re
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

import serial


# ── paths & firmware register addresses ──────────────────────────────────────

SCRIPT_DIR = Path(__file__).resolve().parent
MAIN_C     = SCRIPT_DIR / "firmware" / "main.c"

# Dynamixel device
DEV_ID   = 200
DXL_BAUD = 1_000_000

# Instruction codes
INST_READ        = 0x02
INST_WRITE       = 0x03
INST_CALIBRATION = 0x90   # custom firmware instruction

# Calibration sub-commands
CALIB_CMD_SET  = 0x01
CALIB_CMD_SAVE = 0x04

# Calibration keys for CALIB_CMD_SET
CALIB_KEY_MOTOR_MIN_DUTY        = 0x04
CALIB_KEY_MOTOR_KICK_DUTY       = 0x05
CALIB_KEY_MOTOR_KICK_CYCLES     = 0x06
CALIB_KEY_MOTOR_MIN_DUTY_LEFT   = 0x0B
CALIB_KEY_MOTOR_MIN_DUTY_RIGHT  = 0x0C

# Firmware RAM register addresses
ADDR_MILLIS       = 10    # int32 — uptime ms; any read keeps host-timeout alive
ADDR_TORQUE_EN    = 149   # uint8 — 1 = torque on
ADDR_CMD_LINEAR_X = 150   # int32 — 0.01 m/s units
ADDR_CMD_ANGULAR_Z = 170  # int32 — 0.01 rad/s units
ADDR_DBG_VEL_L    = 212   # float32 — measured left  wheel velocity (m/s)
ADDR_DBG_VEL_R    = 216   # float32 — measured right wheel velocity (m/s)

BRINGUP_SERVICE = "turtlebot3-bringup.service"

PROBE_SETTLE_S   = 0.25
PROBE_DURATION_S = 1.00
PROBE_TICK_S     = 0.05


# ── Dynamixel protocol helpers ────────────────────────────────────────────────

def _make_crc_table() -> list:
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


def _build_packet(dev_id: int, inst: int, params: bytes) -> bytes:
    pkt_len = len(params) + 3
    hdr = bytes([
        0xFF, 0xFF, 0xFD, 0x00,
        dev_id,
        pkt_len & 0xFF, (pkt_len >> 8) & 0xFF,
        inst,
    ])
    pkt = hdr + params
    crc = _crc16(pkt)
    return pkt + bytes([crc & 0xFF, crc >> 8])


def _read_response(ser: serial.Serial, timeout: float = 0.12) -> Optional[bytes]:
    """Read bytes until a complete Dynamixel2 status packet is assembled or
    the deadline expires.  Relies on the port's 100 ms read timeout so each
    ser.read() call returns promptly whether or not bytes arrived.

    Timeout is intentionally short (0.12 s) so that when the firmware stops
    responding (motor overload fault) each Dynamixel call returns quickly and
    keep_alive packets continue firing at the expected ~50 ms rate."""
    buf = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(64)   # returns in ≤100 ms (port timeout)
        if chunk:
            buf += chunk
        i = buf.find(b"\xFF\xFF\xFD\x00")
        if i >= 0:
            buf = buf[i:]
            if len(buf) >= 7:
                pl = buf[5] | (buf[6] << 8)
                total = 7 + pl
                if len(buf) >= total:
                    return buf[:total]
    return None


def dxl_write(ser: serial.Serial, addr: int, data: bytes) -> bool:
    params = struct.pack("<H", addr) + data
    pkt = _build_packet(DEV_ID, INST_WRITE, params)
    ser.write(pkt)
    resp = _read_response(ser)
    return resp is not None


def dxl_read(ser: serial.Serial, addr: int, length: int) -> Optional[bytes]:
    params = struct.pack("<HH", addr, length)
    pkt = _build_packet(DEV_ID, INST_READ, params)
    ser.write(pkt)
    resp = _read_response(ser)
    if resp is None or len(resp) < 9 + length:
        return None
    return resp[9:9 + length]


def dxl_read_f32(ser: serial.Serial, addr: int) -> Optional[float]:
    raw = dxl_read(ser, addr, 4)
    return struct.unpack_from("<f", raw)[0] if raw else None


def dxl_read_i32(ser: serial.Serial, addr: int) -> Optional[int]:
    raw = dxl_read(ser, addr, 4)
    return struct.unpack_from("<i", raw)[0] if raw else None


def send_calib(ser: serial.Serial, subcmd: int,
               key: Optional[int] = None,
               value: Optional[float] = None) -> None:
    """Send a custom 0x90 calibration packet over an already-open serial port."""
    params = bytes([subcmd])
    if subcmd == CALIB_CMD_SET:
        if key is None or value is None:
            raise ValueError("CALIB_CMD_SET requires key and value")
        params += bytes([key]) + struct.pack("<f", float(value))
    pkt = _build_packet(0xFE, INST_CALIBRATION, params)
    ser.write(pkt)
    time.sleep(0.02)


def keep_alive(ser: serial.Serial) -> None:
    """Read MILLIS to keep the firmware host-timeout counter from expiring."""
    dxl_read(ser, ADDR_MILLIS, 4)


# ── systemd bringup helpers ───────────────────────────────────────────────────

def bringup_is_active() -> bool:
    try:
        r = subprocess.run(
            ["systemctl", "is-active", "--quiet", BRINGUP_SERVICE], timeout=5)
        return r.returncode == 0
    except Exception:
        return False


def bringup_stop() -> bool:
    try:
        r = subprocess.run(
            ["sudo", "systemctl", "stop", BRINGUP_SERVICE],
            timeout=15, capture_output=True)
        if r.returncode != 0:
            print(f"  sudo systemctl stop failed: {r.stderr.decode().strip()}")
            return False
        for _ in range(50):
            time.sleep(0.1)
            if not bringup_is_active():
                return True
        return False
    except Exception as e:
        print(f"  bringup stop error: {e}")
        return False


def bringup_start() -> bool:
    try:
        r = subprocess.run(
            ["sudo", "systemctl", "restart", BRINGUP_SERVICE],
            timeout=20, capture_output=True)
        return r.returncode == 0
    except Exception as e:
        print(f"  bringup start error: {e}")
        return False


def tty_name_from_port(port: str) -> Optional[str]:
    try:
        return Path(port).resolve().name
    except Exception:
        return None


def usb_bus_id_for_tty(port: str) -> Optional[str]:
    """Map /dev/ttyTB3 -> ttyACM0 -> USB bus id like '4-1'."""
    tty = tty_name_from_port(port)
    if not tty:
        return None
    try:
        dev = Path(f"/sys/class/tty/{tty}/device").resolve()
        # ttyACM0 device resolves to .../4-1/4-1:1.0 ; parent is the USB device.
        bus = dev.parent.name if ":" in dev.name else dev.name
        return bus if "-" in bus else None
    except Exception:
        return None


def wait_for_port(port: str, timeout: float = 10.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if Path(port).exists():
            return True
        time.sleep(0.1)
    return False


def usb_rebind_for_port(port: str) -> bool:
    """Try to recover a stuck Pico CDC link by unbinding/rebinding its USB node."""
    bus_id = usb_bus_id_for_tty(port)
    if not bus_id:
        return False
    cmd = (
        f"echo -n {bus_id} > /sys/bus/usb/drivers/usb/unbind && "
        f"sleep 1 && "
        f"echo -n {bus_id} > /sys/bus/usb/drivers/usb/bind"
    )
    try:
        r = subprocess.run(["sudo", "sh", "-c", cmd], timeout=10)
        if r.returncode != 0:
            return False
        return wait_for_port(port, timeout=10.0)
    except Exception:
        return False


def ping_pico(ser: serial.Serial, timeout: float = 8.0) -> Optional[int]:
    """Poll ADDR_MILLIS until a valid response arrives or timeout expires."""
    deadline = time.time() + timeout
    dot_next = time.time() + 1.0
    attempt = 0
    while time.time() < deadline:
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass

        millis = dxl_read_i32(ser, ADDR_MILLIS)
        if millis is not None:
            print()
            print(f"Pico uptime     : {millis} ms  ({attempt + 1} attempt(s))\n",
                  flush=True)
            return millis

        attempt += 1
        now = time.time()
        if now >= dot_next:
            print(".", end="", flush=True)
            dot_next = now + 1.0

        if attempt % 5 == 0:
            try:
                ser.close()
                time.sleep(0.5)
                ser.open()
                print("R", end="", flush=True)
            except Exception:
                pass

        time.sleep(0.15)
    print()
    return None


# ── source-file helpers ───────────────────────────────────────────────────────

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
    def repl(m):
        return f"{m.group(1)}{value:.2f}f{suffix}"
    new_content, count = pat.subn(repl, content, count=1)
    if count == 0:
        raise RuntimeError(f"Could not find #define {name} in {path}")
    path.write_text(new_content)


# ── direct serial probe ───────────────────────────────────────────────────────

# Consecutive near-zero samples before declaring a definite stall early
_EARLY_STALL_WINDOW  = 5      # consecutive samples below threshold
_EARLY_STALL_VEL     = 0.005  # m/s — wheel is essentially stopped


def probe_wheel_direct(
    ser: serial.Serial,
    motor: str,
    ang_z_val: float,
    duration: float = PROBE_DURATION_S,
    settle: float = PROBE_SETTLE_S,
    live: Optional[bool] = None,
) -> "tuple[float, float]":
    """
    Command a pivot at ang_z_val rad/s via direct Dynamixel writes, sample
    ADDR_DBG_VEL_L and ADDR_DBG_VEL_R every 50 ms and return
    ``(mean_abs_vel_tested_wheel, mean_abs_vel_other_wheel)`` in m/s.

    Keeps firmware host-timeout alive by calling keep_alive(ser) each cycle.

    motor = "left"  → returns (mean_abs_L, mean_abs_R)
    motor = "right" → returns (mean_abs_R, mean_abs_L)

    Early-exit: if the tested wheel stays below _EARLY_STALL_VEL for
    _EARLY_STALL_WINDOW consecutive samples after the settle phase,
    the probe returns immediately to avoid grinding motors against the floor.
    """
    lin_raw      = struct.pack("<i", 0)
    ang_raw_zero = struct.pack("<i", 0)
    ang_raw      = struct.pack("<i", int(round(ang_z_val * 100)))  # 0.01 rad/s

    tested_idx = 0 if motor == "left" else 1  # index into (L, R) for early-exit
    if live is None:
        live = sys.stdout.isatty()  # suppress \r updates when output is piped/logged

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    # Settle: zero velocity
    t_end = time.time() + settle
    while time.time() < t_end:
        dxl_write(ser, ADDR_CMD_LINEAR_X,  lin_raw)
        dxl_write(ser, ADDR_CMD_ANGULAR_Z, ang_raw_zero)
        keep_alive(ser)
        time.sleep(PROBE_TICK_S)

    # Command phase: sample encoder velocities
    vels_l: list = []
    vels_r: list = []
    stall_streak = 0
    t_end = time.time() + duration
    while time.time() < t_end:
        dxl_write(ser, ADDR_CMD_LINEAR_X,  lin_raw)
        dxl_write(ser, ADDR_CMD_ANGULAR_Z, ang_raw)
        keep_alive(ser)
        v_l = dxl_read_f32(ser, ADDR_DBG_VEL_L) or 0.0
        v_r = dxl_read_f32(ser, ADDR_DBG_VEL_R) or 0.0
        vels_l.append(v_l)
        vels_r.append(v_r)
        if live:
            print(f"      vel L={v_l:+.4f}  R={v_r:+.4f} m/s",
                  end="\r", flush=True)
        time.sleep(PROBE_TICK_S)

        # Early-exit: tested wheel has been stalled for several samples
        tested_vel = abs(v_l) if tested_idx == 0 else abs(v_r)
        if tested_vel < _EARLY_STALL_VEL:
            stall_streak += 1
        else:
            stall_streak = 0
        if stall_streak >= _EARLY_STALL_WINDOW and len(vels_l) > _EARLY_STALL_WINDOW:
            if live:
                print()  # newline after the \r line
            break

    if live:
        print()  # ensure newline after last \r

    # Stop
    dxl_write(ser, ADDR_CMD_LINEAR_X,  lin_raw)
    dxl_write(ser, ADDR_CMD_ANGULAR_Z, ang_raw_zero)
    keep_alive(ser)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    # Trim first 3 samples (settling transient)
    vels_l = vels_l[3:] if len(vels_l) > 3 else vels_l
    vels_r = vels_r[3:] if len(vels_r) > 3 else vels_r

    mean_l = sum(abs(v) for v in vels_l) / len(vels_l) if vels_l else 0.0
    mean_r = sum(abs(v) for v in vels_r) / len(vels_r) if vels_r else 0.0

    if motor == "left":
        return mean_l, mean_r
    return mean_r, mean_l


def find_single_motor_stall(
    ser: serial.Serial,
    motor: str,
    direction: str,
    ang_speed: float,
    search_lo: float = 0.10,
    search_hi: float = 0.80,
    encoder_threshold: float = 0.04,
    resolution: float = 0.02,
    probe_duration: float = 1.5,
    verbose: bool = True,
) -> float:
    """
    Binary-search for the lowest MOTOR_MIN_DUTY at which the specified motor
    reliably turns, using the encoder-measured wheel velocity (m/s) as the
    stall signal.

    The tested motor's MIN_DUTY is set to the candidate value; the other
    motor's MIN_DUTY is held at 0.0 so only the tested motor can start.

    Parameters
    ----------
    motor             : "left" or "right"
    direction         : "forward" or "backward"
    ang_speed         : pivot speed magnitude (rad/s, positive)
    encoder_threshold : m/s below which the wheel is considered stalled
    """
    # Pivot direction: left-forward = CW = negative ang_z
    if motor == "left":
        ang_z = -ang_speed if direction == "forward" else ang_speed
    else:
        ang_z = ang_speed if direction == "forward" else -ang_speed

    key_test  = (CALIB_KEY_MOTOR_MIN_DUTY_LEFT  if motor == "left"
                 else CALIB_KEY_MOTOR_MIN_DUTY_RIGHT)
    key_other = (CALIB_KEY_MOTOR_MIN_DUTY_RIGHT if motor == "left"
                 else CALIB_KEY_MOTOR_MIN_DUTY_LEFT)

    label = f"{motor}-{direction}"

    def _probe_at(duty: float) -> "tuple[bool, str]":
        if verbose:
            print(f"    [{label}] probing duty={duty:.3f} …", flush=True)
        send_calib(ser, CALIB_CMD_SET, key_test,  value=duty)
        send_calib(ser, CALIB_CMD_SET, key_other, value=0.0)
        time.sleep(0.05)
        vel_tested, vel_other = probe_wheel_direct(
            ser, motor, ang_z, duration=probe_duration)
        moving = vel_tested > encoder_threshold
        disp = (f"duty={duty:.3f}  tested={vel_tested:.4f} m/s  "
                f"other={vel_other:.4f} m/s  "
                f"{'MOVING' if moving else 'STALL'}")
        return moving, disp

    # Check upper bound
    moving, disp = _probe_at(search_hi)
    if verbose:
        print(f"    [{label}] {disp}")
    if not moving:
        if verbose:
            print(f"    [{label}] Motor does not move even at {search_hi:.2f}!")
        return search_hi

    # Check lower bound
    moving, disp = _probe_at(search_lo)
    if verbose:
        print(f"    [{label}] {disp}")
    if moving:
        if verbose:
            print(f"    [{label}] Motor moves even at {search_lo:.2f} — below range.")
        return search_lo

    # Binary search
    lo, hi = search_lo, search_hi
    iterations = 0
    while (hi - lo) > resolution and iterations < 20:
        mid = (lo + hi) / 2.0
        moving, disp = _probe_at(mid)
        if verbose:
            print(f"    [{label}] {disp}")
        if moving:
            hi = mid
        else:
            lo = mid
        iterations += 1

    if verbose:
        print(f"    [{label}] stall threshold: {hi:.3f}")
    return hi


def verify_duty(
    ser: serial.Serial,
    duty_left: float,
    duty_right: float,
    ang_speed: float,
    encoder_threshold: float = 0.04,
    verify_threshold: Optional[float] = None,
    trials: int = 3,
    verbose: bool = True,
) -> bool:
    """
    Confirm that both motors reliably start with the candidate MIN_DUTY values.
    Alternates CW / CCW pivots to exercise each motor in both roles.

    verify_threshold : m/s threshold used here (defaults to encoder_threshold/2).
      During verification both motors run simultaneously in a pivot, so each
      wheel produces lower velocity than in the single-motor binary-search phase.
      Using half the search threshold avoids false-stall detection on real motion.
    """
    if verify_threshold is None:
        verify_threshold = encoder_threshold / 2.0

    send_calib(ser, CALIB_CMD_SET, CALIB_KEY_MOTOR_MIN_DUTY_LEFT,  value=duty_left)
    send_calib(ser, CALIB_CMD_SET, CALIB_KEY_MOTOR_MIN_DUTY_RIGHT, value=duty_right)
    time.sleep(0.05)

    if verbose:
        print(f"    (verify threshold: {verify_threshold:.4f} m/s)")

    successes = 0
    for t in range(trials):
        direction = 1.0 if t % 2 == 0 else -1.0
        label_dir = "CCW" if direction > 0 else "CW "
        vel_l, vel_r = probe_wheel_direct(
            ser, "left", direction * ang_speed, duration=1.5)
        both_ok = (vel_l > verify_threshold) and (vel_r > verify_threshold)
        if verbose:
            print(f"    Trial {t+1}/{trials}  {label_dir}  "
                  f"L={vel_l:.4f} m/s  R={vel_r:.4f} m/s  "
                  f"{'OK' if both_ok else 'STALL'}")
        if both_ok:
            successes += 1
    return successes == trials


# ── diagnostic thresholds ─────────────────────────────────────────────────────

# Absolute stall duty thresholds
STALL_DUTY_NOMINAL_LO = 0.10   # below this the motor is unusually free-spinning
STALL_DUTY_NOMINAL_HI = 0.35   # above this something is wrong (friction/wiring)
STALL_DUTY_CRITICAL   = 0.55   # motor barely moves — serious issue

# Directional asymmetry: fwd vs bwd for the SAME motor
DIR_ASYM_WARN  = 0.04   # > 4% duty difference between fwd/bwd
DIR_ASYM_CRIT  = 0.08   # > 8% — likely brush wear or bad connection

# Left-vs-right motor asymmetry
LR_ASYM_WARN   = 0.03   # noticeable manufacturing variation
LR_ASYM_CRIT   = 0.08   # one motor is significantly weaker

# Verification failure
VERIFY_BUMP_WARN = True  # warn if verification needed an extra margin bump


class Diagnostic:
    """Single diagnostic finding."""
    def __init__(self, level: str, category: str, message: str, detail: str = ""):
        self.level = level        # "INFO", "WARNING", "CRITICAL"
        self.category = category  # e.g. "wiring", "motor", "encoder"
        self.message = message
        self.detail = detail

    def __str__(self):
        pfx = {"INFO": "ℹ", "WARNING": "⚠", "CRITICAL": "✖"}.get(self.level, "?")
        s = f"  {pfx}  [{self.level}] {self.message}"
        if self.detail:
            s += f"\n      → {self.detail}"
        return s


def analyse_single_motor(
    motor: str,
    fwd_stall: float,
    bwd_stall: float,
    diagnostics: list,
) -> None:
    """Generate diagnostics for one motor after both directions are probed."""
    worst = max(fwd_stall, bwd_stall)
    dir_diff = abs(fwd_stall - bwd_stall)
    worse_dir = "backward" if bwd_stall > fwd_stall else "forward"

    # ── Absolute stall duty ───────────────────────────────────────────
    if worst >= STALL_DUTY_CRITICAL:
        diagnostics.append(Diagnostic(
            "CRITICAL", "motor",
            f"{motor.upper()} motor needs {worst:.3f} duty to start — extremely high.",
            "Possible causes: bad solder joint on motor wires, damaged H-bridge, "
            "seized gearbox, or heavily corroded brushes. "
            "Inspect motor connector, H-bridge driver, and gearbox."
        ))
    elif worst >= STALL_DUTY_NOMINAL_HI:
        diagnostics.append(Diagnostic(
            "WARNING", "motor",
            f"{motor.upper()} motor stall duty is high ({worst:.3f}).",
            "This may indicate increased mechanical friction, marginal wiring, "
            "or motor brush wear. Check motor wire connections and gearbox for "
            "debris or binding."
        ))
    elif worst < STALL_DUTY_NOMINAL_LO:
        diagnostics.append(Diagnostic(
            "INFO", "motor",
            f"{motor.upper()} motor starts very easily (stall duty {worst:.3f}).",
            "This is fine — the motor has low friction."
        ))

    # ── Directional asymmetry (fwd vs bwd, same motor) ────────────────
    if dir_diff >= DIR_ASYM_CRIT:
        diagnostics.append(Diagnostic(
            "CRITICAL", "wiring",
            f"{motor.upper()} motor: {worse_dir} is much harder to start "
            f"(fwd={fwd_stall:.3f}, bwd={bwd_stall:.3f}, Δ={dir_diff:.3f}).",
            "Large directional asymmetry usually means a bad connection on one "
            "H-bridge channel pin (e.g. loose crimped wire, cold solder joint), "
            "or significant brush wear in one commutator direction. "
            "Re-seat or re-solder the motor wires and retest."
        ))
    elif dir_diff >= DIR_ASYM_WARN:
        diagnostics.append(Diagnostic(
            "WARNING", "wiring",
            f"{motor.upper()} motor: fwd/bwd stall differs by {dir_diff:.3f} "
            f"(fwd={fwd_stall:.3f}, bwd={bwd_stall:.3f}).",
            "Moderate directional asymmetry — could be normal manufacturing "
            "variation, or early sign of brush wear / marginal wiring. "
            "Worth monitoring; if it increases over time, inspect motor wires."
        ))


def analyse_left_right(
    left_stall: float,
    right_stall: float,
    left_fwd: float,
    left_bwd: float,
    right_fwd: float,
    right_bwd: float,
    diagnostics: list,
) -> None:
    """Generate diagnostics comparing left vs right motors."""
    lr_diff = abs(left_stall - right_stall)
    weaker = "RIGHT" if right_stall > left_stall else "LEFT"

    if lr_diff >= LR_ASYM_CRIT:
        diagnostics.append(Diagnostic(
            "CRITICAL", "motor",
            f"Large left/right motor asymmetry: Δ={lr_diff:.3f} "
            f"(L={left_stall:.3f}, R={right_stall:.3f}).",
            f"The {weaker} motor requires significantly more power to start. "
            f"This will cause the robot to pull to one side even with PID. "
            f"Check: (1) motor wiring quality on the {weaker.lower()} side, "
            f"(2) gearbox binding, (3) wheel rubbing against chassis, "
            f"(4) motor manufacturing defect — consider swapping motors to "
            f"see if the problem follows the motor or stays on that side."
        ))
    elif lr_diff >= LR_ASYM_WARN:
        diagnostics.append(Diagnostic(
            "WARNING", "motor",
            f"Noticeable left/right asymmetry: Δ={lr_diff:.3f} "
            f"(L={left_stall:.3f}, R={right_stall:.3f}).",
            f"The {weaker} motor has a higher dead-zone. PID will compensate, "
            f"but at very low speeds the robot may drift. "
            f"Normal for inexpensive TT motors — per-motor MIN_DUTY will help."
        ))
    else:
        diagnostics.append(Diagnostic(
            "INFO", "motor",
            f"Left/right motors are well matched (Δ={lr_diff:.3f}).",
            "Good wiring and motor quality."
        ))

    # Check if both motors have high stall duty (systemic issue)
    if left_stall >= STALL_DUTY_NOMINAL_HI and right_stall >= STALL_DUTY_NOMINAL_HI:
        diagnostics.append(Diagnostic(
            "WARNING", "wiring",
            f"Both motors have high stall duty (L={left_stall:.3f}, R={right_stall:.3f}).",
            "Since both sides are affected, check common factors: "
            "battery voltage / power supply, shared ground wire quality, "
            "level-shifter or H-bridge power, or excessive mechanical load "
            "(e.g. wheels too tight against chassis, heavy payload)."
        ))


def print_diagnostics(diagnostics: list) -> None:
    """Print a categorised health summary."""
    if not diagnostics:
        return

    crits = [d for d in diagnostics if d.level == "CRITICAL"]
    warns = [d for d in diagnostics if d.level == "WARNING"]
    infos = [d for d in diagnostics if d.level == "INFO"]

    print("\n" + "=" * 72)
    print("DIAGNOSTIC SUMMARY")
    print("=" * 72)

    if crits:
        print(f"\n  {len(crits)} critical issue(s):")
        for d in crits:
            print(d)
    if warns:
        print(f"\n  {len(warns)} warning(s):")
        for d in warns:
            print(d)
    if infos:
        print(f"\n  {len(infos)} info note(s):")
        for d in infos:
            print(d)

    if not crits and not warns:
        print("\n  ✓  All checks passed — motors and wiring look healthy.")
    elif crits:
        print("\n  ✖  Critical issues found — inspect hardware before relying on calibration.")
    else:
        print("\n  ⚠  Some warnings — review above; calibration values will compensate but")
        print("     the underlying cause should be investigated if issues worsen.")
    print()


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Auto-calibrate per-motor MOTOR_MIN_DUTY (dead-zone compensation) "
            "using wheel encoders via direct Dynamixel serial. "
            "Stops turtlebot3-bringup.service to acquire the port, probes "
            "each motor individually, then restarts bringup."
        )
    )
    parser.add_argument("--speed", type=float, default=2.0,
                        help="pivot angular speed in rad/s (default 2.0)")
    parser.add_argument("--margin", type=float, default=0.05,
                        help="safety margin above stall threshold (default 0.05)")
    parser.add_argument("--kick-margin", type=float, default=0.05,
                        help="KICK_DUTY = max(MIN_L, MIN_R) + this (default 0.05)")
    parser.add_argument("--kick-cycles", type=int, default=3,
                        help="MOTOR_KICK_CYCLES to set (default 3)")
    parser.add_argument("--encoder-threshold", type=float, default=0.008,
                        metavar="M_S",
                        help="wheel velocity m/s below which wheel is stalled "
                             "(default 0.008 m/s)")
    parser.add_argument("--verify-threshold", type=float, default=None,
                        metavar="M_S",
                        help="velocity threshold used in the verification pivot "
                             "trials (default: encoder-threshold / 2). "
                             "Both motors run simultaneously so each wheel "
                             "moves slower than in single-motor probing.")
    parser.add_argument("--search-lo", type=float, default=0.10,
                        help="lowest MIN_DUTY to try (default 0.10)")
    parser.add_argument("--search-hi", type=float, default=0.80,
                        help="highest MIN_DUTY to try (default 0.80)")
    parser.add_argument("--resolution", type=float, default=0.02,
                        help="binary-search step size (default 0.02)")
    parser.add_argument("--apply", action="store_true",
                        help="persist to flash + firmware/main.c")
    parser.add_argument("--dxl-port", default="/dev/ttyTB3", metavar="PORT",
                        help="Dynamixel serial port (default: /dev/ttyTB3)")
    parser.add_argument("--no-stop-bringup", action="store_true",
                        help="skip stopping turtlebot3-bringup.service "
                             "(port must already be free)")
    args = parser.parse_args()

    # Read current values from firmware source
    cur_min_duty_left  = read_define_float(MAIN_C, "MOTOR_MIN_DUTY_LEFT_DEFAULT")
    cur_min_duty_right = read_define_float(MAIN_C, "MOTOR_MIN_DUTY_RIGHT_DEFAULT")
    cur_kick_duty      = read_define_float(MAIN_C, "MOTOR_KICK_DUTY_DEFAULT")
    cur_kick_cyc       = read_define_float(MAIN_C, "MOTOR_KICK_CYCLES_DEFAULT")

    print("\n=== Dead-Zone Compensation Calibration (direct serial, encoder-based) ===")
    print(f"Firmware        : {MAIN_C}")
    print(f"Current         : MIN_DUTY_L={cur_min_duty_left}  "
          f"MIN_DUTY_R={cur_min_duty_right}  "
          f"KICK_DUTY={cur_kick_duty}  KICK_CYCLES={cur_kick_cyc}")
    _verify_thr = args.verify_threshold if args.verify_threshold is not None \
        else args.encoder_threshold / 2.0
    print(f"Test speed      : {args.speed:.2f} rad/s")
    print(f"Encoder thresh  : {args.encoder_threshold:.4f} m/s  "
          f"(verify thresh: {_verify_thr:.4f} m/s)")
    print(f"Search range    : [{args.search_lo:.2f} … {args.search_hi:.2f}]  "
          f"resolution={args.resolution:.2f}")
    print(f"Safety margin   : {args.margin:.2f}")

    # Sanity check: expected wheel velocity during pivot vs encoder threshold
    _wheel_sep = read_define_float(MAIN_C, "WHEEL_SEPARATION_DEFAULT") or 0.16
    _expected_v = args.speed * (_wheel_sep / 2.0)
    print(f"Wheel separation: {_wheel_sep:.6f} m")
    print(f"Expected wheel v: {_expected_v:.4f} m/s at {args.speed:.1f} rad/s pivot")
    if _expected_v < args.encoder_threshold * 1.5:
        print(f"  WARNING: Expected wheel velocity ({_expected_v:.4f} m/s) is near or below")
        print(f"           encoder threshold ({args.encoder_threshold:.4f} m/s)!")
        print(f"           Increase --speed or decrease --encoder-threshold.")
        print(f"           Recommended: --speed {max(args.speed, args.encoder_threshold * 3.0 / (_wheel_sep / 2.0)):.1f}")
    print()

    stopped_bringup = False
    if not args.no_stop_bringup:
        if bringup_is_active():
            print(f"Stopping {BRINGUP_SERVICE} …")
            if not bringup_stop():
                print("ERROR: could not stop bringup service.")
                print("  Use --no-stop-bringup if the port is already free.")
                return 1
            stopped_bringup = True
            print("  stopped.\n")
        else:
            print(f"  {BRINGUP_SERVICE} not active — proceeding.\n")

    print(f"Opening {args.dxl_port} …", flush=True)

    # Detect any other process already holding the port and warn immediately
    try:
        _holders = subprocess.check_output(
            ["lsof", "-t", args.dxl_port], stderr=subprocess.DEVNULL
        ).decode().split()
        _holders = [p for p in _holders if p.strip().isdigit()
                    and int(p) != os.getpid()]
        if _holders:
            print(f"  WARNING: the following PID(s) have {args.dxl_port} open: "
                  f"{', '.join(_holders)}")
            print(f"  Kill them first:  kill {' '.join(_holders)}")
            print(f"  Then re-run this script.")
            if stopped_bringup:
                bringup_start()
            return 1
    except Exception:
        pass   # lsof may not be available; continue and let open() fail

    try:
        ser = serial.Serial(
            args.dxl_port, DXL_BAUD,
            timeout=0.05,         # 50 ms blocking read; short so failed _read_response
                                  # calls return in ≤150 ms and keep_alive stays timely
            dsrdtr=False, rtscts=False,
        )
        # Flush any stale bytes left over from a previous bringup session
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except serial.SerialException as e:
        print(f"ERROR: cannot open {args.dxl_port}: {e}")
        if stopped_bringup:
            print("Restarting bringup …")
            bringup_start()
        return 1

    try:
        # After a Ctrl-C or bringup stop the USB-CDC link may need several
        # seconds to re-enumerate.  Give it a short head-start, flush, then
        # loop printing a dot every second so the user can see progress.
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print("Pinging Pico ", end="", flush=True)
        millis = ping_pico(ser, timeout=8.0)

        if millis is None:
            print("Trying USB rebind recovery …", flush=True)
            try:
                ser.close()
            except Exception:
                pass

            if usb_rebind_for_port(args.dxl_port):
                time.sleep(1.0)
                ser.open()
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                print("Pinging Pico after USB rebind ", end="", flush=True)
                millis = ping_pico(ser, timeout=10.0)

        if millis is None:
            print(f"ERROR: Pico did not respond on {args.dxl_port}.")
            print("  Recovery attempted: USB rebind.")
            print("  The Pico may need a hardware reboot if its USB stack wedged.")
            return 1

        # Enable torque, disable kick-start for clean stall detection
        dxl_write(ser, ADDR_TORQUE_EN, bytes([1]))
        send_calib(ser, CALIB_CMD_SET, CALIB_KEY_MOTOR_KICK_CYCLES, value=0.0)
        print("Torque ON, kick-start disabled (KICK_CYCLES=0)\n", flush=True)

        diagnostics: list[Diagnostic] = []

        # ── Left motor ────────────────────────────────────────────────────
        print("─" * 64)
        print("Step 1a: Left motor dead-zone (forward)")
        print("─" * 64)
        left_fwd_stall = find_single_motor_stall(
            ser, "left", "forward",
            ang_speed=args.speed,
            search_lo=args.search_lo, search_hi=args.search_hi,
            encoder_threshold=args.encoder_threshold,
            resolution=args.resolution,
        )
        print()

        print("─" * 64)
        print("Step 1b: Left motor dead-zone (backward)")
        print("─" * 64)
        left_bwd_stall = find_single_motor_stall(
            ser, "left", "backward",
            ang_speed=args.speed,
            search_lo=args.search_lo, search_hi=args.search_hi,
            encoder_threshold=args.encoder_threshold,
            resolution=args.resolution,
        )
        left_stall = max(left_fwd_stall, left_bwd_stall)
        print(f"\n  Left:  fwd={left_fwd_stall:.3f}  bwd={left_bwd_stall:.3f}"
              f"  worst={left_stall:.3f}\n")
        analyse_single_motor("left", left_fwd_stall, left_bwd_stall, diagnostics)

        # ── Right motor ───────────────────────────────────────────────────
        print("─" * 64)
        print("Step 2a: Right motor dead-zone (forward)")
        print("─" * 64)
        right_fwd_stall = find_single_motor_stall(
            ser, "right", "forward",
            ang_speed=args.speed,
            search_lo=args.search_lo, search_hi=args.search_hi,
            encoder_threshold=args.encoder_threshold,
            resolution=args.resolution,
        )
        print()

        print("─" * 64)
        print("Step 2b: Right motor dead-zone (backward)")
        print("─" * 64)
        right_bwd_stall = find_single_motor_stall(
            ser, "right", "backward",
            ang_speed=args.speed,
            search_lo=args.search_lo, search_hi=args.search_hi,
            encoder_threshold=args.encoder_threshold,
            resolution=args.resolution,
        )
        right_stall = max(right_fwd_stall, right_bwd_stall)
        print(f"\n  Right: fwd={right_fwd_stall:.3f}  bwd={right_bwd_stall:.3f}"
              f"  worst={right_stall:.3f}\n")
        analyse_single_motor("right", right_fwd_stall, right_bwd_stall, diagnostics)

        # ── Asymmetry report ──────────────────────────────────────────────
        asymmetry = abs(left_stall - right_stall)
        print("─" * 64)
        print("Motor asymmetry")
        print("─" * 64)
        print(f"  Left  stall duty : {left_stall:.3f}")
        print(f"  Right stall duty : {right_stall:.3f}")
        print(f"  Asymmetry        : {asymmetry:.3f}")
        if asymmetry > 0.02:
            weaker = "RIGHT" if right_stall > left_stall else "LEFT"
            print(f"  NOTE: {weaker} motor has higher dead-zone — per-motor values applied.")
        else:
            print("  Motors are well matched.")
        print()

        analyse_left_right(left_stall, right_stall,
                           left_fwd_stall, left_bwd_stall,
                           right_fwd_stall, right_bwd_stall, diagnostics)

        # ── Compute new per-motor values ──────────────────────────────────
        new_min_duty_left  = min(left_stall  + args.margin, 0.95)
        new_min_duty_right = min(right_stall + args.margin, 0.95)
        new_kick_duty = min(max(new_min_duty_left, new_min_duty_right)
                            + args.kick_margin, 0.95)
        new_kick_cycles = args.kick_cycles

        # ── Verify with pivot trials ──────────────────────────────────────
        print("─" * 64)
        print(f"Step 3: Verify  MIN_DUTY_L={new_min_duty_left:.3f}  "
              f"MIN_DUTY_R={new_min_duty_right:.3f}")
        print("─" * 64)
        send_calib(ser, CALIB_CMD_SET, CALIB_KEY_MOTOR_KICK_CYCLES,
                   value=float(new_kick_cycles))
        send_calib(ser, CALIB_CMD_SET, CALIB_KEY_MOTOR_KICK_DUTY,
                   value=new_kick_duty)

        ok = verify_duty(ser, new_min_duty_left, new_min_duty_right,
                         ang_speed=args.speed,
                         encoder_threshold=args.encoder_threshold,
                         verify_threshold=args.verify_threshold,
                         trials=4)
        if not ok:
            print(f"\n  Verification failed — bumping both by extra {args.margin:.2f}")
            diagnostics.append(Diagnostic(
                "WARNING", "motor",
                "Initial verification failed — extra margin was needed.",
                "The motors are borderline at the calculated duty. "
                "This can indicate inconsistent motor behaviour (brush "
                "contact variation) or marginal wiring. The extra margin "
                "will help, but monitor for stall issues during operation."
            ))
            new_min_duty_left  = min(new_min_duty_left  + args.margin, 0.95)
            new_min_duty_right = min(new_min_duty_right + args.margin, 0.95)
            new_kick_duty = min(max(new_min_duty_left, new_min_duty_right)
                                + args.kick_margin, 0.95)
            ok = verify_duty(ser, new_min_duty_left, new_min_duty_right,
                             ang_speed=args.speed,
                             encoder_threshold=args.encoder_threshold,
                             verify_threshold=args.verify_threshold,
                             trials=4)
            if not ok:
                print("  WARNING: still failing verification — applying anyway.")
                diagnostics.append(Diagnostic(
                    "CRITICAL", "motor",
                    "Verification STILL failing after extra margin bump.",
                    "Even with increased duty the motors do not reliably start. "
                    "This strongly suggests a hardware problem: bad motor, "
                    "damaged H-bridge output, poor power supply, or severely "
                    "worn brushes. Inspect all motor connections and power."
                ))

        # Stop motors
        dxl_write(ser, ADDR_CMD_LINEAR_X,  struct.pack("<i", 0))
        dxl_write(ser, ADDR_CMD_ANGULAR_Z, struct.pack("<i", 0))

        # ── Results ───────────────────────────────────────────────────────
        print()
        print("─" * 64)
        print("Result")
        print("─" * 64)
        print(f"  Left  stall threshold : {left_stall:.3f}")
        print(f"  Right stall threshold : {right_stall:.3f}")
        print(f"  Safety margin         : {args.margin:.2f}")
        print(f"  MOTOR_MIN_DUTY_LEFT   : {cur_min_duty_left} → {new_min_duty_left:.3f}")
        print(f"  MOTOR_MIN_DUTY_RIGHT  : {cur_min_duty_right} → {new_min_duty_right:.3f}")
        print(f"  MOTOR_KICK_DUTY       : {cur_kick_duty} → {new_kick_duty:.3f}")
        print(f"  MOTOR_KICK_CYCLES     : {cur_kick_cyc} → {new_kick_cycles}")

        # ── Diagnostic summary ────────────────────────────────────────────
        print_diagnostics(diagnostics)

        if args.apply:
            send_calib(ser, CALIB_CMD_SAVE)
            print("Persisted       : saved to firmware flash")
            write_define_float(MAIN_C, "MOTOR_MIN_DUTY_LEFT_DEFAULT",
                               new_min_duty_left,
                               "calibrated — calibrate_deadzone.py")
            write_define_float(MAIN_C, "MOTOR_MIN_DUTY_RIGHT_DEFAULT",
                               new_min_duty_right,
                               "calibrated — calibrate_deadzone.py")
            write_define_float(MAIN_C, "MOTOR_KICK_DUTY_DEFAULT", new_kick_duty,
                               "calibrated — calibrate_deadzone.py")
            content = MAIN_C.read_text()
            pat = re.compile(
                r"(#define\s+MOTOR_KICK_CYCLES_DEFAULT\s+)\d+u?([ \t]*(?://[^\n]*)?)")
            new_content, count = pat.subn(
                lambda m: (f"{m.group(1)}{new_kick_cycles}u"
                           f"  // calibrated — calibrate_deadzone.py"),
                content, count=1)
            if count:
                MAIN_C.write_text(new_content)
            print(f"Applied to      : {MAIN_C.relative_to(SCRIPT_DIR)}")
            print("\nDone — calibration applied live and persisted.\n")
        else:
            # Restore old values (dry-run)
            try:
                if cur_min_duty_left is not None:
                    send_calib(ser, CALIB_CMD_SET, CALIB_KEY_MOTOR_MIN_DUTY_LEFT,
                               value=cur_min_duty_left)
                if cur_min_duty_right is not None:
                    send_calib(ser, CALIB_CMD_SET, CALIB_KEY_MOTOR_MIN_DUTY_RIGHT,
                               value=cur_min_duty_right)
                if cur_kick_duty is not None:
                    send_calib(ser, CALIB_CMD_SET, CALIB_KEY_MOTOR_KICK_DUTY,
                               value=cur_kick_duty)
                if cur_kick_cyc is not None:
                    send_calib(ser, CALIB_CMD_SET, CALIB_KEY_MOTOR_KICK_CYCLES,
                               value=float(cur_kick_cyc))
                print("Restored        : original motor values (dry-run)")
            except Exception as e:
                print(f"Warning: restore failed ({e})")
            print("\nDry-run complete. Use --apply to persist. No files changed.\n")

    finally:
        try:
            ser.close()
        except Exception:
            pass
        if stopped_bringup:
            print(f"Restarting {BRINGUP_SERVICE} …")
            if bringup_start():
                print("  bringup restarted.\n")
            else:
                print("  WARNING: bringup did not restart — run manually:\n"
                      "    sudo systemctl restart turtlebot3-bringup.service\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

