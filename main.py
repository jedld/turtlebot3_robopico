"""
TurtleBot3 Pi Pico 2 Firmware
=====================================
Emulates the OpenCR board interface for TurtleBot3 using a Raspberry Pi Pico 2
on a Cytron Robo Pico carrier board.

Protocol : Dynamixel Protocol 2.0 (slave / device side)
Interface: USB CDC serial (/dev/ttyACM1 on host — data port)
Baud rate: USB CDC (host sets 1000000, device operates at USB 2.0 speed)

Hardware mapping (Cytron Robo Pico):
  Left  wheel : M1  — GP8  (M1A forward), GP9  (M1B reverse)
  Right wheel : M2  — GP10 (M2A forward), GP11 (M2B reverse)
  Button 1    : GP20 (active low)
  Button 2    : GP21 (active low)
  Buzzer      : GP22 (PWM)
  Battery ADC : GP29 (VSYS monitor, 3× voltage divider)

Reference:
  src/turtlebot3/turtlebot3_node/include/turtlebot3_node/control_table.hpp
  src/turtlebot3/turtlebot3_bringup/param/burger.yaml
"""

import board
import digitalio
import pwmio
import analogio
import time
import struct
import usb_cdc
import microcontroller

# ============================================================
# USER-ADJUSTABLE CONFIGURATION
# ============================================================

DEVICE_ID           = 200       # Must match opencr.id in burger.yaml
MODEL_NUMBER        = 0x5700    # 0x5700 = custom model; 0x0000 = OpenCR
FIRMWARE_VERSION    = 1

# TurtleBot3 Burger physical parameters
WHEEL_RADIUS        = 0.033     # metres
WHEEL_SEPARATION    = 0.160     # metres

# Open-loop speed calibration:
#   A throttle of 1.0 (100% PWM) corresponds to this wheel speed.
#   Tune this to match your actual DC motors.
MAX_WHEEL_SPEED_MS  = 0.22      # m/s at full throttle

# Motor direction flags — flip if your wiring is reversed
LEFT_MOTOR_REVERSED  = False
RIGHT_MOTOR_REVERSED = True     # Robo Pico M2 faces opposite direction

# Odometry update period (seconds)
ODOM_DT = 0.020  # 50 Hz

# Battery model (linear fit between min/max cell voltage)
BATT_MIN_V = 3.0    # per-cell cutoff
BATT_MAX_V = 4.2    # per-cell full
BATT_CELLS = 1      # 1-cell LiPo (adjust for your battery pack)

# Simulated gravity for IMU linear acceleration Z (m/s²) — used when no
# physical IMU is attached.  Set to 0.0 to report all-zeros.
IMU_GRAVITY_Z = 9.81

# ============================================================
# DERIVED CONSTANTS  (do not edit)
# ============================================================

# Dynamixel XL430 velocity units:  1 unit = 0.229 RPM
#   wheel_speed_ms = rpm_value * RPM_TO_MS
#   rpm_value      = wheel_speed_ms / RPM_TO_MS
RPM_TO_MS    = 0.229 * 0.0034557519189487725   # ≈ 7.91767e-4
RPM_PER_MS   = 1.0 / RPM_TO_MS                # ≈ 1263.0

# Dynamixel position tick:  TICK_TO_RAD = 0.001533981 rad/tick
#   (4096 ticks per revolution ≈ 2π / 4096)
TICK_TO_RAD  = 0.001533981
TICKS_PER_RAD = 1.0 / TICK_TO_RAD             # ≈ 652.0

# ============================================================
# INLINE DC MOTOR DRIVER  (no external library needed)
# Replicates adafruit_motor.DCMotor: throttle +1.0…-1.0, None=coast
# ============================================================

class _DCMotor:
    """Minimal DC motor driver using two PWM channels (IN1 / IN2)."""

    def __init__(self, pwm_a: pwmio.PWMOut, pwm_b: pwmio.PWMOut):
        self._a = pwm_a
        self._b = pwm_b
        self._throttle = 0.0

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, val):
        if val is None:
            # Coast
            self._a.duty_cycle = 0
            self._b.duty_cycle = 0
            self._throttle = None
            return
        val = max(-1.0, min(1.0, float(val)))
        self._throttle = val
        duty = int(abs(val) * 65535)
        if val > 0:
            self._a.duty_cycle = duty
            self._b.duty_cycle = 0
        elif val < 0:
            self._a.duty_cycle = 0
            self._b.duty_cycle = duty
        else:
            # Brake (both high)
            self._a.duty_cycle = 65535
            self._b.duty_cycle = 65535


# ============================================================
# PIN INITIALISATION
# ============================================================

# DC motor PWM outputs
_m1a_pwm = pwmio.PWMOut(board.GP8,  frequency=10000)
_m1b_pwm = pwmio.PWMOut(board.GP9,  frequency=10000)
_m2a_pwm = pwmio.PWMOut(board.GP10, frequency=10000)
_m2b_pwm = pwmio.PWMOut(board.GP11, frequency=10000)

_motor_left  = _DCMotor(_m1a_pwm, _m1b_pwm)
_motor_right = _DCMotor(_m2a_pwm, _m2b_pwm)

_motor_left.throttle  = 0.0
_motor_right.throttle = 0.0

# Push buttons (active-LOW with pull-up)
_btn1 = digitalio.DigitalInOut(board.GP20)
_btn2 = digitalio.DigitalInOut(board.GP21)
_btn1.direction = digitalio.Direction.INPUT
_btn2.direction = digitalio.Direction.INPUT
_btn1.pull = digitalio.Pull.UP
_btn2.pull = digitalio.Pull.UP

# Buzzer
_buzzer_pwm = pwmio.PWMOut(board.GP22, frequency=440, variable_frequency=True)
_buzzer_pwm.duty_cycle = 0

# Battery ADC — GP29 = ADC3 = VSYS on Pico / Pico 2
# The Pico has a 3:1 resistor divider on VSYS, so:
#   V_sys = adc_value * 3 * 3.3 / 65535
_vsys_adc = analogio.AnalogIn(board.A0 if hasattr(board, 'A0') else board.GP26)
# Fallback: use GP29 directly if the board definition exposes it
try:
    _vsys_adc_vsys = analogio.AnalogIn(board.GP29)
    _have_vsys = True
except AttributeError:
    _have_vsys = False

# USB CDC serial port
# boot.py disables the REPL console and exposes only the data port.
# If data port is None (original boot / not yet reset), fall back to console.
_serial = usb_cdc.data if (usb_cdc.data is not None) else usb_cdc.console
if _serial is None:
    raise RuntimeError("No usable USB CDC serial port found")
_serial.timeout = 0  # non-blocking

# ============================================================
# DYNAMIXEL PROTOCOL 2.0 CRC-16
# ============================================================

def _make_crc_table():
    t = []
    for i in range(256):
        crc  = 0
        data = i << 8
        for _ in range(8):
            if (crc ^ data) & 0x8000:
                crc = ((crc << 1) ^ 0x8005) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
            data = (data << 1) & 0xFFFF
        t.append(crc)
    return t

_CRC_TABLE = _make_crc_table()


def _crc16(data: bytes) -> int:
    """Compute Dynamixel CRC-16 over data (excludes the CRC field itself)."""
    crc = 0
    for b in data:
        i = ((crc >> 8) ^ b) & 0xFF
        crc = ((crc << 8) ^ _CRC_TABLE[i]) & 0xFFFF
    return crc

# ============================================================
# CONTROL TABLE  (register map, 256 bytes)
# Addresses from control_table.hpp
# ============================================================

_REG_SIZE = 256
_regs = bytearray(_REG_SIZE)


def _r_u8(addr: int) -> int:
    return _regs[addr]

def _w_u8(addr: int, val: int):
    _regs[addr] = val & 0xFF

def _r_u16(addr: int) -> int:
    return struct.unpack_from('<H', _regs, addr)[0]

def _w_u16(addr: int, val: int):
    struct.pack_into('<H', _regs, addr, val & 0xFFFF)

def _r_i32(addr: int) -> int:
    return struct.unpack_from('<i', _regs, addr)[0]

def _w_i32(addr: int, val: int):
    struct.pack_into('<i', _regs, addr, int(val))

def _r_f32(addr: int) -> float:
    return struct.unpack_from('<f', _regs, addr)[0]

def _w_f32(addr: int, val: float):
    struct.pack_into('<f', _regs, addr, float(val))


def _init_registers():
    """Populate EEPROM-area registers with fixed values."""
    _w_u16(0, MODEL_NUMBER)         # model_number
    _w_i32(2, 0)                    # model_information
    _w_u8(6,  FIRMWARE_VERSION)     # firmware_version
    _w_u8(7,  DEVICE_ID)            # id
    _w_u8(8,  4)                    # baud_rate  (4 = 1 Mbaud in Dynamixel table)
    _w_u8(18, 0)                    # device_status = 0 (motors OK)

    # Identity quaternion for IMU orientation (no rotation)
    _w_f32(96, 1.0)   # imu_orientation_w
    _w_f32(100, 0.0)  # imu_orientation_x
    _w_f32(104, 0.0)  # imu_orientation_y
    _w_f32(108, 0.0)  # imu_orientation_z

    # Angular velocity = 0
    _w_f32(60, 0.0)   # imu_angular_velocity_x
    _w_f32(64, 0.0)   # imu_angular_velocity_y
    _w_f32(68, 0.0)   # imu_angular_velocity_z

    # Linear acceleration: only Z = g (robot is flat)
    _w_f32(72, 0.0)        # imu_linear_acceleration_x
    _w_f32(76, 0.0)        # imu_linear_acceleration_y
    _w_f32(80, IMU_GRAVITY_Z)  # imu_linear_acceleration_z

    # Magnetic field = 0 (no magnetometer)
    _w_f32(84, 0.0)   # imu_magnetic_x
    _w_f32(88, 0.0)   # imu_magnetic_y
    _w_f32(92, 0.0)   # imu_magnetic_z

    # Motor torque enabled by default
    _w_u8(149, 1)     # motor_torque_enable

    # Battery placeholder (int32, units of 0.01V / 0.01%)
    _w_i32(42, 400)   # battery_voltage (4.00 V → 400 × 0.01 V)
    _w_i32(46, 8000)  # battery_percentage (80.00 % → 8000 × 0.01 %)

    # Wheel positions / velocities start at 0
    _w_i32(120, 0)   # present_current_left
    _w_i32(124, 0)   # present_current_right
    _w_i32(128, 0)   # present_velocity_left
    _w_i32(132, 0)   # present_velocity_right
    _w_i32(136, 0)   # present_position_left
    _w_i32(140, 0)   # present_position_right


_init_registers()

# ============================================================
# MOTOR CONTROL
# ============================================================

def _set_wheel(mtr, speed_ms: float, reversed_flag: bool):
    """Drive a DCMotor at speed_ms (m/s). Positive = forward."""
    throttle = speed_ms / MAX_WHEEL_SPEED_MS
    throttle = max(-1.0, min(1.0, throttle))
    if reversed_flag:
        throttle = -throttle
    # adafruit_motor expects None for coast; we use 0 for brake-at-zero
    mtr.throttle = throttle


def _coast_all():
    _motor_left.throttle  = None
    _motor_right.throttle = None


def _brake_all():
    _motor_left.throttle  = 0.0
    _motor_right.throttle = 0.0

# ============================================================
# BUZZER
# ============================================================

_NOTES = {
    0: 0,
    1: 262, 2: 277, 3: 294, 4: 311, 5: 330, 6: 349,
    7: 370, 8: 392, 9: 415, 10: 440,
}

def _play_sound(note_id: int):
    freq = _NOTES.get(note_id, 0)
    if freq:
        _buzzer_pwm.frequency   = freq
        _buzzer_pwm.duty_cycle  = 32768   # 50 % duty
    else:
        _buzzer_pwm.duty_cycle  = 0

# ============================================================
# ODOMETRY  (dead-reckoning, open-loop)
# ============================================================

_left_pos_ticks   = 0
_right_pos_ticks  = 0
_last_odom_mono   = time.monotonic()


def _update_odometry():
    global _left_pos_ticks, _right_pos_ticks, _last_odom_mono

    now = time.monotonic()
    dt  = now - _last_odom_mono
    _last_odom_mono = now

    torque_on = _r_u8(149)  # motor_torque_enable

    if torque_on:
        lin_x = _r_i32(150) / 100.0  # m/s  (host sends linear.x * 100)
        ang_z = _r_i32(170) / 100.0  # rad/s (host sends angular.z * 100)

        v_left  = lin_x - ang_z * (WHEEL_SEPARATION / 2.0)
        v_right = lin_x + ang_z * (WHEEL_SEPARATION / 2.0)
    else:
        v_left  = 0.0
        v_right = 0.0

    # --- Drive motors -----------------------------------------------
    if torque_on:
        _set_wheel(_motor_left,  v_left,  LEFT_MOTOR_REVERSED)
        _set_wheel(_motor_right, v_right, RIGHT_MOTOR_REVERSED)
    else:
        _brake_all()

    # --- Wheel velocity (Dynamixel RPM units) -----------------------
    vel_l_rpm = int(v_left  * RPM_PER_MS)
    vel_r_rpm = int(v_right * RPM_PER_MS)

    _w_i32(128, vel_l_rpm)   # present_velocity_left
    _w_i32(132, vel_r_rpm)   # present_velocity_right

    # --- Wheel position (ticks, accumulating) -----------------------
    delta_rad_left  = (v_left  * dt) / WHEEL_RADIUS
    delta_rad_right = (v_right * dt) / WHEEL_RADIUS

    _left_pos_ticks  = _left_pos_ticks  + int(delta_rad_left  * TICKS_PER_RAD)
    _right_pos_ticks = _right_pos_ticks + int(delta_rad_right * TICKS_PER_RAD)

    # Clamp to int32 range
    if _left_pos_ticks > 0x7FFFFFFF:
        _left_pos_ticks -= 0x100000000
    elif _left_pos_ticks < -0x80000000:
        _left_pos_ticks += 0x100000000
    if _right_pos_ticks > 0x7FFFFFFF:
        _right_pos_ticks -= 0x100000000
    elif _right_pos_ticks < -0x80000000:
        _right_pos_ticks += 0x100000000

    _w_i32(136, _left_pos_ticks)   # present_position_left
    _w_i32(140, _right_pos_ticks)  # present_position_right


# ============================================================
# SENSOR UPDATES  (called each loop iteration)
# ============================================================

def _update_sensors():
    # Timekeeping
    mono_ms = int(time.monotonic() * 1000)
    mono_us = int(time.monotonic() * 1_000_000)
    _w_i32(10, mono_ms)   # millis
    _w_i32(14, mono_us)   # micros

    # Push buttons (active LOW → register value 1 when pressed)
    _w_u8(26, 0 if _btn1.value else 1)  # button_1
    _w_u8(27, 0 if _btn2.value else 1)  # button_2

    # Battery voltage via VSYS ADC
    if _have_vsys:
        raw  = _vsys_adc_vsys.value  # 0–65535
        vsys = raw * 3.0 * 3.3 / 65535.0
    else:
        raw  = _vsys_adc.value
        vsys = raw * 3.3 / 65535.0 * 2.0  # assume 2:1 divider on GP26

    pct = (vsys - BATT_MIN_V * BATT_CELLS) / (
           (BATT_MAX_V - BATT_MIN_V) * BATT_CELLS) * 100.0
    pct = max(0.0, min(100.0, pct))
    _w_i32(42, int(vsys * 100))  # battery_voltage (0.01 V units)
    _w_i32(46, int(pct * 100))   # battery_percentage (0.01 % units)

    # Sound / buzzer
    _play_sound(_r_u8(50))

    # IMU re-calibration trigger: host writes 1, we reset and write 0 back
    if _r_u8(59) == 1:
        _w_f32(96, 1.0); _w_f32(100, 0.0)
        _w_f32(104, 0.0); _w_f32(108, 0.0)
        _w_f32(60, 0.0); _w_f32(64, 0.0); _w_f32(68, 0.0)
        _w_f32(72, 0.0); _w_f32(76, 0.0); _w_f32(80, IMU_GRAVITY_Z)
        _w_u8(59, 0)   # clear the flag

# ============================================================
# DYNAMIXEL PROTOCOL 2.0 — PACKET BUILDER
# ============================================================

def _build_status(dev_id: int, error: int, data: bytes) -> bytes:
    """
    Build a Dynamixel Protocol 2.0 STATUS packet.

    Packet layout:
      FF FF FD 00  [ID]  [LEN_L] [LEN_H]  0x55  [ERR]  [DATA…]  [CRC_L] [CRC_H]

    LENGTH counts bytes from INSTRUCTION (0x55) onwards to the last CRC byte:
      LENGTH = 1 (0x55) + 1 (ERR) + len(data) + 2 (CRC) = len(data) + 4
    """
    length = len(data) + 4
    pkt = bytearray([
        0xFF, 0xFF, 0xFD, 0x00,
        dev_id,
        length & 0xFF,
        (length >> 8) & 0xFF,
        0x55,           # STATUS instruction
        error & 0xFF,
    ])
    pkt.extend(data)
    crc = _crc16(bytes(pkt))
    pkt.append(crc & 0xFF)
    pkt.append((crc >> 8) & 0xFF)
    return bytes(pkt)

# ============================================================
# DYNAMIXEL PROTOCOL 2.0 — INSTRUCTION HANDLERS
# ============================================================

def _handle_ping(dev_id: int) -> bytes:
    """
    PING response: model_number (2 bytes) + firmware_version (1 byte).
    """
    return _build_status(dev_id, 0x00, bytes([
        _regs[0], _regs[1],  # model_number  (LE)
        _regs[6],            # firmware_version
    ]))


def _handle_read(dev_id: int, params: bytes) -> bytes:
    """
    READ: params = [ADDR_L, ADDR_H, LEN_L, LEN_H]
    Response: raw bytes from control table.
    """
    if len(params) < 4:
        return _build_status(dev_id, 0x02, b'')  # length error

    addr   = params[0] | (params[1] << 8)
    length = params[2] | (params[3] << 8)

    if addr + length > _REG_SIZE:
        return _build_status(dev_id, 0x02, b'')  # out-of-range

    return _build_status(dev_id, 0x00, bytes(_regs[addr: addr + length]))


def _handle_write(dev_id: int, params: bytes) -> bytes:
    """
    WRITE: params = [ADDR_L, ADDR_H, DATA…]
    Returns an empty-data status reply.
    """
    if len(params) < 2:
        return _build_status(dev_id, 0x02, b'')

    addr = params[0] | (params[1] << 8)
    data = params[2:]

    if addr + len(data) > _REG_SIZE:
        return _build_status(dev_id, 0x02, b'')

    for i, b in enumerate(data):
        _regs[addr + i] = b

    return _build_status(dev_id, 0x00, b'')

# ============================================================
# DYNAMIXEL PROTOCOL 2.0 — PACKET PARSER (state machine)
# ============================================================

# Parser states
_PS_H1   = 0  # waiting for 0xFF
_PS_H2   = 1  # waiting for 0xFF
_PS_H3   = 2  # waiting for 0xFD
_PS_H4   = 3  # waiting for 0x00 (reserved)
_PS_ID   = 4
_PS_LENL = 5
_PS_LENH = 6
_PS_DATA = 7  # collecting remaining (LENGTH) bytes

_ps_state   = _PS_H1
_ps_id      = 0
_ps_length  = 0
_ps_buf     = bytearray()  # [ID, LEN_L, LEN_H, INST, PARAMS..., CRC_L, CRC_H]
_ps_remain  = 0


def _parse_byte(b: int):
    """
    Feed one byte into the packet parser.
    Returns a response bytes object if a complete valid packet was received,
    otherwise returns None.
    """
    global _ps_state, _ps_id, _ps_length, _ps_buf, _ps_remain

    if _ps_state == _PS_H1:
        if b == 0xFF:
            _ps_state = _PS_H2

    elif _ps_state == _PS_H2:
        if b == 0xFF:
            _ps_state = _PS_H3
        else:
            _ps_state = _PS_H1

    elif _ps_state == _PS_H3:
        if b == 0xFD:
            _ps_state = _PS_H4
        elif b == 0xFF:
            pass  # another 0xFF — stay, looking for 0xFD
        else:
            _ps_state = _PS_H1

    elif _ps_state == _PS_H4:
        if b == 0x00:
            _ps_buf   = bytearray()
            _ps_state = _PS_ID
        else:
            _ps_state = _PS_H1

    elif _ps_state == _PS_ID:
        _ps_id  = b
        _ps_buf = bytearray([b])
        _ps_state = _PS_LENL

    elif _ps_state == _PS_LENL:
        _ps_length = b
        _ps_buf.append(b)
        _ps_state = _PS_LENH

    elif _ps_state == _PS_LENH:
        _ps_length |= (b << 8)
        _ps_buf.append(b)
        _ps_remain = _ps_length   # remaining bytes = LENGTH
        _ps_state  = _PS_DATA

    elif _ps_state == _PS_DATA:
        _ps_buf.append(b)
        _ps_remain -= 1
        if _ps_remain == 0:
            _ps_state = _PS_H1
            return _dispatch_packet()

    return None


def _dispatch_packet():
    """
    Validate CRC and dispatch the completed packet to the proper handler.
    _ps_buf layout: [ID, LEN_L, LEN_H, INST, PARAMS..., CRC_L, CRC_H]
    """
    if len(_ps_buf) < 3:
        return None

    # Only respond to our device ID or broadcast (0xFE)
    dev_id = _ps_buf[0]
    if dev_id != DEVICE_ID and dev_id != 0xFE:
        return None

    # CRC covers header (4 bytes) + everything in _ps_buf except last 2 (CRC)
    full_for_crc = bytes([0xFF, 0xFF, 0xFD, 0x00]) + bytes(_ps_buf[:-2])
    crc_calc     = _crc16(full_for_crc)
    crc_received = _ps_buf[-2] | (_ps_buf[-1] << 8)

    if crc_calc != crc_received:
        return None  # CRC mismatch — discard silently

    instruction = _ps_buf[3]
    params      = bytes(_ps_buf[4:-2])   # everything between INST and CRC

    # Don't reply to broadcast writes (0xFE) — standard Dynamixel behaviour
    if dev_id == 0xFE and instruction != 0x01:
        if instruction == 0x03:
            _handle_write(DEVICE_ID, params)  # perform write but send no reply
        return None

    if instruction == 0x01:    # PING
        return _handle_ping(dev_id)
    elif instruction == 0x02:  # READ
        return _handle_read(dev_id, params)
    elif instruction == 0x03:  # WRITE
        return _handle_write(dev_id, params)
    elif instruction == 0x08:  # REBOOT
        # Send status reply, then reset
        reply = _build_status(dev_id, 0, b'')
        _serial.write(reply)
        time.sleep(0.05)   # flush
        microcontroller.reset()  # hardware reset — re-runs boot.py
    # SYNC_WRITE (0x83) / BULK_READ (0x92) can be added here if needed

    return None

# ============================================================
# MAIN LOOP
# ============================================================

def main():
    _last_odom   = time.monotonic()
    _last_sensor = time.monotonic()

    ODOM_INTERVAL   = ODOM_DT     # seconds
    SENSOR_INTERVAL = 0.050       # 20 Hz sensor refresh

    # No startup print — any ASCII output on the serial port would corrupt
    # Dynamixel binary framing.

    while True:
        now = time.monotonic()

        # ---- Odometry update ----------------------------------------
        if now - _last_odom >= ODOM_INTERVAL:
            _update_odometry()
            _last_odom = now

        # ---- Sensor register refresh --------------------------------
        if now - _last_sensor >= SENSOR_INTERVAL:
            _update_sensors()
            _last_sensor = now

        # ---- Serial Rx ----------------------------------------------
        if _serial.in_waiting:
            # Read as many bytes as available this iteration
            chunk = _serial.read(_serial.in_waiting)
            for b in chunk:
                response = _parse_byte(b)
                if response is not None:
                    _serial.write(response)


try:
    main()
except Exception as _fw_exc:
    import traceback as _tb
    try:
        with open("/error.txt", "w") as _ef:
            _tb.print_exception(type(_fw_exc), _fw_exc, _fw_exc.__traceback__,
                                file=_ef)
    except Exception:
        pass
    raise
