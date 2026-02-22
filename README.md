# TurtleBot3 Pi Pico 2 Firmware

Turns a **Raspberry Pi Pico 2** on a **Cytron Robo Pico** board into a drop-in
replacement for the TurtleBot3 OpenCR board.  The firmware implements the
**Dynamixel Protocol 2.0** slave interface that `turtlebot3_node` expects,
handling motor commands, odometry feedback, and simulated IMU data.

---

## How it works

```
Host (Raspberry Pi / PC)                Pi Pico 2 (Cytron Robo Pico)
┌─────────────────────────────┐         ┌──────────────────────────────────┐
│  turtlebot3_node (ROS 2)    │  USB    │  CircuitPython firmware          │
│                             │◄───────►│                                  │
│  Dynamixel SDK              │         │  Dynamixel Protocol 2.0 slave    │
│  /dev/ttyACM1  1 Mbaud      │         │  Control table (register map)    │
│                             │         │                                  │
│  Publishes:                 │         │  GP8/GP9   → Left  wheel (M1)    │
│   /joint_states             │         │  GP10/GP11 → Right wheel (M2)    │
│   /odom                     │         │  GP20      → Button 1            │
│   /imu                      │         │  GP21      → Button 2            │
│   /battery_state            │         │  GP22      → Buzzer              │
│                             │         │  GP29      → Battery ADC         │
│  Subscribes:                │         │                                  │
│   /cmd_vel                  │         │  Dead-reckoning odometry         │
└─────────────────────────────┘         └──────────────────────────────────┘
```

`turtlebot3_node` reads / writes a flat **control table** over Dynamixel
Protocol 2.0 at 1 Mbaud (USB CDC, so the baud rate is advisory).  The
firmware maintains that register map and executes motor commands as they
arrive.

---

## Prerequisites

| Item | Details |
|---|---|
| Raspberry Pi Pico 2 | RP2350-based board |
| Cytron Robo Pico | Carrier with dual DC motor driver |
| CircuitPython ≥ 9.x | For **RP2350** — download from circuitpython.org |
| adafruit_motor library | From the Adafruit CircuitPython Bundle |

---

## Step-by-step setup

### 1 — Flash CircuitPython

1. Hold **BOOTSEL** on the Pico 2 and plug it into your PC.  A `RPI-RP2`
   mass-storage drive appears.
2. Download the latest **Pico 2 (RP2350)** CircuitPython UF2 from
   [circuitpython.org/board/raspberry_pi_pico2](https://circuitpython.org/board/raspberry_pi_pico2).
3. Drag the `.uf2` file onto the drive.  It will reboot as `CIRCUITPY`.

### 2 — Install the motor library

1. Download the [Adafruit CircuitPython Bundle](https://github.com/adafruit/Adafruit_CircuitPython_Bundle/releases).
2. Copy `adafruit_motor/` from the bundle into the `CIRCUITPY/lib/` directory.

### 3 — Deploy the firmware

```bash
# From this directory:
cp boot.py main.py /media/$USER/CIRCUITPY/
```

Or drag-and-drop both files onto the `CIRCUITPY` drive.  The Pico reboots
automatically.

### 4 — Verify the two serial ports appear

```bash
ls -l /dev/ttyACM*
# Expect: /dev/ttyACM0  (REPL)  and  /dev/ttyACM1  (Dynamixel data)
```

If only one port appears, the `boot.py` may not have been picked up yet —
press the reset button or power-cycle the board.

---

## ROS 2 configuration

### burger.yaml (already in src/turtlebot3/turtlebot3_bringup/param/)

The only change required is the serial port:

```yaml
opencr:
  id: 200
  baud_rate: 1000000          # USB CDC ignores this on the device side
  protocol_version: 2.0
```

### Launch TurtleBot3

```bash
# In your turtlebot3_ws:
source install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_bringup robot.launch.py \
  usb_port:=/dev/ttyACM1      # <-- the data CDC port
```

Expected startup sequence:
1. `turtlebot3_node` sends a READ to address 0 to verify connection.
2. It writes `1` to `imu_re_calibration` (address 59) and waits **5 seconds**
   — the firmware resets the simulated IMU registers and clears the flag.
3. Node starts publishing `/joint_states`, `/imu`, `/battery_state`, etc.

---

## Motor calibration

The firmware uses **open-loop PWM** — it has no wheel encoders.  You must
tune `MAX_WHEEL_SPEED_MS` in `main.py` to match your actual motors:

```python
MAX_WHEEL_SPEED_MS = 0.22   # m/s when PWM throttle = 1.0
```

**Procedure:**
1. Command `linear.x = 0.10` (10 cm/s).
2. Measure the actual distance travelled in 2 seconds.
3. Adjust `MAX_WHEEL_SPEED_MS` until measured matches commanded, then rebuild.

If one wheel spins the wrong way, flip the relevant flag:

```python
LEFT_MOTOR_REVERSED  = False   # Set True to invert left  wheel
RIGHT_MOTOR_REVERSED = True    # Set True to invert right wheel
```

---

## Adding a real IMU (optional but recommended)

Without a physical IMU `use_imu: true` in `burger.yaml` will still publish
an identity quaternion and zero angular velocity.  For accurate heading,
attach an **MPU-6050** or **ICM-42688** breakout to the Pico's I²C bus and
extend the `_update_sensors()` function:

```python
# Example stub — add to main.py after the pin definitions:
import busio
from adafruit_mpu6050 import MPU6050

i2c   = busio.I2C(board.GP3, board.GP2)  # SCL, SDA
_imu  = MPU6050(i2c)

# Inside _update_sensors():
ax, ay, az = _imu.acceleration
gx, gy, gz = _imu.gyro
_w_f32(60, gx); _w_f32(64, gy); _w_f32(68, gz)
_w_f32(72, ax); _w_f32(76, ay); _w_f32(80, az)
```

Install `adafruit_mpu6050` from the CircuitPython bundle into `CIRCUITPY/lib/`.

---

## Wheel encoders (optional)

The Cytron Robo Pico does not expose dedicated encoder inputs, but any two
spare GPIO pins can be used as interrupt-driven quadrature counters via
`countio.Counter`:

```python
import countio

# Example — wire encoder A to GP18, B to GP19
enc_left  = countio.Counter(board.GP18, edge=countio.Edge.RISE)
enc_right = countio.Counter(board.GP19, edge=countio.Edge.RISE)

# In _update_odometry(), replace dead-reckoning with:
_left_pos_ticks  = enc_left.count
_right_pos_ticks = enc_right.count
```

---

## Recovering from a bad firmware flash

If the Pico is unresponsive:
1. Hold **BOOTSEL** and plug in USB → mass-storage mode.
2. Replace `main.py` with a minimal (blank) script, or delete both
   `boot.py` and `main.py`.

---

## Control table reference

| Address | Size | Access | Field |
|---------|------|--------|-------|
| 0 | 2 | R | `model_number` |
| 6 | 1 | R | `firmware_version` |
| 10 | 4 | R | `millis` |
| 14 | 4 | R | `micros` |
| 18 | 1 | R | `device_status` |
| 19 | 1 | R/W | `heartbeat` |
| 26 | 1 | R | `button_1` |
| 27 | 1 | R | `button_2` |
| 42 | 4 | R | `battery_voltage` (float, V) |
| 46 | 4 | R | `battery_percentage` (float, %) |
| 50 | 1 | R/W | `sound` (note id) |
| 59 | 1 | R/W | `imu_re_calibration` |
| 60–68 | 4×3 | R | `imu_angular_velocity_xyz` (float, rad/s) |
| 72–80 | 4×3 | R | `imu_linear_acceleration_xyz` (float, m/s²) |
| 84–92 | 4×3 | R | `imu_magnetic_xyz` (float, T) |
| 96–108 | 4×4 | R | `imu_orientation_wxyz` (float) |
| 128–132 | 4×2 | R | `present_velocity_left/right` (int32, Dyn. RPM) |
| 136–140 | 4×2 | R | `present_position_left/right` (int32, ticks) |
| 149 | 1 | R/W | `motor_torque_enable` |
| 150 | 4 | R/W | `cmd_velocity_linear_x` (int32, × 100 m/s) |
| 170 | 4 | R/W | `cmd_velocity_angular_z` (int32, × 100 rad/s) |
| 174–178 | 4×2 | R/W | `profile_acceleration_left/right` |
