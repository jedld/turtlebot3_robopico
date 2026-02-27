# TurtleBot3 Pi Pico 2 Firmware (C / Pico SDK)

Turns a **Raspberry Pi Pico 2** on a **Cytron Robo Pico** board into a drop-in
replacement for the TurtleBot3 OpenCR board.  The firmware implements the
**Dynamixel Protocol 2.0** slave interface that `turtlebot3_node` expects,
handling motor commands, odometry feedback, and simulated/real IMU data.

> **Note:** The firmware is written in C using the Pico SDK 2.x (not
> CircuitPython).  See [firmware/README.md](firmware/README.md) for the full
> architecture description.

---

## How it works

```
Host (Raspberry Pi / PC)                Pi Pico 2 (Cytron Robo Pico)
┌─────────────────────────────┐         ┌──────────────────────────────────┐
│  turtlebot3_node (ROS 2)    │  USB    │  C firmware (Pico SDK 2.x)      │
│                             │◄───────►│                                  │
│  Dynamixel SDK              │         │  Dynamixel Protocol 2.0 slave    │
│  /dev/ttyACM0  (USB CDC)    │         │  Control table (register map)    │
│                             │         │                                  │
│  Publishes:                 │         │  GP8/GP9   → Left  wheel (M1)   │
│   /joint_states             │         │  GP10/GP11 → Right wheel (M2)   │
│   /odom                     │         │  GP20      → Button 1           │
│   /imu                      │         │  GP21      → Button 2           │
│   /battery_state            │         │  GP22      → Buzzer             │
│                             │         │  GP28      → Battery ADC        │
│  Subscribes:                │         │  GP0/GP1   → BNO085 IMU (I2C)  │
│   /cmd_vel                  │         │                                  │
└─────────────────────────────┘         │  Dead-reckoning odometry         │
                                        └──────────────────────────────────┘
```

`turtlebot3_node` reads / writes a flat **control table** over Dynamixel
Protocol 2.0 (USB CDC, so the baud rate is advisory).  The firmware maintains
that register map and executes motor commands as they arrive.

---

## Prerequisites

| Item | Details |
|---|---|
| Raspberry Pi Pico 2 | RP2350-based board |
| Cytron Robo Pico | Carrier with dual DC motor driver (DRV8833) |
| Pico SDK 2.x | Installed via `firmware/build.sh install` |
| arm-none-eabi-gcc | Cross-compiler (installed automatically by build script) |
| Optional: BNO085 | 9-DoF IMU on Grove 1 port (GP0/GP1, I2C0) |

---

## Building and flashing

### Quick start

```bash
cd turtlebot3_pico/firmware

# First time: install SDK + toolchain, build, and flash
./build.sh all

# Subsequent builds
./build.sh build

# Build + flash (via picotool or UF2 bootloader)
./build.sh flash
```

The flashable image is `firmware/build/turtlebot3_pico_fw.uf2`.

### Manual flash (BOOTSEL)

1. Hold **BOOTSEL** on the Pico 2 and plug in USB → mass-storage drive appears.
2. Copy `firmware/build/turtlebot3_pico_fw.uf2` onto the drive.
3. The Pico reboots automatically and appears as `/dev/ttyACM0`.

### Remote flash (1200-baud reset)

If the firmware is already running, `build.sh flash` can trigger a reboot
into bootloader mode by opening the serial port at 1200 baud.

---

## Verify

After flashing, exactly one serial port should appear:

```bash
ls -l /dev/ttyACM*
# Expect: /dev/ttyACM0  (Dynamixel data port)
```

---

## ROS 2 configuration

### burger.yaml (in src/turtlebot3/turtlebot3_bringup/param/)

```yaml
opencr:
  id: 200
  baud_rate: 1000000          # USB CDC ignores this on the device side
  protocol_version: 2.0
```

### Launch TurtleBot3

```bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_bringup robot.launch.py \
  usb_port:=/dev/ttyACM0
```

Expected startup sequence:
1. `turtlebot3_node` sends a READ to address 0 to verify connection.
2. It writes `1` to `imu_re_calibration` (address 59) and waits **5 seconds**
   — the firmware resets the simulated IMU registers and clears the flag.
3. Node starts publishing `/joint_states`, `/imu`, `/battery_state`, etc.

---

## Motor calibration

The firmware uses **open-loop PWM** — it has no wheel encoders.  Tune
`MAX_WHEEL_SPEED_MS` in `firmware/main.c`:

```c
#define MAX_WHEEL_SPEED_MS  0.22f    // m/s at full throttle
```

**Procedure:**
1. Command `linear.x = 0.10` (10 cm/s).
2. Measure the actual distance travelled in 2 seconds.
3. Adjust `MAX_WHEEL_SPEED_MS` until measured matches commanded, then rebuild.

If one wheel spins the wrong way, flip the relevant flag in `main.c`:

```c
#define RIGHT_MOTOR_REVERSED 1   // set to 0 to swap direction
```

---

## Adding a real IMU (BNO085)

The firmware automatically detects a **BNO085** IMU on the Grove 1 I2C port
(GP0 = SDA, GP1 = SCL).  If present, it replaces the simulated IMU data with
live rotation vector, accelerometer, gyroscope, and magnetometer readings at
50 Hz.

If the BNO085 is absent, the firmware falls back to a static simulated IMU
(identity quaternion, Z-gravity only).

---

## Recovering from a bad firmware flash

If the Pico is unresponsive:
1. Hold **BOOTSEL** and plug in USB → mass-storage mode.
2. Flash a known-good UF2 (or the Pico SDK blinky example to test).

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
| 42 | 4 | R | `battery_voltage` (int32, V × 100) |
| 46 | 4 | R | `battery_percentage` (int32, % × 100) |
| 50 | 1 | R/W | `sound` (melody id) |
| 59 | 1 | R/W | `imu_re_calibration` |
| 60–68 | 4×3 | R | `imu_angular_velocity_xyz` (float, rad/s) |
| 72–80 | 4×3 | R | `imu_linear_acceleration_xyz` (float, m/s²) |
| 84–92 | 4×3 | R | `imu_magnetic_xyz` (float, T) |
| 96–108 | 4×4 | R | `imu_orientation_wxyz` (float, quaternion) |
| 120–124 | 4×2 | R | `present_current_left/right` (int32) |
| 128–132 | 4×2 | R | `present_velocity_left/right` (int32, Dyn. RPM units) |
| 136–140 | 4×2 | R | `present_position_left/right` (int32, ticks) |
| 149 | 1 | R/W | `motor_torque_enable` |
| 150 | 4 | R/W | `cmd_velocity_linear_x` (int32, m/s × 100) |
| 154–168 | 4×4 | R/W | `cmd_velocity_{linear_y,linear_z,angular_x,angular_y}` |
| 170 | 4 | R/W | `cmd_velocity_angular_z` (int32, rad/s × 100) |
| 174–178 | 4×2 | R/W | `profile_acceleration_left/right` |

---

## File structure

```
turtlebot3_pico/
├── firmware/              ← C firmware (Pico SDK 2.x)
│   ├── main.c             ← Main firmware source
│   ├── usb_descriptors.c  ← TinyUSB CDC device descriptors
│   ├── tusb_config.h      ← TinyUSB build options
│   ├── CMakeLists.txt     ← Pico SDK CMake build
│   ├── build.sh           ← One-shot install / build / flash
│   └── README.md          ← Detailed architecture docs
├── lds01_emulator/        ← LDS-01 LiDAR emulator (optional)
├── flash_and_bringup.sh   ← Build, flash, and launch ROS 2
├── test_bringup.sh        ← Post-bringup verification
├── 99-turtlebot3-pico.rules ← udev rules
└── README.md              ← This file
```
