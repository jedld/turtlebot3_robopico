# TurtleBot3 Pico 2 Firmware (C / Pico SDK)

Bare-metal C firmware for the **Raspberry Pi Pico 2 (RP2350)** on a
**Cytron Robo Pico** carrier board. It emulates the OpenCR board interface
so that the standard TurtleBot3 ROS 2 stack (`turtlebot3_bringup`) works
unchanged, using a single USB-CDC-ACM serial port at `/dev/ttyACM0`.

---

## Why C instead of CircuitPython

The previous CircuitPython firmware had a fatal race condition:

1. `pyserial` / DynamixelSDK asserts **DTR** when opening the serial port.
2. On the REPL console port (`ttyACM0`), CircuitPython's supervisor interprets
   DTR-assert as a soft-reset trigger.
3. The firmware takes **~60 ms** to restart.
4. The DynamixelSDK times out waiting for a PING response after only **34 ms**.

The C firmware eliminates this entirely: `tud_cdc_line_state_cb()` is an
explicit no-op, so DTR changes are silently ignored. The firmware responds to
the first PING in **< 10 ms** from any point after USB enumeration.

---

## Hardware

| Item | Detail |
|---|---|
| MCU | Raspberry Pi Pico 2 (RP2350-A, dual-core Cortex-M33, 150 MHz) |
| Carrier | Cytron Robo Pico |
| USB port | `/dev/ttyACM0` — single CDC ACM, baud rate irrelevant (USB 2.0 FS) |
| USB identity | VID `0x2E8A` / PID `0x000A`, product "TurtleBot3 Pico OpenCR" |

### Pin assignments

| Signal | GPIO | Notes |
|---|---|---|
| Left motor A (forward) | GP8 | M1A |
| Left motor B (reverse) | GP9 | M1B |
| Right motor A (forward) | GP10 | M2A |
| Right motor B (reverse) | GP11 | M2B |
| Button 1 | GP20 | active-low, internal pull-up |
| Button 2 | GP21 | active-low, internal pull-up |
| Buzzer | GP22 | variable-frequency PWM |
| Battery ADC | GP29 | VSYS via 3:1 resistor divider |

---

## Firmware architecture

```
main.c
├── Configuration #defines         — pins, timings, physical params
├── Control table (regs[256])      — Dynamixel register map
│   ├── Typed accessors            — r_u16/w_u16, r_i32/w_i32, r_f32/w_f32
│   └── init_registers()           — boot defaults
├── CRC-16                         — table-driven (poly 0x8005)
├── Packet builder (build_status)  — assembles STATUS responses
├── Instruction handlers
│   ├── handle_ping()
│   ├── handle_read()
│   ├── handle_write()
│   └── handle_reboot()            — watchdog reset
├── Packet parser (parse_byte)     — 8-state machine; calls dispatch_packet()
├── Motor control                  — 10 kHz PWM on RP2350 slices
├── Buzzer                         — variable-frequency PWM, 11 notes
├── Buttons                        — GP20/GP21 active-low
├── Battery ADC                    — GP29/ADC3, 3:1 divider → V_SYS
├── Odometry (50 Hz)               — dead-reckoning, open-loop
├── Sensor updates (20 Hz)         — timekeeping, buttons, battery, buzzer, IMU
├── TinyUSB callbacks              — line_state = no-op (THE KEY FIX)
└── main()                         — init + event loop

usb_descriptors.c                  — TinyUSB CDC ACM device/config/string descriptors
tusb_config.h                      — TinyUSB build options
CMakeLists.txt                     — Pico SDK 2.x CMake build
build.sh                           — one-shot install / build / flash
```

### Main loop

```
loop:
  tud_task()                      ← USB stack (must be called every iteration)
  if 20 ms elapsed → update_odometry(dt)
  if 50 ms elapsed → update_sensors(now_us)
  if CDC bytes available:
      tud_cdc_read() → parse_byte() for each byte
```

---

## Control table (register map)

All registers are little-endian. Column **Type** matches the Dynamixel XL430
conventions used by `turtlebot3_node/control_table.hpp`.

| Address | Name | Type | Units / Notes |
|--:|---|---|---|
| 0 | MODEL_NUMBER | uint16 | `0x5700` |
| 2 | MODEL_INFORMATION | int32 | `0` |
| 6 | FIRMWARE_VERSION | uint8 | `1` |
| 7 | ID | uint8 | `200` (0xC8) |
| 8 | BAUD_RATE | uint8 | `4` = 1 Mbaud (Dynamixel code) |
| 10 | MILLIS | int32 | milliseconds since boot |
| 14 | MICROS | int32 | microseconds since boot (truncated to int32) |
| 18 | DEVICE_STATUS | uint8 | `0` = OK |
| 26 | BUTTON_1 | uint8 | `1` when pressed |
| 27 | BUTTON_2 | uint8 | `1` when pressed |
| 42 | BATTERY_VOLTAGE | int32 | millivolts × 10 (e.g. 400 = 4.00 V) |
| 46 | BATTERY_PERCENT | int32 | percent × 100 (e.g. 8000 = 80.00%) |
| 50 | SOUND | uint8 | note index 0–10; write to play, auto-clears on next sensor tick |
| 59 | IMU_RECAL | uint8 | write `1` to reset IMU registers to defaults |
| 60 | IMU_ANG_VEL_X | float32 | rad/s |
| 64 | IMU_ANG_VEL_Y | float32 | rad/s |
| 68 | IMU_ANG_VEL_Z | float32 | rad/s |
| 72 | IMU_LIN_ACC_X | float32 | m/s² |
| 76 | IMU_LIN_ACC_Y | float32 | m/s² |
| 80 | IMU_LIN_ACC_Z | float32 | m/s² (default 9.81) |
| 84 | IMU_MAG_X | float32 | µT (stubbed, always 0) |
| 88 | IMU_MAG_Y | float32 | µT |
| 92 | IMU_MAG_Z | float32 | µT |
| 96 | IMU_ORIENT_W | float32 | quaternion (default 1, 0, 0, 0) |
| 100 | IMU_ORIENT_X | float32 | quaternion |
| 104 | IMU_ORIENT_Y | float32 | quaternion |
| 108 | IMU_ORIENT_Z | float32 | quaternion |
| 120 | PRESENT_CURRENT_L | int32 | mA (stubbed, always 0) |
| 124 | PRESENT_CURRENT_R | int32 | mA |
| 128 | PRESENT_VELOCITY_L | int32 | Dynamixel RPM units (1 unit = 0.229 RPM) |
| 132 | PRESENT_VELOCITY_R | int32 | Dynamixel RPM units |
| 136 | PRESENT_POSITION_L | int32 | encoder ticks (1 tick ≈ 0.001534 rad) |
| 140 | PRESENT_POSITION_R | int32 | encoder ticks |
| 149 | MOTOR_TORQUE_ENABLE | uint8 | `1` = motors driven; `0` = coast/brake |
| 150 | CMD_LINEAR_X | int32 | **0.01 m/s units** (e.g. 10 = 0.10 m/s) |
| 170 | CMD_ANGULAR_Z | int32 | **0.01 rad/s units** (e.g. -157 = -1.57 rad/s) |

> **Addresses 152–169 and 170+ are offset from turtlebot3_node defaults.**
> If the robot does not respond to velocity commands, cross-check
> `turtlebot3_node/src/turtlebot3_node.cpp` and
> `turtlebot3_node/include/turtlebot3_node/control_table.hpp` against
> the `ADDR_CMD_*` defines in `main.c`.

---

## Physical calibration parameters

Edit these `#define`s in `main.c` before building:

| Parameter | Default | Description |
|---|---|---|
| `WHEEL_RADIUS` | `0.033f` m | Wheel outer radius |
| `WHEEL_SEPARATION` | `0.160f` m | Distance between wheel centres |
| `MAX_WHEEL_SPEED_MS` | `0.22f` m/s | Speed that maps to 100% PWM duty |
| `RIGHT_MOTOR_REVERSED` | `1` | Set to `0` if right motor turns backwards |
| `BATT_CELLS` | `1` | Number of LiPo cells in series |
| `BATT_MIN_V` | `3.0f` V | Per-cell cutoff voltage |
| `BATT_MAX_V` | `4.2f` V | Per-cell full-charge voltage |

> **Finding `MAX_WHEEL_SPEED_MS`**: command `CMD_LINEAR_X = 10` (0.10 m/s)
> and increase until the wheels just start slipping. Back off 10% for a safe
> full-throttle value.

---

## Build instructions

### Prerequisites

```bash
# ARM cross-compiler and build tools
sudo apt-get install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi \
                     libstdc++-arm-none-eabi-newlib git python3 ninja-build

# Pico SDK 2.1.1 (RP2350 requires SDK >= 2.0)
git clone --depth=1 --branch 2.1.1 \
    https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
cd ~/pico-sdk && git submodule update --init --recursive
```

### One-liner (everything)

```bash
cd turtlebot3_pico/firmware
./build.sh all          # install SDK + build + flash
```

### Manual build

```bash
cd turtlebot3_pico/firmware
export PICO_SDK_PATH=~/pico-sdk
cmake -S . -B build -DPICO_BOARD=pico2
cmake --build build -- -j$(nproc)
# Output: build/turtlebot3_pico_fw.uf2  (~45 KB)
```

### Incremental rebuild after editing main.c

```bash
cd turtlebot3_pico/firmware
cmake --build build -- -j$(nproc)
```

> `PICO_BOARD=pico2` is **required** — using `pico` (RP2040) produces an
> incompatible UF2 that will not boot on the Pico 2.

---

## Flashing

### Method 1 — build.sh (automatic)

```bash
./build.sh flash
```

The script triggers the RP2350 UF2 bootloader by briefly opening
`/dev/ttyACM0` at 1200 baud, waits for the drive to appear, then copies the
UF2. The Pico reboots automatically when the copy completes.

### Method 2 — manual UF2

1. Hold **BOOTSEL** on the Pico while plugging in USB (or send 1200-baud DTR
   to `/dev/ttyACM0` if the C firmware is already running).
2. The RP2350 mounts as a USB mass-storage drive (e.g. `/dev/sdc1`, label
   `RP2350`).
3. Copy the UF2:
   ```bash
   sudo mount /dev/sdc1 /mnt && sudo cp build/turtlebot3_pico_fw.uf2 /mnt && sync
   ```
4. The Pico reboots and `/dev/ttyACM0` reappears.

### Method 3 — picotool

```bash
picotool load build/turtlebot3_pico_fw.uf2 --force
picotool reboot
```

---

## Running TurtleBot3 bringup

```bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-03
ros2 launch turtlebot3_bringup robot.launch.py usb_port:=/dev/ttyACM0
```

Expected output:

```
[turtlebot3_node] Start Calibration of Gyro
[turtlebot3_node] IMU Calibration Done
```

Topics that should be live: `/battery_state`, `/cmd_vel`, `/imu`, `/odom`.

### Quick connectivity test (without ROS)

```python
import serial, time

def crc16(data):
    crc = 0
    for b in data:
        idx = ((crc >> 8) ^ b) & 0xFF
        crc = ((crc << 8) ^ CRC_TABLE[idx]) & 0xFFFF
    return crc

# Build and send a PING to device ID 200
port = serial.Serial('/dev/ttyACM0', timeout=0.1)
time.sleep(0.05)   # wait for USB enumeration
# PING packet: FF FF FD 00 C8 03 00 01 CRC
raw = bytes([0xFF,0xFF,0xFD,0x00,0xC8,0x03,0x00,0x01])
# ... compute CRC and append ...
# Expected response: FF FF FD 00 C8 07 00 55 00 00 57 01 <CRC>
```

---

## Project file map

```
turtlebot3_pico/firmware/
├── main.c                  — complete firmware (724 lines)
├── usb_descriptors.c       — TinyUSB CDC ACM descriptors
├── tusb_config.h           — TinyUSB build configuration
├── CMakeLists.txt          — Pico SDK 2.x CMake build
├── pico_sdk_import.cmake   — SDK helper (copied from ~/pico-sdk/external/)
├── build.sh                — install / build / flash automation
└── build/                  — CMake build tree (generated)
    ├── turtlebot3_pico_fw.uf2   ← flash this to the Pico 2
    ├── turtlebot3_pico_fw.elf
    └── ...
```

---

## Notes for future development

### Adding a new register

1. Add an `ADDR_*` define in the `CONTROL TABLE` section of `main.c`.
2. Initialise the register in `init_registers()` if needed.
3. Read/write it with `r_u16/w_u16`, `r_i32/w_i32`, or `r_f32/w_f32`.
4. Mirror the address in `turtlebot3_node/include/turtlebot3_node/control_table.hpp`
   on the host side.

### Adding a real IMU

The IMU register block (addresses 59–108) is fully wired to the ROS node.
To add a hardware IMU (e.g. MPU-6050 over I²C):

1. Add `hardware_i2c` to `target_link_libraries` in `CMakeLists.txt`.
2. Read sensor in `update_sensors()` and write results with `w_f32()`.
3. Accumulate a quaternion from the angular velocity; replace the stub
   values at `ADDR_IMU_ORIENT_*`.

### Real encoder feedback

Currently odometry is **open-loop** (dead-reckoning from commanded velocity).
To add encoder feedback:

1. Connect quadrature encoders to free GPIOs and enable the PIO quadrature
   decoder (`pico-examples/pio/quadrature_encoder`).
2. Replace the `d_rad_*` computation in `update_odometry()` with actual
   encoder deltas.
3. Update `ADDR_PRESENT_VEL_*` from encoder derivative instead of commanded
   velocity.

### Changing the Device ID

Edit `#define DEVICE_ID 200u` in `main.c` (currently `200` = `0xC8`, which
matches the OpenCR default expected by `turtlebot3_node`). Do **not** change
this without also updating the host-side configuration.

### TinyUSB version / SDK upgrade notes

- The firmware uses `tud_init(0)` (port index, not flags). The older
  `tusb_init()` no-argument form was removed in TinyUSB bundled with Pico SDK ≥ 2.0.
- `CFG_TUSB_RHPORT0_MODE` **must** be defined in `tusb_config.h` when using
  Pico SDK ≥ 2.0; without it the build fails with a confusing error about
  `CFG_TUSB_RHPORT1_MODE`.
- If upgrading the Pico SDK, regenerate `pico_sdk_import.cmake`:
  ```bash
  cp ~/pico-sdk/external/pico_sdk_import.cmake turtlebot3_pico/firmware/
  ```

### DO NOT touch `tud_cdc_line_state_cb()`

This callback is a deliberate no-op. **Any** code that resets, restarts, or
reinitialises the firmware in response to DTR/RTS changes will reintroduce
the 34ms race condition that was the original reason for replacing
CircuitPython. If you need to respond to line-state changes for another
purpose, ensure the main protocol path is never interrupted.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `/dev/ttyACM0` not present | Pico not flashed or in bootloader mode | Check `lsusb` for VID `2e8a:000a`; re-flash if needed |
| `Failed connection with Devices` in bringup | Wrong `usb_port` parameter | Confirm `usb_port:=/dev/ttyACM0` |
| `Failed connection with Devices` after re-flashing CircuitPython | DTR reset race condition | Re-flash the C firmware |
| Wheels spin wrong direction | `RIGHT_MOTOR_REVERSED` | Flip the flag for the offending motor |
| Battery shows 0% | ADC not reading VSYS | Check carrier board — Cytron Robo Pico exposes VSYS on GP29 |
| `/odom` doesn't accumulate | Motor torque disabled | Check `ADDR_MOTOR_TORQUE_EN`; write `1` to enable |
| Build error: `CFG_TUSB_RHPORT*_MODE` | Old `tusb_config.h` | Ensure `CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE \| OPT_MODE_FULL_SPEED)` is present |
| Sound plays continuously | `ADDR_SOUND` not cleared | `play_sound()` reads the register each sensor tick; write `0` to silence |
