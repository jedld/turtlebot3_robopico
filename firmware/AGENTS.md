# AGENTS.md — AI Agent Development Guide

This file is for AI coding assistants (Copilot, Claude, GPT, etc.) working on
the TurtleBot3 Pico 2 firmware. Read this before making any changes.

---

## Project context in one paragraph

This firmware replaces a CircuitPython implementation that failed because
opening `/dev/ttyACM0` asserts DTR, CircuitPython's REPL supervisor treated
DTR-assert as a soft-reset trigger (~60 ms), and the DynamixelSDK times out
in 34 ms. The C firmware eliminates this by making `tud_cdc_line_state_cb()`
a deliberate no-op. Everything else is a Dynamixel Protocol 2.0 slave that
mirrors the OpenCR register map so `turtlebot3_bringup` works unchanged.

Also the reference ROBO PICO SDK can be found in Cytron-ROBO-PICO to help with the Dynamixel bridge.

---

## Hard constraints — do not violate these

### 1. `tud_cdc_line_state_cb()` must remain a no-op

```c
// CORRECT — keep exactly as-is
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf; (void)dtr; (void)rts;
}
```

Any code in this callback that resets, restarts, or re-inits the firmware
reintroduces the 34 ms race condition. The `(void)` casts suppress
compiler warnings — do not remove them.

### 2. Target board is `pico2` (RP2350), NOT `pico` (RP2040)

CMake build flag: `-DPICO_BOARD=pico2` (already in `CMakeLists.txt`).
Generating a `pico` UF2 and flashing it to the Pico 2 will brick the
device until BOOTSEL is used manually.

### 3. Use `tud_init(0)`, not `tusb_init()`

In Pico SDK ≥ 2.0 / TinyUSB ≥ 0.16, the no-argument `tusb_init()` form is
gone. The firmware uses `tud_init(0)` (port 0). Do not revert this.

### 4. `CFG_TUSB_RHPORT0_MODE` must be defined in `tusb_config.h`

```c
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
```

Without it the TinyUSB 2.x build fails with a misleading error about
`CFG_TUSB_RHPORT1_MODE`.

### 5. `pico_enable_stdio_usb` and `pico_enable_stdio_uart` must stay `0`

If either is `1`, the Pico SDK hooks its own printf/stdin into the CDC
interface, which corrupts the Dynamixel binary protocol.

---

## Key data encodings — things that are easy to get wrong

| Register | Storage | Example |
|---|---|---|
| `CMD_LINEAR_X` (addr 150) | int32, × 100 of m/s | `10` → 0.10 m/s |
| `CMD_ANGULAR_Z` (addr 170) | int32, × 100 of rad/s | `-157` → −1.57 rad/s |
| `BATTERY_VOLTAGE` (addr 42) | int32, × 100 of volts | `400` → 4.00 V |
| `BATTERY_PERCENT` (addr 46) | int32, × 100 of percent | `8000` → 80.00% |
| `PRESENT_VELOCITY_*` (128/132) | int32, Dynamixel RPM units | 1 unit = 0.229 RPM |
| `PRESENT_POSITION_*` (136/140) | int32, encoder ticks | 1 tick ≈ 0.001534 rad |
| IMU fields (60–108) | float32 (IEEE 754 LE) | written with `w_f32()` |

The `r_i32` / `w_i32` / `r_f32` / `w_f32` helpers handle endianness; always
use them rather than writing `regs[]` bytes directly for multi-byte types.

---

## How to add a new register

1. Add `#define ADDR_MY_REG  <address>u` in the `CONTROL TABLE` section of
   `main.c`, grouped by address range.
2. If it needs a non-zero default, call `w_u16/w_i32/w_f32` in
   `init_registers()`.
3. Read/write it in `update_sensors()` or `update_odometry()` as appropriate.
4. Mirror the address constant in
   `turtlebot3_node/include/turtlebot3_node/control_table.hpp` on the host.
5. Rebuild with `cmake --build build -- -j$(nproc)`.

---

## How to build and flash

```bash
# Rebuild only (fastest after editing main.c)
cd /home/jedld/turtlebot3_ws/turtlebot3_pico/firmware
cmake --build build -- -j$(nproc)

# Full reconfigure + build
cmake -S . -B build -DPICO_BOARD=pico2 -DPICO_SDK_PATH=$HOME/pico-sdk
cmake --build build -- -j$(nproc)

# Flash (triggers bootloader automatically)
./build.sh flash
```

Build output: `build/turtlebot3_pico_fw.uf2` (~45 KB).

---

## How to verify the firmware without ROS

Send a raw PING and inspect the response:

```python
import serial, struct, time

def crc16_dxl(data: bytes) -> int:
    CRC_POLY = 0x8005
    crc = 0
    for b in data:
        for _ in range(8):
            if (crc ^ (b << 8)) & 0x8000:
                crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
            b = (b << 1) & 0xFF
    return crc

def ping(port_path: str, device_id: int = 200) -> bytes:
    header = bytes([0xFF, 0xFF, 0xFD, 0x00, device_id, 0x03, 0x00, 0x01])
    crc = crc16_dxl(header)
    pkt = header + struct.pack('<H', crc)
    with serial.Serial(port_path, timeout=0.2) as s:
        time.sleep(0.05)
        s.write(pkt)
        return s.read(100)

resp = ping('/dev/ttyACM0')
print(resp.hex())
# Expected: fffffd00c80700550000570154a5
#           ^^^^^^^^^^^^^^^^ header, ID=0xC8, LEN=7, STATUS=0x55, ERR=0
#                                             ^^^^^^^^^ model=0x5700, FW=1
```

---

## Bringup command

```bash
TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-03 \
    ros2 launch turtlebot3_bringup robot.launch.py usb_port:=/dev/ttyACM0
```

Healthy bringup prints `Start Calibration of Gyro` then `IMU Calibration Done`
and all four topics are live: `/battery_state` `/cmd_vel` `/imu` `/odom`.

---

## Source file roles

| File | Role | Edit frequency |
|---|---|---|
| `main.c` | All firmware logic — registers, protocol, motors, odometry | Often |
| `usb_descriptors.c` | USB VID/PID, endpoint sizes, product strings | Rarely |
| `tusb_config.h` | TinyUSB compile-time configuration | Rarely |
| `CMakeLists.txt` | Build targets and library links | When adding .c files |
| `pico_sdk_import.cmake` | SDK bootstrap — copy from `~/pico-sdk/external/` | On SDK upgrade |
| `build.sh` | Developer convenience script | Rarely |

---

## Common errors and fixes

| Error message | Root cause | Fix |
|---|---|---|
| `CFG_TUSB_RHPORT0_MODE must be defined` | Missing define in `tusb_config.h` | Add `#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE \| OPT_MODE_FULL_SPEED)` |
| `undefined reference to tusb_init` | Old `tusb_init()` call | Change to `tud_init(0)` |
| PING times out at < 34 ms | `tud_cdc_line_state_cb` doing a reset | Make it a no-op |
| UF2 doesn't boot | Wrong board target | Rebuild with `-DPICO_BOARD=pico2` |
| `pico_sdk_import.cmake` not found | File missing from firmware dir | `cp ~/pico-sdk/external/pico_sdk_import.cmake .` |
| Garbled Dynamixel responses | stdio USB enabled | Set `pico_enable_stdio_usb 0` |
