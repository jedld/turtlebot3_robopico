# AGENTS.md — LDS-01 LIDAR Emulator Firmware

AI agent guide for the `lds01_emulator` project.  Read this before making
any changes to the firmware.

---

## What this firmware does

Emulates the **ROBOTIS HLS-LFCD-LDS01** (TurtleBot3 LDS-01) LIDAR sensor on
a **Raspberry Pi Pico 2 (RP2350)**.

| Interface | Role |
|---|---|
| UART0 GP0 (TX) 230 400 baud | LDS-01 packet stream output → LIDAR cable / ROS driver |
| USB CDC `/dev/ttyACM0` | Interactive text command port — room / position config |
| USB CDC `/dev/ttyACM1` | Binary LDS-01 packet stream — consumed by `hls_lfcd_lds_driver` |

---

## LDS-01 packet format (42 bytes, verified against hls_lfcd_lds_driver source)

The **installed ROS2 driver** (`hls_lfcd_lds_driver`) reads **2520 bytes** as one
rotation frame (60 packets × 42 bytes). It syncs on `FA A0` and then reads the
remaining 2518 bytes in one call.  The 22-byte / 4-sample format sometimes cited
in older docs is **NOT** what the driver parses.

```
Byte    Field           Notes
----    -----           -----
  0     0xFA            sync byte (constant)
  1     0xA0–0xDB       packet index: 0xA0 + pkt_idx (0–59)
  2–3   speed_cps LE    RPM × 10   (300 RPM → 3000 = 0x0BB8)
  4–9   sample[0]       6 bytes — see below
 10–15  sample[1]
 16–21  sample[2]
 22–27  sample[3]
 28–33  sample[4]
 34–39  sample[5]
 40–41  checksum LE     LFCD algorithm over bytes 0–39
```

**6 bytes per sample** (layout verified from driver source):
```
  byte 0–1: intensity (uint16 LE)   — emulator sends QUALITY_NOMINAL=200
  byte 2–3: range     (uint16 LE)   — distance in mm
  byte 4–5: reserved  (0x00, 0x00)
```

**Angular mapping** — driver uses reversed indexing:
```
  scan_index  = 6 × pkt_idx + sample_k          (0–359)
  scan->ranges[359 − scan_index] = range / 1000.0
```
So *emulator* must compute: `angle_deg = 359 − scan_index`.

**RPM calculation** in driver:
```
  rpms = Σ(speed_cps over good_sets) / good_sets / 10
       = speed_cps / 10
  ⇒  speed_cps = RPM × 10
```

**Motor control bytes**: the driver sends `'b'` (0x62) on port open and `'e'`
(0x65) on port close. The emulator drains the CDC1 RX buffer silently.

**Checksum** (same LFCD algorithm, over 40 bytes instead of 20):
```c
uint32_t acc = 0;
for (int i = 0; i < 40; i += 2)
    acc = (acc << 1) + pkt[i] + ((uint32_t)pkt[i+1] << 8);
uint16_t chk = ((acc & 0x7FFF) + (acc >> 15)) & 0x7FFF;
// Note: driver does NOT validate checksums (only syncs on FA + A0+idx)
```

---

## Hard constraints — do not violate

### 1. Target board is `pico2` (RP2350), NOT `pico` (RP2040)

CMake: `-DPICO_BOARD=pico2`.  Flashing a `pico` UF2 to the Pico 2 will
brick the device until BOOTSEL is used manually.

### 2. Use `tud_init(0)`, not `tusb_init()`

Pico SDK ≥ 2.0 removed the no-argument `tusb_init()`.  Always call
`tud_init(0)`.

### 3. `tud_cdc_line_state_cb()` must remain a no-op

```cpp
extern "C" void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf; (void)dtr; (void)rts;
}
```

Libraries like pyserial assert DTR when they open the port.  Any reset
logic here would restart the firmware mid-scan and corrupt the LIDAR data
stream.

### 4. `CFG_TUSB_RHPORT0_MODE` must be defined in `tusb_config.h`

```c
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
```

TinyUSB 2.x fails with a cryptic error without it.

### 5. `pico_enable_stdio_usb` and `pico_enable_stdio_uart` must be `0`

If either is `1` the SDK hooks printf/stdin into the CDC interface, which
corrupts both the binary UART LDS stream and the text CDC config port.

### 6. UART0 baud rate is exactly 230 400

The LIDAR ROS drivers (`hls_lfcd_lds_driver`, `ld08_driver`) hard-code 230 400.
Do not change `LDS_BAUD`.

---

## Key constants

| Constant | Value | Meaning |
|---|---|---|
| `LDS_BAUD` | 230 400 | UART baud rate |
| `LDS_PACKETS` | 90 | Packets per full rotation |
| `LDS_IDX_FIRST` | 0xA0 | Index byte of packet 0 (0° / forward) |
| `LDS_MIN_MM` | 120 | Minimum valid distance (mm) |
| `LDS_MAX_MM` | 3 500 | Maximum valid distance (mm) |
| `QUALITY_NOMINAL` | 700 | Synthetic signal quality value |

---

## Adding room obstacles

To add furniture / obstacles beyond the four walls, modify `ray_distance_mm()`
in `main.cpp`.  The function currently finds the minimum positive ray-parameter
`t` for the four walls.  Add additional geometric primitives (circles,
rectangles) using the same `t_min` comparison pattern.

Example — circular pillar at (cx, cy) with radius r:
```cpp
// Circle intersection: ||(dx,dy)*t + (rx-cx, ry-cy)||² = r²
// Solve: ||(dx,dy)||²*t² + 2*(dx*(rx-cx)+dy*(ry-cy))*t + ((rx-cx)²+(ry-cy)²-r²) = 0
float ox = g_robot_x - cx, oy = g_robot_y - cy;
float a = dx*dx + dy*dy;
float b = 2.0f*(dx*ox + dy*oy);
float c = ox*ox + oy*oy - r*r;
float disc = b*b - 4.0f*a*c;
if (disc >= 0.0f) {
    float t = (-b - sqrtf(disc)) / (2.0f*a);
    if (t > 0.0f && t < t_min) t_min = t;
}
```

---

## Adding CDC commands

1. Add a new `else if (sscanf(...) == …)` branch in `process_command()`.
2. Add the command to the `HELP` string in the same function.
3. Add a row to the **STATUS** output if it exposes persistent state.

---

## Build

```bash
cd turtlebot3_pico/lds01_emulator
./build.sh install    # first time — installs SDK and toolchain
./build.sh            # rebuild
./build.sh flash      # rebuild + flash
```

Output: `build/lds01_emulator.uf2`
