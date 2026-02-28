/*
 * main.c — TurtleBot3 Pi Pico 2 Firmware (C / Pico SDK)
 *
 * Emulates the OpenCR board interface for TurtleBot3 using a
 * Raspberry Pi Pico 2 (RP2350) on a Cytron Robo Pico carrier board.
 *
 * Protocol : Dynamixel Protocol 2.0 (slave / device side)
 * Interface: USB CDC ACM → /dev/ttyACM0 on the host
 * Baud rate: USB 2.0 full-speed (12 Mbit/s, latency << 1 ms)
 *
 * Key advantage over CircuitPython implementation
 * ------------------------------------------------
 * tud_cdc_line_state_cb() is a deliberate no-op: asserting or dropping
 * DTR (which pyserial / the DynamixelSDK does when opening the port)
 * never triggers a firmware reset.  The firmware is ready to respond
 * within ~20 ms of USB enumeration — well inside the SDK's 34 ms window.
 *
 * Hardware mapping (Cytron Robo Pico)
 * ------------------------------------
 *   Left  wheel : M1 — GP8  (M1A, forward), GP9  (M1B, reverse)
 *   Right wheel : M2 — GP10 (M2A, forward), GP11 (M2B, reverse)
 *   Button 1    : GP20 (active-low with internal pull-up)
 *   Button 2    : GP21 (active-low with internal pull-up)
 *   Buzzer      : GP22 (PWM variable-frequency)
 *   Battery ADC : GP28 / ADC2 = Vbatt sense (solder-jumper on Robo Pico back)
 *                  Divider: VADC2 = VBAT/2 (two equal resistors, per datasheet)
 *                  VBAT = VIN or VUSB or VLiPo, whichever is highest
 *   VSYS monitor : GP29 (3× divider, Pico 2 supply — not used for battery state)
 *   UPS PLD      : GP26 = Grove 6, pin 1 (Yellow) — X-UPS1 Power Loss Detection
 *                  High = mains disconnected (robot running on battery)
 *   UPS LBAT     : GP27 = Grove 6, pin 2 (White)  — X-UPS1 Low Battery signal
 *                  High = battery ≤ 3 V per cell (pack ≤ 12 V); cut-off imminent
 *
 * Register map: control_table.hpp in turtlebot3_node
 */

#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "bsp/board.h"
#include "tusb.h"

// ============================================================
// CONFIGURATION
// ============================================================

#define DEVICE_ID           200u
#define MODEL_NUMBER        0x5700u
#define FIRMWARE_VERSION    1u

// Cytron Robo Pico pin assignments
#define PIN_M1A     8u
#define PIN_M1B     9u
#define PIN_M2A     10u
#define PIN_M2B     11u
#define PIN_BTN1    20u
#define PIN_BTN2    21u
#define PIN_BUZZER  22u
#define ADC_CH_VSYS   3u      // GP29 = ADC3 = VSYS monitor (Pico 2 supply)

// X-UPS1 UPS signal pins — Grove 6 port.
// The X-UPS1 PH2.0 4-pin cable: 5 V and GND are wired to the Raspberry Pi;
// PLD (yellow) and LBAT (white) are wired to Grove 6 on the Robo Pico.
// Both signals are active-HIGH; internal pull-downs keep them low when idle.
#define PIN_UPS_PLD   26u     // GP26 = Grove 6, pin 1 (Yellow) — Power Loss Detect
#define PIN_UPS_LBAT  27u     // GP27 = Grove 6, pin 2 (White)  — Low Battery


// Pico 2 system clock — use the SDK-defined SYS_CLK_HZ (150 MHz for RP2350).
// Do not redefine; it comes from hardware/platform_defs.h.

// Motor PWM: 10 kHz, 1000 resolution steps.
// clkdiv = 15  →  150 MHz / 15 / 1000 = 10 kHz
#define MOTOR_PWM_CLKDIV    15.0f
#define MOTOR_PWM_WRAP      999u    // duty range: 0 … 999

// ============================================================
// MOTOR / WHEEL / CHASSIS CONFIGURATION
// ============================================================
// Adjust these defines when swapping motors, wheels, or chassis.
//
// Current setup:
//   Motor : Cytron TT DC Dual Metal Gearbox Motor (1:90 gear ratio, 3–6 V DC)
//           https://www.cytron.io/p-tt-motor-dual-metal-shaft
//   Wheel : Cytron Small Robot Rubber Wheel 65×26.5 mm (68.1 mm OD with tyre)
//           https://www.cytron.io/p-small-robot-rubber-wheel-65x26.5mm
//   Chassis: Custom, 13.5 cm wheel-to-wheel separation
//
// To change configuration, edit the values below and rebuild.
// The same wheel_radius and wheel_separation must also be set
// in the ROS 2 parameter file:
//   turtlebot3_bringup/param/humble/burger.yaml
// ============================================================

#define WHEEL_RADIUS        0.361910f  // metres — calibrated (723.8 mm diam. effective)
// ANGULAR CALIBRATION: do NOT use a scale factor (it breaks odometry).
// Tune WHEEL_SEPARATION to match effective turning base; run auto_calibrate_imu_turn.py.
#define WHEEL_SEPARATION    0.063355f  // metres — effective turning base; calibrated by auto_calibrate_imu_turn.py
#define MAX_WHEEL_SPEED_MS  0.336960f  // calibrated — straight_calibrate.py
#define RIGHT_MOTOR_REVERSED 0       // set to 1 if right wheel spins backward
#define SWAP_LEFT_RIGHT_MOTORS 1      // set to 1 if M1/M2 are physically wired to opposite sides

// Minimum duty cycle (dead-zone compensation).
// TT DC Dual Metal Gearbox Motor (1:90) — higher gear ratio means more
// torque at lower speed; stall-threshold duty may differ from 1:48.
// Any non-zero throttle is boosted to at least this value.
// Set to 0.0f to disable.  Range: 0.0 – 1.0.
// Re-run straight_calibrate.py if motors stall or overshoot.
#define MOTOR_MIN_DUTY      0.55f

// Kick-start: when a motor transitions from stopped to moving (or reverses
// direction), apply KICK_DUTY for KICK_CYCLES odometry cycles to overcome
// static friction, then settle to normal (dead-zone compensated) duty.
// Particularly important for pivot turns where individual wheel velocities
// are very small.  Set KICK_CYCLES to 0 to disable.
#define MOTOR_KICK_DUTY     0.60f   // 60% duty for the kick pulse
#define MOTOR_KICK_CYCLES   3u      // 3 × 20 ms = 60 ms burst

// Per-motor throttle trim — compensates for manufacturing differences between
// the left and right motors that cause the robot to drift sideways.
// Range: 0.80 – 1.20.  Keep the faster motor at 1.0 and reduce the other.
// These are computed automatically by turtlebot3_pico/straight_calibrate.py.
#define MOTOR_TRIM_LEFT    1.000000f  // calibrated — straight_calibrate.py
#define MOTOR_TRIM_RIGHT   0.700000f  // calibrated — straight_calibrate.py
#define RPM_TO_MS   (0.229f * 0.0034557519189487725f)
#define RPM_PER_MS  (1.0f / RPM_TO_MS)

// Dynamixel position tick: ~0.001534 rad/tick (4096 ticks/rev)
#define TICK_TO_RAD     0.001533981f
#define TICKS_PER_RAD   (1.0f / TICK_TO_RAD)

// Simulated IMU: gravity on Z axis (robot is flat)
#define IMU_GRAVITY_Z   9.81f

// Battery model — X-UPS1 uses a 4-cell (4S) 18650 Li-Ion pack.
// State is determined by two GPIO signals on Grove 6 (PIN_UPS_PLD / PIN_UPS_LBAT)
// rather than an ADC reading, because the X-UPS1 only exposes binary flags.
// Nominal per-state pack voltages reported to ROS:
#define BATT_CELLS   4        // 4S 18650 Li-Ion (≈ 14.8 V nominal)
#define BATT_MIN_V   3.0f     // per-cell cutoff (V)
#define BATT_MAX_V   4.2f     // per-cell full charge (V)
// Reported pack voltages for each binary state:
#define BATT_V_LOW   12.0f    // LBAT=1: any cell ≤ 3.0 V → pack ≤ 12 V
#define BATT_V_MID   14.8f    // LBAT=0, PLD=1: on battery, level unknown (> 25%)
#define BATT_V_FULL  16.8f    // LBAT=0, PLD=0: on mains / fully charged
// Reported percentages for each binary state:
#define BATT_PCT_LOW   5.0f
#define BATT_PCT_MID  50.0f
#define BATT_PCT_FULL 100.0f

// Onboard LED (active-high on Pico 2)
#define PIN_LED         25u

// Host communication timeout: motors stop if the host has not sent ANY
// valid Dynamixel packet (READ, WRITE, heartbeat, etc.) within this
// interval.  This lets the teleop keyboard work — it only publishes on
// keypress, but the ROS node's 50ms bulk-READ and 100ms heartbeat
// prove the host is alive between keypresses.
#define HOST_TIMEOUT_US     2000000u  // 2 seconds

// Update intervals (microseconds)
#define ODOM_INTERVAL_US    20000u  // 50 Hz
#define SENSOR_INTERVAL_US  50000u  // 20 Hz

// ============================================================
// CONTROL TABLE  (256-byte register map)
// ============================================================

#define REG_SIZE 256u
static uint8_t regs[REG_SIZE];

// --- typed register accessors --------------------------------

static inline uint16_t r_u16(uint addr) {
    return (uint16_t)(regs[addr] | ((uint16_t)regs[addr + 1] << 8));
}
static inline void w_u16(uint addr, uint16_t v) {
    regs[addr]     = (uint8_t)(v);
    regs[addr + 1] = (uint8_t)(v >> 8);
}

static inline int32_t r_i32(uint addr) {
    return (int32_t)((uint32_t)regs[addr]
        | ((uint32_t)regs[addr + 1] << 8)
        | ((uint32_t)regs[addr + 2] << 16)
        | ((uint32_t)regs[addr + 3] << 24));
}
static inline void w_i32(uint addr, int32_t v) {
    uint32_t u = (uint32_t)v;
    regs[addr]     = (uint8_t)(u);
    regs[addr + 1] = (uint8_t)(u >> 8);
    regs[addr + 2] = (uint8_t)(u >> 16);
    regs[addr + 3] = (uint8_t)(u >> 24);
}

static inline float r_f32(uint addr) {
    float f;
    memcpy(&f, &regs[addr], sizeof(float));
    return f;
}
static inline void w_f32(uint addr, float v) {
    memcpy(&regs[addr], &v, sizeof(float));
}

// --- register address map (from control_table.hpp) -----------
// Addresses 0–15: identity / EEPROM
#define ADDR_MODEL_NUMBER       0u
#define ADDR_MODEL_INFORMATION  2u
#define ADDR_FIRMWARE_VERSION   6u
#define ADDR_ID                 7u
#define ADDR_BAUD_RATE          8u
#define ADDR_MILLIS             10u
#define ADDR_MICROS             14u
#define ADDR_DEVICE_STATUS      18u
#define ADDR_BUTTON_1           26u
#define ADDR_BUTTON_2           27u
#define ADDR_BATTERY_VOLTAGE    42u
#define ADDR_BATTERY_PERCENT    46u
#define ADDR_SOUND              50u
#define ADDR_IMU_RECAL          59u
#define ADDR_IMU_ANG_VEL_X      60u
#define ADDR_IMU_ANG_VEL_Y      64u
#define ADDR_IMU_ANG_VEL_Z      68u
#define ADDR_IMU_LIN_ACC_X      72u
#define ADDR_IMU_LIN_ACC_Y      76u
#define ADDR_IMU_LIN_ACC_Z      80u
#define ADDR_IMU_MAG_X          84u
#define ADDR_IMU_MAG_Y          88u
#define ADDR_IMU_MAG_Z          92u
#define ADDR_IMU_ORIENT_W       96u
#define ADDR_IMU_ORIENT_X       100u
#define ADDR_IMU_ORIENT_Y       104u
#define ADDR_IMU_ORIENT_Z       108u
#define ADDR_PRESENT_CUR_L      120u
#define ADDR_PRESENT_CUR_R      124u
#define ADDR_PRESENT_VEL_L      128u
#define ADDR_PRESENT_VEL_R      132u
#define ADDR_PRESENT_POS_L      136u
#define ADDR_PRESENT_POS_R      140u
#define ADDR_MOTOR_TORQUE_EN    149u
#define ADDR_CMD_LINEAR_X       150u  // int32, units of 0.01 m/s  (e.g. 10 → 0.10 m/s)
#define ADDR_CMD_ANGULAR_Z      170u  // int32, units of 0.01 rad/s (e.g. -157 → -1.57 rad/s)

// Diagnostic counters (read-only, addr 240–255)
#define ADDR_DIAG_PKT_COUNT     240u  // uint32 — total valid packets dispatched
#define ADDR_DIAG_CRC_FAIL      244u  // uint32 — CRC failures (silently discarded)
#define ADDR_DIAG_VEL_WRITES    248u  // uint32 — velocity register writes detected
#define ADDR_DIAG_READ_COUNT    252u  // uint32 — READ instructions processed
                                      // NOTE: address 170 matches turtlebot3_node control_table.hpp;
                                      //       verify if upgrading the ROS package.

// Debug — IMU init diagnostics (read-only, addresses 174–183)
// These are written once at boot and remain constant; read via Dynamixel or
// the debug_bno085.py script when bringup is NOT running.
#define ADDR_DBG_IMU_SOURCE     174u  // 1 byte: 0=simulated 1=BNO085
#define ADDR_DBG_BNO085_RC      175u  // 1 byte: BNO085 init result
                                      //   0 = init succeeded (BNO085 active)
                                      //   1 = SHTP timeout at 0x4A (no reply in 500 ms)
                                      //   2 = SHTP timeout at both 0x4A AND 0x4B
                                      //   3 = I2C scan found no device at 0x4A or 0x4B
                                      // 0xFF = not yet set (firmware still initialising)
#define ADDR_DBG_I2C0_NDEV      176u  // 1 byte: number of I2C0 devices found during scan
#define ADDR_DBG_I2C0_DEV0      177u  // 7 bytes: found device 7-bit addresses (0xFF = empty)
#define ADDR_DBG_I2C0_DEV1      178u
#define ADDR_DBG_I2C0_DEV2      179u
#define ADDR_DBG_I2C0_DEV3      180u
#define ADDR_DBG_I2C0_DEV4      181u
#define ADDR_DBG_I2C0_DEV5      182u
#define ADDR_DBG_I2C0_DEV6      183u

static void init_registers(void) {
    memset(regs, 0, REG_SIZE);
    w_u16(ADDR_MODEL_NUMBER,    MODEL_NUMBER);
    w_i32(ADDR_MODEL_INFORMATION, 0);
    regs[ADDR_FIRMWARE_VERSION] = FIRMWARE_VERSION;
    regs[ADDR_ID]               = DEVICE_ID;
    regs[ADDR_BAUD_RATE]        = 4;   // Dynamixel code for 1 Mbaud
    regs[ADDR_DEVICE_STATUS]    = 0;   // OK

    // IMU: identity quaternion, only Z gravity
    w_f32(ADDR_IMU_ORIENT_W,     1.0f);
    w_f32(ADDR_IMU_ORIENT_X,     0.0f);
    w_f32(ADDR_IMU_ORIENT_Y,     0.0f);
    w_f32(ADDR_IMU_ORIENT_Z,     0.0f);
    w_f32(ADDR_IMU_LIN_ACC_X,    0.0f);
    w_f32(ADDR_IMU_LIN_ACC_Y,    0.0f);
    w_f32(ADDR_IMU_LIN_ACC_Z,    IMU_GRAVITY_Z);

    // Battery initial placeholder — updated to real GPIO reading within first 50 ms.
    // Default to "on battery, mid-charge" (14.80 V, 50 %) until first sensor tick.
    w_i32(ADDR_BATTERY_VOLTAGE,  1480);
    w_i32(ADDR_BATTERY_PERCENT,  5000);

    // Motor torque enabled by default
    regs[ADDR_MOTOR_TORQUE_EN] = 1;

    // Debug IMU registers — 0xFF sentinel until init completes
    regs[ADDR_DBG_IMU_SOURCE] = 0xFFu;
    regs[ADDR_DBG_BNO085_RC]  = 0xFFu;
    regs[ADDR_DBG_I2C0_NDEV]  = 0u;
    memset(&regs[ADDR_DBG_I2C0_DEV0], 0xFF, 7u);
}

// ============================================================
// DYNAMIXEL PROTOCOL 2.0  —  CRC-16
// ============================================================

static uint16_t crc_table[256];

static void make_crc_table(void) {
    for (int i = 0; i < 256; i++) {
        uint16_t crc  = 0;
        uint16_t data = (uint16_t)(i << 8);
        for (int j = 0; j < 8; j++) {
            if ((crc ^ data) & 0x8000u)
                crc = (uint16_t)((crc << 1) ^ 0x8005u);
            else
                crc = (uint16_t)(crc << 1);
            data <<= 1;
        }
        crc_table[i] = crc;
    }
}

static uint16_t dxl_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t idx = (uint8_t)(((crc >> 8) ^ data[i]) & 0xFFu);
        crc = (uint16_t)((crc << 8) ^ crc_table[idx]);
    }
    return crc;
}

// ============================================================
// DYNAMIXEL PROTOCOL 2.0  —  PACKET BUILDER
// ============================================================

// Maximum response: READ of all 256 registers, with worst-case byte-
// stuffing.  Unstuffed = 4+1+2+1+1+256+2 = 267.  Stuffing can add
// up to ~86 bytes, so 400 is safe.
#define RESP_BUF_SIZE 400u
static uint8_t resp_buf[RESP_BUF_SIZE];

static uint32_t build_status(uint8_t dev_id, uint8_t error,
                              const uint8_t *data, uint16_t data_len) {
    // --- Step 1: build the UN-STUFFED packet (no CRC yet) into tmp[] ---
    uint8_t tmp[RESP_BUF_SIZE];
    uint16_t payload_len = (uint16_t)(data_len + 4u);  // inst + err + data + crc
    tmp[0] = 0xFF;
    tmp[1] = 0xFF;
    tmp[2] = 0xFD;
    tmp[3] = 0x00;
    tmp[4] = dev_id;
    tmp[5] = (uint8_t)(payload_len);
    tmp[6] = (uint8_t)(payload_len >> 8);
    tmp[7] = 0x55;    // STATUS instruction
    tmp[8] = error;
    uint32_t raw_len = 9;  // bytes before CRC
    if (data && data_len > 0) {
        memcpy(&tmp[raw_len], data, data_len);
        raw_len += data_len;
    }

    // --- Step 2: byte-stuff from INSTRUCTION (index 7) onwards ---
    // Copy header through LEN_H (indices 0-6) verbatim.
    uint32_t out = 0;
    for (uint32_t k = 0; k <= 6; k++) {
        resp_buf[out++] = tmp[k];
    }
    uint16_t stuff_count = 0;
    for (uint32_t k = 7; k < raw_len; k++) {
        resp_buf[out++] = tmp[k];
        // Pattern check against ORIGINAL data (matching DynamixelSDK addStuffing)
        if (tmp[k] == 0xFD && k >= 2 && tmp[k-1] == 0xFF && tmp[k-2] == 0xFF) {
            resp_buf[out++] = 0xFD;  // insert stuffing byte
            stuff_count++;
        }
    }

    // --- Step 3: update length field to include stuffed bytes ---
    uint16_t new_payload_len = (uint16_t)(payload_len + stuff_count);
    resp_buf[5] = (uint8_t)(new_payload_len);
    resp_buf[6] = (uint8_t)(new_payload_len >> 8);

    // --- Step 4: CRC over the stuffed packet (per Protocol 2.0 spec) ---
    uint16_t crc = dxl_crc16(resp_buf, out);
    resp_buf[out++] = (uint8_t)(crc);
    resp_buf[out++] = (uint8_t)(crc >> 8);
    return out;
}

// ============================================================
// COMMAND VELOCITY TIMESTAMP
// ============================================================
// last_cmd_vel_us  — set by handle_write() when velocity registers are
//                    written.  Used to know that at least one velocity
//                    command has been received (don't drive on zero).
// last_host_comm_us — set by dispatch_packet() on EVERY valid packet.
//                     Used for the safety timeout: motors stop if the
//                     host disappears (no reads, writes, or heartbeat
//                     for HOST_TIMEOUT_US).
static volatile uint64_t last_cmd_vel_us    = 0;
static volatile uint64_t last_host_comm_us  = 0;

// ============================================================
// DYNAMIXEL PROTOCOL 2.0  —  INSTRUCTION HANDLERS
// ============================================================

// Send a pre-built packet, handling partial writes.
// The TinyUSB CDC TX FIFO is 512 bytes; a worst-case READ response is
// ~190 bytes (172 data + header/CRC + stuffing), so it usually fits in
// one call.  But if the USB endpoint hasn't drained the previous packet
// yet, tud_cdc_write returns < len.  We loop + tud_task to drain.
static void send_packet(uint32_t len) {
    uint32_t sent = 0;
    while (sent < len) {
        uint32_t n = tud_cdc_write(resp_buf + sent, len - sent);
        sent += n;
        tud_cdc_write_flush();
        if (sent < len) {
            tud_task();  // pump USB to drain TX FIFO
        }
    }
}

static void handle_ping(uint8_t dev_id) {
    uint8_t data[3] = {
        regs[ADDR_MODEL_NUMBER],
        regs[ADDR_MODEL_NUMBER + 1],
        regs[ADDR_FIRMWARE_VERSION]
    };
    uint32_t len = build_status(dev_id, 0x00, data, 3);
    send_packet(len);
}

static void handle_read(uint8_t dev_id, const uint8_t *params, uint16_t nparams) {
    if (nparams < 4) {
        uint32_t len = build_status(dev_id, 0x02, NULL, 0);
        send_packet(len);
        return;
    }
    uint16_t addr   = (uint16_t)(params[0] | ((uint16_t)params[1] << 8));
    uint16_t length = (uint16_t)(params[2] | ((uint16_t)params[3] << 8));

    if ((uint32_t)addr + length > REG_SIZE) {
        uint32_t len = build_status(dev_id, 0x02, NULL, 0);
        send_packet(len);
        return;
    }
    uint32_t len = build_status(dev_id, 0x00, &regs[addr], length);
    send_packet(len);
}

static void handle_write(uint8_t dev_id, const uint8_t *params, uint16_t nparams,
                         bool send_reply) {
    if (nparams < 2) {
        if (send_reply) {
            uint32_t len = build_status(dev_id, 0x02, NULL, 0);
            send_packet(len);
        }
        return;
    }
    uint16_t addr    = (uint16_t)(params[0] | ((uint16_t)params[1] << 8));
    const uint8_t *data = &params[2];
    uint16_t ndata   = (uint16_t)(nparams - 2);

    if ((uint32_t)addr + ndata > REG_SIZE) {
        if (send_reply) {
            uint32_t len = build_status(dev_id, 0x02, NULL, 0);
            send_packet(len);
        }
        return;
    }
    memcpy(&regs[addr], data, ndata);

    // Detect velocity command writes — update the timestamp so the
    // safety timeout in update_odometry() knows the host is alive.
    // The ROS node writes 24 bytes starting at addr 150 (linear_x through angular_z).
    uint16_t write_end = (uint16_t)(addr + ndata);
    if (addr <= ADDR_CMD_ANGULAR_Z + 4u && write_end > ADDR_CMD_LINEAR_X) {
        last_cmd_vel_us = time_us_64();
        // Increment diagnostic velocity write counter
        uint32_t vc = (uint32_t)(regs[ADDR_DIAG_VEL_WRITES]
            | ((uint32_t)regs[ADDR_DIAG_VEL_WRITES+1] << 8)
            | ((uint32_t)regs[ADDR_DIAG_VEL_WRITES+2] << 16)
            | ((uint32_t)regs[ADDR_DIAG_VEL_WRITES+3] << 24));
        w_i32(ADDR_DIAG_VEL_WRITES, (int32_t)(vc + 1));
    }

    if (send_reply) {
        uint32_t len = build_status(dev_id, 0x00, NULL, 0);
        send_packet(len);
    }
}

static void handle_reboot(uint8_t dev_id) {
    uint32_t len = build_status(dev_id, 0x00, NULL, 0);
    send_packet(len);
    sleep_ms(50);
    watchdog_enable(1, true);  // reboot via watchdog in 1 ms
    while (1) {}               // wait for watchdog
}

// ============================================================
// DYNAMIXEL PROTOCOL 2.0  —  PACKET PARSER (state machine)
// ============================================================

// Parser states
typedef enum {
    PS_H1 = 0, PS_H2, PS_H3, PS_H4,
    PS_ID, PS_LENL, PS_LENH, PS_DATA
} parser_state_t;

// Maximum packet buffer: 256 params + 2 CRC + instruction + ID + LEN(2) = 262
#define PKT_BUF_SIZE 300u
static uint8_t  pkt_buf[PKT_BUF_SIZE];
static uint32_t pkt_pos    = 0;
static uint32_t pkt_remain = 0;
static parser_state_t ps   = PS_H1;

static void dispatch_packet(void) {
    // pkt_buf: [ID, LEN_L, LEN_H, INST, PARAMS..., CRC_L, CRC_H]
    if (pkt_pos < 4) return;

    uint8_t dev_id = pkt_buf[0];
    if (dev_id != DEVICE_ID && dev_id != 0xFE) return;

    // ---- CRC verification (on the RAW / stuffed wire data) ----
    // Per Dynamixel Protocol 2.0, the sender calculates CRC AFTER
    // byte-stuffing, so we must verify on the stuffed packet.
    //   CRC input = [FF FF FD 00] + pkt_buf[0 .. pkt_pos-3]
    uint8_t crc_buf[4 + PKT_BUF_SIZE];
    crc_buf[0] = 0xFF; crc_buf[1] = 0xFF;
    crc_buf[2] = 0xFD; crc_buf[3] = 0x00;
    memcpy(&crc_buf[4], pkt_buf, pkt_pos - 2);  // exclude the CRC bytes
    uint16_t crc_calc = dxl_crc16(crc_buf, 4 + pkt_pos - 2);
    uint16_t crc_recv = (uint16_t)(pkt_buf[pkt_pos - 2]
                                 | ((uint16_t)pkt_buf[pkt_pos - 1] << 8));
    if (crc_calc != crc_recv) {
        // Increment diagnostic CRC failure counter
        uint32_t fc = (uint32_t)(regs[ADDR_DIAG_CRC_FAIL]
            | ((uint32_t)regs[ADDR_DIAG_CRC_FAIL+1] << 8)
            | ((uint32_t)regs[ADDR_DIAG_CRC_FAIL+2] << 16)
            | ((uint32_t)regs[ADDR_DIAG_CRC_FAIL+3] << 24));
        w_i32(ADDR_DIAG_CRC_FAIL, (int32_t)(fc + 1));
        return;
    }

    // ---- Byte un-stuffing (Protocol 2.0) ----
    // Collapse every [FF FF FD FD] back to [FF FF FD] in the payload
    // (from ID to just before CRC).
    uint32_t dst = 0;
    uint8_t unstuffed[PKT_BUF_SIZE];
    for (uint32_t src = 0; src < pkt_pos - 2u; src++) {
        unstuffed[dst++] = pkt_buf[src];
        if (dst >= 3
            && unstuffed[dst-3] == 0xFF
            && unstuffed[dst-2] == 0xFF
            && unstuffed[dst-1] == 0xFD
            && src + 1 < pkt_pos - 2u
            && pkt_buf[src+1] == 0xFD) {
            src++;  // skip the extra 0xFD (stuffing byte)
        }
    }

    uint8_t instruction = unstuffed[3];
    const uint8_t *params = &unstuffed[4];
    uint16_t nparams = (dst >= 4) ? (uint16_t)(dst - 4) : 0;

    // Record that a valid packet arrived from the host.
    last_host_comm_us = time_us_64();

    // Increment diagnostic packet counter
    {
        uint32_t pc = (uint32_t)(regs[ADDR_DIAG_PKT_COUNT]
            | ((uint32_t)regs[ADDR_DIAG_PKT_COUNT+1] << 8)
            | ((uint32_t)regs[ADDR_DIAG_PKT_COUNT+2] << 16)
            | ((uint32_t)regs[ADDR_DIAG_PKT_COUNT+3] << 24));
        w_i32(ADDR_DIAG_PKT_COUNT, (int32_t)(pc + 1));
    }

    // Broadcast (0xFE): only process writes, never send a reply
    bool send_reply = (dev_id != 0xFE);

    switch (instruction) {
        case 0x01:  // PING
            handle_ping(dev_id);
            break;
        case 0x02:  // READ
            if (send_reply) {
                // Increment diagnostic READ counter
                uint32_t rc = (uint32_t)(regs[ADDR_DIAG_READ_COUNT]
                    | ((uint32_t)regs[ADDR_DIAG_READ_COUNT+1] << 8)
                    | ((uint32_t)regs[ADDR_DIAG_READ_COUNT+2] << 16)
                    | ((uint32_t)regs[ADDR_DIAG_READ_COUNT+3] << 24));
                w_i32(ADDR_DIAG_READ_COUNT, (int32_t)(rc + 1));
                handle_read(dev_id, params, nparams);
            }
            break;
        case 0x03:  // WRITE
            handle_write(dev_id, params, nparams, send_reply);
            break;
        case 0x08:  // REBOOT
            if (send_reply) handle_reboot(dev_id);
            break;
        default:
            break;  // unknown instruction — ignore
    }
}

static void parse_byte(uint8_t b) {
    switch (ps) {
        case PS_H1:
            if (b == 0xFF) ps = PS_H2;
            break;
        case PS_H2:
            ps = (b == 0xFF) ? PS_H3 : PS_H1;
            break;
        case PS_H3:
            if      (b == 0xFD) ps = PS_H4;
            else if (b != 0xFF) ps = PS_H1;   // 0xFF stays in PS_H3
            break;
        case PS_H4:
            if (b == 0x00) { pkt_pos = 0; ps = PS_ID; }
            else ps = PS_H1;
            break;
        case PS_ID:
            pkt_buf[pkt_pos++] = b;
            ps = PS_LENL;
            break;
        case PS_LENL:
            pkt_buf[pkt_pos++] = b;
            pkt_remain = b;
            ps = PS_LENH;
            break;
        case PS_LENH:
            pkt_buf[pkt_pos++] = b;
            pkt_remain |= ((uint32_t)b << 8);
            ps = PS_DATA;
            break;
        case PS_DATA:
            if (pkt_pos < PKT_BUF_SIZE)
                pkt_buf[pkt_pos++] = b;
            if (--pkt_remain == 0) {
                ps = PS_H1;
                dispatch_packet();
                pkt_pos = 0;
            }
            break;
    }
}

// ============================================================
// MOTOR CONTROL
// ============================================================

static uint pwm_slice_m1, pwm_slice_m2;

// Per-motor state for kick-start detection.
// dir: -1 = reverse, 0 = stopped, +1 = forward
// kick_remaining: countdown of odometry cycles with boosted duty
typedef struct {
    int8_t   dir;
    uint8_t  kick_remaining;
} motor_state_t;

static motor_state_t motor_state[2] = {{0, 0}, {0, 0}};

static void init_motors(void) {
    // GP8/GP9 are on the same PWM slice (each pair shares a slice)
    gpio_set_function(PIN_M1A, GPIO_FUNC_PWM);
    gpio_set_function(PIN_M1B, GPIO_FUNC_PWM);
    gpio_set_function(PIN_M2A, GPIO_FUNC_PWM);
    gpio_set_function(PIN_M2B, GPIO_FUNC_PWM);

    pwm_slice_m1 = pwm_gpio_to_slice_num(PIN_M1A);
    pwm_slice_m2 = pwm_gpio_to_slice_num(PIN_M2A);

    // Configure both slices: 10 kHz, wrap=999
    for (uint s = pwm_slice_m1; s <= pwm_slice_m2; s++) {
        pwm_set_clkdiv(s, MOTOR_PWM_CLKDIV);
        pwm_set_wrap(s, MOTOR_PWM_WRAP);
        pwm_set_chan_level(s, PWM_CHAN_A, 0);
        pwm_set_chan_level(s, PWM_CHAN_B, 0);
        pwm_set_enabled(s, true);
    }
}

// Drive one motor given a throttle in [-1.0, +1.0].
// reversed: set true to invert direction for that motor.
// motor_idx: 0 = left, 1 = right (for kick-start state tracking)
static void set_motor(uint slice, bool reversed, float throttle, int motor_idx) {
    if (throttle < -1.0f) throttle = -1.0f;
    if (throttle >  1.0f) throttle =  1.0f;
    if (reversed) throttle = -throttle;

    // Determine new direction
    int8_t new_dir;
    if (throttle > 0.001f) new_dir = 1;
    else if (throttle < -0.001f) new_dir = -1;
    else new_dir = 0;

    motor_state_t *ms = &motor_state[motor_idx];

    // Detect start-from-rest or direction change → trigger kick
    if (new_dir != 0 && new_dir != ms->dir) {
        ms->kick_remaining = MOTOR_KICK_CYCLES;
    }
    ms->dir = new_dir;

    // Dead-zone compensation: boost small non-zero throttle to the minimum
    // duty needed to overcome the motor's static friction.  This maps the
    // input range [0.001, 1.0] → [MOTOR_MIN_DUTY, 1.0] linearly.
    float mag = fabsf(throttle);
    if (mag > 0.001f && MOTOR_MIN_DUTY > 0.0f) {
        mag = MOTOR_MIN_DUTY + mag * (1.0f - MOTOR_MIN_DUTY);
        if (mag > 1.0f) mag = 1.0f;
    }

    // Kick-start: override duty during the kick window
    if (ms->kick_remaining > 0 && mag > 0.001f) {
        if (mag < MOTOR_KICK_DUTY) {
            mag = MOTOR_KICK_DUTY;
        }
        ms->kick_remaining--;
    }

    uint16_t duty = (uint16_t)(mag * MOTOR_PWM_WRAP);

    if (new_dir > 0) {
        pwm_set_chan_level(slice, PWM_CHAN_A, duty);  // forward
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    } else if (new_dir < 0) {
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice, PWM_CHAN_B, duty);  // reverse
    } else {
        // Brake (both low = coasting)
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    }
}

static void brake_all(void) {
    set_motor(pwm_slice_m1, false, 0.0f, 0);
    set_motor(pwm_slice_m2, false, 0.0f, 1);
}

// ============================================================
// BUZZER  —  multi-note melody sequencer
// ============================================================
//
// Note index table (0 = silence):
//   1=C4  2=C#4  3=D4  4=D#4  5=E4  6=F4  7=F#4  8=G4  9=G#4
//  10=A4 11=A#4 12=B4 13=C5  14=D5 15=E5 16=G5
//
// Sound event values (written to ADDR_SOUND by the ROS /sound service):
//   0=OFF  1=ON  2=LOW_BATTERY  3=ERROR  4=BUTTON1  5=BUTTON2
//
#define NOTE_COUNT 17u
static const uint16_t NOTE_FREQ[NOTE_COUNT] = {
    0,    // 0:  silence
    262,  // 1:  C4
    277,  // 2:  C#4
    294,  // 3:  D4
    311,  // 4:  D#4
    330,  // 5:  E4
    349,  // 6:  F4
    370,  // 7:  F#4
    392,  // 8:  G4
    415,  // 9:  G#4
    440,  // 10: A4
    466,  // 11: A#4
    494,  // 12: B4
    523,  // 13: C5
    587,  // 14: D5
    659,  // 15: E5
    784,  // 16: G5
};

// A melody step: note index + duration in ms.  {0,0} terminates a sequence.
typedef struct { uint8_t note; uint16_t ms; } mel_step_t;

// ON (startup / powered-on): ascending major arpeggio  E4-G4-B4-C5
static const mel_step_t MEL_ON[] = {
    {5,  120}, {8,  120}, {12, 120}, {13, 250}, {0, 0}
};
// LOW_BATTERY: three descending warning beeps  A4-G4-E4
static const mel_step_t MEL_LOW_BATT[] = {
    {10, 120}, {0, 60}, {8, 120}, {0, 60}, {5, 250}, {0, 0}
};
// ERROR: two rapid low-pitched bursts  D4-D4
static const mel_step_t MEL_ERROR[] = {
    {3, 100}, {0, 60}, {3, 100}, {0, 0}
};
// BUTTON1: short single blip  G4
static const mel_step_t MEL_BTN1[] = {
    {8, 80}, {0, 0}
};
// BUTTON2: two ascending blips  G4-A4
static const mel_step_t MEL_BTN2[] = {
    {8, 80}, {0, 40}, {10, 80}, {0, 0}
};

static const mel_step_t * const MELODIES[6] = {
    NULL,          // 0: silence — handled separately
    MEL_ON,        // 1
    MEL_LOW_BATT,  // 2
    MEL_ERROR,     // 3
    MEL_BTN1,      // 4
    MEL_BTN2,      // 5
};

static uint pwm_slice_buzz;
static uint pwm_chan_buzz;

static void init_buzzer(void) {
    gpio_set_function(PIN_BUZZER, GPIO_FUNC_PWM);
    pwm_slice_buzz = pwm_gpio_to_slice_num(PIN_BUZZER);
    pwm_chan_buzz  = pwm_gpio_to_channel(PIN_BUZZER);
    pwm_set_clkdiv(pwm_slice_buzz, 10.0f);
    pwm_set_wrap(pwm_slice_buzz, 999);
    pwm_set_chan_level(pwm_slice_buzz, pwm_chan_buzz, 0);
    pwm_set_enabled(pwm_slice_buzz, true);
}

// Low-level: output a single note (or silence) immediately.
static void play_sound(uint8_t note_id) {
    if (note_id >= NOTE_COUNT) note_id = 0;
    uint16_t freq = NOTE_FREQ[note_id];
    if (freq == 0) {
        pwm_set_chan_level(pwm_slice_buzz, pwm_chan_buzz, 0);
        return;
    }
    // wrap = (sys_clk / clkdiv) / freq - 1  →  15_000_000 / freq - 1
    uint32_t wrap = (SYS_CLK_HZ / 10u) / freq - 1u;
    if (wrap > 65535u) wrap = 65535u;
    pwm_set_wrap(pwm_slice_buzz, (uint16_t)wrap);
    pwm_set_chan_level(pwm_slice_buzz, pwm_chan_buzz, (uint16_t)((wrap + 1) / 2));
}

// ---- Non-blocking melody state machine ----
static const mel_step_t *mel_seq     = NULL;
static uint32_t          mel_step_i  = 0;
static uint64_t          mel_next_us = 0;
static uint8_t           mel_last_trigger = 0xFF;  // value that started current melody

// Start playing a named sound event (non-blocking; advances via update_melody).
static void start_melody(uint8_t sound_id) {
    if (sound_id == 0 || sound_id >= 6) {
        mel_seq = NULL;
        play_sound(0);
        return;
    }
    mel_seq          = MELODIES[sound_id];
    mel_step_i       = 0;
    mel_next_us      = time_us_64();  // start first note immediately
    mel_last_trigger = sound_id;
}

// Called every main-loop iteration; advances the melody state machine.
static void update_melody(uint64_t now_us) {
    if (!mel_seq) return;
    if (now_us < mel_next_us) return;

    const mel_step_t *step = &mel_seq[mel_step_i];
    if (step->ms == 0) {
        // Sequence finished — silence and auto-clear the register.
        play_sound(0);
        mel_seq = NULL;
        regs[ADDR_SOUND] = 0;
        mel_last_trigger = 0;
        return;
    }
    play_sound(step->note);
    mel_next_us = now_us + (uint64_t)(step->ms) * 1000u;
    mel_step_i++;
}

// Blocking version — used only for the startup melody before the main loop.
static void play_melody_blocking(const mel_step_t *seq) {
    for (uint32_t i = 0; seq[i].ms != 0; i++) {
        play_sound(seq[i].note);
        sleep_ms(seq[i].ms);
    }
    play_sound(0);
}

// ============================================================
// BUTTONS
// ============================================================

static void init_buttons(void) {
    gpio_init(PIN_BTN1);
    gpio_set_dir(PIN_BTN1, GPIO_IN);
    gpio_pull_up(PIN_BTN1);

    gpio_init(PIN_BTN2);
    gpio_set_dir(PIN_BTN2, GPIO_IN);
    gpio_pull_up(PIN_BTN2);
}

// ============================================================
// X-UPS1 SIGNAL PINS  (Grove 6 — GP26 / GP27)
// ============================================================

static void init_ups_gpio(void) {
    // PLD: Power Loss Detection — active-HIGH, high when mains is disconnected.
    gpio_init(PIN_UPS_PLD);
    gpio_set_dir(PIN_UPS_PLD, GPIO_IN);
    gpio_pull_down(PIN_UPS_PLD);   // default LOW when nothing connected

    // LBAT: Low Battery — active-HIGH, high when battery ≤ 3 V per cell.
    gpio_init(PIN_UPS_LBAT);
    gpio_set_dir(PIN_UPS_LBAT, GPIO_IN);
    gpio_pull_down(PIN_UPS_LBAT);  // default LOW when nothing connected
}

// ============================================================
// BATTERY ADC
// ============================================================

static void init_adc(void) {
    adc_init();
    adc_gpio_init(29);          // GP29 = ADC3 = VSYS (Pico 2 internal monitor)
}

// Read VSYS (Pico 2 supply rail) via GP29's internal 3:1 divider.
static float read_vsys(void) {
    adc_select_input(ADC_CH_VSYS);
    uint16_t raw = adc_read();
    return (float)raw * 3.0f * 3.3f / 4096.0f;
}

static bool  vbatt_low_alerted = false;  // latch so warning plays only once

// ============================================================
// BNO085 IMU  (SHTP over I2C on Grove 1 port)
// ============================================================
//
// Grove 1 connector on the Cytron Robo Pico:
//   Pin 1 (Yellow) = GP0 = SDA  (I2C0)
//   Pin 2 (White)  = GP1 = SCL  (I2C0)
//   Pin 3 (Red)    = 3.3 V supply
//   Pin 4 (Black)  = GND
//
// BNO085 module wiring (SparkFun/Seeed/similar Grove I2C breakout):
//   SA0 → GND  →  I2C address = 0x4A  (default)
//   SA0 → VCC  →  I2C address = 0x4B  (change BNO085_ADDR below)
//
// If BNO085 is absent or fails to init, the firmware automatically
// falls back to a static simulated IMU (identity quaternion, Z-gravity).

#define BNO085_I2C_PORT   i2c0
#define BNO085_SDA_PIN    0u
#define BNO085_SCL_PIN    1u
#define BNO085_I2C_HZ     400000u
#define BNO085_ADDR       0x4Au   // SA0 = GND.  Change to 0x4B if SA0 = VCC.

// SHTP channel numbers (host perspective)
#define SHTP_CH_COMMAND    0u   // BNO085 → host: advertisement / product ID
#define SHTP_CH_EXECUTABLE 1u
#define SHTP_CH_CONTROL    2u   // host → BNO085: Set Feature, DCD config, ...
#define SHTP_CH_REPORTS    3u   // BNO085 → host: sensor data

// SH-2 report IDs
#define SH2_ACCELEROMETER     0x01u
#define SH2_GYROSCOPE_CAL     0x02u
#define SH2_MAG_FIELD_CAL     0x03u   // SH-2 §6.5.16: Magnetic Field Calibrated
#define SH2_ROTATION_VECTOR   0x05u
#define SH2_SET_FEATURE_CMD   0xFDu
#define SH2_COMMAND_REQUEST   0xF2u

// SH-2 command IDs (sent via Command Request report 0xF2)
#define SH2_CMD_DCD_SAVE      0x06u   // Save Dynamic Calibration Data to flash
#define SH2_CMD_ME_CAL        0x07u   // Configure ME (Motion Engine) calibration

// Q-point fixed-point scaling (BNO085 SH-2 reference manual, §6.5)
//   Rotation vector (i,j,k,real) : Q14  →  value / 16384.0
//   Accelerometer                 : Q8   →  value / 256.0   (m/s²)
//   Gyroscope calibrated          : Q9   →  value / 512.0   (rad/s)
//   Magnetic field calibrated     : Q4   →  value / 16.0    (µT)
#define Q14_SCALE  (1.0f / 16384.0f)
#define Q8_SCALE   (1.0f / 256.0f)
#define Q9_SCALE   (1.0f / 512.0f)
#define Q4_UT_TO_T (1.0f / 16.0f * 1e-6f)   // µT → T (ROS uses SI Tesla)

// Probe result — set by init_bno085(); read by update_sensors()
static bool bno085_present = false;

// Runtime I2C address for BNO085; set by init_bno085() after auto-detection.
// Default is BNO085_ADDR (0x4A); overridden to 0x4B if SA0=VCC is detected.
static uint8_t bno085_runtime_addr = BNO085_ADDR;

// Per-channel SHTP sequence numbers (8 channels max)
static uint8_t shtp_seq[8] = {0};

// Sequence number for SH-2 command requests (distinct from SHTP channel seq)
static uint8_t bno085_cmd_seq = 0;

// ------------ low-level SHTP I2C helpers -------------------------

// Write one SHTP packet (header + payload) to BNO085.
// Returns true on success.
static bool shtp_send(uint8_t channel, const uint8_t *payload, uint16_t plen) {
    uint16_t total = (uint16_t)(4u + plen);
    if (total > 256u) return false;

    uint8_t pkt[256];
    pkt[0] = (uint8_t)(total & 0xFFu);
    pkt[1] = (uint8_t)(total >> 8u);
    pkt[2] = channel;
    pkt[3] = shtp_seq[channel & 0x07u]++;
    memcpy(&pkt[4], payload, plen);

    int ret = i2c_write_timeout_us(BNO085_I2C_PORT, bno085_runtime_addr,
                                   pkt, total, false, 5000u);
    return (ret == (int)total);
}

// Read one SHTP packet from BNO085.
// Fills cargo[] (after the 4-byte header) and sets *ch_out to the channel.
// Returns the number of cargo bytes received, or 0 if nothing pending / error.
static uint16_t shtp_receive(uint8_t *cargo, uint16_t max_cargo, uint8_t *ch_out) {
    // Step 1: read exactly 4-byte header to discover total packet length.
    uint8_t hdr[4];
    int ret = i2c_read_timeout_us(BNO085_I2C_PORT, bno085_runtime_addr,
                                  hdr, 4u, false, 3000u);
    if (ret != 4) return 0;

    // Bits [14:0] of the 16-bit length field = total byte count (header included).
    // Bit 15 (MSB of byte 1) is the "continuation" flag — strip it.
    uint16_t pkt_len = (uint16_t)(hdr[0]) | ((uint16_t)(hdr[1] & 0x7Fu) << 8u);
    if (pkt_len < 4u) return 0;   // empty / error packet

    uint16_t cargo_len = pkt_len - 4u;
    if (ch_out) *ch_out = hdr[2];

    if (cargo_len == 0u) return 0;

    // Step 2: re-read the complete packet (header + cargo) in one transaction.
    // BNO085 keeps the packet in its output buffer until the host reads it fully.
    uint8_t full[260];
    uint16_t read_len = pkt_len < (uint16_t)sizeof(full) ? pkt_len : (uint16_t)sizeof(full);
    ret = i2c_read_timeout_us(BNO085_I2C_PORT, bno085_runtime_addr,
                              full, read_len, false, 5000u);
    if (ret != (int)read_len) return 0;

    // Channel may have been updated in the full read's header
    if (ch_out) *ch_out = full[2];

    uint16_t copy = (cargo_len < max_cargo) ? cargo_len : max_cargo;
    memcpy(cargo, &full[4], copy);
    return copy;
}

// ------------ sensor report enabling ----------------------------

// Send a Set Feature Command on the SHTP control channel (2) to enable
// one sensor report type at the specified period (microseconds).
static void bno085_enable_report(uint8_t report_id, uint32_t period_us) {
    // SH-2 Set Feature Command — 17 bytes total (table 6-6 in SH-2 ref. manual).
    uint8_t cmd[17] = {
        SH2_SET_FEATURE_CMD,           // [0] command report ID
        report_id,                     // [1] sensor report to enable
        0x00,                          // [2] feature flags (none)
        0x00,                          // [3] change sensitivity (absolute), LSB
        0x00,                          // [4] change sensitivity MSB
        (uint8_t)(period_us        ),  // [5] report interval µs, byte 0
        (uint8_t)(period_us >>  8u ),  // [6]                      byte 1
        (uint8_t)(period_us >> 16u ),  // [7]                      byte 2
        (uint8_t)(period_us >> 24u ),  // [8]                      byte 3
        0x00, 0x00, 0x00, 0x00,        // [9-12]  batch interval (disabled)
        0x00, 0x00, 0x00, 0x00,        // [13-16] sensor-specific config (default)
    };
    shtp_send(SHTP_CH_CONTROL, cmd, sizeof(cmd));
}

// ------------ cargo parser -------------------------------------

// Parse one complete SHTP channel-3 cargo (may contain multiple back-to-back
// sensor reports) and write real data into the Dynamixel control table.
static void bno085_parse_cargo(const uint8_t *cargo, uint16_t len) {
    uint16_t off = 0;
    while (off < len) {
        if (off + 1u > len) break;
        uint8_t rid = cargo[off];

        if (rid == SH2_ROTATION_VECTOR) {
            // 14-byte report:
            //   [0]    report ID
            //   [1]    sequence / status
            //   [2]    status & accuracy
            //   [3]    delay (×100 µs)
            //   [4-5]  quaternion i  (int16 Q14)
            //   [6-7]  quaternion j  (int16 Q14)
            //   [8-9]  quaternion k  (int16 Q14)
            //   [10-11] quaternion real (int16 Q14)
            //   [12-13] accuracy estimate (int16 Q12, radians)
            if (off + 14u > len) break;
            int16_t qi = (int16_t)((uint16_t)cargo[off+ 4] | ((uint16_t)cargo[off+ 5] << 8u));
            int16_t qj = (int16_t)((uint16_t)cargo[off+ 6] | ((uint16_t)cargo[off+ 7] << 8u));
            int16_t qk = (int16_t)((uint16_t)cargo[off+ 8] | ((uint16_t)cargo[off+ 9] << 8u));
            int16_t qr = (int16_t)((uint16_t)cargo[off+10] | ((uint16_t)cargo[off+11] << 8u));
            w_f32(ADDR_IMU_ORIENT_X, qi * Q14_SCALE);
            w_f32(ADDR_IMU_ORIENT_Y, qj * Q14_SCALE);
            w_f32(ADDR_IMU_ORIENT_Z, qk * Q14_SCALE);
            w_f32(ADDR_IMU_ORIENT_W, qr * Q14_SCALE);
            off += 14u;
        }
        else if (rid == SH2_ACCELEROMETER) {
            // 10-byte report: [0-3] header, [4-5] x, [6-7] y, [8-9] z  (int16 Q8 m/s²)
            if (off + 10u > len) break;
            int16_t ax = (int16_t)((uint16_t)cargo[off+4] | ((uint16_t)cargo[off+5] << 8u));
            int16_t ay = (int16_t)((uint16_t)cargo[off+6] | ((uint16_t)cargo[off+7] << 8u));
            int16_t az = (int16_t)((uint16_t)cargo[off+8] | ((uint16_t)cargo[off+9] << 8u));
            w_f32(ADDR_IMU_LIN_ACC_X, ax * Q8_SCALE);
            w_f32(ADDR_IMU_LIN_ACC_Y, ay * Q8_SCALE);
            w_f32(ADDR_IMU_LIN_ACC_Z, az * Q8_SCALE);
            off += 10u;
        }
        else if (rid == SH2_GYROSCOPE_CAL) {
            // 10-byte report: [0-3] header, [4-5] x, [6-7] y, [8-9] z  (int16 Q9 rad/s)
            if (off + 10u > len) break;
            int16_t gx = (int16_t)((uint16_t)cargo[off+4] | ((uint16_t)cargo[off+5] << 8u));
            int16_t gy = (int16_t)((uint16_t)cargo[off+6] | ((uint16_t)cargo[off+7] << 8u));
            int16_t gz = (int16_t)((uint16_t)cargo[off+8] | ((uint16_t)cargo[off+9] << 8u));
            w_f32(ADDR_IMU_ANG_VEL_X, gx * Q9_SCALE);
            w_f32(ADDR_IMU_ANG_VEL_Y, gy * Q9_SCALE);
            w_f32(ADDR_IMU_ANG_VEL_Z, gz * Q9_SCALE);
            off += 10u;
        }
        else if (rid == SH2_MAG_FIELD_CAL) {
            // 10-byte report: [0-3] header, [4-5] x, [6-7] y, [8-9] z  (int16 Q4 µT)
            // ROS sensor_msgs/MagneticField expects Tesla (SI), so we convert µT → T.
            if (off + 10u > len) break;
            int16_t mx = (int16_t)((uint16_t)cargo[off+4] | ((uint16_t)cargo[off+5] << 8u));
            int16_t my = (int16_t)((uint16_t)cargo[off+6] | ((uint16_t)cargo[off+7] << 8u));
            int16_t mz = (int16_t)((uint16_t)cargo[off+8] | ((uint16_t)cargo[off+9] << 8u));
            w_f32(ADDR_IMU_MAG_X, mx * Q4_UT_TO_T);
            w_f32(ADDR_IMU_MAG_Y, my * Q4_UT_TO_T);
            w_f32(ADDR_IMU_MAG_Z, mz * Q4_UT_TO_T);
            off += 10u;
        }
        else if (rid == 0xFBu) {
            // Base Timestamp Reference (SH-2 §1.3.4): 5-byte header that the BNO085
            // prepends to every cargo payload.  It carries no sensor data — skip it.
            // Format: [0] 0xFB  [1-4] timestamp (uint32, 100 µs units)
            if (off + 5u > len) break;
            off += 5u;
        }
        else if (rid == 0xF8u) {
            // Time Delta Reference (SH-2 §1.3.4): 4-byte record that may follow 0xFB.
            // Format: [0] 0xF8  [1] reserved  [2-3] delta (uint16, 100 µs units)
            if (off + 4u > len) break;
            off += 4u;
        }
        else {
            // Unknown / unsolicited report ID; cannot determine length — stop.
            break;
        }
    }
}

// ------------ calibration commands -----------------------------

// Send a SH-2 command request to enable ME (Motion Engine) calibration
// for accelerometer, gyroscope, and magnetometer.  This allows the BNO085's
// built-in sensor fusion to continuously refine its calibration offsets.
static void bno085_enable_me_calibration(void) {
    uint8_t cmd[12] = {0};
    cmd[0] = SH2_COMMAND_REQUEST;   // Report ID
    cmd[1] = bno085_cmd_seq++;      // Sequence number
    cmd[2] = SH2_CMD_ME_CAL;        // Command: ME Calibration Config
    cmd[3] = 1;  // P1: Accel cal enable
    cmd[4] = 1;  // P2: Gyro cal enable
    cmd[5] = 1;  // P3: Mag cal enable
    cmd[6] = 0;  // P4: Planar accel cal (disabled — not relevant for a mobile robot)
    // P5–P9: reserved, zero
    shtp_send(SHTP_CH_CONTROL, cmd, sizeof(cmd));
}

// Save DCD (Dynamic Calibration Data) to BNO085 on-chip flash so that
// calibration offsets persist across power cycles and reboot.
static void bno085_save_dcd(void) {
    uint8_t cmd[12] = {0};
    cmd[0] = SH2_COMMAND_REQUEST;
    cmd[1] = bno085_cmd_seq++;
    cmd[2] = SH2_CMD_DCD_SAVE;
    // No additional parameters needed
    shtp_send(SHTP_CH_CONTROL, cmd, sizeof(cmd));
}

// Wait for initial sensor reports to arrive and stabilize after enabling
// reports and calibration.  Drains SHTP packets for up to `timeout_ms`
// milliseconds, updating IMU registers as data arrives.
// Returns true if at least one valid sensor report was received.
static bool bno085_wait_for_calibration(uint32_t timeout_ms) {
    uint64_t deadline = time_us_64() + (uint64_t)timeout_ms * 1000u;
    bool got_data = false;
    uint8_t cargo[256];
    uint8_t ch;

    // Temporarily zero the orientation register so we can detect a real write.
    // (init_registers sets ADDR_IMU_ORIENT_W = 1.0f which would otherwise
    //  look identical to a valid identity-quaternion rotation report.)
    w_f32(ADDR_IMU_ORIENT_W, 0.0f);

    while (time_us_64() < deadline) {
        uint16_t clen = shtp_receive(cargo, sizeof(cargo), &ch);
        if (clen > 0u && ch == SHTP_CH_REPORTS) {
            bno085_parse_cargo(cargo, clen);
            // A genuine rotation-vector write sets W to a non-zero value.
            // Angular velocity or accel reports alone won't touch W.
            float qw = r_f32(ADDR_IMU_ORIENT_W);
            if (fabsf(qw) > 0.01f) {
                got_data = true;
            }
        }
        sleep_us(1000u);
    }

    // Restore the identity quaternion W if no real data arrived, so the
    // simulated-IMU fallback stays valid.
    if (!got_data) {
        w_f32(ADDR_IMU_ORIENT_W, 1.0f);
    }
    return got_data;
}

// Drain all pending SHTP packets and update the IMU control registers.
// Called every sensor-update interval (20 Hz).  Each call reads at most
// BNO085_MAX_READS packets to bound latency impact on the USB/Dynamixel loop.
#define BNO085_MAX_READS 8u
static void bno085_update(void) {
    uint8_t cargo[256];
    uint8_t ch;
    for (uint32_t i = 0u; i < BNO085_MAX_READS; i++) {
        uint16_t clen = shtp_receive(cargo, sizeof(cargo), &ch);
        if (clen == 0u) break;           // no more packets waiting
        if (ch == SHTP_CH_REPORTS) {
            bno085_parse_cargo(cargo, clen);
        }
        // Channels 0 (advertisement) and 2 (control responses) are silently ignored.
    }
}

// ------------ I2C bus scanner ----------------------------------

// Scan all valid 7-bit addresses on I2C0 and record up to 7 found device
// addresses in the debug registers.  Expects I2C0 to be already initialised.
// IMPORTANT: must be called AFTER the SHTP init probe — reading from the
// BNO085 address during a scan corrupts its boot advertisement packet.
static void i2c0_scan_and_record(void) {
    uint8_t nfound = 0;
    for (uint8_t addr = 0x08u; addr <= 0x77u && nfound < 7u; addr++) {
        // Skip BNO085 candidate addresses — already probed via SHTP.
        if (addr == 0x4Au || addr == 0x4Bu) {
            // Record them as present only if bno085_runtime_addr matched.
            if (addr == bno085_runtime_addr) {
                regs[ADDR_DBG_I2C0_DEV0 + nfound] = addr;
                nfound++;
            }
            continue;
        }
        uint8_t dummy;
        int ret = i2c_read_timeout_us(BNO085_I2C_PORT, addr,
                                      &dummy, 1u, false, 2000u);
        if (ret == 1) {
            regs[ADDR_DBG_I2C0_DEV0 + nfound] = addr;
            nfound++;
        }
    }
    regs[ADDR_DBG_I2C0_NDEV] = nfound;
}

// ------------ initialisation -----------------------------------

// Probe the BNO085 over I2C, drain its boot advertisement, and enable
// the four sensor reports needed by turtlebot3_node.
// Tries address 0x4A (SA0=GND) first, then 0x4B (SA0=VCC).
// Records I2C scan results and failure codes in debug registers.
// Call once from main() before entering the main loop.
// Returns true if the sensor responded; false if not connected.
static bool init_bno085(void) {
    i2c_init(BNO085_I2C_PORT, BNO085_I2C_HZ);
    gpio_set_function(BNO085_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BNO085_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BNO085_SDA_PIN);
    gpio_pull_up(BNO085_SCL_PIN);

    // BNO085 needs up to 80 ms to boot after power-on.
    sleep_ms(150u);

    // NOTE: The I2C scan is intentionally deferred until AFTER the SHTP init.
    // Scanning 0x4A during boot reads 1 byte from the BNO085, consuming part
    // of its SHTP boot advertisement and corrupting the protocol state.
    // We will call i2c0_scan_and_record() after the SHTP sequence is complete.

    // Try both candidate 7-bit addresses: 0x4A (SA0→GND default) then 0x4B (SA0→VCC).
    const uint8_t candidates[2] = { 0x4Au, 0x4Bu };
    bool alive = false;
    uint8_t tmp[256];
    uint8_t ch;

    for (int ci = 0; ci < 2 && !alive; ci++) {
        bno085_runtime_addr = candidates[ci];

        // Step 1: Soft reset — send byte 0x01 on CHANNEL_EXECUTABLE (channel 1).
        // This mirrors the SparkFun library's softReset() and ensures the BNO085
        // is in a known state regardless of what it was doing before (e.g. the
        // Pico rebooted while the BNO085 stayed powered and was still streaming).
        uint8_t rst = 0x01u;
        shtp_send(SHTP_CH_EXECUTABLE, &rst, 1u);

        // Step 2: Wait 100 ms for the BNO085 to complete its internal reset
        // and generate a fresh boot advertisement, then drain ALL pending
        // packets — not just the first one.  The BNO085 sends several packets
        // during boot (advertisement, product-ID, FRS records).  If we leave
        // any of those unread, its SHTP input FIFO stays blocked and the
        // subsequent Set Feature commands for rotation vector and gyro are
        // silently discarded, causing those reports to never start.
        // We repeat the drain twice (as SparkFun does) to catch any straggler.
        sleep_ms(100u);
        uint16_t drained = 0;
        for (int pass = 0; pass < 2; pass++) {
            uint64_t quiet_deadline = time_us_64() + 150000u;  // 150 ms quiet window
            while (time_us_64() < quiet_deadline) {
                uint16_t n = shtp_receive(tmp, sizeof(tmp), &ch);
                if (n > 0u) {
                    drained++;
                    alive = true;                       // sensor is responding
                    quiet_deadline = time_us_64() + 50000u;  // reset window after each packet
                } else {
                    sleep_us(2000u);
                }
            }
            sleep_ms(50u);  // brief gap between passes
        }

        if (!alive) {
            regs[ADDR_DBG_BNO085_RC] = (ci == 0) ? 1u : 2u;
        }
    }

    // I2C scan runs AFTER the SHTP probe so the BNO085 boot advertisement is
    // never disturbed by a spurious 1-byte read during the address sweep.
    // The scan skips the two BNO085 candidate addresses (already probed via SHTP).
    i2c0_scan_and_record();

    if (!alive) {
        if (regs[ADDR_DBG_I2C0_NDEV] == 0u) regs[ADDR_DBG_BNO085_RC] = 3u;
        return false;
    }

    // Active address is now in bno085_runtime_addr.
    regs[ADDR_DBG_BNO085_RC] = 0u;  // success

    // Enable sensor reports at 50 Hz (20 000 µs period).
    // Each Set Feature command is followed by a short drain+delay so the
    // BNO085 has time to process the command and ACK it before the next one.
    const uint32_t period_us = 20000u;
    bno085_enable_report(SH2_ROTATION_VECTOR, period_us);
    sleep_ms(20u);
    // Drain any immediate responses (feature report ACKs on channel 2)
    for (int i = 0; i < 4; i++) { shtp_receive(tmp, sizeof(tmp), &ch); sleep_us(1000u); }

    bno085_enable_report(SH2_ACCELEROMETER,   period_us);
    sleep_ms(20u);
    for (int i = 0; i < 4; i++) { shtp_receive(tmp, sizeof(tmp), &ch); sleep_us(1000u); }

    bno085_enable_report(SH2_GYROSCOPE_CAL,   period_us);
    sleep_ms(20u);
    for (int i = 0; i < 4; i++) { shtp_receive(tmp, sizeof(tmp), &ch); sleep_us(1000u); }

    bno085_enable_report(SH2_MAG_FIELD_CAL,   period_us);
    sleep_ms(20u);
    for (int i = 0; i < 4; i++) { shtp_receive(tmp, sizeof(tmp), &ch); sleep_us(1000u); }

    // Enable ME (Motion Engine) dynamic calibration for accel, gyro, and mag.
    bno085_enable_me_calibration();
    sleep_ms(20u);

    // Wait for initial sensor reports to arrive (rotation vector must appear).
    bno085_wait_for_calibration(1000u);

    // Persist calibration offsets so subsequent power-ups start with good values.
    bno085_save_dcd();
    sleep_ms(10u);

    return true;
}


// ============================================================
// ODOMETRY  (dead-reckoning, open-loop, 50 Hz)
// ============================================================

static int32_t pos_left_ticks  = 0;
static int32_t pos_right_ticks = 0;

static void update_odometry(float dt) {
    bool torque_on = (regs[ADDR_MOTOR_TORQUE_EN] != 0);

    float lin_x = 0.0f;
    float ang_z = 0.0f;
    if (torque_on) {
        // Safety timeout: keep the last commanded velocity as long as the
        // host is alive (sending any Dynamixel packets — reads, writes,
        // heartbeat).  If the host disappears for HOST_TIMEOUT_US we
        // brake.  This lets the teleop keyboard work even though it only
        // publishes cmd_vel on keypress — the ROS node's 50 ms bulk
        // READ and 100 ms heartbeat keep last_host_comm_us fresh.
        uint64_t now = time_us_64();
        if (last_cmd_vel_us != 0 && last_host_comm_us != 0
            && (now - last_host_comm_us) < HOST_TIMEOUT_US) {
            // ROS node writes velocity in 0.01 m/s and 0.01 rad/s integer units.
            // Divide by 100 to recover SI values.
            lin_x = (float)r_i32(ADDR_CMD_LINEAR_X)  / 100.0f;  // → m/s
            ang_z = (float)r_i32(ADDR_CMD_ANGULAR_Z) / 100.0f;   // → rad/s
        }
        // else: no velocity written yet, or host timed out → lin_x/ang_z stay 0
    }

    float v_left  = lin_x - ang_z * (WHEEL_SEPARATION / 2.0f);
    float v_right = lin_x + ang_z * (WHEEL_SEPARATION / 2.0f);

    // Drive motors — apply per-motor trim before dead-zone/kick logic
    float thr_l = 0.0f, thr_r = 0.0f;
    if (torque_on) {
        thr_l = (v_left  / MAX_WHEEL_SPEED_MS) * MOTOR_TRIM_LEFT;
        thr_r = (v_right / MAX_WHEEL_SPEED_MS) * MOTOR_TRIM_RIGHT;
        if (SWAP_LEFT_RIGHT_MOTORS) {
            set_motor(pwm_slice_m1, RIGHT_MOTOR_REVERSED, thr_r, 1);
            set_motor(pwm_slice_m2, false,               thr_l, 0);
        } else {
            set_motor(pwm_slice_m1, false,               thr_l, 0);
            set_motor(pwm_slice_m2, RIGHT_MOTOR_REVERSED, thr_r, 1);
        }
        // LED feedback: on while motors are actively driven
        gpio_put(PIN_LED, (fabsf(thr_l) > 0.001f || fabsf(thr_r) > 0.001f));
    } else {
        brake_all();
        gpio_put(PIN_LED, false);
    }

    // Compute the effective velocity that matches what set_motor() actually applies.
    // set_motor() expands [0.001, 1.0] → [MOTOR_MIN_DUTY, 1.0] (dead-zone compensation).
    // Using the raw commanded v_left/v_right for tick accumulation would make odom
    // under-report actual motion, causing move_distance to stop too early (e.g. 5 cm
    // instead of 10 cm).  Apply the same expansion here so ticks reflect real motion.
    //
    // We use the UNTRIMMED throttle (v / MAX_WHEEL_SPEED_MS, no MOTOR_TRIM factor)
    // so that straight-line distance odom is unaffected by trim asymmetry corrections —
    // trim accounts for motor hardware differences, not for the commanded trajectory.
    float eff_v_left  = 0.0f;
    float eff_v_right = 0.0f;
    if (torque_on) {
        float raw_l = fabsf(v_left  / MAX_WHEEL_SPEED_MS);
        float raw_r = fabsf(v_right / MAX_WHEEL_SPEED_MS);
        if (raw_l > 0.001f) {
            float eff_l = (MOTOR_MIN_DUTY > 0.0f)
                          ? MOTOR_MIN_DUTY + raw_l * (1.0f - MOTOR_MIN_DUTY)
                          : raw_l;
            eff_v_left  = (v_left  >= 0.0f ? 1.0f : -1.0f) * eff_l * MAX_WHEEL_SPEED_MS;
        }
        if (raw_r > 0.001f) {
            float eff_r = (MOTOR_MIN_DUTY > 0.0f)
                          ? MOTOR_MIN_DUTY + raw_r * (1.0f - MOTOR_MIN_DUTY)
                          : raw_r;
            eff_v_right = (v_right >= 0.0f ? 1.0f : -1.0f) * eff_r * MAX_WHEEL_SPEED_MS;
        }
    }

    // Wheel velocities (Dynamixel RPM units) — report effective velocity
    w_i32(ADDR_PRESENT_VEL_L, (int32_t)(eff_v_left  * RPM_PER_MS));
    w_i32(ADDR_PRESENT_VEL_R, (int32_t)(eff_v_right * RPM_PER_MS));

    // Wheel positions (accumulating ticks) — use effective velocity
    float d_rad_l = (eff_v_left  * dt) / WHEEL_RADIUS;
    float d_rad_r = (eff_v_right * dt) / WHEEL_RADIUS;

    int64_t new_l = (int64_t)pos_left_ticks  + (int64_t)(d_rad_l * TICKS_PER_RAD);
    int64_t new_r = (int64_t)pos_right_ticks + (int64_t)(d_rad_r * TICKS_PER_RAD);

    // Wrap position counters on int32 overflow (wheel has turned many revolutions).
    // The ROS node treats these as signed 32-bit accumulators, so we wrap rather than clamp.
    if (new_l >  2147483647LL) new_l -= 4294967296LL;
    if (new_l < -2147483648LL) new_l += 4294967296LL;
    if (new_r >  2147483647LL) new_r -= 4294967296LL;
    if (new_r < -2147483648LL) new_r += 4294967296LL;

    pos_left_ticks  = (int32_t)new_l;
    pos_right_ticks = (int32_t)new_r;

    w_i32(ADDR_PRESENT_POS_L, pos_left_ticks);
    w_i32(ADDR_PRESENT_POS_R, pos_right_ticks);
}

// ============================================================
// SENSOR UPDATES  (20 Hz)
// ============================================================

static void update_sensors(uint64_t now_us) {
    // Timekeeping
    w_i32(ADDR_MILLIS, (int32_t)(now_us / 1000u));
    w_i32(ADDR_MICROS, (int32_t)(now_us));

    // Push buttons — active LOW
    regs[ADDR_BUTTON_1] = gpio_get(PIN_BTN1) ? 0 : 1;
    regs[ADDR_BUTTON_2] = gpio_get(PIN_BTN2) ? 0 : 1;

    // Battery — read X-UPS1 status via Grove 6 digital signals (GP26/GP27).
    // PLD = 1: mains disconnected (robot running on internal battery pack).
    // LBAT = 1: at least one cell has reached the 3 V low-voltage threshold.
    // Both signals are active-HIGH; pull-downs keep them 0 when disconnected.
    // BATTERY_VOLTAGE stored as (volts × 100), BATTERY_PERCENT as (pct × 100).
    bool ups_pld  = gpio_get(PIN_UPS_PLD);
    bool ups_lbat = gpio_get(PIN_UPS_LBAT);
    float vbatt, pct;
    if (ups_lbat) {
        vbatt = BATT_V_LOW;    // 12.0 V — critically low, cut-off imminent
        pct   = BATT_PCT_LOW;
    } else if (ups_pld) {
        vbatt = BATT_V_MID;    // 14.8 V — on battery, level unknown (> 25 %)
        pct   = BATT_PCT_MID;
    } else {
        vbatt = BATT_V_FULL;   // 16.8 V — on mains power / fully charged
        pct   = BATT_PCT_FULL;
    }
    w_i32(ADDR_BATTERY_VOLTAGE, (int32_t)(vbatt * 100.0f + 0.5f));
    w_i32(ADDR_BATTERY_PERCENT, (int32_t)(pct   * 100.0f + 0.5f));

    // Low-battery warning: play MEL_LOW_BATT once when LBAT asserts.
    // Reset the latch when LBAT de-asserts (mains restored or battery swapped).
    if (ups_lbat && !vbatt_low_alerted) {
        start_melody(2);  // 2 = MEL_LOW_BATT
        vbatt_low_alerted = true;
    } else if (!ups_lbat) {
        vbatt_low_alerted = false;
    }

    // Sound — trigger the melody sequencer when ADDR_SOUND changes.
    // The melody auto-clears ADDR_SOUND when it finishes, so the host
    // does not need to write 0 explicitly.
    uint8_t snd = regs[ADDR_SOUND];
    if (snd != mel_last_trigger) {
        start_melody(snd);
    }

    // IMU re-calibration trigger (host writes 1 to ADDR_IMU_RECAL).
    // Performs a real hardware recalibration on whichever IMU is connected,
    // or resets to static identity values if no hardware IMU is present.
    if (regs[ADDR_IMU_RECAL] == 1) {
        if (bno085_present) {
            // Re-enable ME calibration (resets internal offsets) and persist
            // the updated calibration data to on-chip flash.
            bno085_enable_me_calibration();
            sleep_ms(10u);
            bno085_save_dcd();
            sleep_ms(10u);
        } else {
            // No hardware IMU — reset to static identity values
            w_f32(ADDR_IMU_ORIENT_W,  1.0f);
            w_f32(ADDR_IMU_ORIENT_X,  0.0f);
            w_f32(ADDR_IMU_ORIENT_Y,  0.0f);
            w_f32(ADDR_IMU_ORIENT_Z,  0.0f);
            w_f32(ADDR_IMU_ANG_VEL_X, 0.0f);
            w_f32(ADDR_IMU_ANG_VEL_Y, 0.0f);
            w_f32(ADDR_IMU_ANG_VEL_Z, 0.0f);
            w_f32(ADDR_IMU_LIN_ACC_X, 0.0f);
            w_f32(ADDR_IMU_LIN_ACC_Y, 0.0f);
            w_f32(ADDR_IMU_LIN_ACC_Z, IMU_GRAVITY_Z);
        }
        regs[ADDR_IMU_RECAL] = 0;
    }

    // If a real IMU is connected, overwrite the simulated IMU registers with
    // live sensor data.  BNO085 (Grove 1, I2C0) is used when present.
    if (bno085_present) {
        bno085_update();
    }
}

// ============================================================
// TinyUSB CALLBACKS
// ============================================================

// Called when host asserts or drops DTR/RTS.
// INTENTIONALLY EMPTY — do not reset the firmware on DTR change.
// This is the critical fix: the DynamixelSDK asserts DTR when it opens
// the port, and the firmware must remain running to answer the first
// PING/READ within 34 ms.
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;
    (void)dtr;
    (void)rts;
}

// Called when CDC line coding changes (baud rate etc.)
// Implements the standard 1200-baud bootloader-entry trick so that
// picotool / build.sh can reboot the Pico into UF2 mode without
// needing physical access to the BOOTSEL button.
// Any other baud rate change is ignored.
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *p_line_coding) {
    (void)itf;
    if (p_line_coding->bit_rate == 1200) {
        reset_usb_boot(0, 0);  // reboot into RP2350 UF2 bootloader
    }
}

// ============================================================
// MAIN
// ============================================================

int main(void) {
    // Build CRC table
    make_crc_table();

    // Initialise control table registers
    init_registers();

    // Hardware init
    init_motors();
    init_buttons();
    init_ups_gpio();
    init_buzzer();
    init_adc();

    // Onboard LED for motor-activity feedback
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, false);

    // TinyUSB / board init FIRST so that board_init()'s stdio_uart_init_full()
    // claims GP0/GP1 as UART0 before the IMU drivers touch those pins.
    // init_bno085() runs afterwards and re-muxes GP0/GP1 to I2C0, which is the
    // last (and correct) function assignment for those pads.  This avoids the
    // previous race where board_init() ran after init_bno085() and silently
    // stole GP0/GP1 back to UART, resetting the BNO085's SHTP session.
    board_init();
    tud_init(0);

    // Play the startup melody (same ascending arpeggio as real TurtleBot3 OpenCR)
    // before the IMU probe so the ~500 ms calibration wait runs after the melody.
    play_melody_blocking(MEL_ON);

    // Probe IMU — BNO085 on Grove 1 (GP0/GP1, I2C0).
    // init_bno085() sets GP0/GP1 to GPIO_FUNC_I2C here; board_init() above
    // had set them to UART0 — this override is intentional and permanent.
    // Falls back to static simulated IMU if BNO085 is not detected.
    bno085_present = init_bno085();
    // Record which IMU source is active for the debug register.
    if (bno085_present) regs[ADDR_DBG_IMU_SOURCE] = 1u;
    else                regs[ADDR_DBG_IMU_SOURCE] = 0u;

    uint64_t last_odom_us   = time_us_64();
    uint64_t last_sensor_us = time_us_64();

    while (true) {
        // Drive the USB stack
        tud_task();

        uint64_t now = time_us_64();

        // Advance the melody sequencer every loop iteration for tight timing
        update_melody(now);

        // Odometry at 50 Hz
        if (now - last_odom_us >= ODOM_INTERVAL_US) {
            float dt = (float)(now - last_odom_us) / 1e6f;
            last_odom_us = now;
            update_odometry(dt);
        }

        // Sensor update at 20 Hz
        if (now - last_sensor_us >= SENSOR_INTERVAL_US) {
            last_sensor_us = now;
            update_sensors(now);
        }

        // Process any incoming Dynamixel bytes from the host
        if (tud_cdc_available()) {
            uint8_t buf[64];
            uint32_t count = tud_cdc_read(buf, sizeof(buf));
            for (uint32_t i = 0; i < count; i++) {
                parse_byte(buf[i]);
            }
        }
    }

    return 0;  // unreachable
}
