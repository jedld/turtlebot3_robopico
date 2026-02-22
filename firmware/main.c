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
 *
 * Register map: control_table.hpp in turtlebot3_node
 */

#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
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
#define PIN_VBATT     28u     // GP28 = ADC2 = battery voltage sense (solder pad)
#define ADC_CH_VBATT  2u      // ADC channel for GP28

// Voltage divider on GP28 (per Robo Pico datasheet, Rev 1.0):
//   VADC2 = VBAT / 2   →   VBATT_DIVIDER = 2.0
// Two equal resistors form the divider; no adjustment needed.
#define VBATT_DIVIDER   2.0f

// ADC EMA filter coefficient for battery voltage (0.0 = frozen, 1.0 = no filter).
// 0.05 gives a ~20-sample time constant at 20 Hz → ~1 s settling time.
#define VBATT_EMA_ALPHA 0.05f

// Low-battery warning threshold (volts).  Plays MEL_LOW_BATT once when crossed.
// 3.4 V is a safe cutoff for a 1S LiPo (protection circuit trips at ~3.0 V).
#define VBATT_LOW_THRESH_V  3.4f

// Pico 2 system clock (set by SDK, 150 MHz)
#define SYS_CLK_HZ  150000000u

// Motor PWM: 10 kHz, 1000 resolution steps.
// clkdiv = 15  →  150 MHz / 15 / 1000 = 10 kHz
#define MOTOR_PWM_CLKDIV    15.0f
#define MOTOR_PWM_WRAP      999u    // duty range: 0 … 999

// Physical parameters
#define WHEEL_RADIUS        0.033f   // metres
#define WHEEL_SEPARATION    0.160f   // metres
#define MAX_WHEEL_SPEED_MS  0.22f    // m/s at full throttle
#define RIGHT_MOTOR_REVERSED 1       // set to 0 to invert

// Dynamixel XL430 velocity unit: 1 unit = 0.229 RPM
#define RPM_TO_MS   (0.229f * 0.0034557519189487725f)
#define RPM_PER_MS  (1.0f / RPM_TO_MS)

// Dynamixel position tick: ~0.001534 rad/tick (4096 ticks/rev)
#define TICK_TO_RAD     0.001533981f
#define TICKS_PER_RAD   (1.0f / TICK_TO_RAD)

// Simulated IMU: gravity on Z axis (robot is flat)
#define IMU_GRAVITY_Z   9.81f

// Battery model — Robo Pico uses a single-cell (1S) LiPo/Li-Ion.
// VBAT = VIN or VUSB or VLiPo, whichever is highest (per power tree).
#define BATT_CELLS  1        // 1S LiPo/Li-Ion
#define BATT_MIN_V  3.0f     // per-cell cutoff (V)
#define BATT_MAX_V  4.2f     // per-cell full charge (V)

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
                                      // NOTE: address 170 matches turtlebot3_node control_table.hpp;
                                      //       verify if upgrading the ROS package.

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

    // Battery initial placeholder — updated to real reading within first 50 ms.
    // Values match 1S LiPo mid-charge: 3.70 V, 58 %.
    w_i32(ADDR_BATTERY_VOLTAGE,  370);
    w_i32(ADDR_BATTERY_PERCENT,  5833);

    // Motor torque enabled by default
    regs[ADDR_MOTOR_TORQUE_EN] = 1;
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

// Maximum response: READ of all 256 registers = 4+1+2+1+1+256+2 = 267 bytes
#define RESP_BUF_SIZE 300u
static uint8_t resp_buf[RESP_BUF_SIZE];

static uint32_t build_status(uint8_t dev_id, uint8_t error,
                              const uint8_t *data, uint16_t data_len) {
    uint16_t length = (uint16_t)(data_len + 4u);  // inst + err + data + crc
    uint32_t i = 0;
    resp_buf[i++] = 0xFF;
    resp_buf[i++] = 0xFF;
    resp_buf[i++] = 0xFD;
    resp_buf[i++] = 0x00;
    resp_buf[i++] = dev_id;
    resp_buf[i++] = (uint8_t)(length);
    resp_buf[i++] = (uint8_t)(length >> 8);
    resp_buf[i++] = 0x55;    // STATUS instruction
    resp_buf[i++] = error;
    if (data && data_len > 0) {
        memcpy(&resp_buf[i], data, data_len);
        i += data_len;
    }
    uint16_t crc = dxl_crc16(resp_buf, i);
    resp_buf[i++] = (uint8_t)(crc);
    resp_buf[i++] = (uint8_t)(crc >> 8);
    return i;
}

// ============================================================
// DYNAMIXEL PROTOCOL 2.0  —  INSTRUCTION HANDLERS
// ============================================================

// Send a pre-built packet
static void send_packet(uint32_t len) {
    tud_cdc_write(resp_buf, len);
    tud_cdc_write_flush();
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

static void handle_read(uint8_t dev_id, const uint8_t *params, uint8_t nparams) {
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

static void handle_write(uint8_t dev_id, const uint8_t *params, uint8_t nparams,
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
    uint8_t  ndata   = (uint8_t)(nparams - 2);

    if ((uint32_t)addr + ndata > REG_SIZE) {
        if (send_reply) {
            uint32_t len = build_status(dev_id, 0x02, NULL, 0);
            send_packet(len);
        }
        return;
    }
    memcpy(&regs[addr], data, ndata);
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

    // Verify CRC: covers [0xFF 0xFF 0xFD 0x00] + pkt_buf[0..pos-3]
    uint8_t crc_buf[4 + PKT_BUF_SIZE];
    crc_buf[0] = 0xFF; crc_buf[1] = 0xFF;
    crc_buf[2] = 0xFD; crc_buf[3] = 0x00;
    memcpy(&crc_buf[4], pkt_buf, pkt_pos - 2);  // exclude last 2 = CRC bytes
    uint16_t crc_calc = dxl_crc16(crc_buf, 4 + pkt_pos - 2);
    uint16_t crc_recv = (uint16_t)(pkt_buf[pkt_pos - 2]
                                 | ((uint16_t)pkt_buf[pkt_pos - 1] << 8));
    if (crc_calc != crc_recv) return;  // silently discard

    uint8_t instruction = pkt_buf[3];
    const uint8_t *params = &pkt_buf[4];
    // pkt_buf layout:  [ID(1), LEN_L(1), LEN_H(1), INST(1), PARAMS(n), CRC_L(1), CRC_H(1)]
    // pkt_pos = total bytes stored = 4 + n + 2, so n = pkt_pos - 6
    uint8_t nparams = (uint8_t)(pkt_pos - 6);  // number of parameter bytes

    // Broadcast (0xFE): only process writes, never send a reply
    bool send_reply = (dev_id != 0xFE);

    switch (instruction) {
        case 0x01:  // PING
            handle_ping(dev_id);
            break;
        case 0x02:  // READ
            if (send_reply) handle_read(dev_id, params, nparams);
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
static void set_motor(uint slice, bool reversed, float throttle) {
    if (throttle < -1.0f) throttle = -1.0f;
    if (throttle >  1.0f) throttle =  1.0f;
    if (reversed) throttle = -throttle;

    uint16_t duty = (uint16_t)(fabsf(throttle) * MOTOR_PWM_WRAP);

    if (throttle > 0.001f) {
        pwm_set_chan_level(slice, PWM_CHAN_A, duty);  // forward
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    } else if (throttle < -0.001f) {
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice, PWM_CHAN_B, duty);  // reverse
    } else {
        // Brake (both low = coasting; both high = active brake)
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    }
}

static void brake_all(void) {
    set_motor(pwm_slice_m1, false, 0.0f);
    set_motor(pwm_slice_m2, false, 0.0f);
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
// BATTERY ADC
// ============================================================

static void init_adc(void) {
    adc_init();
    adc_gpio_init(29);          // GP29 = ADC3 = VSYS (Pico 2 internal monitor)
    adc_gpio_init(PIN_VBATT);   // GP28 = ADC2 = Vbatt sense (solder pad)
}

// Read VSYS (Pico 2 supply rail) via GP29's internal 3:1 divider.
static float read_vsys(void) {
    adc_select_input(ADC_CH_VSYS);
    uint16_t raw = adc_read();
    return (float)raw * 3.0f * 3.3f / 4096.0f;
}

// EMA-filtered battery voltage state (initialised to a mid-range value).
static float vbatt_filtered = (float)BATT_CELLS * (BATT_MIN_V + BATT_MAX_V) * 0.5f;
static bool  vbatt_low_alerted = false;  // latch so warning plays only once

// Read and return the EMA-filtered battery voltage via GP28.
// Call at a fixed rate (SENSOR_INTERVAL_US) for consistent filtering.
static float read_vbatt_filtered(void) {
    adc_select_input(ADC_CH_VBATT);
    uint16_t raw = adc_read();                          // 12-bit: 0–4095
    float v_pin  = (float)raw * 3.3f / 4096.0f;        // voltage at ADC pin
    float v_batt = v_pin * VBATT_DIVIDER;               // scale back through divider
    // Exponential moving average to suppress ADC noise
    vbatt_filtered = VBATT_EMA_ALPHA * v_batt
                   + (1.0f - VBATT_EMA_ALPHA) * vbatt_filtered;
    return vbatt_filtered;
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
        // ROS node writes velocity in 0.01 m/s and 0.01 rad/s integer units.
        // Divide by 100 to recover SI values.
        lin_x = (float)r_i32(ADDR_CMD_LINEAR_X)  / 100.0f;  // → m/s
        ang_z = (float)r_i32(ADDR_CMD_ANGULAR_Z) / 100.0f;  // → rad/s
    }

    float v_left  = lin_x - ang_z * (WHEEL_SEPARATION / 2.0f);
    float v_right = lin_x + ang_z * (WHEEL_SEPARATION / 2.0f);

    // Drive motors
    if (torque_on) {
        float thr_l = v_left  / MAX_WHEEL_SPEED_MS;
        float thr_r = v_right / MAX_WHEEL_SPEED_MS;
        set_motor(pwm_slice_m1, false,               thr_l);
        set_motor(pwm_slice_m2, RIGHT_MOTOR_REVERSED, thr_r);
    } else {
        brake_all();
    }

    // Wheel velocities (Dynamixel RPM units)
    w_i32(ADDR_PRESENT_VEL_L, (int32_t)(v_left  * RPM_PER_MS));
    w_i32(ADDR_PRESENT_VEL_R, (int32_t)(v_right * RPM_PER_MS));

    // Wheel positions (accumulating ticks)
    float d_rad_l = (v_left  * dt) / WHEEL_RADIUS;
    float d_rad_r = (v_right * dt) / WHEEL_RADIUS;

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

    // Battery — read real Vbatt from GP28 (EMA-filtered).
    // BATTERY_VOLTAGE stored as (volts × 100), e.g. 7.40 V → 740.
    // BATTERY_PERCENT stored as (percent × 100), e.g. 75.00 % → 7500.
    float vbatt = read_vbatt_filtered();
    float pct   = (vbatt - (float)BATT_CELLS * BATT_MIN_V)
                / ((float)BATT_CELLS * (BATT_MAX_V - BATT_MIN_V)) * 100.0f;
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    w_i32(ADDR_BATTERY_VOLTAGE, (int32_t)(vbatt * 100.0f));
    w_i32(ADDR_BATTERY_PERCENT, (int32_t)(pct   * 100.0f));

    // Low-battery warning: play MEL_LOW_BATT once when voltage drops below threshold.
    // Reset the latch if voltage recovers (e.g. battery swapped).
    if (vbatt < VBATT_LOW_THRESH_V && !vbatt_low_alerted) {
        start_melody(2);  // 2 = MEL_LOW_BATT
        vbatt_low_alerted = true;
    } else if (vbatt >= VBATT_LOW_THRESH_V + 0.2f) {
        vbatt_low_alerted = false;  // hysteresis — reset latch after recovery
    }

    // Sound — trigger the melody sequencer when ADDR_SOUND changes.
    // The melody auto-clears ADDR_SOUND when it finishes, so the host
    // does not need to write 0 explicitly.
    uint8_t snd = regs[ADDR_SOUND];
    if (snd != mel_last_trigger) {
        start_melody(snd);
    }

    // IMU re-calibration trigger (host writes 1)
    if (regs[ADDR_IMU_RECAL] == 1) {
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
        regs[ADDR_IMU_RECAL] = 0;
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
    init_buzzer();
    init_adc();

    // Play the startup melody (same ascending arpeggio as real TurtleBot3 OpenCR)
    // before USB enumeration so it plays on every power-on, regardless of host connection.
    play_melody_blocking(MEL_ON);

    // TinyUSB init — must be called before entering the loop.
    // tud_init(0): initialise USB device on physical port 0 (the only PHY on RP2350).
    // SDK / TinyUSB ≥ 2.0 renamed tusb_init() → tud_init(port); do NOT revert to
    // the no-argument tusb_init() form or the build will fail with a cryptic
    // 'CFG_TUSB_RHPORT*_MODE must be defined' error.
    board_init();
    tud_init(0);

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
