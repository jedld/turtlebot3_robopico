/*
 * main.cpp — LDS-01 LIDAR Emulator Firmware for Raspberry Pi Pico 2
 * ==================================================================
 * Emulates a ROBOTIS HLS-LFCD-LDS01 (TurtleBot3 LDS-01) LIDAR sensor.
 * Outputs the authentic 22-byte binary packet stream on UART0 at 230 400 baud,
 * exactly as the real sensor does, while synthesising distance data for a
 * configurable rectangular room with the robot at a configurable position.
 *
 * Hardware connections (Pico 2 / Cytron Robo Pico)
 * -------------------------------------------------
 *   USB CDC 0  →  /dev/ttyACM0 — text configuration commands
 *   USB CDC 1  →  /dev/ttyACM1 — raw LDS-01 22-byte binary packet stream
 *                 (connect hls_lfcd_lds_driver port:=/dev/ttyACM1)
 *   GP0 (UART0 TX)  →  optional physical serial output (230 400 baud)
 *                       for direct connection to a UART-only host
 *
 * LDS-01 packet format per hls_lfcd_lds_driver source (42 bytes)
 * ---------------------------------------------------------------
 * The driver reads a COMPLETE 2520-byte rotation frame beginning at FA A0.
 * The frame is 60 packets × 42 bytes each.
 *
 * Per-packet layout (42 bytes):
 *   [0]      0xFA
 *   [1]      0xA0–0xDB         packet index (60 packets per rotation)
 *   [2–3]    speed (uint16 LE)  speed_cps = RPM × 10 → driver computes rpms = speed/10
 *   [4–39]   6 × 6-byte samples per packet:
 *              byte 0–1: intensity (uint16 LE)
 *              byte 2–3: range    (uint16 LE)  in mm
 *              byte 4–5: reserved (zeros)
 *   [40–41]  checksum (uint16 LE)   LFCD algorithm over bytes 0–39
 *
 * Angular mapping (to match driver's `scan->ranges[359 - index]`):
 *   scan_index = 6 × pkt_idx + sample_k
 *   angle_deg  = 359 − scan_index
 *
 * Reference: ROBOTIS-GIT/hls_lfcd_lds_driver master/src/hlds_laser_publisher.cpp
 *
 * Configuration commands (USB CDC, newline-terminated)
 * ----------------------------------------------------
 *   ROOM <W> <H>    Set room width W × height H in metres  (default 5.0 × 5.0)
 *   POS  <X> <Y>    Set robot position in metres          (default 2.5 × 2.5)
 *   ANGLE <deg>     Set robot heading (LDS angle-0 offset) (default 0.0°)
 *   RPM  <val>      Set simulated scan RPM 100–600         (default 300)
 *   STATUS          Print current configuration
 *   HELP            Print command list
 *
 * Build constraints (same as sibling firmware/)
 * ---------------------------------------------
 *   • Target board: pico2 (RP2350) — do NOT use pico (RP2040)
 *   • tud_init(0), not tusb_init()
 *   • CFG_TUSB_RHPORT0_MODE defined in tusb_config.h
 *   • pico_enable_stdio_usb and pico_enable_stdio_uart both = 0
 *   • tud_cdc_line_state_cb() is an intentional no-op
 */

#include <cstring>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/bootrom.h"
#include "hardware/uart.h"
#include "bsp/board.h"
#include "tusb.h"

// ============================================================
// COMPILE-TIME CONFIGURATION
// ============================================================

// LDS-01 UART
// uart0 expands to a reinterpret_cast which is not a constant expression in
// C++17; use a plain const pointer instead.
static uart_inst_t * const LDS_UART  = uart0;
constexpr uint         LDS_UART_TX  = 0u;   // GP0
constexpr uint         LDS_UART_RX  = 1u;   // GP1
constexpr uint32_t     LDS_BAUD     = 230400u;

// Scan geometry
constexpr int     LDS_SAMPLES    = 360;   // samples per full rotation
constexpr int     LDS_PACKETS    = 60;    // 6 samples per packet × 60 = 360
constexpr int     LDS_SAMPLES_PER_PKT = 6;
constexpr uint8_t LDS_SYNC       = 0xFA;
constexpr uint8_t LDS_IDX_FIRST  = 0xA0; // index of packet 0
constexpr int     LDS_PKT_BYTES  = 42;   // 2 hdr + 2 speed + 36 samples + 2 chk

// Physical limits of the real LDS-01 (millimetres)
constexpr uint16_t LDS_MIN_MM    = 120u;
constexpr uint16_t LDS_MAX_MM    = 3500u;

// Simulated quality signal (arbitrary; real sensor uses ~500–800 in open air)
constexpr uint16_t QUALITY_NOMINAL = 700u;

// Default room and robot pose
constexpr float DEFAULT_ROOM_W    = 5.0f;  // metres
constexpr float DEFAULT_ROOM_H    = 5.0f;
constexpr float DEFAULT_ROBOT_X   = 2.5f;
constexpr float DEFAULT_ROBOT_Y   = 2.5f;
constexpr float DEFAULT_ROBOT_HDG = 0.0f;  // degrees; 0 = forward (+X)
constexpr float DEFAULT_RPM       = 300.0f;

// ============================================================
// MUTABLE STATE  (updated by CDC command processor)
// ============================================================

static float g_room_w    = DEFAULT_ROOM_W;
static float g_room_h    = DEFAULT_ROOM_H;
static float g_robot_x   = DEFAULT_ROBOT_X;
static float g_robot_y   = DEFAULT_ROBOT_Y;
static float g_robot_hdg = DEFAULT_ROBOT_HDG;
static float g_rpm       = DEFAULT_RPM;

// ============================================================
// ROOM GEOMETRY — RAY CASTING
// ============================================================
//
// The robot sits at (g_robot_x, g_robot_y) inside an axis-aligned rectangle
// [0, g_room_w] × [0, g_room_h].
//
// angle_deg:
//   0°   = robot's forward direction (after adding g_robot_hdg offset)
//   CCW positive (standard mathematical / ROS convention)
//
// Returns the nearest wall distance in millimetres.
// Clamps to [LDS_MIN_MM, LDS_MAX_MM].

static uint16_t ray_distance_mm(float angle_deg) {
    // Convert to world angle by adding the robot heading offset.
    const float rad = (angle_deg + g_robot_hdg) * static_cast<float>(M_PI) / 180.0f;
    const float dx  = cosf(rad);
    const float dy  = sinf(rad);

    constexpr float EPS = 1e-9f;
    float t_min = 1e9f;

    // Right wall  (x = g_room_w)
    if (dx > EPS) {
        float t = (g_room_w - g_robot_x) / dx;
        if (t > 0.0f && t < t_min) t_min = t;
    }
    // Left wall   (x = 0)
    if (dx < -EPS) {
        float t = -g_robot_x / dx;
        if (t > 0.0f && t < t_min) t_min = t;
    }
    // Top wall    (y = g_room_h)
    if (dy > EPS) {
        float t = (g_room_h - g_robot_y) / dy;
        if (t > 0.0f && t < t_min) t_min = t;
    }
    // Bottom wall (y = 0)
    if (dy < -EPS) {
        float t = -g_robot_y / dy;
        if (t > 0.0f && t < t_min) t_min = t;
    }

    uint32_t mm = static_cast<uint32_t>(t_min * 1000.0f);
    if (mm < LDS_MIN_MM) mm = LDS_MIN_MM;
    if (mm > LDS_MAX_MM) mm = LDS_MAX_MM;
    return static_cast<uint16_t>(mm);
}

// ============================================================
// LDS-01 PACKET BUILDER
// ============================================================

// LFCD checksum over the first 40 bytes of the 42-byte packet.
static uint16_t lds_checksum(const uint8_t *pkt) {
    uint32_t acc = 0u;
    for (int i = 0; i < 40; i += 2) {
        acc = (acc << 1u) + pkt[i] + (static_cast<uint32_t>(pkt[i + 1]) << 8u);
    }
    const uint32_t chk = ((acc & 0x7FFFu) + (acc >> 15u)) & 0x7FFFu;
    return static_cast<uint16_t>(chk);
}

// Build the 42-byte LDS-01 packet for zero-based index pkt_idx (0–59).
//
// Angular mapping (matches driver's `scan->ranges[359 - scan_index]`):
//   scan_index = LDS_SAMPLES_PER_PKT * pkt_idx + k
//   angle stored at scan->ranges[359 - scan_index]
//   → emit ray at angle (359 - scan_index) degrees
//
// speed_cps = RPM × 10  (driver computes rpms = accumulated_speed / good_sets / 10)
static void build_packet(uint8_t (&pkt)[LDS_PKT_BYTES],
                         int      pkt_idx,
                         uint16_t speed_cps) {
    pkt[0] = LDS_SYNC;
    pkt[1] = static_cast<uint8_t>(LDS_IDX_FIRST + pkt_idx);
    pkt[2] = static_cast<uint8_t>(speed_cps & 0xFFu);
    pkt[3] = static_cast<uint8_t>(speed_cps >> 8u);

    for (int k = 0; k < LDS_SAMPLES_PER_PKT; k++) {
        const int   scan_index = LDS_SAMPLES_PER_PKT * pkt_idx + k;
        const float angle_deg  = static_cast<float>(359 - scan_index);
        const uint16_t dist_mm = ray_distance_mm(angle_deg);

        // Per-sample layout: [int_L][int_H][dist_L][dist_H][0][0]
        const int off = 4 + k * 6;
        pkt[off + 0] = static_cast<uint8_t>(QUALITY_NOMINAL & 0xFFu); // intensity_L
        pkt[off + 1] = static_cast<uint8_t>(QUALITY_NOMINAL >> 8u);   // intensity_H
        pkt[off + 2] = static_cast<uint8_t>(dist_mm & 0xFFu);          // range_L
        pkt[off + 3] = static_cast<uint8_t>(dist_mm >> 8u);            // range_H
        pkt[off + 4] = 0;                                               // reserved
        pkt[off + 5] = 0;                                               // reserved
    }

    // Bytes 40–41: checksum (driver validates sync/index but not checksum;
    // compute anyway for robustness / future driver versions).
    const uint16_t chk = lds_checksum(pkt);
    pkt[40] = static_cast<uint8_t>(chk & 0xFFu);
    pkt[41] = static_cast<uint8_t>(chk >> 8u);
}

// ============================================================
// USB CDC  —  CONFIGURATION COMMAND PROCESSOR
// ============================================================

static constexpr int CMD_BUF_SIZE = 128;
static char     g_cmd_buf[CMD_BUF_SIZE];
static int      g_cmd_len = 0;

// Write a null-terminated string to the USB CDC TX FIFO and flush.
static void cdc_print(const char *s) {
    tud_cdc_write_str(s);
    tud_cdc_write_flush();
}

// Process one complete, null-terminated command line.
static void process_command(const char *cmd) {
    float a = 0.0f, b = 0.0f;

    if (sscanf(cmd, "ROOM %f %f", &a, &b) == 2) {
        if (a >= 0.5f && a <= 200.0f && b >= 0.5f && b <= 200.0f) {
            g_room_w = a;
            g_room_h = b;
            // Clamp robot position inside new room boundaries.
            g_robot_x = std::min(g_robot_x, g_room_w - 0.05f);
            g_robot_y = std::min(g_robot_y, g_room_h - 0.05f);
            g_robot_x = std::max(g_robot_x, 0.05f);
            g_robot_y = std::max(g_robot_y, 0.05f);
            cdc_print("OK: room updated\r\n");
        } else {
            cdc_print("ERR: room dimensions must be 0.5–200 m\r\n");
        }

    } else if (sscanf(cmd, "POS %f %f", &a, &b) == 2) {
        if (a >= 0.0f && a <= g_room_w && b >= 0.0f && b <= g_room_h) {
            g_robot_x = a;
            g_robot_y = b;
            cdc_print("OK: position updated\r\n");
        } else {
            cdc_print("ERR: position outside room bounds\r\n");
        }

    } else if (sscanf(cmd, "ANGLE %f", &a) == 1) {
        // Normalise to [0, 360)
        a = fmodf(a, 360.0f);
        if (a < 0.0f) a += 360.0f;
        g_robot_hdg = a;
        cdc_print("OK: heading updated\r\n");

    } else if (sscanf(cmd, "RPM %f", &a) == 1) {
        if (a >= 100.0f && a <= 600.0f) {
            g_rpm = a;
            cdc_print("OK: RPM updated\r\n");
        } else {
            cdc_print("ERR: RPM must be 100–600\r\n");
        }

    } else if (strncmp(cmd, "STATUS", 6) == 0) {
        char buf[320];
        snprintf(buf, sizeof(buf),
            "=== LDS-01 Emulator Status ===\r\n"
            "  Room     : %.2f x %.2f m\r\n"
            "  Position : (%.3f, %.3f) m\r\n"
            "  Heading  : %.1f deg\r\n"
            "  RPM      : %.1f\r\n"
            "  UART     : GP%u @ %lu baud\r\n",
            g_room_w, g_room_h,
            g_robot_x, g_robot_y,
            g_robot_hdg,
            g_rpm,
            LDS_UART_TX,
            static_cast<unsigned long>(LDS_BAUD));
        cdc_print(buf);

    } else if (strncmp(cmd, "HELP", 4) == 0) {
        cdc_print(
            "=== LDS-01 Emulator Commands ===\r\n"
            "  ROOM <W> <H>     Set room size (metres, 0.5–200)\r\n"
            "  POS  <X> <Y>     Set robot position (metres)\r\n"
            "  ANGLE <deg>      Set robot heading (degrees, 0=forward)\r\n"
            "  RPM  <val>       Set scan RPM (100–600)\r\n"
            "  STATUS           Show current configuration\r\n"
            "  HELP             Show this list\r\n");

    } else if (cmd[0] != '\0') {
        cdc_print("ERR: unknown command — try HELP\r\n");
    }
}

// Called every main-loop iteration; accumulates characters into g_cmd_buf
// and dispatches on CR / LF.
static void poll_cdc(void) {
    if (!tud_cdc_available()) return;

    uint8_t raw[64];
    const uint32_t n = tud_cdc_read(raw, sizeof(raw));
    for (uint32_t i = 0; i < n; i++) {
        const char c = static_cast<char>(raw[i]);
        if (c == '\r' || c == '\n') {
            if (g_cmd_len > 0) {
                g_cmd_buf[g_cmd_len] = '\0';
                // Echo back so interactive terminals feel connected.
                tud_cdc_write(reinterpret_cast<const uint8_t *>(g_cmd_buf),
                              static_cast<uint32_t>(g_cmd_len));
                tud_cdc_write_str("\r\n");
                tud_cdc_write_flush();
                process_command(g_cmd_buf);
                g_cmd_len = 0;
            }
        } else if (g_cmd_len < CMD_BUF_SIZE - 1) {
            g_cmd_buf[g_cmd_len++] = c;
        }
    }
}

// ============================================================
// TinyUSB CALLBACKS
// ============================================================

// Intentional no-op: asserting/dropping DTR must NOT reset the firmware.
// See AGENTS.md § "Hard constraints".
extern "C" void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;
    (void)dtr;
    (void)rts;
}

// 1200-baud reboot trick — allows picotool / build.sh to enter UF2 mode
// without physical access to the BOOTSEL button.
// ONLY applied to interface 0 (config port).  Interface 1 (LIDAR stream)
// is opened by hls_lfcd_lds_driver at 230 400 baud — never reboot from that.
extern "C" void tud_cdc_line_coding_cb(uint8_t itf,
                                        cdc_line_coding_t const *coding) {
    if (itf == 0 && coding->bit_rate == 1200u) {
        reset_usb_boot(0, 0);
    }
    (void)itf;
}

// ============================================================
// MAIN
// ============================================================

int main(void) {
    // ── UART init (LDS-01 output) ────────────────────────────────────────
    uart_init(LDS_UART, LDS_BAUD);
    gpio_set_function(LDS_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(LDS_UART_RX, GPIO_FUNC_UART);
    uart_set_format(LDS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(LDS_UART, true);

    // ── TinyUSB init (CDC configuration port) ───────────────────────────
    board_init();
    tud_init(0);

    // Brief startup banner on the CDC port (visible once the host opens the port).
    // We attempt to send it immediately; if the host isn't connected yet the
    // bytes sit in the TX FIFO until the port is opened.
    cdc_print("\r\n=== LDS-01 LIDAR Emulator (Pico 2) ===\r\n"
              "Type HELP for command list.\r\n");

    uint8_t pkt[LDS_PKT_BYTES];
    int     pkt_idx    = 0;                  // 0–89
    uint64_t last_pkt_us = time_us_64();

    while (true) {
        // Drive the USB stack — must be called as frequently as possible.
        tud_task();

        // Process any incoming configuration bytes on CDC0 (config port).
        poll_cdc();

        // Drain CDC1 RX (LIDAR stream port).
        // hls_lfcd_lds_driver sends 'b' (0x62) on open to start the motor and
        // 'e' (0x65) on close to stop it.  We absorb these silently — the
        // emulator is always "spinning" at g_rpm.
        if (tud_cdc_n_available(1)) {
            uint8_t discard[64];
            tud_cdc_n_read(1, discard, sizeof(discard));
        }

        // ── LDS-01 packet timing ─────────────────────────────────────────
        //
        // time_per_rotation  = 60 s / RPM
        // time_per_packet_us = 60 000 000 / RPM / LDS_PACKETS
        //                    = 1 000 000 / RPM  (µs)   [since LDS_PACKETS=60]
        //
        // At 300 RPM this is 3 333 µs/packet.
        // UART TX of 42 bytes at 230 400 baud takes ~1 823 µs — safely less.

        const uint32_t pkt_period_us =
            static_cast<uint32_t>(1000000.0f / g_rpm);

        const uint64_t now = time_us_64();
        if (now - last_pkt_us >= pkt_period_us) {
            last_pkt_us = now;

            const uint16_t speed_cps =
                static_cast<uint16_t>(g_rpm * 10.0f);  // driver: rpms = speed/10

            build_packet(pkt, pkt_idx, speed_cps);

            // ── USB CDC 1: primary path (host LIDAR driver reads /dev/ttyACM1) ──
            if (tud_cdc_n_connected(1)) {
                tud_cdc_n_write(1, pkt, LDS_PKT_BYTES);
                tud_cdc_n_write_flush(1);
            }

            // ── UART0 GP0: optional physical serial output ────────────────────
            uart_write_blocking(LDS_UART, pkt, LDS_PKT_BYTES);

            pkt_idx = (pkt_idx + 1) % LDS_PACKETS;
        }
    }

    return 0;  // unreachable
}
