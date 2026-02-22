/*
 * tusb_config.h — TinyUSB configuration for TurtleBot3 Pico 2 firmware
 *
 * Single USB CDC ACM device:
 *   /dev/ttyACM0 on the host — Dynamixel Protocol 2.0 data port
 *
 * Key design choice: tud_cdc_line_state_cb() is intentionally left as a
 * no-op so that asserting / dropping DTR (which pyserial does when opening
 * the port) never triggers a firmware reset.  This is the fix for the
 * "Failed connection with Devices" race condition that plagued the
 * CircuitPython implementation.
 */
#pragma once

// -------------------------------------------------------------------
// MCU & platform
// -------------------------------------------------------------------
// RP2350 (Pico 2) reuses the same USB peripheral as RP2040.
#ifndef CFG_TUSB_MCU
#  define CFG_TUSB_MCU  OPT_MCU_RP2040
#endif

// Operating system (bare-metal)
#define CFG_TUSB_OS     OPT_OS_PICO

// -------------------------------------------------------------------
// Device stack
// -------------------------------------------------------------------
#define CFG_TUD_ENABLED  1

// One CDC ACM interface
#define CFG_TUD_CDC      1

// RX/TX FIFO sizes (bytes).  256 bytes is plenty for a 14-byte Dynamixel
// request and a 267-byte (max READ) response.
#define CFG_TUD_CDC_RX_BUFSIZE  256
#define CFG_TUD_CDC_TX_BUFSIZE  512

// -------------------------------------------------------------------
// Host stack (not used)
// -------------------------------------------------------------------
#define CFG_TUH_ENABLED  0

// One endpoint pair for CDC data
#define CFG_TUD_CDC_EP_BUFSIZE  64

// -------------------------------------------------------------------
// Root hub port (RP2040/RP2350 has a single USB PHY on port 0)
// Required by TinyUSB 2.x when calling tusb_init() with no arguments.
// OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED = standard USB 2.0 FS device.
// -------------------------------------------------------------------
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
