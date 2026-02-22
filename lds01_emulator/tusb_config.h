/*
 * tusb_config.h — TinyUSB configuration for the LDS-01 LIDAR Emulator
 *
 * One CDC ACM interface:  /dev/ttyACM0 on the host — configuration commands.
 *
 * IMPORTANT: tud_cdc_line_state_cb() is an intentional no-op so that
 * libraries that assert DTR when opening the port (e.g. pyserial) do not
 * trigger a firmware reset.
 */
#pragma once

// -------------------------------------------------------------------
// MCU & platform
// -------------------------------------------------------------------
#ifndef CFG_TUSB_MCU
#  define CFG_TUSB_MCU  OPT_MCU_RP2040   // RP2350 reuses the RP2040 USB driver
#endif

#define CFG_TUSB_OS     OPT_OS_PICO

// -------------------------------------------------------------------
// Device stack
// -------------------------------------------------------------------
#define CFG_TUD_ENABLED  1
// Two CDC ACM interfaces:
//   CDC 0 (/dev/ttyACM0) — text configuration commands
//   CDC 1 (/dev/ttyACM1) — raw LDS-01 22-byte binary packet stream
#define CFG_TUD_CDC      2

#define CFG_TUD_CDC_RX_BUFSIZE   256
#define CFG_TUD_CDC_TX_BUFSIZE   512
#define CFG_TUD_CDC_EP_BUFSIZE    64

// -------------------------------------------------------------------
// Host stack (unused)
// -------------------------------------------------------------------
#define CFG_TUH_ENABLED  0

// -------------------------------------------------------------------
// Root hub port — REQUIRED by TinyUSB 2.x  (replaces the no-arg tusb_init())
// -------------------------------------------------------------------
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
