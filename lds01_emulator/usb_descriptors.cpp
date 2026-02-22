/*
 * usb_descriptors.cpp — USB device / configuration / string descriptors
 * for the LDS-01 LIDAR Emulator.
 *
 * Presents as a single USB CDC ACM virtual serial port.
 * The host creates /dev/ttyACM0 (Linux) for configuration commands.
 *
 * Intentionally does NOT reset on DTR assertion.
 */

#include "tusb.h"
#include <cstring>

// -------------------------------------------------------------------
// Device descriptor
// -------------------------------------------------------------------
static const tusb_desc_device_t s_desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // IAD device class (required when using CDC alongside other classes)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0x2E8A,   // Raspberry Pi
    .idProduct          = 0x000B,   // LDS-01 Emulator (distinct from OpenCR emulator 0x000A)
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01,
};

extern "C" const uint8_t *tud_descriptor_device_cb(void) {
    return reinterpret_cast<const uint8_t *>(&s_desc_device);
}

// -------------------------------------------------------------------
// Configuration descriptor — two CDC ACM interfaces
// -------------------------------------------------------------------
//   CDC 0 (interfaces 0+1): config commands  → /dev/ttyACM0
//   CDC 1 (interfaces 2+3): LDS-01 LIDAR stream → /dev/ttyACM1
// -------------------------------------------------------------------
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + 2 * TUD_CDC_DESC_LEN)

// CDC 0 — config port
#define EPNUM_CDC0_NOTIFY  0x81   // EP1 IN  — CDC0 notification
#define EPNUM_CDC0_OUT     0x02   // EP2 OUT — CDC0 data RX
#define EPNUM_CDC0_IN      0x82   // EP2 IN  — CDC0 data TX

// CDC 1 — LIDAR stream
#define EPNUM_CDC1_NOTIFY  0x83   // EP3 IN  — CDC1 notification
#define EPNUM_CDC1_OUT     0x04   // EP4 OUT — CDC1 data RX
#define EPNUM_CDC1_IN      0x84   // EP4 IN  — CDC1 data TX

static const uint8_t s_desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(
        1,                  // bConfigurationValue
        4,                  // bNumInterfaces (2 CDC × 2 interfaces each)
        0,                  // iConfiguration
        CONFIG_TOTAL_LEN,
        0x00,               // bmAttributes: bus-powered
        250                 // bMaxPower: 500 mA
    ),
    // CDC 0 — configuration command port
    TUD_CDC_DESCRIPTOR(
        0,                  // Interface number (control=0, data=1)
        4,                  // iInterface string index
        EPNUM_CDC0_NOTIFY,
        8,
        EPNUM_CDC0_OUT,
        EPNUM_CDC0_IN,
        64
    ),
    // CDC 1 — LDS-01 raw packet stream
    TUD_CDC_DESCRIPTOR(
        2,                  // Interface number (control=2, data=3)
        5,                  // iInterface string index
        EPNUM_CDC1_NOTIFY,
        8,
        EPNUM_CDC1_OUT,
        EPNUM_CDC1_IN,
        64
    ),
};

extern "C" const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return s_desc_configuration;
}

// -------------------------------------------------------------------
// String descriptors
// -------------------------------------------------------------------
static const uint16_t s_desc_lang[] = {
    (TUSB_DESC_STRING << 8u) | 4u,
    0x0409  // English (US)
};

static const char * const s_string_table[] = {
    nullptr,                    // 0: Language (handled above)
    "Raspberry Pi",             // 1: Manufacturer
    "LDS-01 LIDAR Emulator",    // 2: Product
    "LDS01EMU0001",             // 3: Serial
    "LDS-01 Config Port",       // 4: CDC 0 interface
    "LDS-01 LIDAR Stream",      // 5: CDC 1 interface
};

static uint16_t s_desc_str[32];

extern "C" const uint16_t *tud_descriptor_string_cb(uint8_t index,
                                                      uint16_t langid) {
    (void)langid;

    if (index == 0) {
        memcpy(s_desc_str, s_desc_lang, sizeof(s_desc_lang));
        return s_desc_str;
    }

    const auto table_size =
        static_cast<uint8_t>(sizeof(s_string_table) / sizeof(s_string_table[0]));
    if (index >= table_size || !s_string_table[index]) return nullptr;

    const char  *str = s_string_table[index];
    const size_t len = strnlen(str, 31u);

    // Build UTF-16LE descriptor header (length + type)
    s_desc_str[0] = static_cast<uint16_t>((TUSB_DESC_STRING << 8u) |
                                           (2u + 2u * len));
    for (size_t i = 0; i < len; i++) {
        s_desc_str[1 + i] = static_cast<uint16_t>(str[i]);
    }
    return s_desc_str;
}
