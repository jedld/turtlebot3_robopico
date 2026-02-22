/*
 * usb_descriptors.c — USB descriptors for a single CDC ACM interface.
 *
 * Presents as a USB Virtual COM Port (CDC ACM) to the host.
 * The host will create /dev/ttyACM0 (Linux) for Dynamixel traffic.
 *
 * Intentionally does NOT implement the "reset on DTR" trick used by
 * Arduino bootloaders — that reset was the root cause of the 34 ms
 * connection-timeout failure when using CircuitPython.
 */

#include "tusb.h"

// -------------------------------------------------------------------
// Device descriptor
// -------------------------------------------------------------------
static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,           // USB 2.0

    // Use "Interface Association" class so Windows/Linux enumerate correctly
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0x2E8A,           // Raspberry Pi
    .idProduct          = 0x000A,           // Custom CDC
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01,
};

const uint8_t *tud_descriptor_device_cb(void) {
    return (const uint8_t *)&desc_device;
}

// -------------------------------------------------------------------
// Configuration descriptor
// -------------------------------------------------------------------
// CDC needs two interfaces: one control (IAD) + one data.
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

// Endpoint numbers
#define EPNUM_CDC_NOTIFY  0x81   // EP1 IN  — CDC control (interrupt)
#define EPNUM_CDC_OUT     0x02   // EP2 OUT — CDC data RX
#define EPNUM_CDC_IN      0x82   // EP2 IN  — CDC data TX

static const uint8_t desc_configuration[] = {
    // Config descriptor header
    TUD_CONFIG_DESCRIPTOR(
        1,                          // bConfigurationValue
        2,                          // bNumInterfaces (control + data)
        0,                          // iConfiguration
        CONFIG_TOTAL_LEN,
        0x00,                       // bmAttributes: bus-powered, no remote wakeup
        250                         // bMaxPower: 500 mA
    ),

    // CDC descriptor (IAD + Control IF + Data IF)
    TUD_CDC_DESCRIPTOR(
        0,                          // Interface number (control = 0, data = 1)
        4,                          // iInterface string index
        EPNUM_CDC_NOTIFY,           // Notification endpoint (IN)
        8,                          // Notification endpoint max packet size
        EPNUM_CDC_OUT,              // Data OUT endpoint
        EPNUM_CDC_IN,               // Data IN endpoint
        64                          // Data endpoints max packet size
    ),
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

// -------------------------------------------------------------------
// String descriptors
// -------------------------------------------------------------------
// Index 0: Language (English)
static const uint16_t desc_str_lang[] = {
    (TUSB_DESC_STRING << 8) | (4), 0x0409
};

static const char *string_desc_arr[] = {
    NULL,                           // 0: Language (handled above)
    "Raspberry Pi",                 // 1: Manufacturer
    "TurtleBot3 Pico OpenCR",       // 2: Product
    "TB3PICO0001",                  // 3: Serial number
    "TurtleBot3 Dynamixel Port",    // 4: CDC Interface
};

// Buffer for dynamically constructed string descriptors
static uint16_t _desc_str[32];

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    if (index == 0) {
        memcpy(_desc_str, desc_str_lang, sizeof(desc_str_lang));
        return _desc_str;
    }

    if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
        return NULL;
    }

    const char *str = string_desc_arr[index];
    if (!str) return NULL;

    size_t len = strlen(str);
    if (len > 31) len = 31;

    // Header: length (2 + 2*len) and type (TUSB_DESC_STRING)
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 + 2 * len));
    for (size_t i = 0; i < len; i++) {
        _desc_str[1 + i] = str[i];   // ASCII → UTF-16LE
    }

    return _desc_str;
}
