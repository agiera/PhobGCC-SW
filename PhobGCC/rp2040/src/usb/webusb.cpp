// WebUSB calibration interface for PhobGCC on RP2040.
//
// Mirrors the vendor/WebUSB transport used by the GC-Adapter-RP2040 firmware so
// the existing browser calibration host works unchanged. The host filters on
// VID 0x057E / PID 0x0337, interface class 0xFF, claims interface 1, and drives
// two vendor control transfers:
//   OUT  request 10: [0x02, port, cmd_len(LE16), resp_len(LE16), ...cmd bytes]
//   IN   request 11: [0x02, port, resp_len(LE16), ...response bytes]
// Joybus commands are processed in-firmware (this device *is* the controller)
// via handleWebusbJoybusCommand() rather than forwarded onto a physical bus.

#include "usb/webusb.h"

#include "tusb.h"
#include "comms/joybus.hpp"

#include "hardware/resets.h"
#include "pico/time.h"

#include <string.h>

// Suggested landing page shown by Chrome when the device is connected. Purely a
// hint; change it to wherever the calibration web app is hosted.
#define WEBUSB_LANDING_URL "gcc.dizzyforpresident.org"

//--------------------------------------------------------------------+
// Vendor request codes (must match the browser host)
//--------------------------------------------------------------------+
enum {
    VENDOR_REQUEST_WEBUSB      = 1,
    VENDOR_REQUEST_MICROSOFT   = 2,
    VENDOR_REQUEST_WEBUSB_CMD  = 10,
    VENDOR_REQUEST_WEBUSB_RESP = 11,
};

enum {
    ITF_NUM_DUMMY  = 0, // placeholder (interface numbers must start at 0)
    ITF_NUM_VENDOR = 1, // WebUSB command interface claimed by the host
    ITF_NUM_TOTAL  = 2,
};

//--------------------------------------------------------------------+
// WebUSB command protocol
//--------------------------------------------------------------------+
#define WEBUSB_CMD_JOYBUS_CMD   0x02
#define WEBUSB_RESP_HEADER_LEN  4
#define WEBUSB_RAW_PAYLOAD_MAX  1024
#define WEBUSB_CTRL_BUF_SIZE    (6 + WEBUSB_RAW_PAYLOAD_MAX)

static uint8_t _webusb_out_buffer[WEBUSB_RESP_HEADER_LEN + WEBUSB_RAW_PAYLOAD_MAX];
static volatile uint16_t _webusb_response_len = 0; // 0 = no response ready

static void webusb_command_processor(const uint8_t* data) {
    switch (data[0]) {
        case WEBUSB_CMD_JOYBUS_CMD: {
            uint8_t  port     = data[1];
            uint16_t cmd_len  = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
            uint16_t resp_len = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
            if (cmd_len >= 1 && cmd_len <= WEBUSB_RAW_PAYLOAD_MAX && resp_len <= WEBUSB_RAW_PAYLOAD_MAX) {
                static uint8_t rawResp[WEBUSB_RAW_PAYLOAD_MAX];
                int rlen = handleWebusbJoybusCommand(&data[6], cmd_len, rawResp, sizeof(rawResp));
                if (rlen <= 0) {
                    // Unhandled command or no data ready yet: leave the response
                    // empty so the host keeps polling request 11.
                    _webusb_response_len = 0;
                    break;
                }
                if ((uint16_t)rlen > resp_len) rlen = resp_len;
                _webusb_out_buffer[0] = WEBUSB_CMD_JOYBUS_CMD;
                _webusb_out_buffer[1] = port;
                _webusb_out_buffer[2] = (uint8_t)(rlen & 0xff);
                _webusb_out_buffer[3] = (uint8_t)((rlen >> 8) & 0xff);
                memcpy(&_webusb_out_buffer[WEBUSB_RESP_HEADER_LEN], rawResp, (size_t)rlen);
                _webusb_response_len = (uint16_t)(WEBUSB_RESP_HEADER_LEN + rlen);
            }
            break;
        }
        default:
            break;
    }
}

//--------------------------------------------------------------------+
// USB descriptors
//--------------------------------------------------------------------+
static const tusb_desc_device_t webusb_device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0210, // 2.1: required for BOS / WebUSB
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x057E,
    .idProduct          = 0x0337,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

// One configuration, two vendor-specific interfaces with no data endpoints.
// The host uses control transfers only, so no endpoints are declared.
#define WEBUSB_CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 9 + 9)
static const uint8_t webusb_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, WEBUSB_CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_SELF_POWERED, 500),

    // Interface 0 — placeholder vendor interface (0 endpoints)
    9, TUSB_DESC_INTERFACE, ITF_NUM_DUMMY, 0x00, 0x00, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, 0x00,

    // Interface 1 — WebUSB command interface (0 endpoints, control transfers only)
    9, TUSB_DESC_INTERFACE, ITF_NUM_VENDOR, 0x00, 0x00, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, 0x00,
};

//--------------------------------------------------------------------+
// BOS descriptor (required for WebUSB + Windows WinUSB binding)
//--------------------------------------------------------------------+
#define BOS_TOTAL_LEN     (TUD_BOS_DESC_LEN + TUD_BOS_WEBUSB_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)
#define MS_OS_20_DESC_LEN 0xB2

static const uint8_t webusb_bos_descriptor[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 2),
    // WebUSB: vendor request code, landing page index
    TUD_BOS_WEBUSB_DESCRIPTOR(VENDOR_REQUEST_WEBUSB, 1),
    // Microsoft OS 2.0
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT),
};

static const uint8_t desc_ms_os_20[] = {
    // Set header: length, type, windows version, total length
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

    // Configuration subset header: length, type, configuration index, reserved, configuration total length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),

    // Function Subset header: length, type, first interface, reserved, subset length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), ITF_NUM_VENDOR, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),

    // MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sub-compatible

    // MS OS 2.0 Registry property descriptor: length, type
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08 - 0x08 - 0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A), // wPropertyDataType, wPropertyNameLength; "DeviceInterfaceGUIDs\0"
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00,
    'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00,
    U16_TO_U8S_LE(0x0050), // wPropertyDataLength
    // bPropertyData: "{8B3E9D2E-7EEC-4994-AAE7-0C40DE84D36E}"
    '{', 0x00, '8', 0x00, 'B', 0x00, '3', 0x00, 'E', 0x00, '9', 0x00, 'D', 0x00, '2', 0x00, 'E', 0x00, '-', 0x00,
    '7', 0x00, 'E', 0x00, 'E', 0x00, 'C', 0x00, '-', 0x00, '4', 0x00, '9', 0x00, '9', 0x00, '4', 0x00, '-', 0x00,
    'A', 0x00, 'A', 0x00, 'E', 0x00, '7', 0x00, '-', 0x00, '0', 0x00, 'C', 0x00, '4', 0x00, '0', 0x00, 'E', 0x00,
    'E', 0x00, '8', 0x00, '4', 0x00, 'D', 0x00, '3', 0x00, '6', 0x00, 'D', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00
};

TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "Incorrect MS OS 2.0 descriptor size");

// WebUSB landing-page URL descriptor (returned for VENDOR_REQUEST_WEBUSB).
// tusb_desc_webusb_url_t uses a flexible array member which cannot be
// initialised in C++, so use a fixed-size packed equivalent. The url array
// includes room for the string's null terminator, but bLength excludes it so
// only the URL bytes are transferred.
struct webusb_url_desc_t {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bScheme;
    char    url[sizeof(WEBUSB_LANDING_URL)];
} __attribute__((packed));

static const webusb_url_desc_t webusb_landing_url = {
    (uint8_t)(3 + sizeof(WEBUSB_LANDING_URL) - 1),
    3, // WEBUSB URL type
    1, // https
    WEBUSB_LANDING_URL,
};

//--------------------------------------------------------------------+
// String descriptors
//--------------------------------------------------------------------+
static const char* const webusb_string_descriptors[] = {
    (const char[]){ 0x09, 0x04 }, // 0: English (0x0409)
    "PhobGCC",                    // 1: Manufacturer
    "PhobGCC",                    // 2: Product
    "PHOB0001",                   // 3: Serial
};

//--------------------------------------------------------------------+
// TinyUSB callbacks (C linkage)
//--------------------------------------------------------------------+
extern "C" {

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&webusb_device_descriptor;
}

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return webusb_configuration_descriptor;
}

uint8_t const* tud_descriptor_bos_cb(void) {
    return webusb_bos_descriptor;
}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    static uint16_t desc_str[32];
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&desc_str[1], webusb_string_descriptors[0], 2);
        chr_count = 1;
    } else {
        if (index >= (sizeof(webusb_string_descriptors) / sizeof(webusb_string_descriptors[0]))) {
            return NULL;
        }
        const char* str = webusb_string_descriptors[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++) {
            desc_str[1 + i] = str[i];
        }
    }

    // first byte length (including header), second byte string type
    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return desc_str;
}

// Vendor control transfers implement the entire WebUSB command channel.
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
    static uint8_t webusb_ctrl_buf[WEBUSB_CTRL_BUF_SIZE];

    if (stage == CONTROL_STAGE_DATA) {
        if (request->bRequest == VENDOR_REQUEST_WEBUSB_CMD) {
            // Full command payload has arrived in webusb_ctrl_buf.
            webusb_command_processor(webusb_ctrl_buf);
        }
        return true;
    }

    // Nothing to do for ACK; only SETUP needs handling below.
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }

    switch (request->bmRequestType_bit.type) {
        case TUSB_REQ_TYPE_VENDOR:
            switch (request->bRequest) {
                case VENDOR_REQUEST_WEBUSB:
                    // WebUSB landing page
                    return tud_control_xfer(rhport, request,
                                            (void*)(uintptr_t)&webusb_landing_url,
                                            webusb_landing_url.bLength);

                case VENDOR_REQUEST_MICROSOFT:
                    if (request->wIndex == 7) {
                        uint16_t total_len;
                        memcpy(&total_len, desc_ms_os_20 + 8, 2);
                        return tud_control_xfer(rhport, request,
                                                (void*)(uintptr_t)desc_ms_os_20, total_len);
                    }
                    return false;

                case VENDOR_REQUEST_WEBUSB_CMD: {
                    // Receive the command payload into webusb_ctrl_buf; the DATA
                    // stage above then processes it.
                    uint16_t len = tu_min16(request->wLength, sizeof(webusb_ctrl_buf));
                    return tud_control_xfer(rhport, request, webusb_ctrl_buf, len);
                }

                case VENDOR_REQUEST_WEBUSB_RESP: {
                    uint16_t rlen = _webusb_response_len;
                    if (rlen == 0) {
                        // No response ready — return zero-length so the host retries.
                        return tud_control_xfer(rhport, request, NULL, 0);
                    }
                    _webusb_response_len = 0; // consumed
                    uint16_t len = tu_min16(request->wLength, rlen);
                    return tud_control_xfer(rhport, request, _webusb_out_buffer, len);
                }

                default:
                    break;
            }
            break;

        case TUSB_REQ_TYPE_CLASS:
            return tud_control_status(rhport, request);

        default:
            break;
    }

    return false; // stall unknown request
}

} // extern "C"

//--------------------------------------------------------------------+
// VBUS detection & service loop
//--------------------------------------------------------------------+
bool webusbVbusPresent(void) {
    // The RP2040 has no dedicated VBUS pin, and this board does not route USB
    // VBUS to a spare GPIO, so the 5V rail cannot be sensed directly (forcing
    // the SIE VBUS_DETECT override, as this used to, simply reports "present"
    // unconditionally). Instead, bring the USB device up and see whether a real
    // USB host answers: a PC issues a bus reset and begins enumeration (SETUP
    // packets) within a few hundred milliseconds, whereas a GameCube/Wii — which
    // talks only over the Joybus data line, not USB — never does. If no host
    // responds we tear the USB controller back down so it cannot perturb the
    // timing-critical Joybus path.
    tusb_init();

    const uint32_t timeoutUs = 500'000; // 500 ms is ample for a host bus reset
    const uint32_t start = time_us_32();
    while (time_us_32() - start < timeoutUs) {
        tud_task();
        if (tud_connected()) {
            return true; // a USB host is enumerating us -> stay in WebUSB mode
        }
    }

    // No USB host answered; make sure USB can't interfere with Joybus.
    tud_disconnect();
    reset_block(RESETS_RESET_USBCTRL_BITS);
    return false;
}

void webusbRun(void) {
    // webusbVbusPresent() already initialised TinyUSB when it detected a host;
    // guard against a redundant re-init in case the call order ever changes.
    if (!tusb_inited()) {
        tusb_init();
    }
    while (true) {
        tud_task();
    }
}
