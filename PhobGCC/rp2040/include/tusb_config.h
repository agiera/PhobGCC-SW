#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

// CFG_TUSB_MCU and CFG_TUSB_OS are supplied by the pico-sdk tinyusb build
// (OPT_MCU_RP2040 / OPT_OS_PICO). Fall back to sensible defaults just in case.
#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU              OPT_MCU_RP2040
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS               OPT_OS_PICO
#endif

// Device mode, full speed
#ifndef CFG_TUSB_RHPORT0_MODE
#define CFG_TUSB_RHPORT0_MODE     (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#endif

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENABLED
#define CFG_TUD_ENABLED           1
#endif

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE    64
#endif

//------------- CLASS -------------//
#define CFG_TUD_HID               0
#define CFG_TUD_CDC               0
#define CFG_TUD_MSC               0
#define CFG_TUD_MIDI              0
// Two vendor interfaces: interface 0 is a placeholder (USB interface numbers
// must be contiguous from 0) and interface 1 is the WebUSB command interface
// that the calibration host claims. Both use control transfers only (no bulk
// endpoints), so the RX/TX buffer sizes below are unused but must be defined.
#define CFG_TUD_VENDOR            2

#define CFG_TUD_VENDOR_EPSIZE     64
#define CFG_TUD_VENDOR_RX_BUFSIZE 64
#define CFG_TUD_VENDOR_TX_BUFSIZE 64

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
