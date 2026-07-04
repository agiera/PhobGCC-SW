#ifndef PHOB_WEBUSB_H
#define PHOB_WEBUSB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @short Returns true if USB VBUS is present (the controller is plugged into a
 *        host such as a PC), false when powered by a GameCube. Safe to call
 *        before tusb_init(); it powers up only the USB VBUS sense logic and,
 *        when no VBUS is found, leaves the USB controller held in reset so it
 *        cannot interfere with the timing-critical Joybus path.
 */
bool webusbVbusPresent(void);

/**
 * @short Initialise TinyUSB and service the WebUSB calibration interface
 *        forever. Runs on core0 in place of enterMode() when the controller is
 *        connected to a PC. Never returns.
 */
void webusbRun(void);

#ifdef __cplusplus
}
#endif

#endif // PHOB_WEBUSB_H
