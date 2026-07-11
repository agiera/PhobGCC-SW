#ifndef PHOB_WEBUSB_H
#define PHOB_WEBUSB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @short Returns true when the controller is plugged into a real USB host (such
 *        as a PC), false when powered by a GameCube/Wii. The RP2040 cannot sense
 *        USB VBUS on this board, so this brings the USB device up briefly and
 *        checks whether a host actually enumerates it; if none does within a
 *        short timeout the USB controller is held in reset so it cannot interfere
 *        with the timing-critical Joybus path. On a true result TinyUSB is left
 *        initialised, ready for webusbRun().
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
