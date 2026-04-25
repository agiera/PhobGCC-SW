#ifndef __JOYBUS_HPP
#define __JOYBUS_HPP

#include "pico/stdlib.h"
#include "gcReport.hpp"

#include <functional>

/**
 * @short Enters the Joybus communication mode (PIO0, blocking, console-timed).
 *
 * @param dataPin   GPIO number of the console data line pin
 * @param func      Callback that returns the GCReport to send on 0x40 poll
 */
void enterMode(const int dataPin, const int rumblePin, const int brakePin, int &rumblePower, std::function<GCReport()> func);

/**
 * @short Initialise a joybus side-channel on PIO1 for use while PIO0 is
 *        busy driving composite video. Expects system clock at 250 MHz.
 *
 * Shares metadata/display-list state with enterMode; handles the same
 * command set (0x00, 0x40, 0x41, 0xA0, 0xB0, 0xC0).
 *
 * @param dataPin   GPIO number of the console data line pin
 */
void initJoybusForVideoMode(int dataPin);

/**
 * @short Service at most one pending joybus command on the video-mode
 *        side-channel. Non-blocking; returns immediately if no byte is waiting.
 *        Intended to be called in a tight loop (e.g. on core 1).
 *
 * @param reportFn  Optional callback producing the controller state for
 *                  0x40 polls. Pass nullptr for a neutral centered response.
 */
void serviceJoybusForVideoMode(std::function<GCReport()> reportFn);

/**
 * @short Rebuild the precomputed PIO responses for 0xC0 display-list reads.
 *        Call after each frame's draw has populated the display-list buffer.
 *        Safe to call from core 0 while core 1 is servicing joybus.
 */
void precomputeDisplayListChunks();

#endif
