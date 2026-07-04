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
 *        busy driving composite video.
 *
 * @param dataPin   GPIO number of the console data line pin
 */
void initJoybusForVideoMode(int dataPin);

/**
 * @short Service at most one pending joybus command on the video-mode
 *        side-channel. Non-blocking; returns immediately if no byte is waiting.
 *
 * @param reportFn  Optional callback producing controller state for 0x40 polls.
 */
void serviceJoybusForVideoMode(std::function<GCReport()> reportFn);

/**
 * @short Rebuild the precomputed PIO responses for 0xC0 display-list reads.
 *        Call after each frame draw has populated the display-list buffer.
 */
void precomputeDisplayListChunks();

/**
 * @short Handle a Joybus command that arrived over WebUSB rather than the
 *        physical Joybus line. Supports the calibration host opcodes 0xA0
 *        (read metadata chunk), 0xB0 (write metadata chunk) and 0xC0 (read
 *        display-list chunk).
 *
 * @param cmd      Raw command bytes (starting with the opcode)
 * @param cmdLen   Number of command bytes
 * @param resp     Output buffer for the raw response bytes
 * @param maxResp  Capacity of the output buffer
 * @return         Number of response bytes written, or 0 if unhandled / not ready
 */
int handleWebusbJoybusCommand(const uint8_t* cmd, int cmdLen, uint8_t* resp, int maxResp);

#endif
