#include "displayList.h"

#include <stddef.h>

namespace {

constexpr uint16_t DISPLAY_LIST_MAX_BYTES = 4096;

uint8_t g_displayListBuffer[DISPLAY_LIST_MAX_BYTES];
uint16_t g_displayListSize = 0;
uint16_t g_displayListDroppedOps = 0;
uint32_t g_displayListFrameId = 0;
bool g_displayListEnabled = true;

bool reserveBytes(const uint16_t byteCount) {
    if (g_displayListSize + byteCount > DISPLAY_LIST_MAX_BYTES) {
        g_displayListDroppedOps++;
        return false;
    }
    return true;
}

void writeU8(const uint8_t value) {
    g_displayListBuffer[g_displayListSize++] = value;
}

void writeU16(const uint16_t value) {
    g_displayListBuffer[g_displayListSize++] = (uint8_t)(value & 0xffu);
    g_displayListBuffer[g_displayListSize++] = (uint8_t)((value >> 8) & 0xffu);
}

void writeU32(const uint32_t value) {
    g_displayListBuffer[g_displayListSize++] = (uint8_t)(value & 0xffu);
    g_displayListBuffer[g_displayListSize++] = (uint8_t)((value >> 8) & 0xffu);
    g_displayListBuffer[g_displayListSize++] = (uint8_t)((value >> 16) & 0xffu);
    g_displayListBuffer[g_displayListSize++] = (uint8_t)((value >> 24) & 0xffu);
}

uint8_t boundedAsciiLength(const char string[], const uint8_t charLimit) {
    uint8_t len = 0;
    while (len < charLimit && string[len] != '\0') {
        len++;
    }
    return len;
}

// Build the colored opcode byte: high bit set, color in bits 6..3, sub in bits 2..0.
inline uint8_t coloredOp(const uint8_t sub, const uint8_t color) {
    return (uint8_t)(DL_OP_COLORED_FLAG | ((color & 0x0f) << 3) | (sub & 0x07));
}

} // namespace

void displayListSetEnabled(const bool enabled) {
    g_displayListEnabled = enabled;
}

bool displayListIsEnabled() {
    return g_displayListEnabled;
}

void displayListBeginFrame(const uint16_t width, const uint16_t height, const uint8_t redrawType) {
    if (!g_displayListEnabled) {
        return;
    }

    g_displayListSize = 0;
    g_displayListDroppedOps = 0;
    g_displayListFrameId++;

    if (!reserveBytes(1 + 4 + 2 + 2 + 1)) {
        return;
    }

    writeU8(DL_OP_BEGIN_FRAME);
    writeU32(g_displayListFrameId);
    writeU16(width);
    writeU16(height);
    writeU8(redrawType);
}

void displayListEndFrame() {
    if (!g_displayListEnabled) {
        return;
    }

    if (!reserveBytes(1 + 2)) {
        return;
    }

    writeU8(DL_OP_END_FRAME);
    writeU16(g_displayListDroppedOps);
}

void displayListRecordClear(const uint8_t color) {
    if (!g_displayListEnabled || !reserveBytes(1)) {
        return;
    }

    writeU8(coloredOp(DL_SUB_CLEAR, color));
}

void displayListRecordFillRows(const uint16_t y, const uint16_t rows, const uint8_t color) {
    if (!g_displayListEnabled || !reserveBytes(1 + 2 + 2)) {
        return;
    }

    writeU8(coloredOp(DL_SUB_FILL_ROWS, color));
    writeU16(y);
    writeU16(rows);
}

void displayListRecordLine(const uint16_t x0,
                           const uint16_t y0,
                           const uint16_t x1,
                           const uint16_t y1,
                           const uint8_t color) {
    if (!g_displayListEnabled) {
        return;
    }

    // Auto-promote axis-aligned lines to HLINE/VLINE for 3 fewer bytes.
    if (y0 == y1) {
        if (!reserveBytes(1 + 2 + 2 + 2)) return;
        writeU8(coloredOp(DL_SUB_HLINE, color));
        writeU16(y0);
        writeU16(x0);
        writeU16(x1);
        return;
    }
    if (x0 == x1) {
        if (!reserveBytes(1 + 2 + 2 + 2)) return;
        writeU8(coloredOp(DL_SUB_VLINE, color));
        writeU16(x0);
        writeU16(y0);
        writeU16(y1);
        return;
    }

    if (!reserveBytes(1 + 2 + 2 + 2 + 2)) {
        return;
    }

    writeU8(coloredOp(DL_SUB_LINE, color));
    writeU16(x0);
    writeU16(y0);
    writeU16(x1);
    writeU16(y1);
}

void displayListRecordString(const uint16_t x,
                             const uint16_t y,
                             const uint8_t color,
                             const uint8_t scale,
                             const char string[],
                             const uint8_t charLimit) {
    if (!g_displayListEnabled) {
        return;
    }

    const uint8_t len = boundedAsciiLength(string, charLimit);
    if (!reserveBytes((uint16_t)(1 + 2 + 2 + 1 + 1 + len))) {
        return;
    }

    writeU8(coloredOp(DL_SUB_STRING, color));
    writeU16(x);
    writeU16(y);
    writeU8(scale);
    writeU8(len);

    for (uint8_t i = 0; i < len; i++) {
        writeU8((uint8_t)string[i]);
    }
}

void displayListRecordImageRef(const uint16_t x,
                               const uint16_t y,
                               const uint16_t width,
                               const uint16_t height,
                               const uint32_t imageId) {
    if (!g_displayListEnabled || !reserveBytes(1 + 2 + 2 + 2 + 2 + 4)) {
        return;
    }

    writeU8(DL_OP_IMAGE_REF);
    writeU16(x);
    writeU16(y);
    writeU16(width);
    writeU16(height);
    writeU32(imageId);
}

const uint8_t* displayListGetData() {
    return g_displayListBuffer;
}

uint16_t displayListGetSize() {
    return g_displayListSize;
}

uint16_t displayListGetDroppedOps() {
    return g_displayListDroppedOps;
}

uint32_t displayListGetFrameId() {
    return g_displayListFrameId;
}
