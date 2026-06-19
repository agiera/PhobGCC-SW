#ifndef DISPLAY_LIST_H
#define DISPLAY_LIST_H

#include <stdint.h>

// Compact binary display-list stream emitted by firmware draw calls.
// All integer fields are little-endian.
//
// Opcode byte encoding:
// - If bit 7 is set, this is a "colored" op:
//     bits 6..3 = 4-bit palette color
//     bits 2..0 = colored sub-opcode:
//       0 CLEAR_C
//       1 FILL_ROWS_C: y(u16), rows(u16)
//       2 LINE_C:      x0(u16), y0(u16), x1(u16), y1(u16)
//       3 STRING_C:    x(u16), y(u16), scale(u8), len(u8), ascii[len]
//       4 HLINE_C:     y(u16), x0(u16), x1(u16)
//       5 VLINE_C:     x(u16), y0(u16), y1(u16)
// - Else uncolored:
//     0x01 BEGIN_FRAME: frameId(u32), width(u16), height(u16), redrawType(u8)
//     0x12 IMAGE_REF:   x(u16), y(u16), width(u16), height(u16), imageId(u32)
//     0x7f END_FRAME:   droppedOps(u16)

enum DisplayListOpcode : uint8_t {
    DL_OP_BEGIN_FRAME = 0x01,
    DL_OP_IMAGE_REF = 0x12,
    DL_OP_END_FRAME = 0x7f,
    // High bit set => colored op, color in bits 6..3, sub in bits 2..0.
    DL_OP_COLORED_FLAG = 0x80,
    DL_SUB_CLEAR = 0,
    DL_SUB_FILL_ROWS = 1,
    DL_SUB_LINE = 2,
    DL_SUB_STRING = 3,
    DL_SUB_HLINE = 4,
    DL_SUB_VLINE = 5,
};

void displayListSetEnabled(bool enabled);
bool displayListIsEnabled();

void displayListBeginFrame(uint16_t width, uint16_t height, uint8_t redrawType);
void displayListEndFrame();

void displayListRecordClear(uint8_t color);
void displayListRecordFillRows(uint16_t y, uint16_t rows, uint8_t color);
void displayListRecordLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t color);
void displayListRecordString(uint16_t x, uint16_t y, uint8_t color, uint8_t scale, const char string[], uint8_t charLimit);
void displayListRecordImageRef(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint32_t imageId);

const uint8_t* displayListGetData();
uint16_t displayListGetSize();
uint16_t displayListGetDroppedOps();
uint32_t displayListGetFrameId();

#endif // DISPLAY_LIST_H
