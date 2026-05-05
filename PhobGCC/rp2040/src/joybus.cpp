#include "comms/joybus.hpp"

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "hardware/pio.h"
#include "joybus.pio.h"
#include "storage/pages/metadata.h"
#include "displayList.h"
#include <string.h>
#include <math.h>

#define ORG 127

// --- Display-list transfer layout (0xC0 read command) ---
// Each 1024-byte transfer: [0]=numChunks, [1]=chunkIndex, [2..1023]=1022 bytes of DL data.
// 1022-byte chunks let the input-viewer frame fit in a single SI transfer.
#define DISPLAY_LIST_CHUNK_DATA_SIZE     510
#define DISPLAY_LIST_CHUNK_TRANSFER_SIZE 512
#define DISPLAY_LIST_MAX_BYTES           4096
#define DISPLAY_LIST_MAX_CHUNKS ((DISPLAY_LIST_MAX_BYTES + DISPLAY_LIST_CHUNK_DATA_SIZE - 1) / DISPLAY_LIST_CHUNK_DATA_SIZE)

static uint8_t controllerMetadata[CONTROLLER_METADATA_MAX_SIZE];
static uint8_t metadataNumChunks = 0;
static bool metadataLoaded = false;

// Precomputed PIO responses for each metadata chunk
static uint32_t metadataPioChunks[METADATA_MAX_CHUNKS][METADATA_CHUNK_TRANSFER_SIZE / 2 + 1];
static int metadataPioChunkLens[METADATA_MAX_CHUNKS];
static uint32_t metadataAckPioResult[2];
static int metadataAckPioResultLen;

// Precomputed PIO responses for each display-list chunk.
// Double-buffered so core 1 can rebuild a frame's chunks without racing
// against core 0 (the SI handler) reading them. Core 1 writes the inactive
// buffer, then publishes by flipping `displayListActiveBuf` atomically.
// `displayListNumChunks` is published last so readers always see a
// consistent (numChunks, chunks[]) pair.
//
// Across multi-chunk frames the reader's jbPut can span more than one
// 60 Hz draw, by which point the previously-active buffer has become the
// next write target. To prevent that, the reader publishes which buffer
// it is currently using in `displayListReaderBuf`; the writer stalls if
// it would clobber that slot.
static uint32_t displayListPioChunks[2][DISPLAY_LIST_MAX_CHUNKS][DISPLAY_LIST_CHUNK_TRANSFER_SIZE / 2 + 1];
static int displayListPioChunkLens[2][DISPLAY_LIST_MAX_CHUNKS];
static volatile uint8_t displayListActiveBuf = 0;
static volatile int8_t displayListReaderBuf = -1;  // -1 = idle
static volatile uint8_t displayListNumChunks = 0;

extern volatile bool _pleaseCommitMetadata;
// When the host first issues a 0xC0 (DisplayListRead), promote the
// controller into "video over SI" mode so core 1 starts building a real
// display list. Defined in main.cpp.
extern volatile bool _videoOverSi;

/* PIOs are separate state machines for handling IOs with high timing precision. You load a program into them and they do their stuff on their own with deterministic timing,
   communicating with the main cores via FIFOs (and interrupts, if you want).

   Attempting to bit-bang the Joybus protocol by writing down timer snapshots upon GPIO changes and waiting on timers to trigger GPIO changes is bound to encounter some
   issues related to the uncertainty of timings (the STM32F0 is pipelined). Previous versions of this used to do that and would occasionally have the controller disconnect
   for a frame due to a polling fault, which would manifest particularly on the CSS by the hand teleporting back to its default position.
   Ever since the migration to PIOs - the proper tool for the job - this problem is gone.

   This project is adapted from the communication module of pico-rectangle (https://github.com/JulienBernard3383279/pico-rectangle), a digital controller firmware.

   The PIO program expects the system clock to be 125MHz, but you can adapt it by fiddling with the delays in the PIO program.

   One PIO here is configured to wait for new bytes over the joybus protocol. The main core spins a byte to come from the PIO FIFO, which will be the Joybus command
   from the console. It then matches that byte to know whether this is probe, origin or poll request.
   Once it has matched a command and has let the console finish talking, it will start replying, but in the case of the poll command, it will first build the state.
   The state is built by calling a callback passed as parameter to enterMode. This is fine for digital controllers because it takes few microseconds, but isn't
   if your state building is going to take any longer. (The console is fine with some delay between the poll command and response, but adapters don't tolerate more than
   few microseconds) In that case, you'll need to change the control flow for the passed callback to not do any computational work itself.

   Check the T O D O s in this file for things you should check when adapting it to your project.

   I advise checking the RP2040 documentation and this video https://www.youtube.com/watch?v=yYnQYF_Xa8g&ab_channel=stacksmashing to understand
*/

void __time_critical_func(convertToPio)(const uint8_t* command, const int len, uint32_t* result, int& resultLen) {
    // PIO Shifts to the right by default
    // In: pushes batches of 8 shifted left, i.e we get [0x40, 0x03, rumble (the end bit is never pushed)]
    // Out: We push commands for a right shift with an enable pin, ie 5 (101) would be 0b11'10'11
    // So in doesn't need post processing but out does
    if (len == 0) {
        resultLen = 0;
        return;
    }
    resultLen = len/2 + 1;
    int i;
    for (i = 0; i < resultLen; i++) {
        result[i] = 0;
    }
    for (i = 0; i < len; i++) {
        for (int j = 0; j < 8; j++) {
            result[i / 2] += 1 << (2 * (8 * (i % 2) + j) + 1);
            result[i / 2] += (!!(command[i] & (0x80u >> j))) << (2 * (8 * (i % 2) + j));
        }
    }
    // End bit
    result[len / 2] += 3 << (2 * (8 * (len % 2)));
}

static void precomputeMetadataResponses() {
    for (int c = 0; c < metadataNumChunks; c++) {
        uint8_t chunkTransfer[METADATA_CHUNK_TRANSFER_SIZE];
        chunkTransfer[0] = metadataNumChunks;
        chunkTransfer[1] = c;
        memcpy(&chunkTransfer[2], &controllerMetadata[c * METADATA_CHUNK_DATA_SIZE], METADATA_CHUNK_DATA_SIZE);
        convertToPio(chunkTransfer, METADATA_CHUNK_TRANSFER_SIZE, metadataPioChunks[c], metadataPioChunkLens[c]);
    }
}

static void loadControllerMetadata() {
    if (!metadataLoaded) {
        getControllerMetadata(controllerMetadata, metadataNumChunks);
        metadataLoaded = true;
        precomputeMetadataResponses();
        uint8_t ack[1] = { 0x01 };
        convertToPio(ack, 1, metadataAckPioResult, metadataAckPioResultLen);
    }
}

void __time_critical_func(convertGCReport)(GCReport* report, GCReport* dest_report, uint8_t mode) {
    memcpy(dest_report, report, 8);
    if (mode == 1) {
        dest_report->mode1.cxStick = report->cxStick >> 4;
        dest_report->mode1.cyStick = report->cyStick >> 4;
        dest_report->mode1.analogL = report->analogL;
        dest_report->mode1.analogR = report->analogR;
        dest_report->mode1.analogA = 0;
        dest_report->mode1.analogB = 0;
    }
    else if (mode == 2) {
        dest_report->mode2.cxStick = report->cxStick >> 4;
        dest_report->mode2.cyStick = report->cyStick >> 4;
        dest_report->mode2.analogL = report->analogL >> 4;
        dest_report->mode2.analogR = report->analogR >> 4;
        dest_report->mode2.analogA = 0;
        dest_report->mode2.analogB = 0;
    }
    else if (mode == 4) {
        dest_report->mode4.cxStick = report->cxStick;
        dest_report->mode4.cyStick = report->cyStick;
        dest_report->mode4.analogA = 0;
        dest_report->mode4.analogB = 0;
    }
    else if (mode == 3) {
        return;
    }
    else { // Mode 0, 5, 6, 7
        dest_report->mode0.cxStick = report->cxStick;
        dest_report->mode0.cyStick = report->cyStick;
        dest_report->mode0.analogL = report->analogL >> 4;
        dest_report->mode0.analogR = report->analogR >> 4;
        dest_report->mode0.analogA = 0;
        dest_report->mode0.analogB = 0;
    }
}

// ============================================================
// Shared joybus command handlers.
//
// Each handler assumes the command byte has already been consumed from the
// PIO RX FIFO, and the state machine is still in input (`inmode`). Handlers
// switch to output, send their response, and rely on `convertToPio`'s
// trailing zero bits to make the PIO auto-return to `inmode` (via the
// `jmp !Y inmode` at the end of the outmode program).
//
// The same handlers are used by two callers:
//   - enterMode()                  normal SI mode on PIO0
//   - serviceJoybusForVideoMode()  video-mode side channel on PIO1
// ============================================================

struct JoybusCtx {
    PIO pio;
    uint sm;
    uint offset;
    pio_sm_config config;
};

static inline uint8_t jbReadBlocking(JoybusCtx* ctx) {
    return (uint8_t)pio_sm_get_blocking(ctx->pio, ctx->sm);
}

static inline void jbSwitchToOut(JoybusCtx* ctx) {
    pio_sm_set_enabled(ctx->pio, ctx->sm, false);
    pio_sm_init(ctx->pio, ctx->sm, ctx->offset + joybus_offset_outmode, &ctx->config);
    pio_sm_set_enabled(ctx->pio, ctx->sm, true);
}

static inline void jbSwitchToIn(JoybusCtx* ctx) {
    pio_sm_set_enabled(ctx->pio, ctx->sm, false);
    pio_sm_init(ctx->pio, ctx->sm, ctx->offset + joybus_offset_inmode, &ctx->config);
    pio_sm_set_enabled(ctx->pio, ctx->sm, true);
}

static inline void jbPut(JoybusCtx* ctx, const uint32_t* words, int len) {
    for (int i = 0; i < len; i++) {
        pio_sm_put_blocking(ctx->pio, ctx->sm, words[i]);
    }
}

// --- Handlers ---------------------------------------------------------------

static void handleProbe(JoybusCtx* ctx) {
    uint8_t resp[3] = { 0x09, 0x00, 0x03 };
    uint32_t w[2]; int wl;
    convertToPio(resp, 3, w, wl);
    sleep_us(6); // wait out trailing stop bit before driving the line
    jbSwitchToOut(ctx);
    jbPut(ctx, w, wl);
}

static void handleOrigin(JoybusCtx* ctx) {
    uint8_t resp[10] = { 0x00, 0x80, ORG, ORG, ORG, ORG, 0, 0, 0, 0 };
    uint32_t w[6]; int wl;
    convertToPio(resp, 10, w, wl);
    // convertToPio took time; no explicit sleep needed.
    jbSwitchToOut(ctx);
    jbPut(ctx, w, wl);
}

// Poll: reads mode byte, builds report via callback, reads rumble byte,
// sends the 8-byte controller state. Returns the rumble byte so the caller
// can drive rumble hardware if it has any.
static uint8_t handlePoll(JoybusCtx* ctx, std::function<GCReport()>& func) {
    GCReport gcReport = func();
    GCReport dest;

    uint8_t mode = jbReadBlocking(ctx);
    convertGCReport(&gcReport, &dest, mode);

    uint32_t w[5]; int wl;
    convertToPio((uint8_t*)&dest, 8, w, wl);

    uint8_t rumbleByte = jbReadBlocking(ctx);

    sleep_us(7); // don't overwrite the stop bit of the poll command
    jbSwitchToOut(ctx);
    jbPut(ctx, w, wl);
    return rumbleByte;
}

static void handleMetadataRead(JoybusCtx* ctx) {
    uint8_t chunkIndex = jbReadBlocking(ctx);
    if (chunkIndex >= metadataNumChunks) chunkIndex = 0;
    sleep_us(7);
    jbSwitchToOut(ctx);
    jbPut(ctx, metadataPioChunks[chunkIndex], metadataPioChunkLens[chunkIndex]);
}

static void handleMetadataWrite(JoybusCtx* ctx) {
    uint8_t chunkIndex = jbReadBlocking(ctx);
    uint8_t chunkData[METADATA_CHUNK_DATA_SIZE];
    for (int i = 0; i < METADATA_CHUNK_DATA_SIZE; i++) {
        chunkData[i] = jbReadBlocking(ctx);
    }
    if (chunkIndex < METADATA_MAX_CHUNKS) {
        // Writing chunk 0 starts a new sequence — reset count so stale
        // chunks from a previous (larger) write are discarded.
        if (chunkIndex == 0) metadataNumChunks = 0;
        memcpy(&controllerMetadata[chunkIndex * METADATA_CHUNK_DATA_SIZE], chunkData, METADATA_CHUNK_DATA_SIZE);
        if (chunkIndex + 1 > metadataNumChunks) metadataNumChunks = chunkIndex + 1;
        precomputeMetadataResponses();
        setControllerMetadata(controllerMetadata, metadataNumChunks);
        _pleaseCommitMetadata = true;
    }
    sleep_us(7);
    jbSwitchToOut(ctx);
    jbPut(ctx, metadataAckPioResult, metadataAckPioResultLen);
}

static void handleDisplayListRead(JoybusCtx* ctx) {
    // Promote the controller into video-over-SI mode on the first 0xC0
    // we see. Core 1 watches this flag and starts building display lists.
    _videoOverSi = true;
    uint8_t chunkIndex = jbReadBlocking(ctx);
    // Snapshot active buffer + chunk count together so a mid-read swap
    // can't tear (numChunks/chunk slot from different generations).
    uint8_t buf = displayListActiveBuf;
    // Claim the buffer so the writer won't pick it as next write target.
    // The writer checks `displayListReaderBuf` and stalls if we name the
    // buffer it was about to clobber.
    displayListReaderBuf = (int8_t)buf;
    uint8_t n = displayListNumChunks;
    if (n == 0) {
        // No display list built yet. Drop back to input to avoid leaving PIO
        // stuck in output mode with an empty FIFO.
        sleep_us(7);
        jbSwitchToIn(ctx);
        displayListReaderBuf = -1;
        return;
    }
    if (chunkIndex >= n) chunkIndex = 0;
    sleep_us(7);
    jbSwitchToOut(ctx);
    jbPut(ctx, displayListPioChunks[buf][chunkIndex], displayListPioChunkLens[buf][chunkIndex]);
    displayListReaderBuf = -1;
}

static void handleUnknown(JoybusCtx* ctx) {
    // Unknown command — wait for it to finish then reset to input.
    pio_sm_set_enabled(ctx->pio, ctx->sm, false);
    sleep_us(400);
    pio_sm_init(ctx->pio, ctx->sm, ctx->offset + joybus_offset_inmode, &ctx->config);
    pio_sm_set_enabled(ctx->pio, ctx->sm, true);
}

// Dispatch a fully-received command byte. Optional rumble pins (<0 to skip).
static void dispatchCommand(JoybusCtx* ctx,
                            uint8_t cmd,
                            std::function<GCReport()>& func,
                            int rumblePin,
                            int brakePin,
                            int rumblePower) {
    switch (cmd) {
        case 0x00: handleProbe(ctx); break;
        case 0x41: handleOrigin(ctx); break;
        case 0x40: {
            uint8_t rumbleByte = handlePoll(ctx, func);
            if (rumblePin >= 0) {
                bool rumbleBrake = rumbleByte & 2;
                bool rumble = (rumbleByte & 1) && !rumbleBrake;
                if (rumble) {
                    pwm_set_gpio_level(brakePin, 0);
                    pwm_set_gpio_level(rumblePin, rumblePower);
                } else {
                    pwm_set_gpio_level(rumblePin, 0);
                    pwm_set_gpio_level(brakePin, rumbleBrake ? 255 : 0);
                }
            }
            break;
        }
        case 0xA0: handleMetadataRead(ctx); break;
        case 0xB0: handleMetadataWrite(ctx); break;
        case 0xC0: handleDisplayListRead(ctx); break;
        default:   handleUnknown(ctx); break;
    }
}

// ============================================================
// Public: display-list chunk precompute
// Called after each frame's draw completes. Builds PIO-ready transfer words
// from the current displayList buffer so the 0xC0 handler can respond with
// zero work in the hot path.
// ============================================================

void precomputeDisplayListChunks() {
    const uint8_t* data = displayListGetData();
    uint16_t size = displayListGetSize();

    uint8_t nChunks = (uint8_t)((size + DISPLAY_LIST_CHUNK_DATA_SIZE - 1) / DISPLAY_LIST_CHUNK_DATA_SIZE);
    if (nChunks == 0) nChunks = 1;
    if (nChunks > DISPLAY_LIST_MAX_CHUNKS) nChunks = DISPLAY_LIST_MAX_CHUNKS;

    // Write into the inactive buffer so core 0's in-flight reads from the
    // active buffer aren't disturbed.
    uint8_t writeBuf = displayListActiveBuf ^ 1;

    // Across multi-chunk frames, a reader that started against the now-
    // inactive buffer may still be in flight (its jbPut spans this 60 Hz
    // tick).  Stall until that read completes so we don't tear its data.
    while (displayListReaderBuf == (int8_t)writeBuf) {
        tight_loop_contents();
    }

    for (uint8_t c = 0; c < nChunks; c++) {
        // Static so we don't blow core 1's 4 KB stack with a 512-byte
        // local on top of a deep menu-draw call chain.
        static uint8_t buf[DISPLAY_LIST_CHUNK_TRANSFER_SIZE];
        buf[0] = nChunks;
        buf[1] = c;
        uint16_t off = (uint16_t)c * DISPLAY_LIST_CHUNK_DATA_SIZE;
        uint16_t remaining = (size > off) ? (uint16_t)(size - off) : 0;
        uint16_t copy = (remaining < DISPLAY_LIST_CHUNK_DATA_SIZE) ? remaining : DISPLAY_LIST_CHUNK_DATA_SIZE;
        if (copy > 0) {
            memcpy(&buf[2], &data[off], copy);
        }
        if (copy < DISPLAY_LIST_CHUNK_DATA_SIZE) {
            memset(&buf[2 + copy], 0, DISPLAY_LIST_CHUNK_DATA_SIZE - copy);
        }
        convertToPio(buf, DISPLAY_LIST_CHUNK_TRANSFER_SIZE,
                     displayListPioChunks[writeBuf][c], displayListPioChunkLens[writeBuf][c]);
    }

    // Publish the new buffer first, then the chunk count.  A concurrent
    // reader sees either (oldBuf, oldN) or (newBuf, newN); never a torn
    // pair.  The chunkIndex clamp in handleDisplayListRead handles the
    // (newBuf, oldN) transient when newN < oldN.
    displayListActiveBuf = writeBuf;
    displayListNumChunks = nChunks;
}

// ============================================================
// Normal SI mode (PIO0, console/adapter as master)
// ============================================================

void __time_critical_func(enterMode)(const int dataPin,
                                     const int rumblePin,
                                     const int brakePin,
                                     int &rumblePower,
                                     std::function<GCReport()> func) {
    gpio_init(dataPin);
    gpio_set_dir(dataPin, GPIO_IN);
    gpio_pull_up(dataPin);

    sleep_us(100); // Stabilize voltages

    JoybusCtx ctx;
    ctx.pio = pio0;
    ctx.sm = 0;

    pio_gpio_init(ctx.pio, dataPin);
    ctx.offset = pio_add_program(ctx.pio, &joybus_program);

    ctx.config = joybus_program_get_default_config(ctx.offset);
    sm_config_set_in_pins(&ctx.config, dataPin);
    sm_config_set_out_pins(&ctx.config, dataPin, 1);
    sm_config_set_set_pins(&ctx.config, dataPin, 1);
    sm_config_set_clkdiv(&ctx.config, 5); // 125 MHz sys / 5 = 25 MHz PIO
    sm_config_set_out_shift(&ctx.config, true, false, 32);
    sm_config_set_in_shift(&ctx.config, false, true, 8);

    pio_sm_init(ctx.pio, ctx.sm, ctx.offset, &ctx.config);
    pio_sm_set_enabled(ctx.pio, ctx.sm, true);

    loadControllerMetadata();

    while (true) {
        uint8_t cmd = pio_sm_get_blocking(ctx.pio, ctx.sm);
        dispatchCommand(&ctx, cmd, func, rumblePin, brakePin, rumblePower);
    }
}

// ============================================================
// Video mode side channel (PIO1)
// PIO0 is occupied by composite video. System clock is 250 MHz in video
// mode, so clkdiv = 10 gives the same 25 MHz PIO clock the protocol
// timings assume.
// ============================================================

static JoybusCtx _videoCtx;
static bool _videoInited = false;
// Placeholder callback used when the video-mode caller doesn't provide one.
// Returns a neutral controller state so polls get a valid response.
static GCReport videoDefaultReport() {
    GCReport r{};
    r.pad1 = 1;
    r.xStick = ORG;
    r.yStick = ORG;
    r.cxStick = ORG;
    r.cyStick = ORG;
    return r;
}

void initJoybusForVideoMode(int dataPin) {
    if (_videoInited) return;

    _videoCtx.pio = pio1;
    _videoCtx.sm = 0;

    pio_gpio_init(_videoCtx.pio, dataPin);
    gpio_set_dir(dataPin, GPIO_IN);
    gpio_pull_up(dataPin);
    sleep_us(100);

    _videoCtx.offset = pio_add_program(_videoCtx.pio, &joybus_program);
    _videoCtx.config = joybus_program_get_default_config(_videoCtx.offset);
    sm_config_set_in_pins(&_videoCtx.config, dataPin);
    sm_config_set_out_pins(&_videoCtx.config, dataPin, 1);
    sm_config_set_set_pins(&_videoCtx.config, dataPin, 1);
    sm_config_set_clkdiv(&_videoCtx.config, 10); // 250 MHz sys / 10 = 25 MHz PIO
    sm_config_set_out_shift(&_videoCtx.config, true, false, 32);
    sm_config_set_in_shift(&_videoCtx.config, false, true, 8);

    pio_sm_init(_videoCtx.pio, _videoCtx.sm, _videoCtx.offset, &_videoCtx.config);
    pio_sm_set_enabled(_videoCtx.pio, _videoCtx.sm, true);

    loadControllerMetadata();

    // Flush any bytes that arrived during the (slow) init sequence so the
    // first service call isn't processing garbage.
    pio_sm_clear_fifos(_videoCtx.pio, _videoCtx.sm);

    _videoInited = true;
}

// Non-blocking: service one command if the RX FIFO has data. Intended to
// be called in a tight loop from core 1 while core 0 renders video.
// `reportFn` may be null to use a neutral default response.
void serviceJoybusForVideoMode(std::function<GCReport()> reportFn) {
    if (!_videoInited) return;
    if (pio_sm_is_rx_fifo_empty(_videoCtx.pio, _videoCtx.sm)) return;

    uint8_t cmd = (uint8_t)pio_sm_get(_videoCtx.pio, _videoCtx.sm);

    if (!reportFn) {
        std::function<GCReport()> dflt = videoDefaultReport;
        dispatchCommand(&_videoCtx, cmd, dflt, -1, -1, 0);
    } else {
        dispatchCommand(&_videoCtx, cmd, reportFn, -1, -1, 0);
    }
}
