#include "comms/joybus.hpp"

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "joybus.pio.h"
#include "storage/pages/metadata.h"
#include "displayList.h"
#include <string.h>
#include <math.h>

#define ORG 127

// 0xC0 display-list transfer layout:
// [0]=numChunks, [1]=chunkIndex, [2..511]=510 bytes payload
#define DISPLAY_LIST_CHUNK_DATA_SIZE 510
#define DISPLAY_LIST_CHUNK_TRANSFER_SIZE 512
#define DISPLAY_LIST_MAX_BYTES 4096
#define DISPLAY_LIST_MAX_CHUNKS ((DISPLAY_LIST_MAX_BYTES + DISPLAY_LIST_CHUNK_DATA_SIZE - 1) / DISPLAY_LIST_CHUNK_DATA_SIZE)

static uint8_t controllerMetadata[CONTROLLER_METADATA_MAX_SIZE];
static uint8_t metadataNumChunks = 0;
static bool metadataLoaded = false;

// Precomputed PIO responses for each metadata chunk
static uint32_t metadataPioChunks[METADATA_MAX_CHUNKS][METADATA_CHUNK_TRANSFER_SIZE / 2 + 1];
static int metadataPioChunkLens[METADATA_MAX_CHUNKS];
static uint32_t metadataAckPioResult[2];
static int metadataAckPioResultLen;

// Double-buffered precomputed PIO responses for each display-list chunk.
static uint32_t displayListPioChunks[2][DISPLAY_LIST_MAX_CHUNKS][DISPLAY_LIST_CHUNK_TRANSFER_SIZE / 2 + 1];
static int displayListPioChunkLens[2][DISPLAY_LIST_MAX_CHUNKS];
static volatile uint8_t displayListActiveBuf = 0;
static volatile int8_t displayListReaderBuf = -1;
static volatile uint8_t displayListNumChunks = 0;

extern volatile bool _pleaseCommitMetadata;
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

void precomputeDisplayListChunks() {
    const uint8_t* data = displayListGetData();
    uint16_t size = displayListGetSize();

    uint8_t nChunks = (uint8_t)((size + DISPLAY_LIST_CHUNK_DATA_SIZE - 1) / DISPLAY_LIST_CHUNK_DATA_SIZE);
    if (nChunks == 0) nChunks = 1;
    if (nChunks > DISPLAY_LIST_MAX_CHUNKS) nChunks = DISPLAY_LIST_MAX_CHUNKS;

    uint8_t writeBuf = displayListActiveBuf ^ 1;

    while (displayListReaderBuf == (int8_t)writeBuf) {
        tight_loop_contents();
    }

    for (uint8_t c = 0; c < nChunks; c++) {
        static uint8_t transfer[DISPLAY_LIST_CHUNK_TRANSFER_SIZE];
        transfer[0] = nChunks;
        transfer[1] = c;

        uint16_t off = (uint16_t)c * DISPLAY_LIST_CHUNK_DATA_SIZE;
        uint16_t remaining = (size > off) ? (uint16_t)(size - off) : 0;
        uint16_t copy = (remaining < DISPLAY_LIST_CHUNK_DATA_SIZE) ? remaining : DISPLAY_LIST_CHUNK_DATA_SIZE;

        if (copy > 0) {
            memcpy(&transfer[2], &data[off], copy);
        }
        if (copy < DISPLAY_LIST_CHUNK_DATA_SIZE) {
            memset(&transfer[2 + copy], 0, DISPLAY_LIST_CHUNK_DATA_SIZE - copy);
        }

        convertToPio(transfer,
                     DISPLAY_LIST_CHUNK_TRANSFER_SIZE,
                     displayListPioChunks[writeBuf][c],
                     displayListPioChunkLens[writeBuf][c]);
    }

    displayListActiveBuf = writeBuf;
    displayListNumChunks = nChunks;
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

void __time_critical_func(enterMode)(const int dataPin,
                                     const int rumblePin,
                                     const int brakePin,
                                     int &rumblePower,
                                     std::function<GCReport()> func) {
    gpio_init(dataPin);
    gpio_set_dir(dataPin, GPIO_IN);
    gpio_pull_up(dataPin);

    sleep_us(100); // Stabilize voltages

    PIO pio = pio0;
    pio_gpio_init(pio, dataPin);
    uint offset = pio_add_program(pio, &joybus_program);

    pio_sm_config config = joybus_program_get_default_config(offset);
    sm_config_set_in_pins(&config, dataPin);
    sm_config_set_out_pins(&config, dataPin, 1);
    sm_config_set_set_pins(&config, dataPin, 1);
    // The PIO program is calibrated for a 25 MHz PIO clock. Derive the
    // divider from the current system clock so timing is correct whether
    // sys_clk is 125 MHz (clkdiv 5) or 250 MHz (clkdiv 10).
    const float pioTargetHz = 25000000.0f;
    float clkdiv = (float)clock_get_hz(clk_sys) / pioTargetHz;
    sm_config_set_clkdiv(&config, clkdiv);
    sm_config_set_out_shift(&config, true, false, 32);
    sm_config_set_in_shift(&config, false, true, 8);
    // PIO `mov x, status` returns 0xFFFFFFFF when TX FIFO has fewer than 1
    // entries (i.e. is empty), 0 otherwise. The receive loop polls this so
    // it knows when C has pushed a response and it should switch to send.
    sm_config_set_mov_status(&config, STATUS_TX_LESSTHAN, 1);
    // PIO `jmp pin` reads the data line so the receive loop can poll for
    // the next bit's falling edge in software (instead of `wait 0 pin 0`,
    // which would block indefinitely and prevent the TX FIFO check).
    sm_config_set_jmp_pin(&config, dataPin);

    pio_sm_init(pio, 0, offset, &config);
    // Start at `inmode_clean` (skips the residue-flush `push noblock` that
    // `inmode` does). At cold start the ISR is empty, so pushing it would
    // emit a phantom 0x00 into the RX FIFO that C reads as a fake probe,
    // desyncing the command stream until the host's retry logic happens
    // to realign.
    pio_sm_exec(pio, 0, pio_encode_jmp(offset + joybus_offset_inmode_clean));
    // Drain anything the host has been blasting onto the bus while we
    // were booting (probes/origins/polls all queue up in the RX FIFO the
    // instant we enable the SM). Without this, the loop's first reads
    // pull stale bytes and the residue-discard accounting goes off by
    // one for several cycles -- on the first poll the byte read as
    // "rumble" is actually a misaligned earlier byte, which on the Wii
    // happens to have LSB=1 and causes a spurious rumble at plug-in.
    pio_sm_clear_fifos(pio, 0);
    pio_sm_set_enabled(pio, 0, true);

    loadControllerMetadata();
    precomputeDisplayListChunks();

    while (true) {
		uint8_t joybusByte = pio_sm_get_blocking(pio, 0);

        if (joybusByte == 0) { // Probe
            uint8_t probeResponse[3] = { 0x09, 0x00, 0x03 };
            uint32_t result[2];
            int resultLen;
            convertToPio(probeResponse, 3, result, resultLen);
            // PIO auto-transitions to send when it sees TX FIFO has data.
            for (int i = 0; i<resultLen; i++) pio_sm_put_blocking(pio, 0, result[i]);
            // Discard the 1-bit stop-residue word that PIO pushes when it
            // re-enters `inmode` after end_reply. Blocks until PIO is done
            // transmitting and has executed `push noblock`.
            (void)pio_sm_get_blocking(pio, 0);
        }
        else if (joybusByte == 0x41) { // Origin (NOT 0x81)
            uint8_t originResponse[10] = { 0x00, 0x80, ORG, ORG, ORG, ORG, 0, 0, 0, 0 };
            uint32_t result[6];
            int resultLen;
            convertToPio(originResponse, 10, result, resultLen);
            for (int i = 0; i<resultLen; i++) pio_sm_put_blocking(pio, 0, result[i]);
            (void)pio_sm_get_blocking(pio, 0); // discard stop-bit residue
        }
        else if (joybusByte == 0x40) { // Poll
            GCReport gcReport = func();
            GCReport dest_report;

			//get the second byte; we do this interleaved with work that must be done
            joybusByte = pio_sm_get_blocking(pio, 0);

            convertGCReport(&gcReport, &dest_report, joybusByte);

            uint32_t result[5];
            int resultLen;
            convertToPio((uint8_t*)(&dest_report), 8, result, resultLen);

			//get the third byte; we do this interleaved with work that must be done
            joybusByte = pio_sm_get_blocking(pio, 0);

            // PIO absorbs the trailing stop-bit residue (~2us) in cycles
            // before driving the line, so no C-side sleep is required.
            for (int i = 0; i<resultLen; i++) pio_sm_put_blocking(pio, 0, result[i]);

			//Rumble
			bool rumbleBrake = joybusByte & 2;
			bool rumble = (joybusByte & 1) && !rumbleBrake;
            if(rumble) {
                pwm_set_gpio_level(brakePin, 0);
                pwm_set_gpio_level(rumblePin, rumblePower);
            } else {
                pwm_set_gpio_level(rumblePin, 0);
                pwm_set_gpio_level(brakePin, rumbleBrake ? 255 : 0);
            }

            (void)pio_sm_get_blocking(pio, 0); // discard stop-bit residue
        }
        else if (joybusByte == 0xA0) { // Read controller metadata chunk
            uint8_t chunkIndex = pio_sm_get_blocking(pio, 0);

            if (chunkIndex >= metadataNumChunks) chunkIndex = 0;

            for (int i = 0; i<metadataPioChunkLens[chunkIndex]; i++) {
                pio_sm_put_blocking(pio, 0, metadataPioChunks[chunkIndex][i]);
            }

            (void)pio_sm_get_blocking(pio, 0); // discard stop-bit residue
        }
        else if (joybusByte == 0xB0) { // Write controller metadata chunk
            uint8_t chunkIndex = pio_sm_get_blocking(pio, 0);

            uint8_t chunkData[METADATA_CHUNK_DATA_SIZE];
            for (int i = 0; i < METADATA_CHUNK_DATA_SIZE; i++) {
                chunkData[i] = pio_sm_get_blocking(pio, 0);
            }

            for (int i = 0; i<metadataAckPioResultLen; i++) {
                pio_sm_put_blocking(pio, 0, metadataAckPioResult[i]);
            }

            bool validChunk = chunkIndex < METADATA_MAX_CHUNKS;
            if (validChunk) {
                // Writing chunk 0 starts a new write sequence — reset count
                // so stale chunks from a previous (larger) write are discarded.
                if (chunkIndex == 0) metadataNumChunks = 0;
                memcpy(&controllerMetadata[chunkIndex * METADATA_CHUNK_DATA_SIZE], chunkData, METADATA_CHUNK_DATA_SIZE);
                if (chunkIndex + 1 > metadataNumChunks) metadataNumChunks = chunkIndex + 1;
                precomputeMetadataResponses();
                setControllerMetadata(controllerMetadata, metadataNumChunks);
                _pleaseCommitMetadata = true;
            }

            (void)pio_sm_get_blocking(pio, 0); // discard stop-bit residue
        }
        else if (joybusByte == 0xC0) { // Read display-list chunk
            _videoOverSi = true;
            uint8_t chunkIndex = pio_sm_get_blocking(pio, 0);

            uint8_t buf = displayListActiveBuf;
            displayListReaderBuf = (int8_t)buf;
            uint8_t n = displayListNumChunks;
            if (n == 0) {
                displayListReaderBuf = -1;
                (void)pio_sm_get_blocking(pio, 0); // discard stop-bit residue
                continue;
            }
            if (chunkIndex >= n) chunkIndex = 0;

            for (int i = 0; i < displayListPioChunkLens[buf][chunkIndex]; i++) {
                pio_sm_put_blocking(pio, 0, displayListPioChunks[buf][chunkIndex][i]);
            }
            displayListReaderBuf = -1;
            (void)pio_sm_get_blocking(pio, 0); // discard stop-bit residue
        }
        else {
            pio_sm_set_enabled(pio, 0, false);
            sleep_us(400);
            //If an unmatched communication happens, we wait for 400us for it to finish for sure before starting to listen again
            pio_sm_clear_fifos(pio, 0);
            pio_sm_restart(pio, 0);
            // pio_sm_restart clears ISR + shift counter, so enter via
            // `inmode_clean` to avoid the residue push that would emit a
            // phantom 0x00 into the freshly-cleared RX FIFO.
            pio_sm_exec(pio, 0, pio_encode_jmp(offset + joybus_offset_inmode_clean));
            pio_sm_set_enabled(pio, 0, true);
        }
    }
}

static PIO sVideoPio = pio1;
static uint sVideoSm = 0;
static uint sVideoOffset = 0;
static bool sVideoInited = false;

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
    if (sVideoInited) return;

    gpio_init(dataPin);
    gpio_set_dir(dataPin, GPIO_IN);
    gpio_pull_up(dataPin);
    sleep_us(100);

    pio_gpio_init(sVideoPio, dataPin);
    sVideoOffset = pio_add_program(sVideoPio, &joybus_program);

    pio_sm_config config = joybus_program_get_default_config(sVideoOffset);
    sm_config_set_in_pins(&config, dataPin);
    sm_config_set_out_pins(&config, dataPin, 1);
    sm_config_set_set_pins(&config, dataPin, 1);
    const float pioTargetHz = 25000000.0f;
    float clkdiv = (float)clock_get_hz(clk_sys) / pioTargetHz;
    sm_config_set_clkdiv(&config, clkdiv);
    sm_config_set_out_shift(&config, true, false, 32);
    sm_config_set_in_shift(&config, false, true, 8);
    sm_config_set_mov_status(&config, STATUS_TX_LESSTHAN, 1);
    sm_config_set_jmp_pin(&config, dataPin);

    pio_sm_init(sVideoPio, sVideoSm, sVideoOffset, &config);
    pio_sm_exec(sVideoPio, sVideoSm, pio_encode_jmp(sVideoOffset + joybus_offset_inmode_clean));
    pio_sm_clear_fifos(sVideoPio, sVideoSm);
    pio_sm_set_enabled(sVideoPio, sVideoSm, true);

    loadControllerMetadata();
    sVideoInited = true;
}

void serviceJoybusForVideoMode(std::function<GCReport()> reportFn) {
    if (!sVideoInited) return;
    if (pio_sm_is_rx_fifo_empty(sVideoPio, sVideoSm)) return;

    uint8_t joybusByte = (uint8_t)pio_sm_get(sVideoPio, sVideoSm);

    if (joybusByte == 0) { // Probe
        uint8_t probeResponse[3] = { 0x09, 0x00, 0x03 };
        uint32_t result[2];
        int resultLen;
        convertToPio(probeResponse, 3, result, resultLen);
        for (int i = 0; i < resultLen; i++) pio_sm_put_blocking(sVideoPio, sVideoSm, result[i]);
        (void)pio_sm_get_blocking(sVideoPio, sVideoSm);
    }
    else if (joybusByte == 0x41) { // Origin
        uint8_t originResponse[10] = { 0x00, 0x80, ORG, ORG, ORG, ORG, 0, 0, 0, 0 };
        uint32_t result[6];
        int resultLen;
        convertToPio(originResponse, 10, result, resultLen);
        for (int i = 0; i < resultLen; i++) pio_sm_put_blocking(sVideoPio, sVideoSm, result[i]);
        (void)pio_sm_get_blocking(sVideoPio, sVideoSm);
    }
    else if (joybusByte == 0x40) { // Poll
        GCReport gcReport = reportFn ? reportFn() : videoDefaultReport();
        GCReport dest_report;

        joybusByte = pio_sm_get_blocking(sVideoPio, sVideoSm);
        convertGCReport(&gcReport, &dest_report, joybusByte);

        uint32_t result[5];
        int resultLen;
        convertToPio((uint8_t*)(&dest_report), 8, result, resultLen);

        (void)pio_sm_get_blocking(sVideoPio, sVideoSm);
        for (int i = 0; i < resultLen; i++) pio_sm_put_blocking(sVideoPio, sVideoSm, result[i]);
        (void)pio_sm_get_blocking(sVideoPio, sVideoSm);
    }
    else if (joybusByte == 0xA0) { // Read metadata chunk
        uint8_t chunkIndex = pio_sm_get_blocking(sVideoPio, sVideoSm);
        if (chunkIndex >= metadataNumChunks) chunkIndex = 0;
        for (int i = 0; i < metadataPioChunkLens[chunkIndex]; i++) {
            pio_sm_put_blocking(sVideoPio, sVideoSm, metadataPioChunks[chunkIndex][i]);
        }
        (void)pio_sm_get_blocking(sVideoPio, sVideoSm);
    }
    else if (joybusByte == 0xB0) { // Write metadata chunk
        uint8_t chunkIndex = pio_sm_get_blocking(sVideoPio, sVideoSm);
        uint8_t chunkData[METADATA_CHUNK_DATA_SIZE];
        for (int i = 0; i < METADATA_CHUNK_DATA_SIZE; i++) {
            chunkData[i] = pio_sm_get_blocking(sVideoPio, sVideoSm);
        }

        if (chunkIndex < METADATA_MAX_CHUNKS) {
            if (chunkIndex == 0) metadataNumChunks = 0;
            memcpy(&controllerMetadata[chunkIndex * METADATA_CHUNK_DATA_SIZE], chunkData, METADATA_CHUNK_DATA_SIZE);
            if (chunkIndex + 1 > metadataNumChunks) metadataNumChunks = chunkIndex + 1;
            precomputeMetadataResponses();
            setControllerMetadata(controllerMetadata, metadataNumChunks);
            _pleaseCommitMetadata = true;
        }

        for (int i = 0; i < metadataAckPioResultLen; i++) {
            pio_sm_put_blocking(sVideoPio, sVideoSm, metadataAckPioResult[i]);
        }
        (void)pio_sm_get_blocking(sVideoPio, sVideoSm);
    }
    else if (joybusByte == 0xC0) { // Read display-list chunk
        _videoOverSi = true;
        uint8_t chunkIndex = pio_sm_get_blocking(sVideoPio, sVideoSm);
        uint8_t buf = displayListActiveBuf;
        displayListReaderBuf = (int8_t)buf;
        uint8_t n = displayListNumChunks;
        if (n == 0) {
            displayListReaderBuf = -1;
            (void)pio_sm_get_blocking(sVideoPio, sVideoSm);
            return;
        }
        if (chunkIndex >= n) chunkIndex = 0;
        for (int i = 0; i < displayListPioChunkLens[buf][chunkIndex]; i++) {
            pio_sm_put_blocking(sVideoPio, sVideoSm, displayListPioChunks[buf][chunkIndex][i]);
        }
        displayListReaderBuf = -1;
        (void)pio_sm_get_blocking(sVideoPio, sVideoSm);
    }
    else {
        pio_sm_set_enabled(sVideoPio, sVideoSm, false);
        sleep_us(400);
        pio_sm_clear_fifos(sVideoPio, sVideoSm);
        pio_sm_restart(sVideoPio, sVideoSm);
        pio_sm_exec(sVideoPio, sVideoSm, pio_encode_jmp(sVideoOffset + joybus_offset_inmode_clean));
        pio_sm_set_enabled(sVideoPio, sVideoSm, true);
    }
}

