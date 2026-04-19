#ifndef METADATA_H
#define METADATA_H

#include "storage/page_indexes.hpp"
#include "pico/stdlib.h"

#define METADATA_CHUNK_DATA_SIZE 78
#define METADATA_CHUNK_TRANSFER_SIZE 80
#define METADATA_MAX_CHUNKS 8
#define CONTROLLER_METADATA_MAX_SIZE (METADATA_MAX_CHUNKS * METADATA_CHUNK_DATA_SIZE) // 1008

static const uint8_t defaultControllerMetadata[METADATA_CHUNK_DATA_SIZE] = {
    'P', 'h', 'o', 'b', 'G', 'C', 'C', ' ',
    'v', '0', '.', '0', '.', '0', '.', '0',
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0
};

namespace Persistence {
namespace Pages {

struct Metadata {
    static const int index = (int) PageIndexes::METADATA;

    uint8_t numChunks;
    uint8_t controllerMetadata[CONTROLLER_METADATA_MAX_SIZE];
};

}
}

void getControllerMetadata(uint8_t *buf, uint8_t &numChunks);
void setControllerMetadata(const uint8_t *buf, uint8_t numChunks);
void commitMetadata(const bool noLock = false);

// Calibration transfer over SI (chunked like metadata)
void getCalibrationData(uint8_t *buf, uint8_t &numChunks);
void setCalibrationData(const uint8_t *buf, uint8_t numChunks);
void commitCalibration(const bool noLock = false);

#endif //METADATA_H
