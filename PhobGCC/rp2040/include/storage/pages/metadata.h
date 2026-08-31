#ifndef METADATA_H
#define METADATA_H

#include "storage/page_indexes.hpp"
#include "pico/stdlib.h"

#define METADATA_CHUNK_DATA_SIZE 78
#define METADATA_CHUNK_TRANSFER_SIZE 80
#define METADATA_MAX_CHUNKS 8
#define CONTROLLER_METADATA_MAX_SIZE (METADATA_MAX_CHUNKS * METADATA_CHUNK_DATA_SIZE) // 624

// Default metadata blob is generated at runtime (UBJSON)
// in storage.cpp's getMetadataPage()

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

#endif //METADATA_H
