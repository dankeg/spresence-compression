#ifndef ICER_COMPRESSION_H
#define ICER_COMPRESSION_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Forward declarations
class File;
class SDClass;

// ICER compression result
// If flash_filename is non-NULL, compressed_data is NULL and data is in flash
// If flash_filename is NULL, compressed_data contains the data in RAM
typedef struct {
    uint8_t* compressed_data;      // NULL if using flash
    size_t compressed_size;
    bool success;
    int error_code;
    const char* flash_filename;    // Non-NULL if result is stored in flash
} IcerCompressionResult;

// Compress YUV image using ICER
// Input: Y, U, V channels (uint16_t arrays)
// Output: compressed data buffer (caller must free) OR flash filename
// If sd_card and flash_filename are non-NULL, result is written to flash instead of RAM
// If channels_pre_transformed is true, skip wavelet transform (channels are already transformed)
// Returns: compression result with success flag
IcerCompressionResult compressYuvWithIcer(
    uint16_t* y_channel,
    uint16_t* u_channel,
    uint16_t* v_channel,
    size_t width,
    size_t height,
    uint8_t stages,           // Wavelet decomposition stages (typically 4-5)
    uint8_t filter_type,      // ICER filter type (0-6, typically 0 for ICER_FILTER_A)
    uint8_t segments,         // Error containment segments (typically 10)
    size_t target_size,       // Target compressed size in bytes (0 for lossless)
    SDClass* sd_card,         // If non-NULL, write result to flash file instead of RAM
    const char* flash_filename, // Filename for flash file (must be provided if sd_card is non-NULL)
    bool channels_pre_transformed = false  // If true, channels are already wavelet-transformed
);

// Free compressed data
void freeIcerCompression(IcerCompressionResult* result);

// Allocate ICER static buffers dynamically (only when needed)
// Returns 0 on success, negative error code on failure
int allocateIcerBuffers(void);

// Free ICER static buffers (call after compression is complete)
void freeIcerBuffers(void);

// Set GNSS RAM availability (call after up_gnssram_initialize() in main.cpp)
// If true, ICER buffers will be allocated in GNSS RAM to free main RAM for camera
void setGnssRamAvailable(bool available);

#endif // ICER_COMPRESSION_H

